// ProfileMachiningJobService 实现（oap_manufacturing）。
// 持久化外轮廓 root outer-wire 稳定引用，并在重新打开零件后重建区域与
// 派生刀路，见 ProfileMachiningJobService.h。
#include "manufacturing/profile/ProfileMachiningJobService.h"

#include <algorithm>
#include <cctype>
#include <set>
#include <sstream>
#include <unordered_set>
#include <utility>

#include <QCryptographicHash>
#include <QDateTime>

#include "manufacturing/job/MachiningJobStorage.h"
#include "toolpath/profile/geometry/ProfileRegionBuilder.h"
#include "toolpath/program/validation/ToolpathProgramValidator.h"

namespace oap::manufacturing {
namespace {

std::string sha256(const std::string& value)
{
    return QCryptographicHash::hash(
               QByteArray::fromStdString(value),
               QCryptographicHash::Sha256)
        .toHex().toStdString();
}

std::string setupFaceToken(MoldSetupFace face)
{
    std::string result;
    for (const char* value = moldSetupFaceName(face);
         *value != '\0'; ++value) {
        if (*value == '+') result += "pos";
        else if (*value == '-') result += "neg";
        else result += static_cast<char>(std::tolower(
            static_cast<unsigned char>(*value)));
    }
    return result;
}



toolpath::ToolpathParams paramsForTool(
    const toolpath::ToolpathParams& base,
    const toolpath::ToolpathToolContext& tool)
{
    toolpath::ToolpathParams result = base;
    result.tool.diameter = tool.diameter;
    result.tool.cornerRadius = tool.cornerRadius;
    result.tool.spindleSpeed = tool.spindleSpeed;
    result.tool.horizFeed = tool.horizontalFeed;
    result.tool.vertFeed = tool.verticalFeed;
    result.tool.horizRapid = tool.horizontalRapid;
    result.tool.vertRapid = tool.verticalRapid;
    result.tool.cuttingEdgeLength = tool.cuttingEdgeLength;
    result.tool.overallLength = tool.overallLength;
    result.tool.shankDiameter = tool.shankDiameter;
    result.tool.holderDiameter = tool.holderDiameter;
    result.tool.holderLength = tool.holderLength;
    result.tool.spindleEnvelopeDiameter = tool.spindleEnvelopeDiameter;
    result.tool.spindleEnvelopeLength = tool.spindleEnvelopeLength;
    result.tool.tipCompensation = tool.tipCompensation;
    return result;
}

ProfileTargetReference normalizedTarget(
    ProfileTargetReference target,
    const std::string& partFingerprint)
{
    const auto sortUnique = [](std::vector<std::string>& ids) {
        std::sort(ids.begin(), ids.end());
        ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    };
    sortUnique(target.sourceFaceIds);
    sortUnique(target.machiningFaceIds);
    sortUnique(target.terminalFaceIds);
    sortUnique(target.mainOuterFaceIds);
    std::ostringstream value;
    value << partFingerprint << '|' << target.axisLabel << '|'
          << target.normalDirection << '|' << target.rootFaceId << '|'
          << target.rootWireId;
    for (const std::string& id : target.sourceFaceIds)
        value << '|' << id;
    for (const std::string& id : target.machiningFaceIds)
        value << "|d:" << id;
    for (const std::string& id : target.terminalFaceIds)
        value << "|t:" << id;
    for (const std::string& id : target.mainOuterFaceIds)
        value << "|m:" << id;
    target.geometryFingerprint = sha256(value.str());
    return target;
}

std::string parametersFingerprint(const toolpath::ToolpathParams& params)
{
    MachiningJobDefinition snapshot;
    snapshot.profileParameters = params;
    return sha256(
        MachiningJobStorage::serializeDefinition(snapshot).toStdString());
}

std::string operationFingerprint(
    const MachiningJobDefinition& definition,
    const ProfileOperationDefinition& operation,
    const std::string& parameterFingerprint)
{
    const SetupDefinition* setup = nullptr;
    for (const SetupDefinition& item : definition.setups) {
        if (item.id == operation.setupId) {
            setup = &item;
            break;
        }
    }
    const JobToolController* controller = nullptr;
    for (const JobToolController& item : definition.toolControllers) {
        if (item.id == operation.toolControllerId) {
            controller = &item;
            break;
        }
    }
    std::ostringstream value;
    value << operation.setupId << '|'
          << (setup ? MachiningJobStorage::setupInputFingerprint(
                          definition.part, *setup).toStdString()
                    : std::string{}) << '|'
          << operation.toolControllerId << '|'
          << (controller
                  ? toolpath::toolpathToolContextKey(controller->tool)
                  : std::string{}) << '|'
          << operation.algorithmId << '|'
          << operation.target.geometryFingerprint << '|'
          << operation.restMachiningPreviousToolDiameter << '|'
          << (operation.finishPass ? 1 : 0) << '|'
          << parameterFingerprint;
    return sha256(value.str());
}

const SetupDefinition* setupForFace(
    const MachiningJobDefinition& definition, MoldSetupFace face)
{
    const auto found = std::find_if(
        definition.setups.begin(), definition.setups.end(),
        [face](const SetupDefinition& setup) {
            return setup.face == face;
        });
    return found == definition.setups.end() ? nullptr : &*found;
}

int availableSetupIndex(
    const MachiningJobDefinition& definition, int preferred)
{
    std::set<int> used;
    for (const SetupDefinition& setup : definition.setups)
        used.insert(setup.setupIndex);
    if (preferred >= 0 && preferred < 6 && !used.contains(preferred))
        return preferred;
    for (int index = 0; index < 6; ++index) {
        if (!used.contains(index))
            return index;
    }
    return -1;
}

SetupDefinition makeSetup(
    int setupIndex, MoldSetupFace face,
    const toolpath::MoldSetupWorkFrames* frames,
    const PartDefinition& part)
{
    SetupDefinition result;
    result.id = "setup-" + std::to_string(setupIndex + 1) + '-' +
        setupFaceToken(face);
    result.setupIndex = setupIndex;
    result.face = face;
    const toolpath::MoldFaceSetupWorkFrame* frame =
        frames ? frames->forFace(face) : nullptr;
    if (!frame || !frame->valid)
        return result;
    result.workOffset = frame->workOffset;
    result.origin = toJobVector(frame->setupWorkFrame.Location());
    result.xAxis = toJobVector(frame->setupWorkFrame.XDirection());
    result.yAxis = toJobVector(frame->setupWorkFrame.YDirection());
    result.zAxis = toJobVector(frame->setupWorkFrame.Direction());
    result.valid = true;
    result.requiresPhysicalReclamp = frame->requiresPhysicalReclamp;
    result.axisMapping = frame->axisMapping;
    result.operatorInstruction = frame->operatorInstruction;
    result.confirmation.confirmed = frame->confirmed;
    if (result.confirmation.confirmed) {
        result.confirmation.confirmedInputFingerprint =
            MachiningJobStorage::setupInputFingerprint(part, result)
                .toStdString();
    }
    return result;
}

void discardUnreferencedResources(MachiningJobDefinition& definition)
{
    std::unordered_set<std::string> setups;
    std::unordered_set<std::string> controllers;
    const auto retain = [&](const auto& operation) {
        setups.insert(operation.setupId);
        controllers.insert(operation.toolControllerId);
    };
    for (const auto& operation : definition.holeOperations) retain(operation);
    for (const auto& operation : definition.pocketOperations) retain(operation);
    for (const auto& operation : definition.profileOperations) retain(operation);
    for (const auto& operation : definition.finishingOperations) retain(operation);
    std::erase_if(definition.setups, [&](const SetupDefinition& setup) {
        return !setups.contains(setup.id);
    });
    std::erase_if(
        definition.toolControllers,
        [&](const JobToolController& controller) {
            return !controllers.contains(controller.id);
        });
}

} // namespace

bool ProfileMachiningJobService::capture(
    MachiningJob& job,
    const ProfileJobCaptureRequest& request,
    std::string* error) const
{
    if (error) error->clear();
    if (!request.operations || !request.targets || !request.parameters ||
        request.operations->empty() ||
        request.operations->size() != request.targets->size()) {
        if (error) *error = "profile capture input is incomplete";
        return false;
    }

    MachiningJobDefinition candidate = job.definition();
    candidate.profileOperations.clear();
    candidate.schemaVersion = kMachiningJobSchemaVersion;
    candidate.profileParameters = *request.parameters;
    candidate.modifiedAt = QDateTime::currentDateTimeUtc()
        .toString(Qt::ISODateWithMs).toStdString();

    int nextSequence = 0;
    for (const auto& operation : candidate.holeOperations)
        nextSequence = std::max(nextSequence, operation.sequence + 1);
    for (const auto& operation : candidate.pocketOperations)
        nextSequence = std::max(nextSequence, operation.sequence + 1);

    const std::string parameterHash =
        parametersFingerprint(*request.parameters);
    toolpath::ProfileToolpathBatch batch;
    batch.algorithmId = request.operations->front().algorithmId;
    batch.effectiveParams = *request.parameters;
    batch.operations = *request.operations;

    for (std::size_t index = 0; index < request.operations->size(); ++index) {
        const toolpath::ProfileToolpathOperation& generated =
            (*request.operations)[index];
        if (generated.setupFace == MoldSetupFace::Unresolved) {
            if (error) *error = "profile operation setup face is unresolved";
            return false;
        }
        if (!generated.tool.valid) {
            if (error) *error = "profile operation requires a tool-library snapshot";
            return false;
        }
        const toolpath::ToolpathToolContext operationTool = generated.tool;
        const std::string toolHash = sha256(
            toolpath::toolpathToolContextKey(operationTool));
        const std::string controllerId =
            "tool-controller-profile-" + toolHash.substr(0, 12) +
            (request.toolLibraryId && request.toolLibraryId->empty() ? "-direct" : "");
        if (std::none_of(
                candidate.toolControllers.begin(),
                candidate.toolControllers.end(),
                [&](const JobToolController& item) {
                    return item.id == controllerId;
                })) {
            JobToolController controller;
            controller.id = controllerId;
            controller.libraryId = request.toolLibraryId.value_or(candidate.activeToolLibraryId);
            controller.tool = operationTool;
            candidate.toolControllers.push_back(std::move(controller));
        }

        const SetupDefinition* existing = setupForFace(
            candidate, generated.setupFace);
        std::string setupId;
        if (existing) {
            setupId = existing->id;
        } else {
            const int setupIndex = availableSetupIndex(
                candidate, generated.setupIndex);
            if (setupIndex < 0) {
                if (error) *error = "no free six-face setup remains";
                return false;
            }
            SetupDefinition setup = makeSetup(
                setupIndex, generated.setupFace,
                request.setupFrames, candidate.part);
            setupId = setup.id;
            candidate.setups.push_back(std::move(setup));
        }

        ProfileOperationDefinition definition;
        definition.setupId = std::move(setupId);
        definition.toolControllerId = controllerId;
        definition.algorithmId = generated.algorithmId.empty()
            ? toolpath::kBuiltinProfileAlgorithmId
            : generated.algorithmId;
        definition.target = normalizedTarget(
            (*request.targets)[index], candidate.part.sourceFingerprint);
        definition.restMachiningPreviousToolDiameter =
            generated.restMachiningPreviousToolDiameter;
        definition.finishPass = generated.finishPass;
        definition.id = "profile-operation-" +
            definition.target.geometryFingerprint.substr(0, 12) +
            "-s" + std::to_string(nextSequence);
        definition.sequence = nextSequence++;
        definition.state = JobOperationState::Generated;
        definition.inputFingerprint = operationFingerprint(
            candidate, definition, parameterHash);
        batch.operations[index].operationId = definition.id;
        candidate.profileOperations.push_back(std::move(definition));
    }

    std::stable_sort(candidate.finishingOperations.begin(), candidate.finishingOperations.end(),
        [](const auto& a, const auto& b) { return a.sequence < b.sequence; });
    for (auto& operation : candidate.finishingOperations) operation.sequence = nextSequence++;
    discardUnreferencedResources(candidate);
    const toolpath::ToolpathValidationReport validation =
        toolpath::ToolpathProgramValidator().validate(
            batch.toProgram(), batch.effectiveParams);
    for (ProfileOperationDefinition& operation : candidate.profileOperations) {
        const auto setup = std::find_if(
            candidate.setups.begin(), candidate.setups.end(),
            [&](const SetupDefinition& item) {
                return item.id == operation.setupId;
            });
        operation.state = !validation.ok()
            ? JobOperationState::Error
            : setup == candidate.setups.end() || !setup->valid ||
                    !setup->confirmation.confirmed
                ? JobOperationState::NeedsReview
                : validation.warningCount > 0
                    ? JobOperationState::Warning
                    : JobOperationState::Generated;
    }

    MachiningJob validationJob;
    validationJob.definition() = candidate;
    const std::vector<std::string> errors = validationJob.validationErrors();
    if (!errors.empty()) {
        if (error) *error = errors.front();
        return false;
    }
    job.definition() = std::move(candidate);
    job.setProfileResult(
        std::move(batch),
        MachiningJobStorage::profileExecutionFingerprint(
            job.definition()).toStdString());
    return true;
}

toolpath::ProfileToolpathBatch ProfileMachiningJobService::regenerate(
    MachiningJob& job,
    const StepIndexer& indexer,
    const toolpath::ToolpathGenerationControl* control) const
{
    toolpath::ProfileToolpathBatch result;
    result.effectiveParams = job.definition().profileParameters;
    job.clearProfileResult();
    if (!job.validationErrors().empty())
        return result;

    std::vector<ProfileOperationDefinition*> definitions;
    for (ProfileOperationDefinition& operation :
         job.definition().profileOperations) {
        if (operation.enabled)
            definitions.push_back(&operation);
    }
    std::stable_sort(
        definitions.begin(), definitions.end(),
        [](const auto* left, const auto* right) {
            return left->sequence < right->sequence;
        });
    const std::string parameterHash = parametersFingerprint(
        job.definition().profileParameters);
    if (control) control->progress(0, definitions.size());

    for (std::size_t index = 0; index < definitions.size(); ++index) {
        ProfileOperationDefinition& definition = *definitions[index];
        if (control && control->cancelled()) {
            result.cancelled = true;
            definition.state = JobOperationState::Dirty;
            break;
        }
        const SetupDefinition* setup = job.setupById(definition.setupId);
        const JobToolController* controller =
            job.toolControllerById(definition.toolControllerId);
        if (!setup || !controller) {
            definition.state = JobOperationState::Error;
            continue;
        }

        toolpath::RootOuterWireProfileSource source;
        source.axisLabel = definition.target.axisLabel;
        source.normalDirection = definition.target.normalDirection;
        source.sourceFaceIds = definition.target.sourceFaceIds;
        source.machiningFaceIds = definition.target.machiningFaceIds;
        source.terminalFaceIds = definition.target.terminalFaceIds;
        source.mainOuterFaceIds = definition.target.mainOuterFaceIds;
        source.rootFaceId = definition.target.rootFaceId;
        source.rootWireId = definition.target.rootWireId;
        toolpath::ProfileRegionBuildOptions buildOptions;
        buildOptions.stockMarginX = job.definition().stock.marginX;
        buildOptions.stockMarginY = job.definition().stock.marginY;
        buildOptions.stockMarginZ = job.definition().stock.marginZ;
        std::string buildError;
        const std::optional<toolpath::ProfileRegion> region =
            toolpath::ProfileRegionBuilder().buildOne(
                source, indexer, buildOptions, &buildError);
        if (!region) {
            definition.state = JobOperationState::Error;
            toolpath::ProfileGenerationFailure failure;
            failure.regionIndex = static_cast<int>(index);
            failure.contour.failureCode =
                toolpath::ProfileContourFailureCode::InvalidRegion;
            failure.contour.error = std::move(buildError);
            result.failures.push_back(std::move(failure));
            continue;
        }
        toolpath::ToolpathParams operationParams = paramsForTool(
            job.definition().profileParameters, controller->tool);
        operationParams.clearing.restMachiningPreviousToolDiameter =
            definition.restMachiningPreviousToolDiameter;
        operationParams.profile.finishPass = definition.finishPass;
        toolpath::ProfileToolpathBatch generated =
            toolpath::ProfileToolpathService(
                toolpath::defaultToolpathAlgorithmRegistry(),
                definition.algorithmId)
                .generate({*region}, operationParams,
                          definition.id, control);
        if (generated.cancelled) {
            result.cancelled = true;
            definition.state = JobOperationState::Dirty;
            break;
        }
        result.diagnostics.insert(
            result.diagnostics.end(), generated.diagnostics.begin(),
            generated.diagnostics.end());
        if (generated.operations.empty()) {
            definition.state = JobOperationState::Error;
            for (auto& failure : generated.failures) {
                failure.regionIndex = static_cast<int>(index);
                result.failures.push_back(std::move(failure));
            }
            continue;
        }
        const toolpath::ToolpathValidationReport validation =
            toolpath::ToolpathProgramValidator().validate(
                generated.toProgram(), generated.effectiveParams);
        for (toolpath::ProfileToolpathOperation& operation :
             generated.operations) {
            operation.operationId = definition.id;
            operation.algorithmId = definition.algorithmId;
            operation.tool = controller->tool;
            operation.restMachiningPreviousToolDiameter =
                definition.restMachiningPreviousToolDiameter;
            operation.finishPass = definition.finishPass;
            operation.sourceRegionIndex = static_cast<int>(index);
            operation.setupIndex = setup->setupIndex;
            operation.setupFace = setup->face;
            result.operations.push_back(std::move(operation));
        }
        definition.inputFingerprint = operationFingerprint(
            job.definition(), definition, parameterHash);
        definition.state = !validation.ok()
            ? JobOperationState::Error
            : !setup->valid || !setup->confirmation.confirmed
                ? JobOperationState::NeedsReview
                : validation.warningCount > 0
                    ? JobOperationState::Warning
                    : JobOperationState::Generated;
        if (control) control->progress(index + 1, definitions.size());
    }

    if (!result.operations.empty())
        result.algorithmId = result.operations.front().algorithmId;
    job.definition().modifiedAt = QDateTime::currentDateTimeUtc()
        .toString(Qt::ISODateWithMs).toStdString();
    job.setProfileResult(
        result,
        MachiningJobStorage::profileExecutionFingerprint(
            job.definition()).toStdString());
    return result;
}

} // namespace oap::manufacturing

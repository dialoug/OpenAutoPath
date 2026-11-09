// PocketMachiningJobService 的实现:槽腔捕获(capture)与重建(regenerate)。
// 含刀具/参数/语义图元/目标几何指纹、Setup 工作坐标映射、无效资源回收与
// 校验。属于 oap_manufacturing 模块。
#include "manufacturing/pocket/PocketMachiningJobService.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <locale>
#include <set>
#include <sstream>
#include <unordered_set>
#include <utility>

#include <QCryptographicHash>
#include <QDateTime>

#include "manufacturing/job/MachiningJobStorage.h"
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"
#include "toolpath/core/PocketToolBinding.h"
#include "toolpath/pocket/validation/PocketToolpathValidator.h"

namespace oap::manufacturing {
namespace {

std::string sha256(const std::string& value)
{
    return QCryptographicHash::hash(
               QByteArray::fromStdString(value),
               QCryptographicHash::Sha256)
        .toHex().toStdString();
}

// 把装夹面名称(如 +Z)转为小写字母与 pos/neg 前缀的稳定 token,用于 Setup ID。
std::string setupFaceToken(MoldSetupFace face)
{
    std::string result;
    for (const char* value = moldSetupFaceName(face);
         *value != '\0'; ++value) {
        if (*value == '+')
            result += "pos";
        else if (*value == '-')
            result += "neg";
        else
            result += static_cast<char>(std::tolower(
                static_cast<unsigned char>(*value)));
    }
    return result;
}

// 从全局参数构造默认槽腔刀具上下文(用于手动捕获的兜底刀具)。


// 刀具指纹:刀具上下文规范化键的 SHA-256。
std::string toolFingerprint(
    const toolpath::ToolpathToolContext& tool)
{
    return sha256(toolpath::toolpathToolContextKey(tool));
}

toolpath::ToolpathParams paramsForTool(
    const toolpath::ToolpathParams& base,
    const toolpath::ToolpathToolContext& tool)
{
    return toolpath::bindPocketToolParameters(base, tool);
}

// 参数指纹:仅含 pocketParameters 的定义快照的序列化哈希,保证参数确定性。
std::string parametersFingerprint(
    const toolpath::ToolpathParams& params)
{
    MachiningJobDefinition snapshot;
    snapshot.pocketParameters = params;
    return sha256(
        MachiningJobStorage::serializeDefinition(snapshot)
            .toStdString());
}

void appendPointFingerprint(std::ostringstream& value,
                            const gp_Pnt& point)
{
    value << point.X() << ',' << point.Y() << ',' << point.Z();
}

void appendVectorFingerprint(std::ostringstream& value,
                             const gp_Vec& vector)
{
    value << vector.X() << ',' << vector.Y() << ',' << vector.Z();
}

void appendMoveFingerprint(
    std::ostringstream& value,
    const toolpath::ToolpathMove& move)
{
    value << static_cast<int>(move.type) << ':';
    appendPointFingerprint(value, move.from);
    value << ':';
    appendPointFingerprint(value, move.to);
    value << ':' << move.loopSequence
          << ':' << move.depthIndex
          << ':' << move.ring
          << ':' << move.feed
          << ':' << static_cast<int>(move.intent);
}

// 语义图元流的确定性指纹:线性/圆弧/钻孔周期逐字段序列化,含回退移动;
// 用于捕获与重建结果一致性检测。
std::string primitiveStreamFingerprint(
    const std::vector<toolpath::ToolpathPrimitive>& primitives)
{
    std::ostringstream value;
    value.imbue(std::locale::classic());
    value.precision(17);
    value << "pocket-semantic-primitives-v1|"
          << primitives.size();
    for (const toolpath::ToolpathPrimitive& primitive : primitives) {
        value << '|' << static_cast<int>(primitive.type)
              << ':' << primitive.context.loopSequence
              << ':' << primitive.context.depthIndex
              << ':' << primitive.context.ring
              << ':' << static_cast<int>(primitive.context.intent)
              << ':';
        if (primitive.type == toolpath::ToolpathPrimitiveType::Linear) {
            appendMoveFingerprint(value, primitive.linear);
        } else if (
            primitive.type ==
            toolpath::ToolpathPrimitiveType::CircularArc) {
            const toolpath::ToolpathCircularArc& arc = primitive.arc;
            value << static_cast<int>(arc.moveType)
                  << ':' << static_cast<int>(arc.direction) << ':';
            appendPointFingerprint(value, arc.from);
            value << ':';
            appendPointFingerprint(value, arc.to);
            value << ':';
            appendPointFingerprint(value, arc.center);
            value << ':';
            appendVectorFingerprint(value, arc.axis);
            value << ':' << arc.sweepRadians << ':' << arc.feed;
        } else {
            const toolpath::ToolpathDrillCycle& cycle =
                primitive.drillCycle;
            value << static_cast<int>(cycle.type)
                  << ':' << static_cast<int>(cycle.retractMode) << ':';
            appendPointFingerprint(value, cycle.clearance);
            value << ':';
            appendPointFingerprint(value, cycle.retract);
            value << ':';
            appendPointFingerprint(value, cycle.entry);
            value << ':';
            appendPointFingerprint(value, cycle.bottom);
            value << ':';
            appendVectorFingerprint(value, cycle.axis);
            value << ':' << static_cast<int>(cycle.source)
                  << ':' << cycle.nominalDepth
                  << ':' << cycle.depthDiameterRatio
                  << ':' << cycle.peckDepth
                  << ':' << cycle.chipBreakRetract
                  << ':' << cycle.reentryClearance
                  << ':' << cycle.dwellSeconds
                  << ':' << cycle.feed;
        }
        value << ":fallback=" << primitive.fallbackMoves.size();
        for (const toolpath::ToolpathMove& move :
             primitive.fallbackMoves) {
            value << ':';
            appendMoveFingerprint(value, move);
        }
    }
    return sha256(value.str());
}

// 单工序/多工序语义图元指纹。
std::string operationPrimitiveFingerprint(
    const toolpath::PocketToolpathOperation& operation)
{
    return sha256(
        "pocket-operation-primitives-v1|1|" +
        primitiveStreamFingerprint(operation.clearing.primitives));
}

std::string operationPrimitiveFingerprint(
    const std::vector<toolpath::PocketToolpathOperation>& operations)
{
    std::ostringstream value;
    value << "pocket-operation-primitives-v1|"
          << operations.size();
    for (const toolpath::PocketToolpathOperation& operation : operations) {
        value << '|'
              << primitiveStreamFingerprint(
                     operation.clearing.primitives);
    }
    return sha256(value.str());
}

// 规范化目标:排序去重源面 ID、为根内环来源回填 rootFaceId,并计算稳定的
// 几何指纹(部件指纹 + 来源/种类/根面/根线/开放边界等)。
PocketTargetReference normalizedTarget(
    PocketTargetReference target,
    const std::string& partFingerprint)
{
    std::sort(target.sourceFaceIds.begin(),
              target.sourceFaceIds.end());
    target.sourceFaceIds.erase(
        std::unique(target.sourceFaceIds.begin(),
                    target.sourceFaceIds.end()),
        target.sourceFaceIds.end());
    if (target.sourceKind ==
            PocketRegionSourceKind::RootInnerWire &&
        target.rootFaceId.empty()) {
        target.rootFaceId = target.floorFaceId;
    }
    std::ostringstream value;
    value << partFingerprint << '|'
          << static_cast<int>(target.sourceKind) << '|'
          << static_cast<int>(target.kind) << '|'
          << target.floorFaceId << '|'
          << target.rootFaceId << '|'
          << target.rootWireId << '|'
          << target.allowOpenSectionContours << '|'
          << target.openBoundaryFaceCount;
    for (const std::string& faceId : target.sourceFaceIds)
        value << '|' << faceId;
    target.geometryFingerprint = sha256(value.str());
    return target;
}

// 槽腔工序输入指纹:设置输入、刀具键、算法、上一把刀直径、目标几何指纹、
// 参数指纹与语义图元指纹的联合哈希;用于检测重建结果是否与捕获时一致。
std::string operationFingerprint(
    const MachiningJobDefinition& definition,
    const PocketOperationDefinition& operation,
    const std::string& parameterFingerprint,
    const std::string& semanticPrimitiveFingerprint)
{
    std::ostringstream value;
    const auto setup = std::find_if(
        definition.setups.begin(), definition.setups.end(),
        [&](const SetupDefinition& item) {
            return item.id == operation.setupId;
        });
    const auto controller = std::find_if(
        definition.toolControllers.begin(),
        definition.toolControllers.end(),
        [&](const JobToolController& item) {
            return item.id == operation.toolControllerId;
        });
    value << operation.setupId << '|'
          << (setup != definition.setups.end()
                  ? MachiningJobStorage::setupInputFingerprint(
                        definition.part, *setup).toStdString()
                  : std::string{}) << '|'
          << operation.toolControllerId << '|'
          << (controller != definition.toolControllers.end()
                  ? toolpath::toolpathToolContextKey(controller->tool)
                  : std::string{}) << '|'
          << operation.algorithmId << '|'
          << operation.restMachiningPreviousToolDiameter << '|'
          << operation.target.geometryFingerprint << '|'
          << parameterFingerprint << '|'
          << semanticPrimitiveFingerprint;
    return sha256(value.str());
}

// 从生成的槽腔批次推导工序状态:空结果或校验失败 -> Error;存在警告 -> Warning。
JobOperationState generatedState(
    const toolpath::PocketToolpathBatch& batch)
{
    if (batch.operations.empty() || !batch.validation.ok())
        return JobOperationState::Error;
    const bool warning =
        batch.validation.warningCount > 0 ||
        std::any_of(
            batch.operations.begin(), batch.operations.end(),
            [](const toolpath::PocketToolpathOperation& operation) {
                return !operation.clearing.warning.empty();
            }) ||
        std::any_of(
            batch.diagnostics.begin(), batch.diagnostics.end(),
            [](const toolpath::ToolpathDiagnostic& diagnostic) {
                return diagnostic.severity ==
                    toolpath::ToolpathDiagnosticSeverity::Warning;
            });
    return warning ? JobOperationState::Warning
                   : JobOperationState::Generated;
}

// 从装夹工作坐标框架构造 Setup 定义;框架缺失/无效时返回无效 Setup。
SetupDefinition makeSetup(
    int setupIndex,
    MoldSetupFace face,
    const toolpath::MoldSetupWorkFrames* frames,
    const PartDefinition& part)
{
    SetupDefinition result;
    result.id = "setup-" + std::to_string(setupIndex + 1) +
        '-' + setupFaceToken(face);
    result.setupIndex = setupIndex;
    result.face = face;
    const toolpath::MoldFaceSetupWorkFrame* frame =
        frames ? frames->forFace(face) : nullptr;
    if (!frame || !frame->valid)
        return result;
    result.workOffset = frame->workOffset;
    result.origin =
        toJobVector(frame->setupWorkFrame.Location());
    result.xAxis =
        toJobVector(frame->setupWorkFrame.XDirection());
    result.yAxis =
        toJobVector(frame->setupWorkFrame.YDirection());
    result.zAxis =
        toJobVector(frame->setupWorkFrame.Direction());
    result.valid = true;
    result.requiresPhysicalReclamp =
        frame->requiresPhysicalReclamp;
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

// 挑选空闲的六面体设置索引:优先给定 preferred(0..5),否则取最小空闲者。
int availableSetupIndex(
    const MachiningJobDefinition& definition,
    int preferred)
{
    std::set<int> used;
    for (const SetupDefinition& setup : definition.setups)
        used.insert(setup.setupIndex);
    if (preferred >= 0 && preferred < 6 &&
        !used.contains(preferred)) {
        return preferred;
    }
    for (int index = 0; index < 6; ++index) {
        if (!used.contains(index))
            return index;
    }
    return -1;
}

// 按装夹面查找已有 Setup;未找到返回 nullptr。
const SetupDefinition* setupForFace(
    const MachiningJobDefinition& definition,
    MoldSetupFace face)
{
    const auto found = std::find_if(
        definition.setups.begin(), definition.setups.end(),
        [face](const SetupDefinition& setup) {
            return setup.face == face;
        });
    return found == definition.setups.end()
        ? nullptr : &*found;
}

void clearOldPocketOperations(
    MachiningJobDefinition& definition)
{
    definition.pocketOperations.clear();
}

void discardUnreferencedResources(
    MachiningJobDefinition& definition)
{
    std::unordered_set<std::string> retainedSetups;
    std::unordered_set<std::string> retainedControllers;
    for (const HoleOperationDefinition& operation :
         definition.holeOperations) {
        retainedSetups.insert(operation.setupId);
        retainedControllers.insert(operation.toolControllerId);
    }
    for (const PocketOperationDefinition& operation :
         definition.pocketOperations) {
        retainedSetups.insert(operation.setupId);
        retainedControllers.insert(operation.toolControllerId);
    }
    for (const ProfileOperationDefinition& operation :
         definition.profileOperations) {
        retainedSetups.insert(operation.setupId);
        retainedControllers.insert(operation.toolControllerId);
    }
    for (const auto& operation : definition.finishingOperations) {
        retainedSetups.insert(operation.setupId);
        retainedControllers.insert(operation.toolControllerId);
    }
    std::erase_if(
        definition.setups,
        [&retainedSetups](const SetupDefinition& setup) {
            return !retainedSetups.contains(setup.id);
        });
    std::erase_if(
        definition.toolControllers,
        [&retainedControllers](
            const JobToolController& controller) {
            return !retainedControllers.contains(controller.id);
        });
}

} // namespace

bool PocketMachiningJobService::capture(
    MachiningJob& job,
    const PocketJobCaptureRequest& request,
    std::string* error) const
{
    if (error)
        error->clear();
    if (!request.operations || !request.targets ||
        !request.parameters) {
        if (error)
            *error = "pocket capture input is incomplete";
        return false;
    }
    if (request.operations->size() != request.targets->size()) {
        if (error)
            *error = "pocket operation and target counts differ";
        return false;
    }

    // 捕获策略:在 Job 定义的副本上工作,先清空旧槽腔工序与参数,再按预览
    // 批次逐条创建定义;全部成功后才原子替换正式定义,避免半更新状态。
    MachiningJobDefinition candidate = job.definition();
    clearOldPocketOperations(candidate);
    candidate.schemaVersion = kMachiningJobSchemaVersion;
    candidate.pocketParameters = *request.parameters;
    candidate.modifiedAt =
        QDateTime::currentDateTimeUtc()
            .toString(Qt::ISODateWithMs).toStdString();


    // 新槽腔工序的全局序列从已有孔工序最大序列之后开始,保持全局顺序唯一。
    int nextSequence = 0;
    for (const HoleOperationDefinition& operation :
         candidate.holeOperations) {
        nextSequence =
            std::max(nextSequence, operation.sequence + 1);
    }
    for (const ProfileOperationDefinition& operation :
         candidate.profileOperations) {
        nextSequence =
            std::max(nextSequence, operation.sequence + 1);
    }
    const std::string parameterHash =
        parametersFingerprint(*request.parameters);
    candidate.pocketOperations.reserve(
        request.operations->size());
    std::vector<std::string> capturedOperationIds;
    capturedOperationIds.reserve(
        request.operations->size());
    // 逐条把预览工序固化为 PocketOperationDefinition:刀具按指纹去重生成
    // ToolController,装夹面复用/新建 Setup,目标规范化并计算输入指纹。
    for (std::size_t index = 0;
         index < request.operations->size(); ++index) {
        const toolpath::PocketToolpathOperation& generated =
            (*request.operations)[index];
        if (!generated.tool.valid) {
            if (error) *error = "pocket operation requires a tool-library snapshot";
            return false;
        }
        const toolpath::ToolpathToolContext operationTool = generated.tool;
        const std::string toolHash =
            toolFingerprint(operationTool);
        const std::string controllerId =
            "tool-controller-pocket-" +
            toolHash.substr(0, 12) + (request.toolLibraryId && request.toolLibraryId->empty() ? "-direct" : "");
        const bool controllerExists = std::any_of(
            candidate.toolControllers.begin(),
            candidate.toolControllers.end(),
            [&controllerId](
                const JobToolController& existing) {
                return existing.id == controllerId;
            });
        if (!controllerExists) {
            JobToolController controller;
            controller.id = controllerId;
            controller.libraryId =
                request.toolLibraryId.value_or(candidate.activeToolLibraryId);
            controller.tool = operationTool;
            candidate.toolControllers.push_back(
                std::move(controller));
        }
        if (generated.setupFace == MoldSetupFace::Unresolved) {
            if (error)
                *error = "pocket operation setup face is unresolved";
            return false;
        }
        const SetupDefinition* existingSetup =
            setupForFace(candidate, generated.setupFace);
        std::string setupId;
        if (existingSetup) {
            setupId = existingSetup->id;
        } else {
            const int setupIndex = availableSetupIndex(
                candidate, generated.setupIndex);
            if (setupIndex < 0) {
                if (error)
                    *error = "no free six-face setup remains";
                return false;
            }
            SetupDefinition setup = makeSetup(
                setupIndex, generated.setupFace,
                request.setupFrames, candidate.part);
            setupId = setup.id;
            candidate.setups.push_back(std::move(setup));
        }

        PocketOperationDefinition operation;
        operation.setupId = std::move(setupId);
        operation.toolControllerId = controllerId;
        operation.restMachiningPreviousToolDiameter =
            generated.restMachiningPreviousToolDiameter;
        operation.algorithmId = generated.algorithmId.empty()
            ? toolpath::kBuiltinPocketAlgorithmId
            : generated.algorithmId;
        operation.target = normalizedTarget(
            (*request.targets)[index],
            candidate.part.sourceFingerprint);
        operation.id = "pocket-operation-" +
            operation.target.geometryFingerprint.substr(0, 12) +
            "-t" + std::to_string(operationTool.toolNumber) +
            "-s" + std::to_string(nextSequence);
        operation.sequence = nextSequence++;
        operation.state = JobOperationState::Generated;
        operation.inputFingerprint = operationFingerprint(
            candidate, operation, parameterHash,
            operationPrimitiveFingerprint(generated));
        capturedOperationIds.push_back(operation.id);
        candidate.pocketOperations.push_back(
            std::move(operation));
    }
    std::stable_sort(
        candidate.profileOperations.begin(),
        candidate.profileOperations.end(),
        [](const ProfileOperationDefinition& left,
           const ProfileOperationDefinition& right) {
            return left.sequence < right.sequence;
        });
    for (ProfileOperationDefinition& operation :
         candidate.profileOperations) {
        operation.sequence = nextSequence++;
    }
    // 回收不再被任何工序(含孔)引用的 Setup 与 ToolController。
    std::stable_sort(candidate.finishingOperations.begin(), candidate.finishingOperations.end(),
        [](const auto& a, const auto& b) { return a.sequence < b.sequence; });
    for (auto& operation : candidate.finishingOperations) operation.sequence = nextSequence++;
    discardUnreferencedResources(candidate);

    toolpath::PocketToolpathBatch batch;
    batch.effectiveParams = *request.parameters;
    batch.operations = *request.operations;
    for (std::size_t index = 0;
         index < batch.operations.size(); ++index) {
        batch.operations[index].operationId =
            capturedOperationIds[index];
    }
    if (!batch.operations.empty()) {
        batch.algorithmId =
            batch.operations.front().algorithmId;
    }
    batch.validation =
        toolpath::PocketToolpathValidator().validate(
            batch.operations, batch.effectiveParams);
    const JobOperationState state = generatedState(batch);
    for (PocketOperationDefinition& operation :
         candidate.pocketOperations) {
        const auto setup = std::find_if(
            candidate.setups.begin(), candidate.setups.end(),
            [&operation](const SetupDefinition& item) {
                return item.id == operation.setupId;
            });
        operation.state =
            setup == candidate.setups.end() ||
                    !setup->valid ||
                    !setup->confirmation.confirmed
                ? JobOperationState::NeedsReview
                : state;
    }

    // 提交前用独立 Job 副本做整体校验(定义合法、资源引用完整),通过后
    // 才写回正式 Job 并刷新槽腔运行期缓存。
    MachiningJob validationJob;
    validationJob.definition() = candidate;
    const std::vector<std::string> validationErrors =
        validationJob.validationErrors();
    if (!validationErrors.empty()) {
        if (error)
            *error = validationErrors.front();
        return false;
    }

    job.definition() = std::move(candidate);
    job.setPocketResult(
        std::move(batch),
        MachiningJobStorage::pocketExecutionFingerprint(
            job.definition()).toStdString());
    return true;
}

toolpath::PocketToolpathBatch
PocketMachiningJobService::regenerate(
    MachiningJob& job,
    const StepIndexer& indexer,
    const toolpath::ToolpathGenerationControl* control) const
{
    toolpath::PocketToolpathBatch result;
    result.effectiveParams = job.definition().pocketParameters;
    job.clearPocketResult();
    if (!job.validationErrors().empty())
        return result;

    // 重建策略:按序列取出全部启用槽腔工序,逐条从稳定来源引用重建区域并
    // 生成刀路,更新定义状态与输入指纹;取消/失败按工序处理。
    std::vector<PocketOperationDefinition*> operations;
    for (PocketOperationDefinition& operation :
         job.definition().pocketOperations) {
        if (operation.enabled)
            operations.push_back(&operation);
    }
    std::stable_sort(
        operations.begin(), operations.end(),
        [](const PocketOperationDefinition* left,
           const PocketOperationDefinition* right) {
            return left->sequence < right->sequence;
        });
    result.regionOutcomes.resize(operations.size());
    const std::string parameterHash = parametersFingerprint(
        job.definition().pocketParameters);
    if (control)
        control->progress(0, operations.size());

    for (std::size_t index = 0;
         index < operations.size(); ++index) {
        PocketOperationDefinition& definition =
            *operations[index];
        if (control && control->cancelled()) {
            result.cancelled = true;
            definition.state = JobOperationState::Dirty;
            break;
        }
        const SetupDefinition* setup =
            job.setupById(definition.setupId);
        const JobToolController* controller =
            job.toolControllerById(
                definition.toolControllerId);
        if (!setup || !controller) {
            definition.state = JobOperationState::Error;
            continue;
        }

        toolpath::ToolpathDiagnostic buildDiagnostic;
        std::optional<toolpath::PocketRegion> region;
        if (definition.target.sourceKind ==
            PocketRegionSourceKind::RootInnerWire) {
            toolpath::RootInnerWirePocketSource source;
            source.kind = definition.target.kind;
            source.sourceFaceIds =
                definition.target.sourceFaceIds;
            source.rootFaceId =
                definition.target.rootFaceId;
            source.rootWireId =
                definition.target.rootWireId;
            source.allowOpenSectionContours =
                definition.target.allowOpenSectionContours;
            source.openBoundaryFaceCount =
                definition.target.openBoundaryFaceCount;
            region = toolpath::PocketRegionBuilder()
                .buildRootInnerWireRegionDetailed(
                    source, indexer, {}, &buildDiagnostic);
        } else {
            FeatureInstance instance;
            instance.kind = definition.target.kind;
            instance.faceIds =
                definition.target.sourceFaceIds;
            instance.floorFaceId =
                definition.target.floorFaceId;
            instance.manual = true;
            region = toolpath::PocketRegionBuilder()
                .buildInstanceDetailed(
                    instance, indexer, {},
                    &buildDiagnostic);
        }
        if (!region) {
            definition.state = JobOperationState::Error;
            result.diagnostics.push_back(
                std::move(buildDiagnostic));
            continue;
        }

        toolpath::ToolpathParams operationParams =
            paramsForTool(
                job.definition().pocketParameters,
                controller->tool);
        operationParams.clearing
            .restMachiningPreviousToolDiameter =
            definition.restMachiningPreviousToolDiameter;
        toolpath::PocketToolpathBatch generated =
            toolpath::PocketToolpathService(
                toolpath::defaultToolpathAlgorithmRegistry(),
                definition.algorithmId)
                .generate(
                    {*region},
                    operationParams,
                    definition.id, control);
        if (generated.cancelled) {
            result.cancelled = true;
            definition.state = JobOperationState::Dirty;
            break;
        }
        result.diagnostics.insert(
            result.diagnostics.end(),
            generated.diagnostics.begin(),
            generated.diagnostics.end());
        result.materialSectionCacheHits +=
            generated.materialSectionCacheHits;
        result.materialSectionCacheMisses +=
            generated.materialSectionCacheMisses;
        result.materialSectionCacheEntries +=
            generated.materialSectionCacheEntries;
        result.structuralValidationMilliseconds +=
            generated.structuralValidationMilliseconds;
        result.safetyValidationMilliseconds +=
            generated.safetyValidationMilliseconds;
        if (generated.operations.empty()) {
            definition.state = JobOperationState::Error;
            for (toolpath::PocketGenerationFailure& failure :
                 generated.failures) {
                failure.regionIndex =
                    static_cast<int>(index);
                result.regionOutcomes[index].failureIndex =
                    static_cast<int>(result.failures.size());
                result.failures.push_back(
                    std::move(failure));
            }
            continue;
        }
        // 生成结果挂回定义:刷新输入指纹,逐条绑定定义 ID/算法/刀具与设置,
        // 并把状态写为 Generated/Warning(设置未确认时为 NeedsReview)。
        definition.inputFingerprint = operationFingerprint(
            job.definition(), definition, parameterHash,
            operationPrimitiveFingerprint(generated.operations));
        result.regionOutcomes[index].operationIndex =
            static_cast<int>(result.operations.size());
        for (toolpath::PocketToolpathOperation& operation :
             generated.operations) {
            operation.operationId = definition.id;
            operation.algorithmId = definition.algorithmId;
            operation.tool = controller->tool;
            operation.restMachiningPreviousToolDiameter =
                definition.restMachiningPreviousToolDiameter;
            operation.sourceRegionIndex =
                static_cast<int>(index);
            operation.setupIndex = setup->setupIndex;
            operation.setupFace = setup->face;
            result.operations.push_back(std::move(operation));
        }
        definition.state = generatedState(generated);
        if (definition.state != JobOperationState::Error &&
            (!setup->valid ||
             !setup->confirmation.confirmed)) {
            definition.state =
                JobOperationState::NeedsReview;
        }
        if (control)
            control->progress(index + 1, operations.size());
    }

    // 汇总批次:以首工序算法为批次算法,取最小直径刀具刷新有效参数,并对
    // 整批做一次校验后写回 Job。
    if (!result.operations.empty()) {
        result.algorithmId =
            result.operations.front().algorithmId;
        const auto smallestTool = std::min_element(
            result.operations.begin(), result.operations.end(),
            [](const toolpath::PocketToolpathOperation& left,
               const toolpath::PocketToolpathOperation& right) {
                const double leftDiameter = left.tool.valid
                    ? left.tool.diameter
                    : std::numeric_limits<double>::infinity();
                const double rightDiameter = right.tool.valid
                    ? right.tool.diameter
                    : std::numeric_limits<double>::infinity();
                return leftDiameter < rightDiameter;
            });
        if (smallestTool != result.operations.end() &&
            smallestTool->tool.valid) {
            result.effectiveParams = paramsForTool(
                job.definition().pocketParameters,
                smallestTool->tool);
        }
    }
    result.validation =
        toolpath::PocketToolpathValidator().validate(
            result.operations, result.effectiveParams);
    job.definition().modifiedAt =
        QDateTime::currentDateTimeUtc()
            .toString(Qt::ISODateWithMs).toStdString();
    job.setPocketResult(
        result,
        MachiningJobStorage::pocketExecutionFingerprint(
            job.definition()).toStdString());
    return result;
}

} // namespace oap::manufacturing

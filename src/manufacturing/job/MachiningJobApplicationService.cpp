// MachiningJobApplicationService 的实现（oap_manufacturing 应用层）。
// 提供 Setup 捕获/重绑定、运行期状态投影与保存前同步；不直接访问文件与 UI。
#include "manufacturing/job/MachiningJobApplicationService.h"

#include <algorithm>
#include <optional>
#include <utility>

#include "manufacturing/job/MachiningJobStorage.h"
#include "toolpath/program/validation/ToolpathProgramValidator.h"

namespace oap::manufacturing {
namespace {

// 用六面工作框覆盖 Setup 的派生值；frame 无效时置 valid=false 并清空确认。
void applyFrame(SetupDefinition& setup,
                const toolpath::MoldFaceSetupWorkFrame& frame)
{
    setup.workOffset = frame.workOffset;
    setup.origin = toJobVector(frame.setupWorkFrame.Location());
    setup.xAxis = toJobVector(frame.setupWorkFrame.XDirection());
    setup.yAxis = toJobVector(frame.setupWorkFrame.YDirection());
    setup.zAxis = toJobVector(frame.setupWorkFrame.Direction());
    setup.valid = true;
    setup.requiresPhysicalReclamp = frame.requiresPhysicalReclamp;
    setup.axisMapping = frame.axisMapping;
    setup.operatorInstruction = frame.operatorInstruction;
}

const SetupDefinition* setupById(const MachiningJobDefinition& definition,
                                 const std::string& id)
{
    const auto found = std::find_if(
        definition.setups.begin(), definition.setups.end(),
        [&id](const SetupDefinition& setup) { return setup.id == id; });
    return found == definition.setups.end() ? nullptr : &*found;
}

bool hasHoleWarning(const toolpath::HoleToolpathBatch& result)
{
    return result.validation.warningCount > 0 ||
        !result.machining.warnings.empty() ||
        std::any_of(
            result.diagnostics.begin(), result.diagnostics.end(),
            [](const toolpath::ToolpathDiagnostic& diagnostic) {
                return diagnostic.severity ==
                    toolpath::ToolpathDiagnosticSeverity::Warning;
            });
}

bool hasPocketWarning(const toolpath::PocketToolpathBatch& result)
{
    return result.validation.warningCount > 0 ||
        std::any_of(
            result.operations.begin(), result.operations.end(),
            [](const toolpath::PocketToolpathOperation& operation) {
                return !operation.clearing.warning.empty();
            });
}

template <typename Operation>
void reconcileStates(std::vector<Operation>& operations,
                     const MachiningJobDefinition& definition,
                     bool resultValid,
                     bool resultWarning)
{
    // 把某领域的整体校验结果投影到每个持久化工序：结果无效→Error；
    // 装夹缺失/无效/未确认→NeedsReview；否则按是否有警告设 Warning/Generated。
    for (Operation& operation : operations) {
        const SetupDefinition* setup =
            setupById(definition, operation.setupId);
        if (!resultValid) {
            operation.state = JobOperationState::Error;
        } else if (!setup || !setup->valid ||
                   !setup->confirmation.confirmed) {
            operation.state = JobOperationState::NeedsReview;
        } else {
            operation.state = resultWarning
                ? JobOperationState::Warning
                : JobOperationState::Generated;
        }
    }
}

template <typename Operation>
std::size_t markUnconfirmedOperations(
    std::vector<Operation>& operations,
    const MachiningJobDefinition& definition)
{
    // 把装夹未确认的工序统一置为 NeedsReview，返回本次新标记的数量。
    std::size_t marked = 0;
    for (Operation& operation : operations) {
        const SetupDefinition* setup =
            setupById(definition, operation.setupId);
        if (setup && setup->valid && setup->confirmation.confirmed)
            continue;
        if (operation.state != JobOperationState::NeedsReview) {
            operation.state = JobOperationState::NeedsReview;
            ++marked;
        }
    }
    return marked;
}

} // namespace

SetupSynchronizationReport MachiningJobApplicationService::captureSetups(
    MachiningJobDefinition& definition,
    const toolpath::MoldSetupWorkFrames& frames,
    const std::string& confirmationTime,
    const std::string& confirmationActor)
{
    // 用当前六面工作框重写每个 Setup 的派生值；只有 frame 已被人工确认时才
    // 保留确认，且输入指纹不变时保留原确认时间/操作者，避免每次保存刷新审计信息。
    SetupSynchronizationReport report;
    for (SetupDefinition& setup : definition.setups) {
        const toolpath::MoldFaceSetupWorkFrame* frame =
            frames.forFace(setup.face);
        if (!frame || !frame->valid) {
            setup.valid = false;
            if (setup.confirmation.confirmed)
                ++report.clearedConfirmationCount;
            setup.confirmation = {};
            ++report.invalidSetupCount;
            continue;
        }

        applyFrame(setup, *frame);
        ++report.validSetupCount;
        if (!frame->confirmed) {
            if (setup.confirmation.confirmed)
                ++report.clearedConfirmationCount;
            setup.confirmation = {};
            continue;
        }

        const std::string inputFingerprint =
            MachiningJobStorage::setupInputFingerprint(
                definition.part, setup).toStdString();
        if (setup.confirmation.confirmed &&
            setup.confirmation.confirmedInputFingerprint ==
                inputFingerprint) {
            ++report.retainedConfirmationCount;
        } else {
            setup.confirmation.confirmedAt = confirmationTime;
            setup.confirmation.confirmedBy = confirmationActor;
        }
        setup.confirmation.confirmed = true;
        setup.confirmation.confirmedInputFingerprint = inputFingerprint;
    }
    return report;
}

SetupSynchronizationReport
MachiningJobApplicationService::rebindLoadedSetups(
    MachiningJobDefinition& definition,
    const toolpath::MoldSetupWorkFrames& frames)
{
    // 加载后重绑定：以保存的 face/装夹索引重新派生坐标，并按重算后的输入指纹
    // 判断确认是否仍有效；之后清空碰撞场景并把未确认工序统一标记为复核。
    SetupSynchronizationReport report;
    for (SetupDefinition& setup : definition.setups) {
        const toolpath::MoldFaceSetupWorkFrame* frame =
            frames.forFace(setup.face);
        if (!frame || !frame->valid) {
            setup.valid = false;
            if (setup.confirmation.confirmed)
                ++report.clearedConfirmationCount;
            setup.confirmation = {};
            ++report.invalidSetupCount;
            continue;
        }

        SetupDefinition rebound = setup;
        applyFrame(rebound, *frame);
        const std::string expected =
            MachiningJobStorage::setupInputFingerprint(
                definition.part, rebound).toStdString();
        if (setup.confirmation.confirmed &&
            setup.confirmation.confirmedInputFingerprint == expected) {
            ++report.retainedConfirmationCount;
        } else {
            if (setup.confirmation.confirmed)
                ++report.clearedConfirmationCount;
            rebound.confirmation = {};
        }
        setup = std::move(rebound);
        ++report.validSetupCount;
    }

    // 历史碰撞场景只为格式兼容而读取，不参与当前六面 Job 重建。
    definition.collisionScene = {};
    report.operationsMarkedForReview += markUnconfirmedOperations(
        definition.holeOperations, definition);
    report.operationsMarkedForReview += markUnconfirmedOperations(
        definition.pocketOperations, definition);
    report.operationsMarkedForReview += markUnconfirmedOperations(
        definition.profileOperations, definition);
    report.operationsMarkedForReview += markUnconfirmedOperations(
        definition.finishingOperations, definition);
    return report;
}

void MachiningJobApplicationService::reconcileOperationStates(
    MachiningJobDefinition& definition,
    const toolpath::HoleToolpathBatch* holeResult,
    const toolpath::PocketToolpathBatch* pocketResult,
    const toolpath::ProfileToolpathBatch* profileResult,
    const toolpath::SurfaceFinishingBatch* finishingResult)
{
    if (holeResult) {
        reconcileStates(
            definition.holeOperations, definition,
            holeResult->validation.ok(), hasHoleWarning(*holeResult));
    }
    if (pocketResult) {
        reconcileStates(
            definition.pocketOperations, definition,
            pocketResult->validation.ok(), hasPocketWarning(*pocketResult));
    }
    if (profileResult) {
        const toolpath::ToolpathValidationReport validation =
            toolpath::ToolpathProgramValidator().validate(
                profileResult->toProgram(), profileResult->effectiveParams);
        reconcileStates(
            definition.profileOperations, definition,
            validation.ok(), validation.warningCount > 0 ||
                !profileResult->diagnostics.empty());
    }
    if (finishingResult) {
        reconcileStates(definition.finishingOperations, definition,
            !finishingResult->cancelled && finishingResult->validation.ok(), true);
    }
}

bool MachiningJobApplicationService::synchronizeForSave(
    MachiningJob& job,
    const MachiningJobSaveSynchronizationRequest& request,
    std::string* error)
{
    // 保存前完整同步：暂存运行期结果 → 写入保存请求值 → 捕获 Setup 与状态 →
    // 领域校验；成功后按新定义刷新运行期结果的执行指纹。
    if (error)
        error->clear();
    if (!request.setupFrames) {
        if (error)
            *error = "setup work frames are required";
        return false;
    }

    std::optional<toolpath::HoleToolpathBatch> holeResult;
    if (const toolpath::HoleToolpathBatch* current = job.holeResult())
        holeResult = *current;
    std::optional<toolpath::PocketToolpathBatch> pocketResult;
    if (const toolpath::PocketToolpathBatch* current = job.pocketResult())
        pocketResult = *current;
    std::optional<toolpath::ProfileToolpathBatch> profileResult;
    if (const toolpath::ProfileToolpathBatch* current = job.profileResult())
        profileResult = *current;
    std::optional<toolpath::SurfaceFinishingBatch> finishingResult;
    const std::string finishingFingerprint = job.runtime().finishingExecutionFingerprint;
    if (const auto* current = job.currentFinishingResult()) finishingResult = *current;
    MachiningJobDefinition& definition = job.definition();
    definition.modifiedAt = request.modifiedAt;
    definition.post = request.post;
    auto stock = request.stock;
    if (stock.sourceKind == StockSourceKind::StepModel &&
        stock.sourceKind == definition.stock.sourceKind &&
        stock.sourcePath == definition.stock.sourcePath && stock.sourceFingerprint.empty()) {
        stock.sourceFingerprint = definition.stock.sourceFingerprint;
    }
    definition.stock = std::move(stock);
    definition.activeToolLibraryId = request.activeToolLibraryId;
    definition.collisionScene = {};
    captureSetups(
        definition, *request.setupFrames, request.confirmationTime,
        request.confirmationActor);
    markUnconfirmedOperations(
        definition.profileOperations, definition);
    markUnconfirmedOperations(definition.finishingOperations, definition);
    reconcileOperationStates(
        definition,
        holeResult ? &*holeResult : nullptr,
        pocketResult ? &*pocketResult : nullptr,
        profileResult ? &*profileResult : nullptr,
        finishingResult ? &*finishingResult : nullptr);

    const std::vector<std::string> validationErrors =
        job.validationErrors();
    if (!validationErrors.empty()) {
        if (error)
            *error = validationErrors.front();
        return false;
    }

    if (holeResult) {
        job.setHoleResult(
            std::move(*holeResult),
            MachiningJobStorage::holeExecutionFingerprint(
                definition).toStdString());
    }
    if (pocketResult) {
        job.setPocketResult(
            std::move(*pocketResult),
            MachiningJobStorage::pocketExecutionFingerprint(
                definition).toStdString());
    }
    if (profileResult) {
        job.setProfileResult(
            std::move(*profileResult),
            MachiningJobStorage::profileExecutionFingerprint(
                definition).toStdString());
    }
    const std::string currentFingerprint = MachiningJobStorage::finishingExecutionFingerprint(definition).toStdString();
    if (finishingResult && currentFingerprint == finishingFingerprint)
        job.setFinishingResult(std::move(*finishingResult), currentFingerprint);
    else
        job.clearFinishingResult(); // Saving must never re-stamp stale toolpaths as current.
    return true;
}

} // namespace oap::manufacturing

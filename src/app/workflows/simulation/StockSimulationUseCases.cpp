// StockSimulationUseCases 实现：孔/腔毛坯仿真用例主流程与相机输入组装。
#include "app/workflows/simulation/StockSimulationUseCases.h"
#include "app/workflows/simulation/IpwDisplayMeshWorker.h"

#include "manufacturing/job/MachiningJobStorage.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/core/simulation/StockModelLoader.h"
#include "toolpath/workflow/compatibility/LegacyProfileSimulationAdapter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>

#include <QSettings>

#include <Precision.hxx>

namespace oap::app {
namespace {

// RAII：作用域结束时把协调器标记为仿真结束（finishSimulation）。
class SimulationFinishGuard {
public:
    explicit SimulationFinishGuard(StockSimulationCoordinator& coordinator)
        : m_coordinator(coordinator)
    {
    }

    ~SimulationFinishGuard()
    {
        m_coordinator.finishSimulation();
    }

private:
    StockSimulationCoordinator& m_coordinator;
};

// RAII：进入/退出忙状态回调（用于鼠标光标与禁用态）。
class BusyStateGuard {
public:
    explicit BusyStateGuard(const WorkflowBusyStateHandler& handler)
        : m_handler(handler)
    {
        if (m_handler)
            m_handler(true);
    }

    ~BusyStateGuard()
    {
        if (m_handler)
            m_handler(false);
    }

private:
    WorkflowBusyStateHandler m_handler;
};

} // namespace

// 孔毛坯仿真主流程：校验来源 → 应用设置 → 构造目标与初始毛坯 →
// 计算 STEP 指纹 → 运行仿真 → 安装结果并生成完成状态文本。
HoleStockRunOutcome HoleStockSimulationUseCase::execute(
    const HoleStockSimulationUseCaseRequest& request)
{
    HoleStockRunOutcome outcome;
    if (!request.session || !request.coordinator || !request.job ||
        !request.buildTarget) {
        outcome.error = QStringLiteral("孔毛胚仿真请求不完整。");
        return outcome;
    }

    QString error;
    if (!StockSimulationWorkflowController::validateSource(
            request.settings, error)) {
        outcome.error = error;
        return outcome;
    }
    StockSimulationWorkflowController::applyHoleSettings(
        *request.session, request.settings);

    TopoDS_Shape targetShape;
    int targetSolidCount = 0;
    if (!request.buildTarget(
            targetShape, targetSolidCount, error)) {
        outcome.error = error;
        return outcome;
    }

    TopoDS_Shape initialStock;
    int blockCount = 0;
    if (request.settings.sourceMode != 2) {
        if (!request.buildInitialStock ||
            !request.buildInitialStock(
                request.settings, initialStock, blockCount, error)) {
            outcome.error = error.isEmpty()
                ? QStringLiteral("无法构造孔加工初始毛坯。")
                : error;
            return outcome;
        }
    }

    QString stockSourceFingerprint;
    if (request.settings.sourceMode == 2) {
        stockSourceFingerprint =
            manufacturing::MachiningJobStorage::sourceFileFingerprint(
                request.settings.stepPath, &error);
        if (!error.isEmpty() || stockSourceFingerprint.isEmpty()) {
            outcome.error = error.isEmpty()
                ? QStringLiteral("无法计算毛坯 STEP 指纹。")
                : error;
            return outcome;
        }
    }

    const toolpath::HoleToolpathBatch* holeResult =
        request.job->holeResult();
    if (!holeResult) {
        outcome.error = QStringLiteral(
            "当前加工 Job 没有可仿真的孔工序结果。");
        return outcome;
    }

    HoleStockSimulationRequest runRequest;
    runRequest.initialStock = initialStock;
    runRequest.targetShape = targetShape;
    if (request.settings.sourceMode == 2) {
        runRequest.stockStepPath =
            request.settings.stepPath.toUtf8().toStdString();
    }
    runRequest.operations = holeResult->machining.operations;
    runRequest.fallbackParams = holeResult->effectiveParams;

    if (request.beforeRun)
        request.beforeRun();
    const std::shared_ptr<std::atomic_bool> cancellation =
        request.coordinator->beginSimulation();
    SimulationFinishGuard finish(*request.coordinator);
    HoleStockSimulationRunResult runResult = request.runner
        ? request.runner(std::move(runRequest), cancellation)
        : simulateHoleStockWithProgress(
              request.interaction.showProgress,
              std::move(runRequest), cancellation);

    if (request.closeRequested && request.closeRequested()) {
        outcome.closeRequested = true;
        return outcome;
    }
    if (!runResult.errorMessage.empty()) {
        outcome.error = QString::fromStdString(runResult.errorMessage);
        return outcome;
    }

    const bool cancelled = runResult.cancelled;
    StockSimulationWorkflowController::installHoleResult(
        *request.session, runResult, stockSourceFingerprint);
    outcome.ok = true;
    if (!request.session->hasHoleSimulation()) {
        outcome.status = cancelled
            ? QStringLiteral(
                  "孔毛胚仿真已取消，尚未产生可显示快照。")
            : QStringLiteral("孔毛胚仿真未产生可显示结果。");
        return outcome;
    }
    outcome.status =
        StockSimulationWorkflowController::holeCompletionStatus(
            *request.session, cancelled);
    return outcome;
}

// 从会话与预览工序构建对话框初始状态，并按装夹面分组生成选项。
PocketStockSimulationDialogState
PocketStockSimulationUseCase::buildDialogState(
    const StockSimulationSession& session,
    std::span<const toolpath::PocketToolpathOperation> operations,
    int displayQuality,
    bool triangulationEnabled,
    std::size_t displayCacheBudgetBytes,
    const QString& browseDirectory)
{
    PocketStockSimulationDialogState result;
    result.source.sourceMode = session.pocketSourceMode();
    result.source.marginX = session.pocketMarginX();
    result.source.marginY = session.pocketMarginY();
    result.source.marginZ = session.pocketMarginZ();
    result.source.stepPath = session.pocketStockStepPath();
    result.displayQuality = displayQuality;
    result.triangulationEnabled = triangulationEnabled;
    result.displayCacheMiB = static_cast<int>(
        displayCacheBudgetBytes / (1024u * 1024u));
    result.browseDirectory = browseDirectory;

    std::map<int, std::pair<MoldSetupFace, int>> setups;
    for (const auto& operation : operations) {
        auto& setup = setups[operation.setupIndex];
        setup.first = operation.setupFace;
        ++setup.second;
    }
    for (const auto& [setupIndex, setup] : setups) {
        const QString setupName =
            setup.first == MoldSetupFace::Unresolved
            ? QStringLiteral("未解析装夹面")
            : QStringLiteral("%1 / VIEW%2")
                  .arg(QString::fromLatin1(
                      moldSetupFaceName(setup.first)))
                  .arg(setupIndex + 1);
        result.setups.push_back({
            setupIndex,
            QStringLiteral("%1（%2 道腔工序）")
                .arg(setupName)
                .arg(setup.second)});
    }
    if (!result.setups.empty())
        result.selectedSetup = result.setups.front().setupIndex;
    return result;
}

void PocketStockSimulationUseCase::persistDisplaySettings(
    const PocketStockSimulationDialogState& settings)
{
    QSettings storage(
        QStringLiteral("OAP"),
        QStringLiteral("OpenAutoPath"));
    storage.setValue(
        QStringLiteral("ipw/display_quality"),
        settings.displayQuality);
    storage.setValue(
        QStringLiteral("ipw/triangulation_enabled"),
        settings.triangulationEnabled);
    storage.setValue(
        QStringLiteral("ipw/display_cache_mib"),
        settings.displayCacheMiB);
}

// 腔毛坯仿真主流程：校验来源 → 加载 STEP 或复用生成毛坯 →
// 准备工序 → 运行统一 IPW → 安装结果 → 组装逐工序临时帧与相机输入。
PocketStockSimulationUseCaseResult
PocketStockSimulationUseCase::execute(
    const PocketStockSimulationUseCaseRequest& request)
{
    PocketStockSimulationUseCaseResult result;
    if (!request.session || !request.coordinator || !request.job ||
        request.targetShape.IsNull()) {
        result.error = QStringLiteral("统一 IPW 腔加工仿真请求不完整。");
        return result;
    }

    QString error;
    if (!StockSimulationWorkflowController::validateSource(
            request.settings.source, error)) {
        result.failure = PocketStockSimulationFailure::InvalidSource;
        result.error = error;
        return result;
    }
    StockSimulationWorkflowController::applyPocketSettings(
        *request.session, request.settings.source);
    request.coordinator->setDisplayCacheBudgetBytes(
        static_cast<std::size_t>(request.settings.displayCacheMiB) *
        1024u * 1024u);
    if (request.persistSettings)
        persistDisplaySettings(request.settings);

    TopoDS_Shape initialStock = request.generatedInitialStock;
    QString importedFingerprint;
    if (request.settings.source.sourceMode == 2) {
        toolpath::StockModelLoadResult loaded;
        {
            BusyStateGuard busyState(request.interaction.setBusy);
            loaded = toolpath::StockModelLoader().loadStep(
                request.settings.source.stepPath.toUtf8().toStdString());
        }
        if (!loaded.ok()) {
            result.failure =
                PocketStockSimulationFailure::StockLoadFailed;
            result.error = QString::fromStdString(loaded.message);
            return result;
        }
        initialStock = loaded.shape;
        importedFingerprint =
            manufacturing::MachiningJobStorage::sourceFileFingerprint(
                request.settings.source.stepPath, &error);
        if (!error.isEmpty() || importedFingerprint.isEmpty()) {
            result.failure =
                PocketStockSimulationFailure::StockLoadFailed;
            result.error = error.isEmpty()
                ? QStringLiteral("无法计算毛坯 STEP 指纹。")
                : error;
            return result;
        }
    } else if (initialStock.IsNull()) {
        result.failure = PocketStockSimulationFailure::PreparationFailed;
        result.error = QStringLiteral("无法构造腔加工初始毛坯。");
        return result;
    }

    result.preparation =
        StockSimulationWorkflowController::preparePocket(
            *request.job,
            request.settings.selectedSetup,
            request.settings.source, importedFingerprint,
            request.settings.displayQuality,
            initialStock, request.targetShape);
    if (!result.preparation.ok) {
        result.failure = PocketStockSimulationFailure::PreparationFailed;
        result.error = result.preparation.error;
        return result;
    }
    // Simulation stock belongs to this request/session, not to toolpath inputs.
    // Do not invalidate the Job execution identity while starting a simulation.

    if (request.beforeRun)
        request.beforeRun();
    const std::shared_ptr<std::atomic_bool> cancellation =
        request.coordinator->beginSimulation();
    SimulationFinishGuard finish(*request.coordinator);
    result.preparation.request.exactTargetBounds = request.exactTargetBounds;
    result.runResult = request.runner
        ? request.runner(
              *request.job, std::move(result.preparation.request),
              cancellation)
        : simulateIpwWithProgress(
              request.interaction.showProgress, *request.job,
              std::move(result.preparation.request), cancellation);

    if (request.closeRequested && request.closeRequested()) {
        result.failure = PocketStockSimulationFailure::CloseRequested;
        return result;
    }
    if (result.runResult.cancelled) {
        result.failure = PocketStockSimulationFailure::Cancelled;
        result.error = QStringLiteral("统一 IPW 腔加工仿真已取消。");
        return result;
    }

    StockSimulationWorkflowController::installPocketResult(
        *request.session, *request.job,
        result.preparation, result.runResult);
    if (!request.session->hasPocketSimulation()) {
        result.failure = PocketStockSimulationFailure::EmptyResult;
        result.error = result.runResult.errorMessage.empty()
            ? QStringLiteral("统一 IPW 未产生可显示检查点。")
            : QString::fromStdString(result.runResult.errorMessage);
        return result;
    }
    if (result.runResult.failed) {
        result.warning = QString::fromStdString(
            result.runResult.errorMessage);
    }
    result.summary =
        StockSimulationWorkflowController::pocketSummary(
            *request.session, *request.job,
            result.preparation, result.runResult);

    result.camInput.stock = initialStock;
    result.camInput.target = request.targetShape;
    result.camInput.operations = result.preparation.operations;
    result.camInput.params = result.preparation.effectiveParams;
    result.camInput.options.quality = request.settings.displayQuality;
    // The authoritative Multi-Dexel/IPW result, validation and NC stream keep
    // the complete path. OpenGL playback is a bounded visualization: dense
    // jobs are simplified within the scene resolution and report that fact
    // instead of allocating millions of GPU/display motions.
    result.camInput.options.pathSimplificationTolerance = 0.0;
    result.camInput.options.allowAutomaticPathSimplification = true;
    result.camInput.options.maxDisplayMotionCount = 100'000u;
    // 为每个匹配 IPW 节点的预览工序构造临时帧源：从父检查点重放
    // 前 N 个移动，供逐移动显示与视觉验收使用。
    result.camInput.operationInitialIpwSources.resize(
        result.camInput.operations.size());
    const auto& ipwNodes = request.job->ipwSession().nodes();
    // 首帧材料取执行顺序中第一个槽腔节点的父检查点。
    // 这是真正的“槽腔加工前 IPW”，不应依赖预览刀路与
    // Job 定义的运行时 ID 是否恰好命中。
    for (const manufacturing::ipw::IpwNode& node : ipwNodes) {
        if (node.operation.kind !=
            manufacturing::ipw::IpwOperationKind::Pocket) {
            continue;
        }
        const auto parentCheckpoint =
            request.job->runtime().ipwCheckpoints.find(
                node.parentStateKey);
        if (parentCheckpoint && parentCheckpoint->dexelStock &&
            parentCheckpoint->dexelStock->valid()) {
            result.camInput.initialIpwStock =
                parentCheckpoint->dexelStock;
            break;
        }
    }
    for (const manufacturing::ipw::IpwNode& node : ipwNodes) {
        std::vector<std::size_t> matchingOperationIndices;
        std::vector<toolpath::PocketToolpathOperation>
            matchingOperations;
        for (std::size_t operationIndex = 0u;
             operationIndex < result.camInput.operations.size();
             ++operationIndex) {
            const toolpath::PocketToolpathOperation& operation =
                result.camInput.operations[operationIndex];
            if (!StockSimulationWorkflowController::
                    pocketRuntimeIdMatches(
                        node.operation.id,
                        operation.operationId)) {
                continue;
            }
            matchingOperationIndices.push_back(operationIndex);
            matchingOperations.push_back(operation);
        }
        if (matchingOperations.empty())
            continue;
        const auto parentCheckpoint =
            request.job->runtime().ipwCheckpoints.find(
                node.parentStateKey);
        if (!parentCheckpoint ||
            !parentCheckpoint->dexelStock ||
            !parentCheckpoint->dexelStock->valid()) {
            continue;
        }
        const auto operations = std::make_shared<const std::vector<
            toolpath::PocketToolpathOperation>>(
                std::move(matchingOperations));
        std::size_t completedMoveCount = 0u;
        for (std::size_t localIndex = 0u;
             localIndex < operations->size(); ++localIndex) {
            auto source =
                std::make_shared<IpwTransientFrameSource>();
            source->parentStock = parentCheckpoint->dexelStock;
            source->operations = operations;
            source->params = result.camInput.params;
            source->options = result.preparation.pocketOptions;
            source->completedMoveCount = completedMoveCount;
            source->sequenceIndex = node.operation.sequence;
            result.camInput.operationInitialIpwSources[
                matchingOperationIndices[localIndex]] =
                    std::move(source);
            completedMoveCount +=
                (*operations)[localIndex].clearing.moves.size();
        }
    }
    if (result.runResult.execution.finalCheckpoint) {
        result.camInput.finalIpwStock =
            result.runResult.execution.finalCheckpoint->dexelStock;
        if (request.settings.triangulationEnabled &&
            result.camInput.finalIpwStock) {
            const IpwDisplayQualityProfile qualityProfile =
                ipwDisplayQualityProfile(
                    request.settings.displayQuality,
                    result.camInput.finalIpwStock->resolution());
            IpwDisplayArtifactKey key;
            key.contentFingerprint =
                result.camInput.finalIpwStock->contentFingerprint();
            key.kind = IpwDisplayArtifactKind::StockCheckpoint;
            key.quality = qualityProfile.quality;
            const IpwDisplayArtifact artifact =
                request.coordinator->findDisplayArtifact(key);
            result.camInput.finalIpwDisplayMesh = artifact.mesh;
            if (!result.camInput.finalIpwDisplayMesh.IsNull()) {
                result.camInput.finalIpwDisplayMeshFingerprint =
                    toolpath::displayMeshContentFingerprint(
                        result.camInput.finalIpwDisplayMesh);
            }
        }
    }

    result.camInput.visualAcceptanceDirectory =
        qEnvironmentVariable(
            "OAP_IPW_VISUAL_ACCEPTANCE_DIR").trimmed();
    if (!result.camInput.visualAcceptanceDirectory.isEmpty()) {
        const auto view = selectVisualAcceptanceView(
            result.camInput.operations,
            result.camInput.params.tool.diameter);
        if (view) {
            result.camInput.visualAcceptanceView = *view;
            if (result.camInput.finalIpwStock) {
                result.camInput.visualAcceptanceView.linearDeflection =
                    result.camInput.finalIpwStock->resolution() /
                    std::sqrt(2.0);
                result.camInput.visualAcceptanceView.angularDeflection = 0.0;
            }
        }
    }

    result.failure = PocketStockSimulationFailure::None;
    return result;
}

PocketStockSimulationDialogState
ProfileStockSimulationUseCase::buildDialogState(
    const StockSimulationSession& session,
    std::span<const toolpath::ProfileToolpathOperation> operations,
    int displayQuality,
    bool triangulationEnabled,
    std::size_t displayCacheBudgetBytes,
    const QString& browseDirectory)
{
    PocketStockSimulationDialogState result;
    result.source.sourceMode = session.pocketSourceMode();
    result.source.marginX = session.pocketMarginX();
    result.source.marginY = session.pocketMarginY();
    result.source.marginZ = session.pocketMarginZ();
    result.source.stepPath = session.pocketStockStepPath();
    result.displayQuality = displayQuality;
    result.triangulationEnabled = triangulationEnabled;
    result.displayCacheMiB = static_cast<int>(
        displayCacheBudgetBytes / (1024u * 1024u));
    result.browseDirectory = browseDirectory;

    std::map<int, std::pair<MoldSetupFace, int>> setups;
    for (const toolpath::ProfileToolpathOperation& operation : operations) {
        auto& setup = setups[operation.setupIndex];
        setup.first = operation.setupFace;
        ++setup.second;
    }
    for (const auto& [setupIndex, setup] : setups) {
        const QString setupName =
            setup.first == MoldSetupFace::Unresolved
            ? QStringLiteral("未解析装夹面")
            : QStringLiteral("%1 / VIEW%2")
                  .arg(QString::fromLatin1(
                      moldSetupFaceName(setup.first)))
                  .arg(setupIndex + 1);
        result.setups.push_back({
            setupIndex,
            QStringLiteral("%1（%2 道外轮廓工序）")
                .arg(setupName)
                .arg(setup.second)});
    }
    if (!result.setups.empty())
        result.selectedSetup = result.setups.front().setupIndex;
    return result;
}

ProfileStockSimulationUseCaseResult
ProfileStockSimulationUseCase::execute(
    const ProfileStockSimulationUseCaseRequest& request)
{
    ProfileStockSimulationUseCaseResult result;
    if (!request.session || !request.coordinator || !request.job ||
        !request.operations || !request.effectiveParams ||
        request.targetShape.IsNull()) {
        result.error = QStringLiteral(
            "统一 IPW 外轮廓仿真请求不完整。");
        return result;
    }

    QString error;
    if (!StockSimulationWorkflowController::validateSource(
            request.settings.source, error)) {
        result.failure = PocketStockSimulationFailure::InvalidSource;
        result.error = error;
        return result;
    }
    StockSimulationWorkflowController::applyPocketSettings(
        *request.session, request.settings.source);
    request.coordinator->setDisplayCacheBudgetBytes(
        static_cast<std::size_t>(request.settings.displayCacheMiB) *
        1024u * 1024u);
    if (request.persistSettings) {
        PocketStockSimulationUseCase::persistDisplaySettings(
            request.settings);
    }

    TopoDS_Shape initialStock = request.generatedInitialStock;
    QString importedFingerprint;
    if (request.settings.source.sourceMode == 2) {
        toolpath::StockModelLoadResult loaded;
        {
            BusyStateGuard busyState(request.interaction.setBusy);
            loaded = toolpath::StockModelLoader().loadStep(
                request.settings.source.stepPath.toUtf8().toStdString());
        }
        if (!loaded.ok()) {
            result.failure =
                PocketStockSimulationFailure::StockLoadFailed;
            result.error = QString::fromStdString(loaded.message);
            return result;
        }
        initialStock = loaded.shape;
        importedFingerprint =
            manufacturing::MachiningJobStorage::sourceFileFingerprint(
                request.settings.source.stepPath, &error);
        if (!error.isEmpty() || importedFingerprint.isEmpty()) {
            result.failure =
                PocketStockSimulationFailure::StockLoadFailed;
            result.error = error.isEmpty()
                ? QStringLiteral("无法计算毛坯 STEP 指纹。")
                : error;
            return result;
        }
    } else if (initialStock.IsNull()) {
        result.failure =
            PocketStockSimulationFailure::PreparationFailed;
        result.error = QStringLiteral("无法构造外轮廓初始毛坯。");
        return result;
    }

    result.preparation =
        StockSimulationWorkflowController::prepareProfile(
            *request.job, *request.operations,
            request.settings.selectedSetup,
            request.settings.source, importedFingerprint,
            request.settings.displayQuality,
            *request.effectiveParams,
            initialStock, request.targetShape);
    if (!result.preparation.ok) {
        result.failure =
            PocketStockSimulationFailure::PreparationFailed;
        result.error = result.preparation.error;
        return result;
    }
    request.job->definition().stock =
        result.preparation.stockDefinition;

    if (request.beforeRun)
        request.beforeRun();
    const std::shared_ptr<std::atomic_bool> cancellation =
        request.coordinator->beginSimulation();
    SimulationFinishGuard finish(*request.coordinator);
    result.preparation.request.exactTargetBounds = request.exactTargetBounds;
    result.runResult = request.runner
        ? request.runner(
              *request.job,
              std::move(result.preparation.request), cancellation)
        : simulateIpwWithProgress(
              request.interaction.showProgress, *request.job,
              std::move(result.preparation.request), cancellation);

    if (request.closeRequested && request.closeRequested()) {
        result.failure =
            PocketStockSimulationFailure::CloseRequested;
        return result;
    }
    if (result.runResult.cancelled) {
        result.failure = PocketStockSimulationFailure::Cancelled;
        result.error = QStringLiteral(
            "统一 IPW 外轮廓仿真已取消。");
        return result;
    }
    StockSimulationWorkflowController::installProfileResult(
        *request.session, *request.job,
        result.preparation, result.runResult);
    if (!request.session->hasPocketSimulation()) {
        result.failure = PocketStockSimulationFailure::EmptyResult;
        result.error = result.runResult.errorMessage.empty()
            ? QStringLiteral("统一 IPW 未产生可显示检查点。")
            : QString::fromStdString(result.runResult.errorMessage);
        return result;
    }
    if (result.runResult.failed) {
        result.warning = QString::fromStdString(
            result.runResult.errorMessage);
    }
    result.summary =
        StockSimulationWorkflowController::profileSummary(
            *request.session, *request.job,
            result.preparation, result.runResult);

    // Profile keeps its own Job/IPW node kind.  The interactive fixed-axis
    // player consumes a read-only Pocket operation carrier containing the
    // exact same authoritative moves/primitives used by Profile IPW.
    result.camInput.stock = initialStock;
    result.camInput.target = request.targetShape;
    result.camInput.operations =
        toolpath::legacyProfileSimulationOperations(
            result.preparation.operations);
    result.camInput.params = *request.effectiveParams;
    result.camInput.options.quality = request.settings.displayQuality;
    result.camInput.options.pathSimplificationTolerance = 0.0;
    result.camInput.options.allowAutomaticPathSimplification = false;
    result.camInput.options.maxDisplayMotionCount = 2'000'000u;
    result.camInput.operationInitialIpwSources.resize(
        result.camInput.operations.size());

    const auto& ipwNodes = request.job->ipwSession().nodes();
    for (const manufacturing::ipw::IpwNode& node : ipwNodes) {
        if (node.operation.kind !=
            manufacturing::ipw::IpwOperationKind::Profile) {
            continue;
        }
        const auto parentCheckpoint =
            request.job->runtime().ipwCheckpoints.find(
                node.parentStateKey);
        if (parentCheckpoint && parentCheckpoint->dexelStock &&
            parentCheckpoint->dexelStock->valid()) {
            result.camInput.initialIpwStock =
                parentCheckpoint->dexelStock;
            break;
        }
    }
    for (const manufacturing::ipw::IpwNode& node : ipwNodes) {
        if (node.operation.kind !=
            manufacturing::ipw::IpwOperationKind::Profile) {
            continue;
        }
        std::vector<std::size_t> matchingOperationIndices;
        std::vector<toolpath::PocketToolpathOperation>
            matchingOperations;
        for (std::size_t operationIndex = 0u;
             operationIndex < result.camInput.operations.size();
             ++operationIndex) {
            const toolpath::PocketToolpathOperation& operation =
                result.camInput.operations[operationIndex];
            if (!StockSimulationWorkflowController::
                    pocketRuntimeIdMatches(
                        node.operation.id,
                        operation.operationId)) {
                continue;
            }
            matchingOperationIndices.push_back(operationIndex);
            matchingOperations.push_back(operation);
        }
        if (matchingOperations.empty())
            continue;
        const auto parentCheckpoint =
            request.job->runtime().ipwCheckpoints.find(
                node.parentStateKey);
        if (!parentCheckpoint || !parentCheckpoint->dexelStock ||
            !parentCheckpoint->dexelStock->valid()) {
            continue;
        }
        const auto operations = std::make_shared<const std::vector<
            toolpath::PocketToolpathOperation>>(
                std::move(matchingOperations));
        std::size_t completedMoveCount = 0u;
        for (std::size_t localIndex = 0u;
             localIndex < operations->size(); ++localIndex) {
            auto source =
                std::make_shared<IpwTransientFrameSource>();
            source->parentStock = parentCheckpoint->dexelStock;
            source->operations = operations;
            source->params = result.camInput.params;
            source->options = result.preparation.ipwOptions;
            source->completedMoveCount = completedMoveCount;
            source->sequenceIndex = node.operation.sequence;
            result.camInput.operationInitialIpwSources[
                matchingOperationIndices[localIndex]] =
                    std::move(source);
            completedMoveCount +=
                (*operations)[localIndex].clearing.moves.size();
        }
    }
    if (result.runResult.execution.finalCheckpoint) {
        result.camInput.finalIpwStock =
            result.runResult.execution.finalCheckpoint->dexelStock;
        if (request.settings.triangulationEnabled &&
            result.camInput.finalIpwStock) {
            const IpwDisplayQualityProfile qualityProfile =
                ipwDisplayQualityProfile(
                    request.settings.displayQuality,
                    result.camInput.finalIpwStock->resolution());
            IpwDisplayArtifactKey key;
            key.contentFingerprint =
                result.camInput.finalIpwStock->contentFingerprint();
            key.kind = IpwDisplayArtifactKind::StockCheckpoint;
            key.quality = qualityProfile.quality;
            const IpwDisplayArtifact artifact =
                request.coordinator->findDisplayArtifact(key);
            result.camInput.finalIpwDisplayMesh = artifact.mesh;
            if (!result.camInput.finalIpwDisplayMesh.IsNull()) {
                result.camInput.finalIpwDisplayMeshFingerprint =
                    toolpath::displayMeshContentFingerprint(
                        result.camInput.finalIpwDisplayMesh);
            }
        }
    }
    result.camInput.visualAcceptanceDirectory =
        qEnvironmentVariable(
            "OAP_IPW_VISUAL_ACCEPTANCE_DIR").trimmed();
    if (!result.camInput.visualAcceptanceDirectory.isEmpty()) {
        const auto view = PocketStockSimulationUseCase::
            selectVisualAcceptanceView(
                result.camInput.operations,
                result.camInput.params.tool.diameter);
        if (view) {
            result.camInput.visualAcceptanceView = *view;
            if (result.camInput.finalIpwStock) {
                result.camInput.visualAcceptanceView.linearDeflection =
                    result.camInput.finalIpwStock->resolution() /
                    std::sqrt(2.0);
                result.camInput.visualAcceptanceView.angularDeflection = 0.0;
            }
        }
    }
    result.failure = PocketStockSimulationFailure::None;
    return result;
}

// 从腔操作中挑选视觉验收视图：按刀路在刀具轴正交平面上的投影面积
// 择优，取面积极大的候选并构造带 30°/45° 倾斜的相机姿态。
std::optional<PocketCamVisualAcceptanceView>
PocketStockSimulationUseCase::selectVisualAcceptanceView(
    const std::vector<toolpath::PocketToolpathOperation>& operations,
    double toolDiameter)
{
    std::optional<PocketCamVisualAcceptanceView> best;
    double bestArea = -1.0;
    const double toolRadius = std::max(0.0, 0.5 * toolDiameter);
    for (const toolpath::PocketToolpathOperation& operation : operations) {
        if (operation.outwardToolAxis.SquareMagnitude() <=
            Precision::SquareConfusion()) {
            continue;
        }
        const gp_Dir eyeDirection(operation.outwardToolAxis);
        const gp_Vec axis(eyeDirection);
        gp_Vec up = std::abs(eyeDirection.Z()) < 0.9
            ? gp_Vec(0.0, 0.0, 1.0)
            : gp_Vec(0.0, 1.0, 0.0);
        up -= axis.Multiplied(up.Dot(axis));
        if (up.SquareMagnitude() <= Precision::SquareConfusion())
            continue;
        up.Normalize();
        gp_Vec right = up.Crossed(axis);
        if (right.SquareMagnitude() <= Precision::SquareConfusion())
            continue;
        right.Normalize();

        double minimumRight = std::numeric_limits<double>::max();
        double maximumRight = -std::numeric_limits<double>::max();
        double minimumUp = std::numeric_limits<double>::max();
        double maximumUp = -std::numeric_limits<double>::max();
        double minimumAxis = std::numeric_limits<double>::max();
        double maximumAxis = -std::numeric_limits<double>::max();
        std::size_t pointCount = 0u;
        const auto includePoint = [&](const gp_Pnt& point) {
            const gp_Vec value(point.XYZ());
            const double rightValue = value.Dot(right);
            const double upValue = value.Dot(up);
            const double axisValue = value.Dot(axis);
            minimumRight = std::min(minimumRight, rightValue);
            maximumRight = std::max(maximumRight, rightValue);
            minimumUp = std::min(minimumUp, upValue);
            maximumUp = std::max(maximumUp, upValue);
            minimumAxis = std::min(minimumAxis, axisValue);
            maximumAxis = std::max(maximumAxis, axisValue);
            ++pointCount;
        };
        for (const toolpath::ToolpathMove& move :
             operation.clearing.moves) {
            if (move.type != toolpath::ToolpathMoveType::Cut)
                continue;
            includePoint(move.from);
            includePoint(move.to);
        }
        if (pointCount == 0u)
            continue;

        const double paddedWidth =
            std::max(0.0, maximumRight - minimumRight) +
            2.0 * toolRadius;
        const double paddedHeight =
            std::max(0.0, maximumUp - minimumUp) +
            2.0 * toolRadius;
        const double area = paddedWidth * paddedHeight;
        const QString operationId =
            QString::fromStdString(operation.operationId);
        if (best &&
            (area < bestArea ||
             (std::abs(area - bestArea) <= Precision::Confusion() &&
              operationId >= best->operationId))) {
            continue;
        }

        const double centerRight =
            0.5 * (minimumRight + maximumRight);
        const double centerUp = 0.5 * (minimumUp + maximumUp);
        const double centerAxis =
            0.5 * (minimumAxis + maximumAxis);
        gp_Vec center = right.Multiplied(centerRight);
        center += up.Multiplied(centerUp);
        center += axis.Multiplied(centerAxis);

        PocketCamVisualAcceptanceView candidate;
        candidate.valid = true;
        candidate.center = gp_Pnt(center.XYZ());
        constexpr double tiltRadians =
            30.0 * std::numbers::pi / 180.0;
        constexpr double azimuthRadians =
            45.0 * std::numbers::pi / 180.0;
        gp_Vec tiltedEye = axis.Multiplied(std::cos(tiltRadians));
        tiltedEye += right.Multiplied(
            std::sin(tiltRadians) * std::cos(azimuthRadians));
        tiltedEye += up.Multiplied(
            std::sin(tiltRadians) * std::sin(azimuthRadians));
        tiltedEye.Normalize();
        gp_Vec cameraUp =
            up - tiltedEye.Multiplied(up.Dot(tiltedEye));
        cameraUp.Normalize();
        candidate.eyeDirection = gp_Dir(tiltedEye);
        candidate.up = gp_Dir(cameraUp);
        candidate.halfHeight = std::max(
            std::max(toolDiameter, 1.0),
            0.6 * std::max(paddedWidth, paddedHeight));
        candidate.operationId = operationId;
        best = std::move(candidate);
        bestArea = area;
    }
    return best;
}

} // namespace oap::app

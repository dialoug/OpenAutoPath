#include "app/workflows/simulation/StockSimulationUseCases.h"

#include "manufacturing/job/MachiningJobStorage.h"
#include "manufacturing/program/MachiningProgramAssembler.h"
#include "manufacturing/ipw/HoleMultiDexelUpdater.h"
#include "toolpath/core/simulation/StockModelLoader.h"
#include "toolpath/core/PocketToolBinding.h"
#include <algorithm>

namespace oap::app {
namespace {
bool prepareProjectPlayback(const manufacturing::MachiningJob& job,
    const toolpath::StockSimulationOptions& options, PocketCamSimulatorInput& input, QString& error,
    const std::vector<std::string>& includedOperationIds, bool engravingOnly, int selectedSetup)
{
    auto assembled = manufacturing::MachiningProgramAssembler().assemble(job, includedOperationIds);
    if (!assembled.ok()) {
        error = QStringLiteral("项目动画无法组装当前有效工序。");
        return false;
    }
    auto operations = std::make_shared<std::vector<toolpath::PocketToolpathOperation>>();
    std::vector<toolpath::DexelToolProfileDefinition> profiles;
    std::vector<toolpath::ToolpathParams> parameters;
    std::vector<std::shared_ptr<IpwTransientFrameSource>> sources;
    std::size_t globalMoves = 0;
    for (const auto& mapping : assembled.mappings) {
        if (engravingOnly && (mapping.kind != manufacturing::JobOperationKind::SurfaceFinishing ||
            (selectedSetup >= 0 && mapping.setupIndex != selectedSetup))) continue;
        const auto* setup = job.setupById(mapping.setupId);
        const auto& nodes = job.ipwSession().nodes();
        const auto node = std::find_if(nodes.begin(), nodes.end(), [&](const auto& item) {
            return item.operation.id == mapping.formalOperationId;
        });
        if (!setup || node == nodes.end()) {
            error = QStringLiteral("项目动画缺少正式工序或装夹映射。");
            return false;
        }
        const auto parent = job.runtime().ipwCheckpoints.find(node->parentStateKey);
        if (!parent || !parent->dexelStock || !parent->dexelStock->valid()) {
            error = QStringLiteral("工序 %1 缺少加工前 IPW 检查点。").arg(QString::fromStdString(mapping.formalOperationId));
            return false;
        }
        if (!input.initialIpwStock) input.initialIpwStock = parent->dexelStock;
        const auto parentMoveCount = globalMoves;
        for (std::size_t i = 0; i < mapping.programOperationCount; ++i) {
            auto& runtime = assembled.program.operations[mapping.firstProgramOperation + i];
            toolpath::PocketToolpathOperation carrier;
            carrier.operationId = runtime.operationId;
            carrier.tool = runtime.tool;
            carrier.outwardToolAxis = gp_Vec(setup->zAxis.x, setup->zAxis.y, setup->zAxis.z);
            carrier.clearing.moves = std::move(runtime.moves);
            carrier.clearing.primitives = std::move(runtime.primitives);
            toolpath::DexelToolProfileDefinition profile;
            toolpath::ToolpathParams params;
            using Kind = manufacturing::JobOperationKind;
            if (mapping.kind == Kind::Pocket) params = job.currentPocketResult()->effectiveParams;
            else if (mapping.kind == Kind::Profile) params = job.profileResult()->effectiveParams;
            else if (mapping.kind == Kind::Hole) params = job.holeResult()->effectiveParams;
            else {
                const auto* batch = job.currentFinishingResult();
                const auto found = std::find_if(batch->operations.begin(), batch->operations.end(),
                    [&](const auto& item) { return item.operationId == carrier.operationId; });
                if (found == batch->operations.end()) {
                    error = QStringLiteral("项目动画缺少雕刻刀路参数。");
                    return false;
                }
                params = found->result.toolpathParams();
            }
            params = toolpath::bindPocketToolParameters(params, carrier.tool);
            profile.diameter = params.tool.diameter;
            profile.cuttingLength = params.tool.cuttingEdgeLength;
            profile.cornerRadius = params.tool.cornerRadius;
            if (mapping.kind == manufacturing::JobOperationKind::Hole) {
                const auto* batch = job.holeResult();
                const auto found = std::find_if(batch->machining.operations.begin(), batch->machining.operations.end(),
                    [&](const auto& item) { return item.operationId == carrier.operationId; });
                if (found == batch->machining.operations.end()) {
                    error = QStringLiteral("项目动画缺少孔工序的实际刀具轮廓。");
                    return false;
                }
                profile = manufacturing::ipw::holeDexelToolProfile(*found, batch->effectiveParams);
                carrier.outwardToolAxis = gp_Vec(found->bottom, found->entry);
            }
            auto source = std::make_shared<IpwTransientFrameSource>();
            source->parentStock = parent->dexelStock;
            source->firstMoveCount = parentMoveCount;
            source->completedMoveCount = globalMoves;
            source->options = options;
            source->sequenceIndex = 0;
            sources.push_back(source);
            profiles.push_back(profile);
            parameters.push_back(params);
            globalMoves += carrier.clearing.moves.size();
            operations->push_back(std::move(carrier));
        }
    }
    for (auto& source : sources) {
        source->operations = operations;
        source->toolProfiles = profiles;
        source->operationParameters = parameters;
        source->params = input.params;
        input.operationInitialIpwSources.push_back(std::move(source));
    }
    input.authoritativeOperations = std::move(operations);
    input.options.authoritativeIpwPlayback = true;
    return !input.operationInitialIpwSources.empty();
}

class ProjectSimulationFinishGuard {
public:
    explicit ProjectSimulationFinishGuard(StockSimulationCoordinator& coordinator)
        : m_coordinator(coordinator) {}
    ~ProjectSimulationFinishGuard() { m_coordinator.finishSimulation(); }
private:
    StockSimulationCoordinator& m_coordinator;
};
}

ProjectStockSimulationUseCaseResult ProjectStockSimulationUseCase::execute(
    const ProjectStockSimulationUseCaseRequest& request)
{
    ProjectStockSimulationUseCaseResult result;
    if (!request.session || !request.coordinator || !request.job || request.targetShape.IsNull()) {
        result.error = QStringLiteral("加工项目 IPW 请求不完整。");
        return result;
    }
    QString error;
    if (!StockSimulationWorkflowController::validateSource(request.settings.source, error)) {
        result.failure = PocketStockSimulationFailure::InvalidSource;
        result.error = error;
        return result;
    }
    TopoDS_Shape initialStock = request.generatedInitialStock;
    QString importedFingerprint;
    if (request.settings.source.sourceMode == 2) {
        const auto loaded = toolpath::StockModelLoader().loadStep(
            request.settings.source.stepPath.toUtf8().toStdString());
        if (!loaded.ok()) {
            result.failure = PocketStockSimulationFailure::StockLoadFailed;
            result.error = QString::fromStdString(loaded.message);
            return result;
        }
        initialStock = loaded.shape;
        importedFingerprint = manufacturing::MachiningJobStorage::sourceFileFingerprint(
            request.settings.source.stepPath, &error);
        if (!error.isEmpty() || importedFingerprint.isEmpty()) {
            result.failure = PocketStockSimulationFailure::StockLoadFailed;
            result.error = error.isEmpty() ? QStringLiteral("无法计算毛坯 STEP 指纹。") : error;
            return result;
        }
    }
    if (initialStock.IsNull()) {
        result.failure = PocketStockSimulationFailure::PreparationFailed;
        result.error = QStringLiteral("无法构造项目初始毛坯。");
        return result;
    }
    result.preparation = StockSimulationWorkflowController::prepareProject(
        *request.job, request.settings.source, importedFingerprint,
        request.settings.displayQuality, initialStock, request.targetShape,
        request.engravingOnly, request.settings.selectedSetup);
    if (!result.preparation.ok) {
        result.failure = PocketStockSimulationFailure::PreparationFailed;
        result.error = result.preparation.error;
        return result;
    }
    StockSimulationWorkflowController::applyPocketSettings(*request.session, request.settings.source);
    request.coordinator->setDisplayCacheBudgetBytes(
        static_cast<std::size_t>(request.settings.displayCacheMiB) * 1024u * 1024u);
    if (request.persistSettings)
        PocketStockSimulationUseCase::persistDisplaySettings(request.settings);
    // This verifies the installed program against the chosen simulation stock.
    // Its identity is carried by request.initialStockFingerprint; changing the
    // persisted planning stock here would invalidate the very toolpaths being
    // simulated. Job stock changes belong to the project edit/rebuild workflow.
    if (request.beforeRun)
        request.beforeRun();
    const auto cancellation = request.coordinator->beginSimulation();
    ProjectSimulationFinishGuard finish(*request.coordinator);
    result.preparation.request.exactTargetBounds = request.exactTargetBounds;
    result.runResult = request.runner
        ? request.runner(*request.job, result.preparation.request, cancellation)
        : simulateIpwWithProgress(request.interaction.showProgress, *request.job,
            result.preparation.request, cancellation);
    if (request.closeRequested && request.closeRequested()) {
        result.failure = PocketStockSimulationFailure::CloseRequested;
        return result;
    }
    if (result.runResult.cancelled) {
        result.failure = PocketStockSimulationFailure::Cancelled;
        result.error = QStringLiteral("加工项目 IPW 已取消。");
        return result;
    }
    if (result.runResult.failed || !result.runResult.execution.complete()) {
        result.failure = PocketStockSimulationFailure::EmptyResult;
        result.error = QString::fromStdString(result.runResult.errorMessage);
        if (result.error.isEmpty())
            result.error = QStringLiteral("加工项目 IPW 未完整完成。");
        return result;
    }
    StockSimulationWorkflowController::installProjectResult(
        *request.session, *request.job, result.preparation, result.runResult);
    if (!request.session->hasPocketSimulation()) {
        result.failure = PocketStockSimulationFailure::EmptyResult;
        result.error = QStringLiteral("加工项目 IPW 没有可显示的检查点。");
        return result;
    }
    result.summary = StockSimulationWorkflowController::projectSummary(
        *request.session, *request.job, result.preparation, result.runResult);
    if (request.engravingOnly) {
        result.summary.prepend(QStringLiteral("雕刻 IPW（含前序材料准备）｜"));
        result.camInput.title = QStringLiteral("雕刻 IPW — 刀具动画");
    }
    result.camInput.stock = initialStock;
    result.camInput.target = request.targetShape;
    result.camInput.options.quality = request.settings.displayQuality;
    result.camInput.finalIpwStock = result.runResult.execution.finalCheckpoint
        ? result.runResult.execution.finalCheckpoint->dexelStock : nullptr;
    if (!prepareProjectPlayback(*request.job, result.preparation.ipwOptions,
            result.camInput, result.playbackError, result.preparation.includedOperationIds,
            request.engravingOnly, request.settings.selectedSetup) && result.playbackError.isEmpty())
        result.playbackError = QStringLiteral("项目没有可播放的工序。");
    result.warning = QStringLiteral("IPW 为材料去除仿真；不替代刀柄、夹具及机床连续运动碰撞验证。");
    result.failure = PocketStockSimulationFailure::None;
    return result;
}

} // namespace oap::app

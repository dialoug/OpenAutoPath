#include "toolpath/core/HeavyJobScheduler.h"
#include <QCoreApplication>
// 通过共享重计算队列执行毛坯读取和布尔减料，通过交互端口发布进度。
#include "app/workflows/hole/HoleStockSimulationRunner.h"

#include "toolpath/core/ToolpathParallel.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <utility>

#include <BRepAlgoAPI_Fuse.hxx>
#include <OSD.hxx>
#include <Precision.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <gp_Dir.hxx>

#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/simulation/StockModelLoader.h"
#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include "toolpath/workflow/hole/HoleSimulationAdapter.h"
#include "app/workflows/simulation/detail/IpwDisplayArtifactCache.h"

namespace oap::app {
namespace {

// 把孔工序按覆盖的源孔分组，为每个孔构建特征余量范围（圆柱段）；
// 同一孔的多段覆盖用布尔融合合并为单一范围。
std::vector<toolpath::StockFeatureAllowanceScope>
holeFeatureAllowanceScopes(
    const std::vector<toolpath::HoleMachiningOperation>& operations)
{
    struct Accumulator {
        toolpath::StockFeatureAllowanceScope scope;
    };
    std::map<int, Accumulator> accumulated;
    for (const toolpath::HoleMachiningOperation& operation : operations) {
        const gp_Vec axis(operation.entry, operation.bottom);
        const double depth = axis.Magnitude();
        if (!std::isfinite(operation.radius) ||
            operation.radius <= 0.0 || !std::isfinite(depth) ||
            depth <= Precision::Confusion()) {
            continue;
        }
        double bottomExtension = 0.0;
        if (operation.hasCommandedBottom) {
            const double commandedDepth = gp_Vec(
                operation.entry, operation.commandedBottom)
                    .Dot(gp_Vec(gp_Dir(axis)));
            if (std::isfinite(commandedDepth)) {
                bottomExtension = std::max(
                    0.0, commandedDepth - depth);
            }
        }
        const TopoDS_Shape region =
            toolpath::buildHoleFeatureAllowanceRegion(
                operation.entry, operation.bottom,
                operation.radius, Precision::Confusion(),
                bottomExtension + Precision::Confusion());
        if (region.IsNull())
            continue;

        std::vector<int> sourceIndices =
            operation.coveredSourceHoleIndices;
        sourceIndices.push_back(operation.sourceHoleIndex);
        std::sort(sourceIndices.begin(), sourceIndices.end());
        sourceIndices.erase(
            std::unique(sourceIndices.begin(), sourceIndices.end()),
            sourceIndices.end());
        for (const int sourceIndex : sourceIndices) {
            if (sourceIndex < 0)
                continue;
            Accumulator& item = accumulated[sourceIndex];
            if (item.scope.featureId.empty()) {
                item.scope.featureId =
                    "hole:" + std::to_string(sourceIndex);
                item.scope.operationId = operation.operationId;
                item.scope.sourceIndex = sourceIndex;
                item.scope.approximateBoundary =
                    sourceIndex != operation.sourceHoleIndex;
            }
            if (operation.kind == FeatureKind::CountersinkHole ||
                item.scope.featureKind.empty()) {
                item.scope.featureKind =
                    operation.kind == FeatureKind::ThroughHole
                    ? "through_hole"
                    : operation.kind == FeatureKind::CountersinkHole
                        ? "countersink_hole"
                        : "blind_hole";
            }
            if (item.scope.region.IsNull()) {
                item.scope.region = region;
                continue;
            }
            try {
                BRepAlgoAPI_Fuse fuse(item.scope.region, region);
                fuse.SetFuzzyValue(Precision::Confusion());
                fuse.Build();
                if (fuse.IsDone() && !fuse.Shape().IsNull()) {
                    item.scope.region = fuse.Shape();
                } else {
                    item.scope.approximateBoundary = true;
                }
            } catch (...) {
                item.scope.approximateBoundary = true;
            }
        }
    }
    std::vector<toolpath::StockFeatureAllowanceScope> scopes;
    scopes.reserve(accumulated.size());
    for (auto& [sourceIndex, item] : accumulated) {
        static_cast<void>(sourceIndex);
        if (!item.scope.region.IsNull())
            scopes.push_back(std::move(item.scope));
    }
    return scopes;
}

// 后台任务的执行阶段，用于驱动进度标签文案。
enum class RunStage {
    LoadingStock,
    Simulating,
    Inspecting,
    Attributing,
};

QString stageLabel(RunStage stage)
{
    switch (stage) {
        case RunStage::LoadingStock:
            return QCoreApplication::translate("OpenAutoPath", "正在后台读取真实毛坯 STEP…");
        case RunStage::Simulating:
            return QCoreApplication::translate("OpenAutoPath", "正在后台执行孔加工顺序减料…");
        case RunStage::Inspecting:
            return QCoreApplication::translate("OpenAutoPath", "正在计算毛坯缺料、剩余余量和过切区域…");
        case RunStage::Attributing:
            return QCoreApplication::translate("OpenAutoPath", "正在把过切增量归因到具体孔工序…");
    }
    return QCoreApplication::translate("OpenAutoPath", "正在执行孔毛胚仿真…");
}

} // namespace

HoleStockSimulationRunResult simulateHoleStockWithProgress(
    const WorkflowProgressPresenter& progress,
    HoleStockSimulationRequest request,
    std::shared_ptr<std::atomic_bool> cancellation)
{
    // 预计算总步数：加载 + 逐工序减料 + 毛坯比较 + 过切归因。
    const bool importsStep = !request.stockStepPath.empty();
    const std::size_t loadSteps = importsStep ? 1u : 0u;
    const std::size_t inspectionSteps = request.targetShape.IsNull()
        ? 0u
        : toolpath::StockComparison::ProgressStepCount;
    const std::size_t attributionSteps = request.targetShape.IsNull()
        ? 0u
        : request.operations.size();
    const std::size_t totalSteps =
        loadSteps + request.operations.size() + inspectionSteps +
        attributionSteps;
    std::atomic_size_t completed{0};
    std::atomic_size_t total{std::max<std::size_t>(1u, totalSteps)};
    std::atomic<RunStage> stage{
        importsStep ? RunStage::LoadingStock : RunStage::Simulating};
    if (!cancellation)
        cancellation = std::make_shared<std::atomic_bool>(false);

    // 后台线程执行仿真；通过原子变量与交互端口通信进度与取消，
    // 线程内屏蔽 OCCT 信号并捕获异常转换为错误消息。
    auto future = toolpath::submitHeavyJob(
        cancellation,
        [request = std::move(request), cancellation, importsStep, loadSteps,
         inspectionSteps,
         attributionSteps,
         &completed, &total, &stage]() mutable {
          try {
            OCC_CATCH_SIGNALS
            OSD::SetThreadLocalSignal(OSD_SignalMode_Set, Standard_False);
            HoleStockSimulationRunResult runResult;
            runResult.importedStep = importsStep;
            toolpath::ToolpathGenerationControl loadControl;
            toolpath::attachGlobalToolpathParallelBudget(loadControl);
            loadControl.isCancellationRequested = [cancellation] {
                return cancellation->load(std::memory_order_relaxed);
            };

            // 可选：后台加载真实毛坯 STEP；取消时直接返回。
            TopoDS_Shape initialStock = request.initialStock;
            if (importsStep) {
                const toolpath::StockModelLoadResult loaded =
                    toolpath::StockModelLoader().loadStep(
                        request.stockStepPath, &loadControl);
                if (loaded.status == toolpath::StockModelLoadStatus::Cancelled) {
                    runResult.cancelled = true;
                    return runResult;
                }
                if (!loaded.ok()) {
                    runResult.errorMessage = loaded.message;
                    return runResult;
                }
                initialStock = loaded.shape;
                completed.store(1u, std::memory_order_relaxed);
                stage.store(RunStage::Simulating, std::memory_order_relaxed);
            }

            if (cancellation->load(std::memory_order_relaxed)) {
                runResult.cancelled = true;
                return runResult;
            }
            toolpath::ToolpathGenerationControl simulationControl;
            toolpath::attachGlobalToolpathParallelBudget(
                simulationControl);
            simulationControl.isCancellationRequested = [cancellation] {
                return cancellation->load(std::memory_order_relaxed);
            };
            simulationControl.reportProgress =
                [&completed, &total, loadSteps, inspectionSteps,
                 attributionSteps](
                    std::size_t done, std::size_t operationCount) {
                    completed.store(loadSteps + done,
                                    std::memory_order_relaxed);
                    total.store(std::max<std::size_t>(
                                    1u, loadSteps + operationCount +
                                            inspectionSteps +
                                            attributionSteps),
                                std::memory_order_relaxed);
                };
            // 顺序减料仿真：逐孔从初始毛坯减去加工体。
            runResult.simulation = toolpath::HoleStockSimulator().simulate(
                initialStock, request.operations, request.fallbackParams,
                request.options, &simulationControl);
            runResult.cancelled = runResult.simulation.cancelled;
            if (runResult.cancelled || request.targetShape.IsNull())
                return runResult;

            // 毛坯比较阶段：计算缺料、剩余余量与过切区域。
            stage.store(RunStage::Inspecting, std::memory_order_relaxed);
            const std::size_t comparisonBase =
                loadSteps + request.operations.size();
            toolpath::ToolpathGenerationControl comparisonControl;
            toolpath::attachGlobalToolpathParallelBudget(
                comparisonControl);
            comparisonControl.isCancellationRequested = [cancellation] {
                return cancellation->load(std::memory_order_relaxed);
            };
            comparisonControl.reportProgress =
                [&completed, comparisonBase](std::size_t done, std::size_t) {
                    completed.store(comparisonBase + done,
                                    std::memory_order_relaxed);
                };
            toolpath::StockComparisonOptions comparisonOptions;
            // A completed simulation validated its initial stock and every
            // committed result. Reuse only those conclusions at this boundary;
            // partial/failed runs and the independently loaded target still
            // undergo the comparison's normal topology checks.
            comparisonOptions.initialAndFinalShapesPrevalidated =
                runResult.simulation.complete;
            runResult.comparison = toolpath::StockComparison().compare(
                runResult.simulation.initialStock,
                runResult.simulation.finalStock,
                request.targetShape, comparisonOptions, &comparisonControl);
            runResult.cancelled = runResult.comparison.cancelled;
            if (runResult.cancelled || !runResult.comparison.complete)
                return runResult;
            toolpath::analyzeFeatureAllowances(
                runResult.simulation.initialStock,
                request.targetShape,
                holeFeatureAllowanceScopes(request.operations),
                runResult.comparison);

            // 过切体积可忽略时跳过归因，直接标记完成。
            const double attributionTolerance = std::max(
                1.0e-8, 1.0e-9 * runResult.comparison.targetVolume);
            const std::size_t attributionBase =
                comparisonBase + toolpath::StockComparison::ProgressStepCount;
            if (runResult.comparison.overcutVolume <= attributionTolerance) {
                runResult.overcutAttribution.complete = true;
                completed.store(attributionBase, std::memory_order_relaxed);
                total.store(std::max<std::size_t>(1u, attributionBase),
                            std::memory_order_relaxed);
                return runResult;
            }

            // 过切归因阶段：把过切增量分解到具体孔工序。
            stage.store(RunStage::Attributing, std::memory_order_relaxed);
            toolpath::ToolpathGenerationControl attributionControl;
            toolpath::attachGlobalToolpathParallelBudget(
                attributionControl);
            attributionControl.isCancellationRequested = [cancellation] {
                return cancellation->load(std::memory_order_relaxed);
            };
            attributionControl.reportProgress =
                [&completed, attributionBase](std::size_t done, std::size_t) {
                    completed.store(attributionBase + done,
                                    std::memory_order_relaxed);
                };
            runResult.overcutAttribution =
                toolpath::StockComparison().attributeOvercut(
                    runResult.simulation.initialStock,
                    request.targetShape,
                    runResult.simulation.steps, {}, &attributionControl);
            runResult.cancelled = runResult.overcutAttribution.cancelled;
            return runResult;
          } catch (const Standard_Failure& failure) {
            HoleStockSimulationRunResult runResult;
            const char* message = failure.GetMessageString();
            runResult.errorMessage = std::string("OCCT stock simulation failure: ") +
                                     (message ? message : "OCCT failure");
            return runResult;
          } catch (const std::exception& failure) {
            HoleStockSimulationRunResult runResult;
            runResult.errorMessage = std::string("Stock simulation failure: ") +
                                     failure.what();
            return runResult;
          } catch (...) {
            HoleStockSimulationRunResult runResult;
            runResult.errorMessage = "Unknown stock simulation failure";
            return runResult;
          }
        }, [](toolpath::HeavyJobRejection reason) {
            HoleStockSimulationRunResult result;
            result.cancelled = reason == toolpath::HeavyJobRejection::Cancelled;
            if (!result.cancelled)
                result.errorMessage = toolpath::heavyJobRejectionMessage(reason);
            return result;
        });
    toolpath::HeavyJobWaitGuard waitGuard(future, cancellation);

    // 主线程通过交互端口轮询后台任务；取消请求写入共享原子标志。
    WorkflowProgressRequest progressRequest;
    progressRequest.title = QCoreApplication::translate("OpenAutoPath", "孔毛胚仿真");
    progressRequest.label =
        stageLabel(stage.load(std::memory_order_relaxed));
    progressRequest.cancellingLabel =
        QCoreApplication::translate("OpenAutoPath", "正在取消，等待当前 STEP/布尔操作结束…");
    progressRequest.initialTotal = total.load(std::memory_order_relaxed);
    progressRequest.snapshot = [&] {
        return WorkflowProgressSnapshot{
            completed.load(std::memory_order_relaxed),
            total.load(std::memory_order_relaxed),
            stageLabel(stage.load(std::memory_order_relaxed))};
    };
    progressRequest.queued = [cancellation] { return toolpath::heavyJobScheduler().isPending(cancellation); };
    progressRequest.ready = [&future] {
        return future.wait_for(std::chrono::seconds(0)) ==
               std::future_status::ready;
    };
    progressRequest.cancel = [cancellation] {
        toolpath::cancelHeavyJob(cancellation);
    };
    progressRequest.cancellationRequested = [cancellation] {
        return cancellation->load(std::memory_order_relaxed);
    };
    waitForWorkflowTask(progress, progressRequest);

    try {
        return future.get();
    } catch (...) {
        HoleStockSimulationRunResult runResult;
        runResult.errorMessage =
            "unknown exception in background stock simulation task";
        return runResult;
    }
}

HoleOpenGlSimulationResult prepareHoleOpenGlWithProgress(
    const WorkflowProgressPresenter& progress,
    HoleOpenGlSimulationRequest request)
{
    const bool buildsFinalStock = !request.finalStock.IsNull();
    std::atomic_size_t completed{0u};
    const std::size_t total = buildsFinalStock ? 2u : 1u;
    auto cancellation = std::make_shared<std::atomic_bool>(false);

    auto future = toolpath::submitHeavyJob(
        cancellation,
        [request = std::move(request), cancellation, &completed,
         buildsFinalStock]() mutable {
          HoleOpenGlSimulationResult result;
          try {
            OCC_CATCH_SIGNALS
            OSD::SetThreadLocalSignal(
                OSD_SignalMode_Set, Standard_False);
            if (request.initialStock.IsNull()) {
                result.error =
                    "hole OpenGL simulation requires a non-null initial IPW snapshot";
                return result;
            }
            result.input.operations =
                toolpath::holeSimulationOperations(request.operations);
            if (result.input.operations.empty()) {
                result.error =
                    "hole OpenGL simulation requires at least one operation";
                return result;
            }

            gp_Vec outwardAxis =
                result.input.operations.front().outwardToolAxis;
            if (outwardAxis.Magnitude() <= Precision::Confusion()) {
                result.error =
                    "hole OpenGL simulation could not determine the setup tool axis";
                return result;
            }
            outwardAxis.Normalize();
            for (const toolpath::PocketToolpathOperation& operation :
                 result.input.operations) {
                gp_Vec operationAxis = operation.outwardToolAxis;
                if (operation.clearing.moves.empty()) {
                    result.error =
                        "hole OpenGL simulation contains an operation without moves";
                    return result;
                }
                if (operationAxis.Magnitude() <= Precision::Confusion()) {
                    result.error =
                        "hole OpenGL simulation contains an operation without a valid tool axis";
                    return result;
                }
                operationAxis.Normalize();
                if (operationAxis.Dot(outwardAxis) < 1.0 - 1.0e-6) {
                    result.error =
                        "hole OpenGL simulation requires one fixed tool axis per setup";
                    return result;
                }
            }
            double referenceDiameter =
                std::numeric_limits<double>::infinity();
            for (const toolpath::HoleMachiningOperation& operation :
                 request.operations) {
                const double diameter =
                    operation.tool.valid &&
                            std::isfinite(operation.tool.diameter) &&
                            operation.tool.diameter > 0.0
                    ? operation.tool.diameter
                    : std::isfinite(operation.radius) &&
                              operation.radius > 0.0
                        ? 2.0 * operation.radius
                        : request.fallbackParams.tool.diameter;
                if (std::isfinite(diameter) && diameter > 0.0)
                    referenceDiameter =
                        std::min(referenceDiameter, diameter);
            }
            if (!std::isfinite(referenceDiameter) ||
                referenceDiameter <= 0.0) {
                result.error =
                    "hole OpenGL simulation requires a positive cutter diameter";
                return result;
            }

            const IpwSimulationQualityProfile quality =
                ipwSimulationQualityProfile(
                    request.displayQuality, referenceDiameter);
            toolpath::StockSimulationOptions options;
            options.method =
                toolpath::StockSimulationMethod::HeightField;
            options.runParallel = true;
            options.zMapResolution = quality.requestedResolution;
            options.zMapMaxCellCount = quality.maximumCellCount;
            const auto cancelled = [cancellation] {
                return cancellation->load(std::memory_order_relaxed);
            };

            toolpath::MultiDexelBuildMetrics initialMetrics;
            std::string buildError;
            result.input.initialIpwStock =
                toolpath::MultiDexelStock::fromShape(
                    request.initialStock, outwardAxis,
                    referenceDiameter, options, &initialMetrics,
                    &buildError, cancelled);
            if (cancelled()) {
                result.cancelled = true;
                return result;
            }
            if (!result.input.initialIpwStock ||
                !result.input.initialIpwStock->valid()) {
                result.error = buildError.empty()
                    ? "failed to rasterize the hole OpenGL initial IPW snapshot"
                    : std::move(buildError);
                return result;
            }
            result.gridResolution =
                result.input.initialIpwStock->resolution();
            completed.store(1u, std::memory_order_relaxed);

            if (buildsFinalStock) {
                buildError.clear();
                toolpath::MultiDexelBuildMetrics finalMetrics;
                result.input.finalIpwStock =
                    toolpath::MultiDexelStock::fromShapeOnGrid(
                        request.finalStock,
                        *result.input.initialIpwStock,
                        options, &finalMetrics, &buildError,
                        cancelled);
                if (cancelled()) {
                    result.cancelled = true;
                    return result;
                }
                if (!result.input.finalIpwStock ||
                    !result.input.finalIpwStock->valid()) {
                    result.input.finalIpwStock.reset();
                    result.warning = buildError.empty()
                        ? "the final hole IPW snapshot could not be rasterized; playback will retain the GPU-computed completion frame"
                        : std::move(buildError);
                }
                completed.store(2u, std::memory_order_relaxed);
            }

            result.input.stock = request.initialStock;
            result.input.target = request.targetShape;
            result.input.params = request.fallbackParams;
            result.input.options.quality =
                std::clamp(request.displayQuality, 1, 10);
            result.input.options.pathSimplificationTolerance = 0.0;
            result.input.options.allowAutomaticPathSimplification = false;
            result.input.options.maxDisplayMotionCount = 2'000'000u;
            return result;
          } catch (const Standard_Failure& failure) {
            const char* message = failure.GetMessageString();
            result.error =
                std::string("OCCT hole OpenGL preparation failure: ") +
                (message ? message : "OCCT failure");
            return result;
          } catch (const std::exception& failure) {
            result.error =
                std::string("hole OpenGL preparation failure: ") +
                failure.what();
            return result;
          } catch (...) {
            result.error =
                "unknown exception in hole OpenGL preparation";
            return result;
          }
        }, [](toolpath::HeavyJobRejection reason) {
            HoleOpenGlSimulationResult result;
            result.cancelled = reason == toolpath::HeavyJobRejection::Cancelled;
            if (!result.cancelled)
                result.error = toolpath::heavyJobRejectionMessage(reason);
            return result;
        });
    toolpath::HeavyJobWaitGuard waitGuard(future, cancellation);

    WorkflowProgressRequest progressRequest;
    progressRequest.title = QCoreApplication::translate("OpenAutoPath", "孔 OpenGL 仿真");
    progressRequest.label =
        QCoreApplication::translate("OpenAutoPath", "正在后台构建孔加工 GPU Multi-Dexel 材料状态…");
    progressRequest.cancellingLabel =
        QCoreApplication::translate("OpenAutoPath", "正在取消孔 OpenGL 材料准备…");
    progressRequest.initialTotal = total;
    progressRequest.snapshot = [&] {
        const std::size_t current =
            completed.load(std::memory_order_relaxed);
        return WorkflowProgressSnapshot{
            current, total,
            current == 0u
                ? QCoreApplication::translate("OpenAutoPath", "正在栅格化孔加工前 IPW…")
                : buildsFinalStock && current < total
                    ? QCoreApplication::translate("OpenAutoPath", "正在栅格化孔加工后 IPW…")
                    : QCoreApplication::translate("OpenAutoPath", "孔 OpenGL 材料状态已准备完成")};
    };
    progressRequest.queued = [cancellation] { return toolpath::heavyJobScheduler().isPending(cancellation); };
    progressRequest.ready = [&future] {
        return future.wait_for(std::chrono::seconds(0)) ==
               std::future_status::ready;
    };
    progressRequest.cancel = [cancellation] {
        toolpath::cancelHeavyJob(cancellation);
    };
    progressRequest.cancellationRequested = [cancellation] {
        return cancellation->load(std::memory_order_relaxed);
    };
    waitForWorkflowTask(progress, progressRequest);

    try {
        return future.get();
    } catch (...) {
        HoleOpenGlSimulationResult result;
        result.error =
            "unknown exception in background hole OpenGL preparation task";
        return result;
    }
}

} // namespace oap::app

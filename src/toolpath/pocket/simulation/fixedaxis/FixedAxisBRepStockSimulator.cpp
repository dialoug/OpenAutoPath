// FixedAxisBRepStockSimulator::simulate 实现（oap_toolpath_pocket 的
// fixedaxis 私有目录）。按工序把刀路增量构建为去除 BRep，再以独立实体布尔
// 减应用到 InProcessStock，并汇报每工序进度与统计指标。参见
// FixedAxisBRepStockSimulator.h。
#include "toolpath/pocket/simulation/fixedaxis/FixedAxisBRepStockSimulator.h"
#include "toolpath/core/PocketToolBinding.h"
#include "toolpath/core/ToolpathParallel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <utility>

#include <Precision.hxx>

#include "toolpath/core/simulation/InProcessStock.h"
#include "toolpath/pocket/simulation/fixedaxis/FixedAxisBRepPreflight.h"
#include "toolpath/pocket/simulation/fixedaxis/FixedAxisLayeredBRepBuilder.h"

namespace oap::toolpath {
namespace {

using Clock = std::chrono::steady_clock;

double elapsedMilliseconds(const Clock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(
               Clock::now() - start)
        .count();
}

void addDiagnostic(
    StockSimulationResult& result,
    ToolpathDiagnosticCode code,
    ToolpathDiagnosticSeverity severity,
    std::string message,
    const std::string& operationId = {})
{
    ToolpathDiagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.severity = severity;
    diagnostic.message = std::move(message);
    diagnostic.operationId = operationId;
    result.diagnostics.push_back(
        std::move(diagnostic));
}

StockRemovalRecord emptyRecord(
    const InProcessStock& stock,
    const PocketToolpathOperation& operation,
    int sequenceIndex,
    StockRemovalStatus status,
    std::string message)
{
    StockRemovalRecord record;
    record.operationId = operation.operationId;
    record.sequenceIndex = sequenceIndex;
    record.status = status;
    record.volumeBefore = stock.currentVolume();
    record.volumeAfter = stock.currentVolume();
    record.stockAfter = stock.currentShape();
    record.message = std::move(message);
    return record;
}

bool validAxis(
    const gp_Vec& axis,
    double tolerance,
    gp_Vec& normalized)
{
    if (!std::isfinite(axis.X()) ||
        !std::isfinite(axis.Y()) ||
        !std::isfinite(axis.Z()) ||
        axis.Magnitude() <= tolerance) {
        return false;
    }
    normalized = axis;
    normalized.Normalize();
    return true;
}

void reportExecutionStatus(
    const ToolpathGenerationControl* control,
    ToolpathExecutionPhase phase,
    const PocketToolpathOperation* operation = nullptr,
    std::size_t operationIndex = 0,
    std::size_t operationCount = 0,
    std::size_t workItemCount = 0)
{
    if (!control)
        return;
    ToolpathExecutionStatus status;
    status.phase = phase;
    if (operation)
        status.operationId = operation->operationId;
    status.operationIndex = operationIndex;
    status.operationCount = operationCount;
    status.workItemCount = workItemCount;
    control->status(std::move(status));
}

} // namespace

StockSimulationResult
FixedAxisBRepStockSimulator::simulate(
    const TopoDS_Shape& initialStock,
    const std::vector<PocketToolpathOperation>& operations,
    const ToolpathParams& params,
    const StockSimulationOptions& options,
    const ToolpathGenerationControl* control) const
{
    // 主流程：① 初始化 InProcessStock 并校验毛坯 → ② preflight 推导
    // 有效刀具参数 → ③ 逐工序 FixedAxisLayeredBRepBuilder 构造去除体积
    // → ④ stock.applyRemovalWithIndependentSolids 做独立实体布尔减 →
    // ⑤ 汇总统计/诊断与最终毛坯。过程中通过 control 上报进度与阶段。
    ToolpathParallelScope parallelScope(control);
    StockSimulationResult result;
    const Clock::time_point totalStart = Clock::now();
    result.initialStock = initialStock;
    result.heightFieldSimulation = false;
    result.parallelProcessingEnabled =
        options.runParallel;
    result.parallelWorkerCount =
        options.runParallel
        ? static_cast<int>(toolpathHardwareParallelism())
        : 1;

    std::function<bool()> cancellationRequested;
    const std::size_t progressSteps =
        progressStepCount(operations.size());
    if (control) {
        cancellationRequested = [control] {
            return control->cancelled();
        };
        reportExecutionStatus(
            control,
            ToolpathExecutionPhase::PreparingStock,
            nullptr, 0, operations.size());
        control->progress(0u, progressSteps);
    }

    InProcessStock stock(
        initialStock,
        options.geometricTolerance,
        options.runParallel,
        cancellationRequested);
    result.initialVolume = stock.initialVolume();
    result.finalVolume = stock.currentVolume();
    result.finalStock = stock.currentShape();
    if (!stock.valid()) {
        addDiagnostic(
            result,
            ToolpathDiagnosticCode::InvalidInitialStock,
            ToolpathDiagnosticSeverity::Error,
            "固定轴 BRep 仿真要求初始毛坯为有效的正体积实体");
        result.totalMilliseconds =
            elapsedMilliseconds(totalStart);
        return result;
    }

    const FixedAxisBRepPreflightResult preflight =
        FixedAxisBRepPreflight().inspect(
            initialStock, operations,
            params, options);
    result.diagnostics.insert(
        result.diagnostics.end(),
        preflight.diagnostics.begin(),
        preflight.diagnostics.end());
    result.inferredCuttingEdgeLength =
        preflight.inferredCuttingEdgeLength;
    result.effectiveCuttingEdgeLength =
        preflight.effectiveCuttingEdgeLength;
    if (control) {
        control->progress(
            PreparationProgressStepCount,
            progressSteps);
    }

    const double tolerance = std::max(
        options.geometricTolerance,
        Precision::Confusion());
    gp_Vec referenceAxis;
    bool hasReferenceAxis = false;
    bool fatalFailure = false;
    for (std::size_t index = 0;
         index < operations.size(); ++index) {
        const std::size_t operationProgressBase =
            PreparationProgressStepCount +
            ProgressStepsPerOperation * index;
        const auto completeOperationProgress = [&] {
            if (control) {
                control->progress(
                    operationProgressBase +
                        ProgressStepsPerOperation,
                    progressSteps);
            }
        };
        if (cancellationRequested &&
            cancellationRequested()) {
            result.cancelled = true;
            break;
        }
        const PocketToolpathOperation& operation =
            operations[index];
        gp_Vec operationAxis;
        if (!validAxis(
                operation.outwardToolAxis,
                tolerance,
                operationAxis)) {
            StockRemovalRecord record = emptyRecord(
                stock, operation,
                static_cast<int>(index),
                StockRemovalStatus::InvalidRemovalVolume,
                "固定轴 BRep 工序的刀轴无效");
            result.steps.push_back(record);
            addDiagnostic(
                result,
                ToolpathDiagnosticCode::InvalidSetupAxis,
                ToolpathDiagnosticSeverity::Error,
                record.message,
                operation.operationId);
            ++result.processedOperationCount;
            fatalFailure = true;
            completeOperationProgress();
            if (options.stopOnFailure)
                break;
            continue;
        }
        if (!hasReferenceAxis) {
            referenceAxis = operationAxis;
            hasReferenceAxis = true;
        } else if (referenceAxis.Dot(operationAxis) <
                   1.0 - 1.0e-8) {
            StockRemovalRecord record = emptyRecord(
                stock, operation,
                static_cast<int>(index),
                StockRemovalStatus::InvalidRemovalVolume,
                "一次固定轴 BRep 仿真中的所有工序必须使用同一刀轴");
            result.steps.push_back(record);
            addDiagnostic(
                result,
                ToolpathDiagnosticCode::SetupAxisMismatch,
                ToolpathDiagnosticSeverity::Error,
                record.message,
                operation.operationId);
            ++result.processedOperationCount;
            fatalFailure = true;
            completeOperationProgress();
            if (options.stopOnFailure)
                break;
            continue;
        }

        reportExecutionStatus(
            control,
            ToolpathExecutionPhase::BuildingRemovalVolume,
            &operation, index, operations.size());
        FixedAxisLayeredBRepBuildResult build =
            FixedAxisLayeredBRepBuilder().build(
                stock.currentShape(),
                operation,
                bindPocketToolParameters(
                    preflight.effectiveParams,
                    operation.tool),
                options,
                cancellationRequested);
        result.sweepBuildMilliseconds +=
            build.buildMilliseconds;
        result.sweepInflationMilliseconds +=
            build.sweepInflationMilliseconds;
        result.analyticPlanarFallbackOperationCount +=
            build.analyticPlanarFallbackOperationCount;
        result.analyticPlanarPrimitiveCount +=
            build.analyticPlanarPrimitiveCount;
        result.analyticPlanarArcCount +=
            build.analyticPlanarArcCount;
        result.analyticPlanarBooleanGroupCount +=
            build.analyticPlanarBooleanGroupCount;
        result.analyticPlanarMilliseconds +=
            build.analyticPlanarMilliseconds;
        result.maximumCurveFitError =
            std::max(
                result.maximumCurveFitError,
                build.maximumCurveFitError);
        result.maximumChordError =
            std::max(
                result.maximumChordError,
                build.maximumChordError);
        result.sweepSegmentCount +=
            build.cuttingMoveCount;
        result.planarSweepLayerCount +=
            build.planarLayerCount;
        result.removalSolidCount +=
            build.removalSolidCount;
        result.skippedInvalidRemovalSolidCount +=
            build.skippedInvalidRemovalSolidCount;
        result.planarFootprintArea +=
            build.footprintArea;
        result.linearRampMoveCount +=
            build.linearRampMoveCount;
        result.helicalRampMoveCount +=
            build.helicalRampMoveCount;
        result.entrySweepGroupCount +=
            build.rampSweepGroupCount;
        result.containedEntrySweepGroupCount +=
            build.containedRampSweepGroupCount;
        result.containedEntrySweepSegmentCount +=
            build.containedRampMoveCount;
        if (!build.analyticPlanarUsed) {
            result.fallbackSweepSegmentCount +=
                build.linearRampMoveCount +
                build.helicalRampMoveCount -
                build.containedRampMoveCount;
        }
        result.rampSweepSliceCount +=
            build.rampSweepSliceCount;
        result.toolProfileBandCount +=
            build.toolProfileBandCount;
        result.maximumRampAxialError =
            std::max(
                result.maximumRampAxialError,
                build.maximumRampAxialError);
        result.maximumToolProfileAxialError =
            std::max(
                result.maximumToolProfileAxialError,
                build.maximumToolProfileAxialError);
        result.maximumToolProfileRadialError =
            std::max(
                result.maximumToolProfileRadialError,
                build.maximumToolProfileRadialError);
        result.maximumPlanarFootprintArea =
            std::max(
                result.maximumPlanarFootprintArea,
                build.maximumFootprintArea);
        if (build.skippedInvalidRemovalSolidCount > 0) {
            addDiagnostic(
                result,
                ToolpathDiagnosticCode::
                    StockInvalidRemovalSolidSkipped,
                ToolpathDiagnosticSeverity::Warning,
                "固定轴 BRep 跳过了 " +
                    std::to_string(
                        build.skippedInvalidRemovalSolidCount) +
                    " 个无法修复的退化减料实体；结果只增加欠切",
                operation.operationId);
        }

        StockRemovalRecord record;
        if (build.cancelled) {
            record = emptyRecord(
                stock, operation,
                static_cast<int>(index),
                StockRemovalStatus::Cancelled,
                "固定轴 BRep 减料体构造已取消");
            result.cancelled = true;
        } else if (build.noMaterial) {
            record = emptyRecord(
                stock, operation,
                static_cast<int>(index),
                StockRemovalStatus::NoMaterialRemoved,
                "受支持的切削移动未与毛坯的轴向范围相交");
            ++result.noMaterialOperationCount;
            addDiagnostic(
                result,
                ToolpathDiagnosticCode::StockNoMaterialRemoved,
                ToolpathDiagnosticSeverity::Info,
                record.message,
                operation.operationId);
        } else if (!build.ok()) {
            record = emptyRecord(
                stock, operation,
                static_cast<int>(index),
                StockRemovalStatus::InvalidRemovalVolume,
                build.error.empty()
                    ? "固定轴 BRep 减料体构造失败"
                    : build.error);
            addDiagnostic(
                result,
                ToolpathDiagnosticCode::StockSweepConstructionFailed,
                ToolpathDiagnosticSeverity::Error,
                record.message,
                operation.operationId);
            fatalFailure = true;
        } else {
            const auto removalPhaseChanged =
                [control, &operation, index,
                 operationCount = operations.size(),
                 operationProgressBase, progressSteps,
                 removalSolidCount =
                     static_cast<std::size_t>(
                         build.removalSolidCount)](
                    StockRemovalPhase phase) {
                    if (!control)
                        return;
                    ToolpathExecutionPhase executionPhase =
                        ToolpathExecutionPhase::None;
                    std::size_t completed =
                        operationProgressBase;
                    switch (phase) {
                        case StockRemovalPhase::
                                ValidatingRemovalVolume:
                            executionPhase =
                                ToolpathExecutionPhase::
                                    ValidatingRemovalVolume;
                            completed += 1u;
                            break;
                        case StockRemovalPhase::
                                ApplyingBooleanRemoval:
                            executionPhase =
                                ToolpathExecutionPhase::
                                    ApplyingBooleanRemoval;
                            completed += 2u;
                            break;
                        case StockRemovalPhase::
                                ValidatingRemovalResult:
                            executionPhase =
                                ToolpathExecutionPhase::
                                    ValidatingRemovalResult;
                            completed += 3u;
                            break;
                    }
                    control->progress(
                        completed, progressSteps);
                    reportExecutionStatus(
                        control, executionPhase,
                        &operation, index,
                        operationCount,
                        removalSolidCount);
                };
            record =
                stock.applyRemovalWithIndependentSolids(
                    build.removalVolume,
                    operation.operationId,
                    static_cast<int>(index),
                    removalPhaseChanged);
            record.sweepSegmentCount =
                build.cuttingMoveCount;
            record.removalSolidCount =
                build.removalSolidCount;
            record.planarSweepLayerCount =
                build.planarLayerCount;
            record.planarFootprintArea =
                build.footprintArea;
            record.maximumPlanarFootprintArea =
                build.maximumFootprintArea;
            record.entrySweepGroupCount =
                build.rampSweepGroupCount;
            record.containedEntrySweepGroupCount =
                build.containedRampSweepGroupCount;
            record.containedEntrySweepSegmentCount =
                build.containedRampMoveCount;
            record.fallbackSweepSegmentCount =
                build.linearRampMoveCount +
                build.helicalRampMoveCount -
                build.containedRampMoveCount;
            result.inputValidationMilliseconds +=
                record.inputValidationMilliseconds;
            result.booleanMilliseconds +=
                record.booleanMilliseconds;
            result.resultValidationMilliseconds +=
                record.resultValidationMilliseconds;
            result.booleanBatchCount +=
                record.booleanBatchCount;
            if (record.status ==
                StockRemovalStatus::Applied) {
                ++result.appliedOperationCount;
            } else if (record.status ==
                       StockRemovalStatus::NoMaterialRemoved) {
                ++result.noMaterialOperationCount;
                addDiagnostic(
                    result,
                    ToolpathDiagnosticCode::StockNoMaterialRemoved,
                    ToolpathDiagnosticSeverity::Info,
                    record.message,
                    operation.operationId);
            } else if (record.status ==
                       StockRemovalStatus::Cancelled) {
                result.cancelled = true;
            } else {
                fatalFailure = true;
                addDiagnostic(
                    result,
                    record.status ==
                            StockRemovalStatus::BooleanFailed
                        ? ToolpathDiagnosticCode::StockBooleanFailed
                        : ToolpathDiagnosticCode::StockSweepConstructionFailed,
                    ToolpathDiagnosticSeverity::Error,
                    record.message,
                    operation.operationId);
            }
        }

        result.steps.push_back(std::move(record));
        ++result.processedOperationCount;
        result.finalStock = stock.currentShape();
        result.finalVolume = stock.currentVolume();
        result.removedVolume = std::max(
            0.0,
            result.initialVolume -
                result.finalVolume);
        completeOperationProgress();
        if (result.cancelled ||
            (fatalFailure && options.stopOnFailure)) {
            break;
        }
    }

    result.finalStock = stock.currentShape();
    result.finalVolume = stock.currentVolume();
    result.removedVolume = std::max(
        0.0,
        result.initialVolume -
            result.finalVolume);
    result.complete =
        !result.cancelled &&
        !fatalFailure &&
        result.processedOperationCount ==
            static_cast<int>(operations.size());
    result.totalMilliseconds =
        elapsedMilliseconds(totalStart);
    if (control)
        control->status({});
    return result;
}

} // namespace oap::toolpath

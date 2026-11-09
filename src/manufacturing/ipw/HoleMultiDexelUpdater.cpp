// HoleMultiDexelUpdater: expose hole moves through a neutral fixed-axis view,
// 在同一 Multi-Dexel 毛坯上顺序应用(轴对齐走快速路径,异轴走定向扫掠),
// 逐工序累计仿真统计并支持取消。属于 oap_manufacturing::ipw 模块。
#include "manufacturing/ipw/HoleMultiDexelUpdater.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include "toolpath/core/ToolpathParallel.h"
#include <utility>

namespace oap::manufacturing::ipw {
namespace {

using Clock = std::chrono::steady_clock;

double elapsedMilliseconds(Clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(
               Clock::now() - start)
        .count();
}

// 取正值有限数,否则回退到 fallback(刀具直径/切削长度的容错)。
double positive(double preferred, double fallback)
{
    return std::isfinite(preferred) && preferred > 0.0
        ? preferred : fallback;
}

bool validTipAngle(double value)
{
    return std::isfinite(value) && value > 0.0 && value < 180.0;
}

toolpath::DexelToolProfileDefinition toolProfile(
    const toolpath::HoleMachiningOperation& operation,
    const toolpath::ToolpathParams& params)
{
    toolpath::DexelToolProfileDefinition profile;
    profile.diameter = positive(
        operation.tool.diameter, params.tool.diameter);
    profile.cuttingLength = positive(
        operation.tool.cuttingEdgeLength,
        params.tool.cuttingEdgeLength);
    const bool drilling =
        operation.strategy ==
            toolpath::HoleMachiningStrategy::SmallDrill ||
        operation.phase == toolpath::HoleMachiningPhase::CenterSpot ||
        operation.phase == toolpath::HoleMachiningPhase::Drill;
    const double operationTipAngle = operation.tool.tipAngle;
    const double parameterTipAngle =
        params.hole.drillTipAngleDeg;
    const double tipAngle = validTipAngle(operationTipAngle)
        ? operationTipAngle
        : (validTipAngle(parameterTipAngle)
               ? parameterTipAngle : 0.0);
    if (drilling && validTipAngle(tipAngle)) {
        profile.family =
            toolpath::DexelToolProfileFamily::DrillPoint;
        profile.tipAngleDeg = tipAngle;
        profile.cornerRadius = 0.0;
    } else {
        // 钻尖角为零是刻意的“不补偿”设置。HoleStockSimulator 已把该情形建模为
        // 平底圆柱铣刀；这里应使用同样的有效几何，而不是要求严格的 Dexel
        // 轮廓构建器生成一个无效的锥体。
        profile.family =
            toolpath::DexelToolProfileFamily::RoundedEnd;
        profile.cornerRadius = drilling
            ? 0.0
            : (std::isfinite(operation.tool.cornerRadius)
            ? std::max(0.0, operation.tool.cornerRadius)
            : std::max(0.0, params.tool.cornerRadius));
    }
    return profile;
}

// Borrow the authoritative moves; the synchronous material update neither copies
// a Pocket operation nor retains the view after returning.
toolpath::FixedAxisToolpathView axialOperation(
    const toolpath::HoleMachiningOperation& operation)
{
    return {operation.operationId, gp_Vec(operation.bottom, operation.entry),
            operation.moves};
}
} // namespace

toolpath::DexelToolProfileDefinition holeDexelToolProfile(
    const toolpath::HoleMachiningOperation& operation,
    const toolpath::ToolpathParams& params)
{
    return toolProfile(operation, params);
}

// 顺序应用实现:仿真元数据取自初始毛坯;每步工序先用本工序刀具参数覆盖
// 全局参数,再按轴对齐选择快速路径或定向扫掠路径;任一步取消/失败立即
// 终止,且不发布部分结果。
HoleMultiDexelUpdateResult applyHoleOperationsToMultiDexel(
    std::shared_ptr<const toolpath::MultiDexelStock> initial,
    const std::vector<toolpath::HoleMachiningOperation>& operations,
    const toolpath::ToolpathParams& params,
    const toolpath::StockSimulationOptions& options,
    const toolpath::ToolpathGenerationControl* control)
{
    toolpath::ToolpathParallelScope parallelScope(control);
    HoleMultiDexelUpdateResult output;
    output.simulation =
        std::make_shared<toolpath::StockSimulationResult>();
    if (!initial || !initial->valid()) {
        output.error = "hole multi-dexel update requires a valid initial stock";
        return output;
    }
    if (operations.empty()) {
        output.error = "hole multi-dexel update requires at least one operation";
        return output;
    }

    const Clock::time_point start = Clock::now();
    output.simulation->initialVolume = initial->volume();
    output.simulation->finalVolume = initial->volume();
    output.simulation->heightFieldSimulation = true;
    output.simulation->zMapResolution = initial->resolution();
    output.simulation->zMapCellCount = initial->nx() * initial->ny();
    output.simulation->zMapActiveCellCount = initial->activeCellCount();
    output.simulation->parallelProcessingEnabled = options.runParallel;
    output.simulation->parallelWorkerCount = options.runParallel
        ? static_cast<int>(toolpath::toolpathHardwareParallelism())
        : 1;

    // 主循环:current 沿检查点链逐个前进,前一步输出即下一步输入。
    std::shared_ptr<const toolpath::MultiDexelStock> current = initial;
    for (std::size_t index = 0; index < operations.size(); ++index) {
        if (control && control->cancelled()) {
            output.cancelled = true;
            output.simulation->cancelled = true;
            break;
        }
        toolpath::ToolpathParams operationParams = params;
        operationParams.tool.diameter = positive(
            operations[index].tool.diameter,
            operationParams.tool.diameter);
        operationParams.tool.cornerRadius = std::isfinite(
                operations[index].tool.cornerRadius)
            ? std::max(0.0, operations[index].tool.cornerRadius)
            : operationParams.tool.cornerRadius;
        operationParams.tool.cuttingEdgeLength = positive(
            operations[index].tool.cuttingEdgeLength,
            operationParams.tool.cuttingEdgeLength);
        const toolpath::FixedAxisToolpathView converted =
            axialOperation(operations[index]);
        const toolpath::DexelToolProfileDefinition profile =
            toolProfile(operations[index], operationParams);
        const std::function<bool()> cancellationRequested =
            control
            ? control->isCancellationRequested
            : std::function<bool()>{};
        // 轴与当前网格对齐时走快速路径,否则走定向(异轴)半平面扫掠路径;
        // 两种路径都直接更新同一 Multi-Dexel 材料真值。
        const toolpath::MultiDexelApplyResult applied =
            current->axisAligned(converted.outwardToolAxis)
            ? current->applyAxialToolpath(
                  converted, operationParams, profile,
                  static_cast<int>(index), options,
                  cancellationRequested)
            : current->applyOrientedToolpath(
                  converted, operationParams, profile,
                  static_cast<int>(index), options,
                  cancellationRequested);

        output.simulation->steps.push_back(applied.record);
        output.simulation->inputValidationMilliseconds +=
            applied.record.inputValidationMilliseconds;
        output.simulation->booleanMilliseconds +=
            applied.record.booleanMilliseconds;
        output.simulation->zMapUpdateMilliseconds +=
            applied.updateMilliseconds;
        output.simulation->booleanBatchCount +=
            applied.record.booleanBatchCount;
        output.simulation->zMapTileCount += applied.touchedTileCount;
        output.simulation->sweepSegmentCount += applied.cuttingMoveCount;
        output.simulation->removalSolidCount +=
            applied.record.removalSolidCount;
        output.simulation->planarSweepLayerCount +=
            applied.planarLayerCount;
        output.simulation->fallbackSweepSegmentCount +=
            applied.nonPlanarMoveCount;
        output.simulation->toolProfileBandCount +=
            applied.toolProfileBandCount;
        output.simulation->maximumToolProfileAxialError = std::max(
            output.simulation->maximumToolProfileAxialError,
            applied.maximumToolProfileAxialError);
        output.simulation->maximumToolProfileRadialError = std::max(
            output.simulation->maximumToolProfileRadialError,
            applied.maximumToolProfileRadialError);

        if (applied.record.status ==
            toolpath::StockRemovalStatus::Cancelled) {
            output.cancelled = true;
            output.simulation->cancelled = true;
            break;
        }
        if (applied.record.status !=
                toolpath::StockRemovalStatus::Applied &&
            applied.record.status !=
                toolpath::StockRemovalStatus::NoMaterialRemoved) {
            output.error = applied.record.message;
            break;
        }
        ++output.simulation->processedOperationCount;
        if (applied.record.status ==
            toolpath::StockRemovalStatus::Applied) {
            ++output.simulation->appliedOperationCount;
        } else {
            ++output.simulation->noMaterialOperationCount;
        }
        current = applied.stock;
    }

    output.simulation->totalMilliseconds = elapsedMilliseconds(start);
    if (output.cancelled || !output.error.empty()) {
        // current 只是局部不可变子状态；失败时不从结果发布它。
        output.stock.reset();
        return output;
    }
    output.stock = std::move(current);
    output.simulation->finalVolume = output.stock->volume();
    output.simulation->removedVolume = std::max(
        0.0, output.simulation->initialVolume -
                 output.simulation->finalVolume);
    output.simulation->zMapActiveCellCount =
        output.stock->activeCellCount();
    output.simulation->complete =
        output.simulation->processedOperationCount ==
            static_cast<int>(operations.size());
    return output;
}

} // namespace oap::manufacturing::ipw

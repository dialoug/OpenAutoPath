// 中心线钻削策略：解析深径比固定循环，并保留等价展开移动供预览和安全验证。
// 属于 oap_toolpath_hole 模块 detail/strategies 私有实现；由 buildSmallDrill
// 对单个孔生成钻削操作，含钻尖深度补偿与 G81/G82/G83/G73 循环选择。
#include "toolpath/hole/detail/HoleStrategyGenerators.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "toolpath/hole/detail/DrillDepthCompensation.h"
#include "toolpath/hole/detail/strategies/HoleStrategySupport.h"

namespace oap::toolpath::detail {
namespace {

constexpr double kTolerance = 1.0e-7;

// 按名义深径比自动选择固定循环：浅孔用 G81（有暂停则 G82），加深到
// G73 断屑、再深到 G83 啄钻；阈值均可由参数覆盖。
ToolpathDrillCycleType automaticCycleType(
    double ratio,
    const HoleMachiningParams& params)
{
    const double g81 = std::isfinite(params.g81MaxDepthDiameterRatio)
        ? std::max(0.0, params.g81MaxDepthDiameterRatio)
        : 2.0;
    const double requestedG73 =
        std::isfinite(params.g73MaxDepthDiameterRatio)
            ? std::max(0.0, params.g73MaxDepthDiameterRatio)
            : 4.0;
    const double g73 = std::max(g81, requestedG73);
    if (ratio <= g81)
        return ToolpathDrillCycleType::Drilling;
    if (ratio <= g73)
        return ToolpathDrillCycleType::ChipBreak;
    return ToolpathDrillCycleType::Peck;
}

double fallbackPeck(
    ToolpathDrillCycleType type,
    const HoleMachiningParams& params)
{
    if (type == ToolpathDrillCycleType::ChipBreak)
        return params.g73PeckDepth;
    if (type == ToolpathDrillCycleType::Peck)
        return params.g83PeckDepth;
    return 0.0;
}

gp_Pnt pointAtCycleDepth(
    const ToolpathDrillCycle& cycle,
    double depth)
{
    return cycle.retract.Translated(cycle.axis.Multiplied(depth));
}

void appendCycleMove(
    std::vector<ToolpathMove>& moves,
    ToolpathMoveType type,
    const gp_Pnt& from,
    const gp_Pnt& to,
    int operationIndex,
    int depthIndex,
    double feed,
    ToolpathMoveIntent intent = ToolpathMoveIntent::General)
{
    if (from.Distance(to) <= kTolerance)
        return;
    strategy::appendMove(
        moves, type, from, to, operationIndex, depthIndex, feed, intent);
}

bool resolveCycle(
    const HoleMachiningInput& hole,
    const HoleFrame& frame,
    const gp_Pnt& commandedBottom,
    const gp_Pnt& retract,
    const gp_Pnt& clearance,
    const ToolpathParams& params,
    ToolpathDrillCycle& cycle)
{
    cycle.clearance = clearance;
    cycle.retract = retract;
    cycle.retractMode = ToolpathDrillRetractMode::InitialPlane;
    cycle.entry = hole.entry;
    cycle.bottom = commandedBottom;
    cycle.axis = frame.axis;
    cycle.nominalDepth = hole.entry.Distance(hole.bottom);
    cycle.dwellSeconds = std::isfinite(params.hole.dwellSeconds)
        ? std::max(0.0, params.hole.dwellSeconds) : 0.0;
    cycle.feed = params.tool.vertFeed;
    if (!std::isfinite(cycle.nominalDepth) || cycle.nominalDepth <= kTolerance ||
        !std::isfinite(params.tool.diameter) ||
        params.tool.diameter <= kTolerance) {
        return false;
    }
    cycle.depthDiameterRatio = cycle.nominalDepth / params.tool.diameter;

    switch (params.hole.drillCycleMode) {
        case HoleDrillCycleMode::Direct:
            cycle.type = cycle.dwellSeconds > 0.0
                ? ToolpathDrillCycleType::Dwell
                : ToolpathDrillCycleType::Drilling;
            cycle.source = ToolpathDrillCycleSource::ExplicitSetting;
            break;
        case HoleDrillCycleMode::Peck:
            cycle.type = ToolpathDrillCycleType::Peck;
            cycle.source = ToolpathDrillCycleSource::ExplicitSetting;
            break;
        case HoleDrillCycleMode::ChipBreak:
            cycle.type = ToolpathDrillCycleType::ChipBreak;
            cycle.source = ToolpathDrillCycleSource::ExplicitSetting;
            break;
        case HoleDrillCycleMode::Auto:
            cycle.type = automaticCycleType(
                cycle.depthDiameterRatio, params.hole);
            if (cycle.type == ToolpathDrillCycleType::Drilling &&
                cycle.dwellSeconds > 0.0) {
                cycle.type = ToolpathDrillCycleType::Dwell;
            }
            cycle.source =
                ToolpathDrillCycleSource::AutomaticDepthDiameterRatio;
            break;
    }

    if (cycle.type == ToolpathDrillCycleType::Peck ||
        cycle.type == ToolpathDrillCycleType::ChipBreak) {
        cycle.peckDepth =
            std::isfinite(params.hole.peckDepth) &&
                    params.hole.peckDepth > kTolerance
                ? params.hole.peckDepth
                : fallbackPeck(cycle.type, params.hole);
        if (!std::isfinite(cycle.peckDepth) ||
            cycle.peckDepth <= kTolerance) {
            return false;
        }
    }
    cycle.chipBreakRetract = params.hole.g73ChipBreakRetract;
    cycle.reentryClearance = params.hole.g83ReentryClearance;
    if ((cycle.type == ToolpathDrillCycleType::ChipBreak &&
         (!std::isfinite(cycle.chipBreakRetract) ||
          cycle.chipBreakRetract <= kTolerance)) ||
        (cycle.type == ToolpathDrillCycleType::Peck &&
         (!std::isfinite(cycle.reentryClearance) ||
          cycle.reentryClearance < 0.0))) {
        return false;
    }
    return true;
}

bool appendExpandedCycle(
    HoleMachiningOperation& operation,
    const ToolpathDrillCycle& cycle,
    int operationIndex,
    const ToolpathParams& params)
{
    const double totalDepth = cycle.retract.Distance(cycle.bottom);
    if (!std::isfinite(totalDepth) || totalDepth <= kTolerance)
        return false;

    // 把固定循环展开为等价直线移动：普通循环是一次 Plunge+Cut；啄钻/断屑
    // 循环按 peckDepth 逐次进给，G73 只回退断屑量，G83 全回退 R 平面后再
    // 以快移回到上一刀深之前的清空量。展开移动供预览与固实扫掠验证使用。
    if (cycle.type == ToolpathDrillCycleType::Drilling ||
        cycle.type == ToolpathDrillCycleType::Dwell) {
        appendCycleMove(
            operation.moves, ToolpathMoveType::Plunge,
            cycle.retract, cycle.entry, operationIndex, 1,
            params.tool.vertFeed);
        appendCycleMove(
            operation.moves, ToolpathMoveType::Cut,
            cycle.entry, cycle.bottom, operationIndex, 1,
            params.tool.vertFeed);
    } else {
        const double requestedPecks = std::ceil(totalDepth / cycle.peckDepth);
        const int maximumPecks = std::max(16, params.hole.maxSamplePoints);
        if (!std::isfinite(requestedPecks) ||
            requestedPecks > static_cast<double>(maximumPecks)) {
            return false;
        }
        double completedDepth = 0.0;
        gp_Pnt current = cycle.retract;
        int peckIndex = 1;
        while (completedDepth + kTolerance < totalDepth) {
            const double nextDepth = std::min(
                totalDepth, completedDepth + cycle.peckDepth);
            const gp_Pnt next = pointAtCycleDepth(cycle, nextDepth);
            appendCycleMove(
                operation.moves, ToolpathMoveType::Cut,
                current, next, operationIndex, peckIndex,
                params.tool.vertFeed);
            completedDepth = nextDepth;
            current = next;
            if (completedDepth + kTolerance >= totalDepth)
                break;

            if (cycle.type == ToolpathDrillCycleType::ChipBreak) {
                const double retractDepth = std::max(
                    0.0, completedDepth - cycle.chipBreakRetract);
                const gp_Pnt retracted =
                    pointAtCycleDepth(cycle, retractDepth);
                appendCycleMove(
                    operation.moves, ToolpathMoveType::Retract,
                    current, retracted, operationIndex, peckIndex, 0.0);
                current = retracted;
            } else {
                appendCycleMove(
                    operation.moves, ToolpathMoveType::Retract,
                    current, cycle.retract, operationIndex, peckIndex, 0.0);
                const double approachDepth = std::max(
                    0.0, completedDepth - cycle.reentryClearance);
                const gp_Pnt approach =
                    pointAtCycleDepth(cycle, approachDepth);
                appendCycleMove(
                    operation.moves, ToolpathMoveType::Rapid,
                    cycle.retract, approach, operationIndex, peckIndex,
                    params.tool.vertRapid,
                    ToolpathMoveIntent::DrillCycleReentry);
                current = approach;
            }
            ++peckIndex;
        }
    }

    appendCycleMove(
        operation.moves, ToolpathMoveType::Retract,
        cycle.bottom, cycle.retract, operationIndex, 1, 0.0);
    appendCycleMove(
        operation.moves, ToolpathMoveType::Retract,
        cycle.retract, cycle.clearance, operationIndex, 1, 0.0);
    return true;
}

} // namespace

HoleMachiningOperation buildSmallDrill(
    const HoleMachiningInput& hole,
    const HoleFrame& frame,
    int operationIndex,
    const ToolpathParams& params,
    const gp_Pnt* previousClearance,
    HoleRapidPlanner* rapidPlanner)
{
    HoleMachiningOperation operation;
    operation.operationId = "DRILL" + std::to_string(operationIndex + 1);
    operation.kind = hole.kind;
    operation.strategy = HoleMachiningStrategy::SmallDrill;
    operation.entry = hole.entry;
    operation.bottom = hole.bottom;
    operation.radius = hole.radius;

    const DrillDepthTarget depthTarget =
        resolveDrillDepthTarget(hole, frame, params);
    operation.commandedBottom = depthTarget.point;
    operation.hasCommandedBottom = true;
    operation.drillTipLength = depthTarget.tipLength;
    operation.breakthroughDepth = depthTarget.breakthrough;

    const gp_Pnt retract =
        strategy::pointAt(frame, -strategy::safeOffset(params), 0.0, 0.0);
    const gp_Pnt clearance = strategy::pointAt(
        frame,
        -strategy::effectiveClearanceOffset(params, rapidPlanner),
        0.0, 0.0);

    if (previousClearance &&
        !strategy::addClearanceLink(
            operation, *previousClearance, clearance, operationIndex,
            params, rapidPlanner)) {
        return {};
    }
    const std::size_t cycleMoveBegin = operation.moves.size();

    ToolpathDrillCycle cycle;
    if (!resolveCycle(
            hole, frame, depthTarget.point, retract, clearance,
            params, cycle)) {
        return {};
    }
    appendCycleMove(
        operation.moves, ToolpathMoveType::Rapid,
        clearance, retract, operationIndex, 0, params.tool.vertRapid);
    if (!appendExpandedCycle(operation, cycle, operationIndex, params))
        return {};

    appendLinearPrimitives(
        operation.primitives, operation.moves, 0, cycleMoveBegin);
    operation.primitives.push_back(makeDrillCyclePrimitive(
        cycle, std::vector<ToolpathMove>(
            operation.moves.begin() +
                static_cast<std::ptrdiff_t>(cycleMoveBegin),
            operation.moves.end())));
    return operation;
}

} // namespace oap::toolpath::detail

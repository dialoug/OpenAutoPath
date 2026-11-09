// 大孔分层螺旋清料策略：每层从中心向外扩展并执行外圆精加工。
// 属于 oap_toolpath_hole 模块 detail/strategies 私有实现；由 buildLargeSpiral
// 对单个孔生成一个加工操作，各层独立完成下刀、螺旋扩展与抬刀。
#include "toolpath/hole/detail/HoleStrategyGenerators.h"

#include <algorithm>
#include <cmath>

#include "toolpath/hole/detail/strategies/HoleStrategySupport.h"

namespace oap::toolpath::detail {

HoleMachiningOperation buildLargeSpiral(
    const HoleMachiningInput& hole,
    const HoleFrame& frame,
    double cutterRadius,
    int operationIndex,
    const ToolpathParams& params,
    HoleMachiningResult& result,
    const gp_Pnt* previousClearance,
    HoleRapidPlanner* rapidPlanner)
{
    const double allowance = std::isfinite(params.hole.allowance)
        ? std::max(0.0, params.hole.allowance)
        : 0.0;
    const double maximumRadius =
        hole.radius - cutterRadius - allowance;
    const double layerDepth =
        strategy::effectiveLayerDepth(frame, params);
    const double radialStep = strategy::effectiveRadialStep(params);
    // 每转过 2π 径向半径增长一个 radialStep，保证螺旋行距恒定。
    const double spiralGrowth = radialStep / strategy::kTwoPi;
    const double thetaEnd = maximumRadius / spiralGrowth;
    double dTheta = 0.0;
    std::string warning;

    HoleMachiningOperation operation;
    operation.operationId = "SPIRAL" + std::to_string(operationIndex + 1);
    operation.kind = hole.kind;
    operation.strategy = HoleMachiningStrategy::LargeLayeredSpiral;
    operation.entry = hole.entry;
    operation.bottom = hole.bottom;
    operation.commandedBottom = hole.bottom;
    operation.hasCommandedBottom = true;
    operation.radius = hole.radius;

    const double layerCountValue = std::ceil(frame.depth / layerDepth);
    const int maxPoints = std::max(16, params.hole.maxSamplePoints);
    if (!std::isfinite(layerCountValue) ||
        layerCountValue < 1.0 ||
        layerCountValue > static_cast<double>(maxPoints)) {
        addHoleDiagnostic(
            result, ToolpathDiagnosticCode::SamplingBudgetExceeded,
            ToolpathDiagnosticSeverity::Error,
            operation.operationId +
                ": hole path skipped because layer count exceeds the sampling budget",
            -1, operation.operationId);
        return operation;
    }

    const int layerCount = static_cast<int>(layerCountValue);
    if (!strategy::fitSampleStep(
            (thetaEnd + strategy::kTwoPi) * layerCount,
            params, dTheta, warning)) {
        addHoleDiagnostic(
            result, ToolpathDiagnosticCode::SamplingBudgetExceeded,
            ToolpathDiagnosticSeverity::Error,
            operation.operationId + ": " + warning, -1,
            operation.operationId);
        return operation;
    }

    bool firstLayer = true;
    gp_Pnt currentClearance;
    // 每层：清空高度快移到层中心 → 下刀 → 螺旋外扩至最大半径 → 外圆整圈
    // 精加工 → 退到清空高度；层间连接复用当前清空点做直连快移。
    for (int layer = 1; layer <= layerCount; ++layer) {
        const double depth = std::min(frame.depth, layer * layerDepth);
        const gp_Pnt center =
            strategy::pointAt(frame, depth, 0.0, 0.0);
        const gp_Pnt safe = strategy::pointAt(
            frame, -strategy::safeOffset(params), 0.0, 0.0);
        const double clearanceOffset =
            strategy::effectiveClearanceOffset(params, rapidPlanner);
        const gp_Pnt clearance =
            strategy::pointAt(frame, -clearanceOffset, 0.0, 0.0);

        if (firstLayer && previousClearance) {
            if (!strategy::addClearanceLink(
                    operation, *previousClearance, clearance,
                    operationIndex, params, rapidPlanner)) {
                return {};
            }
        } else if (!firstLayer) {
            strategy::addClearanceLink(
                operation, currentClearance, clearance,
                operationIndex, params, nullptr);
        }

        strategy::appendMove(
            operation.moves, ToolpathMoveType::Rapid, clearance, safe,
            operationIndex, 0, params.tool.vertRapid);
        strategy::appendMove(
            operation.moves, ToolpathMoveType::Plunge, safe, center,
            operationIndex, layer, params.tool.vertFeed);

        gp_Pnt previous = center;
        const int spiralSteps =
            std::max(1, static_cast<int>(std::ceil(thetaEnd / dTheta)));
        for (int index = 1; index <= spiralSteps; ++index) {
            const double theta = std::min(thetaEnd, index * dTheta);
            const double radius =
                std::min(maximumRadius, spiralGrowth * theta);
            const gp_Pnt current =
                strategy::pointAt(frame, depth, radius, theta);
            strategy::appendMove(
                operation.moves, ToolpathMoveType::Cut, previous, current,
                operationIndex, layer, params.tool.horizFeed);
            previous = current;
        }

        const int finishSteps = std::max(
            8, static_cast<int>(std::ceil(strategy::kTwoPi / dTheta)));
        const double finishAngle = thetaEnd + strategy::kTwoPi;
        for (int index = 1; index <= finishSteps; ++index) {
            const double theta = thetaEnd + std::min(
                strategy::kTwoPi,
                index * (strategy::kTwoPi / finishSteps));
            const gp_Pnt current = strategy::pointAt(
                frame, depth, maximumRadius, theta);
            strategy::appendMove(
                operation.moves, ToolpathMoveType::Cut, previous, current,
                operationIndex, layer, params.tool.horizFeed);
            previous = current;
        }

        const double centeringOffset =
            strategy::effectiveCenteringOffset(params, rapidPlanner);
        const gp_Pnt finishSafe = strategy::pointAt(
            frame, -centeringOffset, maximumRadius, finishAngle);
        const gp_Pnt centerSafe =
            strategy::pointAt(frame, -centeringOffset, 0.0, 0.0);
        currentClearance =
            strategy::pointAt(frame, -clearanceOffset, 0.0, 0.0);
        strategy::appendMove(
            operation.moves, ToolpathMoveType::Retract, previous,
            finishSafe, operationIndex, layer, params.tool.vertFeed);
        strategy::appendMove(
            operation.moves, ToolpathMoveType::Rapid, finishSafe,
            centerSafe, operationIndex, layer, params.tool.horizRapid);
        strategy::appendMove(
            operation.moves, ToolpathMoveType::Retract, centerSafe,
            currentClearance, operationIndex, layer,
            params.tool.vertRapid);
        firstLayer = false;
    }

    return operation;
}

} // namespace oap::toolpath::detail

// 中孔螺旋铣策略：按螺距下刀，并在孔底追加一圈定径精加工。
// 属于 oap_toolpath_hole 模块 detail/strategies 私有实现；由 buildMediumHelix
// 对单个孔生成一个加工操作，刀路同时输出为直线段 + 圆弧原语。
#include "toolpath/hole/detail/HoleStrategyGenerators.h"

#include <algorithm>
#include <cmath>

#include "toolpath/hole/detail/strategies/HoleStrategySupport.h"

namespace oap::toolpath::detail {

HoleMachiningOperation buildMediumHelix(
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
    const double cutterCenterRadius =
        hole.radius - cutterRadius - allowance;
    const double pitch = strategy::effectivePitch(frame, params);
    // 总转角 = 2π × 孔深 / 螺距：螺旋每圈下深一个螺距，随深度同步旋转。
    const double thetaEnd = strategy::kTwoPi * frame.depth / pitch;
    double dTheta = 0.0;
    std::string warning;

    HoleMachiningOperation operation;
    operation.operationId = "HELIX" + std::to_string(operationIndex + 1);
    operation.kind = hole.kind;
    operation.strategy = HoleMachiningStrategy::MediumHelical;
    operation.entry = hole.entry;
    operation.bottom = hole.bottom;
    operation.commandedBottom = hole.bottom;
    operation.hasCommandedBottom = true;
    operation.radius = hole.radius;

    ToolpathParams samplingParams = params;
    const double tolerance = params.hole.helixChordTolerance;
    bool samplingValid = std::isfinite(tolerance) && tolerance >= 0.0;
    if (!samplingValid) {
        warning = "helix chord tolerance must be finite and non-negative";
    } else if (tolerance > 0.0) {
        // Stable equivalent of 2*acos(1-e/R); cap at pi for minor chords.
        const double ratio = std::min(0.5, 0.5 * (tolerance / cutterCenterRadius));
        const double maximumAngle =
            4.0 * std::asin(std::sqrt(ratio)) * 180.0 / (0.5 * strategy::kTwoPi);
        samplingParams.hole.maxSampleAngleDeg =
            std::min(params.hole.maxSampleAngleDeg, maximumAngle);
        if (!std::isfinite(maximumAngle) || maximumAngle <= 0.0 ||
            samplingParams.hole.maxSampleAngleDeg <
                std::max(0.1, params.hole.minSampleAngleDeg)) {
            samplingValid = false;
            warning = "helix chord tolerance conflicts with the minimum sample angle";
        }
    }
    if (!samplingValid || !strategy::fitSampleStep(
            thetaEnd + strategy::kTwoPi, samplingParams, dTheta, warning)) {
        addHoleDiagnostic(
            result, ToolpathDiagnosticCode::SamplingBudgetExceeded,
            ToolpathDiagnosticSeverity::Error,
            operation.operationId + ": " + warning, -1,
            operation.operationId);
        return operation;
    }

    const gp_Pnt start =
        strategy::pointAt(frame, 0.0, cutterCenterRadius, 0.0);
    const double clearanceOffset =
        strategy::effectiveClearanceOffset(params, rapidPlanner);
    const double centeringOffset =
        strategy::effectiveCenteringOffset(params, rapidPlanner);
    const gp_Pnt clearance =
        strategy::pointAt(frame, -clearanceOffset, 0.0, 0.0);
    const gp_Pnt centerSafe =
        strategy::pointAt(frame, -centeringOffset, 0.0, 0.0);
    const gp_Pnt safe =
        strategy::pointAt(frame, -centeringOffset, cutterCenterRadius, 0.0);

    if (previousClearance &&
        !strategy::addClearanceLink(
            operation, *previousClearance, clearance, operationIndex,
            params, rapidPlanner)) {
        return {};
    }
    strategy::appendMove(
        operation.moves, ToolpathMoveType::Rapid, clearance, centerSafe,
        operationIndex, 0, params.tool.vertRapid);
    // 径向移动必须位于实际安全面之外。
    strategy::appendMove(
        operation.moves, ToolpathMoveType::Rapid, centerSafe, safe,
        operationIndex, 0, params.tool.horizRapid);
    strategy::appendMove(
        operation.moves, ToolpathMoveType::Plunge, safe, start,
        operationIndex, 1, params.tool.vertFeed);
    appendLinearPrimitives(
        operation.primitives, operation.moves, 0, operation.moves.size());

    gp_Pnt previous = start;
    const int helixSteps =
        std::max(1, static_cast<int>(std::ceil(thetaEnd / dTheta)));
    for (int index = 1; index <= helixSteps; ++index) {
        const double theta = std::min(thetaEnd, index * dTheta);
        const double depth = std::min(
            frame.depth, pitch * theta / strategy::kTwoPi);
        const gp_Pnt current = strategy::pointAt(
            frame, depth, cutterCenterRadius, theta);
        strategy::appendMove(
            operation.moves, ToolpathMoveType::Cut, previous, current,
            operationIndex, strategy::pitchDepthIndex(depth, pitch),
            params.tool.horizFeed);
        previous = current;
    }

    // 原生圆弧按最多一整圈拆分，避免依赖控制器私有的多圈 P 语法。
    double arcStart = 0.0;
    while (arcStart < thetaEnd - 1.0e-10) {
        const double arcEnd = std::min(
            thetaEnd, arcStart + strategy::kTwoPi);
        const double startDepth = std::min(
            frame.depth, pitch * arcStart / strategy::kTwoPi);
        const double endDepth = std::min(
            frame.depth, pitch * arcEnd / strategy::kTwoPi);
        ToolpathCircularArc arc;
        arc.direction = ToolpathArcDirection::Counterclockwise;
        arc.from = strategy::pointAt(
            frame, startDepth, cutterCenterRadius, arcStart);
        arc.to = strategy::pointAt(
            frame, endDepth, cutterCenterRadius, arcEnd);
        arc.center = strategy::pointAt(frame, startDepth, 0.0, 0.0);
        arc.axis = frame.axis;
        arc.sweepRadians = arcEnd - arcStart;
        arc.feed = params.tool.horizFeed;
        operation.primitives.push_back(makeArcPrimitive(arc, {}));
        arcStart = arcEnd;
    }

    // 底部定径精加工：孔深保持不动，沿外圆再走一整圈，消除螺旋切入遗留
    // 的径向误差；同时给出对应的整圈圆弧原语。
    const int finishSteps = std::max(
        8, static_cast<int>(std::ceil(strategy::kTwoPi / dTheta)));
    const int finishDepthIndex =
        strategy::pitchDepthIndex(frame.depth, pitch);
    const double finishAngle = thetaEnd + strategy::kTwoPi;
    for (int index = 1; index <= finishSteps; ++index) {
        const double theta = thetaEnd + std::min(
            strategy::kTwoPi,
            index * (strategy::kTwoPi / finishSteps));
        const gp_Pnt current = strategy::pointAt(
            frame, frame.depth, cutterCenterRadius, theta);
        strategy::appendMove(
            operation.moves, ToolpathMoveType::Cut, previous, current,
            operationIndex, finishDepthIndex, params.tool.horizFeed);
        previous = current;
    }
    ToolpathCircularArc finishArc;
    finishArc.direction = ToolpathArcDirection::Counterclockwise;
    finishArc.from = strategy::pointAt(
        frame, frame.depth, cutterCenterRadius, thetaEnd);
    finishArc.to = strategy::pointAt(
        frame, frame.depth, cutterCenterRadius,
        thetaEnd + strategy::kTwoPi);
    finishArc.center = strategy::pointAt(frame, frame.depth, 0.0, 0.0);
    finishArc.axis = frame.axis;
    finishArc.sweepRadians = strategy::kTwoPi;
    finishArc.feed = params.tool.horizFeed;
    operation.primitives.push_back(makeArcPrimitive(finishArc, {}));

    const std::size_t retractMoveBegin = operation.moves.size();
    const gp_Pnt finishSafe = strategy::pointAt(
        frame, -centeringOffset, cutterCenterRadius, finishAngle);
    const gp_Pnt finishClearance =
        strategy::pointAt(frame, -clearanceOffset, 0.0, 0.0);
    strategy::appendMove(
        operation.moves, ToolpathMoveType::Retract, previous, finishSafe,
        operationIndex, finishDepthIndex, params.tool.vertFeed);
    strategy::appendMove(
        operation.moves, ToolpathMoveType::Rapid, finishSafe, centerSafe,
        operationIndex, finishDepthIndex, params.tool.horizRapid);
    strategy::appendMove(
        operation.moves, ToolpathMoveType::Retract, centerSafe,
        finishClearance, operationIndex, finishDepthIndex,
        params.tool.vertRapid);
    appendLinearPrimitives(
        operation.primitives, operation.moves, retractMoveBegin,
        operation.moves.size());
    return operation;
}

} // namespace oap::toolpath::detail

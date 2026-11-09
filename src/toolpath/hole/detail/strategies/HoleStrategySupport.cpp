// 孔策略共享实现：建立局部正交框架、控制采样预算并组装安全连接移动。
// 属于 oap_toolpath_hole 模块 detail/strategies 私有实现。
#include "toolpath/hole/detail/strategies/HoleStrategySupport.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include <gp_Vec.hxx>

namespace oap::toolpath::detail {
namespace {

constexpr double kDefaultSampleAngleDeg = 10.0;
constexpr double kDefaultMinSampleAngleDeg = 3.0;
constexpr double kDefaultMaxSampleAngleDeg = 20.0;

bool finitePoint(const gp_Pnt& point)
{
    return std::isfinite(point.X()) &&
           std::isfinite(point.Y()) &&
           std::isfinite(point.Z());
}

double clearanceOffset(const ToolpathParams& params)
{
    const double safe = strategy::safeOffset(params);
    if (!std::isfinite(params.depth.clearanceHeight))
        return safe;
    return std::max(params.depth.clearanceHeight, safe);
}

double minimumSampleAngleDeg(const ToolpathParams& params)
{
    if (std::isfinite(params.hole.minSampleAngleDeg) &&
        params.hole.minSampleAngleDeg > 0.0) {
        return std::max(0.1, params.hole.minSampleAngleDeg);
    }
    return kDefaultMinSampleAngleDeg;
}

double maximumSampleAngleDeg(const ToolpathParams& params,
                             double minimumDeg)
{
    if (std::isfinite(params.hole.maxSampleAngleDeg) &&
        params.hole.maxSampleAngleDeg >= minimumDeg) {
        return params.hole.maxSampleAngleDeg;
    }
    return std::max(minimumDeg, kDefaultMaxSampleAngleDeg);
}

double clampedSampleAngleRad(const ToolpathParams& params)
{
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double minimumDeg = minimumSampleAngleDeg(params);
    const double maximumDeg = maximumSampleAngleDeg(params, minimumDeg);
    const double requestedDeg = std::isfinite(params.hole.sampleAngleDeg)
        ? params.hole.sampleAngleDeg
        : kDefaultSampleAngleDeg;
    return std::clamp(requestedDeg, minimumDeg, maximumDeg) * kDegToRad;
}

double maximumSampleAngleRad(const ToolpathParams& params)
{
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double minimumDeg = minimumSampleAngleDeg(params);
    return maximumSampleAngleDeg(params, minimumDeg) * kDegToRad;
}

} // namespace

bool makeHoleFrame(const HoleMachiningInput& hole, HoleFrame& frame)
{
    if (!finitePoint(hole.entry) || !finitePoint(hole.bottom))
        return false;

    const double length = hole.entry.Distance(hole.bottom);
    if (!std::isfinite(length) || length <= kHoleTolerance)
        return false;

    gp_Vec axis(hole.entry, hole.bottom);
    axis.Normalize();

    // 选择不接近孔轴的参考向量，再通过叉积建立稳定右手局部坐标系。
    gp_Vec reference(0.0, 0.0, 1.0);
    if (std::abs(axis.Dot(reference)) > 0.9)
        reference = gp_Vec(1.0, 0.0, 0.0);

    gp_Vec u = axis.Crossed(reference);
    if (u.Magnitude() <= kHoleTolerance)
        return false;
    u.Normalize();

    gp_Vec v = axis.Crossed(u);
    if (v.Magnitude() <= kHoleTolerance)
        return false;
    v.Normalize();

    frame.origin = hole.entry;
    frame.axis = axis;
    frame.u = u;
    frame.v = v;
    frame.depth = length;
    return true;
}

namespace strategy {

void appendMove(std::vector<ToolpathMove>& moves,
                ToolpathMoveType type,
                const gp_Pnt& from,
                const gp_Pnt& to,
                int operationIndex,
                int depthIndex,
                double feed,
                ToolpathMoveIntent intent)
{
    if (from.Distance(to) <= kHoleTolerance)
        return;

    ToolpathMove move;
    move.type = type;
    move.from = from;
    move.to = to;
    move.loopSequence = operationIndex;
    move.depthIndex = depthIndex;
    move.ring = 0;
    move.feed = feed;
    move.intent = intent;
    moves.push_back(std::move(move));
}

gp_Pnt pointAt(const HoleFrame& frame,
               double axialDepth,
               double radius,
               double theta)
{
    const gp_Vec radial =
        frame.u.Multiplied(radius * std::cos(theta)) +
        frame.v.Multiplied(radius * std::sin(theta));
    return frame.origin.Translated(frame.axis.Multiplied(axialDepth) + radial);
}

double safeOffset(const ToolpathParams& params)
{
    if (!std::isfinite(params.depth.safeHeight))
        return 0.0;
    return std::max(0.0, params.depth.safeHeight);
}

bool fitSampleStep(double thetaSpan,
                   const ToolpathParams& params,
                   double& dTheta,
                   std::string& warning)
{
    if (!std::isfinite(thetaSpan) || thetaSpan < 0.0) {
        warning = "hole path skipped because angular span is invalid";
        return false;
    }

    // 先使用期望角步长；超出点数预算时逐步放宽，但不超过最大允许角度。
    dTheta = clampedSampleAngleRad(params);
    const int maxPoints = std::max(16, params.hole.maxSamplePoints);
    const auto fitsBudget = [maxPoints](double span, double step) {
        return std::isfinite(step) && step > 0.0 &&
               span / step <= static_cast<double>(maxPoints - 1);
    };

    if (fitsBudget(thetaSpan, dTheta))
        return true;

    const double needed = thetaSpan / static_cast<double>(maxPoints - 1);
    dTheta = std::min(
        std::max(needed, dTheta), maximumSampleAngleRad(params));
    if (fitsBudget(thetaSpan, dTheta))
        return true;

    std::ostringstream message;
    message << "hole path skipped because sampling would exceed "
            << maxPoints << " points";
    warning = message.str();
    return false;
}

double effectivePitch(const HoleFrame& frame,
                      const ToolpathParams& params)
{
    // 螺距优先使用孔专用配置，其次复用通用步下量，最后退化为单层到底。
    if (std::isfinite(params.hole.helixPitch) &&
        params.hole.helixPitch > kHoleTolerance) {
        return params.hole.helixPitch;
    }
    if (std::isfinite(params.depth.stepDown) &&
        params.depth.stepDown > kHoleTolerance) {
        return params.depth.stepDown;
    }
    return frame.depth;
}

double effectiveLayerDepth(const HoleFrame& frame,
                           const ToolpathParams& params)
{
    if (std::isfinite(params.depth.stepDown) &&
        params.depth.stepDown > kHoleTolerance) {
        return params.depth.stepDown;
    }
    return frame.depth;
}

double effectiveRadialStep(const ToolpathParams& params)
{
    // 径向步距未指定时复用槽腔步距百分比，使孔铣与槽腔清料参数语义一致。
    if (std::isfinite(params.hole.radialStep) &&
        params.hole.radialStep > kHoleTolerance) {
        return params.hole.radialStep;
    }

    double step =
        params.tool.diameter * params.clearing.stepOverPercent / 100.0;
    if (!std::isfinite(step) || step <= kHoleTolerance)
        step = params.tool.diameter * 0.5;
    return step;
}

int pitchDepthIndex(double depth, double pitch)
{
    const double adjustedDepth = std::max(0.0, depth - kHoleTolerance);
    return std::max(
        1, static_cast<int>(std::ceil(adjustedDepth / pitch)));
}

double effectiveClearanceOffset(const ToolpathParams& params,
                                const HoleRapidPlanner* planner)
{
    const double minimum = planner
        ? planner->minimumClearanceOffset()
        : params.tool.radius() + 2.0 * kHoleTolerance;
    return std::max(clearanceOffset(params), minimum);
}

double effectiveCenteringOffset(const ToolpathParams& params,
                                const HoleRapidPlanner* planner)
{
    const double minimum = planner
        ? planner->minimumClearanceOffset()
        : params.tool.radius() + 2.0 * kHoleTolerance;
    return std::max(safeOffset(params), minimum);
}

bool addClearanceLink(HoleMachiningOperation& operation,
                      const gp_Pnt& from,
                      const gp_Pnt& to,
                      int operationIndex,
                      const ToolpathParams& params,
                      HoleRapidPlanner* planner)
{
    if (planner) {
        // 存在障碍规划上下文时必须经碰撞检查；无上下文的调用保留旧版直连行为。
        return planner->appendLink(
            operation.moves, from, to, operationIndex);
    }
    appendMove(operation.moves, ToolpathMoveType::Rapid, from, to,
               operationIndex, 0, params.tool.horizRapid);
    return true;
}

} // namespace strategy
} // namespace oap::toolpath::detail

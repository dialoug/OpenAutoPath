// 折线 -> 直线/圆弧语义原语压缩实现：把连续的切削移动段拟合成圆弧
// （圆心+半径+扫掠角），并在安全区域内校验拟合圆弧的刀心可达性。
// 属于 oap_toolpath_pocket 内部实现（detail/），不对外暴露。
#include "toolpath/pocket/planning/detail/PocketCurveSemanticBuilder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include "toolpath/pocket/planning/detail/PocketGeometryUtils.h"
#include "toolpath/pocket/planning/detail/PocketSafetyGeometry.h"

namespace oap::toolpath::detail {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMoveTolerance = 1.0e-9;
constexpr double kStraightTurnTolerance = 0.25 * kPi / 180.0;
constexpr double kMinimumArcSweep = 5.0 * kPi / 180.0;
constexpr double kMaximumArcStep = 30.0 * kPi / 180.0;
constexpr std::size_t kMinimumArcSegments = 3;

struct CircleFit {
    gp_Pnt2d center;
    double radius = 0.0;
    double sweep = 0.0;
    double maximumResidual = 0.0;
    ToolpathArcDirection direction =
        ToolpathArcDirection::Counterclockwise;
};

double normalizedAngle(double angle)
{
    while (angle <= -kPi)
        angle += 2.0 * kPi;
    while (angle > kPi)
        angle -= 2.0 * kPi;
    return angle;
}

bool sameMoveContext(const ToolpathMove& first,
                     const ToolpathMove& second)
{
    const double feedScale = std::max(
        {1.0, std::abs(first.feed), std::abs(second.feed)});
    return first.type == second.type &&
           first.loopSequence == second.loopSequence &&
           first.depthIndex == second.depthIndex &&
           first.ring == second.ring &&
           first.intent == second.intent &&
           std::abs(first.feed - second.feed) <= 1.0e-9 * feedScale;
}

bool solve3x3(std::array<std::array<double, 4>, 3>& matrix,
              std::array<double, 3>& solution)
{
    for (int column = 0; column < 3; ++column) {
        int pivot = column;
        for (int row = column + 1; row < 3; ++row) {
            if (std::abs(matrix[row][column]) >
                std::abs(matrix[pivot][column])) {
                pivot = row;
            }
        }
        if (std::abs(matrix[pivot][column]) <= 1.0e-18)
            return false;
        if (pivot != column)
            std::swap(matrix[pivot], matrix[column]);
        const double divisor = matrix[column][column];
        for (int entry = column; entry < 4; ++entry)
            matrix[column][entry] /= divisor;
        for (int row = 0; row < 3; ++row) {
            if (row == column)
                continue;
            const double factor = matrix[row][column];
            for (int entry = column; entry < 4; ++entry)
                matrix[row][entry] -= factor * matrix[column][entry];
        }
    }
    for (int row = 0; row < 3; ++row)
        solution[row] = matrix[row][3];
    return true;
}

std::optional<CircleFit> fitCircle(
    const std::vector<gp_Pnt2d>& points,
    std::size_t firstPoint,
    std::size_t lastPoint,
    double fitTolerance)
{
    if (lastPoint < firstPoint + kMinimumArcSegments)
        return std::nullopt;

    const std::size_t count = lastPoint - firstPoint + 1;
    // 圆拟合采用“中心在原点的最小二乘”代数解：令点 (x,y) 满足
    // x^2+y^2 = 2cx*x + 2cy*y + (r^2 - c^2)，对未知数 c 构造 3x3
    // 法方程并高斯消元，避免迭代初值依赖。
    double meanX = 0.0;
    double meanY = 0.0;
    for (std::size_t index = firstPoint; index <= lastPoint; ++index) {
        meanX += points[index].X();
        meanY += points[index].Y();
    }
    meanX /= static_cast<double>(count);
    meanY /= static_cast<double>(count);

    std::array<std::array<double, 4>, 3> matrix{};
    for (std::size_t index = firstPoint; index <= lastPoint; ++index) {
        const double x = points[index].X() - meanX;
        const double y = points[index].Y() - meanY;
        const double square = x * x + y * y;
        matrix[0][0] += x * x;
        matrix[0][1] += x * y;
        matrix[0][2] += x;
        matrix[0][3] -= x * square;
        matrix[1][0] += x * y;
        matrix[1][1] += y * y;
        matrix[1][2] += y;
        matrix[1][3] -= y * square;
        matrix[2][0] += x;
        matrix[2][1] += y;
        matrix[2][2] += 1.0;
        matrix[2][3] -= square;
    }
    std::array<double, 3> solution{};
    if (!solve3x3(matrix, solution))
        return std::nullopt;

    const double localCenterX = -0.5 * solution[0];
    const double localCenterY = -0.5 * solution[1];
    const double radiusSquared =
        localCenterX * localCenterX +
        localCenterY * localCenterY - solution[2];
    if (!std::isfinite(radiusSquared) ||
        radiusSquared <= fitTolerance * fitTolerance) {
        return std::nullopt;
    }

    CircleFit fit;
    fit.center = gp_Pnt2d(
        meanX + localCenterX,
        meanY + localCenterY);
    fit.radius = std::sqrt(radiusSquared);
    if (!std::isfinite(fit.radius))
        return std::nullopt;

    int directionSign = 0;
    double previousAngle = std::atan2(
        points[firstPoint].Y() - fit.center.Y(),
        points[firstPoint].X() - fit.center.X());
    for (std::size_t index = firstPoint; index <= lastPoint; ++index) {
        const double radius = points[index].Distance(fit.center);
        fit.maximumResidual = std::max(
            fit.maximumResidual, std::abs(radius - fit.radius));
        if (index == firstPoint)
            continue;
        const double angle = std::atan2(
            points[index].Y() - fit.center.Y(),
            points[index].X() - fit.center.X());
        const double delta = normalizedAngle(angle - previousAngle);
        if (std::abs(delta) <= 1.0e-8 ||
            std::abs(delta) > kMaximumArcStep) {
            return std::nullopt;
        }
        const int sign = delta > 0.0 ? 1 : -1;
        if (directionSign == 0)
            directionSign = sign;
        else if (sign != directionSign)
            return std::nullopt;
        fit.sweep += std::abs(delta);
        previousAngle = angle;
    }
    if (fit.maximumResidual > fitTolerance ||
        fit.sweep < kMinimumArcSweep ||
        fit.sweep > 2.0 * kPi + 1.0e-6) {
        return std::nullopt;
    }
    fit.direction = directionSign > 0
        ? ToolpathArcDirection::Counterclockwise
        : ToolpathArcDirection::Clockwise;
    const auto tangentAgreement = [&](std::size_t radialPoint,
                                      std::size_t chordStart,
                                      std::size_t chordEnd) {
        const double radialX =
            points[radialPoint].X() - fit.center.X();
        const double radialY =
            points[radialPoint].Y() - fit.center.Y();
        double tangentX = directionSign > 0 ? -radialY : radialY;
        double tangentY = directionSign > 0 ? radialX : -radialX;
        const double chordX =
            points[chordEnd].X() - points[chordStart].X();
        const double chordY =
            points[chordEnd].Y() - points[chordStart].Y();
        const double tangentLength = std::hypot(tangentX, tangentY);
        const double chordLength = std::hypot(chordX, chordY);
        if (tangentLength <= kMoveTolerance ||
            chordLength <= kMoveTolerance) {
            return false;
        }
        tangentX /= tangentLength;
        tangentY /= tangentLength;
        return (tangentX * chordX + tangentY * chordY) /
                   chordLength >=
               std::cos(20.0 * kPi / 180.0);
    };
    if (!tangentAgreement(
            firstPoint, firstPoint, firstPoint + 1) ||
        !tangentAgreement(
            lastPoint, lastPoint - 1, lastPoint)) {
        return std::nullopt;
    }
    return fit;
}

double signedTurn(const gp_Pnt2d& first,
                  const gp_Pnt2d& middle,
                  const gp_Pnt2d& last)
{
    const double firstX = middle.X() - first.X();
    const double firstY = middle.Y() - first.Y();
    const double secondX = last.X() - middle.X();
    const double secondY = last.Y() - middle.Y();
    const double firstLength = std::hypot(firstX, firstY);
    const double secondLength = std::hypot(secondX, secondY);
    if (firstLength <= kMoveTolerance ||
        secondLength <= kMoveTolerance) {
        return 0.0;
    }
    return std::atan2(
        firstX * secondY - firstY * secondX,
        firstX * secondX + firstY * secondY);
}

bool fittedArcIsSafe(const CircleFit& fit,
                     const gp_Pnt2d& start,
                     const ToolpathParams& params,
                     const PocketSafetyArea& safetyArea,
                     int depthIndex)
{
    const double requiredMargin =
        params.tool.radius() +
        std::max(0.0, params.clearing.extraOffset);
    const double spacing = std::clamp(
        requiredMargin > 0.0 ? requiredMargin / 3.0 : 0.25,
        0.05, 0.5);
    const double sampleValue = std::ceil(
        fit.radius * fit.sweep / spacing);
    if (!std::isfinite(sampleValue) || sampleValue > 200000.0)
        return false;
    const std::size_t samples = std::max<std::size_t>(
        2, static_cast<std::size_t>(sampleValue));
    const double direction =
        fit.direction == ToolpathArcDirection::Counterclockwise
            ? 1.0 : -1.0;
    const double startAngle = std::atan2(
        start.Y() - fit.center.Y(),
        start.X() - fit.center.X());
    for (std::size_t sample = 0; sample <= samples; ++sample) {
        const double fraction =
            static_cast<double>(sample) /
            static_cast<double>(samples);
        const double angle =
            startAngle + direction * fit.sweep * fraction;
        const gp_Pnt2d point(
            fit.center.X() + fit.radius * std::cos(angle),
            fit.center.Y() + fit.radius * std::sin(angle));
        if (!safetyArea.clear(point, depthIndex)) {
            return false;
        }
    }
    return true;
}

double semanticFitTolerance(const ClearingLoop& loop,
                            const ToolpathParams& params)
{
    const double grid = std::pow(
        10.0, -std::clamp(params.clearing.precision, 0, 8));
    const double configured =
        std::isfinite(params.clearing.arcTolerance) &&
                params.clearing.arcTolerance > 0.0
            ? params.clearing.arcTolerance
            : std::abs(loop.offset) * 0.002;
    return std::clamp(
        std::max({1.0e-5, 2.0 * grid, configured}),
        1.0e-5, 0.01);
}

std::vector<ToolpathMove> moveRange(
    const std::vector<ToolpathMove>& moves,
    std::size_t begin,
    std::size_t end)
{
    return std::vector<ToolpathMove>(
        moves.begin() + static_cast<std::ptrdiff_t>(begin),
        moves.begin() + static_cast<std::ptrdiff_t>(end));
}

} // namespace

bool isPocketArcCutterCenterSafe(
    const ToolpathCircularArc& arc,
    const ToolpathPrimitiveContext& context,
    const PocketRegion& region,
    const ToolpathParams& params,
    const PocketSafetyArea& safetyArea,
    const std::function<bool()>& isCancellationRequested)
{
    if (isCancellationRequested && isCancellationRequested())
        return false;
    if (!std::isfinite(arc.sweepRadians) ||
        arc.sweepRadians <= 0.0 ||
        arc.axis.SquareMagnitude() <= 1.0e-16) {
        return false;
    }
    gp_Vec axis = arc.axis;
    axis.Normalize();
    gp_Vec frameAxis(region.frame.Direction());
    frameAxis.Normalize();
    const double alignment = axis.Dot(frameAxis);
    if (std::abs(alignment) < 1.0 - 1.0e-6)
        return false;

    const gp_Pnt2d center =
        projectToPocketPlane(arc.center, region.frame);
    const gp_Pnt2d start =
        projectToPocketPlane(arc.from, region.frame);
    const gp_Pnt2d end =
        projectToPocketPlane(arc.to, region.frame);
    const double radius = center.Distance(start);
    if (!std::isfinite(radius) || radius <= kMoveTolerance)
        return false;

    double direction =
        arc.direction == ToolpathArcDirection::Counterclockwise
            ? 1.0 : -1.0;
    if (alignment < 0.0)
        direction = -direction;
    const double startAngle = std::atan2(
        start.Y() - center.Y(), start.X() - center.X());
    const double expectedEndAngle =
        startAngle + direction * arc.sweepRadians;
    const gp_Pnt2d expectedEnd(
        center.X() + radius * std::cos(expectedEndAngle),
        center.Y() + radius * std::sin(expectedEndAngle));

    const double requiredMargin =
        params.tool.radius() +
        std::max(0.0, params.clearing.extraOffset);
    const double grid = std::pow(
        10.0, -std::clamp(params.clearing.precision, 0, 8));
    const double tolerance = std::max(0.02, 4.0 * grid);
    if (expectedEnd.Distance(end) >
        std::max(tolerance, 1.0e-6 * radius)) {
        return false;
    }
    const double spacing = std::clamp(
        requiredMargin > 0.0 ? requiredMargin / 3.0 : 0.25,
        0.05, 0.5);
    const double sampleValue = std::ceil(
        radius * arc.sweepRadians / spacing);
    if (!std::isfinite(sampleValue) || sampleValue > 200000.0)
        return false;
    const std::size_t samples = std::max<std::size_t>(
        2, static_cast<std::size_t>(sampleValue));
    for (std::size_t sample = 0; sample <= samples; ++sample) {
        if ((sample & 0xffu) == 0u &&
            isCancellationRequested &&
            isCancellationRequested()) {
            return false;
        }
        const double fraction =
            static_cast<double>(sample) /
            static_cast<double>(samples);
        const double angle =
            startAngle + direction * arc.sweepRadians * fraction;
        const gp_Pnt2d point(
            center.X() + radius * std::cos(angle),
            center.Y() + radius * std::sin(angle));
        if (!safetyArea.clear(point, context.depthIndex)) {
            return false;
        }
    }
    return true;
}

std::vector<ToolpathPrimitive> buildPocketCurvePrimitives(
    const PocketRegion& region,
    const ToolpathParams& params,
    const PocketSafetyArea& safetyArea,
    const ClearingLoop& loop,
    const std::vector<ToolpathMove>& moves,
    std::size_t begin,
    std::size_t end)
{
    std::vector<ToolpathPrimitive> result;
    begin = std::min(begin, moves.size());
    end = std::min(std::max(begin, end), moves.size());
    if (begin == end)
        return result;

    const std::size_t count = end - begin;
    std::vector<gp_Pnt2d> points;
    points.reserve(count + 1);
    points.push_back(projectToPocketPlane(moves[begin].from, region.frame));
    for (std::size_t index = begin; index < end; ++index)
        points.push_back(projectToPocketPlane(moves[index].to, region.frame));

    const double fitTolerance = semanticFitTolerance(loop, params);
    std::size_t local = 0;
    while (local < count) {
        const ToolpathMove& firstMove = moves[begin + local];
        if (firstMove.type != ToolpathMoveType::Cut ||
            local + 1 >= count) {
            result.push_back(makeLinearPrimitive(firstMove));
            ++local;
            continue;
        }

        const double initialTurn = signedTurn(
            points[local], points[local + 1], points[local + 2]);
        // 连续三点出现同向大转角且移动段上下文一致，才尝试把这一段
        // 压缩成圆弧；否则按直线原语合并处理。
        if (std::abs(initialTurn) > kStraightTurnTolerance &&
            sameMoveContext(firstMove, moves[begin + local + 1])) {
            const int sign = initialTurn > 0.0 ? 1 : -1;
            // 从当前点向后扫描，收集转向一致且累计扫掠角不超过一圈
            // 的连续段，构成圆弧候选窗口 runEnd。
            std::size_t runEnd = local + 2;
            double accumulated = std::abs(initialTurn);
            while (runEnd < count) {
                if (!sameMoveContext(
                        firstMove, moves[begin + runEnd])) {
                    break;
                }
                const double turn = signedTurn(
                    points[runEnd - 1], points[runEnd],
                    points[runEnd + 1]);
                if (std::abs(turn) <= kStraightTurnTolerance ||
                    (turn > 0.0 ? 1 : -1) != sign ||
                    accumulated + std::abs(turn) >
                        2.0 * kPi + kStraightTurnTolerance) {
                    break;
                }
                accumulated += std::abs(turn);
                ++runEnd;
            }

            std::size_t candidateEnd = runEnd;
            std::optional<CircleFit> fit;
            for (int attempt = 0;
                 attempt < 3 &&
                 candidateEnd >= local + kMinimumArcSegments;
                 ++attempt, --candidateEnd) {
                fit = fitCircle(
                    points, local, candidateEnd, fitTolerance);
                if (fit && fittedArcIsSafe(
                               *fit, points[local], params,
                               safetyArea, firstMove.depthIndex)) {
                    break;
                }
                fit.reset();
            }
            if (fit) {
                const std::size_t semanticEnd = candidateEnd;
                std::vector<ToolpathMove> fallback = moveRange(
                    moves, begin + local, begin + semanticEnd);
                ToolpathCircularArc arc;
                arc.moveType = ToolpathMoveType::Cut;
                arc.direction = fit->direction;
                arc.from = fallback.front().from;
                arc.to = fallback.back().to;
                arc.center = pointFrom2dAtDepth(
                    fit->center, region.frame, loop.depthOffset);
                arc.axis = gp_Vec(region.frame.Direction());
                arc.sweepRadians = fit->sweep;
                arc.feed = firstMove.feed;
                result.push_back(makeArcPrimitive(
                    arc, std::move(fallback),
                    ToolpathPrimitiveContext{
                        firstMove.loopSequence,
                        firstMove.depthIndex,
                        firstMove.ring,
                        firstMove.intent}));
                local = semanticEnd;
                continue;
            }
            // 同向弯折段整体不满足圆拟合时一次性保留原始线段，避免在长样条上
            // 从每个相邻点重复尝试同一失败窗口而退化为二次复杂度。
            for (std::size_t fallback = local;
                 fallback < runEnd; ++fallback) {
                result.push_back(makeLinearPrimitive(
                    moves[begin + fallback]));
            }
            local = runEnd;
            continue;
        }

        std::size_t linearEnd = local + 1;
        while (linearEnd < count &&
               sameMoveContext(
                   firstMove, moves[begin + linearEnd])) {
            const double turn = signedTurn(
                points[linearEnd - 1], points[linearEnd],
                points[linearEnd + 1]);
            if (std::abs(turn) > 1.0e-8)
                break;
            ++linearEnd;
        }
        ToolpathMove semanticLine = firstMove;
        semanticLine.to = moves[begin + linearEnd - 1].to;
        if (linearEnd == local + 1) {
            result.push_back(makeLinearPrimitive(semanticLine));
        } else {
            result.push_back(makeLinearPrimitive(
                semanticLine,
                moveRange(
                    moves, begin + local,
                    begin + linearEnd)));
        }
        local = linearEnd;
    }
    return result;
}

} // namespace oap::toolpath::detail

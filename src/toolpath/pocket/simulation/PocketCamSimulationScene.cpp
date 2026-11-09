// PocketCamSimulationSceneBuilder 实现（oap_toolpath_pocket）。
// 仿 FreeCAD CAM 模拟器的场景准备：把几何无关的准备工作留在 toolpath 层
// （无需 OpenGL 上下文即可测试），只驱动显示用 CSG 模拟，不参与验证与
// NC 导出。参见 PocketCamSimulationScene.h。
#include "toolpath/pocket/simulation/PocketCamSimulationScene.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <Precision.hxx>

#include "toolpath/core/PocketToolBinding.h"

namespace oap::toolpath {
namespace {

bool finitePoint(const gp_Pnt& point)
{
    return std::isfinite(point.X()) &&
           std::isfinite(point.Y()) &&
           std::isfinite(point.Z());
}

bool finiteVector(const gp_Vec& vector)
{
    return std::isfinite(vector.X()) &&
           std::isfinite(vector.Y()) &&
           std::isfinite(vector.Z());
}

gp_Pnt analyticPointAt(
    const gp_Pnt& from,
    const PocketCamAnalyticSweep& sweep,
    double fraction)
{
    fraction = std::clamp(fraction, 0.0, 1.0);
    gp_Vec axis = sweep.axis;
    axis.Normalize();
    const gp_Vec radial(sweep.center, from);
    const double orientation =
        sweep.direction == ToolpathArcDirection::Counterclockwise
            ? 1.0 : -1.0;
    const double angle =
        orientation * sweep.sweepRadians * fraction;
    const gp_Vec rotated =
        radial.Multiplied(std::cos(angle)) +
        axis.Crossed(radial).Multiplied(std::sin(angle)) +
        axis.Multiplied(
            axis.Dot(radial) * (1.0 - std::cos(angle)));
    return sweep.center.Translated(rotated);
}

struct SourceMoveAnalyticInfo {
    PocketCamAnalyticSweep sweep;
};

constexpr double kMinimumRecoveredArcSweep =
    5.0 * std::numbers::pi / 180.0;
constexpr double kMaximumRecoveredArcStep =
    45.0 * std::numbers::pi / 180.0;
constexpr std::size_t kMinimumRecoveredArcMoves = 3u;
constexpr std::size_t kMaximumRecoveredArcProbeMoves = 256u;

struct RecoveredCircleFit {
    gp_Pnt center;
    double radius = 0.0;
    double sweepRadians = 0.0;
    ToolpathArcDirection direction =
        ToolpathArcDirection::Counterclockwise;
};

double normalizedAngle(double angle)
{
    while (angle <= -std::numbers::pi)
        angle += 2.0 * std::numbers::pi;
    while (angle > std::numbers::pi)
        angle -= 2.0 * std::numbers::pi;
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
           std::abs(first.feed - second.feed) <=
               1.0e-9 * feedScale;
}

bool solveCircleSystem(
    std::array<std::array<double, 4>, 3>& matrix,
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
            for (int entry = column; entry < 4; ++entry) {
                matrix[row][entry] -=
                    factor * matrix[column][entry];
            }
        }
    }
    for (int row = 0; row < 3; ++row)
        solution[row] = matrix[row][3];
    return true;
}

bool planarMove(
    const ToolpathMove& move,
    const gp_Vec& fixedAxis,
    double tolerance)
{
    if (move.type != ToolpathMoveType::Cut ||
        !finitePoint(move.from) || !finitePoint(move.to)) {
        return false;
    }
    return std::abs(
               gp_Vec(move.from, move.to).Dot(fixedAxis)) <=
           tolerance;
}

double directedArcSweep(
    const gp_Pnt& from,
    const gp_Pnt& to,
    const gp_Pnt& center,
    const gp_Vec& sourceAxis,
    ToolpathArcDirection direction,
    double tolerance);

double planarSignedTurn(const gp_Pnt& first,
                        const gp_Pnt& middle,
                        const gp_Pnt& last,
                        const gp_Vec& fixedAxis,
                        double tolerance)
{
    gp_Vec incoming(first, middle);
    gp_Vec outgoing(middle, last);
    const double incomingLength = incoming.Magnitude();
    const double outgoingLength = outgoing.Magnitude();
    if (incomingLength <= tolerance ||
        outgoingLength <= tolerance) {
        return 0.0;
    }
    incoming /= incomingLength;
    outgoing /= outgoingLength;
    return std::atan2(
        fixedAxis.Dot(incoming.Crossed(outgoing)),
        std::clamp(incoming.Dot(outgoing), -1.0, 1.0));
}

std::optional<RecoveredCircleFit> fitRecoveredCircle(
    const std::vector<ToolpathMove>& moves,
    std::size_t begin,
    std::size_t end,
    const gp_Vec& fixedAxis,
    double fitTolerance,
    double geometricTolerance)
{
    if (end < begin + kMinimumRecoveredArcMoves ||
        end > moves.size()) {
        return std::nullopt;
    }

    gp_Vec axis = fixedAxis;
    if (!finiteVector(axis) ||
        axis.Magnitude() <= geometricTolerance) {
        return std::nullopt;
    }
    axis.Normalize();
    gp_Vec basisU = std::abs(axis.X()) < 0.8
        ? gp_Vec(1.0, 0.0, 0.0)
        : gp_Vec(0.0, 1.0, 0.0);
    basisU -= axis.Multiplied(basisU.Dot(axis));
    if (basisU.Magnitude() <= geometricTolerance)
        return std::nullopt;
    basisU.Normalize();
    gp_Vec basisV = axis.Crossed(basisU);
    basisV.Normalize();

    const gp_Pnt origin = moves[begin].from;
    std::vector<std::array<double, 2>> points;
    points.reserve(end - begin + 1u);
    const auto appendPoint = [&](const gp_Pnt& point) {
        const gp_Vec relative(origin, point);
        points.push_back({
            relative.Dot(basisU), relative.Dot(basisV)});
    };
    appendPoint(origin);
    for (std::size_t index = begin; index < end; ++index)
        appendPoint(moves[index].to);

    double meanX = 0.0;
    double meanY = 0.0;
    for (const auto& point : points) {
        meanX += point[0];
        meanY += point[1];
    }
    meanX /= static_cast<double>(points.size());
    meanY /= static_cast<double>(points.size());

    std::array<std::array<double, 4>, 3> matrix{};
    for (const auto& point : points) {
        const double x = point[0] - meanX;
        const double y = point[1] - meanY;
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
    if (!solveCircleSystem(matrix, solution))
        return std::nullopt;

    const double centerX = meanX - 0.5 * solution[0];
    const double centerY = meanY - 0.5 * solution[1];
    const double localCenterX = centerX - meanX;
    const double localCenterY = centerY - meanY;
    const double radiusSquared =
        localCenterX * localCenterX +
        localCenterY * localCenterY - solution[2];
    if (!std::isfinite(radiusSquared) ||
        radiusSquared <= fitTolerance * fitTolerance) {
        return std::nullopt;
    }
    const double radius = std::sqrt(radiusSquared);
    double maximumResidual = 0.0;
    double sweep = 0.0;
    int directionSign = 0;
    double previousAngle = std::atan2(
        points.front()[1] - centerY,
        points.front()[0] - centerX);
    for (std::size_t index = 0u; index < points.size(); ++index) {
        const double dx = points[index][0] - centerX;
        const double dy = points[index][1] - centerY;
        maximumResidual = std::max(
            maximumResidual,
            std::abs(std::hypot(dx, dy) - radius));
        if (index == 0u)
            continue;
        const double angle = std::atan2(dy, dx);
        const double delta = normalizedAngle(angle - previousAngle);
        if (std::abs(delta) <= 1.0e-8 ||
            std::abs(delta) > kMaximumRecoveredArcStep) {
            return std::nullopt;
        }
        const int sign = delta > 0.0 ? 1 : -1;
        if (directionSign == 0)
            directionSign = sign;
        else if (directionSign != sign)
            return std::nullopt;
        sweep += std::abs(delta);
        previousAngle = angle;
    }
    if (maximumResidual > fitTolerance ||
        sweep < kMinimumRecoveredArcSweep ||
        sweep > 2.0 * std::numbers::pi + 1.0e-6) {
        return std::nullopt;
    }

    RecoveredCircleFit fit;
    fit.center = origin;
    fit.center.Translate(
        basisU.Multiplied(centerX) +
        basisV.Multiplied(centerY));
    fit.radius = radius;
    fit.sweepRadians = sweep;
    fit.direction = directionSign > 0
        ? ToolpathArcDirection::Counterclockwise
        : ToolpathArcDirection::Clockwise;
    return fit;
}

void recoverCircularSourceMoves(
    const std::vector<ToolpathMove>& moves,
    const gp_Vec& fixedAxis,
    double fitTolerance,
    double geometricTolerance,
    std::size_t firstSyntheticPrimitiveIndex,
    std::vector<SourceMoveAnalyticInfo>& result)
{
    if (moves.size() < kMinimumRecoveredArcMoves ||
        moves.size() != result.size()) {
        return;
    }
    gp_Vec axis = fixedAxis;
    axis.Normalize();
    std::size_t syntheticPrimitiveIndex =
        firstSyntheticPrimitiveIndex;
    std::size_t begin = 0u;
    while (begin + kMinimumRecoveredArcMoves <= moves.size()) {
        if (!planarMove(moves[begin], axis, geometricTolerance) ||
            result[begin].sweep.kind ==
                PocketCamAnalyticSweepKind::CircularArc) {
            ++begin;
            continue;
        }
        std::size_t contextEnd = begin + 1u;
        while (contextEnd < moves.size() &&
               planarMove(
                   moves[contextEnd], axis,
                   geometricTolerance) &&
               result[contextEnd].sweep.kind !=
                   PocketCamAnalyticSweepKind::CircularArc &&
               sameMoveContext(moves[begin], moves[contextEnd]) &&
               moves[contextEnd - 1u].to.Distance(
                   moves[contextEnd].from) <=
                   std::max(fitTolerance, geometricTolerance)) {
            ++contextEnd;
        }
        if (contextEnd < begin + kMinimumRecoveredArcMoves) {
            begin = contextEnd;
            continue;
        }

        const double firstTurn = planarSignedTurn(
            moves[begin].from, moves[begin].to,
            moves[begin + 1u].to, axis,
            geometricTolerance);
        constexpr double straightTurnTolerance =
            0.25 * std::numbers::pi / 180.0;
        if (std::abs(firstTurn) <= straightTurnTolerance) {
            ++begin;
            continue;
        }
        const int turnSign = firstTurn > 0.0 ? 1 : -1;
        std::size_t curvedEnd = begin + 2u;
        while (curvedEnd < contextEnd) {
            const double turn = planarSignedTurn(
                moves[curvedEnd - 1u].from,
                moves[curvedEnd - 1u].to,
                moves[curvedEnd].to, axis,
                geometricTolerance);
            if (std::abs(turn) <= straightTurnTolerance ||
                (turn > 0.0 ? 1 : -1) != turnSign) {
                break;
            }
            ++curvedEnd;
        }

        const std::size_t maximumEnd = std::min(
            curvedEnd,
            begin + kMaximumRecoveredArcProbeMoves);
        std::size_t probeEnd = maximumEnd;
        std::size_t failedEnd = maximumEnd + 1u;
        std::optional<RecoveredCircleFit> fit;
        while (probeEnd >= begin + kMinimumRecoveredArcMoves) {
            fit = fitRecoveredCircle(
                moves, begin, probeEnd, axis,
                fitTolerance, geometricTolerance);
            if (fit)
                break;
            failedEnd = probeEnd;
            const std::size_t length = probeEnd - begin;
            if (length == kMinimumRecoveredArcMoves)
                break;
            probeEnd = begin + std::max(
                kMinimumRecoveredArcMoves, length / 2u);
        }
        if (!fit) {
            ++begin;
            continue;
        }

        // 二分回扩到首个失败窗口之前，尽量让显示简化后的 motion 仍完整落在同一圆弧中。
        std::size_t acceptedEnd = probeEnd;
        std::size_t upper = failedEnd == maximumEnd + 1u
            ? maximumEnd : failedEnd - 1u;
        while (acceptedEnd < upper) {
            const std::size_t candidate =
                acceptedEnd + (upper - acceptedEnd + 1u) / 2u;
            auto candidateFit = fitRecoveredCircle(
                moves, begin, candidate, axis,
                fitTolerance, geometricTolerance);
            if (candidateFit) {
                acceptedEnd = candidate;
                fit = std::move(candidateFit);
            } else {
                upper = candidate - 1u;
            }
        }

        for (std::size_t index = begin;
             index < acceptedEnd; ++index) {
            const double segmentSweep = directedArcSweep(
                moves[index].from, moves[index].to,
                fit->center, axis, fit->direction,
                geometricTolerance);
            if (segmentSweep <= geometricTolerance)
                continue;
            PocketCamAnalyticSweep& sweep = result[index].sweep;
            sweep.kind = PocketCamAnalyticSweepKind::CircularArc;
            sweep.center = fit->center;
            sweep.axis = axis;
            sweep.direction = fit->direction;
            sweep.sweepRadians = segmentSweep;
            sweep.sourcePrimitiveIndex = syntheticPrimitiveIndex;
            sweep.recoveredFromLinearMoves = true;
        }
        ++syntheticPrimitiveIndex;
        begin = acceptedEnd;
    }
}

double directedArcSweep(
    const gp_Pnt& from,
    const gp_Pnt& to,
    const gp_Pnt& center,
    const gp_Vec& sourceAxis,
    ToolpathArcDirection direction,
    double tolerance)
{
    gp_Vec first(center, from);
    gp_Vec second(center, to);
    gp_Vec axis = sourceAxis;
    if (!finiteVector(first) || !finiteVector(second) ||
        !finiteVector(axis) ||
        first.Magnitude() <= tolerance ||
        second.Magnitude() <= tolerance ||
        axis.Magnitude() <= tolerance) {
        return 0.0;
    }
    first.Normalize();
    second.Normalize();
    axis.Normalize();
    const double signedAngle = std::atan2(
        axis.Dot(first.Crossed(second)),
        std::clamp(first.Dot(second), -1.0, 1.0));
    const double orientation =
        direction == ToolpathArcDirection::Counterclockwise
            ? 1.0 : -1.0;
    const double sweep = orientation * signedAngle;

    // The input here is one controller-independent fallback chord, not an
    // unsplit native arc.  If its minor signed angle disagrees with the arc
    // direction, treating it as 2*pi-|angle| turns a short source move into
    // an almost complete circle.  That display-only reinterpretation makes
    // OpenGL cut far outside the authoritative OCCT/NC polyline.  Reject the
    // analytic cache entry instead; the original linear move remains the
    // exact and safe fallback.
    if (sweep < -tolerance)
        return 0.0;
    return std::max(0.0, sweep);
}

std::vector<SourceMoveAnalyticInfo> sourceAnalyticInfo(
    const PocketToolpathOperation& operation,
    const gp_Vec& fixedAxis,
    double tolerance,
    double recoveredArcFitTolerance)
{
    std::vector<SourceMoveAnalyticInfo> result(
        operation.clearing.moves.size());

    std::size_t moveOffset = 0u;
    bool semanticMappingValid = true;
    for (std::size_t primitiveIndex = 0u;
         primitiveIndex < operation.clearing.primitives.size();
         ++primitiveIndex) {
        const ToolpathPrimitive& primitive =
            operation.clearing.primitives[primitiveIndex];
        const std::size_t expansionCount =
            primitive.type == ToolpathPrimitiveType::Linear &&
                    primitive.fallbackMoves.empty()
                ? 1u
                : primitive.fallbackMoves.size();
        if (expansionCount == 0u ||
            moveOffset + expansionCount > result.size()) {
            semanticMappingValid = false;
            break;
        }

        if (primitive.type == ToolpathPrimitiveType::Linear) {
            for (std::size_t local = 0u;
                 local < expansionCount; ++local) {
                const ToolpathMove& move =
                    operation.clearing.moves[moveOffset + local];
                if (!planarMove(move, fixedAxis, tolerance))
                    continue;
                PocketCamAnalyticSweep& sweep =
                    result[moveOffset + local].sweep;
                sweep.kind = PocketCamAnalyticSweepKind::Linear;
                sweep.sourcePrimitiveIndex = primitiveIndex;
            }
        } else if (
            primitive.type == ToolpathPrimitiveType::CircularArc) {
            const ToolpathCircularArc& arc = primitive.arc;
            gp_Vec arcAxis = arc.axis;
            const bool validArc =
                arc.moveType == ToolpathMoveType::Cut &&
                finitePoint(arc.from) && finitePoint(arc.to) &&
                finitePoint(arc.center) && finiteVector(arcAxis) &&
                arcAxis.Magnitude() > tolerance &&
                std::isfinite(arc.sweepRadians) &&
                arc.sweepRadians > tolerance;
            if (validArc) {
                arcAxis.Normalize();
                const bool aligned =
                    std::abs(arcAxis.Dot(fixedAxis)) >=
                    1.0 - 1.0e-8;
                const bool constantDepth =
                    std::abs(gp_Vec(arc.from, arc.to).Dot(
                                 fixedAxis)) <= tolerance &&
                    std::abs(gp_Vec(arc.center, arc.from).Dot(
                                 fixedAxis)) <= tolerance;
                if (aligned && constantDepth) {
                    for (std::size_t local = 0u;
                         local < expansionCount; ++local) {
                        const ToolpathMove& move =
                            operation.clearing.moves[
                                moveOffset + local];
                        if (!planarMove(
                                move, fixedAxis, tolerance)) {
                            continue;
                        }
                        const double sweepRadians =
                            directedArcSweep(
                                move.from, move.to,
                                arc.center, arc.axis,
                                arc.direction, tolerance);
                        if (!std::isfinite(sweepRadians) ||
                            sweepRadians <= tolerance) {
                            continue;
                        }
                        PocketCamAnalyticSweep& sweep =
                            result[moveOffset + local].sweep;
                        sweep.kind =
                            PocketCamAnalyticSweepKind::CircularArc;
                        sweep.center = arc.center;
                        sweep.axis = arc.axis;
                        sweep.direction = arc.direction;
                        sweep.sweepRadians = sweepRadians;
                        sweep.sourcePrimitiveIndex = primitiveIndex;
                    }
                }
            }
        }
        moveOffset += expansionCount;
    }
    if (!semanticMappingValid || moveOffset != result.size())
        result.assign(operation.clearing.moves.size(), {});

    // Clipper 偏置输出或长螺旋可能让规划语义构建器保留折线。显示层再从权威源
    // moves 中按同一切削上下文恢复局部定半径圆弧；严格残差外仍保持线性 fallback。
    recoverCircularSourceMoves(
        operation.clearing.moves, fixedAxis,
        recoveredArcFitTolerance, tolerance,
        operation.clearing.primitives.size() + 1u,
        result);
    return result;
}

PocketCamAnalyticSweep analyticSweepForRange(
    const std::vector<SourceMoveAnalyticInfo>& source,
    std::size_t begin,
    std::size_t end)
{
    PocketCamAnalyticSweep result;
    if (begin >= end || end > source.size() ||
        !source[begin].sweep.valid()) {
        return result;
    }
    const PocketCamAnalyticSweep& first = source[begin].sweep;
    result = first;
    for (std::size_t index = begin + 1u; index < end; ++index) {
        const PocketCamAnalyticSweep& candidate = source[index].sweep;
        if (!candidate.valid() ||
            candidate.kind != first.kind ||
            candidate.sourcePrimitiveIndex !=
                first.sourcePrimitiveIndex) {
            return {};
        }
        if (result.kind ==
            PocketCamAnalyticSweepKind::CircularArc) {
            result.sweepRadians += candidate.sweepRadians;
        }
    }
    return result;
}

double pointDistanceSquared(const gp_Pnt& first, const gp_Pnt& second)
{
    const double dx = first.X() - second.X();
    const double dy = first.Y() - second.Y();
    const double dz = first.Z() - second.Z();
    return dx * dx + dy * dy + dz * dz;
}

double pointSegmentDistanceSquared(
    const gp_Pnt& point,
    const gp_Pnt& from,
    const gp_Pnt& to)
{
    const gp_Vec segment(from, to);
    const double lengthSquared = segment.SquareMagnitude();
    if (lengthSquared <= std::numeric_limits<double>::epsilon())
        return pointDistanceSquared(point, from);
    const gp_Vec relative(from, point);
    const double t = std::clamp(
        relative.Dot(segment) / lengthSquared, 0.0, 1.0);
    gp_Pnt nearest = from;
    nearest.Translate(segment.Multiplied(t));
    return pointDistanceSquared(point, nearest);
}

bool analyticPlaybackMatchesSource(
    const PocketCamAnalyticSweep& sweep,
    const std::vector<SourceMoveAnalyticInfo>& sourceInfo,
    const std::vector<ToolpathMove>& sourceMoves,
    std::size_t begin,
    std::size_t end,
    double maximumDeviation)
{
    if (sweep.kind != PocketCamAnalyticSweepKind::CircularArc ||
        sweep.recoveredFromLinearMoves ||
        begin >= end || end > sourceInfo.size() ||
        end > sourceMoves.size() ||
        !finitePoint(sweep.center) || !finiteVector(sweep.axis) ||
        sweep.axis.SquareMagnitude() <= 1.0e-16 ||
        !std::isfinite(sweep.sweepRadians) ||
        sweep.sweepRadians <= 0.0 ||
        sweep.sweepRadians > 2.0 * std::numbers::pi + 1.0e-6) {
        return false;
    }

    maximumDeviation = std::max(maximumDeviation, 1.0e-6);
    const double deviationSquared =
        maximumDeviation * maximumDeviation;
    const gp_Pnt expectedEnd = analyticPointAt(
        sourceMoves[begin].from, sweep, 1.0);
    if (expectedEnd.Distance(sourceMoves[end - 1u].to) >
        maximumDeviation) {
        return false;
    }

    gp_Vec referenceAxis = sweep.axis;
    referenceAxis.Normalize();
    constexpr std::array<double, 3> fractions{
        0.25, 0.5, 0.75};
    for (std::size_t index = begin; index < end; ++index) {
        const PocketCamAnalyticSweep& candidate =
            sourceInfo[index].sweep;
        if (candidate.kind !=
                PocketCamAnalyticSweepKind::CircularArc ||
            candidate.recoveredFromLinearMoves ||
            candidate.sourcePrimitiveIndex !=
                sweep.sourcePrimitiveIndex ||
            candidate.direction != sweep.direction ||
            !finitePoint(candidate.center) ||
            candidate.center.Distance(sweep.center) >
                maximumDeviation ||
            !finiteVector(candidate.axis) ||
            candidate.axis.SquareMagnitude() <= 1.0e-16 ||
            !std::isfinite(candidate.sweepRadians) ||
            candidate.sweepRadians <= 0.0 ||
            candidate.sweepRadians > std::numbers::pi) {
            return false;
        }
        gp_Vec candidateAxis = candidate.axis;
        candidateAxis.Normalize();
        if (candidateAxis.Dot(referenceAxis) < 1.0 - 1.0e-8)
            return false;

        const ToolpathMove& move = sourceMoves[index];
        if (analyticPointAt(move.from, candidate, 1.0).Distance(
                move.to) > maximumDeviation) {
            return false;
        }
        for (double fraction : fractions) {
            if (pointSegmentDistanceSquared(
                    analyticPointAt(
                        move.from, candidate, fraction),
                    move.from, move.to) > deviationSquared) {
                return false;
            }
        }
    }
    return true;
}

// 开放折线的 Douglas-Peucker 简化：递归找离首尾连线最远的点，
// 距离超过 tolerance 则保留并拆成两个子区间继续，否则丢弃中间点。
// 输出 kept 为保留点下标（含首尾）。
void simplifyOpenPolyline(
    const std::vector<gp_Pnt>& points,
    double tolerance,
    std::vector<std::size_t>& kept)
{
    kept.clear();
    if (points.empty())
        return;
    if (points.size() <= 2) {
        kept.resize(points.size());
        std::iota(kept.begin(), kept.end(), std::size_t{0});
        return;
    }

    const double toleranceSquared = tolerance * tolerance;
    std::vector<unsigned char> retain(points.size(), 0u);
    retain.front() = 1u;
    retain.back() = 1u;
    std::vector<std::pair<std::size_t, std::size_t>> ranges{
        {0u, points.size() - 1u}};
    while (!ranges.empty()) {
        const auto [first, last] = ranges.back();
        ranges.pop_back();
        double maximumDistance = -1.0;
        std::size_t split = first;
        for (std::size_t index = first + 1u; index < last; ++index) {
            const double distance = pointSegmentDistanceSquared(
                points[index], points[first], points[last]);
            if (distance > maximumDistance) {
                maximumDistance = distance;
                split = index;
            }
        }
        if (maximumDistance <= toleranceSquared || split == first)
            continue;
        retain[split] = 1u;
        if (split > first + 1u)
            ranges.emplace_back(first, split);
        if (last > split + 1u)
            ranges.emplace_back(split, last);
    }
    kept.reserve(points.size());
    for (std::size_t index = 0; index < retain.size(); ++index) {
        if (retain[index] != 0u)
            kept.push_back(index);
    }
}

// 把一段连续的移动序列简化为一串新移动：先把每个 move 的 from/to 拼成
// 折线整体简化，再按终点坐标回查源移动，保持每个简化 move 都能对应到
// 一个真实的源 move 前缀（sourceMoveEndCount），供 Multi-Dexel 临时帧回放。
std::vector<ToolpathMove> simplifyRun(
    const std::vector<ToolpathMove>& run,
    double tolerance)
{
    if (run.size() <= 2 || tolerance <= 0.0)
        return run;
    std::vector<gp_Pnt> points;
    points.reserve(run.size() + 1u);
    points.push_back(run.front().from);
    for (const ToolpathMove& move : run)
        points.push_back(move.to);

    const double closeTolerance = std::max(
        1.0e-9, tolerance * 1.0e-3);
    const bool closed =
        pointDistanceSquared(points.front(), points.back()) <=
        closeTolerance * closeTolerance;
    std::vector<std::size_t> kept;
    if (!closed) {
        simplifyOpenPolyline(points, tolerance, kept);
    } else {
        // 闭合折线的 Douglas-Peucker 端点相同。在离起点最远的点处将其切开,
        // 分别简化两段弧后再拼接,使圆形/偏置的腔走刀被正确简化。
        std::size_t split = 1u;
        double maximumDistance = -1.0;
        for (std::size_t index = 1u; index + 1u < points.size(); ++index) {
            const double distance =
                pointDistanceSquared(points.front(), points[index]);
            if (distance > maximumDistance) {
                maximumDistance = distance;
                split = index;
            }
        }
        if (split <= 1u || split + 1u >= points.size()) {
            kept.resize(points.size());
            std::iota(kept.begin(), kept.end(), std::size_t{0});
        } else {
            std::vector<gp_Pnt> first(
                points.begin(), points.begin() + static_cast<std::ptrdiff_t>(split + 1u));
            std::vector<gp_Pnt> second(
                points.begin() + static_cast<std::ptrdiff_t>(split), points.end());
            std::vector<std::size_t> firstKept;
            std::vector<std::size_t> secondKept;
            simplifyOpenPolyline(first, tolerance, firstKept);
            simplifyOpenPolyline(second, tolerance, secondKept);
            kept = std::move(firstKept);
            for (std::size_t index = 1u; index < secondKept.size(); ++index)
                kept.push_back(split + secondKept[index]);
        }
    }

    if (kept.size() < 2u)
        return run;
    std::vector<ToolpathMove> simplified;
    simplified.reserve(kept.size() - 1u);
    for (std::size_t index = 1u; index < kept.size(); ++index) {
        ToolpathMove move = run[
            std::min(run.size() - 1u, kept[index] - 1u)];
        move.from = points[kept[index - 1u]];
        move.to = points[kept[index]];
        simplified.push_back(std::move(move));
    }
    return simplified;
}

struct SimplifiedOperationMove {
    ToolpathMove move;
    std::size_t sourceMoveEndCount = 0u;
};

// 按「同类型、同 depth/loop/ring/intent、几何连续」把源移动切成若干
// 可独立简化的 run；退化（零长度、非有限坐标）移动先被剔除。
// 返回的每个条目都携带它消费到的源 move 结束下标。
std::vector<SimplifiedOperationMove> simplifyOperation(
    const PocketToolpathOperation& operation,
    double tolerance)
{
    std::vector<SimplifiedOperationMove> output;
    std::vector<ToolpathMove> run;
    std::vector<std::size_t> runSourceMoveEnds;
    auto flush = [&] {
        if (run.empty())
            return;
        std::vector<ToolpathMove> simplified =
            simplifyRun(run, tolerance);
        std::size_t sourceCursor = 0u;
        for (ToolpathMove& move : simplified) {
            std::size_t sourceIndex = sourceCursor;
            while (sourceIndex < run.size() &&
                   pointDistanceSquared(
                       run[sourceIndex].to, move.to) >
                       1.0e-16) {
                ++sourceIndex;
            }
            if (sourceIndex >= run.size())
                sourceIndex = run.size() - 1u;
            output.push_back({
                std::move(move),
                runSourceMoveEnds[sourceIndex]});
            sourceCursor = std::min(
                run.size(), sourceIndex + 1u);
        }
        run.clear();
        runSourceMoveEnds.clear();
    };

    const double continuityTolerance =
        std::max(1.0e-8, tolerance * 1.0e-3);
    for (std::size_t sourceMoveIndex = 0u;
         sourceMoveIndex < operation.clearing.moves.size();
         ++sourceMoveIndex) {
        const ToolpathMove& move =
            operation.clearing.moves[sourceMoveIndex];
        if (!finitePoint(move.from) || !finitePoint(move.to))
            continue;
        if (pointDistanceSquared(move.from, move.to) <=
            continuityTolerance * continuityTolerance) {
            continue;
        }
        const bool continues =
            !run.empty() &&
            run.back().type == move.type &&
            run.back().depthIndex == move.depthIndex &&
            run.back().loopSequence == move.loopSequence &&
            run.back().ring == move.ring &&
            run.back().intent == move.intent &&
            pointDistanceSquared(run.back().to, move.from) <=
                continuityTolerance * continuityTolerance;
        if (!continues)
            flush();
        run.push_back(move);
        runSourceMoveEnds.push_back(sourceMoveIndex + 1u);
    }
    flush();
    return output;
}

// 从工序推导固定刀轴：优先用显式 outwardToolAxis，退化为首个 Plunge 的
// 反方向（to→from 即刀尖指向主轴开口）。
gp_Vec operationAxis(
    const PocketToolpathOperation& operation,
    double tolerance)
{
    gp_Vec axis = operation.outwardToolAxis;
    if (finiteVector(axis) && axis.Magnitude() > tolerance) {
        axis.Normalize();
        return axis;
    }
    for (const ToolpathMove& move : operation.clearing.moves) {
        if (move.type != ToolpathMoveType::Plunge)
            continue;
        axis = gp_Vec(move.to, move.from);
        if (finiteVector(axis) && axis.Magnitude() > tolerance) {
            axis.Normalize();
            return axis;
        }
    }
    return {};
}

double maximumStockDimension(
    const TopoDS_Shape& stock,
    std::string& error)
{
    if (stock.IsNull()) {
        error = "CAM simulation requires a non-null stock";
        return 0.0;
    }
    Bnd_Box bounds;
    BRepBndLib::AddOptimal(
        stock, bounds, Standard_False, Standard_False);
    if (bounds.IsVoid()) {
        error = "CAM simulation stock has no finite bounds";
        return 0.0;
    }
    double xMin = 0.0;
    double yMin = 0.0;
    double zMin = 0.0;
    double xMax = 0.0;
    double yMax = 0.0;
    double zMax = 0.0;
    bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    const double result = std::max({
        xMax - xMin, yMax - yMin, zMax - zMin});
    if (!std::isfinite(result) || result <= Precision::Confusion()) {
        error = "CAM simulation stock bounds are degenerate";
        return 0.0;
    }
    return result;
}

// 由刀具参数构造旋转轮廓（自上而下：柄部 → 刃口 → 圆角/球头），
// 供 OpenGL 旋转体显示。缺少刃长时用毛坯跨度兜底并返回实际长度。
std::vector<std::pair<double, double>> makeToolProfile(
    const ToolDef& tool,
    const ToolpathToolContext& toolContext,
    double fallbackLength,
    double& toolLength,
    bool& flatEnd)
{
    const double radius = 0.5 * tool.diameter;
    const double cuttingLength =
        std::isfinite(tool.cuttingEdgeLength) &&
                tool.cuttingEdgeLength > 0.0
            ? tool.cuttingEdgeLength
            : fallbackLength;
    toolLength =
        std::isfinite(tool.overallLength) &&
                tool.overallLength >= cuttingLength
            ? tool.overallLength
            : cuttingLength;
    const double shankRadius =
        std::isfinite(tool.shankDiameter) &&
                tool.shankDiameter > 0.0
            ? 0.5 * tool.shankDiameter
            : radius;
    const double cornerRadius = std::clamp(
        std::isfinite(tool.cornerRadius)
            ? tool.cornerRadius : 0.0,
        0.0, radius);
    const bool conicalTip =
        std::isfinite(toolContext.tipAngle) &&
        toolContext.tipAngle > Precision::Confusion() &&
        toolContext.tipAngle < 180.0 - Precision::Confusion();
    const double tipHalfAngle = conicalTip
        ? 0.5 * toolContext.tipAngle * std::numbers::pi / 180.0
        : 0.0;
    const double conicalTipLength = conicalTip
        ? std::min(
              cuttingLength,
              radius / std::max(
                  std::tan(tipHalfAngle), Precision::Confusion()))
        : 0.0;
    flatEnd = !conicalTip && cornerRadius <= Precision::Confusion();

    std::vector<std::pair<double, double>> profile;
    profile.emplace_back(shankRadius, toolLength);
    if (toolLength > cuttingLength + Precision::Confusion())
        profile.emplace_back(shankRadius, cuttingLength);
    if (std::abs(shankRadius - radius) > Precision::Confusion())
        profile.emplace_back(radius, cuttingLength);
    if (conicalTip) {
        profile.emplace_back(radius, conicalTipLength);
        profile.emplace_back(0.0, 0.0);
    } else if (cornerRadius <= Precision::Confusion()) {
        profile.emplace_back(radius, 0.0);
    } else {
        profile.emplace_back(radius, cornerRadius);
        constexpr int arcSegments = 8;
        for (int index = 1; index <= arcSegments; ++index) {
            const double angle =
                (0.5 * std::numbers::pi) *
                static_cast<double>(index) /
                static_cast<double>(arcSegments);
            profile.emplace_back(
                radius - cornerRadius +
                    cornerRadius * std::cos(angle),
                cornerRadius -
                    cornerRadius * std::sin(angle));
        }
    }
    if (profile.empty() ||
        profile.back().first > Precision::Confusion()) {
        profile.emplace_back(0.0, 0.0);
    }
    return profile;
}

} // namespace

gp_Pnt PocketCamMotion::playbackPointAt(double fraction) const
{
    fraction = std::clamp(fraction, 0.0, 1.0);
    if (analyticPlaybackTrusted &&
        analyticSweep.kind ==
            PocketCamAnalyticSweepKind::CircularArc &&
        std::isfinite(analyticSweep.sweepRadians) &&
        analyticSweep.sweepRadians > 0.0 &&
        analyticSweep.axis.SquareMagnitude() > 1.0e-16) {
        return analyticPointAt(move.from, analyticSweep, fraction);
    }
    return gp_Pnt(
        move.from.X() + (move.to.X() - move.from.X()) * fraction,
        move.from.Y() + (move.to.Y() - move.from.Y()) * fraction,
        move.from.Z() + (move.to.Z() - move.from.Z()) * fraction);
}

PocketCamSimulationSceneResult
PocketCamSimulationSceneBuilder::build(
    const TopoDS_Shape& stock,
    const std::vector<PocketToolpathOperation>& operations,
    const ToolpathParams& params,
    const PocketCamSimulationOptions& options) const
{
    // 构建步骤：① 校验毛坯并计算最大跨度 → ② 推导运动分辨率与统一刀轴 →
    // ③ 去重合并刀具表 → ④ 生成（并按需简化）显示运动 → ⑤ 为每个运动分配
    // 模拟步长区间。全程只读 operations，不修改经过验证的源刀路。
    PocketCamSimulationSceneResult result;
    PocketCamSimulationScene& scene = result.scene;
    if (operations.empty()) {
        result.error =
            "CAM simulation requires at least one pocket operation";
        return result;
    }
    scene.maximumStockDimension =
        maximumStockDimension(stock, result.error);
    if (!result.error.empty())
        return result;

    scene.quality = std::clamp(
        std::isfinite(options.quality) ? options.quality : 10.0,
        1.0, 10.0);
    scene.motionResolution = std::clamp(
        scene.maximumStockDimension * 0.05 / scene.quality,
        0.5, 4.0);
    double simplificationTolerance =
        std::isfinite(options.pathSimplificationTolerance) &&
                options.pathSimplificationTolerance > 0.0
            ? options.pathSimplificationTolerance
            : 0.0;
    if (simplificationTolerance > 0.0) {
        simplificationTolerance = std::clamp(
            simplificationTolerance, 1.0e-6,
            scene.motionResolution);
    }

    const double axisTolerance =
        std::max(Precision::Confusion(), 1.0e-8);
    scene.authoritativeIpwPlayback = options.authoritativeIpwPlayback;
    for (const PocketToolpathOperation& operation : operations) {
        const gp_Vec axis = operationAxis(operation, axisTolerance);
        if (axis.Magnitude() <= axisTolerance) {
            result.error =
                "CAM simulation requires a valid fixed tool axis";
            return result;
        }
        if (scene.outwardToolAxis.Magnitude() <= axisTolerance) {
            scene.outwardToolAxis = axis;
        } else if (!options.authoritativeIpwPlayback && scene.outwardToolAxis.Dot(axis) < 1.0 - 1.0e-6) {
            result.error =
                "CAM simulation requires one fixed tool axis per VIEW";
            return result;
        }
        scene.sourceMotionCount +=
            operation.clearing.moves.size();
    }

    const std::size_t motionBudget = std::clamp<std::size_t>(
        options.maxDisplayMotionCount, 1000u, 2000000u);
    // Do not first materialize a multi-million-motion OpenGL scene merely to
    // discover that it exceeds the display budget. Start the display-only
    // simplifier before building motions when the source count is already
    // known to be dense.
    if (options.allowAutomaticPathSimplification &&
        scene.sourceMotionCount > motionBudget &&
        simplificationTolerance <= 0.0) {
        simplificationTolerance =
            std::max(1.0e-6, 0.1 * scene.motionResolution);
    }

    std::vector<int> operationToolIndices;
    operationToolIndices.reserve(operations.size());
    for (const PocketToolpathOperation& operation : operations) {
        const ToolDef tool = bindPocketToolParameters(
            params, operation.tool).tool;
        if (!std::isfinite(tool.diameter) ||
            tool.diameter <= Precision::Confusion()) {
            result.error =
                "CAM simulation requires a positive operation tool diameter";
            return result;
        }

        const double fallbackLength =
            scene.maximumStockDimension + tool.diameter;
        double toolLength = 0.0;
        bool flatEnd = false;
        const std::vector<std::pair<double, double>> profile =
            makeToolProfile(
                tool, operation.tool, fallbackLength,
                toolLength, flatEnd);
        const auto existing = std::find_if(
            scene.tools.begin(), scene.tools.end(),
            [&](const PocketCamTool& candidate) {
                return candidate.toolNumber ==
                           operation.tool.toolNumber &&
                       candidate.toolId == operation.tool.toolId &&
                       candidate.toolName == operation.tool.toolName &&
                       candidate.diameter == tool.diameter &&
                       candidate.length == toolLength &&
                       candidate.profile == profile;
            });
        int toolIndex = 0;
        if (existing != scene.tools.end()) {
            toolIndex = static_cast<int>(
                std::distance(scene.tools.begin(), existing));
        } else {
            PocketCamTool visualTool;
            visualTool.toolNumber = operation.tool.toolNumber;
            visualTool.toolId = operation.tool.toolId;
            visualTool.toolName = operation.tool.toolName;
            visualTool.profile = profile;
            visualTool.diameter = tool.diameter;
            visualTool.length = toolLength;
            visualTool.flatEnd = flatEnd;
            scene.tools.push_back(std::move(visualTool));
            toolIndex = static_cast<int>(scene.tools.size() - 1u);
            if (!std::isfinite(tool.cuttingEdgeLength) ||
                tool.cuttingEdgeLength <= 0.0) {
                result.warnings.push_back(
                    "tool length was not specified; the visual simulator inferred a stock-spanning tool profile");
            }
        }
        operationToolIndices.push_back(toolIndex);
    }

    const double analyticTolerance = std::max(
        1.0e-7,
        scene.maximumStockDimension * 1.0e-10);
    const double clipperGrid = std::pow(
        10.0, -std::clamp(params.clearing.precision, 0, 8));
    const double configuredArcTolerance =
        std::isfinite(params.clearing.arcTolerance) &&
                params.clearing.arcTolerance > 0.0
            ? params.clearing.arcTolerance
            : 0.0;
    const double recoveredArcFitTolerance = std::clamp(
        std::max({
            1.0e-5, 2.0 * clipperGrid,
            configuredArcTolerance,
            0.04 * scene.motionResolution}),
        1.0e-5, 0.02);
    std::vector<std::vector<SourceMoveAnalyticInfo>>
        operationAnalyticInfo;
    operationAnalyticInfo.reserve(operations.size());
    const bool buildSourceAnalyticInfo =
        scene.sourceMotionCount <= motionBudget;
    for (const PocketToolpathOperation& operation : operations) {
        if (buildSourceAnalyticInfo) {
            operationAnalyticInfo.push_back(
                sourceAnalyticInfo(
                    operation, operationAxis(operation, axisTolerance),
                    analyticTolerance,
                    recoveredArcFitTolerance));
        } else {
            // Dense source paths are simplified into display chords. Avoid a
            // second source-sized analytic cache and its arc-recovery scan;
            // the authoritative IPW calculation and NC stream remain intact.
            operationAnalyticInfo.emplace_back();
        }
    }

    auto appendMotions = [&](double tolerance) {
        scene.motions.clear();
        for (std::size_t operationIndex = 0;
             operationIndex < operations.size(); ++operationIndex) {
            const PocketToolpathOperation& operation =
                operations[operationIndex];
            std::vector<SimplifiedOperationMove> moves =
                simplifyOperation(operation, tolerance);
            const std::vector<SourceMoveAnalyticInfo>& analyticInfo =
                operationAnalyticInfo[operationIndex];
            std::size_t previousSourceMoveEnd = 0u;
            bool startsChain = true;
            gp_Pnt previousEnd;
            bool hasPrevious = false;
            for (SimplifiedOperationMove& preparedMove : moves) {
                ToolpathMove& move = preparedMove.move;
                const double distance = move.from.Distance(move.to);
                if (!std::isfinite(distance) ||
                    distance <= Precision::Confusion()) {
                    continue;
                }
                if (hasPrevious &&
                    previousEnd.Distance(move.from) >
                        std::max(1.0e-8, tolerance * 1.0e-3)) {
                    startsChain = true;
                }
                PocketCamMotion motion;
                motion.outwardToolAxis = operationAxis(operation, axisTolerance);
                motion.move = std::move(move);
                motion.operationIndex =
                    static_cast<int>(operationIndex);
                motion.toolIndex =
                    operationToolIndices[operationIndex];
                motion.sourceMoveEndCount =
                    preparedMove.sourceMoveEndCount;
                motion.startsChain = startsChain;
                if (scene.tools[static_cast<std::size_t>(
                        motion.toolIndex)].flatEnd &&
                    !analyticInfo.empty()) {
                    motion.analyticSweep = analyticSweepForRange(
                        analyticInfo,
                        previousSourceMoveEnd,
                        motion.sourceMoveEndCount);
                    motion.analyticPlaybackTrusted =
                        analyticPlaybackMatchesSource(
                            motion.analyticSweep,
                            analyticInfo,
                            operation.clearing.moves,
                            previousSourceMoveEnd,
                            motion.sourceMoveEndCount,
                            recoveredArcFitTolerance);
                }
                motion.simulationStepCount = std::max(
                    1, static_cast<int>(
                           distance / scene.motionResolution));
                scene.motions.push_back(std::move(motion));
                previousEnd = scene.motions.back().move.to;
                hasPrevious = true;
                startsChain = false;
                previousSourceMoveEnd =
                    preparedMove.sourceMoveEndCount;
            }
        }
    };

    appendMotions(simplificationTolerance);
    if (options.allowAutomaticPathSimplification) {
        while (scene.motions.size() > motionBudget &&
               simplificationTolerance <
                   scene.motionResolution - Precision::Confusion()) {
            simplificationTolerance = std::min(
                scene.motionResolution,
                simplificationTolerance * 2.0);
            appendMotions(simplificationTolerance);
        }
    }
    if (scene.motions.empty()) {
        result.error =
            "CAM simulation contains no finite non-zero motions";
        return result;
    }
    if (scene.motions.size() > motionBudget) {
        result.error =
            "CAM simulation display motion budget was exceeded";
        return result;
    }

    scene.pathSimplified =
        scene.motions.size() < scene.sourceMotionCount;
    scene.analyticLinearMotionCount = 0u;
    scene.analyticArcMotionCount = 0u;
    scene.trustedAnalyticArcMotionCount = 0u;
    scene.recoveredAnalyticArcMotionCount = 0u;
    scene.analyticFallbackMotionCount = 0u;
    for (const PocketCamMotion& motion : scene.motions) {
        if (motion.analyticSweep.kind ==
            PocketCamAnalyticSweepKind::Linear) {
            ++scene.analyticLinearMotionCount;
        } else if (motion.analyticSweep.kind ==
                   PocketCamAnalyticSweepKind::CircularArc) {
            ++scene.analyticArcMotionCount;
            if (motion.analyticPlaybackTrusted)
                ++scene.trustedAnalyticArcMotionCount;
            if (motion.analyticSweep.recoveredFromLinearMoves)
                ++scene.recoveredAnalyticArcMotionCount;
        } else if (
            motion.move.type == ToolpathMoveType::Cut &&
            scene.tools[static_cast<std::size_t>(
                motion.toolIndex)].flatEnd &&
            planarMove(
                motion.move, motion.outwardToolAxis,
                analyticTolerance)) {
            ++scene.analyticFallbackMotionCount;
        }
    }
    if (scene.pathSimplified) {
        result.warnings.push_back(
            "the OpenGL display path was simplified within " +
            std::to_string(simplificationTolerance) +
            " mm; validation and NC export still use the original path");
    }

    std::size_t nextStep = 0u;
    for (PocketCamMotion& motion : scene.motions) {
        motion.firstSimulationStep = nextStep;
        nextStep += static_cast<std::size_t>(
            motion.simulationStepCount);
    }
    scene.totalSimulationSteps = nextStep;
    return result;
}

} // namespace oap::toolpath

// Fixed-axis frames, geometry keys and axial slab partitioning.
#include "toolpath/pocket/simulation/fixedaxis/AnalyticFixedAxisLayeredBRepBuilder.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisTypes.h"
#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisGeometry.h"

namespace oap::toolpath::analytic_fixed_axis_detail {

bool cancelled(const std::function<bool()>& requested)
{
    return requested && requested();
}

double elapsedMilliseconds(const Clock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(
               Clock::now() - start)
        .count();
}

bool makeFrame(
    const gp_Vec& sourceAxis,
    double tolerance,
    FixedAxisFrame& frame)
{
    if (!std::isfinite(sourceAxis.X()) ||
        !std::isfinite(sourceAxis.Y()) ||
        !std::isfinite(sourceAxis.Z()) ||
        sourceAxis.Magnitude() <= tolerance) {
        return false;
    }
    const gp_Ax3 coordinateSystem(
        gp_Pnt(0.0, 0.0, 0.0),
        gp_Dir(sourceAxis));
    frame.origin = coordinateSystem.Location();
    frame.u = coordinateSystem.XDirection();
    frame.v = coordinateSystem.YDirection();
    frame.w = coordinateSystem.Direction();
    return true;
}

bool stockAxialBounds(
    const TopoDS_Shape& stock,
    const FixedAxisFrame& frame,
    double& minimum,
    double& maximum)
{
    Bnd_Box bounds;
    bounds.SetGap(0.0);
    BRepBndLib::Add(stock, bounds);
    if (bounds.IsVoid() || bounds.IsWhole())
        return false;
    double xMin = 0.0;
    double yMin = 0.0;
    double zMin = 0.0;
    double xMax = 0.0;
    double yMax = 0.0;
    double zMax = 0.0;
    bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    minimum = std::numeric_limits<double>::infinity();
    maximum = -std::numeric_limits<double>::infinity();
    for (int corner = 0; corner < 8; ++corner) {
        const gp_Pnt point(
            (corner & 1) != 0 ? xMax : xMin,
            (corner & 2) != 0 ? yMax : yMin,
            (corner & 4) != 0 ? zMax : zMin);
        const double axial = frame.axial(point);
        minimum = std::min(minimum, axial);
        maximum = std::max(maximum, axial);
    }
    return std::isfinite(minimum) &&
           std::isfinite(maximum) &&
           maximum > minimum;
}

std::vector<double> uniqueBoundaries(
    const std::vector<AnalyticSweepRecord>& records,
    double tolerance)
{
    std::vector<double> boundaries;
    boundaries.reserve(records.size() * 2u);
    for (const AnalyticSweepRecord& record : records) {
        boundaries.push_back(record.bottom);
        boundaries.push_back(record.top);
    }
    std::sort(boundaries.begin(), boundaries.end());
    boundaries.erase(
        std::unique(
            boundaries.begin(), boundaries.end(),
            [tolerance](double first, double second) {
                return std::abs(first - second) <= tolerance;
            }),
        boundaries.end());
    return boundaries;
}

std::string primitiveGeometryKey(
    const ToolpathPrimitive& primitive,
    const FixedAxisFrame& frame)
{
    std::ostringstream output;
    output.precision(17);
    if (primitive.type == ToolpathPrimitiveType::Linear) {
        auto from = frame.project(primitive.linear.from);
        auto to = frame.project(primitive.linear.to);
        if (to < from)
            std::swap(from, to);
        output << "L|" << from.first << '|' << from.second
               << '|' << to.first << '|' << to.second;
        return output.str();
    }
    const auto center = frame.project(primitive.arc.center);
    const auto from = frame.project(primitive.arc.from);
    output << "A|" << center.first << '|' << center.second
           << '|' << from.first << '|' << from.second
           << '|' << primitive.arc.sweepRadians << '|'
           << static_cast<int>(primitive.arc.direction) << '|'
           << primitive.arc.axis.X() << '|'
           << primitive.arc.axis.Y() << '|'
           << primitive.arc.axis.Z();
    return output.str();
}

std::string projectedCoverageKey(
    const ToolpathPrimitive& primitive,
    const FixedAxisFrame& frame)
{
    if (primitive.type != ToolpathPrimitiveType::CircularArc ||
        std::abs(
            primitive.arc.sweepRadians -
            2.0 * std::acos(-1.0)) > 1.0e-10) {
        return primitiveGeometryKey(primitive, frame);
    }

    // 完整投影圆无论其起始相位、方向或轴向圆弧法线符号如何,都覆盖同一条
    // 中心线。将其规范化后,恒定深度底面圆即可证明对应的螺旋进刀已完全
    // 被包含在内。
    const auto center = frame.project(primitive.arc.center);
    const auto from = frame.project(primitive.arc.from);
    std::ostringstream output;
    output.precision(17);
    output << "C|" << center.first << '|' << center.second
           << '|' << std::hypot(
                         from.first - center.first,
                         from.second - center.second);
    return output.str();
}

gp_Pnt interpolateAtAxial(
    const gp_Pnt& from,
    const gp_Pnt& to,
    double fromW,
    double toW,
    double axial)
{
    const double denominator = toW - fromW;
    const double fraction =
        std::abs(denominator) <=
                std::numeric_limits<double>::epsilon()
            ? 0.0
            : std::clamp(
                  (axial - fromW) / denominator,
                  0.0, 1.0);
    gp_Pnt point = from;
    point.Translate(
        gp_Vec(from, to).Multiplied(fraction));
    return point;
}

AnalyticLinearRampGroup& findOrCreateLinearRampGroup(
    std::vector<AnalyticLinearRampGroup>& groups,
    const ToolpathMove& move)
{
    const auto existing = std::find_if(
        groups.begin(), groups.end(),
        [&](const AnalyticLinearRampGroup& group) {
            return group.loopSequence == move.loopSequence &&
                   group.depthIndex == move.depthIndex &&
                   group.ring == move.ring;
        });
    if (existing != groups.end())
        return *existing;
    AnalyticLinearRampGroup group;
    group.loopSequence = move.loopSequence;
    group.depthIndex = move.depthIndex;
    group.ring = move.ring;
    groups.push_back(std::move(group));
    return groups.back();
}

AnalyticHelicalRampGroup& findOrCreateHelicalRampGroup(
    std::vector<AnalyticHelicalRampGroup>& groups,
    const ToolpathPrimitiveContext& context)
{
    const auto existing = std::find_if(
        groups.begin(), groups.end(),
        [&](const AnalyticHelicalRampGroup& group) {
            return group.loopSequence == context.loopSequence &&
                   group.depthIndex == context.depthIndex &&
                   group.ring == context.ring;
        });
    if (existing != groups.end())
        return *existing;
    AnalyticHelicalRampGroup group;
    group.loopSequence = context.loopSequence;
    group.depthIndex = context.depthIndex;
    group.ring = context.ring;
    groups.push_back(std::move(group));
    return groups.back();
}

std::pair<double, double> rotated(
    const std::pair<double, double>& vector,
    double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {
        vector.first * cosine - vector.second * sine,
        vector.first * sine + vector.second * cosine};
}

bool makePlanarHelicalSlice(
    const ToolpathPrimitive& primitive,
    const FixedAxisFrame& frame,
    double startFraction,
    double endFraction,
    double tolerance,
    ToolpathPrimitive& slice)
{
    if (primitive.type != ToolpathPrimitiveType::CircularArc ||
        startFraction < 0.0 || endFraction > 1.0 ||
        endFraction - startFraction <=
            std::numeric_limits<double>::epsilon()) {
        return false;
    }
    const ToolpathCircularArc& arc = primitive.arc;
    gp_Vec axis = arc.axis;
    if (axis.SquareMagnitude() <= tolerance * tolerance)
        return false;
    axis.Normalize();
    const double alignment = axis.Dot(gp_Vec(frame.w));
    if (std::abs(alignment) < 1.0 - 1.0e-8)
        return false;

    const auto center = frame.project(arc.center);
    const auto sourceStart = frame.project(arc.from);
    const std::pair<double, double> radial{
        sourceStart.first - center.first,
        sourceStart.second - center.second};
    if (std::hypot(radial.first, radial.second) <= tolerance)
        return false;

    double direction =
        arc.direction == ToolpathArcDirection::Counterclockwise
            ? 1.0 : -1.0;
    if (alignment < 0.0)
        direction = -direction;
    const auto startRadial = rotated(
        radial,
        direction * arc.sweepRadians * startFraction);
    const auto endRadial = rotated(
        radial,
        direction * arc.sweepRadians * endFraction);

    slice = primitive;
    slice.arc.from = frame.point(
        {center.first + startRadial.first,
         center.second + startRadial.second});
    slice.arc.to = frame.point(
        {center.first + endRadial.first,
         center.second + endRadial.second});
    slice.arc.center = frame.point(center);
    slice.arc.sweepRadians =
        arc.sweepRadians * (endFraction - startFraction);
    slice.fallbackMoves.clear();
    return true;
}

bool projectedSegmentContains(
    const AnalyticPlanarCut& cut,
    const std::pair<double, double>& from,
    const std::pair<double, double>& to,
    double tolerance)
{
    const double dx = cut.to.first - cut.from.first;
    const double dy = cut.to.second - cut.from.second;
    const double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= tolerance * tolerance)
        return false;
    const double length = std::sqrt(lengthSquared);
    const double parameterTolerance = tolerance / length;
    const auto containsPoint = [&](const auto& point) {
        const double px = point.first - cut.from.first;
        const double py = point.second - cut.from.second;
        const double lineDistance =
            std::abs(dx * py - dy * px) / length;
        const double parameter = (px * dx + py * dy) / lengthSquared;
        return lineDistance <= tolerance &&
               parameter >= -parameterTolerance &&
               parameter <= 1.0 + parameterTolerance;
    };
    return containsPoint(from) && containsPoint(to);
}

bool intervalCovered(
    std::vector<std::pair<double, double>> intervals,
    double requiredBottom,
    double requiredTop,
    double tolerance)
{
    if (requiredTop - requiredBottom <= tolerance)
        return true;
    std::sort(intervals.begin(), intervals.end());
    double coveredTop = requiredBottom;
    for (const auto& interval : intervals) {
        if (interval.second <= coveredTop + tolerance)
            continue;
        if (interval.first > coveredTop + tolerance)
            return false;
        coveredTop = std::max(coveredTop, interval.second);
        if (coveredTop >= requiredTop - tolerance)
            return true;
    }
    return false;
}

std::vector<AnalyticSlab> makeSlabs(
    const std::vector<AnalyticSweepRecord>& records,
    double tolerance)
{
    const std::vector<double> boundaries =
        uniqueBoundaries(records, tolerance);
    std::vector<AnalyticSlab> slabs;
    if (boundaries.size() < 2u)
        return slabs;
    for (std::size_t index = 0;
         index + 1u < boundaries.size(); ++index) {
        const double bottom = boundaries[index];
        const double top = boundaries[index + 1u];
        if (top - bottom <= tolerance)
            continue;
        std::vector<std::size_t> active;
        for (std::size_t record = 0;
             record < records.size(); ++record) {
            if (records[record].bottom < top - tolerance &&
                records[record].top > bottom + tolerance) {
                active.push_back(record);
            }
        }
        if (active.empty())
            continue;
        std::vector<std::string> signature;
        signature.reserve(active.size());
        for (const std::size_t record : active)
            signature.push_back(records[record].geometryKey);
        std::sort(signature.begin(), signature.end());
        signature.erase(
            std::unique(signature.begin(), signature.end()),
            signature.end());
        if (!slabs.empty() &&
            std::abs(slabs.back().top - bottom) <= tolerance &&
            slabs.back().geometrySignature == signature) {
            slabs.back().top = top;
        } else {
            slabs.push_back(
                {bottom, top, std::move(active),
                 std::move(signature)});
        }
    }
    return slabs;
}

} // namespace oap::toolpath::analytic_fixed_axis_detail

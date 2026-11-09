// Independently compiled private MultiDexelPrimitives implementation.
#include "toolpath/pocket/simulation/detail/MultiDexelPrimitives.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <ratio>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace oap::toolpath::multi_dexel_detail {

double elapsedMilliseconds(const SteadyClock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(
               SteadyClock::now() - start)
        .count();
}

double resolutionForCellBudget(
    double uSpan,
    double vSpan,
    int maximumCellCount)
{
    // sqrt(area / budget) 忽略了两个轴上的 ceil，接近大预算时可能多出
    // 数千个单元并让本来合法的精细档初始化失败。二分求满足
    // ceil(u/r) * ceil(v/r) <= budget 的最细分辨率，随后向正方向取相邻
    // 浮点数，避免恰落在整数商上时因舍入再次越界。
    auto fits = [&](double resolution) {
        if (!std::isfinite(resolution) ||
            resolution <= 0.0) {
            return false;
        }
        const double nx = std::ceil(uSpan / resolution);
        const double ny = std::ceil(vSpan / resolution);
        return nx >= 1.0 && ny >= 1.0 &&
               nx * ny <=
                   static_cast<double>(maximumCellCount);
    };
    double lower = 0.0;
    double upper = std::max(uSpan, vSpan);
    for (int iteration = 0; iteration < 80; ++iteration) {
        const double middle =
            0.5 * (lower + upper);
        if (fits(middle))
            upper = middle;
        else
            lower = middle;
    }
    return std::nextafter(
        upper,
        std::numeric_limits<double>::infinity());
}

double dot(const LocalPoint& left, const LocalPoint& right)
{
    return left.u * right.u +
           left.v * right.v +
           left.w * right.w;
}

LocalPoint difference(
    const LocalPoint& to,
    const LocalPoint& from)
{
    return {
        to.u - from.u,
        to.v - from.v,
        to.w - from.w};
}

bool finitePoint(const gp_Pnt& point)
{
    return std::isfinite(point.X()) &&
           std::isfinite(point.Y()) &&
           std::isfinite(point.Z());
}

bool cuttingMove(const ToolpathMove& move)
{
    return move.type == ToolpathMoveType::Cut ||
           move.type == ToolpathMoveType::Plunge;
}

gp_Vec normalizedAxis(
    const gp_Vec& source,
    double tolerance)
{
    gp_Vec result = source;
    if (!std::isfinite(result.X()) ||
        !std::isfinite(result.Y()) ||
        !std::isfinite(result.Z()) ||
        result.Magnitude() <= tolerance) {
        return {};
    }
    result.Normalize();
    return result;
}

double exactShapeVolume(const TopoDS_Shape& shape)
{
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties);
    return std::abs(properties.Mass());
}

gp_Pnt localToWorld(
    const gp_Pnt& origin,
    const gp_Vec& uAxis,
    const gp_Vec& vAxis,
    const gp_Vec& wAxis,
    double u,
    double v,
    double w)
{
    gp_Pnt result = origin;
    result.Translate(uAxis.Multiplied(u));
    result.Translate(vAxis.Multiplied(v));
    result.Translate(wAxis.Multiplied(w));
    return result;
}

} // namespace oap::toolpath::multi_dexel_detail

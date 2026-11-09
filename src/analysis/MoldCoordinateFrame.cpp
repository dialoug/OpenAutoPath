// 从模具几何范围构造稳定的局部坐标系，并提供世界/局部坐标转换。
// 属于 oap_analysis 的"模具坐标系"部分;坐标轴分配与方向选择均保证确定性,
// 同一几何重复计算得到完全相同的结果。
#include "MoldCoordinateFrame.h"

#include <algorithm>
#include <array>

#include <gp_Vec.hxx>

namespace oap {
namespace {

struct SpanAxis {
    int worldAxis = 0;
    double span = 0.0;
};

double coordinate(const gp_Pnt& point, int axis)
{
    if (axis == 0) return point.X();
    if (axis == 1) return point.Y();
    return point.Z();
}

gp_Dir inwardDirection(int worldAxis, bool originAtMinimum)
{
    const double sign = originAtMinimum ? 1.0 : -1.0;
    if (worldAxis == 0) return gp_Dir(sign, 0.0, 0.0);
    if (worldAxis == 1) return gp_Dir(0.0, sign, 0.0);
    return gp_Dir(0.0, 0.0, sign);
}

} // namespace

gp_Pnt MoldCoordinateFrame::worldToLocal(const gp_Pnt& world) const
{
    const gp_Vec delta(origin, world);
    return gp_Pnt(delta.Dot(gp_Vec(xAxis)),
                  delta.Dot(gp_Vec(yAxis)),
                  delta.Dot(gp_Vec(zAxis)));
}

gp_Pnt MoldCoordinateFrame::localToWorld(const gp_Pnt& local) const
{
    gp_Vec offset = gp_Vec(xAxis).Multiplied(local.X());
    offset += gp_Vec(yAxis).Multiplied(local.Y());
    offset += gp_Vec(zAxis).Multiplied(local.Z());
    return origin.Translated(offset);
}

std::array<gp_Pnt, 4> MoldCoordinateFrame::planarCorners() const
{
    return {gp_Pnt(0.0, 0.0, 0.0),
            gp_Pnt(0.0, width, 0.0),
            gp_Pnt(length, 0.0, 0.0),
            gp_Pnt(length, width, 0.0)};
}

MoldCoordinateFrame MoldCoordinateFrame::fromModel(const Model& model)
{
    Bbox bounds;
    for (const Unit& unit : model.units)
        bounds.merge(unit.bbox);
    return fromBounds(bounds);
}

// 从包围盒构建模具坐标系：按跨度排序确定局部轴（长/宽/厚），取最近角点为原点。
MoldCoordinateFrame MoldCoordinateFrame::fromBounds(const Bbox& bounds)
{
    MoldCoordinateFrame frame;
    if (!bounds.valid())
        return frame;

    const std::array<double, 3> minimum{bounds.xmin, bounds.ymin, bounds.zmin};
    std::array<SpanAxis, 3> axes{{
        {0, bounds.xmax - bounds.xmin},
        {1, bounds.ymax - bounds.ymin},
        {2, bounds.zmax - bounds.zmin}
    }};
    // 将三个世界轴按跨度从大到小排序:最长 -> 局部 x(length),次长 -> 局部 y(width),
    // 最短 -> 局部 z(thickness);跨度相等时按世界轴编号保序,保证结果确定。
    std::stable_sort(axes.begin(), axes.end(), [](const SpanAxis& a, const SpanAxis& b) {
        if (a.span != b.span)
            return a.span > b.span;
        return a.worldAxis < b.worldAxis;
    });
    if (!(axes[0].span > 0.0) || !(axes[1].span > 0.0) || !(axes[2].span > 0.0))
        return frame;

    gp_Pnt nearest;
    double nearestSquared = 0.0;
    bool hasNearest = false;
    // 从最小坐标角开始枚举，在等距时保持确定性。
    for (int xi = 0; xi < 2; ++xi) {
        for (int yi = 0; yi < 2; ++yi) {
            for (int zi = 0; zi < 2; ++zi) {
                const gp_Pnt candidate(xi == 0 ? bounds.xmin : bounds.xmax,
                                       yi == 0 ? bounds.ymin : bounds.ymax,
                                       zi == 0 ? bounds.zmin : bounds.zmax);
                const double squared = candidate.X() * candidate.X() +
                                       candidate.Y() * candidate.Y() +
                                       candidate.Z() * candidate.Z();
                if (!hasNearest || squared < nearestSquared) {
                    nearest = candidate;
                    nearestSquared = squared;
                    hasNearest = true;
                }
            }
        }
    }

    // 取最靠近世界原点的包围盒角点作为局部原点,使局部坐标尽量取正值;
    // x/y 轴均指向模具内部,再由叉积定出右手系 z 轴。
    const int xIndex = axes[0].worldAxis;
    const int yIndex = axes[1].worldAxis;
    const bool xAtMinimum = coordinate(nearest, xIndex) == minimum[xIndex];
    const bool yAtMinimum = coordinate(nearest, yIndex) == minimum[yIndex];
    frame.origin = nearest;
    frame.xAxis = inwardDirection(xIndex, xAtMinimum);
    frame.yAxis = inwardDirection(yIndex, yAtMinimum);
    const gp_Vec z = gp_Vec(frame.xAxis).Crossed(gp_Vec(frame.yAxis));
    if (z.SquareMagnitude() <= 0.0)
        return MoldCoordinateFrame{};
    frame.zAxis = gp_Dir(z);
    frame.length = axes[0].span;
    frame.width = axes[1].span;
    frame.thickness = axes[2].span;
    frame.isValid = true;
    return frame;
}

} // namespace oap

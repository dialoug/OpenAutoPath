// 在 OCCT 线框与 Clipper2 平面区域之间转换，并统一环方向和几何容差。
#include "toolpath/pocket/geometry/OcctAreaAdapter.h"

#include <algorithm>
#include <cmath>

#include <BRepAdaptor_Curve.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Vec.hxx>

namespace oap::toolpath {

namespace {
// 匿名命名空间：仅供本文件内部使用的坐标判定/追加辅助函数。

bool samePoint(const gp_Pnt2d& a, const gp_Pnt2d& b, double tolerance)
{
    return a.Distance(b) <= tolerance;
}

void appendPoint(std::vector<gp_Pnt2d>& points, const gp_Pnt2d& point, double tolerance)
{
    if (points.empty() || !samePoint(points.back(), point, tolerance))
        points.push_back(point);
}

} // namespace

gp_Pnt2d OcctAreaAdapter::projectPoint(const gp_Pnt& point, const gp_Ax3& frame)
{
    // 把 3D 点投影到 frame 的 XY 平面：原点为 frame.Location，
    // 取差值向量在 X/Y 方向上的分量，忽略 Z(开口方向)分量。
    const gp_Vec delta(frame.Location(), point);
    return gp_Pnt2d(delta.Dot(gp_Vec(frame.XDirection())),
                    delta.Dot(gp_Vec(frame.YDirection())));
}

gp_Pnt OcctAreaAdapter::pointFrom2d(const gp_Pnt2d& point, const gp_Ax3& frame)
{
    // projectPoint 的逆变换：由局部 XY 坐标重建 frame 平面上的 3D 点。
    const gp_Vec x(frame.XDirection());
    const gp_Vec y(frame.YDirection());
    return gp_Pnt(frame.Location().XYZ() + (x * point.X() + y * point.Y()).XYZ());
}

Loop2d OcctAreaAdapter::wireToLoop2d(const TopoDS_Wire& wire,
                                     const gp_Ax3& frame,
                                     double deflection,
                                     double tolerance)
{
    Loop2d loop;
    loop.wire = wire;
    if (wire.IsNull())
        return loop;

    const double tol = std::max(tolerance, Precision::Confusion());
    const double defl = std::max(deflection, Precision::Confusion());

    try {
        for (BRepTools_WireExplorer ex(wire); ex.More(); ex.Next()) {
            const TopoDS_Edge edge = ex.Current();
            if (edge.IsNull())
                continue;

            BRepAdaptor_Curve curve(edge);
            GCPnts_QuasiUniformDeflection disc(curve, defl);
            if (disc.IsDone() && disc.NbPoints() >= 2) {
                // BRepAdaptor_Curve 的参数方向不包含线框中边的拓扑方向；
                // 必须按 Edge::Orientation() 反转采样，否则内环常被拼成
                // 自交折线，面积和刀路边界都会错误。
                if (edge.Orientation() == TopAbs_REVERSED) {
                    for (int i = disc.NbPoints(); i >= 1; --i)
                        appendPoint(
                            loop.points,
                            projectPoint(disc.Value(i), frame),
                            tol);
                } else {
                    for (int i = 1; i <= disc.NbPoints(); ++i)
                        appendPoint(
                            loop.points,
                            projectPoint(disc.Value(i), frame),
                            tol);
                }
            } else {
                TopoDS_Vertex first;
                TopoDS_Vertex last;
                TopExp::Vertices(edge, first, last, Standard_True);
                if (!first.IsNull())
                    appendPoint(loop.points, projectPoint(BRep_Tool::Pnt(first), frame), tol);
                if (!last.IsNull())
                    appendPoint(loop.points, projectPoint(BRep_Tool::Pnt(last), frame), tol);
            }
        }
    } catch (const Standard_Failure&) {
        loop.points.clear();
        return loop;
    }

    if (loop.points.size() >= 2 && samePoint(loop.points.front(), loop.points.back(), tol)) {
        loop.closed = true;
        loop.points.pop_back();
    }

    if (loop.points.size() >= 3) {
        loop.signedArea = signedArea(loop.points);
        if (std::abs(loop.signedArea) <= tol)
            loop.closed = false;
    } else {
        loop.points.clear();
    }

    return loop;
}

double OcctAreaAdapter::signedArea(const std::vector<gp_Pnt2d>& points)
{
    if (points.size() < 3)
        return 0.0;

    // 鞋带公式：按多边形环序累加叉积再取半，正值表示逆时针。
    double twiceArea = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
        const gp_Pnt2d& a = points[i];
        const gp_Pnt2d& b = points[(i + 1) % points.size()];
        twiceArea += a.X() * b.Y() - b.X() * a.Y();
    }
    return 0.5 * twiceArea;
}

void OcctAreaAdapter::reverseLoop(Loop2d& loop)
{
    // 反转点序即可翻转绕向（面积取负），供统一“外环 CCW、内环 CW”约定。
    std::reverse(loop.points.begin(), loop.points.end());
    loop.signedArea = -loop.signedArea;
}

} // namespace oap::toolpath

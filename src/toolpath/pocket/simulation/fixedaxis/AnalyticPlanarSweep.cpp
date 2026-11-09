// AnalyticPlanarSweep 实现（oap_toolpath_pocket 的 fixedaxis 私有目录）。
// 为点/直线/圆弧/整圆原语直接构造解析扫掠面，输出保留 Line/Circle 曲线
// 语义的 Face。参见 AnalyticPlanarSweep.h。
#include "toolpath/pocket/simulation/fixedaxis/AnalyticPlanarSweep.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_CurveType.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace oap::toolpath {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct Point2d {
    double x = 0.0;
    double y = 0.0;
};

Point2d operator+(const Point2d& first, const Point2d& second)
{
    return {first.x + second.x, first.y + second.y};
}

Point2d operator-(const Point2d& first, const Point2d& second)
{
    return {first.x - second.x, first.y - second.y};
}

Point2d operator*(const Point2d& point, double scale)
{
    return {point.x * scale, point.y * scale};
}

double magnitude(const Point2d& point)
{
    return std::hypot(point.x, point.y);
}

Point2d rotated(const Point2d& point, double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {
        point.x * cosine - point.y * sine,
        point.x * sine + point.y * cosine};
}

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

struct PlanarFrame {
    gp_Pnt origin;
    gp_Dir u;
    gp_Dir v;
    gp_Dir w;

    Point2d project(const gp_Pnt& point) const
    {
        const gp_Vec delta(origin, point);
        return {
            delta.Dot(gp_Vec(u)),
            delta.Dot(gp_Vec(v))};
    }

    double axial(const gp_Pnt& point) const
    {
        return gp_Vec(origin, point).Dot(gp_Vec(w));
    }

    gp_Pnt point(const Point2d& planar) const
    {
        gp_Pnt result = origin;
        result.Translate(
            gp_Vec(u).Multiplied(planar.x) +
            gp_Vec(v).Multiplied(planar.y));
        return result;
    }
};

TopoDS_Edge makeLineEdge(
    const PlanarFrame& frame,
    const Point2d& from,
    const Point2d& to)
{
    BRepBuilderAPI_MakeEdge builder(
        frame.point(from), frame.point(to));
    return builder.IsDone()
        ? builder.Edge() : TopoDS_Edge{};
}

TopoDS_Edge makeCircleArcEdge(
    const PlanarFrame& frame,
    const Point2d& from,
    const Point2d& middle,
    const Point2d& to)
{
    const GC_MakeArcOfCircle arc(
        frame.point(from),
        frame.point(middle),
        frame.point(to));
    if (!arc.IsDone())
        return {};
    BRepBuilderAPI_MakeEdge builder(arc.Value());
    return builder.IsDone()
        ? builder.Edge() : TopoDS_Edge{};
}

TopoDS_Wire makeWire(const std::vector<TopoDS_Edge>& edges)
{
    BRepBuilderAPI_MakeWire builder;
    for (const TopoDS_Edge& edge : edges) {
        if (edge.IsNull())
            return {};
        builder.Add(edge);
    }
    return builder.IsDone()
        ? builder.Wire() : TopoDS_Wire{};
}

TopoDS_Face makeFace(const TopoDS_Wire& wire)
{
    if (wire.IsNull())
        return {};
    BRepBuilderAPI_MakeFace builder(wire, Standard_True);
    builder.Build();
    return builder.IsDone()
        ? builder.Face() : TopoDS_Face{};
}

TopoDS_Wire makeCircleWire(
    const PlanarFrame& frame,
    const Point2d& center,
    double radius)
{
    const gp_Ax2 axis(
        frame.point(center), frame.w, frame.u);
    BRepBuilderAPI_MakeEdge edgeBuilder(
        gp_Circ(axis, radius));
    if (!edgeBuilder.IsDone())
        return {};
    BRepBuilderAPI_MakeWire wireBuilder(edgeBuilder.Edge());
    return wireBuilder.IsDone()
        ? wireBuilder.Wire() : TopoDS_Wire{};
}

TopoDS_Face makeDisk(
    const PlanarFrame& frame,
    const Point2d& center,
    double radius)
{
    return makeFace(makeCircleWire(frame, center, radius));
}

TopoDS_Face makeAnnulus(
    const PlanarFrame& frame,
    const Point2d& center,
    double innerRadius,
    double outerRadius)
{
    TopoDS_Wire outer =
        makeCircleWire(frame, center, outerRadius);
    TopoDS_Wire inner =
        makeCircleWire(frame, center, innerRadius);
    if (outer.IsNull() || inner.IsNull())
        return {};
    inner.Reverse();
    BRepBuilderAPI_MakeFace builder(outer, Standard_True);
    builder.Add(inner);
    builder.Build();
    return builder.IsDone()
        ? builder.Face() : TopoDS_Face{};
}

// 直线扫掠 = 胶囊：矩形 + 两端半圆帽，全部由 Line/Circle 边构成，
// 面积可解析计算（2*r*L + πr²）。
TopoDS_Face makeCapsule(
    const PlanarFrame& frame,
    const Point2d& from,
    const Point2d& to,
    double radius)
{
    const Point2d delta = to - from;
    const double length = magnitude(delta);
    const Point2d direction = delta * (1.0 / length);
    const Point2d normal{-direction.y, direction.x};
    const Point2d fromLow = from - normal * radius;
    const Point2d toLow = to - normal * radius;
    const Point2d toHigh = to + normal * radius;
    const Point2d fromHigh = from + normal * radius;

    return makeFace(makeWire({
        makeLineEdge(frame, fromLow, toLow),
        makeCircleArcEdge(
            frame, toLow,
            to + direction * radius,
            toHigh),
        makeLineEdge(frame, toHigh, fromHigh),
        makeCircleArcEdge(
            frame, fromHigh,
            from - direction * radius,
            fromLow)}));
}

// 圆弧扫掠的扇形主体：外弧（圆心半径 + 刀半径）与内弧（圆心半径 - 刀
// 半径）之间的环带；圆心半径小于刀半径时退化为实心扇形（内弧归并到圆心）。
TopoDS_Face makeArcSector(
    const PlanarFrame& frame,
    const Point2d& center,
    const Point2d& startDirection,
    double centreRadius,
    double cutterRadius,
    double signedSweep,
    double tolerance)
{
    const Point2d middleDirection =
        rotated(startDirection, 0.5 * signedSweep);
    const Point2d endDirection =
        rotated(startDirection, signedSweep);
    const double outerRadius = centreRadius + cutterRadius;
    const double innerRadius = std::max(
        0.0, centreRadius - cutterRadius);
    const Point2d outerStart =
        center + startDirection * outerRadius;
    const Point2d outerMiddle =
        center + middleDirection * outerRadius;
    const Point2d outerEnd =
        center + endDirection * outerRadius;

    std::vector<TopoDS_Edge> edges;
    edges.reserve(4);
    edges.push_back(makeCircleArcEdge(
        frame, outerStart, outerMiddle, outerEnd));
    if (innerRadius > tolerance) {
        const Point2d innerStart =
            center + startDirection * innerRadius;
        const Point2d innerMiddle =
            center + middleDirection * innerRadius;
        const Point2d innerEnd =
            center + endDirection * innerRadius;
        edges.push_back(makeLineEdge(
            frame, outerEnd, innerEnd));
        edges.push_back(makeCircleArcEdge(
            frame, innerEnd, innerMiddle, innerStart));
        edges.push_back(makeLineEdge(
            frame, innerStart, outerStart));
    } else {
        edges.push_back(makeLineEdge(
            frame, outerEnd, center));
        edges.push_back(makeLineEdge(
            frame, center, outerStart));
    }

    TopoDS_Wire wire = makeWire(edges);
    if (wire.IsNull())
        return {};
    if (signedSweep < 0.0)
        wire.Reverse();
    return makeFace(wire);
}

bool extractSingleFace(
    const TopoDS_Shape& shape,
    TopoDS_Face& face)
{
    int count = 0;
    for (TopExp_Explorer explorer(shape, TopAbs_FACE);
         explorer.More(); explorer.Next()) {
        face = TopoDS::Face(explorer.Current());
        ++count;
    }
    return count == 1 && !face.IsNull();
}

bool fuseFaces(
    const TopoDS_Face& first,
    const TopoDS_Face& second,
    double tolerance,
    TopoDS_Face& result)
{
    if (first.IsNull() || second.IsNull())
        return false;
    try {
        BRepAlgoAPI_Fuse fuse(first, second);
        fuse.SetFuzzyValue(tolerance);
        fuse.SetRunParallel(Standard_False);
        fuse.Build();
        if (!fuse.IsDone() || fuse.Shape().IsNull())
            return false;
        fuse.SimplifyResult(
            Standard_True, Standard_True);
        return extractSingleFace(fuse.Shape(), result);
    } catch (const Standard_Failure&) {
        return false;
    } catch (...) {
        return false;
    }
}

bool validFace(const TopoDS_Face& face)
{
    return !face.IsNull() &&
           BRepCheck_Analyzer(
               face, Standard_True,
               Standard_False)
               .IsValid();
}

bool finishResult(
    AnalyticPlanarSweepResult& result)
{
    if (!validFace(result.face)) {
        result.error =
            "analytic planar sweep produced an invalid face";
        result.face.Nullify();
        return false;
    }
    try {
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(
            result.face, properties);
        result.area = std::abs(properties.Mass());
    } catch (const Standard_Failure&) {
        result.error =
            "failed to measure analytic planar sweep area";
        result.face.Nullify();
        return false;
    } catch (...) {
        result.error =
            "failed to measure analytic planar sweep area";
        result.face.Nullify();
        return false;
    }
    if (!std::isfinite(result.area) || result.area <= 0.0) {
        result.error =
            "analytic planar sweep has no finite positive area";
        result.face.Nullify();
        return false;
    }

    for (TopExp_Explorer explorer(result.face, TopAbs_EDGE);
         explorer.More(); explorer.Next()) {
        const BRepAdaptor_Curve curve(
            TopoDS::Edge(explorer.Current()));
        if (curve.GetType() == GeomAbs_Line) {
            ++result.lineEdgeCount;
        } else if (curve.GetType() == GeomAbs_Circle) {
            ++result.circleEdgeCount;
        } else {
            ++result.unsupportedEdgeCount;
        }
    }
    if (result.unsupportedEdgeCount != 0) {
        result.error =
            "analytic planar sweep lost line/circle curve semantics";
        result.face.Nullify();
        return false;
    }
    return true;
}

AnalyticPlanarSweepResult failure(std::string error)
{
    AnalyticPlanarSweepResult result;
    result.error = std::move(error);
    return result;
}

} // namespace

// 为单个原语构造解析扫掠面。按类型分派：直线→胶囊（退化零长→圆盘）；
// 圆弧→先校验轴对齐与等深，再分整圆（环/盘）与扇形+两端圆帽，
// 并做端点一致性校验（maximumInputDeviation）。非解析/退化输入返回 error。
AnalyticPlanarSweepResult AnalyticPlanarSweep::build(
    const gp_Ax3& sourceFrame,
    const ToolpathPrimitive& primitive,
    double cutterRadius,
    double sourceTolerance,
    double inputConsistencyTolerance) const
{
    const double tolerance = std::max(
        sourceTolerance, Precision::Confusion());
    const double consistencyTolerance =
        std::isfinite(inputConsistencyTolerance) &&
                inputConsistencyTolerance > tolerance
            ? inputConsistencyTolerance
            : tolerance;
    if (!std::isfinite(cutterRadius) ||
        cutterRadius <= tolerance ||
        !std::isfinite(sourceTolerance) ||
        sourceTolerance <= 0.0) {
        return failure(
            "analytic planar sweep requires a positive radius and tolerance");
    }

    const PlanarFrame frame{
        sourceFrame.Location(),
        sourceFrame.XDirection(),
        sourceFrame.YDirection(),
        sourceFrame.Direction()};
    AnalyticPlanarSweepResult result;

    if (primitive.type == ToolpathPrimitiveType::Linear) {
        const ToolpathMove& move = primitive.linear;
        if (!finitePoint(move.from) || !finitePoint(move.to)) {
            return failure(
                "analytic linear sweep contains a non-finite point");
        }
        const Point2d from = frame.project(move.from);
        const Point2d to = frame.project(move.to);
        result.pathLength = magnitude(to - from);
        if (result.pathLength <= tolerance) {
            result.kind = AnalyticPlanarSweepKind::Point;
            result.pointFallback = true;
            result.face = makeDisk(frame, from, cutterRadius);
            result.expectedArea = kPi * cutterRadius * cutterRadius;
        } else {
            result.kind = AnalyticPlanarSweepKind::Linear;
            result.face = makeCapsule(
                frame, from, to, cutterRadius);
            result.expectedArea =
                2.0 * cutterRadius * result.pathLength +
                kPi * cutterRadius * cutterRadius;
        }
        finishResult(result);
        return result;
    }

    if (primitive.type != ToolpathPrimitiveType::CircularArc) {
        return failure(
            "analytic planar sweep supports only linear and circular primitives");
    }

    const ToolpathCircularArc& arc = primitive.arc;
    if (!finitePoint(arc.from) || !finitePoint(arc.to) ||
        !finitePoint(arc.center) || !finiteVector(arc.axis) ||
        arc.axis.SquareMagnitude() <= tolerance * tolerance ||
        !std::isfinite(arc.sweepRadians) ||
        arc.sweepRadians <= 0.0) {
        return failure(
            "analytic circular sweep contains invalid geometry");
    }
    gp_Vec axis = arc.axis;
    axis.Normalize();
    const double alignment = axis.Dot(gp_Vec(frame.w));
    if (std::abs(alignment) < 1.0 - 1.0e-8) {
        return failure(
            "analytic circular sweep axis is not aligned with the fixed axis");
    }
    if (std::abs(frame.axial(arc.from) - frame.axial(arc.to)) >
            tolerance ||
        std::abs(frame.axial(arc.from) - frame.axial(arc.center)) >
            tolerance) {
        return failure(
            "axially varying circular sweep requires layered subdivision");
    }

    const Point2d center = frame.project(arc.center);
    const Point2d start = frame.project(arc.from);
    const Point2d end = frame.project(arc.to);
    const Point2d radial = start - center;
    const double centreRadius = magnitude(radial);
    if (!std::isfinite(centreRadius) ||
        centreRadius <= tolerance) {
        return failure(
            "analytic circular sweep has a degenerate centre radius");
    }
    const double angularTolerance = std::max(
        1.0e-10, tolerance / centreRadius);
    if (arc.sweepRadians > 2.0 * kPi + angularTolerance) {
        return failure(
            "analytic circular sweep exceeds one complete turn");
    }
    const double sweep = std::min(
        arc.sweepRadians, 2.0 * kPi);
    double direction =
        arc.direction == ToolpathArcDirection::Counterclockwise
            ? 1.0 : -1.0;
    if (alignment < 0.0)
        direction = -direction;
    const Point2d startDirection = radial * (1.0 / centreRadius);
    const Point2d expectedEnd = center +
        rotated(startDirection, direction * sweep) * centreRadius;
    result.maximumInputDeviation = magnitude(expectedEnd - end);
    if (result.maximumInputDeviation >
        std::max(
            consistencyTolerance,
            1.0e-9 * centreRadius)) {
        return failure(
            "analytic circular sweep endpoint does not match its sweep: deviation=" +
            std::to_string(result.maximumInputDeviation) +
            ", tolerance=" +
            std::to_string(
                std::max(
                    consistencyTolerance,
                    1.0e-9 * centreRadius)));
    }

    result.pathLength = centreRadius * sweep;
    if (result.pathLength <= tolerance) {
        result.kind = AnalyticPlanarSweepKind::Point;
        result.pointFallback = true;
        result.face = makeDisk(frame, start, cutterRadius);
        result.expectedArea = kPi * cutterRadius * cutterRadius;
        finishResult(result);
        return result;
    }

    if (sweep >= 2.0 * kPi - angularTolerance) {
        result.kind = AnalyticPlanarSweepKind::FullCircle;
        const double outerRadius = centreRadius + cutterRadius;
        const double innerRadius = centreRadius - cutterRadius;
        if (innerRadius > tolerance) {
            result.face = makeAnnulus(
                frame, center, innerRadius, outerRadius);
            result.expectedArea = kPi *
                (outerRadius * outerRadius -
                 innerRadius * innerRadius);
        } else {
            result.face = makeDisk(
                frame, center, outerRadius);
            result.expectedArea =
                kPi * outerRadius * outerRadius;
        }
        finishResult(result);
        return result;
    }

    result.kind = AnalyticPlanarSweepKind::CircularArc;
    TopoDS_Face area = makeArcSector(
        frame, center, startDirection,
        centreRadius, cutterRadius,
        direction * sweep, tolerance);
    TopoDS_Face combined;
    if (!fuseFaces(
            area,
            makeDisk(frame, start, cutterRadius),
            tolerance, combined) ||
        !fuseFaces(
            combined,
            makeDisk(frame, expectedEnd, cutterRadius),
            tolerance, result.face)) {
        return failure(
            "failed to unite analytic arc sector and endpoint caps");
    }
    result.expectedArea =
        std::numeric_limits<double>::quiet_NaN();
    finishResult(result);
    return result;
}

} // namespace oap::toolpath

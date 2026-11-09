// Planar wire construction and closed-offset sweep records.
#include "toolpath/pocket/simulation/fixedaxis/AnalyticFixedAxisLayeredBRepBuilder.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <GProp_GProps.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisTypes.h"
#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisGeometry.h"
#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisBooleans.h"

namespace oap::toolpath::analytic_fixed_axis_detail {

bool translatedFace(
    const TopoDS_Face& source,
    const FixedAxisFrame& frame,
    double axial,
    TopoDS_Face& output)
{
    gp_Trsf translation;
    translation.SetTranslation(
        gp_Vec(frame.w).Multiplied(axial));
    BRepBuilderAPI_Transform transform(
        source, translation, Standard_True);
    transform.Build();
    if (!transform.IsDone() || transform.Shape().IsNull() ||
        transform.Shape().ShapeType() != TopAbs_FACE) {
        return false;
    }
    output = TopoDS::Face(transform.Shape());
    return true;
}

int wireCount(const TopoDS_Face& face)
{
    int count = 0;
    for (TopExp_Explorer explorer(face, TopAbs_WIRE);
         explorer.More(); explorer.Next()) {
        ++count;
    }
    return count;
}

bool pointOnSegment(
    const std::pair<double, double>& point,
    const std::pair<double, double>& first,
    const std::pair<double, double>& second,
    double tolerance)
{
    const double dx = second.first - first.first;
    const double dy = second.second - first.second;
    const double px = point.first - first.first;
    const double py = point.second - first.second;
    const double length = std::hypot(dx, dy);
    if (length <= tolerance) {
        return std::hypot(px, py) <= tolerance;
    }
    if (std::abs(dx * py - dy * px) >
        tolerance * length) {
        return false;
    }
    const double projection = px * dx + py * dy;
    return projection >= -tolerance * length &&
           projection <=
               length * length + tolerance * length;
}

bool pointInPolygon(
    const std::pair<double, double>& point,
    const std::vector<std::pair<double, double>>& polygon,
    double tolerance)
{
    bool inside = false;
    for (std::size_t firstIndex = 0,
                     secondIndex = polygon.size() - 1u;
         firstIndex < polygon.size();
         secondIndex = firstIndex++) {
        const auto& first = polygon[firstIndex];
        const auto& second = polygon[secondIndex];
        if (pointOnSegment(
                point, first, second, tolerance)) {
            return true;
        }
        const bool crosses =
            (first.second > point.second) !=
            (second.second > point.second);
        if (!crosses)
            continue;
        const double intersectionX =
            first.first +
            (point.second - first.second) *
                (second.first - first.first) /
                (second.second - first.second);
        if (intersectionX >= point.first - tolerance)
            inside = !inside;
    }
    return inside;
}

std::string canonicalPolygonKey(
    const std::vector<std::pair<double, double>>& polygon,
    double tolerance)
{
    using QuantizedPoint = std::pair<long long, long long>;
    const double quantum = std::max(tolerance, 1.0e-9);
    std::vector<QuantizedPoint> points;
    points.reserve(polygon.size());
    for (const auto& point : polygon) {
        const QuantizedPoint quantized{
            std::llround(point.first / quantum),
            std::llround(point.second / quantum)};
        if (points.empty() || points.back() != quantized)
            points.push_back(quantized);
    }
    if (points.size() > 1u &&
        points.front() == points.back()) {
        points.pop_back();
    }
    if (points.empty())
        return {};

    std::vector<QuantizedPoint> best;
    best.reserve(points.size());
    const auto consider = [&](std::size_t start,
                              bool reverse,
                              std::vector<QuantizedPoint>& output) {
        std::vector<QuantizedPoint> candidate;
        candidate.reserve(points.size());
        for (std::size_t offset = 0;
             offset < points.size(); ++offset) {
            const std::size_t index = reverse
                ? (start + points.size() - offset) % points.size()
                : (start + offset) % points.size();
            candidate.push_back(points[index]);
        }
        if (output.empty() || candidate < output)
            output = std::move(candidate);
    };
    for (std::size_t start = 0;
         start < points.size(); ++start) {
        consider(start, false, best);

        consider(start, true, best);
    }

    std::ostringstream key;
    key << 'F';
    for (const QuantizedPoint& point : best)
        key << '|' << point.first << '|' << point.second;
    return key.str();
}

bool referenceWire(
    const TopoDS_Wire& source,
    const FixedAxisFrame& frame,
    double sourceAxial,
    TopoDS_Wire& output)
{
    gp_Trsf translation;
    translation.SetTranslation(
        gp_Vec(frame.w).Multiplied(-sourceAxial));
    BRepBuilderAPI_Transform transform(
        source, translation, Standard_True);
    transform.Build();
    if (!transform.IsDone() || transform.Shape().IsNull() ||
        transform.Shape().ShapeType() != TopAbs_WIRE) {
        return false;
    }
    output = TopoDS::Wire(transform.Shape());
    return BRepCheck_Analyzer(
               output, Standard_True, Standard_False)
        .IsValid();
}

bool outwardOffsetFace(
    const TopoDS_Wire& wire,
    double radialOffset,
    double tolerance,
    TopoDS_Face& output)
{
    output.Nullify();
    if (!std::isfinite(radialOffset) ||
        radialOffset < -tolerance) {
        return false;
    }
    const auto acceptWire = [&](const TopoDS_Wire& candidate,
                                double& bestArea) {
        BRepBuilderAPI_MakeFace faceBuilder(
            candidate, Standard_True);
        faceBuilder.Build();
        if (!faceBuilder.IsDone() ||
            faceBuilder.Face().IsNull() ||
            !BRepCheck_Analyzer(
                 faceBuilder.Face(), Standard_True,
                 Standard_False)
                 .IsValid()) {
            return;
        }
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(
            faceBuilder.Face(), properties);
        const double area = std::abs(properties.Mass());
        if (std::isfinite(area) && area > bestArea) {
            bestArea = area;
            output = faceBuilder.Face();
        }
    };

    double bestArea = 0.0;
    if (radialOffset <= tolerance) {
        acceptWire(wire, bestArea);
        return !output.IsNull();
    }
    for (const double offset :
         {radialOffset, -radialOffset}) {
        try {
            BRepOffsetAPI_MakeOffset builder(
                wire, GeomAbs_Arc, Standard_False);
            builder.SetApprox(Standard_False);
            builder.Perform(offset, 0.0);
            if (!builder.IsDone() ||
                builder.Shape().IsNull()) {
                continue;
            }
            for (TopExp_Explorer explorer(
                     builder.Shape(), TopAbs_WIRE);
                 explorer.More(); explorer.Next()) {
                acceptWire(
                    TopoDS::Wire(explorer.Current()),
                    bestArea);
            }
        } catch (const Standard_Failure&) {
            continue;
        }
    }
    return !output.IsNull();
}

bool buildSemanticLoopWire(
    const PocketToolpathOperation& operation,
    const ClearingLoop& loop,
    const FixedAxisFrame& frame,
    double tolerance,
    double fitTolerance,
    double& maximumFitError,
    TopoDS_Wire& output)
{
    output.Nullify();
    if (loop.world.empty())
        return false;
    const double tip = frame.axial(loop.world.front());
    BRepBuilderAPI_MakeWire wire;
    int edgeCount = 0;
    const auto addLine = [&](const gp_Pnt& first,
                             const gp_Pnt& last) {
        if (std::abs(frame.axial(first) - tip) > tolerance ||
            std::abs(frame.axial(last) - tip) > tolerance ||
            frame.planarDistance(first, last) <= tolerance) {
            return false;
        }
        BRepBuilderAPI_MakeEdge edge(
            frame.point(frame.project(first), tip),
            frame.point(frame.project(last), tip));
        if (!edge.IsDone() || edge.Edge().IsNull())
            return false;
        wire.Add(edge.Edge());
        ++edgeCount;
        return true;
    };
    const auto addArc =
        [&](const gp_Pnt& first,
            const gp_Pnt& middle,
            const gp_Pnt& last) {
            GC_MakeArcOfCircle curve(first, middle, last);
            if (!curve.IsDone())
                return false;
            BRepBuilderAPI_MakeEdge edge(curve.Value());
            if (!edge.IsDone() || edge.Edge().IsNull())
                return false;
            wire.Add(edge.Edge());
            ++edgeCount;
            return true;
        };
    for (const ToolpathPrimitive& primitive :
         operation.clearing.primitives) {
        if (primitive.context.loopSequence != loop.sequence ||
            primitive.context.depthIndex != loop.depthIndex ||
            primitive.context.ring != loop.ring ||
            primitive.context.intent != ToolpathMoveIntent::General) {
            continue;
        }
        if (primitive.type == ToolpathPrimitiveType::Linear) {
            const ToolpathMove& move = primitive.linear;
            if (move.type != ToolpathMoveType::Cut ||
                std::abs(frame.axial(move.from) - tip) > tolerance ||
                std::abs(frame.axial(move.to) - tip) > tolerance ||
                frame.planarDistance(move.from, move.to) <= tolerance) {
                continue;
            }
            if (!addLine(move.from, move.to))
                return false;
            continue;
        }
        if (primitive.type !=
                ToolpathPrimitiveType::CircularArc ||
            primitive.arc.moveType != ToolpathMoveType::Cut) {
            continue;
        }
        const ToolpathCircularArc& arc = primitive.arc;
        if (!std::isfinite(arc.sweepRadians) ||
            arc.sweepRadians <= tolerance ||
            std::abs(frame.axial(arc.from) - tip) > tolerance ||
            std::abs(frame.axial(arc.to) - tip) > tolerance) {
            return false;
        }
        gp_Vec axis = arc.axis;
        if (axis.SquareMagnitude() <= tolerance * tolerance)
            return false;
        axis.Normalize();
        const double alignment = axis.Dot(gp_Vec(frame.w));
        if (std::abs(alignment) < 1.0 - 1.0e-8)
            return false;
        const auto center = frame.project(arc.center);
        const auto start = frame.project(arc.from);
        const std::pair<double, double> radial{
            start.first - center.first,
            start.second - center.second};
        const double radius =
            std::hypot(radial.first, radial.second);
        if (radius <= tolerance)
            return false;
        double fitError = 0.0;
        for (const ToolpathMove& move : primitive.fallbackMoves) {
            for (const gp_Pnt& point : {move.from, move.to}) {
                const auto projected = frame.project(point);
                fitError = std::max(
                    fitError,
                    std::abs(
                        std::hypot(
                            projected.first - center.first,
                            projected.second - center.second) -
                        radius));
            }
        }
        if (fitError > fitTolerance) {
            for (const ToolpathMove& move :
                 primitive.fallbackMoves) {
                if (!addLine(move.from, move.to))
                    return false;
            }
            continue;
        }
        maximumFitError = std::max(
            maximumFitError, fitError);
        double direction =
            arc.direction ==
                    ToolpathArcDirection::Counterclockwise
                ? 1.0 : -1.0;
        if (alignment < 0.0)
            direction = -direction;
        const auto pointAt = [&](double fraction) {
            const auto value = rotated(
                radial,
                direction * arc.sweepRadians * fraction);
            return frame.point(
                {center.first + value.first,
                 center.second + value.second},
                tip);
        };
        const double twoPi = 2.0 * std::acos(-1.0);
        if (arc.sweepRadians >= twoPi - 1.0e-8) {
            if (!addArc(
                    pointAt(0.0), pointAt(0.25),
                    pointAt(0.5)) ||
                !addArc(
                    pointAt(0.5), pointAt(0.75),
                    pointAt(1.0))) {
                return false;
            }
        } else if (!addArc(
                       frame.point(frame.project(arc.from), tip),
                       pointAt(0.5),
                       frame.point(frame.project(arc.to), tip))) {
            return false;
        }
    }
    if (edgeCount == 0 || !wire.IsDone() || wire.Wire().IsNull())
        return false;
    output = wire.Wire();
    return BRepCheck_Analyzer(
               output, Standard_True, Standard_False)
        .IsValid();
}

bool buildClosedOffsetFillRecords(
    const PocketToolpathOperation& operation,
    const ToolpathParams& params,
    const FixedAxisFrame& frame,
    double cuttingLength,
    double stockMinimum,
    double stockMaximum,
    double tolerance,
    double nestingTolerance,
    std::unordered_set<int>& retainedLoopSequences,
    std::vector<AnalyticSweepRecord>& fillRecords,
    double& maximumBoundaryFitError,
    std::string* rejectionReason)
{
    const auto& clearing = operation.clearing;
    const double cutterRadius = 0.5 * params.tool.diameter;
    const bool fullBall =
        std::abs(params.tool.cornerRadius - cutterRadius) <=
        tolerance;
    if (fullBall &&
        cuttingLength < cutterRadius - tolerance) {
        return false;
    }
    const double fullRadiusStartOffset =
        fullBall ? cutterRadius : 0.0;
    if (params.clearing.pattern !=
            ClearingPattern::Offset ||
        clearing.loops.empty() ||
        !std::isfinite(clearing.stepOver) ||
        clearing.stepOver <= tolerance ||
        clearing.stepOver >
            params.tool.diameter + tolerance) {
        return false;
    }

    struct FillCandidate {
        int depthIndex = 0;
        int sequence = -1;
        double tip = 0.0;
        TopoDS_Face face;
        std::vector<std::pair<double, double>> polygon;
        AnalyticSweepRecord record;
    };
    std::vector<FillCandidate> candidates;
    std::unordered_map<int, std::vector<std::size_t>>
        outerByDepth;
    for (const ClearingLoop& loop : clearing.loops) {
        if (!loop.closed || loop.isHole ||
            loop.ring < 0 || loop.sequence < 0 ||
            loop.world.size() < 3u) {
            return false;
        }
        if (loop.ring != 0)
            continue;

        BRepBuilderAPI_MakePolygon polygon;
        const double tip =
            frame.axial(loop.world.front());
        std::vector<std::pair<double, double>>
            projectedPolygon;
        projectedPolygon.reserve(loop.world.size());
        for (const gp_Pnt& point : loop.world) {
            if (std::abs(frame.axial(point) - tip) >
                tolerance) {
                return false;
            }
            const auto projected = frame.project(point);
            projectedPolygon.push_back(projected);
            polygon.Add(frame.point(projected));
        }
        polygon.Close();
        if (!polygon.IsDone())
            return false;
        BRepBuilderAPI_MakeFace faceBuilder(
            polygon.Wire(), Standard_True);
        faceBuilder.Build();
        if (!faceBuilder.IsDone() ||
            faceBuilder.Face().IsNull() ||
            !BRepCheck_Analyzer(
                 faceBuilder.Face(), Standard_True,
                 Standard_False)
                 .IsValid()) {
            return false;
        }
        TopoDS_Wire semanticWire;
        if (!buildSemanticLoopWire(
                operation, loop, frame, tolerance,
                nestingTolerance,
                maximumBoundaryFitError,
                semanticWire)) {
            return false;
        }
        // 所有闭合填充面稍后都由 translatedFace() 作为参考平面截面消费。
        // 若此处保留原始刀尖 Z,每个截面就会两次收到轴向坐标:一次来自
        // semanticWire,一次来自 translatedFace()。在相同台阶高度请求的
        // 相邻轮廓随后会在 Z 方向上被分开,产生 BRep 端点中可见的
        // 长条扭曲片体。
        TopoDS_Wire normalizedWire;
        if (!referenceWire(
                semanticWire, frame, tip,
                normalizedWire)) {
            return false;
        }
        TopoDS_Face sweptFillFace;
        if (!outwardOffsetFace(
                normalizedWire,
                0.5 * params.tool.diameter,
                tolerance, sweptFillFace)) {
            return false;
        }
        const double bottom = std::max(
            tip, stockMinimum);
        const double top = std::min(
            tip + cuttingLength, stockMaximum);
        if (top - bottom <= tolerance)
            continue;
        FillCandidate candidate;
        candidate.depthIndex = loop.depthIndex;
        candidate.sequence = loop.sequence;
        candidate.tip = tip;
        candidate.face = faceBuilder.Face();
        candidate.polygon =
            std::move(projectedPolygon);
        candidate.record = {
            bottom, top, std::move(sweptFillFace),
            canonicalPolygonKey(
                candidate.polygon, tolerance)};
        if (fullBall) {
            candidate.record.centerWire =
                std::move(normalizedWire);
        }
        candidate.record.sourceTip = tip;
        outerByDepth[candidate.depthIndex]
            .push_back(candidates.size());
        candidates.push_back(std::move(candidate));
    }
    if (candidates.empty())
        return false;

    // 当每个深度都嵌套在上一深度内部且相邻深度区间重叠时,将较深的填充
    // 足迹向毛坯顶部延伸,与有限切削刃扫掠完全等价。由此产生的重叠棱柱
    // 避免了大量共面片体分界面。
    std::vector<std::size_t> nestingOrder(candidates.size());
    for (std::size_t index = 0;
         index < candidates.size(); ++index) {
        nestingOrder[index] = index;
    }
    std::stable_sort(
        nestingOrder.begin(), nestingOrder.end(),
        [&](std::size_t first, std::size_t second) {
            return candidates[first].tip >
                   candidates[second].tip;
        });
    for (const std::size_t currentIndex : nestingOrder) {
        double coveredTo = std::min(
            stockMaximum,
            candidates[currentIndex].tip + cuttingLength);
        bool expanded = true;
        while (coveredTo < stockMaximum - tolerance && expanded) {
            expanded = false;
            for (const std::size_t shallowerIndex : nestingOrder) {
                const FillCandidate& shallower =
                    candidates[shallowerIndex];
                if (shallower.tip <=
                        candidates[currentIndex].tip + tolerance ||
                    shallower.tip + fullRadiusStartOffset >
                        coveredTo + tolerance) {
                    continue;
                }
                const bool allInside = std::all_of(
                    candidates[currentIndex].polygon.begin(),
                    candidates[currentIndex].polygon.end(),
                    [&](const auto& point) {
                        return pointInPolygon(
                            point, shallower.polygon,
                            nestingTolerance);
                    });
                if (!allInside)
                    continue;
                const double nextCoveredTo = std::min(
                    stockMaximum,
                    shallower.tip + cuttingLength);
                if (nextCoveredTo > coveredTo + tolerance) {
                    coveredTo = nextCoveredTo;
                    expanded = true;
                }
            }
        }
        if (coveredTo < stockMaximum - tolerance) {
            if (rejectionReason) {
                std::ostringstream message;
                message
                    << "nested fill coverage stops at "
                    << coveredTo << " for depth "
                    << candidates[currentIndex].depthIndex
                    << " (tip "
                    << candidates[currentIndex].tip
                    << ", stock top " << stockMaximum
                    << ')';
                *rejectionReason = message.str();
            }
            return false;
        }
    }

    // 每条被省略的中心线都必须位于保留的外环内部。Minkowski 和是单调的,
    // 因此其刀具扫掠随后被填充外环加上该环边界的精确扫掠所包含。
    for (const ClearingLoop& loop : clearing.loops) {
        if (loop.ring == 0)
            continue;
        const auto outer = outerByDepth.find(loop.depthIndex);
        if (outer == outerByDepth.end())
            return false;
        for (const gp_Pnt& point : loop.world) {
            const auto projected = frame.project(point);
            bool contained = false;
            for (const std::size_t candidateIndex :
                 outer->second) {
                if (pointInPolygon(
                        projected,
                        candidates[candidateIndex].polygon,
                        tolerance)) {
                    contained = true;
                    break;
                }
            }
            if (!contained)
                return false;
        }
    }

    const auto polygonContained =
        [&](const FillCandidate& inner,
            const FillCandidate& outer) {
            return std::all_of(
                inner.polygon.begin(), inner.polygon.end(),
                [&](const auto& point) {
                    return pointInPolygon(
                        point, outer.polygon,
                        nestingTolerance);
                });
        };
    std::vector<FillCandidate> compacted;
    compacted.reserve(candidates.size());
    for (const std::size_t candidateIndex : nestingOrder) {
        FillCandidate& candidate = candidates[candidateIndex];
        auto equivalent = std::find_if(
            compacted.rbegin(), compacted.rend(),
            [&](const FillCandidate& existing) {
                return polygonContained(candidate, existing) &&
                       polygonContained(existing, candidate);
            });
        if (equivalent == compacted.rend()) {
            compacted.push_back(std::move(candidate));
            continue;
        }
        equivalent->tip = std::min(
            equivalent->tip, candidate.tip);
        equivalent->record.bottom = std::min(
            equivalent->record.bottom,
            candidate.record.bottom);
        equivalent->record.sourceTip = std::min(
            equivalent->record.sourceTip,
            candidate.record.sourceTip);
    }
    candidates = std::move(compacted);

    retainedLoopSequences.clear();
    fillRecords.clear();
    retainedLoopSequences.reserve(candidates.size());
    fillRecords.reserve(candidates.size());
    for (FillCandidate& candidate : candidates) {
        retainedLoopSequences.insert(candidate.sequence);
        candidate.record.top = stockMaximum +
                               10.0 * tolerance;
        fillRecords.push_back(std::move(candidate.record));
    }
    return true;
}

} // namespace oap::toolpath::analytic_fixed_axis_detail

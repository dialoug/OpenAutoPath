// 孔口扩径过渡识别实现，集中处理锥面、圆角面和 B-spline 的轴向采样。
// 模块: oap_recognition（detail 层）。
#include "OpeningTransitionRecognizer.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <algorithm>
#include <cmath>
#include <limits>

namespace oap::recognition::detail {

namespace {

struct EdgeAxisRange
{
    bool valid = false;
    double minimumRadius = std::numeric_limits<double>::infinity();
    double maximumRadius = -std::numeric_limits<double>::infinity();
    double parameterAtMaximumRadius = 0.0;
    gp_Pnt pointAtMaximumRadius;
};

// 沿边曲线均匀采样 25 个点，计算相对孔轴的轴向投影与径向距离，返回最小/
// 最大半径，以及最大半径处的轴向参数与空间点。
EdgeAxisRange sampleEdgeAxisRange(const TopoDS_Edge& edge,
                                  const gp_Ax1& axis)
{
    EdgeAxisRange range;
    if (edge.IsNull() || BRep_Tool::Degenerated(edge))
        return range;

    try {
        BRepAdaptor_Curve curve(edge);
        const double first = curve.FirstParameter();
        const double last = curve.LastParameter();
        if (!std::isfinite(first) || !std::isfinite(last))
            return range;

        const gp_Pnt axisOrigin = axis.Location();
        const gp_Vec axisDirection(axis.Direction());
        for (int index = 0; index <= 24; ++index) {
            const double parameter =
                first + (last - first) * static_cast<double>(index) / 24.0;
            const gp_Pnt point = curve.Value(parameter);
            gp_Vec offset(axisOrigin, point);
            const double axial = offset.Dot(axisDirection);
            offset -= axisDirection.Multiplied(axial);
            const double radius = offset.Magnitude();
            range.minimumRadius = std::min(range.minimumRadius, radius);
            if (radius > range.maximumRadius) {
                range.maximumRadius = radius;
                range.parameterAtMaximumRadius = axial;
                range.pointAtMaximumRadius = point;
            }
            range.valid = true;
        }
    } catch (const Standard_Failure&) {
        return {};
    }
    return range;
}

// 把一条边的采样范围并入聚合范围；只保留全局最大半径及其位置。
void mergeEdgeAxisRange(EdgeAxisRange& aggregate,
                        const EdgeAxisRange& sample)
{
    if (!sample.valid)
        return;
    aggregate.minimumRadius =
        std::min(aggregate.minimumRadius, sample.minimumRadius);
    if (!aggregate.valid || sample.maximumRadius > aggregate.maximumRadius) {
        aggregate.maximumRadius = sample.maximumRadius;
        aggregate.parameterAtMaximumRadius = sample.parameterAtMaximumRadius;
        aggregate.pointAtMaximumRadius = sample.pointAtMaximumRadius;
    }
    aggregate.valid = true;
}

// 候选过渡面是否接触一个“外部”平面：若该平面外侧还连接更粗的同轴圆柱
// （台阶肩部），则视为内部过渡而非孔口开口，返回 false。
bool touchesPlanarNeighbor(const AAG& graph,
                           int candidateFaceId,
                           int wallFaceId,
                           const gp_Ax1& wallAxis,
                           double wallRadius)
{
    const auto adjacent = graph.adjacency.find(candidateFaceId);
    if (adjacent == graph.adjacency.end())
        return false;
    return std::any_of(
        adjacent->second.begin(), adjacent->second.end(),
        [&](int neighborId) {
            if (neighborId == wallFaceId || neighborId < 1 ||
                neighborId > graph.faceMap.Extent() ||
                GetSurfaceType(TopoDS::Face(graph.faceMap(neighborId))) !=
                    GeomAbs_Plane) {
                return false;
            }

            // 内部锥形/圆角肩部的平面外侧还连接更大的同轴圆柱；这种面是
            // 台阶过渡而非外部开口，不能合成新的沉头加工段。
            const auto planarAdjacent = graph.adjacency.find(neighborId);
            if (planarAdjacent == graph.adjacency.end())
                return true;
            constexpr double kParallelDot = 0.9998476951563913; // cos(1°)
            const gp_Vec axisDirection(wallAxis.Direction());
            for (const int nextId : planarAdjacent->second) {
                if (nextId == candidateFaceId || nextId == wallFaceId ||
                    nextId < 1 || nextId > graph.faceMap.Extent()) {
                    continue;
                }
                gp_Ax1 nextAxis;
                double nextRadius = 0.0;
                const TopoDS_Face next =
                    TopoDS::Face(graph.faceMap(nextId));
                if (GetSurfaceType(next) != GeomAbs_Cylinder ||
                    !GetCylinderData(next, nextAxis, nextRadius) ||
                    nextRadius <= wallRadius + 1.0e-5 ||
                    std::abs(axisDirection.Dot(
                        gp_Vec(nextAxis.Direction()))) < kParallelDot) {
                    continue;
                }
                gp_Vec offset(wallAxis.Location(), nextAxis.Location());
                offset -= axisDirection.Multiplied(
                    offset.Dot(axisDirection));
                if (offset.Magnitude() <= 1.0e-5)
                    return false;
            }
            return true;
        });
}

} // namespace

HoleOpeningTransition recognizeOpeningTransition(
    const AAG& graph,
    int wallFaceId,
    const gp_Ax1& axis,
    double wallRadius,
    const std::vector<int>& convexFaceIds)
{
    HoleOpeningTransition selected;
    const double radialTolerance = std::max(1.0e-4, 0.03 * wallRadius);
    const TopoDS_Face wall = TopoDS::Face(graph.faceMap(wallFaceId));

    for (int candidateId : convexFaceIds) {
        if (candidateId < 1 || candidateId > graph.faceMap.Extent())
            continue;
        const TopoDS_Face candidate = TopoDS::Face(graph.faceMap(candidateId));
        const GeomAbs_SurfaceType type = GetSurfaceType(candidate);
        if (type == GeomAbs_Plane || type == GeomAbs_Cylinder ||
            !touchesPlanarNeighbor(
                graph, candidateId, wallFaceId, axis, wallRadius)) {
            continue;
        }

        TopoDS_Edge sharedEdge;
        if (!GetSharedEdge(wall, candidate, sharedEdge))
            continue;
        const EdgeAxisRange shared = sampleEdgeAxisRange(sharedEdge, axis);
        if (!shared.valid ||
            std::abs(shared.maximumRadius - wallRadius) > radialTolerance) {
            continue;
        }

        EdgeAxisRange boundary;
        for (TopExp_Explorer edge(candidate, TopAbs_EDGE); edge.More(); edge.Next()) {
            mergeEdgeAxisRange(
                boundary,
                sampleEdgeAxisRange(TopoDS::Edge(edge.Current()), axis));
        }
        if (!boundary.valid ||
            boundary.minimumRadius < wallRadius - radialTolerance ||
            boundary.maximumRadius <= wallRadius + radialTolerance ||
            std::abs(boundary.parameterAtMaximumRadius -
                     shared.parameterAtMaximumRadius) <= 1.0e-6) {
            continue;
        }

        if (selected.faceId < 1 ||
            boundary.maximumRadius > selected.outerRadius + radialTolerance) {
            selected.faceId = candidateId;
            selected.outerRadius = boundary.maximumRadius;
            selected.innerParameter = shared.parameterAtMaximumRadius;
            selected.outerParameter = boundary.parameterAtMaximumRadius;
            selected.outerPoint = boundary.pointAtMaximumRadius;
        }
    }
    return selected;
}

} // namespace oap::recognition::detail

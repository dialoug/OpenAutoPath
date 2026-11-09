// 沉头/沉孔级联关系匹配：依据共轴、轴向区间和肩面归属连接各圆柱阶段。
#include "CountersinkRelationMatcher.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <gp_Vec.hxx>

#include "recognition/AAG.h"
#include "recognition/Recognition.h"

namespace oap::recognition::detail {

namespace {

constexpr double kParallelDot = 0.9998476951563913; // cos(1°)

// 两孔段轴线是否同轴：方向近似平行（1° 内）且第一条轴线的垂向偏移不超过容差。
bool coaxialStageAxes(const CountersinkStageMatchInput& first,
                      const CountersinkStageMatchInput& second,
                      double tolerance)
{
    gp_Vec axis(first.first, first.second);
    gp_Vec secondAxis(second.first, second.second);
    if (axis.Magnitude() <= tolerance ||
        secondAxis.Magnitude() <= tolerance)
        return false;
    axis.Normalize();
    secondAxis.Normalize();
    if (std::abs(axis.Dot(secondAxis)) < kParallelDot)
        return false;
    gp_Vec offset(first.first, second.first);
    offset -= axis.Multiplied(offset.Dot(axis));
    return offset.Magnitude() <= tolerance;
}

// 锥面/环面过渡曲面的旋转轴是否与给定孔轴同轴，用于识别沉头锥形过渡。
bool turnedTransitionMatchesAxis(const TopoDS_Face& face,
                                 const gp_Pnt& axisPoint,
                                 const gp_Vec& axisDirection,
                                 double tolerance)
{
    BRepAdaptor_Surface surface(face);
    gp_Ax1 transitionAxis;
    if (surface.GetType() == GeomAbs_Cone)
        transitionAxis = surface.Cone().Axis();
    else if (surface.GetType() == GeomAbs_Torus)
        transitionAxis = surface.Torus().Axis();
    else
        return false;

    const gp_Vec transitionDirection(transitionAxis.Direction());
    if (std::abs(axisDirection.Dot(transitionDirection)) <
        kParallelDot)
        return false;
    gp_Vec offset(axisPoint, transitionAxis.Location());
    offset -= axisDirection.Multiplied(
        offset.Dot(axisDirection));
    return offset.Magnitude() <= tolerance;
}

// 沿“子孔圆柱 -> 共轴锥/环过渡 -> 轴向环形肩面 -> 父孔圆柱”的邻接链
// 逐跳核对，确认父子沉头段之间真实存在环形肩面结构。
bool shareCoaxialTurnedShoulderChain(
    const CountersinkStageMatchInput& child,
    const CountersinkStageMatchInput& owner,
    const AAG& graph,
    double tolerance)
{
    if (!coaxialStageAxes(child, owner, tolerance))
        return false;
    gp_Vec axis(child.first, child.second);
    axis.Normalize();

    for (int childCylinderId : child.cylinderFaceIds) {
        const auto childAdjacent =
            graph.adjacency.find(childCylinderId);
        if (childAdjacent == graph.adjacency.end())
            continue;
        for (int turnedFaceId : childAdjacent->second) {
            if (turnedFaceId < 1 ||
                turnedFaceId > graph.faceMap.Extent())
                continue;
            const TopoDS_Face turnedFace =
                TopoDS::Face(graph.faceMap(turnedFaceId));
            const GeomAbs_SurfaceType turnedType =
                GetSurfaceType(turnedFace);
            if ((turnedType != GeomAbs_Cone &&
                 turnedType != GeomAbs_Torus) ||
                !turnedTransitionMatchesAxis(
                    turnedFace, child.first, axis, tolerance))
                continue;

            const auto turnedAdjacent =
                graph.adjacency.find(turnedFaceId);
            if (turnedAdjacent == graph.adjacency.end())
                continue;
            for (int shoulderFaceId : turnedAdjacent->second) {
                if (shoulderFaceId < 1 ||
                    shoulderFaceId > graph.faceMap.Extent())
                    continue;
                const TopoDS_Face shoulderFace =
                    TopoDS::Face(graph.faceMap(shoulderFaceId));
                BRepAdaptor_Surface shoulderSurface(shoulderFace);
                if (GetSurfaceType(shoulderFace) != GeomAbs_Plane ||
                    shoulderSurface.GetType() != GeomAbs_Plane ||
                    std::abs(shoulderSurface.Plane().Axis().Direction().Dot(
                        axis)) < kParallelDot)
                    continue;

                const auto shoulderAdjacent =
                    graph.adjacency.find(shoulderFaceId);
                if (shoulderAdjacent == graph.adjacency.end())
                    continue;
                for (int ownerCylinderId : owner.cylinderFaceIds) {
                    if (std::find(
                            shoulderAdjacent->second.begin(),
                            shoulderAdjacent->second.end(),
                            ownerCylinderId) !=
                        shoulderAdjacent->second.end())
                        return true;
                }
            }
        }
    }
    return false;
}

// 两孔段是否至少共享一个公共邻面（肩面），作为直接台阶连接的拓扑证据。
bool shareAdjacentFace(const CountersinkStageMatchInput& first,
                       const CountersinkStageMatchInput& second,
                       const AAG& graph)
{
    for (int firstFace : first.cylinderFaceIds) {
        const auto firstAdjacent = graph.adjacency.find(firstFace);
        if (firstAdjacent == graph.adjacency.end())
            continue;
        for (int secondFace : second.cylinderFaceIds) {
            const auto secondAdjacent = graph.adjacency.find(secondFace);
            if (secondAdjacent == graph.adjacency.end())
                continue;
            for (int neighbor : firstAdjacent->second) {
                if (std::find(secondAdjacent->second.begin(),
                              secondAdjacent->second.end(), neighbor) !=
                    secondAdjacent->second.end())
                    return true;
            }
        }
    }
    return false;
}

bool coaxialIntervalsMeet(const CountersinkStageMatchInput& first,
                          const CountersinkStageMatchInput& second,
                          double tolerance)
{
    gp_Vec axis(first.first, first.second);
    gp_Vec secondAxis(second.first, second.second);
    if (axis.Magnitude() <= tolerance || secondAxis.Magnitude() <= tolerance)
        return false;
    axis.Normalize();
    secondAxis.Normalize();
    if (std::abs(axis.Dot(secondAxis)) < kParallelDot)
        return false;

    gp_Vec offset(first.first, second.first);
    offset -= axis.Multiplied(offset.Dot(axis));
    if (offset.Magnitude() > tolerance)
        return false;

    const auto parameter = [&](const gp_Pnt& point) {
        return gp_Vec(first.first, point).Dot(axis);
    };
    const double firstMinimum = 0.0;
    const double firstMaximum = first.first.Distance(first.second);
    double secondMinimum = parameter(second.first);
    double secondMaximum = parameter(second.second);
    if (secondMinimum > secondMaximum)
        std::swap(secondMinimum, secondMaximum);

    // 两段只能在一个轴向边界处接触。若轴向区间明显重叠，它们不是沉头孔
    // 的相邻台阶段；若存在明显间隙，也不能跨过实体材料强行归组。
    const double overlap = std::min(firstMaximum, secondMaximum) -
                           std::max(firstMinimum, secondMinimum);
    if (overlap > tolerance)
        return false;
    const double gap = std::max({firstMinimum, secondMinimum}) -
                       std::min({firstMaximum, secondMaximum});
    return gap <= tolerance;
}

// 指定面是否为任一圆柱壁的直接邻面。
bool containsAdjacentFace(const std::vector<int>& cylinderFaceIds,
                          int faceId,
                          const AAG& graph)
{
    for (int cylinderFaceId : cylinderFaceIds) {
        const auto adjacent = graph.adjacency.find(cylinderFaceId);
        if (adjacent != graph.adjacency.end() &&
            std::find(adjacent->second.begin(), adjacent->second.end(), faceId) !=
                adjacent->second.end())
            return true;
    }
    return false;
}

// 仅判断两孔段轴线方向是否平行（不要求共线），用于平底子孔的初步筛选。
bool axesAreParallel(const CountersinkStageMatchInput& first,
                     const CountersinkStageMatchInput& second,
                     double tolerance)
{
    gp_Vec firstAxis(first.first, first.second);
    gp_Vec secondAxis(second.first, second.second);
    if (firstAxis.Magnitude() <= tolerance ||
        secondAxis.Magnitude() <= tolerance)
        return false;
    firstAxis.Normalize();
    secondAxis.Normalize();
    return std::abs(firstAxis.Dot(secondAxis)) >= kParallelDot;
}

// 判断两孔段除平行外是否还共享同一轴线；同轴台阶不能降级为偏心平底子孔。
bool axesAreCoaxial(const CountersinkStageMatchInput& first,
                    const CountersinkStageMatchInput& second,
                    double tolerance)
{
    gp_Vec axis(first.first, first.second);
    if (axis.Magnitude() <= tolerance)
        return false;
    axis.Normalize();
    gp_Vec offset(first.first, second.first);
    offset -= axis.Multiplied(offset.Dot(axis));
    return offset.Magnitude() <= 10.0 * tolerance;
}

// 共享边被布尔运算拆分时，在调用方已确认轴线平行后，以平底平面和圆截面
// 包容关系确认子孔孔口。
bool floorGeometryContainsChildOpening(
    const CountersinkStageMatchInput& child,
    const CountersinkStageMatchInput& owner,
    double tolerance)
{
    gp_Vec ownerAxis(owner.first, owner.second);
    if (ownerAxis.Magnitude() <= tolerance)
        return false;
    ownerAxis.Normalize();

    const gp_Pnt floorCenter = owner.second;
    const auto axialDistanceToFloor = [&](const gp_Pnt& point) {
        return std::abs(gp_Vec(floorCenter, point).Dot(ownerAxis));
    };
    const gp_Pnt childOpening =
        axialDistanceToFloor(child.second) < axialDistanceToFloor(child.first)
        ? child.second : child.first;
    if (axialDistanceToFloor(childOpening) > 10.0 * tolerance)
        return false;

    gp_Vec radialOffset(floorCenter, childOpening);
    radialOffset -= ownerAxis.Multiplied(radialOffset.Dot(ownerAxis));
    return radialOffset.Magnitude() + child.radius <=
           owner.radius + 10.0 * tolerance;
}

// 子孔圆柱壁与平底面共享边中点是否落在子孔轴端点附近，验证真实孔口接触。
bool floorEdgeTouchesChildEndpoint(const CountersinkStageMatchInput& child,
                                   int floorFaceId,
                                   const AAG& graph,
                                   double tolerance)
{
    if (floorFaceId < 1 || floorFaceId > graph.faceMap.Extent())
        return false;
    const TopoDS_Face floor = TopoDS::Face(graph.faceMap(floorFaceId));
    gp_Vec axis(child.first, child.second);
    if (axis.Magnitude() <= tolerance)
        return false;
    axis.Normalize();

    for (int cylinderFaceId : child.cylinderFaceIds) {
        if (cylinderFaceId < 1 || cylinderFaceId > graph.faceMap.Extent())
            continue;
        TopoDS_Edge sharedEdge;
        if (!GetSharedEdge(
                TopoDS::Face(graph.faceMap(cylinderFaceId)), floor, sharedEdge) ||
            sharedEdge.IsNull() || BRep_Tool::Degenerated(sharedEdge))
            continue;
        try {
            BRepAdaptor_Curve curve(sharedEdge);
            const double firstParameter = curve.FirstParameter();
            const double lastParameter = curve.LastParameter();
            if (!std::isfinite(firstParameter) || !std::isfinite(lastParameter))
                continue;
            const gp_Pnt point = curve.Value(
                0.5 * (firstParameter + lastParameter));
            const double firstDistance =
                std::abs(gp_Vec(child.first, point).Dot(axis));
            const double secondDistance =
                std::abs(gp_Vec(child.second, point).Dot(axis));
            if (std::min(firstDistance, secondDistance) <= tolerance)
                return true;
        } catch (const Standard_Failure&) {
            continue;
        }
    }
    return false;
}

} // namespace

std::size_t findCountersinkStageOwner(
    const std::vector<CountersinkStageMatchInput>& inputs,
    std::size_t candidateIndex,
    const AAG& graph,
    double linearTolerance)
{
    if (candidateIndex >= inputs.size())
        return inputs.size();
    const CountersinkStageMatchInput& candidate = inputs[candidateIndex];
    const double tolerance = std::max(1.0e-7, std::abs(linearTolerance));
    std::size_t ownerIndex = inputs.size();
    double ownerRadius = -std::numeric_limits<double>::infinity();

    // 在全部候选父沉头段中取半径最大的满足者；只有孔段存在真实台阶拓扑
    // （直接邻接或共轴环形肩链）时才允许继承，不按纯几何位置合并。
    for (std::size_t index = 0; index < inputs.size(); ++index) {
        const CountersinkStageMatchInput& possibleOwner = inputs[index];
        if (index == candidateIndex ||
            possibleOwner.kind != FeatureKind::CountersinkHole ||
            possibleOwner.setupFace == MoldSetupFace::Unresolved ||
            possibleOwner.radius <= candidate.radius + tolerance)
            continue;

        const bool directStep =
            shareAdjacentFace(candidate, possibleOwner, graph) &&
            coaxialIntervalsMeet(
                candidate, possibleOwner, 10.0 * tolerance);
        const bool turnedShoulderStep =
            shareCoaxialTurnedShoulderChain(
                candidate, possibleOwner, graph,
                10.0 * tolerance);
        if (!directStep && !turnedShoulderStep)
            continue;
        if (possibleOwner.radius > ownerRadius + tolerance) {
            ownerIndex = index;
            ownerRadius = possibleOwner.radius;
        }
    }
    return ownerIndex;
}

CounterboreFloorOwner findCounterboreFloorOwner(
    const std::vector<CountersinkStageMatchInput>& inputs,
    std::size_t candidateIndex,
    const AAG& graph,
    double linearTolerance)
{
    CounterboreFloorOwner result;
    result.ownerIndex = inputs.size();
    if (candidateIndex >= inputs.size())
        return result;
    const CountersinkStageMatchInput& candidate = inputs[candidateIndex];
    const bool independentHole =
        candidate.kind == FeatureKind::ThroughHole ||
        candidate.kind == FeatureKind::BlindHole;
    const bool countersinkSmallStage =
        candidate.kind == FeatureKind::CountersinkHole &&
        !candidate.largestCountersinkStage;
    if (!independentHole && !countersinkSmallStage)
        return result;

    const double tolerance = std::max(1.0e-7, std::abs(linearTolerance));
    double selectedRadius = -std::numeric_limits<double>::infinity();
    // 只接受最大的沉头平底段作为父级；子孔须轴线平行且非同轴，并与平底
    // 有共享孔口拓扑（或平底几何包含子孔开口）才继承装夹面。
    for (std::size_t ownerIndex = 0; ownerIndex < inputs.size(); ++ownerIndex) {
        const CountersinkStageMatchInput& owner = inputs[ownerIndex];
        if (ownerIndex == candidateIndex ||
            owner.kind != FeatureKind::CountersinkHole ||
            !owner.largestCountersinkStage ||
            owner.bottomProfile != HoleBottomProfile::Planar ||
            owner.setupFace == MoldSetupFace::Unresolved ||
            owner.radius <= candidate.radius + tolerance ||
            !axesAreParallel(candidate, owner, tolerance) ||
            axesAreCoaxial(candidate, owner, tolerance))
            continue;

        for (int floorFaceId : owner.planarBoundaryFaceIds) {
            if (!containsAdjacentFace(
                    owner.cylinderFaceIds, floorFaceId, graph))
                continue;
            const bool sharesFloorTopology =
                containsAdjacentFace(
                    candidate.cylinderFaceIds, floorFaceId, graph) &&
                floorEdgeTouchesChildEndpoint(
                    candidate, floorFaceId, graph, 10.0 * tolerance);
            if (!sharesFloorTopology &&
                !floorGeometryContainsChildOpening(
                    candidate, owner, tolerance))
                continue;
            if (!result.valid || owner.radius > selectedRadius + tolerance) {
                result.ownerIndex = ownerIndex;
                result.floorFaceId = floorFaceId;
                result.valid = true;
                selectedRadius = owner.radius;
            }
        }
    }
    return result;
}

} // namespace oap::recognition::detail

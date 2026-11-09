// 通用孔与面特征识别实现，基于 AAG、曲面类型和拓扑关系形成候选结果。
// 模块: oap_recognition（依赖 oap_analysis 的 SurfaceClassify 与 DihedralAngle
//       凹凸分析）。主要流程：收集完整内凹圆柱候选 -> 按端部凹接判盲孔/
//       凸接判通孔 -> 孔轴投影端部分析组装孔结构面 -> 面归属冲突消解。
#include "Recognition.h"

#include "DihedralAngle.h"
#include "OpeningTransitionRecognizer.h"
#include "RecognitionGeometryCache.h"
#include "analysis/SurfaceClassify.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>

namespace
{
enum class EdgeConvexity
{
    CONVEX,
    CONCAVE,
    SMOOTH
};

constexpr double kParamMidpoint = 0.5;
constexpr double kDegenerateVectorTol = 1.0e-10;
constexpr double kAxisFacingDotTol = 1.0e-6;
constexpr double kFullCircleAngularTol = 1.0e-3;
constexpr double kAxialLinearTol = 1.0e-5;
constexpr double kTwoPi =
    6.283185307179586476925286766559;

EdgeConvexity ClassifyEdge(const TopoDS_Face& first,
                           const TopoDS_Face& second,
                           const TopoDS_Edge& edge)
{
    DihedralAngleChecker checker;
    if (!edge.IsNull())
        checker.SetCommonEdge(edge);
    const FeatureAngleType type = checker.AngleBetweenFaces(
        first, second, /*allowSmooth=*/false,
        /*smoothAngularTol=*/0.0);
    if (FeatureAngle::IsConvex(type))
        return EdgeConvexity::CONVEX;
    if (FeatureAngle::IsConcave(type))
        return EdgeConvexity::CONCAVE;
    return EdgeConvexity::SMOOTH;
}
} // namespace

GeomAbs_SurfaceType GetSurfaceType(const TopoDS_Face& face)
{
    return oap::effectiveSurfaceType(face);
}

bool GetCylinderData(const TopoDS_Face& face,
                     gp_Ax1& axis,
                     double& radius)
{
    BRepAdaptor_Surface surface(face);
    if (surface.GetType() != GeomAbs_Cylinder)
        return false;
    const gp_Cylinder cylinder = surface.Cylinder();
    axis = cylinder.Axis();
    radius = cylinder.Radius();
    return true;
}

int GetVertexCount(const TopoDS_Face& face)
{
    TopTools_IndexedMapOfShape vertices;
    TopExp::MapShapes(face, TopAbs_VERTEX, vertices);
    return vertices.Extent();
}

int GetEdgeCount(const TopoDS_Face& face)
{
    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(face, TopAbs_EDGE, edges);
    return edges.Extent();
}

namespace
{
bool IsSameEdgeGeom(const TopoDS_Edge& first,
                    const TopoDS_Edge& second)
{
    return first.IsSame(second) ||
           first.IsEqual(second) ||
           first.TShape() == second.TShape();
}
} // namespace

bool GetSharedEdge(const TopoDS_Face& first,
                   const TopoDS_Face& second,
                   TopoDS_Edge& sharedEdge)
{
    TopTools_IndexedMapOfShape firstEdges;
    TopExp::MapShapes(first, TopAbs_EDGE, firstEdges);
    for (TopExp_Explorer explorer(second, TopAbs_EDGE);
         explorer.More(); explorer.Next()) {
        const TopoDS_Edge secondEdge =
            TopoDS::Edge(explorer.Current());
        for (int index = 1;
             index <= firstEdges.Extent(); ++index) {
            const TopoDS_Edge firstEdge =
                TopoDS::Edge(firstEdges(index));
            if (IsSameEdgeGeom(firstEdge, secondEdge) &&
                !BRep_Tool::Degenerated(firstEdge)) {
                sharedEdge = firstEdge;
                return true;
            }
        }
    }
    return false;
}

namespace
{
// 判定面是否为孔壁圆柱：要求圆柱面且中点法向朝向圆柱轴（内凹几何门控）。
bool IsCylinderHoleWall(const TopoDS_Face& face)
{
    BRepAdaptor_Surface surface(face, Standard_True);
    if (surface.GetType() != GeomAbs_Cylinder)
        return false;

    const gp_Ax1 axis = surface.Cylinder().Axis();
    const double uMiddle = kParamMidpoint *
        (surface.FirstUParameter() +
         surface.LastUParameter());
    const double vMiddle = kParamMidpoint *
        (surface.FirstVParameter() +
         surface.LastVParameter());

    gp_Pnt point;
    gp_Vec du;
    gp_Vec dv;
    surface.D1(
        uMiddle, vMiddle, point, du, dv);
    gp_Vec normal = du.Crossed(dv);
    if (normal.Magnitude() < kDegenerateVectorTol)
        return false;
    if (face.Orientation() == TopAbs_REVERSED)
        normal.Reverse();
    normal.Normalize();

    const gp_Vec fromAxis(axis.Location(), point);
    const double parameter =
        fromAxis.Dot(gp_Vec(axis.Direction()));
    const gp_Pnt projection =
        axis.Location().Translated(
            gp_Vec(axis.Direction()) * parameter);
    gp_Vec towardAxis(point, projection);
    if (towardAxis.Magnitude() <
        kDegenerateVectorTol)
        return false;
    towardAxis.Normalize();
    return normal.Dot(towardAxis) >
           kAxisFacingDotTol;
}
} // namespace

bool IsFullCylinderFace(const TopoDS_Face& face)
{
    BRepAdaptor_Surface surface(face);
    if (surface.GetType() != GeomAbs_Cylinder)
        return false;

    Standard_Real uMinimum = 0.0;
    Standard_Real uMaximum = 0.0;
    Standard_Real vMinimum = 0.0;
    Standard_Real vMaximum = 0.0;
    BRepTools::UVBounds(
        face, uMinimum, uMaximum,
        vMinimum, vMaximum);
    return std::abs(uMaximum - uMinimum) >=
           kTwoPi - kFullCircleAngularTol;
}

namespace
{
std::uint64_t DirectedFacePairKey(
    int firstFaceId,
    int secondFaceId)
{
    return
        (static_cast<std::uint64_t>(
             static_cast<std::uint32_t>(
                 firstFaceId))
         << 32U) |
        static_cast<std::uint64_t>(
            static_cast<std::uint32_t>(
                secondFaceId));
}

class HoleRecognitionRunContext
{
public:
    HoleRecognitionRunContext(
        const AAG& graph,
        const oap::RecognitionGeometryCache&
            geometryCache)
        : m_graph(graph),
          m_geometryCache(geometryCache)
    {
        std::size_t adjacencyCount = 0;
        for (const auto& entry :
             m_graph.adjacency) {
            adjacencyCount +=
                entry.second.size();
        }
        m_edgeConvexities.reserve(
            adjacencyCount);
    }

    const AAG& graph() const
    {
        return m_graph;
    }

    const oap::RecognitionGeometryCache&
    geometryCache() const
    {
        return m_geometryCache;
    }

    EdgeConvexity classifySharedEdge(
        int firstFaceId,
        int secondFaceId,
        TopoDS_Edge& sharedEdge)
    {
        sharedEdge.Nullify();
        const TopoDS_Edge* cachedEdge =
            m_graph.sharedEdge(
                firstFaceId, secondFaceId);
        if (!cachedEdge)
            return EdgeConvexity::SMOOTH;
        sharedEdge = *cachedEdge;

        const std::uint64_t key =
            DirectedFacePairKey(
                firstFaceId, secondFaceId);
        const auto cached =
            m_edgeConvexities.find(key);
        if (cached !=
            m_edgeConvexities.end()) {
            return cached->second;
        }

        const TopoDS_Face first =
            TopoDS::Face(
                m_graph.faceMap(firstFaceId));
        const TopoDS_Face second =
            TopoDS::Face(
                m_graph.faceMap(secondFaceId));
        const EdgeConvexity convexity =
            ClassifyEdge(
                first, second, sharedEdge);
        m_edgeConvexities.emplace(
            key, convexity);
        return convexity;
    }

private:
    const AAG& m_graph;
    const oap::RecognitionGeometryCache&
        m_geometryCache;
    std::unordered_map<
        std::uint64_t, EdgeConvexity>
        m_edgeConvexities;
};

struct WallEdgeConvexity
{
    int concaveCount = 0;
    bool hasAdjacency = false;
};

// 统计孔壁邻接边中凹接的数量：任一凹接即判盲孔（端部封闭），全凸接视为
// 通孔。邻接列表来自 AAG 去重后的拓扑邻接。
WallEdgeConvexity CountWallEdgeConvexity(
    HoleRecognitionRunContext& context,
    int wallFaceId)
{
    const AAG& graph = context.graph();
    WallEdgeConvexity result;
    const auto adjacent =
        graph.adjacency.find(wallFaceId);
    if (adjacent == graph.adjacency.end())
        return result;
    result.hasAdjacency = true;

    for (const int neighborFaceId :
         adjacent->second) {
        TopoDS_Edge sharedEdge;
        if (context.classifySharedEdge(
                wallFaceId, neighborFaceId,
                sharedEdge) ==
            EdgeConvexity::CONCAVE) {
            ++result.concaveCount;
        }
    }
    return result;
}

struct HoleCandidate
{
    int faceId = -1;
    gp_Ax1 axis;
    double radius = 0.0;
    HoleType type = HoleType::UNKNOWN;
};

bool CollectHoleCandidate(
                          HoleRecognitionRunContext& context,
                          int faceId,
                          HoleCandidate& candidate)
{
    const AAG& graph = context.graph();
    const TopoDS_Face face =
        TopoDS::Face(graph.faceMap(faceId));
    // The canonical classifier only promotes freeform surfaces to planes.
    // Regressing every spline face cannot help identify a cylindrical wall.
    try {
        if (BRepAdaptor_Surface(face).GetType() != GeomAbs_Cylinder)
            return false;
    } catch (const Standard_Failure&) {
        return false;
    }
    if (!IsFullCylinderFace(face) ||
        !IsCylinderHoleWall(face)) {
        return false;
    }

    const WallEdgeConvexity convexity =
        CountWallEdgeConvexity(
            context, faceId);
    if (!convexity.hasAdjacency)
        return false;
    if (!GetCylinderData(
            face, candidate.axis,
            candidate.radius)) {
        return false;
    }

    candidate.faceId = faceId;
    candidate.type =
        convexity.concaveCount > 0
            ? HoleType::BLIND
            : HoleType::THROUGH;
    return true;
}

double AxisProjection(const gp_Pnt& point,
                      const gp_Ax1& axis)
{
    return gp_Vec(axis.Location(), point).Dot(
        gp_Vec(axis.Direction()));
}

double AxialTolerance(double first,
                      double second)
{
    return std::max(
        kAxialLinearTol,
        std::max(
            {1.0, std::abs(first),
             std::abs(second)}) *
            1.0e-9);
}

bool SameAxialPosition(double first,
                       double second)
{
    return std::abs(first - second) <=
           AxialTolerance(first, second);
}

bool ConstantEdgeAxialPosition(
    const TopoDS_Edge& edge,
    const gp_Ax1& axis,
    double& axialPosition)
{
    if (edge.IsNull() ||
        BRep_Tool::Degenerated(edge)) {
        return false;
    }
    try {
        BRepAdaptor_Curve curve(edge);
        const double first =
            curve.FirstParameter();
        const double last =
            curve.LastParameter();
        if (!std::isfinite(first) ||
            !std::isfinite(last)) {
            return false;
        }

        double minimum =
            std::numeric_limits<double>::infinity();
        double maximum =
            -std::numeric_limits<double>::infinity();
        constexpr int kSampleCount = 9;
        for (int sample = 0;
             sample < kSampleCount; ++sample) {
            const double ratio =
                static_cast<double>(sample) /
                static_cast<double>(
                    kSampleCount - 1);
            const gp_Pnt point = curve.Value(
                first +
                (last - first) * ratio);
            const double coordinate =
                AxisProjection(point, axis);
            minimum =
                std::min(minimum, coordinate);
            maximum =
                std::max(maximum, coordinate);
        }
        if (!std::isfinite(minimum) ||
            !std::isfinite(maximum) ||
            !SameAxialPosition(
                minimum, maximum)) {
            return false;
        }
        axialPosition =
            kParamMidpoint *
            (minimum + maximum);
        return true;
    } catch (const Standard_Failure&) {
        return false;
    }
}

bool SampleEdgePoint(const TopoDS_Edge& edge,
                     gp_Pnt& point)
{
    if (edge.IsNull() ||
        BRep_Tool::Degenerated(edge)) {
        return false;
    }
    try {
        BRepAdaptor_Curve curve(edge);
        const double first =
            curve.FirstParameter();
        const double last =
            curve.LastParameter();
        if (!std::isfinite(first) ||
            !std::isfinite(last)) {
            return false;
        }
        point = curve.Value(
            kParamMidpoint *
            (first + last));
        return true;
    } catch (const Standard_Failure&) {
        return false;
    }
}

struct HoleEndConnection
{
    int neighborFaceId = -1;
    EdgeConvexity convexity =
        EdgeConvexity::SMOOTH;
    bool hasSamplePoint = false;
    gp_Pnt samplePoint;
};

struct HoleEndGroup
{
    double axialPosition = 0.0;
    std::vector<HoleEndConnection>
        connections;
};

// 把孔壁的端部邻接面按共享边轴向位置分组建组（轴向等高归为一组）；组内
// 记录凸/凹连接类型与共享边采样点，供后续判定孔口与孔底。相邻圆柱面不进
// 组（独立孔壁，不作为当前孔的端部）。
std::vector<HoleEndGroup> BuildHoleEndGroups(
    HoleRecognitionRunContext& context,
    const HoleCandidate& candidate)
{
    const AAG& graph = context.graph();
    std::vector<HoleEndGroup> groups;
    const auto adjacent =
        graph.adjacency.find(
            candidate.faceId);
    if (adjacent == graph.adjacency.end())
        return groups;

    for (const int neighborFaceId :
         adjacent->second) {
        const TopoDS_Face neighbor =
            TopoDS::Face(
                graph.faceMap(
                    neighborFaceId));
        // 相邻圆柱仍是独立孔壁，不作为当前孔的底面或孔口承载面。
        if (GetSurfaceType(neighbor) ==
            GeomAbs_Cylinder) {
            continue;
        }

        TopoDS_Edge sharedEdge;
        const EdgeConvexity convexity =
            context.classifySharedEdge(
                candidate.faceId,
                neighborFaceId,
                sharedEdge);
        if (convexity ==
            EdgeConvexity::SMOOTH) {
            continue;
        }
        double axialPosition = 0.0;
        if (!ConstantEdgeAxialPosition(
                sharedEdge, candidate.axis,
                axialPosition)) {
            continue;
        }

        auto group = std::find_if(
            groups.begin(), groups.end(),
            [&](const HoleEndGroup& value) {
                return SameAxialPosition(
                    value.axialPosition,
                    axialPosition);
            });
        if (group == groups.end()) {
            groups.push_back(
                HoleEndGroup{
                    axialPosition, {}});
            group =
                std::prev(groups.end());
        }
        const bool alreadyCollected =
            std::any_of(
                group->connections.begin(),
                group->connections.end(),
                [&](const HoleEndConnection&
                        connection) {
                    return connection
                               .neighborFaceId ==
                           neighborFaceId &&
                           connection.convexity ==
                               convexity;
                });
        if (alreadyCollected)
            continue;

        HoleEndConnection connection;
        connection.neighborFaceId =
            neighborFaceId;
        connection.convexity = convexity;
        connection.hasSamplePoint =
            SampleEdgePoint(
                sharedEdge,
                connection.samplePoint);
        group->connections.push_back(
            std::move(connection));
    }

    std::sort(
        groups.begin(), groups.end(),
        [](const HoleEndGroup& first,
           const HoleEndGroup& second) {
            return first.axialPosition <
                   second.axialPosition;
        });
    return groups;
}

void SortUnique(std::vector<int>& values)
{
    std::sort(values.begin(), values.end());
    values.erase(
        std::unique(
            values.begin(), values.end()),
        values.end());
}

// 平面盲端若还承载半径更小、轴线平行的圆柱孔，则该平面是沉孔肩面或
// 平底，而不是实体盲孔底。允许小孔偏心，后续关系匹配器再判定具体父子关系。
bool HasSmallerParallelCylinderNeighbor(
    const AAG& graph,
    int boundaryFaceId,
    int wallFaceId,
    const gp_Ax1& wallAxis,
    double wallRadius)
{
    const auto adjacent = graph.adjacency.find(boundaryFaceId);
    if (adjacent == graph.adjacency.end())
        return false;
    constexpr double kParallelDot = 0.9998476951563913; // cos(1°)
    for (const int neighborFaceId : adjacent->second) {
        if (neighborFaceId == wallFaceId || neighborFaceId < 1 ||
            neighborFaceId > graph.faceMap.Extent()) {
            continue;
        }
        const TopoDS_Face neighbor =
            TopoDS::Face(graph.faceMap(neighborFaceId));
        gp_Ax1 neighborAxis;
        double neighborRadius = 0.0;
        if (GetSurfaceType(neighbor) == GeomAbs_Cylinder &&
            GetCylinderData(neighbor, neighborAxis, neighborRadius) &&
            neighborRadius < wallRadius - kAxialLinearTol &&
            std::abs(gp_Vec(wallAxis.Direction()).Dot(
                gp_Vec(neighborAxis.Direction()))) >= kParallelDot) {
            return true;
        }
    }
    return false;
}

oap::RecognitionGeometryCache BuildGeometryCache(
    const AAG& graph)
{
    BRep_Builder builder;
    TopoDS_Compound shape;
    builder.MakeCompound(shape);
    for (int faceId = 1;
         faceId <= graph.faceMap.Extent();
         ++faceId) {
        builder.Add(shape, graph.faceMap(faceId));
    }

    oap::RecognitionGeometryCache cache;
    cache.build(
        shape, graph,
        oap::RecognitionGeometryCacheMode::
            MainOuterFacesOnly);
    return cache;
}

// 组装单个孔的完整结果：遍历端部组，凸接面作为孔口（主外表面及平面/
// BSpline/Bezier 承载面只参与定位，不进入结构面集），盲孔凹接面作为底面；
// 平面底承载更小平行圆柱时提升为沉孔，开口扩径过渡命中时同样提升为沉孔。
HoleResult MakeHoleResult(
    HoleRecognitionRunContext& context,
    const HoleCandidate& candidate)
{
    const AAG& graph = context.graph();
    const oap::RecognitionGeometryCache&
        geometryCache =
            context.geometryCache();
    HoleResult result;
    result.type = candidate.type;
    result.mainFaceId = candidate.faceId;
    result.axis = candidate.axis;
    result.radius = candidate.radius;
    result.faces.push_back(
        candidate.faceId);

    int openingGroupCount = 0;
    bool hasOpeningSample = false;
    gp_Pnt openingSample;
    std::vector<int> convexFaceIds;
    const std::vector<HoleEndGroup> groups =
        BuildHoleEndGroups(
            context, candidate);
    for (const HoleEndGroup& group :
         groups) {
        bool groupHasOpening = false;
        bool groupHasOpeningSample = false;
        gp_Pnt groupOpeningSample;
        for (const HoleEndConnection&
                 connection :
             group.connections) {
            const TopoDS_Face neighbor =
                TopoDS::Face(
                    graph.faceMap(
                        connection
                            .neighborFaceId));
            const GeomAbs_SurfaceType type =
                GetSurfaceType(neighbor);
            if (connection.convexity ==
                EdgeConvexity::CONVEX) {
                convexFaceIds.push_back(
                    connection.neighborFaceId);
                // 凸接端为孔口承载面。通孔可有两个孔口，盲孔通常只有一个。
                // 主外表面以及凸接的平面、BSpline、Bezier 承载面只用于
                // 确定孔口位置，不进入孔结构面集。
                const bool excludedOpeningCarrier =
                    type == GeomAbs_Plane ||
                    type ==
                        GeomAbs_BSplineSurface ||
                    type ==
                        GeomAbs_BezierSurface;
                if (!excludedOpeningCarrier &&
                    !geometryCache.isMainOuterFace(
                        connection.neighborFaceId)) {
                    result.faces.push_back(
                        connection
                            .neighborFaceId);
                }
                groupHasOpening = true;
                if (!groupHasOpeningSample &&
                    connection
                        .hasSamplePoint) {
                    groupOpeningSample =
                        connection.samplePoint;
                    groupHasOpeningSample =
                        true;
                }
                continue;
            }
            if (candidate.type !=
                    HoleType::BLIND ||
                connection.convexity !=
                    EdgeConvexity::CONCAVE) {
                continue;
            }

            // 凹接端为盲孔底。所有底面进入结构面集；平面/锥面另供刀路
            // 解析实际底部轮廓，孔口承载面不会写入这些列表。
            if (geometryCache.isMainOuterFace(
                    connection.neighborFaceId)) {
                continue;
            }
            result.faces.push_back(
                connection.neighborFaceId);
            if (type == GeomAbs_Plane) {
                result.planarBottomFaceIds
                    .push_back(
                        connection
                            .neighborFaceId);
                if (HasSmallerParallelCylinderNeighbor(
                        graph, connection.neighborFaceId,
                        candidate.faceId, candidate.axis,
                        candidate.radius)) {
                    result.type = HoleType::COUNTERSINK;
                }
            } else if (
                type == GeomAbs_Cone) {
                result.conicalBottomFaceIds
                    .push_back(
                        connection
                            .neighborFaceId);
            }
        }
        if (groupHasOpening) {
            ++openingGroupCount;
            if (groupHasOpeningSample) {
                openingSample =
                    groupOpeningSample;
                hasOpeningSample = true;
            }
        }
    }

    SortUnique(convexFaceIds);
    result.openingTransition =
        oap::recognition::detail::recognizeOpeningTransition(
            graph, candidate.faceId, candidate.axis,
            candidate.radius, convexFaceIds);
    if (result.openingTransition.faceId > 0) {
        result.type = HoleType::COUNTERSINK;
        result.faces.push_back(
            result.openingTransition.faceId);
        result.hasOpeningPoint = true;
        result.openingPoint =
            result.openingTransition.outerPoint;
    }

    // 通孔两端均可作为入口，不设置唯一入口提示；盲孔只有一个等高孔口组
    // 时保留其共享边采样点，供刀路方向解析。
    if (result.openingTransition.faceId < 1 &&
        candidate.type ==
            HoleType::BLIND &&
        openingGroupCount == 1 &&
        hasOpeningSample) {
        result.hasOpeningPoint = true;
        result.openingPoint = openingSample;
    }

    SortUnique(result.faces);
    SortUnique(
        result.planarBottomFaceIds);
    SortUnique(
        result.conicalBottomFaceIds);
    return result;
}

// 通孔与盲孔面归属冲突时，把冲突面从通孔中移除（保留给盲孔）。
void PreferBlindHoleFaces(
    std::vector<HoleResult>& holes)
{
    std::unordered_set<int> blindFaces;
    for (const HoleResult& hole : holes) {
        if (hole.type != HoleType::BLIND)
            continue;
        blindFaces.insert(
            hole.faces.begin(), hole.faces.end());
    }

    for (HoleResult& hole : holes) {
        if (hole.type != HoleType::THROUGH)
            continue;
        hole.faces.erase(
            std::remove_if(
                hole.faces.begin(), hole.faces.end(),
                [&](int faceId) {
                    return blindFaces.count(faceId) > 0;
                }),
            hole.faces.end());
    }
}

// 多个盲孔争用同一底面时，与底面凹接的盲孔优先；仅凸接的盲孔从自己的
// 面集中让出该面。
void PreferConcaveBlindHoleOwners(
    HoleRecognitionRunContext& context,
    std::vector<HoleResult>& holes)
{
    const AAG& graph = context.graph();
    std::unordered_map<
        int, std::vector<std::size_t>>
        blindOwnersByFace;
    for (std::size_t holeIndex = 0;
         holeIndex < holes.size(); ++holeIndex) {
        const HoleResult& hole = holes[holeIndex];
        if (hole.type != HoleType::BLIND)
            continue;
        for (const int faceId : hole.faces)
            blindOwnersByFace[faceId].push_back(
                holeIndex);
    }

    for (const auto& [faceId, owners] :
         blindOwnersByFace) {
        if (owners.size() < 2 ||
            faceId < 1 ||
            faceId > graph.faceMap.Extent()) {
            continue;
        }

        std::unordered_set<std::size_t>
            concaveOwners;
        for (const std::size_t ownerIndex :
             owners) {
            const HoleResult& owner =
                holes[ownerIndex];
            if (owner.mainFaceId < 1 ||
                owner.mainFaceId >
                    graph.faceMap.Extent()) {
                continue;
            }
            TopoDS_Edge sharedEdge;
            if (context.classifySharedEdge(
                    owner.mainFaceId, faceId,
                    sharedEdge) ==
                EdgeConvexity::CONCAVE) {
                concaveOwners.insert(
                    ownerIndex);
            }
        }
        if (concaveOwners.empty())
            continue;

        for (const std::size_t ownerIndex :
             owners) {
            if (concaveOwners.count(
                    ownerIndex) > 0) {
                continue;
            }
            std::vector<int>& faces =
                holes[ownerIndex].faces;
            faces.erase(
                std::remove(
                    faces.begin(), faces.end(),
                    faceId),
                faces.end());
        }
    }
}

std::vector<HoleResult>
RecognizeHoleSetsOCCT(
    const AAG& graph,
    const oap::RecognitionGeometryCache&
        geometryCache)
{
    HoleRecognitionRunContext context(
        graph, geometryCache);
    std::vector<HoleCandidate> candidates;
    for (int faceId = 1;
         faceId <= graph.faceMap.Extent();
         ++faceId) {
        HoleCandidate candidate;
        if (CollectHoleCandidate(
                context, faceId,
                candidate)) {
            candidates.push_back(
                std::move(candidate));
        }
    }

    std::vector<HoleResult> holes;
    holes.reserve(candidates.size());
    // 按用户可见类别分两次执行同一套孔轴投影端部分析，分类结果在
    // 分析前已经固定；此阶段只负责补齐底面和孔口承载面。
    for (const HoleType type :
         {HoleType::THROUGH,
          HoleType::BLIND}) {
        for (const HoleCandidate& candidate :
             candidates) {
            if (candidate.type == type) {
                holes.push_back(
                    MakeHoleResult(
                        context, candidate));
            }
        }
    }
    // 盲孔之间抢面时先归给与该面凹接的孔；随后再处理通孔与盲孔冲突。
    PreferConcaveBlindHoleOwners(
        context, holes);
    PreferBlindHoleFaces(holes);
    return holes;
}
} // namespace

// 单面查询入口：完整跑一遍全图孔识别后按 mainFaceId 匹配返回；多次查询
// 请改用 RecognizeHolesOCCT，避免重复全图分析。
HoleResult ClassifyHoleOCCT(
    const AAG& graph, int faceId)
{
    const oap::RecognitionGeometryCache
        geometryCache =
            BuildGeometryCache(graph);
    const std::vector<HoleResult> holes =
        RecognizeHoleSetsOCCT(
            graph, geometryCache);
    const auto found = std::find_if(
        holes.begin(), holes.end(),
        [faceId](const HoleResult& hole) {
            return hole.mainFaceId ==
                   faceId;
        });
    return found == holes.end()
               ? HoleResult{}
               : *found;
}

// 无外部缓存版本：先按 AAG 面集组装 compound 构建几何缓存
// （MainOuterFacesOnly），再委托带缓存的重载执行识别。
std::vector<HoleResult> RecognizeHolesOCCT(
    AAG& graph, HoleCallback callback)
{
    const oap::RecognitionGeometryCache
        geometryCache =
            BuildGeometryCache(graph);
    return RecognizeHolesOCCT(
        graph, geometryCache,
        std::move(callback));
}

std::vector<HoleResult> RecognizeHolesOCCT(
    AAG& graph,
    const oap::RecognitionGeometryCache&
        geometryCache,
    HoleCallback callback)
{
    std::vector<HoleResult> holes =
        RecognizeHoleSetsOCCT(
            graph, geometryCache);
    if (callback) {
        for (const HoleResult& hole : holes)
            callback(hole);
    }
    return holes;
}

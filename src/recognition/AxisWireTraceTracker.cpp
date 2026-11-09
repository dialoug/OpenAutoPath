// AxisWireTraceTracker 实现 — 参见 AxisWireTraceTracker.h。
// 模块: oap_recognition。核心扩散算法 traceAxisPlaneWiresOCCT：对每个轴向平面
//       根，沿种子线环（内环/外环）进入邻接面，只接受相对高度不低于根面的面
//       （高度单调），终止于平行平面；随后按边连通分量拆分分支，并用末级邻
//       接面法向是否垂直根面法向区分盲槽/盲腔。
#include "AxisWireTraceTracker.h"

#include "AAG.h"
#include "RecognitionGeometryCache.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <BRepAdaptor_Surface.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_State.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_MapOfShape.hxx>
#include <gp.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

namespace oap {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kAngularTolerance = kPi / 180.0; // 1 度
constexpr double kParallelPlaneAngularTolerance =
    2.0 * kPi / 180.0;
constexpr double kAxisAlignedComponentTolerance = 0.01;
constexpr double kHeightTolerance = 1.0e-6;
constexpr double kNormalMagnitudeTolerance = 1.0e-12;
constexpr int kInteriorNormalSampleCount = 17;
constexpr int kBoundaryNormalSampleCount = 9;
constexpr int kMinimumValidNormalSampleCount = 9;
const double kParallelDotTolerance =
    std::cos(kParallelPlaneAngularTolerance);

struct TraceState {
    int faceIndex = 0;
};

struct FaceBoundaryProjection {
    signed char state = -1; // -1=未计算，0=无效，1=有效
    double minimum = 0.0;
    double maximum = 0.0;
};

struct RootPlane {
    int faceIndex = 0;
    double height = 0.0;
    gp_Dir normal = gp::DZ();
};

enum class WireSeedMode {
    InnerShrink,
    OuterSpread,
    Combined
};

enum class NormalSampleResult {
    Invalid,
    Perpendicular,
    NotPerpendicular
};

NormalSampleResult checkNormalAt(
    const BRepAdaptor_Surface& surface,
    double u,
    double v,
    const gp_Dir& referenceNormal)
{
    gp_Pnt point;
    gp_Vec du;
    gp_Vec dv;
    try {
        surface.D1(u, v, point, du, dv);
    } catch (const Standard_Failure&) {
        return NormalSampleResult::Invalid;
    }

    gp_Vec normal = du.Crossed(dv);
    if (normal.SquareMagnitude() <=
        kNormalMagnitudeTolerance * kNormalMagnitudeTolerance)
        return NormalSampleResult::Invalid;
    normal.Normalize();
    return std::abs(normal.Dot(referenceNormal)) <=
                   std::sin(kAngularTolerance)
               ? NormalSampleResult::Perpendicular
               : NormalSampleResult::NotPerpendicular;
}

bool sampledNormalsArePerpendicular(
    const TopoDS_Face& face,
    const BRepAdaptor_Surface& surface,
    const gp_Dir& referenceNormal)
{
    int validSampleCount = 0;
    const auto acceptSample =
        [&](double u, double v) {
            const NormalSampleResult sample =
                checkNormalAt(
                    surface, u, v, referenceNormal);
            if (sample ==
                NormalSampleResult::NotPerpendicular)
                return false;
            if (sample == NormalSampleResult::Perpendicular)
                ++validSampleCount;
            return true;
        };

    // 先覆盖裁剪边，避免窄面或带孔面的有效区域落不到内部规则网格。
    for (TopExp_Explorer explorer(face, TopAbs_EDGE);
         explorer.More(); explorer.Next()) {
        const TopoDS_Edge edge =
            TopoDS::Edge(explorer.Current());
        double first = 0.0;
        double last = 0.0;
        const Handle(Geom2d_Curve) curve =
            BRep_Tool::CurveOnSurface(
                edge, face, first, last);
        if (curve.IsNull() ||
            !std::isfinite(first) ||
            !std::isfinite(last))
            continue;
        for (int sampleIndex = 1;
             sampleIndex <= kBoundaryNormalSampleCount;
             ++sampleIndex) {
            const double fraction =
                static_cast<double>(sampleIndex) /
                static_cast<double>(
                    kBoundaryNormalSampleCount + 1);
            const gp_Pnt2d uv =
                curve->Value(
                    first + (last - first) * fraction);
            if (!acceptSample(uv.X(), uv.Y()))
                return false;
        }
    }

    double uMin = 0.0;
    double uMax = 0.0;
    double vMin = 0.0;
    double vMax = 0.0;
    BRepTools::UVBounds(
        face, uMin, uMax, vMin, vMax);
    if (!std::isfinite(uMin) ||
        !std::isfinite(uMax) ||
        !std::isfinite(vMin) ||
        !std::isfinite(vMax) ||
        uMax <= uMin || vMax <= vMin)
        return false;

    for (int uIndex = 0;
         uIndex < kInteriorNormalSampleCount;
         ++uIndex) {
        const double u =
            uMin + (uMax - uMin) *
                (static_cast<double>(uIndex) + 0.5) /
                static_cast<double>(
                    kInteriorNormalSampleCount);
        for (int vIndex = 0;
             vIndex < kInteriorNormalSampleCount;
             ++vIndex) {
            const double v =
                vMin + (vMax - vMin) *
                    (static_cast<double>(vIndex) + 0.5) /
                    static_cast<double>(
                        kInteriorNormalSampleCount);
            BRepClass_FaceClassifier classifier(
                face, gp_Pnt2d(u, v),
                Precision::PConfusion());
            if (classifier.State() != TopAbs_IN &&
                classifier.State() != TopAbs_ON)
                continue;
            if (!acceptSample(u, v))
                return false;
        }
    }
    return validSampleCount >=
           kMinimumValidNormalSampleCount;
}

bool allFaceNormalsArePerpendicular(
    const TopoDS_Face& face,
    int faceIndex,
    const RecognitionGeometryCache& geometryCache,
    const gp_Dir& referenceNormal)
{
    // 有效平面必须先于原始曲面类型处理；回归平面的 adaptor 仍会报告
    // Bezier/BSpline/拉伸面，直接走采样会与根面和终止面判定不一致。
    gp_Pln plane;
    if (geometryCache.effectivePlane(
            faceIndex, plane)) {
        return plane.Axis().Direction().IsNormal(
            referenceNormal, kAngularTolerance);
    }

    BRepAdaptor_Surface surface(face, Standard_True);
    switch (surface.GetType()) {
    case GeomAbs_Cylinder:
        return surface.Cylinder().Axis().Direction().IsParallel(
            referenceNormal, kAngularTolerance);
    case GeomAbs_SurfaceOfExtrusion:
        return surface.Direction().IsParallel(
            referenceNormal, kAngularTolerance);
    default:
        return sampledNormalsArePerpendicular(
            face, surface, referenceNormal);
    }
}

gp_Dir orientedPlaneNormal(const TopoDS_Face& face,
                           const gp_Pln& plane)
{
    gp_Dir normal = plane.Axis().Direction();
    if (face.Orientation() == TopAbs_REVERSED)
        normal.Reverse();
    return normal;
}

bool isAxisParallelPlane(int faceIndex,
                         const gp_Dir& axisDirection,
                         const RecognitionGeometryCache& geometryCache,
                         gp_Pln* matchedPlane = nullptr)
{
    gp_Pln plane;
    if (!geometryCache.effectivePlane(
            faceIndex, plane))
        return false;
    const double alignment = std::abs(
        plane.Axis().Direction().Dot(axisDirection));
    if (std::abs(alignment - 1.0) >=
        kAxisAlignedComponentTolerance)
        return false;
    if (matchedPlane)
        *matchedPlane = plane;
    return true;
}

bool isParallelTerminalPlane(
    int faceIndex,
    const gp_Dir& rootNormal,
    const RecognitionGeometryCache& geometryCache)
{
    gp_Pln plane;
    if (!geometryCache.effectivePlane(
            faceIndex, plane))
        return false;
    const gp_Dir normal = plane.Axis().Direction();
    return std::abs(normal.Dot(rootNormal)) >=
           kParallelDotTolerance;
}

std::vector<int> adjacentFacesAcrossEdge(
    const TopoDS_Edge& edge,
    int currentFaceIndex,
    const AAG& graph,
    const TopTools_IndexedDataMapOfShapeListOfShape& edgeFaces)
{
    std::vector<int> result;
    if (!edgeFaces.Contains(edge))
        return result;

    const TopTools_ListOfShape& faces =
        edgeFaces.FindFromKey(edge);
    for (TopTools_ListIteratorOfListOfShape iterator(faces);
         iterator.More(); iterator.Next()) {
        if (iterator.Value().ShapeType() != TopAbs_FACE)
            continue;
        const int faceIndex =
            graph.faceMap.FindIndex(iterator.Value());
        if (faceIndex > 0 && faceIndex != currentFaceIndex)
            result.push_back(faceIndex);
    }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()),
                 result.end());
    return result;
}

bool faceHeight(int faceIndex,
                RecognitionGeometryCache& geometryCache,
                const gp_Dir& axisDirection,
                double& height)
{
    // 根面已经被限定为轴向平面。使用解析平面的位置作为高度基准，
    // 避免三角化/公差导致的包围盒厚度把中心推离真实平面，进而将与
    // 根面共边的首层面误判到扩散方向的反侧。
    gp_Pln plane;
    if (!geometryCache.effectivePlane(faceIndex, plane))
        return false;
    const gp_Pnt location = plane.Location();
    height = location.X() * axisDirection.X() +
             location.Y() * axisDirection.Y() +
             location.Z() * axisDirection.Z();
    return std::isfinite(height);
}

bool faceLowestBoundaryEdgeRelativeHeight(
    int faceIndex,
    int rootDirection,
    const gp_Dir& axisDirection,
    const RecognitionGeometryCache& geometryCache,
    std::vector<FaceBoundaryProjection>& projectionCache,
    double& relativeHeight)
{
    if (faceIndex < 1 ||
        static_cast<std::size_t>(faceIndex) >=
            projectionCache.size()) {
        return false;
    }
    FaceBoundaryProjection& projection =
        projectionCache[
            static_cast<std::size_t>(faceIndex)];
    if (projection.state < 0) {
        projection.state =
            geometryCache.faceBoundaryEdgeProjectionRange(
                faceIndex, axisDirection,
                projection.minimum,
                projection.maximum)
                ? 1
                : 0;
    }
    if (projection.state != 1)
        return false;
    relativeHeight =
        rootDirection > 0
            ? projection.minimum
            : -projection.maximum;
    return std::isfinite(relativeHeight);
}

// 单条种子线环的广度扩散：从根面出发沿线环边进入邻接面，再逐层向邻接面
// 扩散。每个新面要求其边界边相对高度不低于根面（防止反向/回落）；遇平行
// 终止平面（不染为路径面）停止，同时记录 discoveredPlanes 与 traversedFaces。
void traceWire(
    int rootFaceIndex,
    const TopoDS_Wire& seedWire,
    double rootHeight,
    const gp_Dir& rootNormal,
    const gp_Dir& axisDirection,
    const AAG& graph,
    const RecognitionGeometryCache& geometryCache,
    const std::unordered_set<int>& blockedFaces,
    std::vector<signed char>& terminalPlaneCache,
    std::vector<FaceBoundaryProjection>&
        boundaryProjectionCache,
    std::unordered_set<int>& discoveredPlanes,
    std::unordered_set<int>& traversedFaces,
    std::unordered_set<int>* firstLevelFaces,
    AxisWireTraceResult& result)
{
    std::queue<TraceState> pending;
    std::unordered_set<int> seenFaces;
    std::unordered_set<std::uint64_t> evaluatedTransitions;
    const int rootDirection =
        rootNormal.Dot(axisDirection) >= 0.0 ? 1 : -1;
    const double rootRelativeHeight =
        static_cast<double>(rootDirection) * rootHeight;

    const auto considerNeighbor =
        [&](int fromFaceIndex,
            int neighborFaceIndex) {
            if (neighborFaceIndex < 1 ||
                neighborFaceIndex > graph.faceMap.Extent() ||
                blockedFaces.count(neighborFaceIndex) > 0)
                return false;
            const std::uint64_t transitionKey =
                (static_cast<std::uint64_t>(
                     static_cast<std::uint32_t>(
                         fromFaceIndex)) << 32U) |
                static_cast<std::uint32_t>(
                    neighborFaceIndex);
            if (!evaluatedTransitions
                     .insert(transitionKey)
                     .second)
                return false;

            try {
                signed char& terminalState =
                    terminalPlaneCache[
                        static_cast<std::size_t>(
                            neighborFaceIndex)];
                if (terminalState < 0) {
                    terminalState =
                        isParallelTerminalPlane(
                            neighborFaceIndex, rootNormal,
                            geometryCache)
                            ? 1
                            : 0;
                }
                if (terminalState > 0) {
                    discoveredPlanes.insert(neighborFaceIndex);
                    return false;
                }

                double neighborLowestEdgeRelativeHeight = 0.0;
                if (!faceLowestBoundaryEdgeRelativeHeight(
                        neighborFaceIndex, rootDirection,
                        axisDirection, geometryCache,
                        boundaryProjectionCache,
                        neighborLowestEdgeRelativeHeight)) {
                    ++result.skippedInvalidFaceCount;
                    return false;
                }
                if (neighborLowestEdgeRelativeHeight <
                    rootRelativeHeight - kHeightTolerance) {
                    ++result.stoppedOppositeHeightCount;
                    return false;
                }

                if (seenFaces.insert(neighborFaceIndex).second) {
                    pending.push({neighborFaceIndex});
                    ++result.directionalHeightStepCount;
                }
                return true;
            } catch (const Standard_Failure&) {
                ++result.skippedInvalidFaceCount;
                return false;
            }
        };

    TopTools_MapOfShape seenSeedEdges;
    for (TopExp_Explorer explorer(seedWire, TopAbs_EDGE);
         explorer.More(); explorer.Next()) {
        const TopoDS_Edge edge =
            TopoDS::Edge(explorer.Current());
        if (!seenSeedEdges.Add(edge))
            continue;
        for (const int neighbor : adjacentFacesAcrossEdge(
                 edge, rootFaceIndex, graph,
                 geometryCache.edgeFaces())) {
            if (considerNeighbor(
                    rootFaceIndex, neighbor) &&
                firstLevelFaces)
                firstLevelFaces->insert(neighbor);
        }
    }

    while (!pending.empty()) {
        const TraceState state = pending.front();
        pending.pop();

        ++result.traversedFaceCount;
        traversedFaces.insert(state.faceIndex);
        bool hasDirectionalNeighbor = false;
        for (const int neighbor :
             geometryCache.faceNeighbors(
                 state.faceIndex)) {
            if (considerNeighbor(
                    state.faceIndex,
                    neighbor))
                hasDirectionalNeighbor = true;
        }
        if (!hasDirectionalNeighbor)
            ++result.stoppedWithoutDirectionalNeighborCount;
    }
}

// 分支分类：若不存在末级邻接面则直接视为盲腔；否则要求末级每张面的全部
// 有效法向均垂直根面法向（盲槽特征），任一不满足即回退为盲腔。
AxisWireTraceFeatureKind classifyTraceBranch(
    const std::unordered_set<int>& lastLevelFaces,
    const gp_Dir& rootNormal,
    const AAG& graph,
    const RecognitionGeometryCache& geometryCache,
    std::unordered_map<int, bool>&
        perpendicularClassificationCache,
    AxisWireTraceResult& result)
{
    if (lastLevelFaces.empty())
        return AxisWireTraceFeatureKind::BlindPocket;

    for (const int faceIndex : lastLevelFaces) {
        try {
            const auto cached =
                perpendicularClassificationCache.find(
                    faceIndex);
            if (cached !=
                perpendicularClassificationCache.end()) {
                if (!cached->second)
                    return AxisWireTraceFeatureKind::BlindPocket;
                continue;
            }
            const TopoDS_Face firstLevelFace =
                TopoDS::Face(graph.faceMap(faceIndex));
            const bool perpendicular =
                allFaceNormalsArePerpendicular(
                    firstLevelFace, faceIndex,
                    geometryCache, rootNormal);
            perpendicularClassificationCache.emplace(
                faceIndex, perpendicular);
            if (!perpendicular)
                return AxisWireTraceFeatureKind::BlindPocket;
        } catch (const Standard_Failure&) {
            ++result.skippedInvalidFaceCount;
            return AxisWireTraceFeatureKind::BlindPocket;
        }
    }
    return AxisWireTraceFeatureKind::BlindSlot;
}

// 从首级面做多源 BFS，按共享边邻接分层，返回层号最大（距离最远）的一层；
// 若存在不可达面则视为拓扑不可靠并返回 false，让调用方保守回退为盲腔。
bool collectLastLevelFaces(
    const std::unordered_set<int>& firstLevelFaces,
    const std::unordered_set<int>& componentFaces,
    const AAG& graph,
    const RecognitionGeometryCache& geometryCache,
    std::unordered_set<int>& lastLevelFaces,
    AxisWireTraceResult& result)
{
    lastLevelFaces.clear();
    if (firstLevelFaces.empty() || componentFaces.empty())
        return false;

    std::queue<int> pending;
    std::unordered_map<int, int> levelByFace;
    for (const int faceIndex : firstLevelFaces) {
        if (componentFaces.count(faceIndex) == 0)
            continue;
        if (levelByFace.emplace(faceIndex, 0).second)
            pending.push(faceIndex);
    }
    if (pending.empty())
        return false;

    int maximumLevel = 0;
    while (!pending.empty()) {
        const int faceIndex = pending.front();
        pending.pop();
        const int nextLevel = levelByFace.at(faceIndex) + 1;
        try {
            if (faceIndex < 1 ||
                faceIndex > graph.faceMap.Extent()) {
                ++result.skippedInvalidFaceCount;
                lastLevelFaces.clear();
                return false;
            }
            for (const int neighborFaceIndex :
                 geometryCache.faceNeighbors(faceIndex)) {
                if (componentFaces.count(
                        neighborFaceIndex) == 0) {
                    continue;
                }
                if (!levelByFace
                         .emplace(
                             neighborFaceIndex, nextLevel)
                         .second) {
                    continue;
                }
                maximumLevel =
                    std::max(maximumLevel, nextLevel);
                pending.push(neighborFaceIndex);
            }
        } catch (const Standard_Failure&) {
            ++result.skippedInvalidFaceCount;
            lastLevelFaces.clear();
            return false;
        }
    }

    // 分支拆分和层级计算必须基于同一套共享边邻接。若仍有不可达面，
    // 说明拓扑不可靠，保守地让分类回退为盲腔。
    if (levelByFace.size() != componentFaces.size()) {
        ++result.skippedInvalidFaceCount;
        return false;
    }
    for (const auto& item : levelByFace) {
        if (item.second == maximumLevel)
            lastLevelFaces.insert(item.first);
    }
    return !lastLevelFaces.empty();
}

void populateTraceBranch(
    AxisWireTraceBranch& branch,
    const std::unordered_set<int>& firstLevelFaces,
    const std::unordered_set<int>& lastLevelFaces,
    const std::unordered_set<int>& traversedFaces,
    const std::unordered_set<int>& terminalFaces,
    const gp_Dir& rootNormal,
    const AAG& graph,
    const RecognitionGeometryCache& geometryCache,
    std::unordered_map<int, bool>&
        perpendicularClassificationCache,
    AxisWireTraceResult& result)
{
    branch.firstLevelFaceIndices.assign(
        firstLevelFaces.begin(), firstLevelFaces.end());
    std::sort(
        branch.firstLevelFaceIndices.begin(),
        branch.firstLevelFaceIndices.end());
    branch.lastLevelFaceIndices.assign(
        lastLevelFaces.begin(), lastLevelFaces.end());
    std::sort(
        branch.lastLevelFaceIndices.begin(),
        branch.lastLevelFaceIndices.end());
    branch.traversedFaceIndices.assign(
        traversedFaces.begin(), traversedFaces.end());
    std::sort(
        branch.traversedFaceIndices.begin(),
        branch.traversedFaceIndices.end());
    branch.terminalFaceIndices.assign(
        terminalFaces.begin(), terminalFaces.end());
    std::sort(
        branch.terminalFaceIndices.begin(),
        branch.terminalFaceIndices.end());
    branch.featureKind = classifyTraceBranch(
        lastLevelFaces, rootNormal, graph,
        geometryCache,
        perpendicularClassificationCache, result);
}

// 把一个线环扩散得到的走过面按共享边连通分量拆分为多条独立分支，每条分支
// 独立计算首级/末级/终止面并分类；返回顺序稳定（以排序后的种子面索引驱动
// BFS，分量内按索引升序收集）。
std::vector<AxisWireTraceBranch> splitTraceBranches(
    const std::unordered_set<int>& firstLevelFaces,
    const std::unordered_set<int>& traversedFaces,
    const std::unordered_set<int>& terminalFaces,
    const gp_Dir& rootNormal,
    const AAG& graph,
    const RecognitionGeometryCache& geometryCache,
    std::unordered_map<int, bool>&
        perpendicularClassificationCache,
    AxisWireTraceResult& result)
{
    std::vector<AxisWireTraceBranch> branches;
    if (traversedFaces.empty())
        return branches;

    std::vector<int> orderedFaces(
        traversedFaces.begin(), traversedFaces.end());
    std::sort(orderedFaces.begin(), orderedFaces.end());
    std::unordered_set<int> remaining(
        traversedFaces.begin(), traversedFaces.end());

    for (const int seedFaceIndex : orderedFaces) {
        if (remaining.erase(seedFaceIndex) == 0)
            continue;

        std::queue<int> pending;
        pending.push(seedFaceIndex);
        std::unordered_set<int> componentFaces;
        componentFaces.insert(seedFaceIndex);

        while (!pending.empty()) {
            const int faceIndex = pending.front();
            pending.pop();
            try {
                for (const int neighborFaceIndex :
                     geometryCache.faceNeighbors(faceIndex)) {
                    if (traversedFaces.count(
                            neighborFaceIndex) == 0 ||
                        remaining.erase(
                            neighborFaceIndex) == 0)
                        continue;
                    componentFaces.insert(neighborFaceIndex);
                    pending.push(neighborFaceIndex);
                }
            } catch (const Standard_Failure&) {
                ++result.skippedInvalidFaceCount;
            }
        }

        std::unordered_set<int> componentFirstLevelFaces;
        std::unordered_set<int> componentLastLevelFaces;
        std::unordered_set<int> componentTerminalFaces;
        for (const int faceIndex : componentFaces) {
            if (firstLevelFaces.count(faceIndex) > 0)
                componentFirstLevelFaces.insert(faceIndex);
            try {
                for (const int neighborFaceIndex :
                     geometryCache.faceNeighbors(faceIndex)) {
                    if (terminalFaces.count(
                            neighborFaceIndex) > 0) {
                        componentTerminalFaces.insert(
                            neighborFaceIndex);
                    }
                }
            } catch (const Standard_Failure&) {
                ++result.skippedInvalidFaceCount;
            }
        }
        collectLastLevelFaces(
            componentFirstLevelFaces, componentFaces,
            graph, geometryCache,
            componentLastLevelFaces,
            result);

        AxisWireTraceBranch branch;
        populateTraceBranch(
            branch, componentFirstLevelFaces,
            componentLastLevelFaces,
            componentFaces, componentTerminalFaces,
            rootNormal, graph, geometryCache,
            perpendicularClassificationCache, result);
        branches.push_back(std::move(branch));
    }
    return branches;
}

} // namespace

static AxisWireTraceResult traceAxisPlaneWiresOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const gp_Dir& axisDirection,
    const std::unordered_set<int>& excludedFaceIndices,
    WireSeedMode mode,
    RecognitionGeometryCache& geometryCache)
{
    AxisWireTraceResult result;
    if (shape.IsNull() || graph.faceMap.Extent() == 0)
        return result;

    std::unordered_set<int> blockedFaces =
        excludedFaceIndices;
    std::vector<RootPlane> rootPlanes;

    // 排除面在任何几何类型或入口筛选前跳过。所有未排除轴向平面
    // 都可作为独立入口；逐体 AABB 主外表面只作为入口，不可进入路径。
    for (int faceIndex = 1;
         faceIndex <= graph.faceMap.Extent(); ++faceIndex) {
        if (blockedFaces.count(faceIndex) > 0)
            continue;
        try {
            const TopoDS_Face face =
                TopoDS::Face(graph.faceMap(faceIndex));
            if (geometryCache.isMainOuterFace(faceIndex)) {
                blockedFaces.insert(faceIndex);
                ++result.mainOuterFaceCount;
            }
            gp_Pln plane;
            if (!isAxisParallelPlane(
                    faceIndex, axisDirection,
                    geometryCache, &plane))
                continue;

            double height = 0.0;
            if (!faceHeight(
                    faceIndex, geometryCache,
                    axisDirection, height)) {
                ++result.skippedInvalidFaceCount;
                continue;
            }
            rootPlanes.push_back(
                {faceIndex, height,
                 orientedPlaneNormal(face, plane)});
        } catch (const Standard_Failure&) {
            blockedFaces.insert(faceIndex);
            ++result.skippedInvalidFaceCount;
        }
    }
    std::sort(
        rootPlanes.begin(), rootPlanes.end(),
        [&](const RootPlane& first, const RootPlane& second) {
            const int firstDirection =
                first.normal.Dot(axisDirection) >= 0.0
                    ? 1 : -1;
            const int secondDirection =
                second.normal.Dot(axisDirection) >= 0.0
                    ? 1 : -1;
            if (firstDirection != secondDirection)
                return firstDirection > secondDirection;
            if (std::abs(first.height - second.height) >
                kHeightTolerance) {
                return firstDirection > 0
                           ? first.height < second.height
                           : first.height > second.height;
            }
            return first.faceIndex < second.faceIndex;
        });
    result.rootFaceCount =
        static_cast<int>(rootPlanes.size());
    result.excludedFaceCount =
        static_cast<int>(std::count_if(
            blockedFaces.begin(), blockedFaces.end(),
            [&](int faceIndex) {
                return faceIndex >= 1 &&
                       faceIndex <= graph.faceMap.Extent();
            }));

    std::unordered_set<int> allDiscoveredPlanes;
    std::unordered_set<int> allTraversedFaces;
    std::unordered_map<int, bool>
        perpendicularClassificationCache;
    // 根面法向仅可能与当前轴同向或反向；终止平面判定使用绝对点积，
    // 因而每个轴对每张面只需计算一次。
    std::vector<signed char> terminalPlaneCache(
        static_cast<std::size_t>(
            graph.faceMap.Extent() + 1),
        static_cast<signed char>(-1));
    std::vector<FaceBoundaryProjection>
        boundaryProjectionCache(
            static_cast<std::size_t>(
                graph.faceMap.Extent() + 1));
    for (const RootPlane& root : rootPlanes) {
        const int rootFaceIndex = root.faceIndex;
        const TopoDS_Face rootFace =
            TopoDS::Face(graph.faceMap(rootFaceIndex));
        const bool rootIsMainOuterFace =
            geometryCache.isMainOuterFace(rootFaceIndex);
        std::unordered_set<int> outerDiscoveredPlanes;
        std::unordered_set<int> outerTraversedFaces;
        std::unordered_set<int> outerFirstLevelFaces;
        AxisWireTracePath path;
        path.rootFaceIndex = rootFaceIndex;
        path.rootHeight = root.height;
        path.normalDirection =
            root.normal.Dot(axisDirection) >= 0.0
                ? 1 : -1;
        try {
            const TopoDS_Wire outerWire =
                BRepTools::OuterWire(rootFace);

            if (mode != WireSeedMode::InnerShrink) {
                if (!outerWire.IsNull()) {
                    ++result.seedWireCount;
                    traceWire(
                        rootFaceIndex, outerWire, root.height,
                        root.normal, axisDirection,
                        graph, geometryCache,
                        blockedFaces, terminalPlaneCache,
                        boundaryProjectionCache,
                        outerDiscoveredPlanes,
                        outerTraversedFaces,
                        &outerFirstLevelFaces, result);
                }
            }
            if (mode != WireSeedMode::OuterSpread) {
                // 主外表面采用实体外法向；从其内环进入槽腔时，实际追踪
                // 方向应指向实体内部，即外法向的反方向。内部根面仍沿
                // 原有相对高方向追踪，外环分支也保持原规则。
                gp_Dir innerTraceNormal = root.normal;
                if (rootIsMainOuterFace)
                    innerTraceNormal.Reverse();
                for (TopExp_Explorer explorer(
                         rootFace, TopAbs_WIRE);
                     explorer.More(); explorer.Next()) {
                    const TopoDS_Wire wire =
                        TopoDS::Wire(explorer.Current());
                    if (!outerWire.IsNull() &&
                        wire.IsSame(outerWire))
                        continue;
                    std::unordered_set<int>
                        innerDiscoveredPlanes;
                    std::unordered_set<int>
                        innerTraversedFaces;
                    std::unordered_set<int>
                        innerFirstLevelFaces;
                    ++result.seedWireCount;
                    traceWire(
                        rootFaceIndex, wire, root.height,
                        innerTraceNormal, axisDirection,
                        graph, geometryCache,
                        blockedFaces, terminalPlaneCache,
                        boundaryProjectionCache,
                        innerDiscoveredPlanes,
                        innerTraversedFaces,
                        &innerFirstLevelFaces, result);
                    std::vector<AxisWireTraceBranch>
                        innerBranches = splitTraceBranches(
                        innerFirstLevelFaces,
                        innerTraversedFaces,
                        innerDiscoveredPlanes,
                        innerTraceNormal, graph, geometryCache,
                        perpendicularClassificationCache,
                        result);
                    for (AxisWireTraceBranch& branch :
                         innerBranches) {
                        path.innerTraces.push_back(
                            std::move(branch));
                    }
                }
            }
        } catch (const Standard_Failure&) {
            ++result.skippedInvalidFaceCount;
        }

        path.outerTraces = splitTraceBranches(
            outerFirstLevelFaces,
            outerTraversedFaces, outerDiscoveredPlanes,
            root.normal, graph, geometryCache,
            perpendicularClassificationCache, result);
        std::unordered_set<int> discoveredPlanes =
            outerDiscoveredPlanes;
        for (const AxisWireTraceBranch& branch :
             path.innerTraces) {
            discoveredPlanes.insert(
                branch.terminalFaceIndices.begin(),
                branch.terminalFaceIndices.end());
        }
        path.terminalFaceIndices.assign(
            discoveredPlanes.begin(), discoveredPlanes.end());
        std::sort(path.terminalFaceIndices.begin(),
                  path.terminalFaceIndices.end());

        std::unordered_set<int> traversedFaces(
            outerTraversedFaces.begin(),
            outerTraversedFaces.end());
        for (const AxisWireTraceBranch& branch :
             path.innerTraces) {
            traversedFaces.insert(
                branch.traversedFaceIndices.begin(),
                branch.traversedFaceIndices.end());
        }
        path.traversedFaceIndices.assign(
            traversedFaces.begin(), traversedFaces.end());
        std::sort(path.traversedFaceIndices.begin(),
                  path.traversedFaceIndices.end());
        result.paths.push_back(std::move(path));

        allDiscoveredPlanes.insert(
            discoveredPlanes.begin(), discoveredPlanes.end());
        allTraversedFaces.insert(
            traversedFaces.begin(), traversedFaces.end());
    }

    result.terminalFaceIndices.assign(
        allDiscoveredPlanes.begin(), allDiscoveredPlanes.end());
    std::sort(result.terminalFaceIndices.begin(),
              result.terminalFaceIndices.end());
    result.traversedFaceIndices.assign(
        allTraversedFaces.begin(), allTraversedFaces.end());
    std::sort(result.traversedFaceIndices.begin(),
              result.traversedFaceIndices.end());
    return result;
}

InnerWireShrinkTraceResult TraceInnerWireShrinkAlongAxisOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const gp_Dir& axisDirection,
    const std::unordered_set<int>& excludedFaceIndices)
{
    RecognitionGeometryCache geometryCache;
    geometryCache.build(shape, graph);
    return traceAxisPlaneWiresOCCT(
        shape, graph, axisDirection, excludedFaceIndices,
        WireSeedMode::InnerShrink, geometryCache);
}

OuterWireSpreadTraceResult TraceOuterWireSpreadAlongAxisOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const gp_Dir& axisDirection,
    const std::unordered_set<int>& excludedFaceIndices)
{
    RecognitionGeometryCache geometryCache;
    geometryCache.build(shape, graph);
    return traceAxisPlaneWiresOCCT(
        shape, graph, axisDirection, excludedFaceIndices,
        WireSeedMode::OuterSpread, geometryCache);
}

CombinedWireTraceResult TraceCombinedWireLoopsAlongAxisOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const gp_Dir& axisDirection,
    const std::unordered_set<int>& excludedFaceIndices)
{
    RecognitionGeometryCache geometryCache;
    geometryCache.build(shape, graph);
    return traceAxisPlaneWiresOCCT(
        shape, graph, axisDirection, excludedFaceIndices,
        WireSeedMode::Combined, geometryCache);
}

CombinedWireTraceResult TraceCombinedWireLoopsAlongAxisOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const gp_Dir& axisDirection,
    const std::unordered_set<int>& excludedFaceIndices,
    RecognitionGeometryCache& geometryCache)
{
    return traceAxisPlaneWiresOCCT(
        shape, graph, axisDirection, excludedFaceIndices,
        WireSeedMode::Combined, geometryCache);
}

InnerWireShrinkTraceResult TraceInnerWireShrinkOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const std::unordered_set<int>& excludedFaceIndices)
{
    return TraceInnerWireShrinkAlongAxisOCCT(
        shape, graph, gp::DZ(), excludedFaceIndices);
}

OuterWireSpreadTraceResult TraceOuterWireSpreadOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const std::unordered_set<int>& excludedFaceIndices)
{
    return TraceOuterWireSpreadAlongAxisOCCT(
        shape, graph, gp::DZ(), excludedFaceIndices);
}

CombinedWireTraceResult TraceCombinedWireLoopsOCCT(
    const TopoDS_Shape& shape,
    const AAG& graph,
    const std::unordered_set<int>& excludedFaceIndices)
{
    return TraceCombinedWireLoopsAlongAxisOCCT(
        shape, graph, gp::DZ(), excludedFaceIndices);
}

} // namespace oap

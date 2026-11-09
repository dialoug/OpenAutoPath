// ThroughStructureRecognizer 实现（oap_recognition）。
// 通结构面识别：筛选整面法向垂直 Z 轴的面并检查非 Z 向共享边的凸性，
// 见 ThroughStructureRecognizer.h。
#include "ThroughStructureRecognizer.h"

#include "AAG.h"
#include "DihedralAngle.h"
#include "RecognitionGeometryCache.h"
#include "Recognition.h"

#include <cmath>
#include <unordered_set>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopAbs_State.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <gp.hxx>
#include <gp_Pnt2d.hxx>

namespace oap {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kOrientationAngularTolerance = kPi / 180.0; // 1 度
constexpr double kNormalMagnitudeTolerance = 1.0e-12;
constexpr int kInteriorSampleCount = 17;
constexpr int kBoundarySampleCount = 9;
constexpr int kMinimumGeneralSurfaceSamples = 9;
constexpr int kEdgeDirectionSampleCount = 17;

enum class NormalSampleResult {
    Invalid,
    Perpendicular,
    NotPerpendicular
};

// 在 (u,v) 处求一阶偏导得到法向；叉积模长过小（退化点）记 Invalid，
// 法向与 Z 轴夹角不超过 1° 记 Perpendicular。
NormalSampleResult checkNormalAt(const BRepAdaptor_Surface& surface,
                                 double u,
                                 double v)
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

    return std::abs(normal.Dot(gp::DZ())) <=
                   std::sin(kOrientationAngularTolerance)
               ? NormalSampleResult::Perpendicular
               : NormalSampleResult::NotPerpendicular;
}

bool allSampledNormalsPerpendicularToZ(const TopoDS_Face& face,
                                       const BRepAdaptor_Surface& surface)
{
    int validSampleCount = 0;
    auto acceptSample = [&](double u, double v) {
        const NormalSampleResult sample = checkNormalAt(surface, u, v);
        if (sample == NormalSampleResult::NotPerpendicular)
            return false;
        if (sample == NormalSampleResult::Perpendicular)
            ++validSampleCount;
        return true;
    };

    // 先检查每条裁剪边的参数曲线，覆盖窄面、孔环和普通 UV 网格难以落入的区域。
    for (TopExp_Explorer explorer(face, TopAbs_EDGE); explorer.More(); explorer.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(explorer.Current());
        double first = 0.0;
        double last = 0.0;
        Handle(Geom2d_Curve) curve = BRep_Tool::CurveOnSurface(
            edge, face, first, last);
        if (curve.IsNull() || !std::isfinite(first) || !std::isfinite(last))
            continue;
        for (int i = 1; i <= kBoundarySampleCount; ++i) {
            const double fraction = static_cast<double>(i) /
                                    static_cast<double>(kBoundarySampleCount + 1);
            const gp_Pnt2d uv = curve->Value(first + (last - first) * fraction);
            if (!acceptSample(uv.X(), uv.Y()))
                return false;
        }
    }

    double uMin = 0.0;
    double uMax = 0.0;
    double vMin = 0.0;
    double vMax = 0.0;
    BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);
    if (!std::isfinite(uMin) || !std::isfinite(uMax) ||
        !std::isfinite(vMin) || !std::isfinite(vMax) ||
        uMax <= uMin || vMax <= vMin)
        return false;

    // 单元中心采样避开参数奇点；分类器保证只验证裁剪面域内部。
    for (int ui = 0; ui < kInteriorSampleCount; ++ui) {
        const double u = uMin + (uMax - uMin) *
            (static_cast<double>(ui) + 0.5) /
            static_cast<double>(kInteriorSampleCount);
        for (int vi = 0; vi < kInteriorSampleCount; ++vi) {
            const double v = vMin + (vMax - vMin) *
                (static_cast<double>(vi) + 0.5) /
                static_cast<double>(kInteriorSampleCount);
            BRepClass_FaceClassifier classifier(
                face, gp_Pnt2d(u, v), Precision::PConfusion());
            if (classifier.State() != TopAbs_IN && classifier.State() != TopAbs_ON)
                continue;
            if (!acceptSample(u, v))
                return false;
        }
    }

    // 一般曲面只有在裁剪域内取得足够多的有效法向时才允许通过，避免退化面误判。
    return validSampleCount >= kMinimumGeneralSurfaceSamples;
}

bool isSurfacePerpendicularToXoy(
    const TopoDS_Face& face,
    int faceIndex,
    const RecognitionGeometryCache& geometryCache)
{
    gp_Pln plane;
    if (geometryCache.effectivePlane(
            faceIndex, plane)) {
        // 平面垂直于 XOY 时，其法向位于 XY 平面。
        return plane.Axis().Direction().IsNormal(
            gp::DZ(), kOrientationAngularTolerance);
    }

    BRepAdaptor_Surface surface(face, Standard_True);

    switch (surface.GetType()) {
    case GeomAbs_Cylinder:
        // 圆柱轴沿 Z 时，整张圆柱面的法向都位于 XY 平面。
        return surface.Cylinder().Axis().Direction().IsParallel(
            gp::DZ(), kOrientationAngularTolerance);
    case GeomAbs_SurfaceOfExtrusion:
        // 拉伸方向沿 Z 是整张拉伸面满足条件的解析充分条件。
        return surface.Direction().IsParallel(
            gp::DZ(), kOrientationAngularTolerance);
    default:
        // 圆锥、球、环面、旋转面、Bezier/BSpline 等均检查整个裁剪域；
        // 任一有效法向超差即拒绝，退化或有效样本不足也拒绝。
        return allSampledNormalsPerpendicularToZ(face, surface);
    }
}

// 判定边是否沿 Z 轴：直线直接判方向，一般曲线沿参数域采样切向避免误判。
bool isZDirectedEdgeImpl(const TopoDS_Edge& edge)
{
    if (edge.IsNull())
        return false;

    BRepAdaptor_Curve curve(edge);
    if (curve.GetType() == GeomAbs_Line) {
        return curve.Line().Direction().IsParallel(
            gp::DZ(), kOrientationAngularTolerance);
    }

    // STEP 中几何上为直线的边也可能以 B-spline/Bezier 等一般曲线保存。
    // 先用首尾弦排除闭合边、退化边和整体非 Z 向边，再沿参数域检查切向，
    // 避免只看端点而把中途横向弯曲的曲线误判为 Z 向。
    const double first = curve.FirstParameter();
    const double last = curve.LastParameter();
    if (!std::isfinite(first) || !std::isfinite(last) || last <= first)
        return false;

    const gp_Pnt firstPoint = curve.Value(first);
    const gp_Pnt lastPoint = curve.Value(last);
    const gp_Vec chord(firstPoint, lastPoint);
    if (chord.SquareMagnitude() <=
            kNormalMagnitudeTolerance * kNormalMagnitudeTolerance ||
        !gp_Dir(chord).IsParallel(gp::DZ(), kOrientationAngularTolerance))
        return false;

    for (int sampleIndex = 0;
         sampleIndex < kEdgeDirectionSampleCount;
         ++sampleIndex) {
        const double fraction =
            (static_cast<double>(sampleIndex) + 0.5) /
            static_cast<double>(kEdgeDirectionSampleCount);
        gp_Pnt point;
        gp_Vec tangent;
        curve.D1(first + (last - first) * fraction, point, tangent);
        if (tangent.SquareMagnitude() <=
                kNormalMagnitudeTolerance * kNormalMagnitudeTolerance ||
            !gp_Dir(tangent).IsParallel(
                gp::DZ(), kOrientationAngularTolerance))
            return false;
    }
    return true;
}

// 用二面角检查器判断沿共享边是否外凸；平滑过渡视为非凸，保证通结构
// 边界是真实的凸棱。
bool isConvexAlongEdge(const TopoDS_Face& face,
                       const TopoDS_Face& neighbor,
                       const TopoDS_Edge& edge)
{
    DihedralAngleChecker checker;
    checker.SetCommonEdge(edge);
    const FeatureAngleType type = checker.AngleBetweenFaces(
        face, neighbor, /*allowSmooth=*/false, /*smoothAngularTol=*/0.0);
    return FeatureAngle::IsConvex(type);
}

} // namespace

namespace detail {

bool isEdgeGeometricallyZDirected(const TopoDS_Edge& edge)
{
    return isZDirectedEdgeImpl(edge);
}

} // namespace detail

ThroughStructureRecognitionResult RecognizeThroughStructuresOCCT(
    const TopoDS_Shape& shape, AAG& graph)
{
    RecognitionGeometryCache geometryCache;
    geometryCache.build(shape, graph);

    // 孔结构优先级高于通结构：孔识别实例涉及的所有面均加入排除集合。
    // 孔与通结构共享同一份完整几何缓存；失败的体不应继续产生可能包含
    // 孔面的通结构结果。
    std::unordered_set<int> holeFaceIndices;
    for (const HoleResult& hole :
         RecognizeHolesOCCT(
             graph, geometryCache)) {
        for (const int faceIndex : hole.faces) {
            if (faceIndex >= 1 && faceIndex <= graph.faceMap.Extent())
                holeFaceIndices.insert(faceIndex);
        }
    }

    return RecognizeThroughStructuresOCCT(
        shape, graph, holeFaceIndices,
        geometryCache);
}

ThroughStructureRecognitionResult RecognizeThroughStructuresOCCT(
    const TopoDS_Shape& shape,
    AAG& graph,
    const std::unordered_set<int>& recognizedHoleFaceIndices)
{
    RecognitionGeometryCache geometryCache;
    geometryCache.build(shape, graph);
    return RecognizeThroughStructuresOCCT(
        shape, graph, recognizedHoleFaceIndices,
        geometryCache);
}

ThroughStructureRecognitionResult RecognizeThroughStructuresOCCT(
    const TopoDS_Shape& shape,
    AAG& graph,
    const std::unordered_set<int>& recognizedHoleFaceIndices,
    const RecognitionGeometryCache& geometryCache)
{
    ThroughStructureRecognitionResult result;
    if (shape.IsNull() || graph.faceMap.Extent() == 0)
        return result;

    // 与“识别基面”共用同一逐体 AABB 主外表面规则。联合入口直接复用
    // STEP 索引阶段已经得到的紧致包围盒。
    std::unordered_set<int> mainOuterFaceIndices;
    for (int faceIndex = 1;
         faceIndex <= graph.faceMap.Extent(); ++faceIndex)
        if (geometryCache.isMainOuterFace(faceIndex))
            mainOuterFaceIndices.insert(faceIndex);

    // 主流程：筛选垂直 Z 的候选面 -> 排除孔/主外表面 -> 对每条非 Z 向
    // 共享边检查凸性，全部满足才认定为通结构面。
    for (int faceIndex = 1; faceIndex <= graph.faceMap.Extent(); ++faceIndex) {
        const TopoDS_Face face = TopoDS::Face(graph.faceMap(faceIndex));

        try {
            if (!isSurfacePerpendicularToXoy(
                    face, faceIndex, geometryCache))
                continue;
        } catch (const Standard_Failure&) {
            continue;
        }

        ++result.verticalSurfaceCount;
        if (recognizedHoleFaceIndices.count(faceIndex) > 0) {
            ++result.excludedHoleFaceCount;
            continue;
        }
        if (mainOuterFaceIndices.count(faceIndex) > 0) {
            ++result.excludedMainOuterFaceCount;
            continue;
        }
        bool hasNonZNeighbor = false;
        bool allNonZEdgesConvex = true;

        // 直接复用逐体 edge -> faces 缓存。旧实现会为每个邻面重新枚举
        // 一次邻面的全部边，再做集合相交；这里每条本面边只访问一次。
        TopTools_IndexedMapOfShape faceEdges;
        TopExp::MapShapes(face, TopAbs_EDGE, faceEdges);
        const auto& edgeFaces = geometryCache.edgeFaces();
        for (int edgeIndex = 1;
             edgeIndex <= faceEdges.Extent(); ++edgeIndex) {
            const TopoDS_Edge edge =
                TopoDS::Edge(faceEdges(edgeIndex));
            if (!edgeFaces.Contains(edge))
                continue;
            const TopTools_ListOfShape& ancestors =
                edgeFaces.FindFromKey(edge);
            // AAG 只登记恰好由两个面共享的流形边；保持同一判定范围。
            if (ancestors.Extent() != 2)
                continue;
            int neighborIndex = 0;
            for (TopTools_ListIteratorOfListOfShape iterator(
                     ancestors);
                 iterator.More(); iterator.Next()) {
                if (iterator.Value().ShapeType() != TopAbs_FACE)
                    continue;
                const int candidateIndex =
                    graph.faceMap.FindIndex(iterator.Value());
                if (candidateIndex > 0 &&
                    candidateIndex != faceIndex) {
                    neighborIndex = candidateIndex;
                    break;
                }
            }
            if (neighborIndex < 1 ||
                neighborIndex > graph.faceMap.Extent()) {
                continue;
            }
            const TopoDS_Face neighbor =
                TopoDS::Face(graph.faceMap(neighborIndex));
            try {
                // 沿 Z 轴的共享边不参与通结构判定；只检查其余共享边。
                if (detail::isEdgeGeometricallyZDirected(edge))
                    continue;
                hasNonZNeighbor = true;
                if (!isConvexAlongEdge(face, neighbor, edge)) {
                    allNonZEdgesConvex = false;
                    break;
                }
            } catch (const Standard_Failure&) {
                // 无法可靠计算的共享边不能按凸边放行。
                hasNonZNeighbor = true;
                allNonZEdgesConvex = false;
                break;
            }
        }

        if (!hasNonZNeighbor) {
            ++result.rejectedWithoutNonZNeighborCount;
        } else if (!allNonZEdgesConvex) {
            ++result.rejectedNonConvexCount;
        } else {
            result.faceIndices.push_back(faceIndex);
        }
    }

    return result;
}

} // namespace oap

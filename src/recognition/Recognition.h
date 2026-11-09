// 基础识别算法接口及孔识别结果数据结构。
// 模块: oap_recognition（依赖 oap_analysis 的 SurfaceClassify）。
// 定义孔识别结果（HoleResult / HoleOpeningTransition）与各 OCCT 识别入口，
// 被 FeatureRecognizer 与 ThreeAxisWireTraceUseCase 消费。
#pragma once
#include <BRepAdaptor_Surface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <GeomLProp_SLProps.hxx>
#include <gp_Pnt.hxx>

#include <functional>

#include "AAG.h"

namespace oap {
class RecognitionGeometryCache;
}

// 孔类型：通孔 / 盲孔 / 沉孔 / 未知。识别顺序与面归属冲突规则
// 见 RecognizeHolesOCCT 的注释。
enum class HoleType
{
    THROUGH,
    BLIND,
    COUNTERSINK,
    UNKNOWN
};

// 描述孔口扩径过渡的几何结果，faceId 小于 1 表示未识别到过渡。
struct HoleOpeningTransition
{
    int faceId = -1;
    double outerRadius = 0.0;
    double innerParameter = 0.0;
    double outerParameter = 0.0;
    gp_Pnt outerPoint;
};

// 单个孔段识别结果：faces 为该孔结构面的 AAG 1-based 索引集合，mainFaceId
// 为孔壁圆柱面，axis / radius 描述孔轴几何；另附孔口采样点、底部平面/锥面
// 与开口过渡对象，供刀路构建使用。
struct HoleResult
{
    HoleType type = HoleType::UNKNOWN;
    std::vector<int> faces;
    int mainFaceId = -1;
    gp_Ax1 axis;
    double radius = 0.0;
    // 圆柱壁与孔口承载面凸边上的采样点，其轴向位置标识真实可达入口。
    bool hasOpeningPoint = false;
    gp_Pnt openingPoint;
    // 孔区域内可能作为圆柱段轴向终止面的平面/锥面。保留 AAG 的 1-based
    // 索引；刀路构建器还会要求与该圆柱拓扑邻接且位于 bottom 端，因此不会
    // 把外部开口面误判为孔底。
    std::vector<int> planarBottomFaceIds;
    std::vector<int> conicalBottomFaceIds;
    // STEP 常把沉头锥面或圆角过渡离散为 B-spline；该对象集中保存其几何。
    HoleOpeningTransition openingTransition;
};

using HoleCallback = std::function<void(const HoleResult&)>;

// 面的底层 OCCT 曲面类型（平面/圆柱/锥面/BSpline 等）。
GeomAbs_SurfaceType GetSurfaceType(const TopoDS_Face &face);

bool GetCylinderData(const TopoDS_Face &face,
                     gp_Ax1 &axis,
                     double &radius);

// 面的顶点数 / 边数，供孔识别几何门控与统计使用。
int GetVertexCount(const TopoDS_Face &face);

int GetEdgeCount(const TopoDS_Face &face);

// 完整圆柱（周向跨度≈2π）判定，孔识别的第一道几何门控。
bool IsFullCylinderFace(const TopoDS_Face &face);

// 使用全图通盲分类与孔轴投影端面分析结果返回 fid 所在的圆柱段分类。
// 单面查询仍会执行完整全图分析，批量识别应使用 RecognizeHolesOCCT。
HoleResult ClassifyHoleOCCT(const AAG &graph, int fid);

// 先按完整内凹圆柱几何门控收集候选；任一端部共享边为凹接即判盲孔，
// 否则判通孔。分类完成后再沿候选孔轴对等轴向位置的端边进行二次分析：
// 凹接邻面作为孔底，凸接邻面作为孔口承载面，并与圆柱壁共同组成孔结构。
// 主外表面仍参与孔口定位，但不加入孔结构面集。
// 与通孔或盲孔凸接的平面、BSpline 面、Bezier 面同样只参与孔口分析，
// 不加入孔结构面集。
// 通孔与盲孔结构发生面归属冲突时，该面只归属于盲孔。
// 多个盲孔争用同一面时，凹接该面的盲孔优先于仅凸接该面的盲孔。
// 平面盲端还承载较小平行圆柱孔，或孔口存在有效扩径过渡时，结果会提升为
// 沉孔；各圆柱候选仍独立生成，刀路关系层再协调同轴阶段与偏心平底子孔。
std::vector<HoleResult> RecognizeHolesOCCT(AAG &graph,
    HoleCallback callback = nullptr);

// 已由联合识别入口构建几何缓存时使用，避免孔识别重新组装面 Compound 并
// 重算主外表面。缓存的 faceMap 必须与 graph 来自同一次 AAG 构建。
std::vector<HoleResult> RecognizeHolesOCCT(
    AAG& graph,
    const oap::RecognitionGeometryCache& geometryCache,
    HoleCallback callback = nullptr);

bool GetSharedEdge(const TopoDS_Face &f1,
                   const TopoDS_Face &f2,
                   TopoDS_Edge &outEdge);

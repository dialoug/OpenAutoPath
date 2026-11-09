// SceneController 实现 — 参见 SceneController.h。
#include "app/presentation/SceneController.h"

#include <TopoDS_Face.hxx>
#include <Quantity_Color.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <algorithm>
#include <cmath>
#include <utility>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <SelectMgr_Selection.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Aspect_TypeOfDeflection.hxx>
#include <GeomAbs_Shape.hxx>
#include <Graphic3d_DisplayPriority.hxx>
#include <Graphic3d_MaterialAspect.hxx>
#include <QElapsedTimer>
#include <QDebug>

#include "app/view/DisplayMaterials.h"
#include "recognition/DihedralAngle.h"

namespace oap {
namespace {
const Handle(AIS_Shape) kNullAis;

// 边线叠加层线宽。
constexpr double kEdgeLineWidth = 1.0;
constexpr double kConvexityEdgeLineWidth = 2.5;
constexpr double kFeatureBoundaryLineWidth = 1.0;

Quantity_Color concaveEdgeColor() {
    return Quantity_Color(1.0, 0.08, 0.08, Quantity_TOC_sRGB);
}
Quantity_Color smoothEdgeColor() {
    return Quantity_Color(0.0, 0.25, 1.0, Quantity_TOC_sRGB);
}
Quantity_Color convexEdgeColor() {
    return Quantity_Color(0.05, 0.85, 0.18, Quantity_TOC_sRGB);
}

bool hasEncodedSmoothContinuity(const TopoDS_Edge& edge,
                                const TopoDS_Face& first,
                                const TopoDS_Face& second) {
    try {
        return BRep_Tool::HasContinuity(edge, first, second) &&
               BRep_Tool::Continuity(edge, first, second) >= GeomAbs_G1;
    } catch (...) {
        return false;
    }
}

// 中性灰回退色,与查看器默认基调一致。
Quantity_Color fallbackColor() {
    return Quantity_Color(0.78, 0.78, 0.80, Quantity_TOC_sRGB);
}

Quantity_Color fadedColor() {
    return Quantity_Color(0.86, 0.86, 0.88, Quantity_TOC_sRGB);
}

// 显示前的并行三角剖分:把全部待显示面合进一个 compound,用一个统一的"绝对"
// 弦高偏差(deflection)一次性并行剖分(BRepMesh 单面调用是单线程的,合批 + 开
// 并行才能吃满多核)。返回所用的绝对 deflection;调用方需把它设到每个 AIS_Shape
// 的 drawer 上(同样按绝对偏差),使 Display 复用已算好的网格而不再逐面重剖。
// 角度偏差与 deflection<=0(空/退化包围盒)等异常一律安全跳过,绝不影响显示正确性。
double preMeshFacesParallel(const std::vector<const TopoDS_Face*>& faces) {
    if (faces.empty()) return 0.0;
    TopoDS_Compound comp;
    BRep_Builder builder;
    builder.MakeCompound(comp);
    for (const TopoDS_Face* f : faces)
        if (f && !f->IsNull()) builder.Add(comp, *f);

    // 绝对 deflection 由整体包围盒对角线缩放得到(对齐 AIS 默认的相对系数 0.001
    // 量级),既保证全模型视觉精度一致,又是 BRepMesh 与各面 AIS 共用的同一阈值。
    Bnd_Box box;
    BRepBndLib::Add(comp, box);
    if (box.IsVoid()) return 0.0;
    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    const double dx = xmax - xmin, dy = ymax - ymin, dz = zmax - zmin;
    const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double deflection = diag * 0.001;
    if (!(deflection > 0.0)) return 0.0;

    constexpr double kAngularDeflection = 0.5; // 弧度,OCCT 显示默认量级
    // 构造函数即自动执行 Perform(),无需再显式调用。
    BRepMesh_IncrementalMesh mesher(comp, deflection, Standard_False,
                                    kAngularDeflection, Standard_True /*并行*/);
    (void)mesher;
    return deflection;
}

// 边线叠加层:承载全部去重边的单一 AIS_Shape。覆写 ComputeSelection 为空,
// 使其即便在 Edge/Wire 选择模式被 Activate 也不产生可拾取实体,避免与真实面的
// 边重叠造成拾取歧义(拾取仍只命中按面拆分的 face AIS)。
class EdgeOverlayShape : public AIS_Shape {
public:
    explicit EdgeOverlayShape(const TopoDS_Shape& sh) : AIS_Shape(sh) {}
    void ComputeSelection(const Handle(SelectMgr_Selection)&,
                          const Standard_Integer) override {}
};
} // namespace

void SceneController::clear() {
    if (!m_context.IsNull()) {
        for (auto& kv : m_faceAis)
            m_context->Remove(kv.second, Standard_False);
        if (!m_edgeOverlay.IsNull())
            m_context->Remove(m_edgeOverlay, Standard_False);
        if (!m_concaveEdgeOverlay.IsNull())
            m_context->Remove(m_concaveEdgeOverlay, Standard_False);
        if (!m_smoothEdgeOverlay.IsNull())
            m_context->Remove(m_smoothEdgeOverlay, Standard_False);
        if (!m_convexEdgeOverlay.IsNull())
            m_context->Remove(m_convexEdgeOverlay, Standard_False);
        if (!m_featureBoundaryOverlay.IsNull())
            m_context->Remove(m_featureBoundaryOverlay, Standard_False);
    }
    m_edgeOverlay.Nullify();
    m_concaveEdgeOverlay.Nullify();
    m_smoothEdgeOverlay.Nullify();
    m_convexEdgeOverlay.Nullify();
    m_featureBoundaryOverlay.Nullify();
    m_featureBoundaries.clear();
    m_edgeRecs.clear();
    m_edgeConvexityClassified = false;
    m_visibleConcaveEdgeCount = 0;
    m_visibleSmoothEdgeCount = 0;
    m_visibleConvexEdgeCount = 0;
    m_faceAis.clear();
    m_baseColor.clear();
    m_hidden.clear();
    m_model = nullptr;
    m_stepIndexer = nullptr;
    m_modelTransform = gp_Trsf();
}

void SceneController::setModelTransform(const gp_Trsf& transform) {
    m_modelTransform = transform;
    if (m_context.IsNull()) return;

    for (auto& kv : m_faceAis)
        kv.second->SetLocalTransformation(m_modelTransform);
    for (Handle(AIS_Shape)* overlay : {&m_edgeOverlay, &m_concaveEdgeOverlay,
                                       &m_smoothEdgeOverlay, &m_convexEdgeOverlay,
                                       &m_featureBoundaryOverlay}) {
        if (!overlay->IsNull())
            (*overlay)->SetLocalTransformation(m_modelTransform);
    }
}

void SceneController::displayModel(const Handle(AIS_InteractiveContext)& ctx,
                                   const Model& model, const StepIndexer& indexer,
                                   const Handle(XCAFDoc_ColorTool)& colorTool) {
    m_context = ctx;
    if (m_context.IsNull()) return;

    clear();        // 移除上一模型的按面 AIS;clear() 不会重置 m_context
    m_context = ctx;
    m_model = &model;
    m_stepIndexer = &indexer;

    // 先并行预剖分:把全部待显示面合批,用统一绝对 deflection 一次性多核三角化。
    // 之后逐面 Display 时,只要 AIS drawer 用同一绝对 deflection,网格即被复用而非
    // 单线程逐面重算(命中缓存后的主要 CPU 热点)。deflection<=0 表示跳过(退化包围盒)。
    QElapsedTimer dtimer; dtimer.start();
    std::vector<const TopoDS_Face*> topoFaces;
    topoFaces.reserve(model.faces.size());
    for (const auto& f : model.faces)
        topoFaces.push_back(indexer.faceForId(f.face_id));
    const double meshDeflection = preMeshFacesParallel(topoFaces);
    const qint64 tPreMesh = dtimer.restart();
    const Graphic3d_MaterialAspect faceMaterial =
        view::matteShapeMaterial();

    size_t faceIdx = 0;
    for (const auto& f : model.faces) {
        const TopoDS_Face* topo = topoFaces[faceIdx++];
        if (!topo) continue;

        Handle(AIS_Shape) ais = new AIS_Shape(*topo);
        ais->SetLocalTransformation(m_modelTransform);
        ais->SetDisplayMode(m_shaded ? AIS_Shaded : AIS_WireFrame);
        // 用与预剖分一致的绝对弦高偏差,使 Display 复用上面已算好的网格(不再逐面重剖)。
        // 绝对模式下 OCCT 取 MaximalChordialDeviation 作为 deflection,故只需设它。
        if (meshDeflection > 0.0) {
            ais->Attributes()->SetTypeOfDeflection(Aspect_TOD_ABSOLUTE);
            ais->Attributes()->SetMaximalChordialDeviation(meshDeflection);
        }

        // 颜色解析顺序(对齐 NX 的颜色继承):
        //   1) 逐面 XDE 颜色:表面色优先,其次通用色;
        //   2) 回退到所属 solid 的颜色(表面色 / 通用色)—— 多数面无单独颜色时靠它继承部件色;
        //   3) 仍无则用中性灰。
        // (本文件约 6 成的面没有逐面颜色,回退到 solid 色以与 NX 的"继承部件色"显示一致。)
        Quantity_Color col;
        bool gotColor = false;
        if (!colorTool.IsNull()) {
            if (colorTool->GetColor(*topo, XCAFDoc_ColorSurf, col) ||
                colorTool->GetColor(*topo, XCAFDoc_ColorGen, col)) {
                gotColor = true;
            } else {
                // 逐面查色失败 -> 回查该面所属 solid/shell 的颜色。face 记录自带 unit_id,
                // 无需另建 face_id->unit_id 映射(Face::unit_id 即来源,见 Model.h)。
                const TopoDS_Shape* owner = indexer.shapeForUnit(f.unit_id);
                if (owner &&
                    (colorTool->GetColor(*owner, XCAFDoc_ColorSurf, col) ||
                     colorTool->GetColor(*owner, XCAFDoc_ColorGen, col)))
                    gotColor = true;
            }
        }
        Quantity_Color base = gotColor ? col : fallbackColor();
        ais->SetColor(m_faded ? fadedColor() : base);
        ais->SetMaterial(faceMaterial);

        m_context->Display(ais, Standard_False);
        m_faceAis[f.face_id] = ais;
        m_baseColor[f.face_id] = base;  // 记录原色,供识别染色后恢复
    }
    const qint64 tFaceLoop = dtimer.restart();

    // 采集"边 + 其所属面"记录,并按当前可见性首次构建边线叠加层(方案 A+B)。
    buildEdgeOverlay(indexer);
    const qint64 tEdgeOverlay = dtimer.elapsed();

    qDebug().nospace()
        << "[OpenAutoPath] displayModel faces=" << (qulonglong)model.faces.size()
        << " edges=" << (qulonglong)model.edges.size()
        << "  ms: premesh=" << tPreMesh
        << " face_loop(display)=" << tFaceLoop
        << " edge_overlay=" << tEdgeOverlay
        << " total=" << (tPreMesh + tFaceLoop + tEdgeOverlay);
}

void SceneController::buildEdgeOverlay(const StepIndexer& indexer) {
    m_edgeRecs.clear();
    m_edgeOverlay.Nullify();
    m_concaveEdgeOverlay.Nullify();
    m_smoothEdgeOverlay.Nullify();
    m_convexEdgeOverlay.Nullify();
    m_edgeConvexityClassified = false;
    if (!m_model) return;
    // 用去重后的 Model.edges 采集每条边及其 face_ids(方案 B 判断"是否还有可见面")。
    m_edgeRecs.reserve(m_model->edges.size());
    for (const auto& e : m_model->edges) {
        const TopoDS_Edge* shape = indexer.edgeForId(e.edge_id);
        if (!shape) continue;
        m_edgeRecs.push_back(EdgeRec{*shape, e.face_ids,
                                     EdgeConvexityDisplay::Unknown});
    }
    rebuildEdgeOverlay();  // 按当前 m_edges / m_hidden 构建并(必要时)显示
    rebuildConvexityEdgeOverlays();
}

void SceneController::rebuildEdgeOverlay() {
    if (m_context.IsNull()) return;
    if (!m_edgeOverlay.IsNull()) {
        m_context->Remove(m_edgeOverlay, Standard_False);
        m_edgeOverlay.Nullify();
    }
    if (!m_edges || m_edgeRecs.empty()) return;

    TopoDS_Compound comp;
    BRep_Builder builder;
    builder.MakeCompound(comp);
    int n = 0;
    for (const auto& rec : m_edgeRecs) {
        // 方案 B:边只要还有一个可见面就保留;其全部所属面被隐藏时随之消失。
        bool anyVisible = rec.faceIds.empty();  // 无面归属的边(理论上不应出现)默认显示
        for (const auto& fid : rec.faceIds)
            if (m_hidden.find(fid) == m_hidden.end()) { anyVisible = true; break; }
        if (anyVisible) { builder.Add(comp, rec.shape); ++n; }
    }
    if (n == 0) return;

    m_edgeOverlay = new EdgeOverlayShape(comp);
    m_edgeOverlay->SetLocalTransformation(m_modelTransform);
    m_edgeOverlay->SetDisplayMode(AIS_WireFrame);
    m_edgeOverlay->SetColor(Quantity_NOC_BLACK);
    m_edgeOverlay->SetWidth(kEdgeLineWidth);
    m_edgeOverlay->Attributes()->SetWireAspect(
        new Prs3d_LineAspect(Quantity_NOC_BLACK, Aspect_TOL_SOLID, kEdgeLineWidth));
    m_context->Display(m_edgeOverlay, Standard_False);
}

void SceneController::classifyEdgeConvexity() {
    if (m_edgeConvexityClassified ||
        !m_stepIndexer) {
        return;
    }
    for (EdgeRec& rec : m_edgeRecs) {
        rec.convexity = EdgeConvexityDisplay::Unknown;
        if (rec.faceIds.size() != 2 || rec.faceIds[0] == rec.faceIds[1]) continue;
        const TopoDS_Face* first =
            m_stepIndexer->faceForId(
                rec.faceIds[0]);
        const TopoDS_Face* second =
            m_stepIndexer->faceForId(
                rec.faceIds[1]);
        if (!first || !second) continue;
        if (hasEncodedSmoothContinuity(rec.shape, *first, *second)) {
            rec.convexity = EdgeConvexityDisplay::Smooth;
            continue;
        }
        try {
            DihedralAngleChecker checker;
            checker.SetCommonEdge(rec.shape);
            const FeatureAngleType type =
                checker.AngleBetweenFaces(*first, *second, true, 0.0);
            if (FeatureAngle::IsSmooth(type)) rec.convexity = EdgeConvexityDisplay::Smooth;
            else if (FeatureAngle::IsConcave(type)) rec.convexity = EdgeConvexityDisplay::Concave;
            else if (FeatureAngle::IsConvex(type)) rec.convexity = EdgeConvexityDisplay::Convex;
        } catch (...) {
            rec.convexity = EdgeConvexityDisplay::Unknown;
        }
    }
    m_edgeConvexityClassified = true;
}

void SceneController::rebuildConvexityEdgeOverlays() {
    m_visibleConcaveEdgeCount = m_visibleSmoothEdgeCount = m_visibleConvexEdgeCount = 0;
    if (m_context.IsNull()) return;
    for (Handle(AIS_Shape)* overlay : {&m_concaveEdgeOverlay, &m_smoothEdgeOverlay,
                                       &m_convexEdgeOverlay}) {
        if (!overlay->IsNull()) {
            m_context->Remove(*overlay, Standard_False);
            overlay->Nullify();
        }
    }
    if (!m_convexityEdges || m_edgeRecs.empty()) return;
    classifyEdgeConvexity();

    TopoDS_Compound concave, smooth, convex;
    BRep_Builder builder;
    builder.MakeCompound(concave);
    builder.MakeCompound(smooth);
    builder.MakeCompound(convex);
    for (const EdgeRec& rec : m_edgeRecs) {
        bool visible = rec.faceIds.empty();
        for (const std::string& faceId : rec.faceIds)
            if (!m_hidden.contains(faceId)) { visible = true; break; }
        if (!visible) continue;
        switch (rec.convexity) {
            case EdgeConvexityDisplay::Concave:
                builder.Add(concave, rec.shape); ++m_visibleConcaveEdgeCount; break;
            case EdgeConvexityDisplay::Smooth:
                builder.Add(smooth, rec.shape); ++m_visibleSmoothEdgeCount; break;
            case EdgeConvexityDisplay::Convex:
                builder.Add(convex, rec.shape); ++m_visibleConvexEdgeCount; break;
            default: break;
        }
    }
    auto display = [&](const TopoDS_Compound& compound, const Quantity_Color& color,
                       Handle(AIS_Shape)& overlay) {
        overlay = new EdgeOverlayShape(compound);
        overlay->SetLocalTransformation(m_modelTransform);
        overlay->SetDisplayMode(AIS_WireFrame);
        overlay->SetColor(color);
        overlay->SetWidth(kConvexityEdgeLineWidth);
        overlay->Attributes()->SetWireAspect(
            new Prs3d_LineAspect(color, Aspect_TOL_SOLID, kConvexityEdgeLineWidth));
        m_context->Display(overlay, Standard_False);
        m_context->SetDisplayPriority(overlay, Graphic3d_DisplayPriority_Topmost);
    };
    if (m_visibleConcaveEdgeCount) display(concave, concaveEdgeColor(), m_concaveEdgeOverlay);
    if (m_visibleSmoothEdgeCount) display(smooth, smoothEdgeColor(), m_smoothEdgeOverlay);
    if (m_visibleConvexEdgeCount) display(convex, convexEdgeColor(), m_convexEdgeOverlay);
}

void SceneController::setFeatureBoundaries(
    const std::vector<FeatureBoundary>& boundaries) {
    m_featureBoundaries = boundaries;
    rebuildFeatureBoundaryOverlays();
}

void SceneController::clearFeatureBoundaries() {
    m_featureBoundaries.clear();
    rebuildFeatureBoundaryOverlays();
}

void SceneController::rebuildFeatureBoundaryOverlays() {
    if (m_context.IsNull())
        return;
    if (!m_featureBoundaryOverlay.IsNull()) {
        m_context->Remove(m_featureBoundaryOverlay, Standard_False);
        m_featureBoundaryOverlay.Nullify();
    }
    // 凹凸边具有更高的诊断优先级；开启期间保留子集数据但暂停白线显示。
    if (m_convexityEdges || m_featureBoundaries.empty() || m_edgeRecs.empty())
        return;

    std::vector<bool> boundaryEdges(m_edgeRecs.size(), false);
    for (const FeatureBoundary& boundary : m_featureBoundaries) {
        const std::unordered_set<std::string> faceIds(
            boundary.faceIds.begin(), boundary.faceIds.end());
        if (faceIds.empty())
            continue;

        for (std::size_t edgeIndex = 0;
             edgeIndex < m_edgeRecs.size(); ++edgeIndex) {
            const EdgeRec& edge = m_edgeRecs[edgeIndex];
            bool hasVisibleFeatureFace = false;
            bool hasOutsideOrHiddenFace = false;
            for (const std::string& faceId : edge.faceIds) {
                if (faceIds.contains(faceId)) {
                    if (m_hidden.contains(faceId))
                        hasOutsideOrHiddenFace = true;
                    else
                        hasVisibleFeatureFace = true;
                } else {
                    hasOutsideOrHiddenFace = true;
                }
            }

            const bool isFreeBoundary = edge.faceIds.size() == 1;
            if (hasVisibleFeatureFace &&
                (hasOutsideOrHiddenFace || isFreeBoundary)) {
                boundaryEdges[edgeIndex] = true;
            }
        }
    }

    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    int boundaryCount = 0;
    for (std::size_t edgeIndex = 0;
         edgeIndex < boundaryEdges.size(); ++edgeIndex) {
        if (!boundaryEdges[edgeIndex])
            continue;
        builder.Add(compound, m_edgeRecs[edgeIndex].shape);
        ++boundaryCount;
    }
    if (boundaryCount == 0)
        return;

    const Quantity_Color boundaryColor(Quantity_NOC_WHITE);
    m_featureBoundaryOverlay = new EdgeOverlayShape(compound);
    m_featureBoundaryOverlay->SetLocalTransformation(m_modelTransform);
    m_featureBoundaryOverlay->SetDisplayMode(AIS_WireFrame);
    m_featureBoundaryOverlay->SetColor(boundaryColor);
    m_featureBoundaryOverlay->SetWidth(kFeatureBoundaryLineWidth);
    m_featureBoundaryOverlay->Attributes()->SetWireAspect(
        new Prs3d_LineAspect(
            boundaryColor, Aspect_TOL_SOLID, kFeatureBoundaryLineWidth));
    m_context->Display(m_featureBoundaryOverlay, Standard_False);
    m_context->SetDisplayPriority(
        m_featureBoundaryOverlay, Graphic3d_DisplayPriority_Topmost);
}

const Handle(AIS_Shape)& SceneController::aisForFace(const std::string& faceId) const {
    auto it = m_faceAis.find(faceId);
    return it == m_faceAis.end() ? kNullAis : it->second;
}

std::vector<std::string> SceneController::facesForUnits(
    const std::vector<std::string>& unitIds) const {
    std::vector<std::string> out;
    if (!m_model) return out;
    std::unordered_set<std::string> want(unitIds.begin(), unitIds.end());
    for (const auto& u : m_model->units)
        if (want.contains(u.unit_id))
            out.insert(out.end(), u.face_ids.begin(), u.face_ids.end());
    return out;
}

void SceneController::hideFaces(const std::vector<std::string>& ids) {
    for (const auto& id : ids)
        if (m_faceAis.contains(id)) m_hidden.insert(id);
    applyVisibility();
}

void SceneController::onlyFaces(const std::vector<std::string>& ids) {
    std::unordered_set<std::string> keep(ids.begin(), ids.end());
    m_hidden.clear();
    for (const auto& kv : m_faceAis)
        if (!keep.contains(kv.first)) m_hidden.insert(kv.first);
    applyVisibility();
}

void SceneController::showFaces(const std::vector<std::string>& ids) {
    for (const auto& id : ids)
        m_hidden.erase(id);
    applyVisibility();
}

void SceneController::showAll() {
    m_hidden.clear();
    applyVisibility();
}

void SceneController::invertVisibility() {
    std::unordered_set<std::string> hidden;
    for (const auto& item : m_faceAis)
        if (!m_hidden.contains(item.first)) hidden.insert(item.first);
    m_hidden = std::move(hidden);
    applyVisibility();
}

void SceneController::restoreHidden(const std::unordered_set<std::string>& hidden) {
    // 用快照整体替换隐藏集。过滤掉当前模型已不存在的面 id(快照可能来自上一模型,
    // 虽然 MainWindow 在换模型时会清空撤回栈,这里仍做一次防御性过滤)。
    m_hidden.clear();
    for (const auto& id : hidden)
        if (m_faceAis.contains(id)) m_hidden.insert(id);
    applyVisibility();
}

void SceneController::setShaded(bool shaded) {
    m_shaded = shaded;
    if (m_context.IsNull()) return;
    const Standard_Integer mode = shaded ? AIS_Shaded : AIS_WireFrame;
    for (auto& kv : m_faceAis) {
        if (kv.second.IsNull()) continue;
        m_context->SetDisplayMode(kv.second, mode, Standard_False);
    }
    m_context->UpdateCurrentViewer();
}

void SceneController::setEdges(bool on) {
    m_edges = on;
    if (m_context.IsNull()) return;
    rebuildEdgeOverlay();   // 按当前可见性构建/移除边线层(方案 B 在隐藏面时已联动)
    m_context->UpdateCurrentViewer();
}

void SceneController::setConvexityEdges(bool on) {
    m_convexityEdges = on;
    if (m_context.IsNull()) return;
    rebuildConvexityEdgeOverlays();
    rebuildFeatureBoundaryOverlays();
    m_context->UpdateCurrentViewer();
}

void SceneController::setFaceColor(const std::string& faceId,
                                   const Quantity_Color& color) {
    auto it = m_faceAis.find(faceId);
    if (it == m_faceAis.end() || it->second.IsNull()) return;
    if (m_context.IsNull()) return;
    m_context->SetColor(it->second, color, Standard_False);
}

void SceneController::resetColors() {
    if (m_context.IsNull()) return;
    clearFeatureBoundaries();
    for (auto& kv : m_faceAis) {
        if (kv.second.IsNull()) continue;
        auto bit = m_baseColor.find(kv.first);
        const Quantity_Color col = m_faded
            ? fadedColor()
            : ((bit != m_baseColor.end()) ? bit->second : fallbackColor());
        m_context->SetColor(kv.second, col, Standard_False);
    }
    m_context->UpdateCurrentViewer();
}

void SceneController::setFaded(bool faded) {
    m_faded = faded;
    resetColors();
}

void SceneController::applyVisibility() {
    if (m_context.IsNull()) return;
    for (auto& kv : m_faceAis) {
        const bool hidden = m_hidden.count(kv.first) > 0;
        if (hidden) {
            if (m_context->IsDisplayed(kv.second))
                m_context->Erase(kv.second, Standard_False);
        } else {
            if (!m_context->IsDisplayed(kv.second))
                m_context->Display(kv.second, Standard_False);
        }
    }
    rebuildEdgeOverlay();  // 方案 B:可见面集合变化后,边线层随之只保留可见面的边
    rebuildConvexityEdgeOverlays();
    rebuildFeatureBoundaryOverlays();
    m_context->UpdateCurrentViewer();
}

} // namespace oap

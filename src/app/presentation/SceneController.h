// SceneController — 管理 3D 场景中“按面拆分”的 AIS 对象,并提供 face / unit 粒度的
// 显示隐藏(Hide / Only / Show / Show All)。隐藏即对该面的 AIS 对象做 Erase,
// 被 Erase 的对象不参与拾取,满足需求 §12.4“隐藏对象不可被拾取”。
#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <gp_Trsf.hxx>
#include <XCAFDoc_ColorTool.hxx>

#include "analysis/Model.h"
#include "analysis/StepIndexer.h"

namespace oap {

// 一个最终追踪结构。SceneController 提取其外轮廓边，并以统一白色绘制；
// 已合并盲腔内部原子子集之间的共享边不会成为外轮廓。
struct FeatureBoundary {
    std::vector<std::string> faceIds;
};

// 场景控制器：模型面 AIS 所有权、显示模式与可见性状态。
class SceneController {
public:
    // 清空场景并按面建立 AIS 对象(逐面回查 XDE 颜色,失败回退所属 solid 颜色 / 中性灰)。
    // 之后所有可见性操作都基于此 face_id -> AIS 映射。
    void displayModel(const Handle(AIS_InteractiveContext)& ctx,
                      const Model& model, const StepIndexer& indexer,
                      const Handle(XCAFDoc_ColorTool)& colorTool);

    void clear();

    // face 粒度可见性。ids 为 face_id 集合。
    void hideFaces(const std::vector<std::string>& ids);
    void showFaces(const std::vector<std::string>& ids);   // 把这些面从隐藏集移除(重新显示)
    void onlyFaces(const std::vector<std::string>& ids);   // 仅这些可见,其余隐藏
    void showAll();
    void invertVisibility();

    // 全局显示模式切换:true=着色(AIS_Shaded),false=线框(AIS_WireFrame)。
    // 作用于全部面 AIS,并记住状态供后续新建对象沿用。
    void setShaded(bool shaded);
    bool isShaded() const { return m_shaded; }

    // 边线叠加:着色模式下额外绘制面边界(Shaded with Edges),实现"线框+着色同时显示"。
    void setEdges(bool on);
    bool edgesOn() const { return m_edges; }

    void setConvexityEdges(bool on);
    bool convexityEdgesOn() const { return m_convexityEdges; }
    size_t visibleConcaveEdgeCount() const { return m_visibleConcaveEdgeCount; }
    size_t visibleSmoothEdgeCount() const { return m_visibleSmoothEdgeCount; }
    size_t visibleConvexEdgeCount() const { return m_visibleConvexEdgeCount; }

    // 把指定面染成给定颜色(特征识别高亮用)。其余面保持原色。
    void setFaceColor(const std::string& faceId, const Quantity_Color& color);
    void setFeatureBoundaries(const std::vector<FeatureBoundary>& boundaries);
    void clearFeatureBoundaries();
    // 恢复所有面到加载时的初始颜色(XDE 原色 / 中性灰回退),清除识别染色和白线。
    void resetColors();
    void setFaded(bool faded);
    bool faded() const { return m_faded; }

    // 固定相机旋转模型时，对模型面和边线应用统一显示变换。
    void setModelTransform(const gp_Trsf& transform);
    const gp_Trsf& modelTransform() const { return m_modelTransform; }

    // 把 unit_id 集合展开为其所属的全部 face_id —— 让 unit 粒度操作复用 face 实现。
    std::vector<std::string> facesForUnits(const std::vector<std::string>& unitIds) const;

    const Handle(AIS_Shape)& aisForFace(const std::string& faceId) const;
    bool isHidden(const std::string& faceId) const { return m_hidden.contains(faceId); }
    size_t hiddenCount() const { return m_hidden.size(); }
    size_t faceCount() const { return m_faceAis.size(); }

    // 可见性撤回支持:取当前隐藏集快照 / 用快照整体恢复(过滤掉已不存在的面后
    // 重建可见性与边线层)。供 MainWindow 的 Ctrl+Z 撤回栈使用。
    std::unordered_set<std::string> hiddenSnapshot() const { return m_hidden; }
    void restoreHidden(const std::unordered_set<std::string>& hidden);

private:
    void applyVisibility();  // 按 m_hidden 对每个面 Display / Erase
    void buildEdgeOverlay(const StepIndexer& indexer);  // 采集边-面映射并首次构建叠加层
    void rebuildEdgeOverlay();  // 方案 B:按当前隐藏集重建仅含可见面的边线层
    void classifyEdgeConvexity();
    void rebuildConvexityEdgeOverlays();
    void rebuildFeatureBoundaryOverlays();

    Handle(AIS_InteractiveContext)           m_context;
    const Model*                             m_model = nullptr;
    const StepIndexer*                       m_stepIndexer = nullptr;
    std::unordered_map<std::string, Handle(AIS_Shape)> m_faceAis;
    std::unordered_map<std::string, Quantity_Color>    m_baseColor; // 加载时的初始色,供 resetColors 恢复
    std::unordered_set<std::string>                    m_hidden;
    gp_Trsf                                            m_modelTransform;
    Handle(AIS_Shape)                                  m_edgeOverlay; // 去重边线叠加层(单一对象)
    Handle(AIS_Shape)                                  m_concaveEdgeOverlay;
    Handle(AIS_Shape)                                  m_smoothEdgeOverlay;
    Handle(AIS_Shape)                                  m_convexEdgeOverlay;
    Handle(AIS_Shape)                                  m_featureBoundaryOverlay;
    std::vector<FeatureBoundary>                       m_featureBoundaries;
    // 方案 B 用:每条去重边及其所属 face_ids。重建叠加层时,只纳入"至少有一个
    // 可见面"的边,使隐藏面时其专属边线随之消失,共享边只要还有可见面就保留。
    enum class EdgeConvexityDisplay { Unknown, Concave, Smooth, Convex };
    // 边记录：边凸性分类的显示条目。
    struct EdgeRec {
        TopoDS_Edge shape;
        std::vector<std::string> faceIds;
        EdgeConvexityDisplay convexity = EdgeConvexityDisplay::Unknown;
    };
    std::vector<EdgeRec>                               m_edgeRecs;
    bool                                               m_edgeConvexityClassified = false;
    size_t                                             m_visibleConcaveEdgeCount = 0;
    size_t                                             m_visibleSmoothEdgeCount = 0;
    size_t                                             m_visibleConvexEdgeCount = 0;
    bool                                               m_shaded = true; // 全局显示模式,默认着色
    bool                                               m_edges = true;  // 默认显示边线
    bool                                               m_convexityEdges = false;
    bool                                               m_faded = true;
};

} // namespace oap

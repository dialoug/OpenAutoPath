// 播放光标、轨迹与剩余路径的语义化 OCCT 呈现器。
// 模块:oap_app_presentation。刀尖光标由“球体(AIS_Shape)+ 中心点(AIS_Point)”组成,
// 剩余路径使用轻量 GPU 线段对象;全部对象经 OverlayController 的
// PlaybackCursor / PlaybackRemainingPath / PlaybackTrail 覆盖组管理。
#pragma once

#include <vector>

#include <AIS_InteractiveContext.hxx>
#include <AIS_Point.hxx>
#include <AIS_Shape.hxx>
#include <Geom_CartesianPoint.hxx>
#include <Quantity_Color.hxx>
#include <V3d_View.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include "app/presentation/detail/PlaybackRemainingPathObject.h"
#include "app/presentation/PlaybackOverlayPolicy.h"

namespace oap::app::presentation {

class OverlayController;

// 播放呈现器:随播放进度更新刀尖光标与剩余路径。所有 AIS 句柄归
// OverlayController 覆盖组所有,本类只持有非拥有指针与弱引用句柄
// (m_context / m_view 仅为 Display/Redraw 复用,不负责释放)。
class PlaybackPresenter {
public:
    PlaybackPresenter(
        OverlayController& overlays,
        const Handle(AIS_InteractiveContext)& context,
        const Handle(V3d_View)& view);

    // 重新绑定延迟创建的 OCCT context / view(查看器初始化后调用)。
    void bindViewer(
        const Handle(AIS_InteractiveContext)& context,
        const Handle(V3d_View)& view);
    // 清空全部播放相关覆盖组(剩余路径 + 轨迹 + 光标)。
    void clear();
    void clearCursorAndTrail();
    void clearCursor();
    void clearRemainingPath();
    bool hasRemainingPath() const;
    // 查询当前剩余路径的实际显示范围及工序过滤结果,不暴露私有 AIS 类型。
    bool remainingSegmentVisible(int moveIndex) const;
    // 按播放策略设置静态覆盖层(Toolpath/CurrentToolpath/HoleStageHighlight)的可见性。
    void setStaticOverlayVisibility(
        const PlaybackStaticOverlayVisibility& visibility);
    // 把刀路相关覆盖组统一切到指定 Z 层(避免被其它对象遮挡)。
    void applyToolpathDisplayLayer(int zLayer);

    // 更新刀尖光标:按刀具直径重建球体(直径变化时才重建),并把球/点放到
    // tip 位置(叠加 modelTransform 后);点对象复用同一 Geom_CartesianPoint。
    void updateTipMarker(
        const gp_Pnt& tip,
        double toolDiameter,
        const gp_Trsf& modelTransform,
        const Quantity_Color& color,
        double pointSize);
    // 更新剩余路径:替换或复用 PlaybackRemainingPathObject 并设置进度
    // [firstIndex, endIndex) 与首段 fraction;区间为空时隐藏。
    void updateRemainingPath(
        std::vector<PlaybackRemainingSegment> segments,
        int firstIndex,
        int endIndex,
        double fraction,
        const gp_Trsf& modelTransform,
        int zLayer);
    void setRemainingPathVisible(bool visible);
    void setRemainingSegmentVisibility(const std::vector<bool>& visibility);
    // 模型坐标变换改变时同步给剩余路径与光标对象。
    void applyModelTransform(const gp_Trsf& transform);

    Handle(AIS_Shape) tipSpherePresentation() const;
    Handle(AIS_Point) tipPointPresentation() const;

private:
    Handle(PlaybackRemainingPathObject) remainingPathPresentation() const;

    OverlayController* m_overlays = nullptr;
    Handle(AIS_InteractiveContext) m_context;
    Handle(V3d_View) m_view;
    Handle(Geom_CartesianPoint) m_tipPointGeometry;
    double m_tipSphereRadius = 0.0;
};

} // namespace oap::app::presentation

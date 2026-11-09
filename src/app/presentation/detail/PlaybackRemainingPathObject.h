// 播放期间“刀尖之后剩余路径”的轻量 GPU/AIS 呈现对象。
// 模块私有实现(detail)。持有全部剩余线段,但渲染时只画 [firstIndex, endIndex)
// 区间;首段可按 fraction 做插值起点,实现随播放进度动态缩短的效果。
#pragma once

#include <vector>

#include <AIS_InteractiveObject.hxx>
#include <Quantity_Color.hxx>
#include <gp_Pnt.hxx>

namespace oap::app::presentation {

// 线段样式:切削段 / 快移段 / 进退刀段,渲染时各自独立成组并设不同线宽。
enum class PlaybackRemainingLineStyle {
    Cut,
    Rapid,
    EntryRetract,
};

// 一条剩余路径线段:颜色按原刀路语义携带;outlined 表示额外绘制黑色描边。
struct PlaybackRemainingSegment {
    gp_Pnt from;
    gp_Pnt to;
    Quantity_Color color;
    PlaybackRemainingLineStyle style = PlaybackRemainingLineStyle::Cut;
    bool outlined = false;
    // 保留原移动索引;隐藏工序只能屏蔽线段,不能从数组中删除。
    bool visible = true;
};

// 剩余路径呈现对象:仅实现 mode-0 线段绘制,不做拾取(ComputeSelection 为空)。
class PlaybackRemainingPathObject final : public AIS_InteractiveObject {
public:
    explicit PlaybackRemainingPathObject(
        std::vector<PlaybackRemainingSegment> segments);

    // 设置可见区间与首段进度(均钳制到合法范围);无实际变化时不触发重绘。
    // firstIndex/endIndex 是半开区间 [first, end),fraction 是首段起点比例。
    void setProgress(int firstIndex, int endIndex, double fraction);
    void setSegmentVisibility(const std::vector<bool>& visibility);
    bool segmentVisible(int moveIndex) const noexcept;

private:
    void Compute(
        const Handle(PrsMgr_PresentationManager)&,
        const Handle(Prs3d_Presentation)& presentation,
        Standard_Integer mode) override;
    void ComputeSelection(
        const Handle(SelectMgr_Selection)&,
        Standard_Integer mode) override;

    std::vector<PlaybackRemainingSegment> m_segments;
    int m_firstIndex = 0;
    int m_endIndex = 0;
    double m_fraction = 1.0;
};

} // namespace oap::app::presentation

// 普通刀路的轻量 GPU/AIS 呈现对象。
//
// 与只负责播放剩余路径的 PlaybackRemainingPathObject 不同，本对象保留
// 每条显示线段的工序身份，并为可见线段建立独立拾取 owner。这样视图层
// 可以在不创建 TopoDS_Edge 的情况下完成悬停、点击和方向提示。
#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include <AIS_InteractiveObject.hxx>
#include <Quantity_Color.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <gp_Pnt.hxx>

#include "app/view/HolePreviewGeometryBuilder.h"

namespace oap::app::presentation {

// 刀路预览端点对：线段两端点与样式身份。
struct ToolpathPreviewEndpointPair {
    gp_Pnt start;
    gp_Pnt end;
};

// 一条预览线段可追溯到的加工上下文。字段为 -1/空串时表示来源没有提供
// 对应信息；呈现层不会据此改变实际程序或后处理结果。
struct ToolpathSegmentIdentity {
    int moveIndex = -1;
    int operationMoveIndex = -1;
    int primitiveIndex = -1;
    int primitiveMoveIndex = -1;
    int primitiveMoveCount = 0;
    int sourceIndex = -1;
    int setupIndex = -1;
    int toolNumber = -1;
    int toolBatchIndex = -1;
    int loopSequence = -1;
    int depthIndex = -1;
    int ring = -1;
    double feed = 0.0;
    std::string operationId;
    std::string commandName;
    std::string commandSummary;
    std::string featureName;
    std::string toolName;
    std::string toolType;
};

// 刀路预览线段：起止点、类型与身份。
struct ToolpathPreviewSegment {
    gp_Pnt from;
    gp_Pnt to;
    appdetail::HolePreviewSegmentKind kind =
        appdetail::HolePreviewSegmentKind::Cut;
    ToolpathSegmentIdentity identity;
};

// 普通刀路线宽、颜色、节点和端点样式。
struct ToolpathPreviewStyle {
    Quantity_Color cutColor;
    Quantity_Color rapidColor;
    Quantity_Color verticalColor;
    Quantity_Color outlineColor;
    Quantity_Color startColor;
    Quantity_Color endColor;
    Quantity_Color directionColor = Quantity_Color(1.0, 0.65, 0.0,
                                                     Quantity_TOC_sRGB);
    double cutWidth = 1.5;
    double rapidWidth = 1.0;
    double verticalWidth = 1.0;
    double outlineWidth = 1.6;
    double endpointSize = 6.0;
    double nodeSize = 3.0;
    double directionWidth = 3.0;
    int zLayer = 0;
    bool outlineRapid = true;
    bool showEndpoints = true;
    bool showNodes = false;
};

// 刀路线段的 GPU/AIS 呈现对象：持线段缓冲与样式、可见区间。
class ToolpathSegmentsObject final : public AIS_InteractiveObject {
    DEFINE_STANDARD_RTTIEXT(ToolpathSegmentsObject, AIS_InteractiveObject)
public:
    ToolpathSegmentsObject(
        std::vector<ToolpathPreviewSegment> segments,
        std::vector<ToolpathPreviewEndpointPair> endpoints,
        ToolpathPreviewStyle style);

    int segmentCount() const noexcept
    {
        return static_cast<int>(m_segments.size());
    }
    const ToolpathPreviewSegment* segment(int index) const noexcept;
    bool isSegmentVisible(int index) const noexcept
    {
        return index >= m_firstIndex && index < m_endIndex &&
            operationVisible(m_segments[static_cast<std::size_t>(index)]);
    }
    const ToolpathPreviewStyle& style() const noexcept { return m_style; }

    // 设置局部可见区间 [firstIndex, firstIndex + count)。count < 0 表示
    // 一直显示到末尾。范围变化同时更新显示与敏感线段。
    void setVisibleRange(int firstIndex, int count);
    void setShowNodes(bool show);
    void setOperationFilter(
        std::unordered_set<std::string> hiddenOperationIds,
        std::string soloOperationId);

private:
    // This object owns per-segment picking in OcctView. Once
    // SetAutoHilight(false) is used OCCT requires these callbacks; the base
    // implementations throw Standard_NotImplemented on hover/selection.
    // They intentionally do not draw an OCCT immediate presentation because
    // OcctView owns a separate lightweight hover presentation.
    void HilightOwnerWithColor(
        const Handle(PrsMgr_PresentationManager)& presentationManager,
        const Handle(Prs3d_Drawer)& style,
        const Handle(SelectMgr_EntityOwner)& owner) override;
    void ClearDynamicHighlight(
        const Handle(PrsMgr_PresentationManager)& presentationManager) override;
    void HilightSelected(
        const Handle(PrsMgr_PresentationManager)& presentationManager,
        const SelectMgr_SequenceOfOwner& owners) override;
    void ClearSelected() override;

    void Compute(
        const Handle(PrsMgr_PresentationManager)&,
        const Handle(Prs3d_Presentation)& presentation,
        Standard_Integer mode) override;
    void ComputeSelection(
        const Handle(SelectMgr_Selection)& selection,
        Standard_Integer mode) override;
    bool operationVisible(
        const ToolpathPreviewSegment& segment) const noexcept;

    std::vector<ToolpathPreviewSegment> m_segments;
    std::vector<ToolpathPreviewEndpointPair> m_endpoints;
    ToolpathPreviewStyle m_style;
    int m_firstIndex = 0;
    int m_endIndex = 0;
    std::unordered_set<std::string> m_hiddenOperationIds;
    std::string m_soloOperationId;
};

DEFINE_STANDARD_HANDLE(ToolpathSegmentsObject, AIS_InteractiveObject)

// A transient six-vertex presentation for the currently detected toolpath
// segment. Keeping it separate ensures hover updates never rebuild the main
// toolpath arrays or their per-segment selection structures.
class ToolpathHoverObject final : public AIS_InteractiveObject {
    DEFINE_STANDARD_RTTIEXT(ToolpathHoverObject, AIS_InteractiveObject)
public:
    ToolpathHoverObject();

    void setSegment(
        const ToolpathPreviewSegment& segment,
        const ToolpathPreviewStyle& style);
    void clearSegment();
    bool hasSegment() const noexcept { return m_hasSegment; }
    const ToolpathPreviewSegment* segment() const noexcept
    {
        return m_hasSegment ? &m_segment : nullptr;
    }

private:
    void Compute(
        const Handle(PrsMgr_PresentationManager)&,
        const Handle(Prs3d_Presentation)& presentation,
        Standard_Integer mode) override;
    void ComputeSelection(
        const Handle(SelectMgr_Selection)& selection,
        Standard_Integer mode) override;

    ToolpathPreviewSegment m_segment;
    Quantity_Color m_color =
        Quantity_Color(1.0, 0.65, 0.0, Quantity_TOC_sRGB);
    double m_width = 3.0;
    bool m_hasSegment = false;
};

DEFINE_STANDARD_HANDLE(ToolpathHoverObject, AIS_InteractiveObject)

// 每条敏感线段的独立 owner。OcctView 通过该类型取得稳定的段下标，
// 不依赖 AIS_Shape/StdSelect_BRepOwner 的拓扑子形状映射。
class ToolpathSegmentOwner final : public SelectMgr_EntityOwner {
    DEFINE_STANDARD_RTTIEXT(ToolpathSegmentOwner, SelectMgr_EntityOwner)
public:
    ToolpathSegmentOwner(
        const Handle(SelectMgr_SelectableObject)& selectable,
        int segmentIndex);

    int segmentIndex() const noexcept { return m_segmentIndex; }

private:
    int m_segmentIndex = -1;
};

DEFINE_STANDARD_HANDLE(ToolpathSegmentOwner, SelectMgr_EntityOwner)

} // namespace oap::app::presentation

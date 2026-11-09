// 完整/当前刀路预览线组的语义化 OCCT 呈现器。
// 模块:oap_app_presentation。把刀路段聚合成一个可拾取的轻量 GPU 对象，并
// 附带端点、节点与来源身份；对象归调用方指定的 OverlayGroup。
#pragma once

#include <vector>
#include <string>
#include <unordered_set>

#include <gp_Trsf.hxx>

#include "app/presentation/OverlayController.h"
#include "app/presentation/ToolpathSegmentsObject.h"
#include "app/view/HolePreviewGeometryBuilder.h"

namespace oap::app::presentation {

// 呈现统计:各类边数量与发布的 AIS 对象数,供上层判断本次更新是否生效。
struct ToolpathPreviewPresentationStats {
    int cutEdges = 0;
    int rapidEdges = 0;
    int verticalEdges = 0;
    int objectCount = 0;
};

// 刀路预览呈现器:replace() 整组替换、append() 向组内追加,均接受任意
// OverlayGroup(通常为 Toolpath / CurrentToolpath)。不持有 AIS 句柄所有权。
class ToolpathPreviewPresenter {
public:
    explicit ToolpathPreviewPresenter(OverlayController& overlays) noexcept;

    // 清空指定组。
    void clear(OverlayGroup group);
    // 整组替换:按样式构建线框对象并替换 group 的现有内容。
    ToolpathPreviewPresentationStats replace(
        OverlayGroup group,
        const std::vector<appdetail::HolePreviewEdge>& edges,
        const std::vector<ToolpathPreviewEndpointPair>& endpoints,
        const gp_Trsf& modelTransform,
        const ToolpathPreviewStyle& style,
        bool visible = true);
    ToolpathPreviewPresentationStats replace(
        OverlayGroup group,
        const std::vector<ToolpathPreviewSegment>& segments,
        const std::vector<ToolpathPreviewEndpointPair>& endpoints,
        const gp_Trsf& modelTransform,
        const ToolpathPreviewStyle& style,
        bool visible = true);
    // 追加到组末尾(不清除既有对象)。
    ToolpathPreviewPresentationStats append(
        OverlayGroup group,
        const std::vector<appdetail::HolePreviewEdge>& edges,
        const std::vector<ToolpathPreviewEndpointPair>& endpoints,
        const gp_Trsf& modelTransform,
        const ToolpathPreviewStyle& style);
    ToolpathPreviewPresentationStats append(
        OverlayGroup group,
        const std::vector<ToolpathPreviewSegment>& segments,
        const std::vector<ToolpathPreviewEndpointPair>& endpoints,
        const gp_Trsf& modelTransform,
        const ToolpathPreviewStyle& style);

    // 以整组线段序号设置普通刀路的显示窗口；一个组内有多个追加对象时
    // 自动换算为各对象的局部区间。count < 0 表示显示到组末尾。
    void setDisplayRange(
        OverlayGroup group, int firstIndex, int count, bool showNodes);
    void setOperationFilter(
        OverlayGroup group,
        const std::unordered_set<std::string>& hiddenOperationIds,
        const std::string& soloOperationId);
    int segmentCount(OverlayGroup group) const;

private:
    ToolpathPreviewPresentationStats present(
        OverlayGroup group,
        const std::vector<ToolpathPreviewSegment>& segments,
        const std::vector<ToolpathPreviewEndpointPair>& endpoints,
        const gp_Trsf& modelTransform,
        const ToolpathPreviewStyle& style,
        bool visible,
        bool appendToGroup);

    OverlayController* m_overlays = nullptr;
};

} // namespace oap::app::presentation

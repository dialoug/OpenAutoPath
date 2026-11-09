// ToolpathPreviewPresenter 实现（oap_app_presentation）。
// 完整/当前刀路预览线组的语义化 OCCT 呈现器，见 ToolpathPreviewPresenter.h。
#include "ToolpathPreviewPresenter.h"

#include <algorithm>

#include <Graphic3d_DisplayPriority.hxx>
#include <BRep_Tool.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopoDS_Vertex.hxx>

namespace oap::app::presentation {

ToolpathPreviewPresenter::ToolpathPreviewPresenter(
    OverlayController& overlays) noexcept
    : m_overlays(&overlays)
{
}

void ToolpathPreviewPresenter::clear(OverlayGroup group)
{
    if (m_overlays)
        m_overlays->clear(group);
}

ToolpathPreviewPresentationStats ToolpathPreviewPresenter::replace(
    OverlayGroup group,
    const std::vector<appdetail::HolePreviewEdge>& edges,
    const std::vector<ToolpathPreviewEndpointPair>& endpoints,
    const gp_Trsf& modelTransform,
    const ToolpathPreviewStyle& style,
    bool visible)
{
    std::vector<ToolpathPreviewSegment> segments;
    segments.reserve(edges.size());
    for (const appdetail::HolePreviewEdge& item : edges) {
        if (item.edge.IsNull())
            continue;
        TopoDS_Vertex first;
        TopoDS_Vertex last;
        TopExp::Vertices(item.edge, first, last, Standard_True);
        if (first.IsNull() || last.IsNull())
            continue;
        segments.push_back({
            BRep_Tool::Pnt(first), BRep_Tool::Pnt(last), item.kind, {}});
    }
    return replace(
        group, segments, endpoints, modelTransform, style, visible);
}

ToolpathPreviewPresentationStats ToolpathPreviewPresenter::replace(
    OverlayGroup group,
    const std::vector<ToolpathPreviewSegment>& segments,
    const std::vector<ToolpathPreviewEndpointPair>& endpoints,
    const gp_Trsf& modelTransform,
    const ToolpathPreviewStyle& style,
    bool visible)
{
    return present(
        group, segments, endpoints, modelTransform, style,
        visible, false);
}

ToolpathPreviewPresentationStats ToolpathPreviewPresenter::append(
    OverlayGroup group,
    const std::vector<appdetail::HolePreviewEdge>& edges,
    const std::vector<ToolpathPreviewEndpointPair>& endpoints,
    const gp_Trsf& modelTransform,
    const ToolpathPreviewStyle& style)
{
    std::vector<ToolpathPreviewSegment> segments;
    segments.reserve(edges.size());
    for (const appdetail::HolePreviewEdge& item : edges) {
        if (item.edge.IsNull())
            continue;
        TopoDS_Vertex first;
        TopoDS_Vertex last;
        TopExp::Vertices(item.edge, first, last, Standard_True);
        if (first.IsNull() || last.IsNull())
            continue;
        segments.push_back({
            BRep_Tool::Pnt(first), BRep_Tool::Pnt(last), item.kind, {}});
    }
    return append(group, segments, endpoints, modelTransform, style);
}

ToolpathPreviewPresentationStats ToolpathPreviewPresenter::append(
    OverlayGroup group,
    const std::vector<ToolpathPreviewSegment>& segments,
    const std::vector<ToolpathPreviewEndpointPair>& endpoints,
    const gp_Trsf& modelTransform,
    const ToolpathPreviewStyle& style)
{
    return present(
        group, segments, endpoints, modelTransform, style,
        true, true);
}

ToolpathPreviewPresentationStats ToolpathPreviewPresenter::present(
    OverlayGroup group,
    const std::vector<ToolpathPreviewSegment>& segments,
    const std::vector<ToolpathPreviewEndpointPair>& endpoints,
    const gp_Trsf& modelTransform,
    const ToolpathPreviewStyle& style,
    bool visible,
    bool appendToGroup)
{
    ToolpathPreviewPresentationStats stats;
    if (!m_overlays)
        return stats;

    for (const ToolpathPreviewSegment& item : segments) {
        switch (item.kind) {
            case appdetail::HolePreviewSegmentKind::Cut:
                ++stats.cutEdges;
                break;
            case appdetail::HolePreviewSegmentKind::Rapid:
                ++stats.rapidEdges;
                break;
            case appdetail::HolePreviewSegmentKind::Vertical:
                ++stats.verticalEdges;
                break;
        }
    }

    std::vector<Handle(AIS_InteractiveObject)> objects;
    try {
        OCC_CATCH_SIGNALS
        if (stats.cutEdges + stats.rapidEdges + stats.verticalEdges > 0) {
            Handle(ToolpathSegmentsObject) object =
                new ToolpathSegmentsObject(segments, endpoints, style);
            object->SetLocalTransformation(modelTransform);
            objects.push_back(object);
        }
    } catch (const Standard_Failure&) {
        objects.clear();
    } catch (...) {
        objects.clear();
    }

    stats.objectCount = static_cast<int>(objects.size());
    if (appendToGroup) {
        for (Handle(AIS_InteractiveObject)& object : objects)
            m_overlays->append(
                group, std::move(object),
                OverlayDisplayOptions{false,
                    static_cast<int>(Graphic3d_DisplayPriority_Above)});
    } else {
        m_overlays->replace(
            group, std::move(objects), visible,
            OverlayDisplayOptions{false,
                static_cast<int>(Graphic3d_DisplayPriority_Above)});
    }
    return stats;
}

void ToolpathPreviewPresenter::setDisplayRange(
    OverlayGroup group, int firstIndex, int count, bool showNodes)
{
    if (!m_overlays)
        return;
    const int total = segmentCount(group);
    const int first = std::clamp(firstIndex, 0, total);
    const int end = count < 0
        ? total
        : std::clamp(first + std::max(0, count), first, total);
    int objectOffset = 0;
    for (std::size_t index = 0; index < m_overlays->size(group); ++index) {
        Handle(ToolpathSegmentsObject) object =
            Handle(ToolpathSegmentsObject)::DownCast(
                m_overlays->at(group, index));
        if (object.IsNull())
            continue;
        const int objectCount = object->segmentCount();
        const int localFirst = std::clamp(
            first - objectOffset, 0, objectCount);
        const int localEnd = std::clamp(
            end - objectOffset, localFirst, objectCount);
        object->setVisibleRange(localFirst, localEnd - localFirst);
        object->setShowNodes(showNodes);
        objectOffset += objectCount;
    }
    m_overlays->redisplay(group);
}

void ToolpathPreviewPresenter::setOperationFilter(
    OverlayGroup group,
    const std::unordered_set<std::string>& hiddenOperationIds,
    const std::string& soloOperationId)
{
    if (!m_overlays)
        return;
    for (std::size_t index = 0; index < m_overlays->size(group); ++index) {
        Handle(ToolpathSegmentsObject) object =
            Handle(ToolpathSegmentsObject)::DownCast(
                m_overlays->at(group, index));
        if (!object.IsNull()) {
            object->setOperationFilter(
                hiddenOperationIds, soloOperationId);
        }
    }
    m_overlays->redisplay(group);
}

int ToolpathPreviewPresenter::segmentCount(OverlayGroup group) const
{
    if (!m_overlays)
        return 0;
    int total = 0;
    for (std::size_t index = 0; index < m_overlays->size(group); ++index) {
        const Handle(ToolpathSegmentsObject) object =
            Handle(ToolpathSegmentsObject)::DownCast(
                m_overlays->at(group, index));
        if (!object.IsNull())
            total += object->segmentCount();
    }
    return total;
}

} // namespace oap::app::presentation

// ToolpathSegmentsObject 实现（oap_app_presentation）。
// 普通刀路的轻量 GPU/AIS 呈现对象，见 ToolpathSegmentsObject.h。
#include "app/presentation/ToolpathSegmentsObject.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Aspect_TypeOfLine.hxx>
#include <Aspect_TypeOfMarker.hxx>
#include <Graphic3d_ArrayOfPoints.hxx>
#include <Graphic3d_ArrayOfSegments.hxx>
#include <Graphic3d_AspectLine3d.hxx>
#include <Graphic3d_AspectMarker3d.hxx>
#include <Graphic3d_Group.hxx>
#include <Prs3d_Presentation.hxx>
#include <Select3D_SensitiveSegment.hxx>
#include <SelectMgr_Selection.hxx>
#include <gp_Vec.hxx>

namespace oap::app::presentation {

IMPLEMENT_STANDARD_RTTIEXT(ToolpathSegmentsObject, AIS_InteractiveObject)
IMPLEMENT_STANDARD_RTTIEXT(ToolpathHoverObject, AIS_InteractiveObject)
IMPLEMENT_STANDARD_RTTIEXT(ToolpathSegmentOwner, SelectMgr_EntityOwner)

namespace {

constexpr double kMinimumSegmentLength = 1.0e-9;

bool isFinite(const gp_Pnt& point) noexcept
{
    return std::isfinite(point.X()) && std::isfinite(point.Y()) &&
           std::isfinite(point.Z());
}

void displayLines(
    const Handle(Prs3d_Presentation)& presentation,
    const Handle(Graphic3d_ArrayOfSegments)& segments,
    const Quantity_Color& color,
    double width)
{
    if (segments.IsNull() || segments->VertexNumber() <= 0)
        return;
    const Handle(Graphic3d_Group) group = presentation->NewGroup();
    group->SetPrimitivesAspect(new Graphic3d_AspectLine3d(
        color, Aspect_TOL_SOLID, std::max(0.1, width)));
    group->AddPrimitiveArray(segments);
}

void displayPoints(
    const Handle(Prs3d_Presentation)& presentation,
    const Handle(Graphic3d_ArrayOfPoints)& points,
    const Quantity_Color& color,
    double size)
{
    if (points.IsNull() || points->VertexNumber() <= 0)
        return;
    const Handle(Graphic3d_Group) group = presentation->NewGroup();
    group->SetPrimitivesAspect(new Graphic3d_AspectMarker3d(
        Aspect_TOM_POINT, color, std::max(1.0, size)));
    group->AddPrimitiveArray(points);
}

} // namespace

ToolpathSegmentsObject::ToolpathSegmentsObject(
    std::vector<ToolpathPreviewSegment> segments,
    std::vector<ToolpathPreviewEndpointPair> endpoints,
    ToolpathPreviewStyle style)
    : m_segments(std::move(segments)),
      m_endpoints(std::move(endpoints)),
      m_style(std::move(style)),
      m_endIndex(static_cast<int>(m_segments.size()))
{
    SetDisplayMode(0);
    SetZLayer(m_style.zLayer);
    // 动态悬停由 OcctView 按 owner 下标更新本对象，禁止 OCCT 默认把整条
    // 刀路作为一个整体高亮。
    SetAutoHilight(Standard_False);
}

const ToolpathPreviewSegment* ToolpathSegmentsObject::segment(
    int index) const noexcept
{
    if (index < 0 || index >= static_cast<int>(m_segments.size()))
        return nullptr;
    return &m_segments[static_cast<std::size_t>(index)];
}

void ToolpathSegmentsObject::setVisibleRange(int firstIndex, int count)
{
    const int total = static_cast<int>(m_segments.size());
    const int first = std::clamp(firstIndex, 0, total);
    const int end = count < 0
        ? total
        : std::clamp(first + std::max(0, count), first, total);
    if (first == m_firstIndex && end == m_endIndex)
        return;
    m_firstIndex = first;
    m_endIndex = end;
    SetToUpdate();
    UpdateSelection();
}

void ToolpathSegmentsObject::setShowNodes(bool show)
{
    if (m_style.showNodes == show)
        return;
    m_style.showNodes = show;
    SetToUpdate();
}

void ToolpathSegmentsObject::setOperationFilter(
    std::unordered_set<std::string> hiddenOperationIds,
    std::string soloOperationId)
{
    if (m_hiddenOperationIds == hiddenOperationIds &&
        m_soloOperationId == soloOperationId) {
        return;
    }
    m_hiddenOperationIds = std::move(hiddenOperationIds);
    m_soloOperationId = std::move(soloOperationId);
    SetToUpdate();
    UpdateSelection();
}

bool ToolpathSegmentsObject::operationVisible(
    const ToolpathPreviewSegment& segment) const noexcept
{
    const std::string& operationId = segment.identity.operationId;
    if (!m_soloOperationId.empty())
        return operationId == m_soloOperationId;
    return operationId.empty() ||
        m_hiddenOperationIds.find(operationId) ==
            m_hiddenOperationIds.end();
}

void ToolpathSegmentsObject::HilightOwnerWithColor(
    const Handle(PrsMgr_PresentationManager)&,
    const Handle(Prs3d_Drawer)&,
    const Handle(SelectMgr_EntityOwner)&)
{
    // OcctView reads the detected ToolpathSegmentOwner immediately after
    // MoveTo() and updates a separate hover object itself. A no-op callback prevents
    // OCCT from invoking SelectMgr_SelectableObject's throwing default while
    // also avoiding a highlight of the complete (potentially very large)
    // toolpath presentation.
}

void ToolpathSegmentsObject::ClearDynamicHighlight(
    const Handle(PrsMgr_PresentationManager)&)
{
    // The hover arrow belongs to a separate transient presentation and is
    // cleared by OcctView::clearToolpathHover().
}

void ToolpathSegmentsObject::HilightSelected(
    const Handle(PrsMgr_PresentationManager)&,
    const SelectMgr_SequenceOfOwner&)
{
    // Selection is reported through ToolpathSegmentOwner and the status bar.
    // Keeping this callback explicit is required while auto-highlight is off;
    // the base implementation throws Standard_NotImplemented.
}

void ToolpathSegmentsObject::ClearSelected()
{
    // No separate selected-owner presentation is allocated.
}

void ToolpathSegmentsObject::Compute(
    const Handle(PrsMgr_PresentationManager)&,
    const Handle(Prs3d_Presentation)& presentation,
    Standard_Integer)
{
    const int visibleCount = std::max(0, m_endIndex - m_firstIndex);
    if (visibleCount <= 0)
        return;
    const Standard_Integer maxVertices =
        static_cast<Standard_Integer>(2 * visibleCount);
    Handle(Graphic3d_ArrayOfSegments) outlines =
        new Graphic3d_ArrayOfSegments(maxVertices);
    Handle(Graphic3d_ArrayOfSegments) cuts =
        new Graphic3d_ArrayOfSegments(maxVertices);
    Handle(Graphic3d_ArrayOfSegments) rapids =
        new Graphic3d_ArrayOfSegments(maxVertices);
    Handle(Graphic3d_ArrayOfSegments) verticals =
        new Graphic3d_ArrayOfSegments(maxVertices);
    Handle(Graphic3d_ArrayOfPoints) nodes;
    if (m_style.showNodes)
        nodes = new Graphic3d_ArrayOfPoints(maxVertices);

    for (int index = m_firstIndex; index < m_endIndex; ++index) {
        const ToolpathPreviewSegment& segment =
            m_segments[static_cast<std::size_t>(index)];
        if (!operationVisible(segment) ||
            !isFinite(segment.from) || !isFinite(segment.to) ||
            segment.from.Distance(segment.to) <= kMinimumSegmentLength) {
            continue;
        }

        Handle(Graphic3d_ArrayOfSegments) target;
        switch (segment.kind) {
            case appdetail::HolePreviewSegmentKind::Cut:
                target = cuts;
                outlines->AddVertex(segment.from);
                outlines->AddVertex(segment.to);
                break;
            case appdetail::HolePreviewSegmentKind::Rapid:
                target = rapids;
                if (m_style.outlineRapid) {
                    outlines->AddVertex(segment.from);
                    outlines->AddVertex(segment.to);
                }
                break;
            case appdetail::HolePreviewSegmentKind::Vertical:
                target = verticals;
                break;
        }
        target->AddVertex(segment.from);
        target->AddVertex(segment.to);
        if (!nodes.IsNull()) {
            nodes->AddVertex(segment.from);
            nodes->AddVertex(segment.to);
        }
    }

    displayLines(
        presentation, outlines, m_style.outlineColor,
        m_style.outlineWidth);
    displayLines(
        presentation, rapids, m_style.rapidColor,
        m_style.rapidWidth);
    displayLines(
        presentation, verticals, m_style.verticalColor,
        m_style.verticalWidth);
    displayLines(
        presentation, cuts, m_style.cutColor,
        m_style.cutWidth);
    displayPoints(
        presentation, nodes, m_style.outlineColor,
        m_style.nodeSize);

    // Endpoint pairs currently describe a whole setup rather than an
    // individual operation. Suppress them while filtering so an endpoint
    // from a hidden operation cannot remain as misleading visual residue.
    if (m_style.showEndpoints && !m_endpoints.empty() &&
        m_hiddenOperationIds.empty() && m_soloOperationId.empty()) {
        Handle(Graphic3d_ArrayOfPoints) starts =
            new Graphic3d_ArrayOfPoints(
                static_cast<Standard_Integer>(m_endpoints.size()));
        Handle(Graphic3d_ArrayOfPoints) ends =
            new Graphic3d_ArrayOfPoints(
                static_cast<Standard_Integer>(m_endpoints.size()));
        for (const ToolpathPreviewEndpointPair& endpoint : m_endpoints) {
            starts->AddVertex(endpoint.start);
            ends->AddVertex(endpoint.end);
        }
        displayPoints(
            presentation, starts, m_style.startColor,
            m_style.endpointSize);
        displayPoints(
            presentation, ends, m_style.endColor,
            m_style.endpointSize);
    }

}

ToolpathHoverObject::ToolpathHoverObject()
{
    SetDisplayMode(0);
    SetInfiniteState(Standard_True);
}

void ToolpathHoverObject::setSegment(
    const ToolpathPreviewSegment& segment,
    const ToolpathPreviewStyle& style)
{
    m_segment = segment;
    m_color = style.directionColor;
    m_width = style.directionWidth;
    m_hasSegment = true;
    SetZLayer(style.zLayer);
    SetToUpdate();
}

void ToolpathHoverObject::clearSegment()
{
    if (!m_hasSegment)
        return;
    m_hasSegment = false;
    SetToUpdate();
}

void ToolpathHoverObject::Compute(
    const Handle(PrsMgr_PresentationManager)&,
    const Handle(Prs3d_Presentation)& presentation,
    Standard_Integer)
{
    if (!m_hasSegment || !isFinite(m_segment.from) ||
        !isFinite(m_segment.to)) {
        return;
    }
    gp_Vec direction(m_segment.from, m_segment.to);
    const double length = direction.Magnitude();
    if (!std::isfinite(length) || length <= kMinimumSegmentLength)
        return;
    direction /= length;
    gp_Vec reference = std::abs(direction.Z()) < 0.85
        ? gp_Vec(0.0, 0.0, 1.0)
        : gp_Vec(0.0, 1.0, 0.0);
    gp_Vec side = direction.Crossed(reference);
    if (side.Magnitude() <= kMinimumSegmentLength)
        side = gp_Vec(1.0, 0.0, 0.0);
    side.Normalize();
    const double arrowLength = std::clamp(length * 0.18, 0.5, 5.0);
    const gp_Vec backward = direction * (-arrowLength);
    const gp_Vec lateral = side * (arrowLength * 0.45);
    const gp_Pnt arrowLeft = m_segment.to.Translated(backward + lateral);
    const gp_Pnt arrowRight = m_segment.to.Translated(backward - lateral);
    Handle(Graphic3d_ArrayOfSegments) directionLines =
        new Graphic3d_ArrayOfSegments(6);
    directionLines->AddVertex(m_segment.from);
    directionLines->AddVertex(m_segment.to);
    directionLines->AddVertex(m_segment.to);
    directionLines->AddVertex(arrowLeft);
    directionLines->AddVertex(m_segment.to);
    directionLines->AddVertex(arrowRight);
    displayLines(presentation, directionLines, m_color, m_width);
}

void ToolpathHoverObject::ComputeSelection(
    const Handle(SelectMgr_Selection)&,
    Standard_Integer)
{
    // Hover feedback must never participate in picking; OcctView also displays
    // this object with selection mode -1 as a second line of defense.
}

void ToolpathSegmentsObject::ComputeSelection(
    const Handle(SelectMgr_Selection)& selection,
    Standard_Integer)
{
    Handle(SelectMgr_SelectableObject) selectable(this);
    for (int index = m_firstIndex; index < m_endIndex; ++index) {
        const ToolpathPreviewSegment& segment =
            m_segments[static_cast<std::size_t>(index)];
        if (!operationVisible(segment) ||
            !isFinite(segment.from) || !isFinite(segment.to) ||
            segment.from.Distance(segment.to) <= kMinimumSegmentLength) {
            continue;
        }
        Handle(ToolpathSegmentOwner) owner =
            new ToolpathSegmentOwner(selectable, index);
        selection->Add(new Select3D_SensitiveSegment(
            owner, segment.from, segment.to));
    }
}

ToolpathSegmentOwner::ToolpathSegmentOwner(
    const Handle(SelectMgr_SelectableObject)& selectable,
    int segmentIndex)
    : SelectMgr_EntityOwner(selectable, 6),
      m_segmentIndex(segmentIndex)
{
}

} // namespace oap::app::presentation

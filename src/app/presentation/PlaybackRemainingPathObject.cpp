// PlaybackRemainingPathObject 实现（oap_app_presentation 的 detail 私有目录）。
// 播放期间「刀尖之后剩余路径」的轻量 GPU/AIS 呈现对象，见
// detail/PlaybackRemainingPathObject.h。
#include "app/presentation/detail/PlaybackRemainingPathObject.h"

#include <algorithm>
#include <cmath>

#include <Aspect_TypeOfLine.hxx>
#include <Graphic3d_ArrayOfSegments.hxx>
#include <Graphic3d_AspectLine3d.hxx>
#include <Graphic3d_Group.hxx>
#include <Prs3d_Presentation.hxx>
#include <Quantity_NameOfColor.hxx>

namespace oap::app::presentation {
namespace {

constexpr double kCutLineWidth = 1.5;
constexpr double kOutlineWidth = 1.6;
constexpr double kRapidLineWidth = 1.0;
constexpr double kEntryRetractLineWidth = 1.0;

} // namespace

PlaybackRemainingPathObject::PlaybackRemainingPathObject(
    std::vector<PlaybackRemainingSegment> segments)
    : m_segments(std::move(segments)),
      m_endIndex(static_cast<int>(m_segments.size()))
{
    // 主 AIS context 通常使用着色模式 1。此自定义对象只实现
    // mode-0 的线段呈现,因此隐式的 context Display() 不应选中
    // 着色模式,否则将什么都发布不出来。
    SetDisplayMode(0);
}

void PlaybackRemainingPathObject::setProgress(
    int firstIndex, int endIndex, double fraction)
{
    const int total = static_cast<int>(m_segments.size());
    const int first = std::clamp(firstIndex, 0, total);
    const int end = std::clamp(endIndex, first, total);
    const double clampedFraction = std::clamp(fraction, 0.0, 1.0);
    if (first == m_firstIndex && end == m_endIndex &&
        std::abs(clampedFraction - m_fraction) <= 1.0e-9) {
        return;
    }

    m_firstIndex = first;
    m_endIndex = end;
    m_fraction = clampedFraction;
    SetToUpdate();
}

void PlaybackRemainingPathObject::setSegmentVisibility(
    const std::vector<bool>& visibility)
{
    if (visibility.size() != m_segments.size())
        return;
    bool changed = false;
    for (std::size_t index = 0; index < m_segments.size(); ++index) {
        changed |= m_segments[index].visible != visibility[index];
        m_segments[index].visible = visibility[index];
    }
    if (changed)
        SetToUpdate();
}

bool PlaybackRemainingPathObject::segmentVisible(int moveIndex) const noexcept
{
    return moveIndex >= m_firstIndex && moveIndex < m_endIndex &&
        m_segments[static_cast<std::size_t>(moveIndex)].visible;
}

void PlaybackRemainingPathObject::Compute(
    const Handle(PrsMgr_PresentationManager)&,
    const Handle(Prs3d_Presentation)& presentation,
    Standard_Integer)
{
    if (m_firstIndex >= m_endIndex)
        return;

    const Standard_Integer maxVertices =
        static_cast<Standard_Integer>(2 * (m_endIndex - m_firstIndex));
    Handle(Graphic3d_ArrayOfSegments) outlines =
        new Graphic3d_ArrayOfSegments(maxVertices);
    Handle(Graphic3d_ArrayOfSegments) cuts =
        new Graphic3d_ArrayOfSegments(maxVertices, 0, Standard_True);
    Handle(Graphic3d_ArrayOfSegments) rapids =
        new Graphic3d_ArrayOfSegments(maxVertices, 0, Standard_True);
    Handle(Graphic3d_ArrayOfSegments) entryRetracts =
        new Graphic3d_ArrayOfSegments(maxVertices, 0, Standard_True);

    // 按样式分桶填充四组独立顶点数组(描边/切削/快移/进退刀),同组共享线宽;
    // 首段按 fraction 从起点向后插值,形成“刀尖逼近终点”的动态缩短效果。
    for (int index = m_firstIndex; index < m_endIndex; ++index) {
        if (!segmentVisible(index))
            continue;
        const PlaybackRemainingSegment& segment =
            m_segments[static_cast<std::size_t>(index)];
        gp_Pnt start = segment.from;
        if (index == m_firstIndex && m_fraction < 1.0) {
            start.SetCoord(
                segment.from.X() +
                    (segment.to.X() - segment.from.X()) * m_fraction,
                segment.from.Y() +
                    (segment.to.Y() - segment.from.Y()) * m_fraction,
                segment.from.Z() +
                    (segment.to.Z() - segment.from.Z()) * m_fraction);
        }
        if (start.Distance(segment.to) <= 1.0e-9)
            continue;

        if (segment.outlined) {
            outlines->AddVertex(start);
            outlines->AddVertex(segment.to);
        }

        Handle(Graphic3d_ArrayOfSegments) target;
        switch (segment.style) {
            case PlaybackRemainingLineStyle::Cut:
                target = cuts;
                break;
            case PlaybackRemainingLineStyle::Rapid:
                target = rapids;
                break;
            case PlaybackRemainingLineStyle::EntryRetract:
                target = entryRetracts;
                break;
        }
        target->AddVertex(start, segment.color);
        target->AddVertex(segment.to, segment.color);
    }

    const auto display = [&presentation](
        const Handle(Graphic3d_ArrayOfSegments)& segments,
        const Quantity_Color& color,
        double width) {
        if (segments->VertexNumber() <= 0)
            return;
        const Handle(Graphic3d_Group) group = presentation->NewGroup();
        group->SetPrimitivesAspect(
            new Graphic3d_AspectLine3d(color, Aspect_TOL_SOLID, width));
        group->AddPrimitiveArray(segments);
    };

    display(outlines, Quantity_NOC_BLACK, kOutlineWidth);
    display(cuts, Quantity_NOC_WHITE, kCutLineWidth);
    display(rapids, Quantity_NOC_WHITE, kRapidLineWidth);
    display(
        entryRetracts, Quantity_NOC_WHITE, kEntryRetractLineWidth);
}

void PlaybackRemainingPathObject::ComputeSelection(
    const Handle(SelectMgr_Selection)&,
    Standard_Integer)
{
}

} // namespace oap::app::presentation

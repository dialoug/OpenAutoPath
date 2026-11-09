// PlaybackPresenter 实现（oap_app_presentation）。
// 播放光标、轨迹与剩余路径的语义化 OCCT 呈现器，见 PlaybackPresenter.h。
#include "app/presentation/PlaybackPresenter.h"

#include "app/presentation/OverlayController.h"

#include <cmath>
#include <utility>

#include <AIS_InteractiveObject.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <Prs3d_PointAspect.hxx>
#include <Aspect_TypeOfMarker.hxx>
#include <gp_Vec.hxx>

namespace oap::app::presentation {

PlaybackPresenter::PlaybackPresenter(
    OverlayController& overlays,
    const Handle(AIS_InteractiveContext)& context,
    const Handle(V3d_View)& view)
    : m_overlays(&overlays), m_context(context), m_view(view)
{
}

void PlaybackPresenter::bindViewer(
    const Handle(AIS_InteractiveContext)& context,
    const Handle(V3d_View)& view)
{
    m_context = context;
    m_view = view;
}

void PlaybackPresenter::clear()
{
    if (!m_overlays)
        return;
    clearRemainingPath();
    clearCursorAndTrail();
}

void PlaybackPresenter::clearCursorAndTrail()
{
    if (!m_overlays)
        return;
    m_overlays->clear(OverlayGroup::PlaybackTrail);
    clearCursor();
}

void PlaybackPresenter::clearCursor()
{
    if (m_overlays)
        m_overlays->clear(OverlayGroup::PlaybackCursor);
    m_tipPointGeometry.Nullify();
    m_tipSphereRadius = 0.0;
}

void PlaybackPresenter::clearRemainingPath()
{
    if (m_overlays)
        m_overlays->clear(OverlayGroup::PlaybackRemainingPath);
}

Handle(AIS_Shape) PlaybackPresenter::tipSpherePresentation() const
{
    if (!m_overlays)
        return {};
    const auto group = OverlayGroup::PlaybackCursor;
    for (std::size_t index = 0; index < m_overlays->size(group); ++index) {
        Handle(AIS_Shape) value =
            Handle(AIS_Shape)::DownCast(m_overlays->at(group, index));
        if (!value.IsNull())
            return value;
    }
    return {};
}

Handle(AIS_Point) PlaybackPresenter::tipPointPresentation() const
{
    if (!m_overlays)
        return {};
    const auto group = OverlayGroup::PlaybackCursor;
    for (std::size_t index = 0; index < m_overlays->size(group); ++index) {
        Handle(AIS_Point) value =
            Handle(AIS_Point)::DownCast(m_overlays->at(group, index));
        if (!value.IsNull())
            return value;
    }
    return {};
}

void PlaybackPresenter::updateTipMarker(
    const gp_Pnt& tip,
    double toolDiameter,
    const gp_Trsf& modelTransform,
    const Quantity_Color& color,
    double pointSize)
{
    if (!m_overlays)
        return;

    double radius = toolDiameter * 0.5;
    if (!(radius > 1.0e-3))
        radius = 1.0;

    // 球体仅在“不存在”或“刀具半径变化”时才重建;同一半径下只移动位置,
    // 避免每帧重建 AIS_Shape 造成拾取/显示状态抖动。
    Handle(AIS_Shape) tipSphere = tipSpherePresentation();
    if (tipSphere.IsNull() ||
        std::abs(m_tipSphereRadius - radius) > 1.0e-9) {
        m_overlays->clear(OverlayGroup::PlaybackCursor);
        m_tipPointGeometry.Nullify();
        tipSphere.Nullify();

        BRepPrimAPI_MakeSphere sphere(gp_Pnt(0.0, 0.0, 0.0), radius);
        sphere.Build();
        if (sphere.IsDone()) {
            tipSphere = new AIS_Shape(sphere.Shape());
            tipSphere->SetDisplayMode(AIS_Shaded);
            tipSphere->SetColor(color);
            tipSphere->SetZLayer(Graphic3d_ZLayerId_Topmost);
            m_overlays->append(OverlayGroup::PlaybackCursor, tipSphere);
            m_tipSphereRadius = radius;
        } else {
            m_tipSphereRadius = 0.0;
        }
    }

    if (!tipSphere.IsNull()) {
        gp_Trsf placement;
        placement.SetTranslation(gp_Vec(tip.X(), tip.Y(), tip.Z()));
        placement.PreMultiply(modelTransform);
        tipSphere->SetLocalTransformation(placement);
    }

    Handle(AIS_Point) tipPoint = tipPointPresentation();
    if (tipPoint.IsNull()) {
        m_tipPointGeometry = new Geom_CartesianPoint(tip);
        tipPoint = new AIS_Point(m_tipPointGeometry);
        tipPoint->SetColor(color);
        tipPoint->SetLocalTransformation(modelTransform);
        tipPoint->Attributes()->SetPointAspect(
            new Prs3d_PointAspect(
                Aspect_TOM_BALL, color, pointSize));
        tipPoint->SetZLayer(Graphic3d_ZLayerId_Topmost);
        m_overlays->append(OverlayGroup::PlaybackCursor, tipPoint);
    } else if (!m_tipPointGeometry.IsNull()) {
        m_tipPointGeometry->SetPnt(tip);
        tipPoint->SetLocalTransformation(modelTransform);
    }
    m_overlays->redisplay(OverlayGroup::PlaybackCursor);
}

Handle(PlaybackRemainingPathObject)
PlaybackPresenter::remainingPathPresentation() const
{
    if (!m_overlays)
        return {};
    return Handle(PlaybackRemainingPathObject)::DownCast(
        m_overlays->first(OverlayGroup::PlaybackRemainingPath));
}

bool PlaybackPresenter::remainingSegmentVisible(int moveIndex) const
{
    const auto remaining = remainingPathPresentation();
    return m_overlays && m_overlays->visible(OverlayGroup::PlaybackRemainingPath) &&
        !remaining.IsNull() && remaining->segmentVisible(moveIndex);
}

bool PlaybackPresenter::hasRemainingPath() const
{
    return !remainingPathPresentation().IsNull();
}

void PlaybackPresenter::setStaticOverlayVisibility(
    const PlaybackStaticOverlayVisibility& visibility)
{
    if (!m_overlays)
        return;
    m_overlays->setVisible(
        OverlayGroup::Toolpath, visibility.fullToolpath);
    m_overlays->setVisible(
        OverlayGroup::CurrentToolpath, visibility.currentToolpath);
    m_overlays->setVisible(
        OverlayGroup::HoleStageHighlight,
        visibility.holeStageHighlight);
}

void PlaybackPresenter::applyToolpathDisplayLayer(int zLayer)
{
    if (!m_overlays)
        return;
    const auto apply = [this, zLayer](OverlayGroup group) {
        const std::size_t count = m_overlays->size(group);
        for (std::size_t index = 0; index < count; ++index) {
            Handle(AIS_InteractiveObject) object =
                m_overlays->at(group, index);
            if (!object.IsNull())
                object->SetZLayer(zLayer);
        }
        m_overlays->redisplay(group);
    };
    apply(OverlayGroup::Toolpath);
    apply(OverlayGroup::CurrentToolpath);
    apply(OverlayGroup::PlaybackRemainingPath);
    apply(OverlayGroup::PlaybackTrail);
    apply(OverlayGroup::HoleStageHighlight);
}

void PlaybackPresenter::updateRemainingPath(
    std::vector<PlaybackRemainingSegment> segments,
    int firstIndex,
    int endIndex,
    double fraction,
    const gp_Trsf& modelTransform,
    int zLayer)
{
    if (!m_overlays)
        return;

    Handle(PlaybackRemainingPathObject) remaining =
        remainingPathPresentation();
    if (remaining.IsNull()) {
        // 首次更新时惰性创建剩余路径对象;此时以不可见方式替换进覆盖组,
        // 待下面设置进度并确认非空后再统一显示,避免空路径闪现。
        if (segments.empty()) {
            setRemainingPathVisible(false);
            return;
        }
        remaining =
            new PlaybackRemainingPathObject(std::move(segments));
        remaining->SetLocalTransformation(modelTransform);
        remaining->SetZLayer(zLayer);
        Handle(AIS_InteractiveObject) object = remaining;
        m_overlays->replace(
            OverlayGroup::PlaybackRemainingPath, {object}, false);
    }

    remaining->setProgress(firstIndex, endIndex, fraction);
    if (firstIndex >= endIndex) {
        setRemainingPathVisible(false);
        return;
    }

    setRemainingPathVisible(true);
    if (!m_context.IsNull()) {
        m_context->Display(
            remaining, AIS_WireFrame, -1, Standard_False);
    }
    m_overlays->redisplay(OverlayGroup::PlaybackRemainingPath);
}

void PlaybackPresenter::setRemainingSegmentVisibility(
    const std::vector<bool>& visibility)
{
    const auto remaining = remainingPathPresentation();
    if (remaining.IsNull())
        return;
    remaining->setSegmentVisibility(visibility);
    if (m_overlays)
        m_overlays->redisplay(OverlayGroup::PlaybackRemainingPath);
}

void PlaybackPresenter::setRemainingPathVisible(bool visible)
{
    if (m_overlays)
        m_overlays->setVisible(OverlayGroup::PlaybackRemainingPath, visible);
}

void PlaybackPresenter::applyModelTransform(const gp_Trsf& transform)
{
    if (!m_overlays)
        return;
    m_overlays->setLocalTransformation(
        OverlayGroup::PlaybackRemainingPath, transform);

    const Handle(AIS_Point) tipPoint = tipPointPresentation();
    if (!tipPoint.IsNull())
        tipPoint->SetLocalTransformation(transform);
    const Handle(AIS_Shape) tipSphere = tipSpherePresentation();
    if (!tipSphere.IsNull() && !m_tipPointGeometry.IsNull()) {
        gp_Trsf placement;
        placement.SetTranslation(
            gp_Vec(gp_Pnt(0.0, 0.0, 0.0), m_tipPointGeometry->Pnt()));
        placement.PreMultiply(transform);
        tipSphere->SetLocalTransformation(placement);
    }
}

} // namespace oap::app::presentation

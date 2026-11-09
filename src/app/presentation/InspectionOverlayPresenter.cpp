// InspectionOverlayPresenter 实现（oap_app_presentation）。
// 临时检查叠加层的 OCCT 呈现器，见 InspectionOverlayPresenter.h。
#include "app/presentation/InspectionOverlayPresenter.h"

#include "app/presentation/OverlayController.h"

#include <AIS_Line.hxx>
#include <AIS_Shape.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <Geom_CartesianPoint.hxx>

namespace oap::app::presentation {

InspectionOverlayPresenter::InspectionOverlayPresenter(
    OverlayController& overlays) noexcept
    : m_overlays(&overlays)
{
}

void InspectionOverlayPresenter::clear()
{
    if (m_overlays)
        m_overlays->clear(OverlayGroup::TemporaryHighlight);
}

void InspectionOverlayPresenter::appendFace(
    const TopoDS_Face& face,
    const Quantity_Color& color,
    const gp_Trsf& modelTransform)
{
    if (!m_overlays || face.IsNull())
        return;
    Handle(AIS_Shape) presentation = new AIS_Shape(face);
    presentation->SetDisplayMode(AIS_Shaded);
    presentation->SetColor(color);
    presentation->SetLocalTransformation(modelTransform);
    m_overlays->append(OverlayGroup::TemporaryHighlight, presentation);
}

bool InspectionOverlayPresenter::appendWitnessLine(
    const gp_Pnt& first,
    const gp_Pnt& second,
    const Quantity_Color& color,
    double width,
    double coincidenceTolerance,
    const gp_Trsf& modelTransform)
{
    if (!m_overlays || first.Distance(second) < coincidenceTolerance)
        return false;
    Handle(Geom_CartesianPoint) firstPoint =
        new Geom_CartesianPoint(first);
    Handle(Geom_CartesianPoint) secondPoint =
        new Geom_CartesianPoint(second);
    Handle(AIS_Line) line = new AIS_Line(firstPoint, secondPoint);
    line->SetColor(color);
    line->SetWidth(width);
    line->SetLocalTransformation(modelTransform);
    m_overlays->append(OverlayGroup::TemporaryHighlight, line);
    return true;
}

bool InspectionOverlayPresenter::appendBoundingBox(
    const Bnd_Box& bounds,
    const Quantity_Color& color,
    double width,
    const gp_Trsf& modelTransform)
{
    if (!m_overlays || bounds.IsVoid())
        return false;
    Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
    bounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    const double dx = xmax - xmin;
    const double dy = ymax - ymin;
    const double dz = zmax - zmin;
    if (dx <= 0.0 || dy <= 0.0 || dz <= 0.0)
        return false;
    // 从 Bnd_Box 还原实体盒并以线框模式显示,作为包围范围的可视参考。
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(
        gp_Pnt(xmin, ymin, zmin), dx, dy, dz).Shape();
    Handle(AIS_Shape) presentation = new AIS_Shape(box);
    presentation->SetLocalTransformation(modelTransform);
    presentation->SetColor(color);
    presentation->SetWidth(width);
    presentation->SetDisplayMode(AIS_WireFrame);
    m_overlays->append(OverlayGroup::TemporaryHighlight, presentation);
    return true;
}

} // namespace oap::app::presentation

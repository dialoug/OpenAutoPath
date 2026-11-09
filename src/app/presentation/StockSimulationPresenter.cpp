// StockSimulationPresenter 实现（oap_app_presentation）。
// 孔毛坯与腔 IPW 叠加层的 OCCT 所有权和显示状态，见 StockSimulationPresenter.h。
#include "StockSimulationPresenter.h"

#include "OverlayController.h"
#include "app/presentation/detail/MultiDexelPrimitivesObject.h"
#include "app/view/DisplayMaterials.h"

#include <algorithm>
#include <new>
#include <vector>

#include <AIS_ListOfInteractive.hxx>
#include <Aspect_InteriorStyle.hxx>
#include <Aspect_PolygonOffsetMode.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <Graphic3d_AspectFillArea3d.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <Standard_Failure.hxx>

namespace oap::app::presentation {

StockSimulationPresenter::StockSimulationPresenter(
    OverlayController& overlays,
    const Handle(AIS_InteractiveContext)& context,
    const Handle(V3d_View)& view)
    : m_overlays(&overlays),
      m_context(context),
      m_view(view),
      m_pocketOwner(new Standard_Transient())
{
}

void StockSimulationPresenter::bindViewer(
    const Handle(AIS_InteractiveContext)& context,
    const Handle(V3d_View)& view)
{
    m_context = context;
    m_view = view;
}

void StockSimulationPresenter::clearHole()
{
    if (m_overlays)
        m_overlays->clear(OverlayGroup::HoleStock);
}

Handle(AIS_Shape) StockSimulationPresenter::holePresentation() const
{
    if (!m_overlays)
        return {};
    return Handle(AIS_Shape)::DownCast(
        m_overlays->first(OverlayGroup::HoleStock));
}

StockSimulationPresenter::Result
StockSimulationPresenter::presentHoleShape(
    const TopoDS_Shape& shape,
    const Quantity_Color& color,
    double transparency,
    int zLayer,
    const gp_Trsf& modelTransform,
    bool matteMaterial)
{
    if (shape.IsNull() || !m_overlays)
        return {false, "empty hole stock shape"};
    try {
        Handle(AIS_Shape) presentation = holePresentation();
        if (presentation.IsNull()) {
            presentation = new AIS_Shape(shape);
            presentation->SetLocalTransformation(modelTransform);
            presentation->SetDisplayMode(AIS_Shaded);
            m_overlays->replace(OverlayGroup::HoleStock, {presentation});
        } else {
            presentation->SetShape(shape);
        }
        presentation->SetColor(color);
        if (matteMaterial)
            presentation->SetMaterial(view::matteShapeMaterial());
        presentation->SetTransparency(transparency);
        presentation->SetZLayer(zLayer);
        m_overlays->redisplay(OverlayGroup::HoleStock);
        if (!m_view.IsNull())
            m_view->Redraw();
        return {true, {}};
    } catch (const Standard_Failure& failure) {
        const char* message = failure.GetMessageString();
        return {false, message ? message : "OCCT hole stock display failure"};
    } catch (const std::bad_alloc&) {
        return {false, "hole stock display allocation failure"};
    }
}

Handle(AIS_Triangulation)
StockSimulationPresenter::pocketTriangulation() const
{
    if (!m_overlays)
        return {};
    return Handle(AIS_Triangulation)::DownCast(
        m_overlays->first(OverlayGroup::PocketIpw));
}

void StockSimulationPresenter::removePocketObjects()
{
    // 收集三类候选:覆盖组内对象、本类记录的 m_pocketObject、以及 context 中
    // 所有 Owner==m_pocketOwner 的对象(防止只经 context 直接 Display 的对象
    // 漏网)。去重后统一 Deactivate/Erase/Remove,最后清空覆盖组并刷新视图。
    std::vector<Handle(AIS_InteractiveObject)> presentations;
    const auto appendUnique = [&presentations](
                                  const Handle(AIS_InteractiveObject)& object) {
        if (object.IsNull())
            return;
        const auto duplicate = std::find_if(
            presentations.begin(), presentations.end(),
            [&object](const Handle(AIS_InteractiveObject)& existing) {
                return existing.get() == object.get();
            });
        if (duplicate == presentations.end())
            presentations.push_back(object);
    };

    appendUnique(m_pocketObject);
    if (m_overlays) {
        const std::size_t count = m_overlays->size(OverlayGroup::PocketIpw);
        for (std::size_t index = 0; index < count; ++index)
            appendUnique(m_overlays->at(OverlayGroup::PocketIpw, index));
    }
    if (!m_context.IsNull() && !m_pocketOwner.IsNull()) {
        AIS_ListOfInteractive registeredObjects;
        m_context->ObjectsInside(registeredObjects);
        for (AIS_ListOfInteractive::Iterator iterator(registeredObjects);
             iterator.More(); iterator.Next()) {
            const Handle(AIS_InteractiveObject)& object = iterator.Value();
            if (!object.IsNull() &&
                object->GetOwner().get() == m_pocketOwner.get()) {
                appendUnique(object);
            }
        }
    }
    m_pocketObject.Nullify();

    if (!m_context.IsNull()) {
        for (const Handle(AIS_InteractiveObject)& presentation :
             presentations) {
            try {
                m_context->Deactivate(presentation);
            } catch (const Standard_Failure&) {
            }
            try {
                m_context->Erase(presentation, Standard_False);
            } catch (const Standard_Failure&) {
            }
            try {
                m_context->Remove(presentation, Standard_False);
            } catch (const Standard_Failure&) {
            }
        }
    }
    if (m_overlays) {
        try {
            m_overlays->clear(OverlayGroup::PocketIpw);
        } catch (...) {
        }
    }
    if (!m_context.IsNull())
        m_context->UpdateCurrentViewer();
    if (!m_view.IsNull())
        m_view->Redraw();
}

StockSimulationPresenter::Result
StockSimulationPresenter::presentPocketTriangulation(
    const Handle(Poly_Triangulation)& mesh,
    const Quantity_Color& color,
    double transparency,
    bool topLayer,
    IpwOcctDisplayMode displayMode,
    const gp_Trsf& modelTransform)
{
    if (mesh.IsNull())
        return {false, "empty IPW triangulation"};
    if (!m_overlays)
        return {false, "IPW presentation is unavailable"};
    try {
        removePocketObjects();
        // 三角化呈现:显式构造着色/边线属性(默认关闭边线、自定义边缘颜色与
        // 多边形偏移避免与毛坯表面 Z 冲突),再按 topLayer 落到 Top/Default 层。
        Handle(AIS_Triangulation) presentation =
            new AIS_Triangulation(mesh);
        presentation->SetOwner(m_pocketOwner);
        presentation->SetDisplayMode(AIS_WireFrame);
        presentation->SetLocalTransformation(modelTransform);
        presentation->Attributes()->SetupOwnShadingAspect();
        presentation->SetColor(color);
        presentation->SetTransparency(transparency);
        const Handle(Prs3d_ShadingAspect)& shading =
            presentation->Attributes()->ShadingAspect();
        if (!shading.IsNull() && !shading->Aspect().IsNull()) {
            shading->SetColor(color);
            shading->SetTransparency(transparency);
            const Handle(Graphic3d_AspectFillArea3d)& fill =
                shading->Aspect();
            if (displayMode == IpwOcctDisplayMode::SurfaceShaded)
                fill->SetEdgeOff();
            else
                fill->SetEdgeOn();
            if (displayMode == IpwOcctDisplayMode::SurfaceWireframe)
                fill->SetInteriorStyle(Aspect_IS_EMPTY);
            fill->SetEdgeColor(Quantity_Color(
                0.04, 0.10, 0.08, Quantity_TOC_RGB));
            fill->SetEdgeWidth(1.5);
            fill->SetEdgeLineType(Aspect_TOL_SOLID);
            fill->SetPolygonOffsets(Aspect_POM_All, -1.0f, -1.0f);
            fill->SetSkipFirstEdge(false);
            fill->SetDrawSilhouette(true);
        }
        presentation->SetZLayer(topLayer
            ? Graphic3d_ZLayerId_Top
            : Graphic3d_ZLayerId_Default);
        m_pocketObject = presentation;
        m_overlays->replace(OverlayGroup::PocketIpw, {presentation});
        if (!m_context.IsNull()) {
            m_context->Display(
                presentation, AIS_WireFrame, -1, Standard_False);
            m_context->Redisplay(presentation, Standard_False);
        }
        if (!m_view.IsNull())
            m_view->Redraw();
        return {true, {}};
    } catch (const Standard_Failure& failure) {
        removePocketObjects();
        const char* message = failure.GetMessageString();
        return {false, message ? message : "OCCT IPW display failure"};
    } catch (const std::bad_alloc&) {
        removePocketObjects();
        return {false, "IPW display allocation failure"};
    }
}

StockSimulationPresenter::Result
StockSimulationPresenter::presentPocketPrimitiveGeometry(
    const IpwDisplayPrimitiveGeometryPtr& geometry,
    const Quantity_Color& color,
    double transparency,
    bool topLayer,
    const gp_Trsf& modelTransform)
{
    if (!geometry)
        return {false, "empty IPW primitive geometry"};
    if (!m_overlays)
        return {false, "IPW presentation is unavailable"};
    try {
        removePocketObjects();
        Handle(MultiDexelPrimitivesObject) presentation =
            new MultiDexelPrimitivesObject(geometry, color, transparency);
        presentation->SetOwner(m_pocketOwner);
        presentation->SetLocalTransformation(modelTransform);
        presentation->SetZLayer(topLayer
            ? Graphic3d_ZLayerId_Top
            : Graphic3d_ZLayerId_Default);
        Handle(AIS_InteractiveObject) object = presentation;
        m_pocketObject = object;
        m_overlays->replace(OverlayGroup::PocketIpw, {object});
        if (!m_context.IsNull()) {
            m_context->Display(object, 0, -1, Standard_False);
            m_context->Redisplay(object, Standard_False);
        }
        if (!m_view.IsNull())
            m_view->Redraw();
        return {true, {}};
    } catch (const Standard_Failure& failure) {
        removePocketObjects();
        const char* message = failure.GetMessageString();
        return {false, message ? message : "OCCT IPW primitive display failure"};
    } catch (const std::bad_alloc&) {
        removePocketObjects();
        return {false, "IPW primitive display allocation failure"};
    }
}

StockSimulationPresenter::Result
StockSimulationPresenter::presentPocketBRep(
    const TopoDS_Shape& shape,
    const Quantity_Color& color,
    double transparency,
    bool topLayer,
    const gp_Trsf& modelTransform)
{
    if (shape.IsNull())
        return {false, "empty IPW BRep"};
    if (!m_overlays)
        return {false, "IPW presentation is unavailable"};
    try {
        removePocketObjects();
        Handle(AIS_Shape) presentation = new AIS_Shape(shape);
        presentation->SetOwner(m_pocketOwner);
        presentation->SetLocalTransformation(modelTransform);
        presentation->SetDisplayMode(AIS_Shaded);
        presentation->SetColor(color);
        presentation->SetTransparency(transparency);
        presentation->SetZLayer(topLayer
            ? Graphic3d_ZLayerId_Top
            : Graphic3d_ZLayerId_Default);
        Handle(AIS_InteractiveObject) object = presentation;
        m_pocketObject = object;
        m_overlays->replace(OverlayGroup::PocketIpw, {object});
        if (!m_context.IsNull()) {
            m_context->Display(object, AIS_Shaded, -1, Standard_False);
            m_context->Redisplay(object, Standard_False);
        }
        if (!m_view.IsNull())
            m_view->Redraw();
        return {true, {}};
    } catch (const Standard_Failure& failure) {
        removePocketObjects();
        const char* message = failure.GetMessageString();
        return {false, message ? message : "OCCT IPW BRep display failure"};
    } catch (const std::bad_alloc&) {
        removePocketObjects();
        return {false, "IPW BRep display allocation failure"};
    }
}

void StockSimulationPresenter::resetDisplayedFrame() noexcept
{
    m_displayedStepIndex = -1;
    m_displayedOperationIndex = -1;
    m_displayedCompletedMoveCount = -1;
    m_displayedFrameIsTransient = false;
}

} // namespace oap::app::presentation

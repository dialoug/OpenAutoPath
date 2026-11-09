#include "StepLoadPreviewPresenter.h"
#include <AIS_ListOfInteractive.hxx>
#include <AIS_ViewCube.hxx>
#include <Prs3d_Drawer.hxx>

namespace oap {
void StepLoadPreviewPresenter::display(const Handle(AIS_InteractiveContext)& context,
                                      const Handle(V3d_View)& view,
                                      const TopoDS_Shape& shape, double deflection) {
    clear(true);
    if (context.IsNull() || view.IsNull() || shape.IsNull()) return;
    m_context = context;
    m_view = view;
    m_camera = new Graphic3d_Camera(view->Camera());
    try {
        AIS_ListOfInteractive displayed;
        context->DisplayedObjects(displayed);
        for (const auto& object : displayed) {
            if (object->IsKind(STANDARD_TYPE(AIS_ViewCube))) continue;
            m_previous.push_back(object);
            context->Erase(object, false);
        }
        m_preview = new AIS_Shape(shape);
        m_preview->SetDisplayMode(AIS_Shaded);
        if (deflection > 0) {
            m_preview->Attributes()->SetTypeOfDeflection(Aspect_TOD_ABSOLUTE);
            m_preview->Attributes()->SetMaximalChordialDeviation(deflection);
        }
        context->Display(m_preview, false);
        context->Deactivate(m_preview);
        view->FitAll(0.05, false);
        view->ZFitAll();
        view->Redraw();
    } catch (...) { clear(true); throw; }
}

void StepLoadPreviewPresenter::clear(bool restore) noexcept {
    try {
        if (!m_context.IsNull()) {
            if (!m_preview.IsNull()) m_context->Remove(m_preview, false);
            if (restore) {
                for (const auto& object : m_previous) m_context->Display(object, false);
                if (!m_view.IsNull() && !m_camera.IsNull()) {
                    m_view->SetCamera(m_camera);
                    m_view->Redraw();
                }
            }
        }
    } catch (...) { /* 关闭/显示失败的清理不能传播异常。 */ }
    m_previous.clear();
    m_preview.Nullify();
    m_camera.Nullify();
    m_view.Nullify();
    m_context.Nullify();
}
}

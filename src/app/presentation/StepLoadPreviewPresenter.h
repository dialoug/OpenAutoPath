// 加载期间的非交互预览；不替换文档会话，取消时恢复旧显示与相机。
#pragma once
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <Graphic3d_Camera.hxx>
#include <V3d_View.hxx>
#include <vector>

namespace oap {
class StepLoadPreviewPresenter {
public:
    ~StepLoadPreviewPresenter() { clear(false); }
    void display(const Handle(AIS_InteractiveContext)& context, const Handle(V3d_View)& view,
                 const TopoDS_Shape& shape, double deflection);
    void clear(bool restore) noexcept;
    bool active() const noexcept { return !m_context.IsNull(); }
private:
    Handle(AIS_InteractiveContext) m_context;
    Handle(V3d_View) m_view;
    Handle(Graphic3d_Camera) m_camera;
    Handle(AIS_Shape) m_preview;
    std::vector<Handle(AIS_InteractiveObject)> m_previous;
};
}

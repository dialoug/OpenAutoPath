// HoleStageHighlightPresenter 实现（oap_app_presentation）。
// 选中孔位阶段壁面与切削路径的 OCCT 呈现，见 HoleStageHighlightPresenter.h。
#include "HoleStageHighlightPresenter.h"

#include <utility>
#include <vector>

#include <AIS_InteractiveObject.hxx>
#include <AIS_Shape.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <Graphic3d_DisplayPriority.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>

#include "OverlayController.h"

namespace oap::app::presentation {

HoleStageHighlightPresenter::HoleStageHighlightPresenter(
    OverlayController& overlays) noexcept
    : m_overlays(&overlays)
{
}

void HoleStageHighlightPresenter::clear()
{
    if (m_overlays)
        m_overlays->clear(OverlayGroup::HoleStageHighlight);
}

bool HoleStageHighlightPresenter::replace(
    const TopoDS_Shape& walls,
    const TopoDS_Shape& path,
    const gp_Trsf& modelTransform,
    const HoleStageHighlightStyle& style,
    bool visible)
{
    if (!m_overlays)
        return false;

    std::vector<Handle(AIS_InteractiveObject)> objects;
    std::vector<int> priorities;
    try {
        OCC_CATCH_SIGNALS
        if (!walls.IsNull()) {
            Handle(AIS_Shape) wallObject = new AIS_Shape(walls);
            wallObject->SetLocalTransformation(modelTransform);
            wallObject->SetDisplayMode(AIS_Shaded);
            wallObject->SetColor(style.wallColor);
            wallObject->SetTransparency(style.wallTransparency);
            objects.push_back(wallObject);
            priorities.push_back(
                static_cast<int>(Graphic3d_DisplayPriority_Above));
        }
        if (!path.IsNull()) {
            // 路径分两层发布:下层为更宽的 outlineColor 描边,上层为窄的
            // pathColor 主色,制造“深色描边内嵌亮色线”的对比,保证在任意
            // 背景色下都清晰可辨。上层 Topmost 确保不被壁面遮挡。
            const auto appendPath =
                [&objects, &priorities, &modelTransform, &style, &path](
                    const Quantity_Color& color,
                    double width,
                    Graphic3d_DisplayPriority priority) {
                    Handle(AIS_Shape) object = new AIS_Shape(path);
                    object->SetLocalTransformation(modelTransform);
                    object->SetDisplayMode(AIS_WireFrame);
                    object->SetColor(color);
                    object->SetWidth(width);
                    object->SetZLayer(style.zLayer);
                    object->Attributes()->SetWireAspect(
                        new Prs3d_LineAspect(
                            color, Aspect_TOL_SOLID, width));
                    objects.push_back(object);
                    priorities.push_back(static_cast<int>(priority));
                };
            appendPath(
                style.outlineColor, style.outlineWidth,
                Graphic3d_DisplayPriority_Above);
            appendPath(
                style.pathColor, style.pathWidth,
                Graphic3d_DisplayPriority_Topmost);
        }
    } catch (const Standard_Failure&) {
        clear();
        return false;
    } catch (...) {
        clear();
        return false;
    }

    if (objects.empty()) {
        clear();
        return false;
    }
    m_overlays->replace(
        OverlayGroup::HoleStageHighlight,
        std::move(objects), visible);
    for (std::size_t index = 0; index < priorities.size(); ++index) {
        m_overlays->setDisplayPriority(
            OverlayGroup::HoleStageHighlight,
            index, priorities[index]);
    }
    return true;
}

} // namespace oap::app::presentation

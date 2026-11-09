// 选中孔位阶段壁面与切削路径的 OCCT 呈现。
#pragma once

#include <Quantity_Color.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>

namespace oap::app::presentation {

class OverlayController;

// 高亮样式:壁面以半透明着色显示,路径用“描边 + 彩色”两层线框叠显。
struct HoleStageHighlightStyle {
    Quantity_Color wallColor;
    Quantity_Color pathColor;
    Quantity_Color outlineColor;
    double wallTransparency = 0.35;
    double pathWidth = 4.0;
    double outlineWidth = 6.0;
    int zLayer = 0;
};

// 孔阶段高亮呈现器:一次 replace() 以“整组替换”语义发布壁面与切削路径,
// 对象句柄归 OverlayController::HoleStageHighlight 覆盖组,本类仅借用控制器。
class HoleStageHighlightPresenter {
public:
    explicit HoleStageHighlightPresenter(
        OverlayController& overlays) noexcept;

    // 清空高亮组。
    void clear();
    // 重建高亮:壁面用着色+透明,路径按 outlineColor/pathColor 双层线框。
    // OCCT 构建异常时清空组并返回 false;所有对象可见性由 visible 决定。
    bool replace(
        const TopoDS_Shape& walls,
        const TopoDS_Shape& path,
        const gp_Trsf& modelTransform,
        const HoleStageHighlightStyle& style,
        bool visible);

private:
    OverlayController* m_overlays = nullptr;
};

} // namespace oap::app::presentation

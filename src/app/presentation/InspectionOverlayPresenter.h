// 临时检查叠加层的 OCCT 呈现器。
// 模块:oap_app_presentation。面向面检查/尺寸测量的临时高亮(着色面、
// 见证线、包围盒线框),全部对象追加进 TemporaryHighlight 覆盖组,
// 由 clear() 一次性清除;本类不持有 AIS 句柄所有权。
#pragma once

#include <Bnd_Box.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

namespace oap::app::presentation {

class OverlayController;

// 检查叠加呈现器:以“追加”语义不断累积高亮对象,直到 clear() 整体移除。
class InspectionOverlayPresenter {
public:
    explicit InspectionOverlayPresenter(OverlayController& overlays) noexcept;

    // 清空 TemporaryHighlight 覆盖组。
    void clear();
    // 追加一个着色面高亮;face 为空句柄时静默忽略。
    void appendFace(
        const TopoDS_Face& face,
        const Quantity_Color& color,
        const gp_Trsf& modelTransform);
    // 追加见证线;两端点距离小于 coincidenceTolerance 时视为退化,返回 false。
    bool appendWitnessLine(
        const gp_Pnt& first,
        const gp_Pnt& second,
        const Quantity_Color& color,
        double width,
        double coincidenceTolerance,
        const gp_Trsf& modelTransform);
    // 追加包围盒线框;bounds 为空(void)或退化为零体积时返回 false。
    bool appendBoundingBox(
        const Bnd_Box& bounds,
        const Quantity_Color& color,
        double width,
        const gp_Trsf& modelTransform);

private:
    OverlayController* m_overlays = nullptr;
};

} // namespace oap::app::presentation

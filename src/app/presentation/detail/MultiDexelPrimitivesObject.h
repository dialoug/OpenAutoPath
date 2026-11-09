// 已准备好的 Multi-Dexel 直接图元的 AIS 呈现对象。
// 模块私有实现(detail)。把后台线程算好的中性图元(Segments/Points/Prisms/
// Quads)一次性上屏;仅实现 mode-0,不做拾取(ComputeSelection 为空)。
#pragma once

#include <AIS_InteractiveObject.hxx>
#include <Quantity_Color.hxx>

#include "app/view/IpwDisplayGeometry.h"

namespace oap::app::presentation {

class MultiDexelPrimitivesObject final :
    public AIS_InteractiveObject {
public:
    MultiDexelPrimitivesObject(
        IpwDisplayPrimitiveGeometryPtr geometry,
        const Quantity_Color& color,
        double transparency = 0.15);

private:
    void Compute(
        const Handle(PrsMgr_PresentationManager)&,
        const Handle(Prs3d_Presentation)& presentation,
        Standard_Integer mode) override;
    void ComputeSelection(
        const Handle(SelectMgr_Selection)&,
        Standard_Integer mode) override;

    IpwDisplayPrimitiveGeometryPtr m_geometry;
    Quantity_Color m_color;
    double m_transparency = 0.15;
};

} // namespace oap::app::presentation

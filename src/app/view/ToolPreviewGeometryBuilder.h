// 刀具编辑器三维预览几何：将刀库参数转换为轻量 OCCT 旋转体。
#pragma once

#include "tooling/library/ToolLibrary.h"

#include <TopoDS_Shape.hxx>

namespace oap::app::view {

// 刀具预览几何：刀具轮廓的显示几何。
struct ToolPreviewGeometry {
    TopoDS_Shape cuttingPart;
    TopoDS_Shape shank;
    TopoDS_Shape holder;
    TopoDS_Shape spindleEnvelope;

    bool isEmpty() const noexcept
    {
        return cuttingPart.IsNull() && shank.IsNull() && holder.IsNull() &&
               spindleEnvelope.IsNull();
    }
};

// 刀尖位于 Z=0，刀具沿 +Z 指向刀柄。返回值仅用于编辑器预览；
// 各部件保持独立，便于显示层使用不同颜色与透明度。
ToolPreviewGeometry buildToolPreviewGeometry(
    tooling::ToolType type,
    const tooling::ToolGeometry& geometry);

} // namespace oap::app::view

// 显示材质工具。模块:oap_app_view(与窗口无关的显示几何/材质层)。
// 提供各模块共用的 OCCT 材质常量;matteShapeMaterial 用于零件/毛坯面,
// 使场景呈现哑光质感、避免高光干扰识别与检查。
#pragma once

#include <Graphic3d_MaterialAspect.hxx>
#include <Quantity_Color.hxx>

namespace oap::view {

// 返回哑光材质(Plastered 基底 + 纯黑高光 + 零光泽度),供模型面、
// 毛坯等实体着色统一使用。
inline Graphic3d_MaterialAspect matteShapeMaterial()
{
    Graphic3d_MaterialAspect material(
        Graphic3d_NameOfMaterial_Plastered);
    material.SetSpecularColor(Quantity_NOC_BLACK);
    material.SetShininess(0.0f);
    return material;
}

} // namespace oap::view

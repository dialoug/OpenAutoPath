// 统一绑定钻头专属钻尖几何，并提供规划与生成共用的标量计算。
// 属于 oap_toolpath_hole 模块；由参数绑定、深度补偿与刃长推算共享的纯函数。
#pragma once

#include <algorithm>
#include <cmath>

#include "toolpath/core/ToolpathParams.h"

namespace oap::toolpath {

// 根据刀径和钻尖包含角计算圆柱刃口平面到钻尖的轴向高度。
inline double drillTipLength(double toolDiameter,
                             double tipAngle) noexcept
{
    if (!std::isfinite(toolDiameter) || toolDiameter <= 0.0 ||
        !std::isfinite(tipAngle) || tipAngle <= 0.0 ||
        tipAngle >= 180.0) {
        return 0.0;
    }
    constexpr double kPi = 3.14159265358979323846;
    const double tangent =
        std::tan(0.5 * tipAngle * kPi / 180.0);
    if (!std::isfinite(tangent) || tangent <= 0.0)
        return 0.0;
    const double length = 0.5 * toolDiameter / tangent;
    return std::isfinite(length) && length > 0.0 ? length : 0.0;
}

// 计算钻尖在给定轴向触碰深度处形成的实际圆锥直径，并限制在名义刀径内。
inline double effectiveDrillConeDiameter(double toolDiameter,
                                         double tipAngle,
                                         double axialDepth) noexcept
{
    if (!std::isfinite(toolDiameter) || toolDiameter <= 0.0 ||
        !std::isfinite(axialDepth) || axialDepth <= 0.0) {
        return 0.0;
    }
    if (!std::isfinite(tipAngle) || tipAngle <= 0.0 ||
        tipAngle >= 180.0) {
        return toolDiameter;
    }
    constexpr double kPi = 3.14159265358979323846;
    const double radius = axialDepth *
        std::tan(0.5 * tipAngle * kPi / 180.0);
    if (!std::isfinite(radius) || radius <= 0.0)
        return toolDiameter;
    return std::min(toolDiameter, 2.0 * radius);
}

// 刀具直接补偿优先；仅有有效刀尖角时改用角度公式；两项均未填写时保留
// 调用方的全局默认值。
inline void applyDrillTipGeometry(double tipCompensation,
                                  double tipAngle,
                                  ToolpathParams& params) noexcept
{
    const bool hasDirectCompensation =
        std::isfinite(tipCompensation) && tipCompensation > 0.0;
    const bool hasTipAngle =
        std::isfinite(tipAngle) && tipAngle > 0.0 && tipAngle < 180.0;
    if (hasDirectCompensation)
        params.tool.tipCompensation = tipCompensation;
    else if (hasTipAngle)
        params.tool.tipCompensation = 0.0;
    if (hasTipAngle)
        params.hole.drillTipAngleDeg = tipAngle;
}

} // namespace oap::toolpath

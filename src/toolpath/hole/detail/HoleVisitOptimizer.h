// 单装夹物理孔访问顺序优化及不安全连线修复接口。
// 属于 oap_toolpath_hole 模块 detail 私有部分；仅由 HoleMachining 在装夹
// 分组后调用，不构成稳定对外接口。
#pragma once

#include <cstddef>
#include <vector>

#include <gp_Pnt.hxx>

#include "analysis/MoldCoordinateFrame.h"
#include "toolpath/hole/HoleMachining.h"
#include "toolpath/core/ToolpathParams.h"

namespace oap::toolpath::detail {

class HoleRapidPlanner;

struct HoleVisitInput {
    std::size_t originalIndex = 0;
    gp_Pnt entry;
    gp_Pnt clearance;
};

// 以所选模具角为固定首点，依次执行最近邻、2-opt、点重定位和候选 3-opt。
// 返回相对 inputs 的访问下标顺序。
std::vector<std::size_t> optimizeHoleVisitOrder(
    const std::vector<HoleVisitInput>& inputs,
    const MoldCoordinateFrame& moldFrame,
    HoleStartCorner startCorner,
    HoleMachiningResult& result,
    const ToolpathGenerationControl* control = nullptr);

// 对最终候选顺序做真实避碰评估，并用局部重定位尽量修复不安全连线。
std::vector<std::size_t> repairUnsafeHoleVisitOrder(
    const std::vector<HoleVisitInput>& orderedInputs,
    HoleRapidPlanner& rapidPlanner,
    HoleMachiningResult& result,
    const ToolpathGenerationControl* control = nullptr);

} // namespace oap::toolpath::detail

// 孔工序专用结构验证器。
// 属于 oap_toolpath_hole 模块；在通用参数/移动/原语/刀具验证之上叠加孔工序
// 专属结构检查（孔轴、径向余量、中心钻切削段、退刀位置）。
#pragma once

#include <vector>

#include "toolpath/core/ToolpathParams.h"
#include "toolpath/hole/HoleMachining.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"

namespace oap::toolpath {

// Check that tagged rapid reentries retrace a continuous, previously cut
// centerline interval. This is motion evidence, not a replacement for IPW
// or tool-assembly collision checks.
void validateHoleDrillReentries(
    const std::vector<ToolpathMove>& moves,
    const gp_Vec& inwardAxis,
    const std::string& operationId,
    ToolpathValidationReport& report);

// 孔刀路校验器：校验孔加工工序与刀路安全。
class HoleToolpathValidator {
public:
    ToolpathValidationReport validate(
        const std::vector<HoleMachiningOperation>& operations,
        const ToolpathParams& params) const;
};

} // namespace oap::toolpath

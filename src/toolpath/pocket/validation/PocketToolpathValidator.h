// 槽/腔工序专用结构验证器。
//
// 属于 oap_toolpath_pocket 的刀路校验层：在安全校验（PocketSafetyValidator）
// 之外检查工序间的结构性一致性，供生成后与导出前统一把关。
#pragma once

#include <vector>

#include "toolpath/core/ToolpathParams.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"

namespace oap::toolpath {

// 校验整批槽腔工序与批次参数。前置条件：operations 由本模块生成；
// 逐个工序检查 clearing 状态、深度层/环数、刀具一致性、rest 加工链、
// 逐层 section 计数，并把 moves/primitives 委托给通用校验器。
class PocketToolpathValidator {
public:
    ToolpathValidationReport validate(
        const std::vector<PocketToolpathOperation>& operations,
        const ToolpathParams& params) const;
};

} // namespace oap::toolpath

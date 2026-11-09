// 现有组合参数模型的结构检查；参数归一化仍由 ToolpathParamNormalizer 负责。
#pragma once

#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"

namespace oap::toolpath {

// 检查组合参数的结构性约束（有限性、取值范围、依赖关系），
// 与归一化器互补：归一化修值，这里在归一化前后都做只读审查。
void validateToolpathParameters(
    const ToolpathParams& params,
    ToolpathValidationReport& report);

} // namespace oap::toolpath

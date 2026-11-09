// 中立圆弧和固定循环原语的公共结构验证。
#pragma once

#include <string>
#include <vector>

#include "toolpath/core/ToolpathPrimitives.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"

namespace oap::toolpath {

// 逐个原语检查有限性、退化性与工艺参数；问题按原顺序写入 report。
void validateToolpathPrimitives(
    const std::vector<ToolpathPrimitive>& primitives,
    const std::string& operationId,
    ToolpathValidationReport& report);

// 验证原语展开流与 moves 完全对应；requireCompleteExpansion 为 true 时
// 非线原语缺少 fallback 视为错误。
void validateToolpathPrimitiveExpansion(
    const std::vector<ToolpathPrimitive>& primitives,
    const std::vector<ToolpathMove>& moves,
    const std::string& operationId,
    ToolpathValidationReport& report,
    bool requireCompleteExpansion = false);

} // namespace oap::toolpath

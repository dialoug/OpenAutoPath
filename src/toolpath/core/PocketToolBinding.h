// 槽腔工序刀具快照到通用刀路参数的公共绑定。
#pragma once

#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/ToolpathToolContext.h"

namespace oap::toolpath {

// 把刀具快照的几何与工艺字段覆盖到基准参数上并返回副本。
// 前置条件：无。base 始终保持不变；tool.valid 为 false 时原样返回 base。
ToolpathParams bindPocketToolParameters(
    const ToolpathParams& base,
    const ToolpathToolContext& tool);

} // namespace oap::toolpath

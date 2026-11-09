// 已分配刀具到 CAM 刀具上下文的适配（oap_toolpath_adapter）。
// 把库刀具 AssignedTool 及其 T 刀槽切削默认值捕获为自包含的 CAM 值
// ToolpathToolContext；Job 的 ToolController 复制该值而不保留刀库链接。
#pragma once

#include "tooling/library/ToolLibrary.h"
#include "toolpath/core/ToolpathToolContext.h"

namespace oap::tooling {

// Pocket/profile strategies support cylindrical and rounded milling envelopes.
bool isSupportedMillingToolType(ToolType type);
bool hasSupportedMillingGeometry(const ToolBit& tool);

// Captures a library tool and its T-slot defaults as a self-contained CAM
// value. Job ToolControllers copy this value and never retain a library link.
oap::toolpath::ToolpathToolContext makeToolpathToolContext(
    const AssignedTool& assigned);

} // namespace oap::tooling

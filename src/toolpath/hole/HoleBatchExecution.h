// 自动规划与 Job 重建共享的孔批次参数绑定和结果聚合内核。
// 属于 oap_toolpath_hole 模块；供自动规划链路与 Job 重建服务共同消费。
#pragma once

#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/ToolpathToolContext.h"
#include "toolpath/hole/HoleMachining.h"

namespace oap::toolpath {

// 将刀库控制器快照绑定到实际生成参数；非钻刀关闭固定钻削循环。
// base 提供全局参数，tool 提供刀具几何、进给、转速与钻削偏好。
ToolpathParams bindHoleBatchParameters(
    const ToolpathParams& base,
    const ToolpathToolContext& tool);

// 聚合批次统计和诊断，不复制 Operation、setupCount 或 cancelled；调用方在
// 完成稳定 ID/来源映射和事务取消判定后自行处理这些字段。
void appendHoleMachiningSummary(
    HoleMachiningResult& target,
    const HoleMachiningResult& source);

} // namespace oap::toolpath

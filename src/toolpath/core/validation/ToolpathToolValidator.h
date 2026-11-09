// 工序级刀具几何约束。实际刀具数据和由特征推导的最小需求保持分离，
// 使预览可以标出未知参数，而不会把推导值冒充为刀库实测值。
#pragma once

#include <string>

#include "toolpath/core/ToolpathToolContext.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"

namespace oap::toolpath {

void validateCuttingEdgeRequirement(
    const ToolpathToolContext* operationTool,
    double fallbackCuttingEdgeLength,
    double minimumCuttingEdgeLength,
    const std::string& operationId,
    ToolpathValidationReport& report);

// 验证刀尖至刀柄端的已知长度能覆盖工序轴向可达深度；未知值仅告警。
void validateToolReachRequirement(
    const ToolpathToolContext* operationTool,
    double fallbackOverallLength,
    double minimumToolReach,
    const std::string& operationId,
    ToolpathValidationReport& report);

} // namespace oap::toolpath

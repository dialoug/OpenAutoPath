// 基于已知 2.5D 区域的槽/腔切削及进刀刀心边界安全校验。
//
// 属于 oap_toolpath_pocket 的安全校验层：把切削/进刀段采样到平面，
// 用 PocketSafetyArea 校验刀心是否落在考虑刀具半径后的可达域内，
// 且不切入受保护的工作材料；圆弧图元使用解析安全判定。
#pragma once

#include <functional>
#include <vector>

#include "toolpath/core/ToolpathDiagnostics.h"
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"

namespace oap::toolpath {

// 校验结果：统一报告 + 结构化诊断 + 取消标记。
struct PocketSafetyValidationResult {
    ToolpathValidationReport report;
    std::vector<ToolpathDiagnostic> diagnostics;
    bool cancelled = false;
};

// 槽/腔安全校验器。validate() 前置条件：region 有合法边界多边形、
// operation.clearing.moves 非空且至少含一个 Cut 移动；校验中可随时通过
// isCancellationRequested 取消（结果置 cancelled）。
class PocketSafetyValidator {
public:
    PocketSafetyValidationResult validate(
        const PocketRegion& region,
        const PocketToolpathOperation& operation,
        const ToolpathParams& params,
        const std::function<bool()>& isCancellationRequested = {}) const;
};

} // namespace oap::toolpath

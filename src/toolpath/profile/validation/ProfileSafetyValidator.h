// 外轮廓固定轴安全校验（oap_toolpath_profile）。
// 针对 ProfileRegion 的直壁 2.5D 成品包络连续采样，沿刀轴检查切削刃、
// 刀杆、刀柄/夹头与主轴头的轴向分段包络，产出校验报告与诊断；
// Error 阻止工序进入批次，Warning 保留已生成刀路。
#pragma once

#include <functional>
#include <vector>

#include "toolpath/core/ToolpathDiagnostics.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"
#include "toolpath/profile/geometry/ProfileRegion.h"
#include "toolpath/profile/program/ProfileToolpathOperation.h"

namespace oap::toolpath {

// 外轮廓固定轴安全校验结果。Error 阻止工序进入批次，Warning 表示
// 缺少装配尺寸或达到采样预算，但仍保留已生成刀路。
struct ProfileSafetyValidationResult {
    ToolpathValidationReport report;
    std::vector<ToolpathDiagnostic> diagnostics;
    bool cancelled = false;
};

// 针对 ProfileRegion 的直壁 2.5D 成品包络执行连续采样，并沿刀轴检查
// 切削刃、刀杆、刀柄/夹头及主轴头轴向分段包络。
class ProfileSafetyValidator {
public:
    ProfileSafetyValidationResult validate(
        const ProfileRegion& region,
        const ProfileToolpathOperation& operation,
        const ToolpathParams& params,
        const std::function<bool()>& isCancellationRequested = {}) const;
};

} // namespace oap::toolpath

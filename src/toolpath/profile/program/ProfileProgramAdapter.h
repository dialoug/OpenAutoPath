// 轮廓工序 → 中立程序适配（oap_toolpath_profile）。
// 把 ProfileToolpathOperation 转成 ToolpathOperation，并组装 ToolpathProgram，
// 供程序校验、后处理与导出消费。
#pragma once

#include <span>

#include "toolpath/core/ToolpathProgram.h"
#include "toolpath/profile/program/ProfileToolpathOperation.h"

namespace oap::toolpath {

ToolpathOperation toToolpathOperation(
    const ProfileToolpathOperation& operation);
ToolpathProgram makeToolpathProgram(
    std::span<const ProfileToolpathOperation> operations);
void appendToolpathOperations(
    ToolpathProgram& program,
    std::span<const ProfileToolpathOperation> operations);

} // namespace oap::toolpath

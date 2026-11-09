// 孔加工结果到中立程序模型的投影。
// 属于 oap_toolpath_hole 模块；把孔专用 Operation 无损转为中立的
// ToolpathOperation/ToolpathProgram，供播放、后处理与导出链路消费。
#pragma once

#include <vector>

#include "toolpath/core/ToolpathProgram.h"
#include "toolpath/hole/HoleMachining.h"

namespace oap::toolpath {

// 把单个孔工序投影为中立 ToolpathOperation，保留策略、阶段与刀路元数据。
ToolpathOperation toToolpathOperation(
    const HoleMachiningOperation& operation);

// 由一组孔工序构造完整程序；内部复用 appendToolpathOperations。
ToolpathProgram makeToolpathProgram(
    const std::vector<HoleMachiningOperation>& operations);

// 把一组孔工序追加到已有程序末尾（预留容量后逐条转换）。
void appendToolpathOperations(
    ToolpathProgram& program,
    const std::vector<HoleMachiningOperation>& operations);

} // namespace oap::toolpath

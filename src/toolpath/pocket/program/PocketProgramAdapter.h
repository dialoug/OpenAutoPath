// 槽/腔加工结果到中立程序模型的投影。
//
// 属于 oap_toolpath_pocket 的程序适配层：把 PocketToolpathOperation
// （含 algorithm_id、floor_face_id 等槽腔专属字段）投影为统一
// ToolpathOperation / ToolpathProgram，供验证与 G-code 导出复用。
#pragma once

#include <vector>

#include "toolpath/core/ToolpathProgram.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"

namespace oap::toolpath {

// 单个槽腔工序 → 中立工序：设置 type=PocketClearing、填充元数据
// 与 clearing.moves/primitives。
ToolpathOperation toToolpathOperation(
    const PocketToolpathOperation& operation);

// 由一组槽腔工序构造完整中立程序。
ToolpathProgram makeToolpathProgram(
    const std::vector<PocketToolpathOperation>& operations);

// 把一组槽腔工序追加到既有程序末尾，保留原程序中的其他工序。
void appendToolpathOperations(
    ToolpathProgram& program,
    const std::vector<PocketToolpathOperation>& operations);

} // namespace oap::toolpath

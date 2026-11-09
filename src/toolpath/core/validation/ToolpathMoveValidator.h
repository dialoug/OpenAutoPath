// 所有工序共享的移动结构检查。
#pragma once

#include <string>
#include <vector>

#include <gp_Pnt.hxx>

#include "toolpath/core/ToolpathMoves.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"

namespace oap::toolpath {

inline constexpr double kMinimumToolpathMoveLength = 1.0e-9;

// 各移动类型计数汇总，供上层判断是否缺少切削/进退刀结构。
struct ToolpathMoveStats {
    int rapid = 0;
    int plunge = 0;
    int cut = 0;
    int retract = 0;
};

bool finiteToolpathPoint(const gp_Pnt& point);

// 检查移动序列：有限坐标、非退化长度、正进给、相邻连续性，并统计
// 移动类型计数；问题写入 report，返回统计。
ToolpathMoveStats validateToolpathMoves(
    const std::vector<ToolpathMove>& moves,
    const std::string& operationId,
    ToolpathValidationReport& report);

} // namespace oap::toolpath

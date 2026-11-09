// 把活动刀库投影为孔加工候选的公共查询 API。
// 模块归属:oap_toolpath_adapter;被自动选刀规划器(HoleToolSelectionPlanner 及
// detail 阶段规划器)与持久化计划重建共用,提供刀具筛选、尺寸匹配、
// 刃长/伸长可用性判定等统一规则,避免各阶段重复实现选刀逻辑。
#pragma once

#include "tooling/library/ToolLibrary.h"
#include "toolpath/core/ToolpathToolContext.h"

#include <vector>

namespace oap::tooling {

// 刀具尺寸匹配容差(直径)与刃长/伸长判定容差,统一各阶段选刀阈值。
inline constexpr double kHoleToolMatchTolerance = 1.0e-4;
inline constexpr double kHoleToolReachTolerance = 1.0e-6;

// 一种候选刀具:类型 + 翻译自刀库的刀路刀具上下文(转速/进给/循环等)。
struct HoleToolCandidate {
    ToolType type = ToolType::Unknown;
    oap::toolpath::ToolpathToolContext context;
};

// 按加工类型分组的候选刀具目录:中心钻、钻头、平底铣刀。
struct HoleToolCatalog {
    std::vector<HoleToolCandidate> centerDrills;
    std::vector<HoleToolCandidate> drills;
    std::vector<HoleToolCandidate> flatEndMills;

    bool empty() const noexcept
    {
        return centerDrills.empty() && drills.empty() &&
               flatEndMills.empty();
    }
};

// 从活动刀库建立分组目录(仅保留刀具上下文有效的刀具)。
HoleToolCatalog buildHoleToolCatalog(const ToolLibrary& library);

// 按 toolId 查指定刀具并翻译为候选;返回 false 表示不存在或上下文无效。
bool assignedHoleToolContext(const ToolLibrary& library,
                             const std::string& toolId,
                             HoleToolCandidate& result);

// 在按直径升序排列的候选列表中找“不超过 diameter(容差内)的最大刀”。
const HoleToolCandidate* findLargestNotExceeding(
    const std::vector<HoleToolCandidate>& tools,
    double diameter,
    double tolerance);

// 刃长是否满足要求;未知刃长按不阻断处理(由验证器告警)。
bool toolHasRequiredCuttingEdge(
    const HoleToolCandidate& tool,
    double requiredLength) noexcept;

// 总伸长是否满足要求;未知伸长按不阻断处理。
bool toolHasRequiredReach(
    const HoleToolCandidate& tool,
    double requiredLength) noexcept;

// 直径、刃长、伸长三项同时满足时,选择“不超过 diameter 的最大可用刀”。
const HoleToolCandidate* findLargestUsableNotExceeding(
    const std::vector<HoleToolCandidate>& tools,
    double diameter,
    double requiredCuttingEdgeLength,
    double requiredReach,
    double tolerance);

} // namespace oap::tooling

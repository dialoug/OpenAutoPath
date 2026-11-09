// 无窗口的桌面刀路请求装配辅助函数。
#pragma once

#include "analysis/MoldCoordinateFrame.h"
#include "toolpath/workflow/hole/HoleToolpathService.h"
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"
#include "toolpath/workflow/pocket/PocketToolpathService.h"

#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>

namespace oap::app {

// 孔刀路快速规划的障碍物来源:稳定 id + 参与碰撞的形状。
struct HoleObstacleSource {
    std::string stableId;
    TopoDS_Shape shape;
};

// 装配结果:已就绪的快速规划上下文与成功加入的障碍体数量。
struct HoleRapidPlanningContextBuildResult {
    toolpath::HoleRapidPlanningContext context;
    int obstacleBodyCount = 0;
};

// 把外部障碍物装配进孔快速规划的碰撞场景(compound + 场景体)。
HoleRapidPlanningContextBuildResult buildHoleRapidPlanningContext(
    const std::vector<HoleObstacleSource>& obstacles,
    const MoldCoordinateFrame* moldFrame);

// 为每个槽腔工序按区域开方向与模具坐标系确定装夹面与装夹序号。
void assignPocketSetupContexts(
    toolpath::PocketToolpathBatch& batch,
    const std::vector<toolpath::PocketRegion>& regions,
    const MoldCoordinateFrame& moldFrame);

// Profile 与槽腔使用同一装夹判定规则：以真实区域外法向相对模具坐标系
// 分类，不能把 STEP 世界坐标 X/Y/Z 直接当成模具局部六面。
} // namespace oap::app

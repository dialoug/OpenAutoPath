// 单个装夹面内的三类工序规划入口。
// 属于 hole 模块 detail 私有实现:由 HoleSetupPlanner 按装夹面调用,
// 分别生成中心定位、钻削、平底铣三类批次,共享物理孔计划与刀具目录。
#pragma once

#include <vector>

#include "tooling/adapter/hole/HoleToolSelectionPlanner.h"
#include "tooling/adapter/hole/detail/HolePhysicalPlan.h"
#include "tooling/adapter/hole/HoleToolCatalog.h"

namespace oap::tooling::detail {

// 中心定位:为需要引导的物理孔选择中心钻,生成 CenterSpot 批次;
// 目录为空或刀径超限时跳过并以警告诊断说明。
void appendCenterSpotBatch(
    const HoleToolCatalog& catalog,
    const std::vector<oap::toolpath::HoleMachiningInput>& holes,
    const std::vector<PlannedPhysicalHole>& eligible,
    oap::MoldSetupFace face,
    int setupIndex,
    const oap::toolpath::ToolpathParams& params,
    double tolerance,
    HoleToolSelectionPlan& result);

// 钻削:按孔径/深度/策略筛选可用钻头,按 T 号聚批生成 Drill 批次;
// 无法钻削的孔标记跳过,留给后续平底铣阶段处理。
void appendDrillBatches(
    const HoleToolCatalog& catalog,
    const std::vector<oap::toolpath::HoleMachiningInput>& holes,
    const std::vector<PlannedPhysicalHole>& eligible,
    oap::MoldSetupFace face,
    int setupIndex,
    const oap::toolpath::ToolpathParams& params,
    double tolerance,
    HoleToolSelectionPlan& result);

// 平底铣:为钻头无法覆盖的平底盲孔/沉孔肩部选刀,生成 FlatEndMill 批次。
void appendFlatEndMillBatch(
    const HoleToolCatalog& catalog,
    const std::vector<oap::toolpath::HoleMachiningInput>& holes,
    const std::vector<PlannedPhysicalHole>& eligible,
    oap::MoldSetupFace face,
    int setupIndex,
    const oap::toolpath::ToolpathParams& params,
    double tolerance,
    HoleToolSelectionPlan& result);

} // namespace oap::tooling::detail

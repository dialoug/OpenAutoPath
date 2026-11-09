// 为单个装夹面创建中心定位、钻削和平底刀批次。
// 属于 hole 模块 detail 私有实现;由 HoleToolSelectionPlanner 按固定六面
// 顺序调用,是本模块“按面拆批”的编排核心。
#pragma once

#include <cstddef>
#include <vector>

#include "tooling/adapter/hole/HoleToolSelectionPlanner.h"
#include "tooling/adapter/hole/HoleToolCatalog.h"

namespace oap::tooling::detail {

// 对单个装夹面:建立该面物理孔计划并依次追加三个阶段的批次,
// 只有实际产生刀路的装夹面才推进 setupCount 编号。
void planHoleSetup(
    const HoleToolCatalog& catalog,
    const std::vector<oap::toolpath::HoleMachiningInput>& holes,
    oap::MoldSetupFace face,
    const oap::toolpath::ToolpathParams& params,
    std::size_t& nextPhysicalIndex,
    HoleToolSelectionPlan& result);

} // namespace oap::tooling::detail

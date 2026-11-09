// 按装夹面拆分物理孔并确定加工次序，避免不同工件坐标系混入同一批次。
// 属于 hole 模块 detail 私有实现;面内阶段顺序:中心定位(可选) ->
// 钻削 -> 平底铣,三个阶段共享同一装夹面与 setupIndex。
#include "tooling/adapter/hole/detail/HoleSetupPlanner.h"

#include <utility>

#include "tooling/adapter/hole/detail/HolePhasePlanners.h"
#include "tooling/adapter/hole/detail/HolePhysicalPlan.h"
#include "tooling/adapter/hole/detail/HolePlanningSupport.h"

namespace oap::tooling::detail {

void planHoleSetup(
    const HoleToolCatalog& catalog,
    const std::vector<oap::toolpath::HoleMachiningInput>& holes,
    oap::MoldSetupFace face,
    const oap::toolpath::ToolpathParams& params,
    std::size_t& nextPhysicalIndex,
    HoleToolSelectionPlan& result)
{
    std::vector<PlannedPhysicalHole> eligible =
        buildPhysicalHolePlan(holes, face, nextPhysicalIndex);
    result.physicalHoleCount += static_cast<int>(eligible.size());
    if (eligible.empty())
        return;

    // setupIndex 只对实际产生刀路的装夹面连续编号。
    const int setupIndex = result.setupCount;
    const std::size_t batchCountBefore = result.batches.size();
    const double tolerance = std::max(
        kHoleToolMatchTolerance,
        params.hole.smallHoleDiameterTolerance);

    // 中心钻是可选的预加工阶段；关闭时不创建批次，也不产生缺少中心钻诊断。
    if (params.hole.useCenterSpot) {
        appendCenterSpotBatch(
            catalog, holes, eligible, face, setupIndex,
            params, tolerance, result);
    }
    appendDrillBatches(
        catalog, holes, eligible, face, setupIndex,
        params, tolerance, result);
    appendFlatEndMillBatch(
        catalog, holes, eligible, face, setupIndex,
        params, tolerance, result);

    if (result.batches.size() > batchCountBefore)
        ++result.setupCount;
}

} // namespace oap::tooling::detail

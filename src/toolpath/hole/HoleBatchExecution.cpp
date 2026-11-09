// HoleBatchExecution —— 自动孔规划与 Job 重建共享的参数绑定/结果聚合实现。
// 属于 oap_toolpath_hole 模块：把刀库控制器快照及全局工艺参数投影为实际
// 生成参数（bindHoleBatchParameters），并把多个局部批次结果合并为一份可
// 审计的汇总统计（appendHoleMachiningSummary）。
#include "toolpath/hole/HoleBatchExecution.h"

#include <cmath>

#include "toolpath/hole/DrillTipParameterBinding.h"

namespace oap::toolpath {
namespace {

void assignPositive(double source, double& destination)
{
    if (std::isfinite(source) && source > 0.0)
        destination = source;
}

} // namespace

ToolpathParams bindHoleBatchParameters(
    const ToolpathParams& base,
    const ToolpathToolContext& tool)
{
    ToolpathParams result = base;
    result.tool.diameter = tool.diameter;
    result.tool.cornerRadius = tool.cornerRadius;
    assignPositive(tool.horizontalFeed, result.tool.horizFeed);
    assignPositive(tool.verticalFeed, result.tool.vertFeed);
    assignPositive(tool.horizontalRapid, result.tool.horizRapid);
    assignPositive(tool.verticalRapid, result.tool.vertRapid);
    assignPositive(tool.spindleSpeed, result.tool.spindleSpeed);
    result.tool.cuttingEdgeLength = tool.cuttingEdgeLength;
    result.tool.overallLength = tool.overallLength;
    result.tool.shankDiameter = tool.shankDiameter;
    result.tool.holderDiameter = tool.holderDiameter;
    result.tool.holderLength = tool.holderLength;
    result.tool.spindleEnvelopeDiameter = tool.spindleEnvelopeDiameter;
    result.tool.spindleEnvelopeLength = tool.spindleEnvelopeLength;

    const bool drillingTool =
        tool.toolType == "drill" || tool.toolType == "indexable_drill";
    if (drillingTool) {
        applyDrillTipGeometry(
            tool.tipCompensation, tool.tipAngle, result);
        if (std::isfinite(tool.peckDepth) && tool.peckDepth >= 0.0)
            result.hole.peckDepth = tool.peckDepth;
        switch (tool.drillCycle) {
            case ToolpathDrillCyclePreference::Auto:
                result.hole.drillCycleMode = HoleDrillCycleMode::Auto;
                break;
            case ToolpathDrillCyclePreference::Direct:
                result.hole.drillCycleMode = HoleDrillCycleMode::Direct;
                break;
            case ToolpathDrillCyclePreference::Peck:
                result.hole.drillCycleMode = HoleDrillCycleMode::Peck;
                break;
            case ToolpathDrillCyclePreference::ChipBreak:
                result.hole.drillCycleMode = HoleDrillCycleMode::ChipBreak;
                break;
        }
    } else if (tool.valid) {
        result.hole.drillCycleMode = HoleDrillCycleMode::Direct;
        result.hole.peckDepth = 0.0;
        result.hole.drillTipAngleDeg = 0.0;
        result.tool.tipCompensation = 0.0;
    }
    return result;
}

void appendHoleMachiningSummary(
    HoleMachiningResult& target,
    const HoleMachiningResult& source)
{
    target.smallCount += source.smallCount;
    target.mediumCount += source.mediumCount;
    target.largeCount += source.largeCount;
    target.skippedCount += source.skippedCount;
    target.directRapidLinkCount += source.directRapidLinkCount;
    target.liftedRapidLinkCount += source.liftedRapidLinkCount;
    target.unsafeRapidLinkCount += source.unsafeRapidLinkCount;
    target.broadPhaseSafeSegmentCount += source.broadPhaseSafeSegmentCount;
    target.exactCollisionCheckCount += source.exactCollisionCheckCount;
    target.solidSweepCheckCount += source.solidSweepCheckCount;
    target.solidSweepConstructedPartCount +=
        source.solidSweepConstructedPartCount;
    target.nonCuttingMoveCollisionCount +=
        source.nonCuttingMoveCollisionCount;
    target.cuttingMoveCollisionCount += source.cuttingMoveCollisionCount;
    target.arcSweepPrimitiveCount += source.arcSweepPrimitiveCount;
    target.twoOptImprovementCount += source.twoOptImprovementCount;
    target.relocationImprovementCount += source.relocationImprovementCount;
    target.threeOptImprovementCount += source.threeOptImprovementCount;
    target.rapidRepairRelocationCount += source.rapidRepairRelocationCount;
    target.rapidRepairRecoveredLinkCount +=
        source.rapidRepairRecoveredLinkCount;
    target.nearestNeighborRouteLength += source.nearestNeighborRouteLength;
    target.twoOptRouteLength += source.twoOptRouteLength;
    target.relocationRouteLength += source.relocationRouteLength;
    target.optimizedRouteLength += source.optimizedRouteLength;
    target.warnings.insert(
        target.warnings.end(), source.warnings.begin(), source.warnings.end());
    target.diagnostics.insert(
        target.diagnostics.end(), source.diagnostics.begin(),
        source.diagnostics.end());
}

} // namespace oap::toolpath

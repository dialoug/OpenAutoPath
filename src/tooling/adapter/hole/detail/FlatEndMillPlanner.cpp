// 为钻头无法覆盖的孔选择平底铣刀及孔加工策略，并形成同刀具加工批次。
// 属于 hole 模块 detail 私有实现;仅由 HoleSetupPlanner 调用。
#include "tooling/adapter/hole/detail/HolePhasePlanners.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <sstream>
#include <utility>

#include "tooling/adapter/hole/detail/HolePlanningSupport.h"

namespace oap::tooling::detail {

void appendFlatEndMillBatch(
    const HoleToolCatalog& catalog,
    const std::vector<oap::toolpath::HoleMachiningInput>& holes,
    const std::vector<PlannedPhysicalHole>& eligible,
    oap::MoldSetupFace face,
    int setupIndex,
    const oap::toolpath::ToolpathParams& params,
    double tolerance,
    HoleToolSelectionPlan& result)
{
    // 平底铣目标分两类:平面底盲孔(钻头无法成形平面底)与沉孔大径肩部
    // (需铣削扩径);后者逐孔段判定是否真正需要铣削(显式大径/旧版更大径/
    // 孤立沉孔)且具备可铣肩部。
    std::vector<HoleToolTask> targets;
    for (const PlannedPhysicalHole& physical : eligible) {
        const oap::toolpath::HoleMachiningInput& minimum =
            holes[physical.minimumSegmentIndex];
        if (minimum.kind == oap::FeatureKind::BlindHole &&
            minimum.bottomProfile ==
                oap::HoleBottomProfile::Planar) {
            HoleToolTask task;
            task.hole = makePilotHole(physical, holes);
            task.sourceHoleIndex = physical.minimumSegmentIndex;
            task.coveredSourceHoleIndices =
                physical.minimumDiameterSegmentIndices;
            task.physicalHoleIndex = physical.stableIndex;
            targets.push_back(std::move(task));
        }

        if (minimum.kind != oap::FeatureKind::CountersinkHole)
            continue;
        for (const std::size_t sourceIndex :
             physical.segmentIndices) {
            const oap::toolpath::HoleMachiningInput& stage = holes[sourceIndex];
            const bool explicitLarge =
                stage.stageRole ==
                oap::HoleStageRole::CountersinkLargeDiameter;
            const bool legacyLarger =
                2.0 * stage.radius >
                physical.minimumDiameter + tolerance;
            const bool legacyStandalone =
                physical.segmentIndices.size() == 1 &&
                stage.kind == oap::FeatureKind::CountersinkHole;
            const bool millableShoulder =
                stage.bottomProfile ==
                    oap::HoleBottomProfile::Planar ||
                (explicitLarge &&
                 stage.bottomProfile ==
                    oap::HoleBottomProfile::Unknown);
            if ((!explicitLarge && !legacyLarger &&
                 !legacyStandalone) ||
                !millableShoulder) {
                continue;
            }
            HoleToolTask task;
            task.hole = stage;
            task.hole.entry = physical.outerEntry;
            task.hole.preserveEntrySide = true;
            task.sourceHoleIndex = sourceIndex;
            task.coveredSourceHoleIndices = {sourceIndex};
            task.physicalHoleIndex = physical.stableIndex;
            targets.push_back(std::move(task));
        }
    }

    if (targets.empty())
        return;
    if (catalog.flatEndMills.empty()) {
        // 与可省略的中心定位不同，平底盲孔/沉孔肩部没有平底刀就无法
        // 完成材料去除。逐孔记录 Error，让覆盖审计视图明确显示“缺刀”，
        // 而不是只留下一个没有原因的漏孔编号。
        for (const HoleToolTask& task : targets) {
            std::ostringstream message;
            message << "setup " << oap::moldSetupFaceName(face)
                    << " flat target " << task.sourceHoleIndex + 1
                    << " has no assigned flat end mill";
            appendHoleToolDiagnostic(
                result,
                oap::toolpath::ToolpathDiagnosticCode::AutoToolUnavailable,
                oap::toolpath::ToolpathDiagnosticSeverity::Error,
                message.str(),
                static_cast<int>(task.sourceHoleIndex));
            markPhysicalHoleSkipped(
                result, task.physicalHoleIndex);
        }
        return;
    }

    // 对每个目标选最大可用平底铣刀:自动推导加工策略,
    // 排除点钻策略,并校验刃长/伸长;均不满足时报缺刀并跳过该物理孔。
    std::vector<HoleToolBatch> batches;
    for (HoleToolTask& task : targets) {
        const double targetDiameter = 2.0 * task.hole.radius;
        const HoleToolCandidate* sizeMatched =
            findLargestNotExceeding(
                catalog.flatEndMills, targetDiameter, tolerance);
        if (!sizeMatched) {
            std::ostringstream message;
            message << "setup "
                    << oap::moldSetupFaceName(face)
                    << " flat target "
                    << task.sourceHoleIndex + 1
                    << " diameter "
                    << targetDiameter
                    << " mm is below the minimum flat end mill";
            appendHoleToolDiagnostic(
                result,
                oap::toolpath::ToolpathDiagnosticCode::
                    AutoHoleBelowToolDiameter,
                oap::toolpath::ToolpathDiagnosticSeverity::Warning,
                message.str(),
                static_cast<int>(task.sourceHoleIndex));
            markPhysicalHoleSkipped(
                result, task.physicalHoleIndex);
            continue;
        }

        const HoleToolCandidate* selected = nullptr;
        oap::toolpath::HoleMachiningStrategy selectedStrategy =
            oap::toolpath::HoleMachiningStrategy::Auto;
        for (const HoleToolCandidate& candidate :
             catalog.flatEndMills) {
            if (candidate.context.diameter >
                targetDiameter + tolerance) {
                break;
            }
            const oap::toolpath::HoleMachiningStrategy strategy =
                automaticHoleStrategy(
                        task.hole, candidate.context.diameter,
                        params.hole, false);
            if (strategy ==
                oap::toolpath::HoleMachiningStrategy::SmallDrill) {
                continue;
            }
            oap::toolpath::ToolpathParams effective = params;
            effective.tool.diameter = candidate.context.diameter;
            effective.tool.tipCompensation = 0.0;
            effective.hole.drillTipAngleDeg = 0.0;
            const double cuttingRequirement =
                oap::toolpath::holeMinimumCuttingEdgeLength(
                    task.hole, strategy, effective);
            const double reachRequirement =
                oap::toolpath::holeMinimumToolReach(
                    task.hole, strategy, effective);
            if (!toolHasRequiredCuttingEdge(
                    candidate, cuttingRequirement) ||
                !toolHasRequiredReach(candidate, reachRequirement)) {
                continue;
            }
            if (!selected ||
                candidate.context.diameter >
                    selected->context.diameter +
                        kHoleToolMatchTolerance) {
                selected = &candidate;
                selectedStrategy = strategy;
            }
        }
        if (!selected) {
            std::ostringstream message;
            message << "setup "
                    << oap::moldSetupFaceName(face)
                    << " flat target "
                    << task.sourceHoleIndex + 1
                    << " has no usable T"
                    << sizeMatched->context.toolNumber << ' '
                    << sizeMatched->context.toolName
                    << " or smaller flat end mill satisfying both "
                       "cutting-edge and total-reach requirements";
            appendHoleToolDiagnostic(
                result,
                oap::toolpath::ToolpathDiagnosticCode::
                    AutoToolReachInsufficient,
                oap::toolpath::ToolpathDiagnosticSeverity::Error,
                message.str(),
                static_cast<int>(task.sourceHoleIndex));
            markPhysicalHoleSkipped(
                result, task.physicalHoleIndex);
            continue;
        }

        task.strategy = selectedStrategy;

        auto batch = std::find_if(
            batches.begin(), batches.end(),
            [selected](const HoleToolBatch& value) {
                return value.tool.toolNumber ==
                       selected->context.toolNumber;
            });
        if (batch == batches.end()) {
            HoleToolBatch value;
            value.setupFace = face;
            value.setupIndex = setupIndex;
            value.phase =
                oap::toolpath::HoleMachiningPhase::FlatEndMill;
            value.tool = selected->context;
            batches.push_back(std::move(value));
            batch = std::prev(batches.end());
        }
        batch->tasks.push_back(std::move(task));
    }

    std::stable_sort(
        batches.begin(), batches.end(),
        [](const HoleToolBatch& first,
           const HoleToolBatch& second) {
            if (std::abs(first.tool.diameter -
                         second.tool.diameter) >
                kHoleToolMatchTolerance) {
                return first.tool.diameter <
                       second.tool.diameter;
            }
            return first.tool.toolNumber <
                   second.tool.toolNumber;
        });
    for (HoleToolBatch& batch : batches) {
        appendHoleAssignmentDiagnostic(result, batch);
        result.batches.push_back(std::move(batch));
    }
}

} // namespace oap::tooling::detail

// 按孔径、深度和可用钻头规划钻削批次，无法钻削的孔留给后续铣削阶段。
// 属于 hole 模块 detail 私有实现;仅由 HoleSetupPlanner 调用。
#include "tooling/adapter/hole/detail/HolePhasePlanners.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <sstream>
#include <utility>

#include "toolpath/hole/DrillTipParameterBinding.h"
#include "tooling/adapter/hole/detail/HolePlanningSupport.h"

namespace oap::tooling::detail {
namespace {

constexpr double kMaximumConicalBlindDrillDeficit = 1.0;

bool isMillingStrategy(oap::toolpath::HoleMachiningStrategy strategy)
{
    return strategy ==
               oap::toolpath::HoleMachiningStrategy::MediumHelical ||
           strategy ==
               oap::toolpath::HoleMachiningStrategy::LargeLayeredSpiral;
}

// 闭底孔直径匹配策略:锥形盲孔允许钻头欠径至多 kMaximumConicalBlindDrillDeficit
// (靠钻尖锥形参与成形),平面盲孔/沉孔必须严格等径。
bool candidateMatchesClosedBottom(
    const HoleToolCandidate& candidate,
    double targetDiameter,
    bool conicalBlind,
    double tolerance)
{
    if (candidate.context.diameter > targetDiameter + tolerance)
        return false;
    if (conicalBlind) {
        return targetDiameter - candidate.context.diameter <=
            kMaximumConicalBlindDrillDeficit +
                kHoleToolMatchTolerance;
    }
    return std::abs(candidate.context.diameter - targetDiameter) <=
        tolerance;
}

// 为单个钻削任务选刀:遍历升序钻头,逐项检查直径策略/刃长/伸长,
// 记录第一个匹配直径的候选(hasDiameterCandidate),并选择直径最大者;
// 返回选定刀具与对应策略(自动策略按孔型与刀径比推导)。
const HoleToolCandidate* selectUsableDrill(
    const HoleToolCatalog& catalog,
    const HoleToolTask& task,
    const oap::toolpath::ToolpathParams& params,
    double tolerance,
    oap::toolpath::HoleMachiningStrategy& selectedStrategy,
    bool& hasDiameterCandidate)
{
    const double targetDiameter = 2.0 * task.hole.radius;
    const bool openTarget =
        task.hole.kind == oap::FeatureKind::ThroughHole ||
        task.hole.bottomProfile == oap::HoleBottomProfile::Open;
    const bool conicalBlind =
        task.hole.kind == oap::FeatureKind::BlindHole &&
        task.hole.bottomProfile == oap::HoleBottomProfile::Conical;
    const oap::toolpath::HoleMachiningStrategy requested =
        oap::toolpath::HoleMachiningStrategy::Auto;

    const HoleToolCandidate* selected = nullptr;
    for (const HoleToolCandidate& candidate : catalog.drills) {
        if (candidate.context.diameter > targetDiameter + tolerance)
            break;
        const bool diameterMatches = openTarget ||
            candidateMatchesClosedBottom(
                candidate, targetDiameter, conicalBlind, tolerance);
        if (!diameterMatches)
            continue;
        hasDiameterCandidate = true;

        const oap::toolpath::HoleMachiningStrategy automatic =
            openTarget
                ? automaticHoleStrategy(
                      task.hole, candidate.context.diameter,
                      params.hole, false)
                : oap::toolpath::HoleMachiningStrategy::SmallDrill;
        const oap::toolpath::HoleMachiningStrategy strategy =
            requested == oap::toolpath::HoleMachiningStrategy::Auto
                ? automatic
                : requested;
        if (!openTarget && isMillingStrategy(strategy))
            continue;
        if (strategy ==
                oap::toolpath::HoleMachiningStrategy::SmallDrill &&
            openTarget &&
            std::abs(candidate.context.diameter - targetDiameter) >
                tolerance) {
            continue;
        }
        if (isMillingStrategy(strategy) &&
            0.5 * candidate.context.diameter +
                    std::max(0.0, params.hole.allowance) >=
                task.hole.radius - tolerance) {
            continue;
        }

        oap::toolpath::ToolpathParams effective = params;
        effective.tool.diameter = candidate.context.diameter;
        oap::toolpath::applyDrillTipGeometry(
            candidate.context.tipCompensation,
            candidate.context.tipAngle, effective);
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
    return selected;
}

} // namespace

void appendDrillBatches(
    const HoleToolCatalog& catalog,
    const std::vector<oap::toolpath::HoleMachiningInput>& holes,
    const std::vector<PlannedPhysicalHole>& eligible,
    oap::MoldSetupFace face,
    int setupIndex,
    const oap::toolpath::ToolpathParams& params,
    double tolerance,
    HoleToolSelectionPlan& result)
{
    // 钻削目标:跳过沉孔大径段与平面盲孔段(两者由平底铣承担),
    // 其余物理孔以最小直径段构造引导孔。
    std::vector<HoleToolTask> targets;
    for (const PlannedPhysicalHole& physical : eligible) {
        const oap::toolpath::HoleMachiningInput& minimum =
            holes[physical.minimumSegmentIndex];
        const bool counterboreLarge =
            minimum.stageRole ==
            oap::HoleStageRole::CountersinkLargeDiameter;
        if (counterboreLarge)
            continue;
        const bool planarBlind =
            minimum.kind == oap::FeatureKind::BlindHole &&
            minimum.bottomProfile ==
                oap::HoleBottomProfile::Planar;
        if (planarBlind)
            continue;

        HoleToolTask task;
        task.hole = makePilotHole(physical, holes);
        task.sourceHoleIndex = physical.minimumSegmentIndex;
        task.coveredSourceHoleIndices =
            physical.minimumDiameterSegmentIndices;
        task.physicalHoleIndex = physical.stableIndex;
        targets.push_back(std::move(task));
    }

    std::vector<HoleToolBatch> batches;
    for (HoleToolTask& task : targets) {
        const double targetDiameter = 2.0 * task.hole.radius;
        const bool openTarget =
            task.hole.kind == oap::FeatureKind::ThroughHole ||
            task.hole.bottomProfile == oap::HoleBottomProfile::Open;
        const oap::toolpath::HoleMachiningStrategy requested =
            oap::toolpath::HoleMachiningStrategy::Auto;
        if (!openTarget && isMillingStrategy(requested)) {
            std::ostringstream message;
            message << "setup " << oap::moldSetupFaceName(face)
                    << " closed-bottom hole "
                    << task.sourceHoleIndex + 1
                    << " cannot use a milling strategy";
            appendHoleToolDiagnostic(
                result,
                oap::toolpath::ToolpathDiagnosticCode::AutoToolUnavailable,
                oap::toolpath::ToolpathDiagnosticSeverity::Error,
                message.str(), static_cast<int>(task.sourceHoleIndex));
            markPhysicalHoleSkipped(result, task.physicalHoleIndex);
            continue;
        }

        bool hasDiameterCandidate = false;
        oap::toolpath::HoleMachiningStrategy selectedStrategy =
            oap::toolpath::HoleMachiningStrategy::SmallDrill;
        const HoleToolCandidate* selected =
            selectUsableDrill(
                catalog, task, params, tolerance,
                selectedStrategy, hasDiameterCandidate);
        if (!selected) {
            std::ostringstream message;
            if (hasDiameterCandidate) {
                message << "setup " << oap::moldSetupFaceName(face)
                        << " hole " << task.sourceHoleIndex + 1
                        << " has no diameter-compatible drill satisfying "
                           "cutting-edge, total-reach, and strategy requirements";
                appendHoleToolDiagnostic(
                    result,
                    oap::toolpath::ToolpathDiagnosticCode::
                        AutoToolReachInsufficient,
                    oap::toolpath::ToolpathDiagnosticSeverity::Error,
                    message.str(),
                    static_cast<int>(task.sourceHoleIndex));
            } else {
                message << "setup " << oap::moldSetupFaceName(face)
                        << " hole " << task.sourceHoleIndex + 1
                        << " diameter " << targetDiameter << " mm has no "
                        << (openTarget
                                ? "drill not exceeding the target diameter"
                                : "drill matching the closed-bottom policy");
                appendHoleToolDiagnostic(
                    result,
                    oap::toolpath::ToolpathDiagnosticCode::
                        AutoToolUnavailable,
                    oap::toolpath::ToolpathDiagnosticSeverity::Error,
                    message.str(),
                    static_cast<int>(task.sourceHoleIndex));
            }
            markPhysicalHoleSkipped(result, task.physicalHoleIndex);
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
                oap::toolpath::HoleMachiningPhase::Drill;
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

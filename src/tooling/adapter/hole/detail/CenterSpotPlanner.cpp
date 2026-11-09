// 为需要定位的物理孔选择中心钻，并按装夹和刀具组织点钻批次。
// 属于 hole 模块 detail 私有实现;仅由 HoleSetupPlanner 调用,
// 中心钻作为可选预加工,不满足时降级为跳过(非阻断错误)。
#include "tooling/adapter/hole/detail/HolePhasePlanners.h"

#include <algorithm>
#include <sstream>
#include <utility>

#include <Precision.hxx>
#include <gp_Vec.hxx>

#include "tooling/adapter/hole/detail/HolePlanningSupport.h"

namespace oap::tooling::detail {

void appendCenterSpotBatch(
    const HoleToolCatalog& catalog,
    const std::vector<oap::toolpath::HoleMachiningInput>& holes,
    const std::vector<PlannedPhysicalHole>& eligible,
    oap::MoldSetupFace face,
    int setupIndex,
    const oap::toolpath::ToolpathParams& params,
    double tolerance,
    HoleToolSelectionPlan& result)
{
    if (catalog.centerDrills.empty()) {
        appendHoleToolDiagnostic(
            result,
            oap::toolpath::ToolpathDiagnosticCode::AutoToolUnavailable,
            oap::toolpath::ToolpathDiagnosticSeverity::Warning,
            "setup " + std::string(oap::moldSetupFaceName(face)) +
                " has no assigned center drill");
        return;
    }

    // 每个装夹面尽量只换一次中心钻：从活动刀库选择不超过该面最小孔径的
    // 最大中心钻；若全部超径，则使用实际最小刀并逐孔跳过不兼容任务。
    const double minimumHoleDiameter = std::min_element(
        eligible.begin(), eligible.end(),
        [](const PlannedPhysicalHole& first,
           const PlannedPhysicalHole& second) {
            return first.minimumDiameter < second.minimumDiameter;
        })->minimumDiameter;
    // 点钻深度:取 centerSpotDepth 与各孔“外端到最深底”投影深度的较小值,
    // 深度不足容差的物理孔不点钻。
    double maximumSpotDepth = 0.0;
    for (const PlannedPhysicalHole& physical : eligible) {
        const double totalDepth =
            gp_Vec(physical.outerEntry, physical.deepestBottom)
                .Dot(physical.inwardAxis);
        maximumSpotDepth = std::max(
            maximumSpotDepth,
            std::min(
                params.hole.centerSpotDepth,
                std::max(0.0, totalDepth - 1.0e-4)));
    }
    if (maximumSpotDepth <= Precision::Confusion())
        return;

    const HoleToolCandidate* center =
        findLargestUsableNotExceeding(
            catalog.centerDrills, minimumHoleDiameter,
            0.0, maximumSpotDepth, tolerance);
    if (!center) {
        const auto reachable = std::find_if(
            catalog.centerDrills.begin(),
            catalog.centerDrills.end(),
            [maximumSpotDepth](const HoleToolCandidate& candidate) {
                return toolHasRequiredReach(
                    candidate, maximumSpotDepth);
            });
        if (reachable == catalog.centerDrills.end()) {
            for (const PlannedPhysicalHole& physical : eligible) {
                std::ostringstream message;
                message << "setup " << oap::moldSetupFaceName(face)
                        << " physical hole "
                        << physical.stableIndex + 1
                        << " not spotted: assigned center drills do not reach "
                        << maximumSpotDepth << " mm";
                appendHoleToolDiagnostic(
                    result,
                    oap::toolpath::ToolpathDiagnosticCode::
                        AutoCenterSpotSkipped,
                    oap::toolpath::ToolpathDiagnosticSeverity::Warning,
                    message.str(),
                    static_cast<int>(physical.minimumSegmentIndex));
            }
            return;
        }
        center = &*reachable;
    }

    HoleToolBatch batch;
    batch.setupFace = face;
    batch.setupIndex = setupIndex;
    batch.phase = oap::toolpath::HoleMachiningPhase::CenterSpot;
    batch.tool = center->context;
    for (const PlannedPhysicalHole& physical : eligible) {
        if (center->context.diameter >
            physical.minimumDiameter + tolerance) {
            std::ostringstream message;
            message << "setup " << oap::moldSetupFaceName(face)
                    << " physical hole "
                    << physical.stableIndex + 1
                    << " not spotted: T"
                    << center->context.toolNumber
                    << " diameter exceeds hole";
            appendHoleToolDiagnostic(
                result,
                oap::toolpath::ToolpathDiagnosticCode::
                    AutoCenterSpotSkipped,
                oap::toolpath::ToolpathDiagnosticSeverity::Warning,
                message.str(),
                static_cast<int>(physical.minimumSegmentIndex));
            continue;
        }

        const double totalDepth =
            gp_Vec(physical.outerEntry, physical.deepestBottom)
                .Dot(physical.inwardAxis);
        const double spotDepth = std::min(
            params.hole.centerSpotDepth,
            std::max(0.0, totalDepth - 1.0e-4));
        if (spotDepth <= Precision::Confusion())
            continue;

        HoleToolTask task;
        task.hole = makePilotHole(physical, holes);
        task.hole.bottom = physical.outerEntry.Translated(
            physical.inwardAxis * spotDepth);
        task.hole.bottomProfile =
            oap::HoleBottomProfile::Unknown;
        task.sourceHoleIndex = physical.minimumSegmentIndex;
        task.physicalHoleIndex = physical.stableIndex;
        task.strategy =
            oap::toolpath::HoleMachiningStrategy::SmallDrill;
        batch.tasks.push_back(std::move(task));
    }

    if (!batch.tasks.empty()) {
        appendHoleAssignmentDiagnostic(result, batch);
        result.batches.push_back(std::move(batch));
    }
}

} // namespace oap::tooling::detail

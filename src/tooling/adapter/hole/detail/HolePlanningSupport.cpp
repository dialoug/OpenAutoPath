// 孔选刀流程共享的参数快照、诊断构造、稳定排序和批次键辅助实现。
// 属于 hole 模块 detail 私有实现;自动策略推导为各阶段统一口径。
#include "tooling/adapter/hole/detail/HolePlanningSupport.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace oap::tooling::detail {

oap::toolpath::HoleMachiningStrategy automaticHoleStrategy(
    const oap::toolpath::HoleMachiningInput& hole,
    double toolDiameter,
    const oap::toolpath::HoleMachiningParams& params,
    bool forceBlindCenterline)
{
    using oap::toolpath::HoleMachiningStrategy;
    if (forceBlindCenterline)
        return HoleMachiningStrategy::SmallDrill;
    // 策略推导:点钻(等径或强制盲孔中心线) -> 中孔螺旋(刀径比阈值内)
    // -> 大孔分层螺旋。mediumHoleRadiusToolDiameterRatio 可调,默认 0.95。
    const double holeDiameter = 2.0 * hole.radius;
    if (std::abs(holeDiameter - toolDiameter) <=
        params.smallHoleDiameterTolerance) {
        return HoleMachiningStrategy::SmallDrill;
    }
    const double mediumRatio =
        std::isfinite(params.mediumHoleRadiusToolDiameterRatio) &&
                params.mediumHoleRadiusToolDiameterRatio > 0.0
            ? std::min(params.mediumHoleRadiusToolDiameterRatio, 1.0)
            : 0.95;
    if (hole.radius <= mediumRatio * toolDiameter) {
        return HoleMachiningStrategy::MediumHelical;
    }
    return HoleMachiningStrategy::LargeLayeredSpiral;
}



void appendHoleToolDiagnostic(
    HoleToolSelectionPlan& result,
    oap::toolpath::ToolpathDiagnosticCode code,
    oap::toolpath::ToolpathDiagnosticSeverity severity,
    std::string message,
    int sourceIndex)
{
    oap::toolpath::ToolpathDiagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.severity = severity;
    diagnostic.message = std::move(message);
    diagnostic.sourceIndex = sourceIndex;
    result.diagnostics.push_back(std::move(diagnostic));
}

void appendHoleAssignmentDiagnostic(
    HoleToolSelectionPlan& result,
    const HoleToolBatch& batch)
{
    std::ostringstream message;
    message << "setup " << oap::moldSetupFaceName(batch.setupFace)
            << " phase="
            << oap::toolpath::holeMachiningPhaseName(batch.phase)
            << " T" << batch.tool.toolNumber << " "
            << batch.tool.toolName
            << " diameter=" << batch.tool.diameter
            << " mm tasks=" << batch.tasks.size();
    appendHoleToolDiagnostic(
        result,
        oap::toolpath::ToolpathDiagnosticCode::AutoToolAssignment,
        oap::toolpath::ToolpathDiagnosticSeverity::Info,
        message.str());
}

void markPhysicalHoleSkipped(
    HoleToolSelectionPlan& result,
    std::size_t physicalHoleIndex)
{
    if (std::find(
            result.skippedPhysicalHoleIndices.begin(),
            result.skippedPhysicalHoleIndices.end(),
            physicalHoleIndex) !=
        result.skippedPhysicalHoleIndices.end()) {
        return;
    }
    result.skippedPhysicalHoleIndices.push_back(physicalHoleIndex);
    result.skippedPhysicalHoleCount =
        static_cast<int>(result.skippedPhysicalHoleIndices.size());
}

} // namespace oap::tooling::detail

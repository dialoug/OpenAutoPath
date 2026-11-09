// 检查归一化前后都应满足的公共工艺参数约束，并以警告说明可回退配置。
#include "toolpath/core/validation/ToolpathParameterValidator.h"

#include <cmath>

namespace oap::toolpath {

// 参数结构校验：检查归一化前后都应满足的公共约束；
// Error 级（刀径非法）阻断生成，Warning 级说明生成器将回退的配置。
void validateToolpathParameters(
    const ToolpathParams& params,
    ToolpathValidationReport& report)
{
    if (!std::isfinite(params.tool.diameter) ||
        params.tool.diameter <= 0.0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Error,
            "invalid_tool_diameter",
            "tool diameter must be positive");
    }
    if (!std::isfinite(params.depth.safeHeight) ||
        !std::isfinite(params.depth.clearanceHeight) ||
        params.depth.safeHeight < 0.0 ||
        params.depth.clearanceHeight < 0.0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "negative_safe_height",
            "safe/clearance offsets should be finite and non-negative");
    }
    if (std::isfinite(params.depth.clearanceHeight) &&
        std::isfinite(params.depth.safeHeight) &&
        params.depth.clearanceHeight < params.depth.safeHeight) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "clearance_below_safe",
            "clearance offset is below safe offset; generator will raise it");
    }
    if (!std::isfinite(params.depth.stepDown) ||
        params.depth.stepDown <= 0.0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "invalid_step_down",
            "step-down should be positive; generator may fall back to a single layer");
    }
    if (params.clearing.expansionDirection !=
            PocketClearingDirection::OutsideToInside &&
        params.clearing.expansionDirection !=
            PocketClearingDirection::InsideToOutside) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "invalid_pocket_clearing_direction",
            params.clearing.pattern == ClearingPattern::FreePath
                ? "FreePath clearing direction is invalid; the generator "
                  "will use the pyocc-exp center-out order"
                : "pocket clearing direction is invalid; the generator will "
                  "use outside-to-inside");
    }
    if (params.clearing.pattern == ClearingPattern::FreePath) {
        const AdaptiveClearingParams& adaptive = params.clearing.adaptive;
        if (!std::isfinite(adaptive.targetEngagement) ||
            adaptive.targetEngagement <= 0.0 ||
            adaptive.targetEngagement > 1.0 ||
            !std::isfinite(adaptive.targetEngagementAngleDeg) ||
            !std::isfinite(adaptive.maximumEngagementAngleDeg) ||
            adaptive.targetEngagementAngleDeg <= 0.0 ||
            adaptive.targetEngagementAngleDeg >
                adaptive.maximumEngagementAngleDeg ||
            adaptive.maximumEngagementAngleDeg > 360.0 ||
            adaptive.engagementAngleSamples < 8 ||
            !std::isfinite(adaptive.minimumTurnRadiusFactor) ||
            adaptive.minimumTurnRadiusFactor <= 0.0 ||
            !std::isfinite(adaptive.remainingTolerance) ||
            adaptive.remainingTolerance < 0.0 ||
            adaptive.remainingTolerance >= 1.0 ||
            !std::isfinite(adaptive.materialCellSize) ||
            adaptive.materialCellSize < 0.0 ||
            adaptive.maximumAdaptivePasses <= 0 ||
            adaptive.lookaheadDepth < 1 || adaptive.lookaheadDepth > 4 ||
            adaptive.beamWidth < 1 || adaptive.beamWidth > 16) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Warning,
                "invalid_adaptive_clearing_parameters",
                "Adaptive FreePath engagement, turn-radius, residual-grid, "
                "pass, or lookahead settings are invalid; the generator "
                "will normalize them to pyocc-exp-compatible bounds");
        }
    }
    if (!std::isfinite(params.clearing.rampAngleDeg) ||
        params.clearing.rampAngleDeg <= 0.0 ||
        params.clearing.rampAngleDeg > 45.0 ||
        !std::isfinite(params.clearing.maximumRampLength) ||
        params.clearing.maximumRampLength < 0.0 ||
        !std::isfinite(params.clearing.helixDiameter) ||
        params.clearing.helixDiameter < 0.0 ||
        !std::isfinite(params.clearing.zigzagRampWidth) ||
        params.clearing.zigzagRampWidth < 0.0 ||
        !std::isfinite(params.clearing.rampFeed) ||
        params.clearing.rampFeed < 0.0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "invalid_pocket_entry_parameters",
            "pocket ramp angle/length/helix diameter/zigzag width/feed are invalid; "
            "the generator will normalize or fall back to vertical plunge");
    }
    if (!std::isfinite(params.hole.allowance)) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "invalid_hole_allowance",
            "hole allowance is not finite; generator will use zero");
    } else if (params.hole.allowance < 0.0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "negative_hole_allowance",
            "negative hole allowance will be clamped to zero");
    }
    if (!std::isfinite(params.hole.mediumHoleRadiusToolDiameterRatio) ||
        params.hole.mediumHoleRadiusToolDiameterRatio <= 0.0 ||
        params.hole.mediumHoleRadiusToolDiameterRatio > 1.0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "invalid_automatic_medium_ratio",
            "automatic medium-hole radius/tool-diameter ratio must be in (0, 1]");
    }
    if (!std::isfinite(params.tool.tipCompensation) ||
        params.tool.tipCompensation < 0.0 ||
        !std::isfinite(params.hole.throughHoleBreakthrough) ||
        params.hole.throughHoleBreakthrough < 0.0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "invalid_drill_depth_compensation",
            "drill-tip compensation and through-hole breakthrough must be finite and non-negative");
    }
    if (!std::isfinite(params.hole.g81MaxDepthDiameterRatio) ||
        !std::isfinite(params.hole.g73MaxDepthDiameterRatio) ||
        params.hole.g81MaxDepthDiameterRatio < 0.0 ||
        params.hole.g73MaxDepthDiameterRatio <
            params.hole.g81MaxDepthDiameterRatio ||
        !std::isfinite(params.hole.g73PeckDepth) ||
        params.hole.g73PeckDepth <= 0.0 ||
        !std::isfinite(params.hole.g83PeckDepth) ||
        params.hole.g83PeckDepth <= 0.0 ||
        !std::isfinite(params.hole.g73ChipBreakRetract) ||
        params.hole.g73ChipBreakRetract <= 0.0 ||
        !std::isfinite(params.hole.g83ReentryClearance) ||
        params.hole.g83ReentryClearance < 0.0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "invalid_drill_cycle_policy",
            "drill-cycle thresholds and G73/G83 expansion parameters are invalid");
    }
    if (!std::isfinite(
            params.hole.smallHoleDiameterTolerance) ||
        params.hole.smallHoleDiameterTolerance < 0.0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "invalid_small_hole_diameter_tolerance",
            "small-hole diameter tolerance must be finite and non-negative");
    }
    if (!std::isfinite(params.hole.sampleAngleDeg)) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "invalid_hole_sample_angle",
            "hole sampling angle is not finite; generator will use 10 degrees");
    }
    if (!std::isfinite(params.hole.helixChordTolerance) ||
        params.hole.helixChordTolerance < 0.0) {
        addToolpathIssue(report, ToolpathIssueSeverity::Error,
                        "invalid_helix_chord_tolerance",
                        "helix chord tolerance must be finite and non-negative");
    }
    if (params.hole.maxSamplePoints < 16) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "small_hole_sample_budget",
            "hole sampling point budget is too small; generator will use at least 16");
    }
    if (!std::isfinite(params.hole.peckDepth) ||
        params.hole.peckDepth < 0.0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "invalid_drill_peck_depth",
            "drill peck-depth override must be finite and non-negative");
    }
    if (!std::isfinite(params.hole.dwellSeconds) ||
        params.hole.dwellSeconds < 0.0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "invalid_drill_dwell",
            "drill dwell time must be finite and non-negative");
    }
    if (!std::isfinite(params.hole.drillTipAngleDeg) ||
        params.hole.drillTipAngleDeg < 0.0 ||
        params.hole.drillTipAngleDeg >= 180.0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "invalid_drill_tip_angle",
            "drill tip angle must be zero (disabled) or in [0, 180) degrees");
    }
    if (!std::isfinite(params.hole.collisionEnvelopeSampleStep) ||
        params.hole.collisionEnvelopeSampleStep <= 0.0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "invalid_tool_envelope_sample_step",
            "tool-assembly envelope sample step must be finite and positive");
    }
    const bool completeToolAssembly =
        std::isfinite(params.tool.cuttingEdgeLength) &&
        params.tool.cuttingEdgeLength > 0.0 &&
        std::isfinite(params.tool.overallLength) &&
        params.tool.overallLength >= params.tool.cuttingEdgeLength &&
        std::isfinite(params.tool.shankDiameter) &&
        params.tool.shankDiameter > 0.0 &&
        std::isfinite(params.tool.holderDiameter) &&
        params.tool.holderDiameter > 0.0 &&
        std::isfinite(params.tool.holderLength) &&
        params.tool.holderLength > 0.0 &&
        std::isfinite(params.tool.spindleEnvelopeDiameter) &&
        params.tool.spindleEnvelopeDiameter > 0.0 &&
        std::isfinite(params.tool.spindleEnvelopeLength) &&
        params.tool.spindleEnvelopeLength > 0.0;
    if (params.hole.collisionEnvelopeMode !=
            HoleCollisionEnvelopeMode::CutterOnly &&
        !completeToolAssembly) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "incomplete_tool_assembly",
            "tool-assembly geometry is incomplete; collision checks will use available components and omit unknown components");
    }
    if ((params.tool.spindleEnvelopeDiameter > 0.0) !=
        (params.tool.spindleEnvelopeLength > 0.0)) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "incomplete_spindle_envelope",
            "spindle-head collision envelope is incomplete and will be omitted from collision checks");
    }
}

} // namespace oap::toolpath

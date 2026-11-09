// 对外部刀路参数执行一次性归一化，并为每个自动修正生成稳定诊断。
#include "ToolpathParamNormalizer.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace oap::toolpath {

namespace {

void addDiagnostic(ToolpathParamNormalization& result,
                   ToolpathDiagnosticCode code,
                   ToolpathDiagnosticSeverity severity,
                   const char* message)
{
    ToolpathDiagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.severity = severity;
    diagnostic.message = message;
    result.diagnostics.push_back(std::move(diagnostic));
}

bool invalidNonNegative(double value)
{
    return !std::isfinite(value) || value < 0.0;
}

void normalizeNonNegative(double& value,
                          ToolpathParamNormalization& result,
                          ToolpathDiagnosticCode code,
                          const char* field)
{
    // 负数、NaN/Inf 一律归零并告警；正数保留原值。返回值修改的是
    // result.effective 中的字段，保证生成器拿到的是修正后的参数。
    if (!invalidNonNegative(value))
        return;
    value = 0.0;
    const std::string message = std::string(field) + " was invalid and was set to zero";
    addDiagnostic(result, code, ToolpathDiagnosticSeverity::Warning,
                  message.c_str());
}

} // namespace

bool ToolpathParamNormalization::ok() const noexcept
{
    return std::none_of(
        diagnostics.begin(), diagnostics.end(),
        [](const ToolpathDiagnostic& diagnostic) {
            return diagnostic.severity == ToolpathDiagnosticSeverity::Error;
        });
}

ToolpathParamNormalization ToolpathParamNormalizer::normalize(
    const ToolpathParams& requested) const
{
    ToolpathParamNormalization result;
    result.effective = requested;
    ToolpathParams& params = result.effective;

    // 刀径决定几何可行性，无法安全回退，因此作为阻断性错误保留。
    if (!std::isfinite(params.tool.diameter) || params.tool.diameter <= 1.0e-7) {
        addDiagnostic(result, ToolpathDiagnosticCode::InvalidToolDiameter,
                      ToolpathDiagnosticSeverity::Error,
                      "tool diameter must be a finite positive value");
    }

    // 进给和主轴允许归零，便于上层决定使用机床默认值或仅做几何预览。
    normalizeNonNegative(params.tool.horizFeed, result,
                         ToolpathDiagnosticCode::FeedRateAdjusted,
                         "horizontal feed");
    normalizeNonNegative(params.tool.vertFeed, result,
                         ToolpathDiagnosticCode::FeedRateAdjusted,
                         "vertical feed");
    normalizeNonNegative(params.tool.horizRapid, result,
                         ToolpathDiagnosticCode::FeedRateAdjusted,
                         "horizontal rapid feed");
    normalizeNonNegative(params.tool.vertRapid, result,
                         ToolpathDiagnosticCode::FeedRateAdjusted,
                         "vertical rapid feed");
    normalizeNonNegative(params.tool.spindleSpeed, result,
                         ToolpathDiagnosticCode::SpindleSpeedAdjusted,
                         "spindle speed");
    normalizeNonNegative(params.tool.cornerRadius, result,
                         ToolpathDiagnosticCode::SamplingParameterAdjusted,
                         "tool corner radius");
    normalizeNonNegative(params.tool.cuttingEdgeLength, result,
                         ToolpathDiagnosticCode::SamplingParameterAdjusted,
                         "cutting-edge length");
    normalizeNonNegative(params.tool.overallLength, result,
                         ToolpathDiagnosticCode::SamplingParameterAdjusted,
                         "tool overall length");
    normalizeNonNegative(params.tool.shankDiameter, result,
                         ToolpathDiagnosticCode::SamplingParameterAdjusted,
                         "tool shank diameter");
    normalizeNonNegative(params.tool.holderDiameter, result,
                         ToolpathDiagnosticCode::SamplingParameterAdjusted,
                         "tool holder diameter");
    normalizeNonNegative(params.tool.holderLength, result,
                         ToolpathDiagnosticCode::SamplingParameterAdjusted,
                         "tool holder length");
    normalizeNonNegative(params.tool.spindleEnvelopeDiameter, result,
                         ToolpathDiagnosticCode::SamplingParameterAdjusted,
                         "spindle envelope diameter");
    normalizeNonNegative(params.tool.spindleEnvelopeLength, result,
                         ToolpathDiagnosticCode::SamplingParameterAdjusted,
                         "spindle envelope length");
    normalizeNonNegative(params.tool.tipCompensation, result,
                         ToolpathDiagnosticCode::DepthParameterAdjusted,
                         "drill-tip direct compensation");

    // 高度关系必须满足 clearance >= safe；深度数值则修正为可预测默认值。
    if (invalidNonNegative(params.depth.safeHeight)) {
        params.depth.safeHeight = 0.0;
        addDiagnostic(result, ToolpathDiagnosticCode::SafeHeightAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "safe height was invalid and was set to zero");
    }
    if (invalidNonNegative(params.depth.clearanceHeight)) {
        params.depth.clearanceHeight = params.depth.safeHeight;
        addDiagnostic(result, ToolpathDiagnosticCode::ClearanceHeightAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "clearance height was invalid and was set to safe height");
    }
    if (params.depth.clearanceHeight < params.depth.safeHeight) {
        params.depth.clearanceHeight = params.depth.safeHeight;
        addDiagnostic(result, ToolpathDiagnosticCode::ClearanceRaisedToSafeHeight,
                      ToolpathDiagnosticSeverity::Warning,
                      "clearance height was raised to safe height");
    }
    if (!std::isfinite(params.depth.startDepth)) {
        params.depth.startDepth = 0.0;
        addDiagnostic(result, ToolpathDiagnosticCode::DepthParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "start depth was invalid and was set to zero");
    }
    if (!std::isfinite(params.depth.finalDepth)) {
        params.depth.finalDepth = 0.0;
        addDiagnostic(result, ToolpathDiagnosticCode::DepthParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "final depth was invalid and was set to zero");
    }
    if (!std::isfinite(params.depth.stepDown) || params.depth.stepDown <= 1.0e-7) {
        params.depth.stepDown = 2.0;
        addDiagnostic(result, ToolpathDiagnosticCode::DepthParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "step-down was invalid and was set to 2 mm");
    }
    if (invalidNonNegative(params.depth.finishDepth)) {
        params.depth.finishDepth = 0.0;
        addDiagnostic(result, ToolpathDiagnosticCode::DepthParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "finish allowance was invalid and was set to zero");
    }

    // 平面清料参数同时受百分比范围和 Clipper 坐标精度限制。
    if (!std::isfinite(params.clearing.stepOverPercent) ||
        params.clearing.stepOverPercent <= 0.0 ||
        params.clearing.stepOverPercent > 100.0) {
        params.clearing.stepOverPercent = 50.0;
        addDiagnostic(result, ToolpathDiagnosticCode::StepOverAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "step-over was outside (0, 100] and was set to 50 percent");
    }
    if (invalidNonNegative(params.clearing.extraOffset)) {
        params.clearing.extraOffset = 0.0;
        addDiagnostic(result, ToolpathDiagnosticCode::OffsetAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "extra offset was invalid and was set to zero");
    }
    if (invalidNonNegative(
            params.clearing.restMachiningPreviousToolDiameter)) {
        params.clearing.restMachiningPreviousToolDiameter = 0.0;
        addDiagnostic(
            result,
            ToolpathDiagnosticCode::OffsetAdjusted,
            ToolpathDiagnosticSeverity::Warning,
            "rest-machining previous tool diameter was reset to zero");
    }
    if (params.clearing.precision < 0 || params.clearing.precision > 8) {
        params.clearing.precision = std::clamp(params.clearing.precision, 0, 8);
        addDiagnostic(result, ToolpathDiagnosticCode::PrecisionAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "clipper precision was clamped to [0, 8]");
    }
    if (invalidNonNegative(params.clearing.arcTolerance)) {
        params.clearing.arcTolerance = 0.0;
        addDiagnostic(result, ToolpathDiagnosticCode::OffsetAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "arc tolerance was invalid and was set to automatic");
    }
    AdaptiveClearingParams& adaptive = params.clearing.adaptive;
    if (!std::isfinite(adaptive.targetEngagement) ||
        adaptive.targetEngagement <= 0.0 ||
        adaptive.targetEngagement > 1.0) {
        adaptive.targetEngagement = 0.10;
        addDiagnostic(result, ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "adaptive target engagement was reset to 0.10");
    }
    if (!std::isfinite(adaptive.targetEngagementAngleDeg) ||
        adaptive.targetEngagementAngleDeg <= 0.0 ||
        adaptive.targetEngagementAngleDeg > 360.0) {
        adaptive.targetEngagementAngleDeg = 70.0;
        addDiagnostic(result, ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "adaptive target engagement angle was reset to 70 degrees");
    }
    if (!std::isfinite(adaptive.maximumEngagementAngleDeg) ||
        adaptive.maximumEngagementAngleDeg <
            adaptive.targetEngagementAngleDeg ||
        adaptive.maximumEngagementAngleDeg > 360.0) {
        adaptive.maximumEngagementAngleDeg = 150.0;
        addDiagnostic(result, ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "adaptive maximum engagement angle was reset to 150 degrees");
    }
    adaptive.engagementAngleSamples = std::clamp(
        adaptive.engagementAngleSamples, 8, 720);
    if (!std::isfinite(adaptive.minimumTurnRadiusFactor) ||
        adaptive.minimumTurnRadiusFactor <= 0.0) {
        adaptive.minimumTurnRadiusFactor = 0.60;
    }
    if (!std::isfinite(adaptive.remainingTolerance) ||
        adaptive.remainingTolerance < 0.0 ||
        adaptive.remainingTolerance >= 1.0) {
        adaptive.remainingTolerance = 0.015;
    }
    if (invalidNonNegative(adaptive.materialCellSize))
        adaptive.materialCellSize = 0.0;
    adaptive.maximumAdaptivePasses = std::clamp(
        adaptive.maximumAdaptivePasses, 1, 500);
    adaptive.lookaheadDepth = std::clamp(adaptive.lookaheadDepth, 1, 4);
    adaptive.beamWidth = std::clamp(adaptive.beamWidth, 1, 16);
    normalizeNonNegative(params.profile.sideAllowance, result,
                         ToolpathDiagnosticCode::OffsetAdjusted,
                         "profile side allowance");
    switch (params.profile.machiningMode) {
        case ProfileMachiningMode::ExternalClearing:
        case ProfileMachiningMode::ContourOnly:
            break;
        default:
            params.profile.machiningMode =
                ProfileMachiningMode::ExternalClearing;
            addDiagnostic(result, ToolpathDiagnosticCode::OffsetAdjusted,
                          ToolpathDiagnosticSeverity::Warning,
                          "profile machining mode was invalid and was set to external clearing");
            break;
    }
    normalizeNonNegative(params.profile.roughingAllowance, result,
                         ToolpathDiagnosticCode::OffsetAdjusted,
                         "profile roughing allowance");
    if (params.profile.roughingAllowance <
        params.profile.sideAllowance) {
        params.profile.roughingAllowance =
            params.profile.sideAllowance;
        addDiagnostic(result, ToolpathDiagnosticCode::OffsetAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "profile roughing allowance was raised to the final side allowance");
    }
    if (params.profile.radialPassCount < 1) {
        params.profile.radialPassCount = 1;
        addDiagnostic(result, ToolpathDiagnosticCode::StepOverAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "profile radial pass count was set to one");
    }
    normalizeNonNegative(params.profile.radialStepOver, result,
                         ToolpathDiagnosticCode::StepOverAdjusted,
                         "profile radial step-over");
    normalizeNonNegative(params.profile.leadInLength, result,
                         ToolpathDiagnosticCode::SamplingParameterAdjusted,
                         "profile lead-in length");
    normalizeNonNegative(params.profile.leadOutLength, result,
                         ToolpathDiagnosticCode::SamplingParameterAdjusted,
                         "profile lead-out length");
    if (params.profile.precision < 0 || params.profile.precision > 8) {
        params.profile.precision = std::clamp(params.profile.precision, 0, 8);
        addDiagnostic(result, ToolpathDiagnosticCode::PrecisionAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "profile precision was clamped to [0, 8]");
    }
    normalizeNonNegative(params.profile.arcTolerance, result,
                         ToolpathDiagnosticCode::OffsetAdjusted,
                         "profile arc tolerance");
    if (!std::isfinite(params.clearing.zigzagAngle)) {
        params.clearing.zigzagAngle = 0.0;
        addDiagnostic(result, ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "zigzag angle was invalid and was set to zero");
    }
    switch (params.clearing.expansionDirection) {
        case PocketClearingDirection::OutsideToInside:
        case PocketClearingDirection::InsideToOutside:
            break;
        default:
            params.clearing.expansionDirection =
                effectivePocketClearingDirection(
                    params.clearing.pattern,
                    PocketClearingDirection::OutsideToInside);
            addDiagnostic(
                result,
                ToolpathDiagnosticCode::SamplingParameterAdjusted,
                ToolpathDiagnosticSeverity::Warning,
                params.clearing.pattern == ClearingPattern::FreePath
                    ? "FreePath clearing direction was invalid and was set "
                      "to the pyocc-exp center-out order"
                    : "pocket clearing direction was invalid and was set "
                      "to outside-to-inside");
            break;
    }
    params.clearing.expansionDirection = effectivePocketClearingDirection(
        params.clearing.pattern, params.clearing.expansionDirection);
    switch (params.clearing.entryMode) {
        case PocketEntryMode::VerticalPlunge:
        case PocketEntryMode::RampOnShape:
        case PocketEntryMode::LinearRamp:
        case PocketEntryMode::HelicalRamp:
        case PocketEntryMode::ZigZagRamp:
            break;
        default:
            params.clearing.entryMode =
                PocketEntryMode::VerticalPlunge;
            addDiagnostic(
                result,
                ToolpathDiagnosticCode::SamplingParameterAdjusted,
                ToolpathDiagnosticSeverity::Warning,
                "pocket entry mode was invalid and was set to vertical plunge");
            break;
    }
    switch (params.clearing.rampStartReference) {
        case PocketRampStartReference::PreviousLevel:
        case PocketRampStartReference::OpeningPlane:
        case PocketRampStartReference::SafePlane:
            break;
        default:
            params.clearing.rampStartReference =
                PocketRampStartReference::PreviousLevel;
            addDiagnostic(
                result,
                ToolpathDiagnosticCode::SamplingParameterAdjusted,
                ToolpathDiagnosticSeverity::Warning,
                "pocket ramp height reference was invalid and was set to previous level");
            break;
    }
    if (!std::isfinite(params.clearing.rampAngleDeg)) {
        params.clearing.rampAngleDeg = 3.0;
        addDiagnostic(result,
                      ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "ramp angle was invalid and was set to 3 degrees");
    }
    const double rampAngle = std::clamp(
        params.clearing.rampAngleDeg, 0.1, 45.0);
    if (rampAngle != params.clearing.rampAngleDeg) {
        params.clearing.rampAngleDeg = rampAngle;
        addDiagnostic(result,
                      ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "ramp angle was clamped to [0.1, 45] degrees");
    }
    if (invalidNonNegative(params.clearing.maximumRampLength)) {
        params.clearing.maximumRampLength = 0.0;
        addDiagnostic(result,
                      ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "maximum ramp length was invalid and was set to automatic");
    }
    if (invalidNonNegative(params.clearing.helixDiameter)) {
        params.clearing.helixDiameter = 0.0;
        addDiagnostic(result,
                      ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "pocket-entry helix diameter was invalid and was set to automatic");
    }
    if (invalidNonNegative(params.clearing.zigzagRampWidth)) {
        params.clearing.zigzagRampWidth = 0.0;
        addDiagnostic(
            result,
            ToolpathDiagnosticCode::SamplingParameterAdjusted,
            ToolpathDiagnosticSeverity::Warning,
            "pocket-entry zigzag width was invalid and was set to automatic");
    }
    if (invalidNonNegative(params.clearing.rampFeed)) {
        params.clearing.rampFeed = 0.0;
        addDiagnostic(
            result,
            ToolpathDiagnosticCode::SamplingParameterAdjusted,
            ToolpathDiagnosticSeverity::Warning,
            "pocket-entry ramp feed was invalid and was set to automatic");
    }

    // 孔工艺的自动值以零表示“由具体策略按几何和刀具推导”。
    if (invalidNonNegative(params.hole.allowance)) {
        params.hole.allowance = 0.0;
        addDiagnostic(result, ToolpathDiagnosticCode::HoleAllowanceAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "hole allowance was invalid and was set to zero");
    }
    if (invalidNonNegative(params.hole.smallHoleDiameterTolerance)) {
        params.hole.smallHoleDiameterTolerance = 0.05;
        addDiagnostic(result,
                      ToolpathDiagnosticCode::HoleDiameterToleranceAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "small-hole diameter tolerance was invalid and was set to 0.05 mm");
    }
    if (params.hole.useCenterSpot &&
        (!std::isfinite(params.hole.centerSpotDepth) ||
         params.hole.centerSpotDepth <= 0.0)) {
        params.hole.centerSpotDepth = 1.0;
        addDiagnostic(result, ToolpathDiagnosticCode::DepthParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "center-spot depth was invalid and was set to 1 mm");
    }
    normalizeNonNegative(params.hole.throughHoleBreakthrough, result,
                         ToolpathDiagnosticCode::DepthParameterAdjusted,
                         "through-hole breakthrough");
    if (!std::isfinite(params.hole.mediumHoleRadiusToolDiameterRatio) ||
        params.hole.mediumHoleRadiusToolDiameterRatio <= 0.0) {
        params.hole.mediumHoleRadiusToolDiameterRatio = 0.95;
        addDiagnostic(result, ToolpathDiagnosticCode::HoleRatioAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "automatic medium-hole radius/tool-diameter ratio was invalid and was set to 0.95");
    }
    params.hole.mediumHoleRadiusToolDiameterRatio = std::min(
        params.hole.mediumHoleRadiusToolDiameterRatio, 1.0);
    if (!std::isfinite(params.hole.helixPitch)) {
        params.hole.helixPitch = 0.0;
        addDiagnostic(result, ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "helix pitch was invalid and was set to automatic");
    }
    if (!std::isfinite(params.hole.radialStep)) {
        params.hole.radialStep = 0.0;
        addDiagnostic(result, ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "radial step was invalid and was set to automatic");
    }
    if (!std::isfinite(params.hole.collisionEnvelopeSampleStep) ||
        params.hole.collisionEnvelopeSampleStep <= 0.0) {
        params.hole.collisionEnvelopeSampleStep = 2.0;
        addDiagnostic(result, ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "tool-envelope sample step was invalid and was set to 2 mm");
    }
    params.hole.collisionEnvelopeSampleStep = std::clamp(
        params.hole.collisionEnvelopeSampleStep, 0.1, 50.0);
    normalizeNonNegative(params.hole.collisionClearance, result,
                         ToolpathDiagnosticCode::DepthParameterAdjusted,
                         "hole collision clearance");
    normalizeNonNegative(params.hole.peckDepth, result,
                         ToolpathDiagnosticCode::DepthParameterAdjusted,
                         "drill peck depth");
    normalizeNonNegative(params.hole.dwellSeconds, result,
                         ToolpathDiagnosticCode::DepthParameterAdjusted,
                         "drill dwell time");
    normalizeNonNegative(params.hole.g81MaxDepthDiameterRatio, result,
                         ToolpathDiagnosticCode::HoleRatioAdjusted,
                         "G81 maximum depth/diameter ratio");
    normalizeNonNegative(params.hole.g73MaxDepthDiameterRatio, result,
                         ToolpathDiagnosticCode::HoleRatioAdjusted,
                         "G73 maximum depth/diameter ratio");
    if (params.hole.g73MaxDepthDiameterRatio <
        params.hole.g81MaxDepthDiameterRatio) {
        params.hole.g73MaxDepthDiameterRatio =
            params.hole.g81MaxDepthDiameterRatio;
        addDiagnostic(result, ToolpathDiagnosticCode::HoleRatioAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "G73 maximum ratio was raised to the G81 threshold");
    }
    normalizeNonNegative(params.hole.g73PeckDepth, result,
                         ToolpathDiagnosticCode::DepthParameterAdjusted,
                         "G73 peck depth");
    normalizeNonNegative(params.hole.g83PeckDepth, result,
                         ToolpathDiagnosticCode::DepthParameterAdjusted,
                         "G83 peck depth");
    normalizeNonNegative(params.hole.g73ChipBreakRetract, result,
                         ToolpathDiagnosticCode::DepthParameterAdjusted,
                         "G73 chip-break retract");
    normalizeNonNegative(params.hole.g83ReentryClearance, result,
                         ToolpathDiagnosticCode::DepthParameterAdjusted,
                         "G83 re-entry clearance");
    if (params.hole.g73PeckDepth <= 1.0e-7)
        params.hole.g73PeckDepth = 7.0;
    if (params.hole.g83PeckDepth <= 1.0e-7)
        params.hole.g83PeckDepth = 20.0;
    if (params.hole.g73ChipBreakRetract <= 1.0e-7)
        params.hole.g73ChipBreakRetract = 0.25;
    if (!std::isfinite(params.hole.drillTipAngleDeg) ||
        params.hole.drillTipAngleDeg < 0.0 ||
        params.hole.drillTipAngleDeg >= 180.0) {
        params.hole.drillTipAngleDeg = 0.0;
        addDiagnostic(result, ToolpathDiagnosticCode::DepthParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "drill tip angle was invalid and compensation was disabled");
    }

    // 采样角和点数预算共同限制螺旋/圆周离散规模，防止精度配置导致路径爆炸。
    if (!std::isfinite(params.hole.minSampleAngleDeg) ||
        params.hole.minSampleAngleDeg <= 0.0) {
        params.hole.minSampleAngleDeg = 3.0;
        addDiagnostic(result, ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "minimum sample angle was invalid and was set to 3 degrees");
    }
    params.hole.minSampleAngleDeg =
        std::clamp(params.hole.minSampleAngleDeg, 0.1, 180.0);
    if (!std::isfinite(params.hole.maxSampleAngleDeg) ||
        params.hole.maxSampleAngleDeg < params.hole.minSampleAngleDeg) {
        params.hole.maxSampleAngleDeg =
            std::max(params.hole.minSampleAngleDeg, 20.0);
        addDiagnostic(result, ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "maximum sample angle was invalid and was normalized");
    }
    params.hole.maxSampleAngleDeg =
        std::clamp(params.hole.maxSampleAngleDeg,
                   params.hole.minSampleAngleDeg, 180.0);
    if (!std::isfinite(params.hole.sampleAngleDeg)) {
        params.hole.sampleAngleDeg = 10.0;
        addDiagnostic(result, ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "sample angle was invalid and was set to 10 degrees");
    }
    const double clampedSample = std::clamp(
        params.hole.sampleAngleDeg,
        params.hole.minSampleAngleDeg,
        params.hole.maxSampleAngleDeg);
    if (clampedSample != params.hole.sampleAngleDeg) {
        params.hole.sampleAngleDeg = clampedSample;
        addDiagnostic(result, ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "sample angle was clamped to the configured range");
    }
    if (!std::isfinite(params.hole.helixChordTolerance) ||
        params.hole.helixChordTolerance < 0.0) {
        addDiagnostic(result, ToolpathDiagnosticCode::SamplingParameterAdjusted,
                      ToolpathDiagnosticSeverity::Error,
                      "helix chord tolerance must be finite and non-negative");
    }
    if (params.hole.maxSamplePoints < 16) {
        params.hole.maxSamplePoints = 16;
        addDiagnostic(result, ToolpathDiagnosticCode::SamplingBudgetAdjusted,
                      ToolpathDiagnosticSeverity::Warning,
                      "sample point budget was raised to 16");
    }

    return result;
}

} // namespace oap::toolpath

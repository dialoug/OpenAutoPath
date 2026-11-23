
// MachiningJobStorage 的实现（oap_manufacturing 持久化层）：MachiningJobDefinition
// 的 JSON 编解码、.oapjob 文件读写、旧 schema 迁移以及定义/执行/来源指纹计算。
// 全部为无状态逻辑，错误通过 std::string* error 返回；不涉及刀路生成与 UI。
#include "support/Files.h"
#include "analysis/ContentHash.h"
#include "support/Json.h"
#include <format>
#include <vector>
#include <string>
#include <set>
#include "manufacturing/job/MachiningJobStorage.h"
#include "manufacturing/job/detail/SurfaceFinishingJson.h"


#include <algorithm>
#include <array>
#include <limits>
#include <utility>


namespace oap::manufacturing {
namespace {

// ---- 以下为定义的 JSON 编解码辅助函数，字段名即文件格式的一部分，不得改名。 ----

constexpr std::int64_t kMaxJobBytes = 32ll * 1024ll * 1024ll;
constexpr auto kFormatId = "oap-machining-job";

std::string fromUtf8(const std::string& value)
{
    return std::string(value.data(), static_cast<std::size_t>(value.size()));
}

std::string toUtf8(const oap::JsonValue& value)
{
    return value.toString();
}

oap::JsonObject vectorToJson(const JobVector3& value)
{
    return {{std::string("x"), value.x},
            {std::string("y"), value.y},
            {std::string("z"), value.z}};
}

JobVector3 vectorFromJson(const oap::JsonValue& value)
{
    const oap::JsonObject object = value.toObject();
    return {object.value(std::string("x")).toDouble(),
            object.value(std::string("y")).toDouble(),
            object.value(std::string("z")).toDouble()};
}

oap::JsonArray stringsToJson(const std::vector<std::string>& values)
{
    oap::JsonArray result;
    for (const std::string& value : values)
        result.append(fromUtf8(value));
    return result;
}

std::vector<std::string> stringsFromJson(const oap::JsonValue& value)
{
    std::vector<std::string> result;
    const oap::JsonArray array = value.toArray();
    result.reserve(static_cast<std::size_t>(array.size()));
    for (const oap::JsonValue& item : array)
        result.push_back(toUtf8(item));
    return result;
}

oap::JsonObject toolDefToJson(const toolpath::ToolDef& value)
{
    return {
        {std::string("diameter"), value.diameter},
        {std::string("corner_radius"), value.cornerRadius},
        {std::string("horizontal_feed"), value.horizFeed},
        {std::string("vertical_feed"), value.vertFeed},
        {std::string("horizontal_rapid"), value.horizRapid},
        {std::string("vertical_rapid"), value.vertRapid},
        {std::string("spindle_speed"), value.spindleSpeed},
        {std::string("cutting_edge_length"), value.cuttingEdgeLength},
        {std::string("overall_length"), value.overallLength},
        {std::string("shank_diameter"), value.shankDiameter},
        {std::string("holder_diameter"), value.holderDiameter},
        {std::string("holder_length"), value.holderLength},
        {std::string("spindle_envelope_diameter"),
         value.spindleEnvelopeDiameter},
        {std::string("spindle_envelope_length"),
         value.spindleEnvelopeLength},
        {std::string("tip_compensation"), value.tipCompensation},
    };
}

toolpath::ToolDef toolDefFromJson(const oap::JsonValue& value)
{
    const oap::JsonObject object = value.toObject();
    toolpath::ToolDef result;
    result.diameter = object.value(std::string("diameter")).toDouble(result.diameter);
    result.cornerRadius = object.value(std::string("corner_radius")).toDouble();
    result.horizFeed = object.value(std::string("horizontal_feed")).toDouble(result.horizFeed);
    result.vertFeed = object.value(std::string("vertical_feed")).toDouble(result.vertFeed);
    result.horizRapid = object.value(std::string("horizontal_rapid")).toDouble(result.horizRapid);
    result.vertRapid = object.value(std::string("vertical_rapid")).toDouble(result.vertRapid);
    result.spindleSpeed = object.value(std::string("spindle_speed")).toDouble(result.spindleSpeed);
    result.cuttingEdgeLength = object.value(std::string("cutting_edge_length")).toDouble();
    result.overallLength = object.value(std::string("overall_length")).toDouble();
    result.shankDiameter = object.value(std::string("shank_diameter")).toDouble();
    result.holderDiameter = object.value(std::string("holder_diameter")).toDouble();
    result.holderLength = object.value(std::string("holder_length")).toDouble();
    result.spindleEnvelopeDiameter = object.value(
        std::string("spindle_envelope_diameter")).toDouble();
    result.spindleEnvelopeLength = object.value(
        std::string("spindle_envelope_length")).toDouble();
    result.tipCompensation = object.value(std::string("tip_compensation")).toDouble();
    return result;
}

oap::JsonObject depthToJson(const toolpath::DepthParams& value)
{
    return {
        {std::string("clearance_height"), value.clearanceHeight},
        {std::string("safe_height"), value.safeHeight},
        {std::string("use_feature_depth"), value.useFeatureDepth},
        {std::string("start_depth"), value.startDepth},
        {std::string("final_depth"), value.finalDepth},
        {std::string("step_down"), value.stepDown},
        {std::string("finish_depth"), value.finishDepth},
    };
}

toolpath::DepthParams depthFromJson(const oap::JsonValue& value)
{
    const oap::JsonObject object = value.toObject();
    toolpath::DepthParams result;
    result.clearanceHeight = object.value(std::string("clearance_height")).toDouble(result.clearanceHeight);
    result.safeHeight = object.value(std::string("safe_height")).toDouble(result.safeHeight);
    result.useFeatureDepth = object.value(std::string("use_feature_depth")).toBool(result.useFeatureDepth);
    result.startDepth = object.value(std::string("start_depth")).toDouble(result.startDepth);
    result.finalDepth = object.value(std::string("final_depth")).toDouble(result.finalDepth);
    result.stepDown = object.value(std::string("step_down")).toDouble(result.stepDown);
    result.finishDepth = object.value(std::string("finish_depth")).toDouble(result.finishDepth);
    return result;
}

oap::JsonObject clearingToJson(const toolpath::ClearingParams& value)
{
    const toolpath::AdaptiveClearingParams& adaptive = value.adaptive;
    return {
        {std::string("pattern"), static_cast<int>(value.pattern)},
        {std::string("expansion_direction"),
         static_cast<int>(toolpath::effectivePocketClearingDirection(
             value.pattern, value.expansionDirection))},
        {std::string("cut_mode"), static_cast<int>(value.cutMode)},
        {std::string("entry_mode"), static_cast<int>(value.entryMode)},
        {std::string("ramp_start_reference"),
         static_cast<int>(value.rampStartReference)},
        {std::string("ramp_angle_deg"), value.rampAngleDeg},
        {std::string("maximum_ramp_length"), value.maximumRampLength},
        {std::string("helix_diameter"), value.helixDiameter},
        {std::string("zigzag_ramp_width"), value.zigzagRampWidth},
        {std::string("ramp_feed"), value.rampFeed},
        {std::string("step_over_percent"), value.stepOverPercent},
        {std::string("extra_offset"), value.extraOffset},
        {std::string("precision"), value.precision},
        {std::string("arc_tolerance"), value.arcTolerance},
        {std::string("zigzag_angle"), value.zigzagAngle},
        {std::string("perimeter_pass"), value.perimeterPass},
        {std::string("adaptive"), oap::JsonObject{
             {std::string("target_engagement"), adaptive.targetEngagement},
             {std::string("target_angle_deg"), adaptive.targetEngagementAngleDeg},
             {std::string("maximum_angle_deg"), adaptive.maximumEngagementAngleDeg},
             {std::string("angle_samples"), adaptive.engagementAngleSamples},
             {std::string("minimum_turn_radius_factor"), adaptive.minimumTurnRadiusFactor},
             {std::string("remaining_tolerance"), adaptive.remainingTolerance},
             {std::string("material_cell_size"), adaptive.materialCellSize},
             {std::string("maximum_passes"), adaptive.maximumAdaptivePasses},
             {std::string("lookahead_depth"), adaptive.lookaheadDepth},
             {std::string("beam_width"), adaptive.beamWidth}}},
    };
}

toolpath::ClearingParams clearingFromJson(const oap::JsonValue& value)
{
    const oap::JsonObject object = value.toObject();
    toolpath::ClearingParams result;
    result.pattern = static_cast<toolpath::ClearingPattern>(
        object.value(std::string("pattern")).toInt(static_cast<int>(result.pattern)));
    result.expansionDirection =
        static_cast<toolpath::PocketClearingDirection>(
            object.value(std::string("expansion_direction")).toInt(
                static_cast<int>(result.expansionDirection)));
    result.expansionDirection =
        toolpath::effectivePocketClearingDirection(
            result.pattern, result.expansionDirection);
    result.cutMode = static_cast<toolpath::CutMode>(
        object.value(std::string("cut_mode")).toInt(static_cast<int>(result.cutMode)));
    result.entryMode = static_cast<toolpath::PocketEntryMode>(
        object.value(std::string("entry_mode")).toInt(
            static_cast<int>(result.entryMode)));
    result.rampStartReference =
        static_cast<toolpath::PocketRampStartReference>(
            object.value(std::string("ramp_start_reference")).toInt(
                static_cast<int>(result.rampStartReference)));
    result.rampAngleDeg =
        object.value(std::string("ramp_angle_deg")).toDouble(
            result.rampAngleDeg);
    result.maximumRampLength =
        object.value(std::string("maximum_ramp_length")).toDouble(
            result.maximumRampLength);
    result.helixDiameter =
        object.value(std::string("helix_diameter")).toDouble(
            result.helixDiameter);
    result.zigzagRampWidth =
        object.value(std::string("zigzag_ramp_width")).toDouble(
            result.zigzagRampWidth);
    result.rampFeed =
        object.value(std::string("ramp_feed")).toDouble(
            result.rampFeed);
    result.stepOverPercent = object.value(std::string("step_over_percent")).toDouble(result.stepOverPercent);
    result.extraOffset = object.value(std::string("extra_offset")).toDouble(result.extraOffset);
    result.precision = object.value(std::string("precision")).toInt(result.precision);
    result.arcTolerance = object.value(std::string("arc_tolerance")).toDouble(result.arcTolerance);
    result.zigzagAngle = object.value(std::string("zigzag_angle")).toDouble(result.zigzagAngle);
    result.perimeterPass = object.value(std::string("perimeter_pass")).toBool(result.perimeterPass);
    const oap::JsonObject adaptive =
        object.value(std::string("adaptive")).toObject();
    result.adaptive.targetEngagement = adaptive.value(
        std::string("target_engagement")).toDouble(
            result.adaptive.targetEngagement);
    result.adaptive.targetEngagementAngleDeg = adaptive.value(
        std::string("target_angle_deg")).toDouble(
            result.adaptive.targetEngagementAngleDeg);
    result.adaptive.maximumEngagementAngleDeg = adaptive.value(
        std::string("maximum_angle_deg")).toDouble(
            result.adaptive.maximumEngagementAngleDeg);
    result.adaptive.engagementAngleSamples = adaptive.value(
        std::string("angle_samples")).toInt(
            result.adaptive.engagementAngleSamples);
    result.adaptive.minimumTurnRadiusFactor = adaptive.value(
        std::string("minimum_turn_radius_factor")).toDouble(
            result.adaptive.minimumTurnRadiusFactor);
    result.adaptive.remainingTolerance = adaptive.value(
        std::string("remaining_tolerance")).toDouble(
            result.adaptive.remainingTolerance);
    result.adaptive.materialCellSize = adaptive.value(
        std::string("material_cell_size")).toDouble(
            result.adaptive.materialCellSize);
    result.adaptive.maximumAdaptivePasses = adaptive.value(
        std::string("maximum_passes")).toInt(
            result.adaptive.maximumAdaptivePasses);
    result.adaptive.lookaheadDepth = adaptive.value(
        std::string("lookahead_depth")).toInt(
            result.adaptive.lookaheadDepth);
    result.adaptive.beamWidth = adaptive.value(
        std::string("beam_width")).toInt(
            result.adaptive.beamWidth);
    return result;
}

oap::JsonObject profileToJson(const toolpath::ProfileParams& value)
{
    return {
        {std::string("machining_mode"), static_cast<int>(value.machiningMode)},
        {std::string("cut_mode"), static_cast<int>(value.cutMode)},
        {std::string("side_allowance"), value.sideAllowance},
        {std::string("roughing_allowance"), value.roughingAllowance},
        {std::string("finish_pass"), value.finishPass},
        {std::string("radial_pass_count"), value.radialPassCount},
        {std::string("radial_step_over"), value.radialStepOver},
        {std::string("lead_in_length"), value.leadInLength},
        {std::string("lead_out_length"), value.leadOutLength},
        {std::string("precision"), value.precision},
        {std::string("arc_tolerance"), value.arcTolerance},
    };
}

toolpath::ProfileParams profileFromJson(const oap::JsonValue& value)
{
    const oap::JsonObject object = value.toObject();
    toolpath::ProfileParams result;
    result.machiningMode = static_cast<toolpath::ProfileMachiningMode>(
        object.value(std::string("machining_mode")).toInt(
            static_cast<int>(result.machiningMode)));
    result.cutMode = static_cast<toolpath::CutMode>(
        object.value(std::string("cut_mode")).toInt(
            static_cast<int>(result.cutMode)));
    result.sideAllowance = object.value(
        std::string("side_allowance")).toDouble(result.sideAllowance);
    result.roughingAllowance = object.value(
        std::string("roughing_allowance")).toDouble(
            result.roughingAllowance);
    result.finishPass = object.value(
        std::string("finish_pass")).toBool(result.finishPass);
    result.radialPassCount = object.value(
        std::string("radial_pass_count")).toInt(result.radialPassCount);
    result.radialStepOver = object.value(
        std::string("radial_step_over")).toDouble(result.radialStepOver);
    result.leadInLength = object.value(
        std::string("lead_in_length")).toDouble(result.leadInLength);
    result.leadOutLength = object.value(
        std::string("lead_out_length")).toDouble(result.leadOutLength);
    result.precision = object.value(
        std::string("precision")).toInt(result.precision);
    result.arcTolerance = object.value(
        std::string("arc_tolerance")).toDouble(result.arcTolerance);
    return result;
}

oap::JsonObject holeParamsToJson(const toolpath::HoleMachiningParams& value)
{
    return {
        {std::string("start_corner"), static_cast<int>(value.startCorner)},
        {std::string("allowance"), value.allowance},
        {std::string("small_hole_diameter_tolerance"), value.smallHoleDiameterTolerance},
        {std::string("center_spot_depth"), value.centerSpotDepth},
        {std::string("through_hole_breakthrough"), value.throughHoleBreakthrough},
        {std::string("medium_hole_ratio"), value.mediumHoleRadiusToolDiameterRatio},
        {std::string("drill_cycle_mode"), static_cast<int>(value.drillCycleMode)},
        {std::string("drill_retract_mode"), static_cast<int>(value.drillRetractMode)},
        {std::string("peck_depth"), value.peckDepth},
        {std::string("dwell_seconds"), value.dwellSeconds},
        {std::string("drill_tip_angle_deg"), value.drillTipAngleDeg},
        {std::string("g81_max_ratio"), value.g81MaxDepthDiameterRatio},
        {std::string("g73_max_ratio"), value.g73MaxDepthDiameterRatio},
        {std::string("g73_peck_depth"), value.g73PeckDepth},
        {std::string("g83_peck_depth"), value.g83PeckDepth},
        {std::string("g73_chip_break_retract"), value.g73ChipBreakRetract},
        {std::string("g83_reentry_clearance"), value.g83ReentryClearance},
        {std::string("collision_envelope_mode"), static_cast<int>(value.collisionEnvelopeMode)},
        {std::string("collision_sample_step"), value.collisionEnvelopeSampleStep},
        {std::string("helix_pitch"), value.helixPitch},
        {std::string("radial_step"), value.radialStep},
        {std::string("sample_angle_deg"), value.sampleAngleDeg},
        {std::string("helix_chord_tolerance"), value.helixChordTolerance},
        {std::string("min_sample_angle_deg"), value.minSampleAngleDeg},
        {std::string("max_sample_angle_deg"), value.maxSampleAngleDeg},
        {std::string("max_sample_points"), value.maxSamplePoints},
    };
}

toolpath::HoleMachiningParams holeParamsFromJson(const oap::JsonValue& value)
{
    const oap::JsonObject object = value.toObject();
    toolpath::HoleMachiningParams result;
    result.startCorner = static_cast<toolpath::HoleStartCorner>(
        object.value(std::string("start_corner")).toInt(static_cast<int>(result.startCorner)));
    result.allowance = object.value(std::string("allowance")).toDouble(result.allowance);
    result.smallHoleDiameterTolerance = object.value(std::string("small_hole_diameter_tolerance")).toDouble(result.smallHoleDiameterTolerance);
    result.centerSpotDepth = object.value(std::string("center_spot_depth")).toDouble(result.centerSpotDepth);
    result.throughHoleBreakthrough = object.value(std::string("through_hole_breakthrough")).toDouble(result.throughHoleBreakthrough);
    result.mediumHoleRadiusToolDiameterRatio = object.value(std::string("medium_hole_ratio")).toDouble(result.mediumHoleRadiusToolDiameterRatio);
    result.drillCycleMode = static_cast<toolpath::HoleDrillCycleMode>(
        object.value(std::string("drill_cycle_mode")).toInt(static_cast<int>(result.drillCycleMode)));
    result.drillRetractMode = static_cast<toolpath::HoleDrillRetractMode>(
        object.value(std::string("drill_retract_mode")).toInt(static_cast<int>(result.drillRetractMode)));
    result.peckDepth = object.value(std::string("peck_depth")).toDouble(result.peckDepth);
    result.dwellSeconds = object.value(std::string("dwell_seconds")).toDouble(result.dwellSeconds);
    result.drillTipAngleDeg = object.value(std::string("drill_tip_angle_deg")).toDouble(result.drillTipAngleDeg);
    result.g81MaxDepthDiameterRatio = object.value(std::string("g81_max_ratio")).toDouble(result.g81MaxDepthDiameterRatio);
    result.g73MaxDepthDiameterRatio = object.value(std::string("g73_max_ratio")).toDouble(result.g73MaxDepthDiameterRatio);
    result.g73PeckDepth = object.value(std::string("g73_peck_depth")).toDouble(result.g73PeckDepth);
    result.g83PeckDepth = object.value(std::string("g83_peck_depth")).toDouble(result.g83PeckDepth);
    result.g73ChipBreakRetract = object.value(std::string("g73_chip_break_retract")).toDouble(result.g73ChipBreakRetract);
    result.g83ReentryClearance = object.value(std::string("g83_reentry_clearance")).toDouble(result.g83ReentryClearance);
    result.collisionEnvelopeMode = static_cast<toolpath::HoleCollisionEnvelopeMode>(
        object.value(std::string("collision_envelope_mode")).toInt(static_cast<int>(result.collisionEnvelopeMode)));
    result.collisionEnvelopeSampleStep = object.value(std::string("collision_sample_step")).toDouble(result.collisionEnvelopeSampleStep);
    result.helixPitch = object.value(std::string("helix_pitch")).toDouble(result.helixPitch);
    result.radialStep = object.value(std::string("radial_step")).toDouble(result.radialStep);
    result.sampleAngleDeg = object.value(std::string("sample_angle_deg")).toDouble(result.sampleAngleDeg);
    result.helixChordTolerance = object.value(std::string("helix_chord_tolerance")).toDouble(result.helixChordTolerance);
    result.minSampleAngleDeg = object.value(std::string("min_sample_angle_deg")).toDouble(result.minSampleAngleDeg);
    result.maxSampleAngleDeg = object.value(std::string("max_sample_angle_deg")).toDouble(result.maxSampleAngleDeg);
    result.maxSamplePoints = object.value(std::string("max_sample_points")).toInt(result.maxSamplePoints);
    return result;
}

oap::JsonObject parametersToJson(const toolpath::ToolpathParams& value)
{
    return {{std::string("tool"), toolDefToJson(value.tool)},
            {std::string("depth"), depthToJson(value.depth)},
            {std::string("clearing"), clearingToJson(value.clearing)},
            {std::string("profile"), profileToJson(value.profile)},
            {std::string("hole"), holeParamsToJson(value.hole)}};
}

toolpath::ToolpathParams parametersFromJson(const oap::JsonValue& value)
{
    const oap::JsonObject object = value.toObject();
    toolpath::ToolpathParams result;
    result.tool = toolDefFromJson(object.value(std::string("tool")));
    result.depth = depthFromJson(object.value(std::string("depth")));
    result.clearing = clearingFromJson(object.value(std::string("clearing")));
    result.profile = profileFromJson(object.value(std::string("profile")));
    result.hole = holeParamsFromJson(object.value(std::string("hole")));
    return result;
}

oap::JsonObject toolContextToJson(const toolpath::ToolpathToolContext& value)
{
    return {
        {std::string("valid"), value.valid},
        {std::string("tool_number"), value.toolNumber},
        {std::string("tool_id"), fromUtf8(value.toolId)},
        {std::string("tool_name"), fromUtf8(value.toolName)},
        {std::string("tool_type"), fromUtf8(value.toolType)},
        {std::string("diameter"), value.diameter},
        {std::string("corner_radius"), value.cornerRadius},
        {std::string("spindle_speed"), value.spindleSpeed},
        {std::string("horizontal_feed"), value.horizontalFeed},
        {std::string("vertical_feed"), value.verticalFeed},
        {std::string("horizontal_rapid"), value.horizontalRapid},
        {std::string("vertical_rapid"), value.verticalRapid},
        {std::string("cutting_edge_length"), value.cuttingEdgeLength},
        {std::string("overall_length"), value.overallLength},
        {std::string("shank_diameter"), value.shankDiameter},
        {std::string("holder_diameter"), value.holderDiameter},
        {std::string("holder_length"), value.holderLength},
        {std::string("spindle_envelope_diameter"),
         value.spindleEnvelopeDiameter},
        {std::string("spindle_envelope_length"),
         value.spindleEnvelopeLength},
        {std::string("tip_angle"), value.tipAngle},
        {std::string("tip_compensation"), value.tipCompensation},
        {std::string("peck_depth"), value.peckDepth},
        {std::string("drill_cycle"), static_cast<int>(value.drillCycle)},
        {std::string("coolant"), static_cast<int>(value.coolant)},
    };
}

toolpath::ToolpathToolContext toolContextFromJson(const oap::JsonValue& value)
{
    const oap::JsonObject object = value.toObject();
    toolpath::ToolpathToolContext result;
    result.valid = object.value(std::string("valid")).toBool();
    result.toolNumber = object.value(std::string("tool_number")).toInt();
    result.toolId = toUtf8(object.value(std::string("tool_id")));
    result.toolName = toUtf8(object.value(std::string("tool_name")));
    result.toolType = toUtf8(object.value(std::string("tool_type")));
    result.diameter = object.value(std::string("diameter")).toDouble();
    result.cornerRadius = object.value(std::string("corner_radius")).toDouble();
    result.spindleSpeed = object.value(std::string("spindle_speed")).toDouble();
    result.horizontalFeed = object.value(std::string("horizontal_feed")).toDouble();
    result.verticalFeed = object.value(std::string("vertical_feed")).toDouble();
    result.horizontalRapid = object.value(std::string("horizontal_rapid")).toDouble();
    result.verticalRapid = object.value(std::string("vertical_rapid")).toDouble();
    result.cuttingEdgeLength = object.value(std::string("cutting_edge_length")).toDouble();
    result.overallLength = object.value(std::string("overall_length")).toDouble();
    result.shankDiameter = object.value(std::string("shank_diameter")).toDouble();
    result.holderDiameter = object.value(std::string("holder_diameter")).toDouble();
    result.holderLength = object.value(std::string("holder_length")).toDouble();
    result.spindleEnvelopeDiameter = object.value(
        std::string("spindle_envelope_diameter")).toDouble();
    result.spindleEnvelopeLength = object.value(
        std::string("spindle_envelope_length")).toDouble();
    result.tipAngle = object.value(std::string("tip_angle")).toDouble();
    result.tipCompensation = object.value(std::string("tip_compensation")).toDouble();
    result.peckDepth = object.value(std::string("peck_depth")).toDouble();
    result.drillCycle = static_cast<toolpath::ToolpathDrillCyclePreference>(
        object.value(std::string("drill_cycle")).toInt());
    result.coolant = static_cast<toolpath::ToolpathCoolantMode>(
        object.value(std::string("coolant")).toInt());
    return result;
}

oap::JsonObject holeToJson(const toolpath::HoleMachiningInput& value)
{
    return {
        {std::string("kind"), static_cast<int>(value.kind)},
        {std::string("entry"), vectorToJson(toJobVector(value.entry))},
        {std::string("bottom"), vectorToJson(toJobVector(value.bottom))},
        {std::string("radius"), value.radius},
        {std::string("bottom_profile"), static_cast<int>(value.bottomProfile)},
        {std::string("entry_resolution"), static_cast<int>(value.entryResolution)},
        {std::string("setup_face"), static_cast<int>(value.setupFace)},
        {std::string("setup_face_entry_distance"), value.setupFaceEntryDistance},
        {std::string("setup_face_other_distance"), value.setupFaceOtherEndpointDistance},
        {std::string("setup_face_decision"), static_cast<int>(value.setupFaceDecision)},
        {std::string("preserve_entry_side"), value.preserveEntrySide},
        {std::string("manual"), value.manuallyConfirmed},
        {std::string("source_face_ids"), stringsToJson(value.sourceFaceIds)},
        {std::string("stage_wall_face_ids"),
         stringsToJson(value.stageWallFaceIds)},
        {std::string("stage_role"), static_cast<int>(value.stageRole)},
        {std::string("parent_relation"), static_cast<int>(value.parentRelation)},
        {std::string("parent_counterbore_face_id"), fromUtf8(value.parentCounterboreFaceId)},
        {std::string("parent_counterbore_floor_face_id"), fromUtf8(value.parentCounterboreFloorFaceId)},
        {std::string("parent_counterbore_radius"), value.parentCounterboreRadius},
    };
}

toolpath::HoleMachiningInput holeFromJson(const oap::JsonValue& value)
{
    const oap::JsonObject object = value.toObject();
    toolpath::HoleMachiningInput result;
    result.kind = static_cast<FeatureKind>(object.value(std::string("kind")).toInt());
    result.entry = toPoint(vectorFromJson(object.value(std::string("entry"))));
    result.bottom = toPoint(vectorFromJson(object.value(std::string("bottom"))));
    result.radius = object.value(std::string("radius")).toDouble();
    result.bottomProfile = static_cast<HoleBottomProfile>(
        object.value(std::string("bottom_profile")).toInt());
    result.entryResolution = static_cast<HoleEntryResolution>(
        object.value(std::string("entry_resolution")).toInt());
    result.setupFace = static_cast<MoldSetupFace>(
        object.value(std::string("setup_face")).toInt());
    result.setupFaceEntryDistance = object.value(std::string("setup_face_entry_distance")).toDouble(-1.0);
    result.setupFaceOtherEndpointDistance = object.value(std::string("setup_face_other_distance")).toDouble(-1.0);
    result.setupFaceDecision = static_cast<MoldSetupFaceDecision>(
        object.value(std::string("setup_face_decision")).toInt());
    result.preserveEntrySide = object.value(std::string("preserve_entry_side")).toBool();
    result.manuallyConfirmed = object.value(std::string("manual")).toBool();
    result.sourceFaceIds = stringsFromJson(object.value(std::string("source_face_ids")));
    result.stageWallFaceIds = stringsFromJson(
        object.value(std::string("stage_wall_face_ids")));
    result.stageRole = static_cast<HoleStageRole>(
        object.value(std::string("stage_role")).toInt(
            static_cast<int>(HoleStageRole::Standard)));
    result.parentRelation = static_cast<HoleParentRelation>(
        object.value(std::string("parent_relation")).toInt());
    result.parentCounterboreFaceId = toUtf8(object.value(std::string("parent_counterbore_face_id")));
    result.parentCounterboreFloorFaceId = toUtf8(object.value(std::string("parent_counterbore_floor_face_id")));
    result.parentCounterboreRadius = object.value(std::string("parent_counterbore_radius")).toDouble();
    if (result.entryResolution == HoleEntryResolution::Unresolved) {
        result.mappingState =
            toolpath::HoleMachiningMappingState::UnresolvedEntry;
    } else if (result.setupFace == MoldSetupFace::Unresolved) {
        result.mappingState =
            toolpath::HoleMachiningMappingState::UnresolvedSetup;
    } else {
        result.mappingState = toolpath::HoleMachiningMappingState::Ready;
    }
    return result;
}

oap::JsonObject definitionToJson(const MachiningJobDefinition& value)
{
    // 把可序列化定义整体投影为 JSON；字段名构成文件格式的稳定契约。
    oap::JsonArray recognizedHoles;
    for (const toolpath::HoleMachiningInput& hole : value.recognizedHoles)
        recognizedHoles.append(holeToJson(hole));
    oap::JsonArray collisionModels;
    for (const CollisionModelReference& model : value.collisionScene.models) {
        collisionModels.append(oap::JsonObject{
            {std::string("source_path"), fromUtf8(model.sourcePath)},
            {std::string("source_fingerprint"),
             fromUtf8(model.sourceFingerprint)},
            {std::string("role"), static_cast<int>(model.role)},
            {std::string("translation"),
             vectorToJson(model.translation)},
            {std::string("rotation_deg"),
             vectorToJson(model.rotationDeg)},
        });
    }

    oap::JsonArray setups;
    for (const SetupDefinition& setup : value.setups) {
        setups.append(oap::JsonObject{
            {std::string("id"), fromUtf8(setup.id)},
            {std::string("setup_index"), setup.setupIndex},
            {std::string("face"), static_cast<int>(setup.face)},
            {std::string("work_offset"), static_cast<int>(setup.workOffset)},
            {std::string("origin"), vectorToJson(setup.origin)},
            {std::string("x_axis"), vectorToJson(setup.xAxis)},
            {std::string("y_axis"), vectorToJson(setup.yAxis)},

            {std::string("z_axis"), vectorToJson(setup.zAxis)},
            {std::string("valid"), setup.valid},
            {std::string("requires_physical_reclamp"), setup.requiresPhysicalReclamp},
            {std::string("axis_mapping"), fromUtf8(setup.axisMapping)},
            {std::string("operator_instruction"), fromUtf8(setup.operatorInstruction)},
            {std::string("confirmation"), oap::JsonObject{
                {std::string("confirmed"), setup.confirmation.confirmed},
                {std::string("confirmed_by"), fromUtf8(setup.confirmation.confirmedBy)},
                {std::string("confirmed_at"), fromUtf8(setup.confirmation.confirmedAt)},
                {std::string("input_fingerprint"), fromUtf8(setup.confirmation.confirmedInputFingerprint)},
            }},
        });
    }

    oap::JsonArray controllers;
    for (const JobToolController& controller : value.toolControllers) {
        controllers.append(oap::JsonObject{
            {std::string("id"), fromUtf8(controller.id)},
            {std::string("library_id"), fromUtf8(controller.libraryId)},
            {std::string("tool"), toolContextToJson(controller.tool)},
        });
    }

    oap::JsonArray holeOperations;
    for (const HoleOperationDefinition& operation : value.holeOperations) {
        oap::JsonArray targets;
        for (const HoleTargetReference& target : operation.targets) {
            oap::JsonArray coveredSourceIndices;
            for (const int sourceIndex : target.coveredSourceHoleIndices)
                coveredSourceIndices.append(sourceIndex);
            targets.append(oap::JsonObject{
                {std::string("feature_id"), fromUtf8(target.featureId)},
                {std::string("geometry_fingerprint"), fromUtf8(target.geometryFingerprint)},
                {std::string("source_hole_index"), target.sourceHoleIndex},
                {std::string("covered_source_hole_indices"),
                 coveredSourceIndices},
                {std::string("physical_hole_index"), target.physicalHoleIndex},
                {std::string("strategy"), static_cast<int>(target.strategy)},
                {std::string("hole"), holeToJson(target.hole)},
            });
        }
        holeOperations.append(oap::JsonObject{
            {std::string("id"), fromUtf8(operation.id)},
            {std::string("setup_id"), fromUtf8(operation.setupId)},
            {std::string("tool_controller_id"), fromUtf8(operation.toolControllerId)},
            {std::string("phase"), static_cast<int>(operation.phase)},
            {std::string("targets"), targets},
            {std::string("enabled"), operation.enabled},
            {std::string("sequence"), operation.sequence},
            {std::string("state"), static_cast<int>(operation.state)},
            {std::string("input_fingerprint"), fromUtf8(operation.inputFingerprint)},
        });
    }

    oap::JsonArray pocketOperations;
    for (const PocketOperationDefinition& operation :
         value.pocketOperations) {
        oap::JsonObject entry{
            {std::string("id"), fromUtf8(operation.id)},
            {std::string("setup_id"), fromUtf8(operation.setupId)},
            {std::string("tool_controller_id"),
             fromUtf8(operation.toolControllerId)},
            {std::string("rest_previous_tool_diameter"),
             operation.restMachiningPreviousToolDiameter},
            {std::string("algorithm_id"),
             fromUtf8(operation.algorithmId)},
            {std::string("target"), oap::JsonObject{
                {std::string("source_kind"),
                 static_cast<int>(operation.target.sourceKind)},
                {std::string("kind"),
                 static_cast<int>(operation.target.kind)},
                {std::string("source_face_ids"),
                 stringsToJson(operation.target.sourceFaceIds)},
                {std::string("floor_face_id"),
                 fromUtf8(operation.target.floorFaceId)},
                {std::string("root_face_id"),
                 fromUtf8(operation.target.rootFaceId)},
                {std::string("root_wire_id"),
                 fromUtf8(operation.target.rootWireId)},
                {std::string("allow_open_section_contours"),
                 operation.target.allowOpenSectionContours},
                {std::string("open_boundary_face_count"),
                 operation.target.openBoundaryFaceCount},
                {std::string("geometry_fingerprint"),
                 fromUtf8(operation.target.geometryFingerprint)},
            }},
            {std::string("enabled"), operation.enabled},
            {std::string("sequence"), operation.sequence},
            {std::string("state"), static_cast<int>(operation.state)},
            {std::string("input_fingerprint"),
             fromUtf8(operation.inputFingerprint)},
        };
        if (operation.parameters)
            entry.insert(std::string("parameters"), parametersToJson(*operation.parameters));
        pocketOperations.append(entry);
    }

    oap::JsonArray profileOperations;
    for (const ProfileOperationDefinition& operation :
         value.profileOperations) {
        profileOperations.append(oap::JsonObject{
            {std::string("id"), fromUtf8(operation.id)},
            {std::string("setup_id"), fromUtf8(operation.setupId)},
            {std::string("tool_controller_id"),
             fromUtf8(operation.toolControllerId)},
            {std::string("algorithm_id"),
             fromUtf8(operation.algorithmId)},
            {std::string("target"), oap::JsonObject{
                {std::string("axis_label"),
                 fromUtf8(operation.target.axisLabel)},
                {std::string("normal_direction"),
                 operation.target.normalDirection},
                {std::string("source_face_ids"),
                 stringsToJson(operation.target.sourceFaceIds)},
                {std::string("machining_face_ids"),
                 stringsToJson(operation.target.machiningFaceIds)},
                {std::string("terminal_face_ids"),
                 stringsToJson(operation.target.terminalFaceIds)},
                {std::string("main_outer_face_ids"),
                 stringsToJson(operation.target.mainOuterFaceIds)},
                {std::string("root_face_id"),
                 fromUtf8(operation.target.rootFaceId)},
                {std::string("root_wire_id"),
                 fromUtf8(operation.target.rootWireId)},
                {std::string("geometry_fingerprint"),
                 fromUtf8(operation.target.geometryFingerprint)},
            }},
            {std::string("rest_previous_tool_diameter"),
             operation.restMachiningPreviousToolDiameter},
            {std::string("finish_pass"), operation.finishPass},
            {std::string("enabled"), operation.enabled},
            {std::string("sequence"), operation.sequence},
            {std::string("state"), static_cast<int>(operation.state)},
            {std::string("input_fingerprint"),
             fromUtf8(operation.inputFingerprint)},
        });
    }

    return {
        {std::string("schema_version"), kMachiningJobSchemaVersion},
        {std::string("id"), fromUtf8(value.id)},
        {std::string("name"), fromUtf8(value.name)},
        {std::string("created_at"), fromUtf8(value.createdAt)},
        {std::string("modified_at"), fromUtf8(value.modifiedAt)},
        {std::string("part"), oap::JsonObject{
            {std::string("source_path"), fromUtf8(value.part.sourcePath)},
            {std::string("source_fingerprint"), fromUtf8(value.part.sourceFingerprint)},
            {std::string("units"), fromUtf8(value.part.units)},
            {std::string("recognition_profile_version"), fromUtf8(value.part.recognitionProfileVersion)},
        }},
        {std::string("stock"), oap::JsonObject{
            {std::string("source_kind"), static_cast<int>(value.stock.sourceKind)},
            {std::string("source_path"), fromUtf8(value.stock.sourcePath)},
            {std::string("source_fingerprint"), fromUtf8(value.stock.sourceFingerprint)},
            {std::string("margin_x"), value.stock.marginX},
            {std::string("margin_y"), value.stock.marginY},
            {std::string("margin_z"), value.stock.marginZ},
        }},
        {std::string("post"), oap::JsonObject{
            {std::string("id"), fromUtf8(value.post.id)},
            {std::string("version"), fromUtf8(value.post.version)},
            {std::string("program_name"), fromUtf8(value.post.options.programName)},
            {std::string("tool_id"), fromUtf8(value.post.options.toolId)},
            {std::string("tool_name"), fromUtf8(value.post.options.toolName)},
            {std::string("coolant"), value.post.options.coolant},
        }},
        {std::string("collision_scene"), oap::JsonObject{
            {std::string("confirmed"), value.collisionScene.confirmed},
            {std::string("models"), collisionModels},
        }},
        {std::string("active_tool_library_id"), fromUtf8(value.activeToolLibraryId)},
        {std::string("algorithm_id"), fromUtf8(value.algorithmId)},
        {std::string("hole_parameters"), parametersToJson(value.holeParameters)},
        {std::string("pocket_parameters"),
         parametersToJson(value.pocketParameters)},
        {std::string("profile_parameters"),
         parametersToJson(value.profileParameters)},
        {std::string("recognized_holes"), recognizedHoles},
        {std::string("recognized_physical_hole_count"),
         value.recognizedPhysicalHoleCount},
        {std::string("setups"), setups},
        {std::string("tool_controllers"), controllers},
        {std::string("hole_operations"), holeOperations},
        {std::string("pocket_operations"), pocketOperations},
        {std::string("profile_operations"), profileOperations},
        {std::string("surface_finishing_operations"), detail::finishingOperationsToJson(value.finishingOperations)},
    };
}

MachiningJobDefinition definitionFromJson(const oap::JsonObject& object)
{
    // 反方向还原定义；缺失的可选字段回退到默认值，结构错误由后续校验捕获。
    MachiningJobDefinition result;
    result.schemaVersion = object.value(std::string("schema_version")).toInt();
    result.id = toUtf8(object.value(std::string("id")));
    result.finishingOperations = detail::finishingOperationsFromJson(object.value(std::string("surface_finishing_operations")).toArray());
    result.name = toUtf8(object.value(std::string("name")));
    result.createdAt = toUtf8(object.value(std::string("created_at")));
    result.modifiedAt = toUtf8(object.value(std::string("modified_at")));

    const oap::JsonObject part = object.value(std::string("part")).toObject();
    result.part.sourcePath = toUtf8(part.value(std::string("source_path")));
    result.part.sourceFingerprint = toUtf8(part.value(std::string("source_fingerprint")));
    result.part.units = toUtf8(part.value(std::string("units")));
    result.part.recognitionProfileVersion = toUtf8(part.value(std::string("recognition_profile_version")));

    const oap::JsonObject stock = object.value(std::string("stock")).toObject();
    result.stock.sourceKind = static_cast<StockSourceKind>(stock.value(std::string("source_kind")).toInt());
    result.stock.sourcePath = toUtf8(stock.value(std::string("source_path")));
    result.stock.sourceFingerprint = toUtf8(stock.value(std::string("source_fingerprint")));
    result.stock.marginX = stock.value(std::string("margin_x")).toDouble();
    result.stock.marginY = stock.value(std::string("margin_y")).toDouble();
    result.stock.marginZ = stock.value(std::string("margin_z")).toDouble();

    const oap::JsonObject post = object.value(std::string("post")).toObject();
    result.post.id = toUtf8(post.value(std::string("id")));
    result.post.version = toUtf8(post.value(std::string("version")));
    result.post.options.programName = toUtf8(post.value(std::string("program_name")));
    result.post.options.toolId = toUtf8(post.value(std::string("tool_id")));
    result.post.options.toolName = toUtf8(post.value(std::string("tool_name")));
    result.post.options.coolant = post.value(std::string("coolant")).toBool();
    const oap::JsonObject collisionScene =
        object.value(std::string("collision_scene")).toObject();
    result.collisionScene.confirmed =
        collisionScene.value(std::string("confirmed")).toBool();
    const oap::JsonArray collisionModels =
        collisionScene.value(std::string("models")).toArray();
    result.collisionScene.models.reserve(
        static_cast<std::size_t>(collisionModels.size()));
    for (const oap::JsonValue& value : collisionModels) {
        const oap::JsonObject modelObject = value.toObject();
        CollisionModelReference model;
        model.sourcePath = toUtf8(
            modelObject.value(std::string("source_path")));
        model.sourceFingerprint = toUtf8(
            modelObject.value(std::string("source_fingerprint")));
        model.role = static_cast<toolpath::CollisionBodyRole>(
            modelObject.value(std::string("role")).toInt());
        model.translation = vectorFromJson(
            modelObject.value(std::string("translation")));
        model.rotationDeg = vectorFromJson(
            modelObject.value(std::string("rotation_deg")));
        result.collisionScene.models.push_back(std::move(model));
    }

    result.activeToolLibraryId = toUtf8(object.value(std::string("active_tool_library_id")));
    result.algorithmId = toUtf8(object.value(std::string("algorithm_id")));
    result.holeParameters = parametersFromJson(object.value(std::string("hole_parameters")));
    if (object.contains(std::string("pocket_parameters"))) {
        result.pocketParameters = parametersFromJson(
            object.value(std::string("pocket_parameters")));
    } else {
        result.pocketParameters = result.holeParameters;
    }
    if (object.contains(std::string("profile_parameters"))) {
        result.profileParameters = parametersFromJson(
            object.value(std::string("profile_parameters")));
    } else {
        result.profileParameters = result.pocketParameters;
    }
    const oap::JsonArray recognizedHoles =
        object.value(std::string("recognized_holes")).toArray();
    result.recognizedHoles.reserve(
        static_cast<std::size_t>(recognizedHoles.size()));
    for (const oap::JsonValue& value : recognizedHoles)
        result.recognizedHoles.push_back(holeFromJson(value));
    result.recognizedPhysicalHoleCount = object.value(
        std::string("recognized_physical_hole_count")).toInt();

    const oap::JsonArray setups = object.value(std::string("setups")).toArray();
    result.setups.reserve(static_cast<std::size_t>(setups.size()));
    for (const oap::JsonValue& value : setups) {
        const oap::JsonObject item = value.toObject();
        SetupDefinition setup;
        setup.id = toUtf8(item.value(std::string("id")));
        setup.setupIndex = item.value(std::string("setup_index")).toInt();
        setup.face = static_cast<MoldSetupFace>(item.value(std::string("face")).toInt());
        setup.workOffset = static_cast<toolpath::WorkCoordinateSystem>(item.value(std::string("work_offset")).toInt());
        setup.origin = vectorFromJson(item.value(std::string("origin")));
        setup.xAxis = vectorFromJson(item.value(std::string("x_axis")));
        setup.yAxis = vectorFromJson(item.value(std::string("y_axis")));
        setup.zAxis = vectorFromJson(item.value(std::string("z_axis")));
        setup.valid = item.value(std::string("valid")).toBool();
        setup.requiresPhysicalReclamp = item.value(std::string("requires_physical_reclamp")).toBool();
        setup.axisMapping = toUtf8(item.value(std::string("axis_mapping")));
        setup.operatorInstruction = toUtf8(item.value(std::string("operator_instruction")));
        const oap::JsonObject confirmation = item.value(std::string("confirmation")).toObject();
        setup.confirmation.confirmed = confirmation.value(std::string("confirmed")).toBool();
        setup.confirmation.confirmedBy = toUtf8(confirmation.value(std::string("confirmed_by")));
        setup.confirmation.confirmedAt = toUtf8(confirmation.value(std::string("confirmed_at")));
        setup.confirmation.confirmedInputFingerprint = toUtf8(confirmation.value(std::string("input_fingerprint")));
        result.setups.push_back(std::move(setup));
    }

    const oap::JsonArray controllers = object.value(std::string("tool_controllers")).toArray();
    result.toolControllers.reserve(static_cast<std::size_t>(controllers.size()));
    for (const oap::JsonValue& value : controllers) {
        const oap::JsonObject item = value.toObject();
        JobToolController controller;
        controller.id = toUtf8(item.value(std::string("id")));
        controller.libraryId = toUtf8(item.value(std::string("library_id")));
        controller.tool = toolContextFromJson(item.value(std::string("tool")));
        result.toolControllers.push_back(std::move(controller));
    }

    const oap::JsonArray operations = object.value(std::string("hole_operations")).toArray();
    result.holeOperations.reserve(static_cast<std::size_t>(operations.size()));
    for (const oap::JsonValue& value : operations) {
        const oap::JsonObject item = value.toObject();
        HoleOperationDefinition operation;
        operation.id = toUtf8(item.value(std::string("id")));
        operation.setupId = toUtf8(item.value(std::string("setup_id")));
        operation.toolControllerId = toUtf8(item.value(std::string("tool_controller_id")));
        operation.phase = static_cast<toolpath::HoleMachiningPhase>(item.value(std::string("phase")).toInt());
        operation.enabled = item.value(std::string("enabled")).toBool(true);
        operation.sequence = item.value(std::string("sequence")).toInt();
        operation.state = static_cast<JobOperationState>(item.value(std::string("state")).toInt());
        operation.inputFingerprint = toUtf8(item.value(std::string("input_fingerprint")));
        const oap::JsonArray targets = item.value(std::string("targets")).toArray();
        operation.targets.reserve(static_cast<std::size_t>(targets.size()));
        for (const oap::JsonValue& targetValue : targets) {
            const oap::JsonObject targetObject = targetValue.toObject();
            HoleTargetReference target;
            target.featureId = toUtf8(targetObject.value(std::string("feature_id")));
            target.geometryFingerprint = toUtf8(targetObject.value(std::string("geometry_fingerprint")));
            target.sourceHoleIndex = targetObject.value(std::string("source_hole_index")).toInt(-1);
            const oap::JsonArray coveredSourceIndices = targetObject.value(
                std::string("covered_source_hole_indices")).toArray();
            target.coveredSourceHoleIndices.reserve(
                static_cast<std::size_t>(coveredSourceIndices.size()));
            for (const oap::JsonValue& sourceIndex : coveredSourceIndices)
                target.coveredSourceHoleIndices.push_back(sourceIndex.toInt(-1));
            target.physicalHoleIndex = targetObject.value(std::string("physical_hole_index")).toInt(-1);
            target.strategy = static_cast<toolpath::HoleMachiningStrategy>(targetObject.value(std::string("strategy")).toInt());
            target.hole = holeFromJson(targetObject.value(std::string("hole")));
            operation.targets.push_back(std::move(target));
        }
        result.holeOperations.push_back(std::move(operation));
    }
    const oap::JsonArray pocketOperations =
        object.value(std::string("pocket_operations")).toArray();
    result.pocketOperations.reserve(
        static_cast<std::size_t>(pocketOperations.size()));
    for (const oap::JsonValue& value : pocketOperations) {
        const oap::JsonObject item = value.toObject();
        PocketOperationDefinition operation;
        if (item.contains(std::string("parameters")))
            operation.parameters = parametersFromJson(item.value(std::string("parameters")));
        operation.id = toUtf8(item.value(std::string("id")));
        operation.setupId = toUtf8(item.value(std::string("setup_id")));
        operation.toolControllerId = toUtf8(
            item.value(std::string("tool_controller_id")));
        operation.restMachiningPreviousToolDiameter =
            item.value(std::string("rest_previous_tool_diameter"))
                .toDouble(0.0);
        operation.algorithmId =
            toUtf8(item.value(std::string("algorithm_id")));
        const oap::JsonObject target =
            item.value(std::string("target")).toObject();
        operation.target.sourceKind =
            static_cast<PocketRegionSourceKind>(
                target.value(std::string("source_kind")).toInt(
                    static_cast<int>(
                        PocketRegionSourceKind::FeatureFaces)));
        operation.target.kind = static_cast<FeatureKind>(
            target.value(std::string("kind")).toInt(
                static_cast<int>(FeatureKind::Pocket)));
        operation.target.sourceFaceIds = stringsFromJson(
            target.value(std::string("source_face_ids")));
        operation.target.floorFaceId =
            toUtf8(target.value(std::string("floor_face_id")));
        operation.target.rootFaceId =
            toUtf8(target.value(std::string("root_face_id")));
        operation.target.rootWireId =
            toUtf8(target.value(std::string("root_wire_id")));
        operation.target.allowOpenSectionContours = target.value(
            std::string("allow_open_section_contours")).toBool();
        operation.target.openBoundaryFaceCount = target.value(
            std::string("open_boundary_face_count")).toInt();
        operation.target.geometryFingerprint = toUtf8(
            target.value(std::string("geometry_fingerprint")));
        operation.enabled =
            item.value(std::string("enabled")).toBool(true);
        operation.sequence =
            item.value(std::string("sequence")).toInt();
        operation.state = static_cast<JobOperationState>(
            item.value(std::string("state")).toInt());
        operation.inputFingerprint = toUtf8(
            item.value(std::string("input_fingerprint")));
        result.pocketOperations.push_back(std::move(operation));
    }
    const oap::JsonArray profileOperations =
        object.value(std::string("profile_operations")).toArray();
    result.profileOperations.reserve(
        static_cast<std::size_t>(profileOperations.size()));
    for (const oap::JsonValue& value : profileOperations) {
        const oap::JsonObject item = value.toObject();
        ProfileOperationDefinition operation;
        operation.id = toUtf8(item.value(std::string("id")));
        operation.setupId = toUtf8(item.value(std::string("setup_id")));
        operation.toolControllerId = toUtf8(
            item.value(std::string("tool_controller_id")));
        operation.algorithmId = toUtf8(
            item.value(std::string("algorithm_id")));
        const oap::JsonObject target =
            item.value(std::string("target")).toObject();
        operation.target.axisLabel = toUtf8(
            target.value(std::string("axis_label")));
        operation.target.normalDirection = target.value(
            std::string("normal_direction")).toInt(1);
        operation.target.sourceFaceIds = stringsFromJson(
            target.value(std::string("source_face_ids")));
        operation.target.machiningFaceIds = stringsFromJson(
            target.value(std::string("machining_face_ids")));
        operation.target.terminalFaceIds = stringsFromJson(
            target.value(std::string("terminal_face_ids")));
        operation.target.mainOuterFaceIds = stringsFromJson(
            target.value(std::string("main_outer_face_ids")));
        operation.target.rootFaceId = toUtf8(
            target.value(std::string("root_face_id")));
        operation.target.rootWireId = toUtf8(
            target.value(std::string("root_wire_id")));
        operation.target.geometryFingerprint = toUtf8(
            target.value(std::string("geometry_fingerprint")));
        operation.restMachiningPreviousToolDiameter = item.value(
            std::string("rest_previous_tool_diameter")).toDouble(0.0);
        operation.finishPass = item.value(
            std::string("finish_pass")).toBool(true);
        operation.enabled =
            item.value(std::string("enabled")).toBool(true);
        operation.sequence =
            item.value(std::string("sequence")).toInt();
        operation.state = static_cast<JobOperationState>(
            item.value(std::string("state")).toInt());
        operation.inputFingerprint = toUtf8(
            item.value(std::string("input_fingerprint")));
        result.profileOperations.push_back(std::move(operation));
    }
    return result;
}

void migrateLegacyHoleSources(
    MachiningJobDefinition& definition,
    int sourceVersion)
{
    // 旧 schema（V1/V2/V3）到 V4 的孔来源迁移；V4 之后只补 Profile
    // 及其多刀字段的默认值，不需要改动孔定义。
    if (sourceVersion >= 4)
        return;

    int maximumSourceIndex = -1;
    int maximumPhysicalIndex = -1;
    for (HoleOperationDefinition& operation :
         definition.holeOperations) {
        for (HoleTargetReference& target : operation.targets) {
            maximumSourceIndex = std::max(
                maximumSourceIndex, target.sourceHoleIndex);
            maximumPhysicalIndex = std::max(
                maximumPhysicalIndex, target.physicalHoleIndex);
            // 早期 V1 目标早于合并覆盖来源，其隐式地代表自身的识别阶段。
            if (target.coveredSourceHoleIndices.empty() &&
                target.sourceHoleIndex >= 0) {
                target.coveredSourceHoleIndices.push_back(
                    target.sourceHoleIndex);
            }
        }
    }

    // 最早的 V1 写入器只持久化了每个 Operation 目标内嵌的
    // toolpath::HoleMachiningInput。重建后来才成为权威来源的孔段数组时
    // 不要臆造几何；格式错误的空缺在验证时仍保持无效。
    if (definition.recognizedHoles.empty() && maximumSourceIndex >= 0) {
        definition.recognizedHoles.resize(
            static_cast<std::size_t>(maximumSourceIndex) + 1);
        std::vector<bool> populated(
            definition.recognizedHoles.size(), false);
        for (const HoleOperationDefinition& operation :
             definition.holeOperations) {
            for (const HoleTargetReference& target : operation.targets) {
                if (target.sourceHoleIndex < 0 ||
                    target.sourceHoleIndex > maximumSourceIndex) {
                    continue;
                }
                const std::size_t index = static_cast<std::size_t>(
                    target.sourceHoleIndex);
                if (populated[index])
                    continue;
                definition.recognizedHoles[index] = target.hole;
                populated[index] = true;
            }
        }
    }
    if (!definition.recognizedHoles.empty() &&
        definition.recognizedPhysicalHoleCount <= 0 &&
        maximumPhysicalIndex >= 0) {
        definition.recognizedPhysicalHoleCount =
            maximumPhysicalIndex + 1;
    }
}

bool parseDocument(const std::string& data,
                   oap::JsonObject& root,
                   std::string* error)
{
    // 统一入口：体积上限 → JSON 语法 → 格式标识 → schema 版本区间 → job 节点。
    if (data.size() > kMaxJobBytes) {
        if (error) *error = std::format("加工项目超过 {0} MB 上限", kMaxJobBytes / (1024 * 1024));
        return false;
    }
    oap::JsonParseError parseError;
    const oap::JsonDocument document = oap::JsonDocument::fromJson(data, &parseError);
    if (parseError || !document.isObject()) {
        if (error) *error = std::format("加工项目 JSON 无效：{0}", parseError.message);
        return false;
    }
    root = document.object();
    // 旧项目可显式打开；重新保存统一写入 OAP 格式标识。
    const std::string format = root.value(std::string("format")).toString();
    if (format != kFormatId &&
        format != std::string("faceface-machining-job")) {
        if (error) *error = std::string("不是 OpenAutoPath 加工项目文件");
        return false;
    }
    const int version = root.value(std::string("schema_version")).toInt();
    if (version < kMinimumSupportedMachiningJobSchemaVersion ||
        version > kMachiningJobSchemaVersion) {
        if (error) *error = std::format("不支持的加工项目版本：{0}", version);
        return false;
    }
    const oap::JsonValue jobValue = root.value(std::string("job"));
    if (!jobValue.isObject()) {
        if (error) *error = std::string("加工项目缺少 job 定义");
        return false;
    }
    const oap::JsonValue legacySurfaces = jobValue.toObject()
        .value(std::string("surface_operations"));
    if (!legacySurfaces.isUndefined() &&
        (!legacySurfaces.isArray() || !legacySurfaces.toArray().empty())) {
        if (error) *error = std::string("旧版曲面加工模块已移除，其工序与 OpenCAMLib 精加工定义不兼容，无法自动迁移。请使用旧版处理这些工序。");
        return false;
    }
    const int definitionVersion = jobValue.toObject()
        .value(std::string("schema_version")).toInt();
    if (definitionVersion != version) {
        if (error) {
            *error = std::string("加工项目根版本与 job 定义版本不一致");
        }
        return false;
    }
    return true;
}

std::string retiredManualInputError(const oap::JsonObject& job)
{
    const auto plan = job.value(std::string("manual_hole_plan"));
    if (!plan.isUndefined()) {
        if (!plan.isObject() || !plan.toObject().value(std::string("stages")).isArray() ||
            !plan.toObject().value(std::string("stages")).toArray().empty())
            return std::string("项目包含已移除的人工孔段计划或无效人工计划；请使用旧版本处理原项目。");
    }
    for (const auto& value : job.value(std::string("hole_operations")).toArray()) {
        const auto operation = value.toObject();
        const auto overrides = operation.value(std::string("operation_overrides"));
        const auto local = operation.value(std::string("local_batch"));
        const auto overrideObject = overrides.toObject();
        const auto retract = overrideObject.value(std::string("retract_preference"));
        // V4-V10 wrote the inherited retract preference even without any override.
        const bool inheritedOnly = overrideObject.size() == 1 && retract.isDouble() && retract.toDouble() == 0.0;
        if ((!overrides.isUndefined() && (!overrides.isObject() || (!overrideObject.empty() && !inheritedOnly))) ||
            (!local.isUndefined() && (!local.isBool() || local.toBool())))
            return std::string("项目包含已移除的手动孔工艺覆盖或局部人工计划；请使用旧版本处理原项目。");
    }
    for (const auto& value : job.value(std::string("tool_controllers")).toArray()) {
        const auto controller = value.toObject();
        const auto manual = controller.value(std::string("manual"));
        if (!manual.isUndefined() && (!manual.isBool() || manual.toBool()))
            return std::string("项目包含已移除的手工刀具；请使用旧版本将工序绑定到刀库后再迁移。");
    }
    return {};
}

std::string validateCurrentShape(const oap::JsonObject& job)
{
    if (!job.value(std::string("stock")).isObject() ||
        !job.value(std::string("stock")).toObject().value(std::string("source_fingerprint")).isString())
        return std::string("V12 planning stock content identity must be a string");
    // 当前 schema 的快速结构预检（不含语义校验）。
    const oap::JsonValue recognizedValue =
        job.value(std::string("recognized_holes"));
    if (!recognizedValue.isArray())
        return std::string("V4 recognized_holes is not an array");
    for (const oap::JsonValue& value : recognizedValue.toArray()) {
        const oap::JsonObject hole = value.toObject();
        if (!value.isObject() ||
            !hole.value(std::string("manual")).isBool() ||
            !hole.value(std::string("stage_wall_face_ids")).isArray() ||
            !hole.value(std::string("stage_role")).isDouble()) {
            return std::string("V4 recognized hole fields are incomplete");
        }
    }
    for (const auto& value : job.value(std::string("pocket_operations")).toArray()) {
        const auto params = value.toObject().value(std::string("parameters"));
        if (params.isUndefined()) continue;
        if (job.value(std::string("schema_version")).toInt() < 13)
            return std::string("Independent pocket parameters require V13");
        const auto normalized = parametersToJson(parametersFromJson(params));
        if (!params.isObject() || normalized != params.toObject())
            return std::string("Invalid pocket operation parameters");
    }
    const oap::JsonValue operationsValue =
        job.value(std::string("hole_operations"));
    if (!operationsValue.isArray())
        return std::string("V4 hole_operations is not an array");
    for (const oap::JsonValue& value : operationsValue.toArray()) {
        const oap::JsonObject operation = value.toObject();
        if (!value.isObject()) {
            return std::string("V4 hole operation fields are incomplete");
        }
    }
    if (!job.value(std::string("profile_parameters")).isObject() ||
        !job.value(std::string("profile_operations")).isArray()) {
        return std::string("V5 profile fields are incomplete");
    }
    for (const oap::JsonValue& value :
         job.value(std::string("profile_operations")).toArray()) {
        const oap::JsonObject operation = value.toObject();
        if (!value.isObject() ||
            !operation.value(
                std::string("rest_previous_tool_diameter")).isDouble() ||
            !operation.value(std::string("finish_pass")).isBool()) {
            return std::string("V6 profile rest-machining fields are incomplete");
        }
    }
    return {};
}

oap::JsonArray executionOperations(oap::JsonArray operations)
{
    // 归一化操作数组用于执行指纹：去掉运行期 state，并按 sequence+id 排序，
    // 使声明顺序等非执行因素不影响指纹。
    std::vector<oap::JsonObject> normalized;
    normalized.reserve(static_cast<std::size_t>(operations.size()));
    for (std::size_t index = 0; index < operations.size(); ++index) {
        oap::JsonObject operation = operations.at(index).toObject();
        operation.remove(std::string("state"));
        normalized.push_back(std::move(operation));
    }
    std::sort(
        normalized.begin(), normalized.end(),
        [](const oap::JsonObject& left, const oap::JsonObject& right) {
            const int leftSequence =
                left.value(std::string("sequence")).toInt();
            const int rightSequence =
                right.value(std::string("sequence")).toInt();
            if (leftSequence != rightSequence)
                return leftSequence < rightSequence;
            return left.value(std::string("id")).toString() <
                right.value(std::string("id")).toString();
        });
    oap::JsonArray result;
    for (const oap::JsonObject& operation : normalized)
        result.append(operation);
    return result;
}

oap::JsonArray referencedResources(
    const oap::JsonArray& resources,
    const oap::JsonArray& operations,
    const std::string& referenceKey)
{
    // 只保留执行指纹真正依赖的资源；Setup 还剔除确认/指令等非执行字段。
    std::set<std::string> referenced;
    for (const oap::JsonValue& value : operations) {
        referenced.insert(
            value.toObject().value(referenceKey).toString());
    }
    std::vector<oap::JsonObject> normalized;
    for (const oap::JsonValue& value : resources) {
        oap::JsonObject item = value.toObject();
        if (!referenced.contains(item.value(std::string("id")).toString()))
            continue;
        if (referenceKey == std::string("setup_id")) {
            item.remove(std::string("confirmation"));
            item.remove(std::string("operator_instruction"));
        }
        normalized.push_back(std::move(item));
    }
    std::sort(
        normalized.begin(), normalized.end(),
        [](const oap::JsonObject& left, const oap::JsonObject& right) {
            return left.value(std::string("id")).toString() <
                right.value(std::string("id")).toString();
        });
    oap::JsonArray result;
    for (const oap::JsonObject& item : normalized)
        result.append(item);
    return result;
}



enum class ExecutionDomain {
    Hole,
    Pocket,
    Profile,
    SurfaceFinishing,
};

oap::JsonObject domainExecutionJson(
    const MachiningJobDefinition& definition, ExecutionDomain domain)
{
    // 按领域（孔/槽腔）裁剪定义生成执行 JSON，作为各自执行指纹的输入；
    // 移除 id/名称/时间等元数据与另一领域的参数和工序。
    oap::JsonObject result = definitionToJson(definition);
    result.remove(std::string("id"));
    result.remove(std::string("name"));
    result.remove(std::string("created_at"));
    result.remove(std::string("modified_at"));
    result.remove(std::string("post"));
    result.remove(std::string("active_tool_library_id"));

    const std::string operationsKey =
        domain == ExecutionDomain::Hole
            ? std::string("hole_operations")
            : domain == ExecutionDomain::Pocket
                ? std::string("pocket_operations")
                : domain == ExecutionDomain::Profile
                    ? std::string("profile_operations") : std::string("surface_finishing_operations");
    const oap::JsonArray operations = executionOperations(
        result.value(operationsKey).toArray());
    result.insert(operationsKey, operations);
    result.insert(
        std::string("setups"),
        referencedResources(
            result.value(std::string("setups")).toArray(),
            operations, std::string("setup_id")));
    result.insert(
        std::string("tool_controllers"),
        referencedResources(
            result.value(std::string("tool_controllers")).toArray(),
            operations, std::string("tool_controller_id")));
    if (domain != ExecutionDomain::SurfaceFinishing)
        result.remove(std::string("surface_finishing_operations"));
    if (domain == ExecutionDomain::SurfaceFinishing) {
        for (const char* key : {"algorithm_id", "hole_parameters", "recognized_holes", "recognized_physical_hole_count",
             "hole_operations", "manual_hole_plan", "pocket_parameters", "pocket_operations", "profile_parameters", "profile_operations"})
            result.remove(key);
    }
    if (domain == ExecutionDomain::Hole) {
        result.remove(std::string("pocket_parameters"));
        result.remove(std::string("pocket_operations"));
        result.remove(std::string("profile_parameters"));
        result.remove(std::string("profile_operations"));
    } else if (domain == ExecutionDomain::Pocket) {
        result.remove(std::string("algorithm_id"));
        result.remove(std::string("hole_parameters"));
        result.remove(std::string("recognized_holes"));
        result.remove(std::string("recognized_physical_hole_count"));
        result.remove(std::string("hole_operations"));
        result.remove(std::string("manual_hole_plan"));
        result.remove(std::string("profile_parameters"));
        result.remove(std::string("profile_operations"));
    } else if (domain == ExecutionDomain::Profile) {
        result.remove(std::string("algorithm_id"));
        result.remove(std::string("hole_parameters"));
        result.remove(std::string("recognized_holes"));
        result.remove(std::string("recognized_physical_hole_count"));
        result.remove(std::string("hole_operations"));
        result.remove(std::string("manual_hole_plan"));
        result.remove(std::string("pocket_parameters"));
        result.remove(std::string("pocket_operations"));
    }
    return result;
}

std::string jsonFingerprint(const oap::JsonObject& value)
{
    return std::string(oap::sha256Hex(oap::JsonDocument(value).toJson(oap::JsonDocument::Compact)));
}

} // namespace

std::string MachiningJobStorage::serializeDefinition(
    const MachiningJobDefinition& definition)
{
    // 输出 {format, schema_version, job} 根结构，job 内嵌定义 JSON；
    // 定义指纹直接以此完整序列化结果为输入。
    auto job = definitionToJson(definition);
    const bool hasPocketOverrides = std::any_of(definition.pocketOperations.begin(), definition.pocketOperations.end(),
        [](const PocketOperationDefinition& op) { return op.parameters.has_value(); });
    const int fileVersion = hasPocketOverrides ? kMachiningJobSchemaVersion : 12;
    job.insert(std::string("schema_version"), fileVersion);
    const oap::JsonObject root{
        {std::string("format"), kFormatId},
        {std::string("schema_version"), fileVersion},
        {std::string("job"), job},
    };
    return oap::JsonDocument(root).toJson(oap::JsonDocument::Indented);
}

bool MachiningJobStorage::deserializeDefinition(
    const std::string& data,
    MachiningJobDefinition& definition,
    std::string* error)
{
    // 加载流水线：解析 → 当前结构预检 → 反序列化 → 旧版本迁移 →
    // Setup 确认失效校验 → 领域校验，全部通过后才提交到调用方定义。
    oap::JsonObject root;
    if (!parseDocument(data, root, error))
        return false;
    const oap::JsonObject jobObject =
        root.value(std::string("job")).toObject();
    const int sourceVersion =
        root.value(std::string("schema_version")).toInt();
    const std::string manualError = retiredManualInputError(jobObject);
    if (!manualError.empty()) {
        if (error) *error = manualError;
        return false;
    }
    const std::string finishingShapeError = detail::validateFinishingJson(jobObject, sourceVersion >= 10);
    if (!finishingShapeError.empty()) {
        if (error) *error = finishingShapeError;
        return false;
    }
    if (sourceVersion >= 12) {
        const std::string shapeError = validateCurrentShape(jobObject);
        if (!shapeError.empty()) {
            if (error)
                *error = std::format("加工项目结构无效：{0}", shapeError);
            return false;
        }
    }
    MachiningJobDefinition parsed =
        definitionFromJson(jobObject);
    migrateLegacyHoleSources(parsed, sourceVersion);
    // 旧版本迁移只发生在内存，再保存统一写当前 schema。
    parsed.schemaVersion = kMachiningJobSchemaVersion;
    std::vector<std::string> invalidConfirmations;
    // 重算每个已确认 Setup 的输入指纹：与保存值不一致说明零件/装夹已变化，
    // 清空确认并把关联工序标记为 NeedsReview，防止沿用过时确认。
    for (SetupDefinition& setup : parsed.setups) {
        if (!setup.confirmation.confirmed)

            continue;
        const std::string expected = setupInputFingerprint(
            parsed.part, setup);
        if (setup.confirmation.confirmedInputFingerprint == expected)
            continue;
        setup.confirmation = {};
        invalidConfirmations.push_back(setup.id);
    }
    for (HoleOperationDefinition& operation : parsed.holeOperations) {
        if (std::find(invalidConfirmations.begin(),
                      invalidConfirmations.end(),
                      operation.setupId) != invalidConfirmations.end()) {
            operation.state = JobOperationState::NeedsReview;
        }
    }
    for (PocketOperationDefinition& operation : parsed.pocketOperations) {
        if (std::find(invalidConfirmations.begin(),
                      invalidConfirmations.end(),
                      operation.setupId) != invalidConfirmations.end()) {
            operation.state = JobOperationState::NeedsReview;
        }
    }
    for (ProfileOperationDefinition& operation : parsed.profileOperations) {
        if (std::find(invalidConfirmations.begin(),
                      invalidConfirmations.end(),
                      operation.setupId) != invalidConfirmations.end()) {
            operation.state = JobOperationState::NeedsReview;
        }
    }
    for (auto& operation : parsed.finishingOperations) {
        if (std::find(invalidConfirmations.begin(), invalidConfirmations.end(), operation.setupId) != invalidConfirmations.end())
            operation.state = JobOperationState::NeedsReview;
    }
    MachiningJob job;
    job.definition() = parsed;
    const std::vector<std::string> errors = job.validationErrors();
    if (!errors.empty()) {
        if (error) *error = std::format("加工项目结构无效：{0}", std::string(errors.front()));
        return false;
    }
    definition = std::move(parsed);
    return true;
}

bool MachiningJobStorage::save(
    const std::string& filePath,
    const MachiningJobDefinition& definition,
    std::string* error)
{
    return oap::writeFileAtomically(filePath, serializeDefinition(definition), error);
}

bool MachiningJobStorage::load(
    const std::string& filePath,
    MachiningJobDefinition& definition,
    std::string* error)
{
    std::string data;
    return oap::readFile(filePath, data, error, kMaxJobBytes) &&
           deserializeDefinition(data, definition, error);
}

std::string MachiningJobStorage::definitionFingerprint(
    const MachiningJobDefinition& definition)
{
    // 整个定义（含未启用内容）的规范化序列化哈希，任何字段变化都会改变它。
    return std::string(oap::sha256Hex(serializeDefinition(definition)));
}

std::string MachiningJobStorage::holeExecutionFingerprint(
    const MachiningJobDefinition& definition)
{
    // 孔执行指纹基于孔领域执行 JSON；其他工序族不参与其身份。
    return jsonFingerprint(domainExecutionJson(
        definition, ExecutionDomain::Hole));
}

std::string MachiningJobStorage::pocketExecutionFingerprint(
    const MachiningJobDefinition& definition)
{
    // 槽腔执行指纹基于槽腔领域执行 JSON；孔相关字段不参与其身份。
    return jsonFingerprint(domainExecutionJson(
        definition, ExecutionDomain::Pocket));
}

std::string MachiningJobStorage::profileExecutionFingerprint(
    const MachiningJobDefinition& definition)
{
    return jsonFingerprint(domainExecutionJson(
        definition, ExecutionDomain::Profile));
}

std::string MachiningJobStorage::finishingExecutionFingerprint(const MachiningJobDefinition& definition)
{
    return jsonFingerprint(domainExecutionJson(definition, ExecutionDomain::SurfaceFinishing));
}

std::string MachiningJobStorage::setupInputFingerprint(
    const PartDefinition& part,
    const SetupDefinition& setup)
{
    // 零件指纹 + 装夹关键值构成 Setup 的输入身份，用于确认失效与操作指纹。
    const oap::JsonObject input{
        {std::string("part_fingerprint"),
         fromUtf8(part.sourceFingerprint)},
        {std::string("setup_id"), fromUtf8(setup.id)},
        {std::string("setup_index"), setup.setupIndex},
        {std::string("face"), static_cast<int>(setup.face)},
        {std::string("work_offset"),
         static_cast<int>(setup.workOffset)},
        {std::string("origin"), vectorToJson(setup.origin)},
        {std::string("x_axis"), vectorToJson(setup.xAxis)},
        {std::string("y_axis"), vectorToJson(setup.yAxis)},
        {std::string("z_axis"), vectorToJson(setup.zAxis)},
        {std::string("axis_mapping"), fromUtf8(setup.axisMapping)},
    };
    return oap::sha256Hex(oap::JsonDocument(input).toFingerprintJson());
}

std::string MachiningJobStorage::sourceFileFingerprint(
    const std::string& filePath,
    std::string* error)
{
    // 对源文件做分块流式 SHA-256，避免一次性读入大模型文件。
    std::string hash;
    if (!oap::sha256FileHex(filePath, hash)) {
        if (error) *error = std::format("无法读取文件并计算指纹：{}", filePath);
        return {};
    }
    return hash;
}

} // namespace oap::manufacturing

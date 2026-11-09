
// MachiningJobStorage 的实现（oap_manufacturing 持久化层）：MachiningJobDefinition
// 的 JSON 编解码、.oapjob 文件读写、旧 schema 迁移以及定义/执行/来源指纹计算。
// 全部为无状态逻辑，错误通过 QString* error 返回；不涉及刀路生成与 UI。
#include "manufacturing/job/MachiningJobStorage.h"
#include "manufacturing/job/detail/SurfaceFinishingJson.h"


#include <algorithm>
#include <array>
#include <limits>
#include <utility>

#include <QCryptographicHash>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSet>
#include <QJsonParseError>
#include <QSaveFile>

namespace oap::manufacturing {
namespace {

// ---- 以下为定义的 JSON 编解码辅助函数，字段名即文件格式的一部分，不得改名。 ----

constexpr qint64 kMaxJobBytes = 32ll * 1024ll * 1024ll;
constexpr auto kFormatId = "oap-machining-job";

QString fromUtf8(const std::string& value)
{
    return QString::fromUtf8(value.data(), static_cast<qsizetype>(value.size()));
}

std::string toUtf8(const QJsonValue& value)
{
    return value.toString().toUtf8().toStdString();
}

QJsonObject vectorToJson(const JobVector3& value)
{
    return {{QStringLiteral("x"), value.x},
            {QStringLiteral("y"), value.y},
            {QStringLiteral("z"), value.z}};
}

JobVector3 vectorFromJson(const QJsonValue& value)
{
    const QJsonObject object = value.toObject();
    return {object.value(QStringLiteral("x")).toDouble(),
            object.value(QStringLiteral("y")).toDouble(),
            object.value(QStringLiteral("z")).toDouble()};
}

QJsonArray stringsToJson(const std::vector<std::string>& values)
{
    QJsonArray result;
    for (const std::string& value : values)
        result.append(fromUtf8(value));
    return result;
}

std::vector<std::string> stringsFromJson(const QJsonValue& value)
{
    std::vector<std::string> result;
    const QJsonArray array = value.toArray();
    result.reserve(static_cast<std::size_t>(array.size()));
    for (const QJsonValue& item : array)
        result.push_back(toUtf8(item));
    return result;
}

QJsonObject toolDefToJson(const toolpath::ToolDef& value)
{
    return {
        {QStringLiteral("diameter"), value.diameter},
        {QStringLiteral("corner_radius"), value.cornerRadius},
        {QStringLiteral("horizontal_feed"), value.horizFeed},
        {QStringLiteral("vertical_feed"), value.vertFeed},
        {QStringLiteral("horizontal_rapid"), value.horizRapid},
        {QStringLiteral("vertical_rapid"), value.vertRapid},
        {QStringLiteral("spindle_speed"), value.spindleSpeed},
        {QStringLiteral("cutting_edge_length"), value.cuttingEdgeLength},
        {QStringLiteral("overall_length"), value.overallLength},
        {QStringLiteral("shank_diameter"), value.shankDiameter},
        {QStringLiteral("holder_diameter"), value.holderDiameter},
        {QStringLiteral("holder_length"), value.holderLength},
        {QStringLiteral("spindle_envelope_diameter"),
         value.spindleEnvelopeDiameter},
        {QStringLiteral("spindle_envelope_length"),
         value.spindleEnvelopeLength},
        {QStringLiteral("tip_compensation"), value.tipCompensation},
    };
}

toolpath::ToolDef toolDefFromJson(const QJsonValue& value)
{
    const QJsonObject object = value.toObject();
    toolpath::ToolDef result;
    result.diameter = object.value(QStringLiteral("diameter")).toDouble(result.diameter);
    result.cornerRadius = object.value(QStringLiteral("corner_radius")).toDouble();
    result.horizFeed = object.value(QStringLiteral("horizontal_feed")).toDouble(result.horizFeed);
    result.vertFeed = object.value(QStringLiteral("vertical_feed")).toDouble(result.vertFeed);
    result.horizRapid = object.value(QStringLiteral("horizontal_rapid")).toDouble(result.horizRapid);
    result.vertRapid = object.value(QStringLiteral("vertical_rapid")).toDouble(result.vertRapid);
    result.spindleSpeed = object.value(QStringLiteral("spindle_speed")).toDouble(result.spindleSpeed);
    result.cuttingEdgeLength = object.value(QStringLiteral("cutting_edge_length")).toDouble();
    result.overallLength = object.value(QStringLiteral("overall_length")).toDouble();
    result.shankDiameter = object.value(QStringLiteral("shank_diameter")).toDouble();
    result.holderDiameter = object.value(QStringLiteral("holder_diameter")).toDouble();
    result.holderLength = object.value(QStringLiteral("holder_length")).toDouble();
    result.spindleEnvelopeDiameter = object.value(
        QStringLiteral("spindle_envelope_diameter")).toDouble();
    result.spindleEnvelopeLength = object.value(
        QStringLiteral("spindle_envelope_length")).toDouble();
    result.tipCompensation = object.value(QStringLiteral("tip_compensation")).toDouble();
    return result;
}

QJsonObject depthToJson(const toolpath::DepthParams& value)
{
    return {
        {QStringLiteral("clearance_height"), value.clearanceHeight},
        {QStringLiteral("safe_height"), value.safeHeight},
        {QStringLiteral("use_feature_depth"), value.useFeatureDepth},
        {QStringLiteral("start_depth"), value.startDepth},
        {QStringLiteral("final_depth"), value.finalDepth},
        {QStringLiteral("step_down"), value.stepDown},
        {QStringLiteral("finish_depth"), value.finishDepth},
    };
}

toolpath::DepthParams depthFromJson(const QJsonValue& value)
{
    const QJsonObject object = value.toObject();
    toolpath::DepthParams result;
    result.clearanceHeight = object.value(QStringLiteral("clearance_height")).toDouble(result.clearanceHeight);
    result.safeHeight = object.value(QStringLiteral("safe_height")).toDouble(result.safeHeight);
    result.useFeatureDepth = object.value(QStringLiteral("use_feature_depth")).toBool(result.useFeatureDepth);
    result.startDepth = object.value(QStringLiteral("start_depth")).toDouble(result.startDepth);
    result.finalDepth = object.value(QStringLiteral("final_depth")).toDouble(result.finalDepth);
    result.stepDown = object.value(QStringLiteral("step_down")).toDouble(result.stepDown);
    result.finishDepth = object.value(QStringLiteral("finish_depth")).toDouble(result.finishDepth);
    return result;
}

QJsonObject clearingToJson(const toolpath::ClearingParams& value)
{
    const toolpath::AdaptiveClearingParams& adaptive = value.adaptive;
    return {
        {QStringLiteral("pattern"), static_cast<int>(value.pattern)},
        {QStringLiteral("expansion_direction"),
         static_cast<int>(toolpath::effectivePocketClearingDirection(
             value.pattern, value.expansionDirection))},
        {QStringLiteral("cut_mode"), static_cast<int>(value.cutMode)},
        {QStringLiteral("entry_mode"), static_cast<int>(value.entryMode)},
        {QStringLiteral("ramp_start_reference"),
         static_cast<int>(value.rampStartReference)},
        {QStringLiteral("ramp_angle_deg"), value.rampAngleDeg},
        {QStringLiteral("maximum_ramp_length"), value.maximumRampLength},
        {QStringLiteral("helix_diameter"), value.helixDiameter},
        {QStringLiteral("zigzag_ramp_width"), value.zigzagRampWidth},
        {QStringLiteral("ramp_feed"), value.rampFeed},
        {QStringLiteral("step_over_percent"), value.stepOverPercent},
        {QStringLiteral("extra_offset"), value.extraOffset},
        {QStringLiteral("precision"), value.precision},
        {QStringLiteral("arc_tolerance"), value.arcTolerance},
        {QStringLiteral("zigzag_angle"), value.zigzagAngle},
        {QStringLiteral("perimeter_pass"), value.perimeterPass},
        {QStringLiteral("adaptive"), QJsonObject{
             {QStringLiteral("target_engagement"), adaptive.targetEngagement},
             {QStringLiteral("target_angle_deg"), adaptive.targetEngagementAngleDeg},
             {QStringLiteral("maximum_angle_deg"), adaptive.maximumEngagementAngleDeg},
             {QStringLiteral("angle_samples"), adaptive.engagementAngleSamples},
             {QStringLiteral("minimum_turn_radius_factor"), adaptive.minimumTurnRadiusFactor},
             {QStringLiteral("remaining_tolerance"), adaptive.remainingTolerance},
             {QStringLiteral("material_cell_size"), adaptive.materialCellSize},
             {QStringLiteral("maximum_passes"), adaptive.maximumAdaptivePasses},
             {QStringLiteral("lookahead_depth"), adaptive.lookaheadDepth},
             {QStringLiteral("beam_width"), adaptive.beamWidth}}},
    };
}

toolpath::ClearingParams clearingFromJson(const QJsonValue& value)
{
    const QJsonObject object = value.toObject();
    toolpath::ClearingParams result;
    result.pattern = static_cast<toolpath::ClearingPattern>(
        object.value(QStringLiteral("pattern")).toInt(static_cast<int>(result.pattern)));
    result.expansionDirection =
        static_cast<toolpath::PocketClearingDirection>(
            object.value(QStringLiteral("expansion_direction")).toInt(
                static_cast<int>(result.expansionDirection)));
    result.expansionDirection =
        toolpath::effectivePocketClearingDirection(
            result.pattern, result.expansionDirection);
    result.cutMode = static_cast<toolpath::CutMode>(
        object.value(QStringLiteral("cut_mode")).toInt(static_cast<int>(result.cutMode)));
    result.entryMode = static_cast<toolpath::PocketEntryMode>(
        object.value(QStringLiteral("entry_mode")).toInt(
            static_cast<int>(result.entryMode)));
    result.rampStartReference =
        static_cast<toolpath::PocketRampStartReference>(
            object.value(QStringLiteral("ramp_start_reference")).toInt(
                static_cast<int>(result.rampStartReference)));
    result.rampAngleDeg =
        object.value(QStringLiteral("ramp_angle_deg")).toDouble(
            result.rampAngleDeg);
    result.maximumRampLength =
        object.value(QStringLiteral("maximum_ramp_length")).toDouble(
            result.maximumRampLength);
    result.helixDiameter =
        object.value(QStringLiteral("helix_diameter")).toDouble(
            result.helixDiameter);
    result.zigzagRampWidth =
        object.value(QStringLiteral("zigzag_ramp_width")).toDouble(
            result.zigzagRampWidth);
    result.rampFeed =
        object.value(QStringLiteral("ramp_feed")).toDouble(
            result.rampFeed);
    result.stepOverPercent = object.value(QStringLiteral("step_over_percent")).toDouble(result.stepOverPercent);
    result.extraOffset = object.value(QStringLiteral("extra_offset")).toDouble(result.extraOffset);
    result.precision = object.value(QStringLiteral("precision")).toInt(result.precision);
    result.arcTolerance = object.value(QStringLiteral("arc_tolerance")).toDouble(result.arcTolerance);
    result.zigzagAngle = object.value(QStringLiteral("zigzag_angle")).toDouble(result.zigzagAngle);
    result.perimeterPass = object.value(QStringLiteral("perimeter_pass")).toBool(result.perimeterPass);
    const QJsonObject adaptive =
        object.value(QStringLiteral("adaptive")).toObject();
    result.adaptive.targetEngagement = adaptive.value(
        QStringLiteral("target_engagement")).toDouble(
            result.adaptive.targetEngagement);
    result.adaptive.targetEngagementAngleDeg = adaptive.value(
        QStringLiteral("target_angle_deg")).toDouble(
            result.adaptive.targetEngagementAngleDeg);
    result.adaptive.maximumEngagementAngleDeg = adaptive.value(
        QStringLiteral("maximum_angle_deg")).toDouble(
            result.adaptive.maximumEngagementAngleDeg);
    result.adaptive.engagementAngleSamples = adaptive.value(
        QStringLiteral("angle_samples")).toInt(
            result.adaptive.engagementAngleSamples);
    result.adaptive.minimumTurnRadiusFactor = adaptive.value(
        QStringLiteral("minimum_turn_radius_factor")).toDouble(
            result.adaptive.minimumTurnRadiusFactor);
    result.adaptive.remainingTolerance = adaptive.value(
        QStringLiteral("remaining_tolerance")).toDouble(
            result.adaptive.remainingTolerance);
    result.adaptive.materialCellSize = adaptive.value(
        QStringLiteral("material_cell_size")).toDouble(
            result.adaptive.materialCellSize);
    result.adaptive.maximumAdaptivePasses = adaptive.value(
        QStringLiteral("maximum_passes")).toInt(
            result.adaptive.maximumAdaptivePasses);
    result.adaptive.lookaheadDepth = adaptive.value(
        QStringLiteral("lookahead_depth")).toInt(
            result.adaptive.lookaheadDepth);
    result.adaptive.beamWidth = adaptive.value(
        QStringLiteral("beam_width")).toInt(
            result.adaptive.beamWidth);
    return result;
}

QJsonObject profileToJson(const toolpath::ProfileParams& value)
{
    return {
        {QStringLiteral("machining_mode"), static_cast<int>(value.machiningMode)},
        {QStringLiteral("cut_mode"), static_cast<int>(value.cutMode)},
        {QStringLiteral("side_allowance"), value.sideAllowance},
        {QStringLiteral("roughing_allowance"), value.roughingAllowance},
        {QStringLiteral("finish_pass"), value.finishPass},
        {QStringLiteral("radial_pass_count"), value.radialPassCount},
        {QStringLiteral("radial_step_over"), value.radialStepOver},
        {QStringLiteral("lead_in_length"), value.leadInLength},
        {QStringLiteral("lead_out_length"), value.leadOutLength},
        {QStringLiteral("precision"), value.precision},
        {QStringLiteral("arc_tolerance"), value.arcTolerance},
    };
}

toolpath::ProfileParams profileFromJson(const QJsonValue& value)
{
    const QJsonObject object = value.toObject();
    toolpath::ProfileParams result;
    result.machiningMode = static_cast<toolpath::ProfileMachiningMode>(
        object.value(QStringLiteral("machining_mode")).toInt(
            static_cast<int>(result.machiningMode)));
    result.cutMode = static_cast<toolpath::CutMode>(
        object.value(QStringLiteral("cut_mode")).toInt(
            static_cast<int>(result.cutMode)));
    result.sideAllowance = object.value(
        QStringLiteral("side_allowance")).toDouble(result.sideAllowance);
    result.roughingAllowance = object.value(
        QStringLiteral("roughing_allowance")).toDouble(
            result.roughingAllowance);
    result.finishPass = object.value(
        QStringLiteral("finish_pass")).toBool(result.finishPass);
    result.radialPassCount = object.value(
        QStringLiteral("radial_pass_count")).toInt(result.radialPassCount);
    result.radialStepOver = object.value(
        QStringLiteral("radial_step_over")).toDouble(result.radialStepOver);
    result.leadInLength = object.value(
        QStringLiteral("lead_in_length")).toDouble(result.leadInLength);
    result.leadOutLength = object.value(
        QStringLiteral("lead_out_length")).toDouble(result.leadOutLength);
    result.precision = object.value(
        QStringLiteral("precision")).toInt(result.precision);
    result.arcTolerance = object.value(
        QStringLiteral("arc_tolerance")).toDouble(result.arcTolerance);
    return result;
}

QJsonObject holeParamsToJson(const toolpath::HoleMachiningParams& value)
{
    return {
        {QStringLiteral("start_corner"), static_cast<int>(value.startCorner)},
        {QStringLiteral("allowance"), value.allowance},
        {QStringLiteral("small_hole_diameter_tolerance"), value.smallHoleDiameterTolerance},
        {QStringLiteral("center_spot_depth"), value.centerSpotDepth},
        {QStringLiteral("through_hole_breakthrough"), value.throughHoleBreakthrough},
        {QStringLiteral("medium_hole_ratio"), value.mediumHoleRadiusToolDiameterRatio},
        {QStringLiteral("drill_cycle_mode"), static_cast<int>(value.drillCycleMode)},
        {QStringLiteral("drill_retract_mode"), static_cast<int>(value.drillRetractMode)},
        {QStringLiteral("peck_depth"), value.peckDepth},
        {QStringLiteral("dwell_seconds"), value.dwellSeconds},
        {QStringLiteral("drill_tip_angle_deg"), value.drillTipAngleDeg},
        {QStringLiteral("g81_max_ratio"), value.g81MaxDepthDiameterRatio},
        {QStringLiteral("g73_max_ratio"), value.g73MaxDepthDiameterRatio},
        {QStringLiteral("g73_peck_depth"), value.g73PeckDepth},
        {QStringLiteral("g83_peck_depth"), value.g83PeckDepth},
        {QStringLiteral("g73_chip_break_retract"), value.g73ChipBreakRetract},
        {QStringLiteral("g83_reentry_clearance"), value.g83ReentryClearance},
        {QStringLiteral("collision_envelope_mode"), static_cast<int>(value.collisionEnvelopeMode)},
        {QStringLiteral("collision_sample_step"), value.collisionEnvelopeSampleStep},
        {QStringLiteral("helix_pitch"), value.helixPitch},
        {QStringLiteral("radial_step"), value.radialStep},
        {QStringLiteral("sample_angle_deg"), value.sampleAngleDeg},
        {QStringLiteral("helix_chord_tolerance"), value.helixChordTolerance},
        {QStringLiteral("min_sample_angle_deg"), value.minSampleAngleDeg},
        {QStringLiteral("max_sample_angle_deg"), value.maxSampleAngleDeg},
        {QStringLiteral("max_sample_points"), value.maxSamplePoints},
    };
}

toolpath::HoleMachiningParams holeParamsFromJson(const QJsonValue& value)
{
    const QJsonObject object = value.toObject();
    toolpath::HoleMachiningParams result;
    result.startCorner = static_cast<toolpath::HoleStartCorner>(
        object.value(QStringLiteral("start_corner")).toInt(static_cast<int>(result.startCorner)));
    result.allowance = object.value(QStringLiteral("allowance")).toDouble(result.allowance);
    result.smallHoleDiameterTolerance = object.value(QStringLiteral("small_hole_diameter_tolerance")).toDouble(result.smallHoleDiameterTolerance);
    result.centerSpotDepth = object.value(QStringLiteral("center_spot_depth")).toDouble(result.centerSpotDepth);
    result.throughHoleBreakthrough = object.value(QStringLiteral("through_hole_breakthrough")).toDouble(result.throughHoleBreakthrough);
    result.mediumHoleRadiusToolDiameterRatio = object.value(QStringLiteral("medium_hole_ratio")).toDouble(result.mediumHoleRadiusToolDiameterRatio);
    result.drillCycleMode = static_cast<toolpath::HoleDrillCycleMode>(
        object.value(QStringLiteral("drill_cycle_mode")).toInt(static_cast<int>(result.drillCycleMode)));
    result.drillRetractMode = static_cast<toolpath::HoleDrillRetractMode>(
        object.value(QStringLiteral("drill_retract_mode")).toInt(static_cast<int>(result.drillRetractMode)));
    result.peckDepth = object.value(QStringLiteral("peck_depth")).toDouble(result.peckDepth);
    result.dwellSeconds = object.value(QStringLiteral("dwell_seconds")).toDouble(result.dwellSeconds);
    result.drillTipAngleDeg = object.value(QStringLiteral("drill_tip_angle_deg")).toDouble(result.drillTipAngleDeg);
    result.g81MaxDepthDiameterRatio = object.value(QStringLiteral("g81_max_ratio")).toDouble(result.g81MaxDepthDiameterRatio);
    result.g73MaxDepthDiameterRatio = object.value(QStringLiteral("g73_max_ratio")).toDouble(result.g73MaxDepthDiameterRatio);
    result.g73PeckDepth = object.value(QStringLiteral("g73_peck_depth")).toDouble(result.g73PeckDepth);
    result.g83PeckDepth = object.value(QStringLiteral("g83_peck_depth")).toDouble(result.g83PeckDepth);
    result.g73ChipBreakRetract = object.value(QStringLiteral("g73_chip_break_retract")).toDouble(result.g73ChipBreakRetract);
    result.g83ReentryClearance = object.value(QStringLiteral("g83_reentry_clearance")).toDouble(result.g83ReentryClearance);
    result.collisionEnvelopeMode = static_cast<toolpath::HoleCollisionEnvelopeMode>(
        object.value(QStringLiteral("collision_envelope_mode")).toInt(static_cast<int>(result.collisionEnvelopeMode)));
    result.collisionEnvelopeSampleStep = object.value(QStringLiteral("collision_sample_step")).toDouble(result.collisionEnvelopeSampleStep);
    result.helixPitch = object.value(QStringLiteral("helix_pitch")).toDouble(result.helixPitch);
    result.radialStep = object.value(QStringLiteral("radial_step")).toDouble(result.radialStep);
    result.sampleAngleDeg = object.value(QStringLiteral("sample_angle_deg")).toDouble(result.sampleAngleDeg);
    result.helixChordTolerance = object.value(QStringLiteral("helix_chord_tolerance")).toDouble(result.helixChordTolerance);
    result.minSampleAngleDeg = object.value(QStringLiteral("min_sample_angle_deg")).toDouble(result.minSampleAngleDeg);
    result.maxSampleAngleDeg = object.value(QStringLiteral("max_sample_angle_deg")).toDouble(result.maxSampleAngleDeg);
    result.maxSamplePoints = object.value(QStringLiteral("max_sample_points")).toInt(result.maxSamplePoints);
    return result;
}

QJsonObject parametersToJson(const toolpath::ToolpathParams& value)
{
    return {{QStringLiteral("tool"), toolDefToJson(value.tool)},
            {QStringLiteral("depth"), depthToJson(value.depth)},
            {QStringLiteral("clearing"), clearingToJson(value.clearing)},
            {QStringLiteral("profile"), profileToJson(value.profile)},
            {QStringLiteral("hole"), holeParamsToJson(value.hole)}};
}

toolpath::ToolpathParams parametersFromJson(const QJsonValue& value)
{
    const QJsonObject object = value.toObject();
    toolpath::ToolpathParams result;
    result.tool = toolDefFromJson(object.value(QStringLiteral("tool")));
    result.depth = depthFromJson(object.value(QStringLiteral("depth")));
    result.clearing = clearingFromJson(object.value(QStringLiteral("clearing")));
    result.profile = profileFromJson(object.value(QStringLiteral("profile")));
    result.hole = holeParamsFromJson(object.value(QStringLiteral("hole")));
    return result;
}

QJsonObject toolContextToJson(const toolpath::ToolpathToolContext& value)
{
    return {
        {QStringLiteral("valid"), value.valid},
        {QStringLiteral("tool_number"), value.toolNumber},
        {QStringLiteral("tool_id"), fromUtf8(value.toolId)},
        {QStringLiteral("tool_name"), fromUtf8(value.toolName)},
        {QStringLiteral("tool_type"), fromUtf8(value.toolType)},
        {QStringLiteral("diameter"), value.diameter},
        {QStringLiteral("corner_radius"), value.cornerRadius},
        {QStringLiteral("spindle_speed"), value.spindleSpeed},
        {QStringLiteral("horizontal_feed"), value.horizontalFeed},
        {QStringLiteral("vertical_feed"), value.verticalFeed},
        {QStringLiteral("horizontal_rapid"), value.horizontalRapid},
        {QStringLiteral("vertical_rapid"), value.verticalRapid},
        {QStringLiteral("cutting_edge_length"), value.cuttingEdgeLength},
        {QStringLiteral("overall_length"), value.overallLength},
        {QStringLiteral("shank_diameter"), value.shankDiameter},
        {QStringLiteral("holder_diameter"), value.holderDiameter},
        {QStringLiteral("holder_length"), value.holderLength},
        {QStringLiteral("spindle_envelope_diameter"),
         value.spindleEnvelopeDiameter},
        {QStringLiteral("spindle_envelope_length"),
         value.spindleEnvelopeLength},
        {QStringLiteral("tip_angle"), value.tipAngle},
        {QStringLiteral("tip_compensation"), value.tipCompensation},
        {QStringLiteral("peck_depth"), value.peckDepth},
        {QStringLiteral("drill_cycle"), static_cast<int>(value.drillCycle)},
        {QStringLiteral("coolant"), static_cast<int>(value.coolant)},
    };
}

toolpath::ToolpathToolContext toolContextFromJson(const QJsonValue& value)
{
    const QJsonObject object = value.toObject();
    toolpath::ToolpathToolContext result;
    result.valid = object.value(QStringLiteral("valid")).toBool();
    result.toolNumber = object.value(QStringLiteral("tool_number")).toInt();
    result.toolId = toUtf8(object.value(QStringLiteral("tool_id")));
    result.toolName = toUtf8(object.value(QStringLiteral("tool_name")));
    result.toolType = toUtf8(object.value(QStringLiteral("tool_type")));
    result.diameter = object.value(QStringLiteral("diameter")).toDouble();
    result.cornerRadius = object.value(QStringLiteral("corner_radius")).toDouble();
    result.spindleSpeed = object.value(QStringLiteral("spindle_speed")).toDouble();
    result.horizontalFeed = object.value(QStringLiteral("horizontal_feed")).toDouble();
    result.verticalFeed = object.value(QStringLiteral("vertical_feed")).toDouble();
    result.horizontalRapid = object.value(QStringLiteral("horizontal_rapid")).toDouble();
    result.verticalRapid = object.value(QStringLiteral("vertical_rapid")).toDouble();
    result.cuttingEdgeLength = object.value(QStringLiteral("cutting_edge_length")).toDouble();
    result.overallLength = object.value(QStringLiteral("overall_length")).toDouble();
    result.shankDiameter = object.value(QStringLiteral("shank_diameter")).toDouble();
    result.holderDiameter = object.value(QStringLiteral("holder_diameter")).toDouble();
    result.holderLength = object.value(QStringLiteral("holder_length")).toDouble();
    result.spindleEnvelopeDiameter = object.value(
        QStringLiteral("spindle_envelope_diameter")).toDouble();
    result.spindleEnvelopeLength = object.value(
        QStringLiteral("spindle_envelope_length")).toDouble();
    result.tipAngle = object.value(QStringLiteral("tip_angle")).toDouble();
    result.tipCompensation = object.value(QStringLiteral("tip_compensation")).toDouble();
    result.peckDepth = object.value(QStringLiteral("peck_depth")).toDouble();
    result.drillCycle = static_cast<toolpath::ToolpathDrillCyclePreference>(
        object.value(QStringLiteral("drill_cycle")).toInt());
    result.coolant = static_cast<toolpath::ToolpathCoolantMode>(
        object.value(QStringLiteral("coolant")).toInt());
    return result;
}

QJsonObject holeToJson(const toolpath::HoleMachiningInput& value)
{
    return {
        {QStringLiteral("kind"), static_cast<int>(value.kind)},
        {QStringLiteral("entry"), vectorToJson(toJobVector(value.entry))},
        {QStringLiteral("bottom"), vectorToJson(toJobVector(value.bottom))},
        {QStringLiteral("radius"), value.radius},
        {QStringLiteral("bottom_profile"), static_cast<int>(value.bottomProfile)},
        {QStringLiteral("entry_resolution"), static_cast<int>(value.entryResolution)},
        {QStringLiteral("setup_face"), static_cast<int>(value.setupFace)},
        {QStringLiteral("setup_face_entry_distance"), value.setupFaceEntryDistance},
        {QStringLiteral("setup_face_other_distance"), value.setupFaceOtherEndpointDistance},
        {QStringLiteral("setup_face_decision"), static_cast<int>(value.setupFaceDecision)},
        {QStringLiteral("preserve_entry_side"), value.preserveEntrySide},
        {QStringLiteral("manual"), value.manuallyConfirmed},
        {QStringLiteral("source_face_ids"), stringsToJson(value.sourceFaceIds)},
        {QStringLiteral("stage_wall_face_ids"),
         stringsToJson(value.stageWallFaceIds)},
        {QStringLiteral("stage_role"), static_cast<int>(value.stageRole)},
        {QStringLiteral("parent_relation"), static_cast<int>(value.parentRelation)},
        {QStringLiteral("parent_counterbore_face_id"), fromUtf8(value.parentCounterboreFaceId)},
        {QStringLiteral("parent_counterbore_floor_face_id"), fromUtf8(value.parentCounterboreFloorFaceId)},
        {QStringLiteral("parent_counterbore_radius"), value.parentCounterboreRadius},
    };
}

toolpath::HoleMachiningInput holeFromJson(const QJsonValue& value)
{
    const QJsonObject object = value.toObject();
    toolpath::HoleMachiningInput result;
    result.kind = static_cast<FeatureKind>(object.value(QStringLiteral("kind")).toInt());
    result.entry = toPoint(vectorFromJson(object.value(QStringLiteral("entry"))));
    result.bottom = toPoint(vectorFromJson(object.value(QStringLiteral("bottom"))));
    result.radius = object.value(QStringLiteral("radius")).toDouble();
    result.bottomProfile = static_cast<HoleBottomProfile>(
        object.value(QStringLiteral("bottom_profile")).toInt());
    result.entryResolution = static_cast<HoleEntryResolution>(
        object.value(QStringLiteral("entry_resolution")).toInt());
    result.setupFace = static_cast<MoldSetupFace>(
        object.value(QStringLiteral("setup_face")).toInt());
    result.setupFaceEntryDistance = object.value(QStringLiteral("setup_face_entry_distance")).toDouble(-1.0);
    result.setupFaceOtherEndpointDistance = object.value(QStringLiteral("setup_face_other_distance")).toDouble(-1.0);
    result.setupFaceDecision = static_cast<MoldSetupFaceDecision>(
        object.value(QStringLiteral("setup_face_decision")).toInt());
    result.preserveEntrySide = object.value(QStringLiteral("preserve_entry_side")).toBool();
    result.manuallyConfirmed = object.value(QStringLiteral("manual")).toBool();
    result.sourceFaceIds = stringsFromJson(object.value(QStringLiteral("source_face_ids")));
    result.stageWallFaceIds = stringsFromJson(
        object.value(QStringLiteral("stage_wall_face_ids")));
    result.stageRole = static_cast<HoleStageRole>(
        object.value(QStringLiteral("stage_role")).toInt(
            static_cast<int>(HoleStageRole::Standard)));
    result.parentRelation = static_cast<HoleParentRelation>(
        object.value(QStringLiteral("parent_relation")).toInt());
    result.parentCounterboreFaceId = toUtf8(object.value(QStringLiteral("parent_counterbore_face_id")));
    result.parentCounterboreFloorFaceId = toUtf8(object.value(QStringLiteral("parent_counterbore_floor_face_id")));
    result.parentCounterboreRadius = object.value(QStringLiteral("parent_counterbore_radius")).toDouble();
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

QJsonObject definitionToJson(const MachiningJobDefinition& value)
{
    // 把可序列化定义整体投影为 JSON；字段名构成文件格式的稳定契约。
    QJsonArray recognizedHoles;
    for (const toolpath::HoleMachiningInput& hole : value.recognizedHoles)
        recognizedHoles.append(holeToJson(hole));
    QJsonArray collisionModels;
    for (const CollisionModelReference& model : value.collisionScene.models) {
        collisionModels.append(QJsonObject{
            {QStringLiteral("source_path"), fromUtf8(model.sourcePath)},
            {QStringLiteral("source_fingerprint"),
             fromUtf8(model.sourceFingerprint)},
            {QStringLiteral("role"), static_cast<int>(model.role)},
            {QStringLiteral("translation"),
             vectorToJson(model.translation)},
            {QStringLiteral("rotation_deg"),
             vectorToJson(model.rotationDeg)},
        });
    }

    QJsonArray setups;
    for (const SetupDefinition& setup : value.setups) {
        setups.append(QJsonObject{
            {QStringLiteral("id"), fromUtf8(setup.id)},
            {QStringLiteral("setup_index"), setup.setupIndex},
            {QStringLiteral("face"), static_cast<int>(setup.face)},
            {QStringLiteral("work_offset"), static_cast<int>(setup.workOffset)},
            {QStringLiteral("origin"), vectorToJson(setup.origin)},
            {QStringLiteral("x_axis"), vectorToJson(setup.xAxis)},
            {QStringLiteral("y_axis"), vectorToJson(setup.yAxis)},

            {QStringLiteral("z_axis"), vectorToJson(setup.zAxis)},
            {QStringLiteral("valid"), setup.valid},
            {QStringLiteral("requires_physical_reclamp"), setup.requiresPhysicalReclamp},
            {QStringLiteral("axis_mapping"), fromUtf8(setup.axisMapping)},
            {QStringLiteral("operator_instruction"), fromUtf8(setup.operatorInstruction)},
            {QStringLiteral("confirmation"), QJsonObject{
                {QStringLiteral("confirmed"), setup.confirmation.confirmed},
                {QStringLiteral("confirmed_by"), fromUtf8(setup.confirmation.confirmedBy)},
                {QStringLiteral("confirmed_at"), fromUtf8(setup.confirmation.confirmedAt)},
                {QStringLiteral("input_fingerprint"), fromUtf8(setup.confirmation.confirmedInputFingerprint)},
            }},
        });
    }

    QJsonArray controllers;
    for (const JobToolController& controller : value.toolControllers) {
        controllers.append(QJsonObject{
            {QStringLiteral("id"), fromUtf8(controller.id)},
            {QStringLiteral("library_id"), fromUtf8(controller.libraryId)},
            {QStringLiteral("tool"), toolContextToJson(controller.tool)},
        });
    }

    QJsonArray holeOperations;
    for (const HoleOperationDefinition& operation : value.holeOperations) {
        QJsonArray targets;
        for (const HoleTargetReference& target : operation.targets) {
            QJsonArray coveredSourceIndices;
            for (const int sourceIndex : target.coveredSourceHoleIndices)
                coveredSourceIndices.append(sourceIndex);
            targets.append(QJsonObject{
                {QStringLiteral("feature_id"), fromUtf8(target.featureId)},
                {QStringLiteral("geometry_fingerprint"), fromUtf8(target.geometryFingerprint)},
                {QStringLiteral("source_hole_index"), target.sourceHoleIndex},
                {QStringLiteral("covered_source_hole_indices"),
                 coveredSourceIndices},
                {QStringLiteral("physical_hole_index"), target.physicalHoleIndex},
                {QStringLiteral("strategy"), static_cast<int>(target.strategy)},
                {QStringLiteral("hole"), holeToJson(target.hole)},
            });
        }
        holeOperations.append(QJsonObject{
            {QStringLiteral("id"), fromUtf8(operation.id)},
            {QStringLiteral("setup_id"), fromUtf8(operation.setupId)},
            {QStringLiteral("tool_controller_id"), fromUtf8(operation.toolControllerId)},
            {QStringLiteral("phase"), static_cast<int>(operation.phase)},
            {QStringLiteral("targets"), targets},
            {QStringLiteral("enabled"), operation.enabled},
            {QStringLiteral("sequence"), operation.sequence},
            {QStringLiteral("state"), static_cast<int>(operation.state)},
            {QStringLiteral("input_fingerprint"), fromUtf8(operation.inputFingerprint)},
        });
    }

    QJsonArray pocketOperations;
    for (const PocketOperationDefinition& operation :
         value.pocketOperations) {
        pocketOperations.append(QJsonObject{
            {QStringLiteral("id"), fromUtf8(operation.id)},
            {QStringLiteral("setup_id"), fromUtf8(operation.setupId)},
            {QStringLiteral("tool_controller_id"),
             fromUtf8(operation.toolControllerId)},
            {QStringLiteral("rest_previous_tool_diameter"),
             operation.restMachiningPreviousToolDiameter},
            {QStringLiteral("algorithm_id"),
             fromUtf8(operation.algorithmId)},
            {QStringLiteral("target"), QJsonObject{
                {QStringLiteral("source_kind"),
                 static_cast<int>(operation.target.sourceKind)},
                {QStringLiteral("kind"),
                 static_cast<int>(operation.target.kind)},
                {QStringLiteral("source_face_ids"),
                 stringsToJson(operation.target.sourceFaceIds)},
                {QStringLiteral("floor_face_id"),
                 fromUtf8(operation.target.floorFaceId)},
                {QStringLiteral("root_face_id"),
                 fromUtf8(operation.target.rootFaceId)},
                {QStringLiteral("root_wire_id"),
                 fromUtf8(operation.target.rootWireId)},
                {QStringLiteral("allow_open_section_contours"),
                 operation.target.allowOpenSectionContours},
                {QStringLiteral("open_boundary_face_count"),
                 operation.target.openBoundaryFaceCount},
                {QStringLiteral("geometry_fingerprint"),
                 fromUtf8(operation.target.geometryFingerprint)},
            }},
            {QStringLiteral("enabled"), operation.enabled},
            {QStringLiteral("sequence"), operation.sequence},
            {QStringLiteral("state"), static_cast<int>(operation.state)},
            {QStringLiteral("input_fingerprint"),
             fromUtf8(operation.inputFingerprint)},
        });
    }

    QJsonArray profileOperations;
    for (const ProfileOperationDefinition& operation :
         value.profileOperations) {
        profileOperations.append(QJsonObject{
            {QStringLiteral("id"), fromUtf8(operation.id)},
            {QStringLiteral("setup_id"), fromUtf8(operation.setupId)},
            {QStringLiteral("tool_controller_id"),
             fromUtf8(operation.toolControllerId)},
            {QStringLiteral("algorithm_id"),
             fromUtf8(operation.algorithmId)},
            {QStringLiteral("target"), QJsonObject{
                {QStringLiteral("axis_label"),
                 fromUtf8(operation.target.axisLabel)},
                {QStringLiteral("normal_direction"),
                 operation.target.normalDirection},
                {QStringLiteral("source_face_ids"),
                 stringsToJson(operation.target.sourceFaceIds)},
                {QStringLiteral("machining_face_ids"),
                 stringsToJson(operation.target.machiningFaceIds)},
                {QStringLiteral("terminal_face_ids"),
                 stringsToJson(operation.target.terminalFaceIds)},
                {QStringLiteral("main_outer_face_ids"),
                 stringsToJson(operation.target.mainOuterFaceIds)},
                {QStringLiteral("root_face_id"),
                 fromUtf8(operation.target.rootFaceId)},
                {QStringLiteral("root_wire_id"),
                 fromUtf8(operation.target.rootWireId)},
                {QStringLiteral("geometry_fingerprint"),
                 fromUtf8(operation.target.geometryFingerprint)},
            }},
            {QStringLiteral("rest_previous_tool_diameter"),
             operation.restMachiningPreviousToolDiameter},
            {QStringLiteral("finish_pass"), operation.finishPass},
            {QStringLiteral("enabled"), operation.enabled},
            {QStringLiteral("sequence"), operation.sequence},
            {QStringLiteral("state"), static_cast<int>(operation.state)},
            {QStringLiteral("input_fingerprint"),
             fromUtf8(operation.inputFingerprint)},
        });
    }

    return {
        {QStringLiteral("schema_version"), kMachiningJobSchemaVersion},
        {QStringLiteral("id"), fromUtf8(value.id)},
        {QStringLiteral("name"), fromUtf8(value.name)},
        {QStringLiteral("created_at"), fromUtf8(value.createdAt)},
        {QStringLiteral("modified_at"), fromUtf8(value.modifiedAt)},
        {QStringLiteral("part"), QJsonObject{
            {QStringLiteral("source_path"), fromUtf8(value.part.sourcePath)},
            {QStringLiteral("source_fingerprint"), fromUtf8(value.part.sourceFingerprint)},
            {QStringLiteral("units"), fromUtf8(value.part.units)},
            {QStringLiteral("recognition_profile_version"), fromUtf8(value.part.recognitionProfileVersion)},
        }},
        {QStringLiteral("stock"), QJsonObject{
            {QStringLiteral("source_kind"), static_cast<int>(value.stock.sourceKind)},
            {QStringLiteral("source_path"), fromUtf8(value.stock.sourcePath)},
            {QStringLiteral("source_fingerprint"), fromUtf8(value.stock.sourceFingerprint)},
            {QStringLiteral("margin_x"), value.stock.marginX},
            {QStringLiteral("margin_y"), value.stock.marginY},
            {QStringLiteral("margin_z"), value.stock.marginZ},
        }},
        {QStringLiteral("post"), QJsonObject{
            {QStringLiteral("id"), fromUtf8(value.post.id)},
            {QStringLiteral("version"), fromUtf8(value.post.version)},
            {QStringLiteral("program_name"), fromUtf8(value.post.options.programName)},
            {QStringLiteral("tool_id"), fromUtf8(value.post.options.toolId)},
            {QStringLiteral("tool_name"), fromUtf8(value.post.options.toolName)},
            {QStringLiteral("coolant"), value.post.options.coolant},
        }},
        {QStringLiteral("collision_scene"), QJsonObject{
            {QStringLiteral("confirmed"), value.collisionScene.confirmed},
            {QStringLiteral("models"), collisionModels},
        }},
        {QStringLiteral("active_tool_library_id"), fromUtf8(value.activeToolLibraryId)},
        {QStringLiteral("algorithm_id"), fromUtf8(value.algorithmId)},
        {QStringLiteral("hole_parameters"), parametersToJson(value.holeParameters)},
        {QStringLiteral("pocket_parameters"),
         parametersToJson(value.pocketParameters)},
        {QStringLiteral("profile_parameters"),
         parametersToJson(value.profileParameters)},
        {QStringLiteral("recognized_holes"), recognizedHoles},
        {QStringLiteral("recognized_physical_hole_count"),
         value.recognizedPhysicalHoleCount},
        {QStringLiteral("setups"), setups},
        {QStringLiteral("tool_controllers"), controllers},
        {QStringLiteral("hole_operations"), holeOperations},
        {QStringLiteral("pocket_operations"), pocketOperations},
        {QStringLiteral("profile_operations"), profileOperations},
        {QStringLiteral("surface_finishing_operations"), detail::finishingOperationsToJson(value.finishingOperations)},
    };
}

MachiningJobDefinition definitionFromJson(const QJsonObject& object)
{
    // 反方向还原定义；缺失的可选字段回退到默认值，结构错误由后续校验捕获。
    MachiningJobDefinition result;
    result.schemaVersion = object.value(QStringLiteral("schema_version")).toInt();
    result.id = toUtf8(object.value(QStringLiteral("id")));
    result.finishingOperations = detail::finishingOperationsFromJson(object.value(QStringLiteral("surface_finishing_operations")).toArray());
    result.name = toUtf8(object.value(QStringLiteral("name")));
    result.createdAt = toUtf8(object.value(QStringLiteral("created_at")));
    result.modifiedAt = toUtf8(object.value(QStringLiteral("modified_at")));

    const QJsonObject part = object.value(QStringLiteral("part")).toObject();
    result.part.sourcePath = toUtf8(part.value(QStringLiteral("source_path")));
    result.part.sourceFingerprint = toUtf8(part.value(QStringLiteral("source_fingerprint")));
    result.part.units = toUtf8(part.value(QStringLiteral("units")));
    result.part.recognitionProfileVersion = toUtf8(part.value(QStringLiteral("recognition_profile_version")));

    const QJsonObject stock = object.value(QStringLiteral("stock")).toObject();
    result.stock.sourceKind = static_cast<StockSourceKind>(stock.value(QStringLiteral("source_kind")).toInt());
    result.stock.sourcePath = toUtf8(stock.value(QStringLiteral("source_path")));
    result.stock.sourceFingerprint = toUtf8(stock.value(QStringLiteral("source_fingerprint")));
    result.stock.marginX = stock.value(QStringLiteral("margin_x")).toDouble();
    result.stock.marginY = stock.value(QStringLiteral("margin_y")).toDouble();
    result.stock.marginZ = stock.value(QStringLiteral("margin_z")).toDouble();

    const QJsonObject post = object.value(QStringLiteral("post")).toObject();
    result.post.id = toUtf8(post.value(QStringLiteral("id")));
    result.post.version = toUtf8(post.value(QStringLiteral("version")));
    result.post.options.programName = toUtf8(post.value(QStringLiteral("program_name")));
    result.post.options.toolId = toUtf8(post.value(QStringLiteral("tool_id")));
    result.post.options.toolName = toUtf8(post.value(QStringLiteral("tool_name")));
    result.post.options.coolant = post.value(QStringLiteral("coolant")).toBool();
    const QJsonObject collisionScene =
        object.value(QStringLiteral("collision_scene")).toObject();
    result.collisionScene.confirmed =
        collisionScene.value(QStringLiteral("confirmed")).toBool();
    const QJsonArray collisionModels =
        collisionScene.value(QStringLiteral("models")).toArray();
    result.collisionScene.models.reserve(
        static_cast<std::size_t>(collisionModels.size()));
    for (const QJsonValue& value : collisionModels) {
        const QJsonObject modelObject = value.toObject();
        CollisionModelReference model;
        model.sourcePath = toUtf8(
            modelObject.value(QStringLiteral("source_path")));
        model.sourceFingerprint = toUtf8(
            modelObject.value(QStringLiteral("source_fingerprint")));
        model.role = static_cast<toolpath::CollisionBodyRole>(
            modelObject.value(QStringLiteral("role")).toInt());
        model.translation = vectorFromJson(
            modelObject.value(QStringLiteral("translation")));
        model.rotationDeg = vectorFromJson(
            modelObject.value(QStringLiteral("rotation_deg")));
        result.collisionScene.models.push_back(std::move(model));
    }

    result.activeToolLibraryId = toUtf8(object.value(QStringLiteral("active_tool_library_id")));
    result.algorithmId = toUtf8(object.value(QStringLiteral("algorithm_id")));
    result.holeParameters = parametersFromJson(object.value(QStringLiteral("hole_parameters")));
    if (object.contains(QStringLiteral("pocket_parameters"))) {
        result.pocketParameters = parametersFromJson(
            object.value(QStringLiteral("pocket_parameters")));
    } else {
        result.pocketParameters = result.holeParameters;
    }
    if (object.contains(QStringLiteral("profile_parameters"))) {
        result.profileParameters = parametersFromJson(
            object.value(QStringLiteral("profile_parameters")));
    } else {
        result.profileParameters = result.pocketParameters;
    }
    const QJsonArray recognizedHoles =
        object.value(QStringLiteral("recognized_holes")).toArray();
    result.recognizedHoles.reserve(
        static_cast<std::size_t>(recognizedHoles.size()));
    for (const QJsonValue& value : recognizedHoles)
        result.recognizedHoles.push_back(holeFromJson(value));
    result.recognizedPhysicalHoleCount = object.value(
        QStringLiteral("recognized_physical_hole_count")).toInt();

    const QJsonArray setups = object.value(QStringLiteral("setups")).toArray();
    result.setups.reserve(static_cast<std::size_t>(setups.size()));
    for (const QJsonValue& value : setups) {
        const QJsonObject item = value.toObject();
        SetupDefinition setup;
        setup.id = toUtf8(item.value(QStringLiteral("id")));
        setup.setupIndex = item.value(QStringLiteral("setup_index")).toInt();
        setup.face = static_cast<MoldSetupFace>(item.value(QStringLiteral("face")).toInt());
        setup.workOffset = static_cast<toolpath::WorkCoordinateSystem>(item.value(QStringLiteral("work_offset")).toInt());
        setup.origin = vectorFromJson(item.value(QStringLiteral("origin")));
        setup.xAxis = vectorFromJson(item.value(QStringLiteral("x_axis")));
        setup.yAxis = vectorFromJson(item.value(QStringLiteral("y_axis")));
        setup.zAxis = vectorFromJson(item.value(QStringLiteral("z_axis")));
        setup.valid = item.value(QStringLiteral("valid")).toBool();
        setup.requiresPhysicalReclamp = item.value(QStringLiteral("requires_physical_reclamp")).toBool();
        setup.axisMapping = toUtf8(item.value(QStringLiteral("axis_mapping")));
        setup.operatorInstruction = toUtf8(item.value(QStringLiteral("operator_instruction")));
        const QJsonObject confirmation = item.value(QStringLiteral("confirmation")).toObject();
        setup.confirmation.confirmed = confirmation.value(QStringLiteral("confirmed")).toBool();
        setup.confirmation.confirmedBy = toUtf8(confirmation.value(QStringLiteral("confirmed_by")));
        setup.confirmation.confirmedAt = toUtf8(confirmation.value(QStringLiteral("confirmed_at")));
        setup.confirmation.confirmedInputFingerprint = toUtf8(confirmation.value(QStringLiteral("input_fingerprint")));
        result.setups.push_back(std::move(setup));
    }

    const QJsonArray controllers = object.value(QStringLiteral("tool_controllers")).toArray();
    result.toolControllers.reserve(static_cast<std::size_t>(controllers.size()));
    for (const QJsonValue& value : controllers) {
        const QJsonObject item = value.toObject();
        JobToolController controller;
        controller.id = toUtf8(item.value(QStringLiteral("id")));
        controller.libraryId = toUtf8(item.value(QStringLiteral("library_id")));
        controller.tool = toolContextFromJson(item.value(QStringLiteral("tool")));
        result.toolControllers.push_back(std::move(controller));
    }

    const QJsonArray operations = object.value(QStringLiteral("hole_operations")).toArray();
    result.holeOperations.reserve(static_cast<std::size_t>(operations.size()));
    for (const QJsonValue& value : operations) {
        const QJsonObject item = value.toObject();
        HoleOperationDefinition operation;
        operation.id = toUtf8(item.value(QStringLiteral("id")));
        operation.setupId = toUtf8(item.value(QStringLiteral("setup_id")));
        operation.toolControllerId = toUtf8(item.value(QStringLiteral("tool_controller_id")));
        operation.phase = static_cast<toolpath::HoleMachiningPhase>(item.value(QStringLiteral("phase")).toInt());
        operation.enabled = item.value(QStringLiteral("enabled")).toBool(true);
        operation.sequence = item.value(QStringLiteral("sequence")).toInt();
        operation.state = static_cast<JobOperationState>(item.value(QStringLiteral("state")).toInt());
        operation.inputFingerprint = toUtf8(item.value(QStringLiteral("input_fingerprint")));
        const QJsonArray targets = item.value(QStringLiteral("targets")).toArray();
        operation.targets.reserve(static_cast<std::size_t>(targets.size()));
        for (const QJsonValue& targetValue : targets) {
            const QJsonObject targetObject = targetValue.toObject();
            HoleTargetReference target;
            target.featureId = toUtf8(targetObject.value(QStringLiteral("feature_id")));
            target.geometryFingerprint = toUtf8(targetObject.value(QStringLiteral("geometry_fingerprint")));
            target.sourceHoleIndex = targetObject.value(QStringLiteral("source_hole_index")).toInt(-1);
            const QJsonArray coveredSourceIndices = targetObject.value(
                QStringLiteral("covered_source_hole_indices")).toArray();
            target.coveredSourceHoleIndices.reserve(
                static_cast<std::size_t>(coveredSourceIndices.size()));
            for (const QJsonValue& sourceIndex : coveredSourceIndices)
                target.coveredSourceHoleIndices.push_back(sourceIndex.toInt(-1));
            target.physicalHoleIndex = targetObject.value(QStringLiteral("physical_hole_index")).toInt(-1);
            target.strategy = static_cast<toolpath::HoleMachiningStrategy>(targetObject.value(QStringLiteral("strategy")).toInt());
            target.hole = holeFromJson(targetObject.value(QStringLiteral("hole")));
            operation.targets.push_back(std::move(target));
        }
        result.holeOperations.push_back(std::move(operation));
    }
    const QJsonArray pocketOperations =
        object.value(QStringLiteral("pocket_operations")).toArray();
    result.pocketOperations.reserve(
        static_cast<std::size_t>(pocketOperations.size()));
    for (const QJsonValue& value : pocketOperations) {
        const QJsonObject item = value.toObject();
        PocketOperationDefinition operation;
        operation.id = toUtf8(item.value(QStringLiteral("id")));
        operation.setupId = toUtf8(item.value(QStringLiteral("setup_id")));
        operation.toolControllerId = toUtf8(
            item.value(QStringLiteral("tool_controller_id")));
        operation.restMachiningPreviousToolDiameter =
            item.value(QStringLiteral("rest_previous_tool_diameter"))
                .toDouble(0.0);
        operation.algorithmId =
            toUtf8(item.value(QStringLiteral("algorithm_id")));
        const QJsonObject target =
            item.value(QStringLiteral("target")).toObject();
        operation.target.sourceKind =
            static_cast<PocketRegionSourceKind>(
                target.value(QStringLiteral("source_kind")).toInt(
                    static_cast<int>(
                        PocketRegionSourceKind::FeatureFaces)));
        operation.target.kind = static_cast<FeatureKind>(
            target.value(QStringLiteral("kind")).toInt(
                static_cast<int>(FeatureKind::Pocket)));
        operation.target.sourceFaceIds = stringsFromJson(
            target.value(QStringLiteral("source_face_ids")));
        operation.target.floorFaceId =
            toUtf8(target.value(QStringLiteral("floor_face_id")));
        operation.target.rootFaceId =
            toUtf8(target.value(QStringLiteral("root_face_id")));
        operation.target.rootWireId =
            toUtf8(target.value(QStringLiteral("root_wire_id")));
        operation.target.allowOpenSectionContours = target.value(
            QStringLiteral("allow_open_section_contours")).toBool();
        operation.target.openBoundaryFaceCount = target.value(
            QStringLiteral("open_boundary_face_count")).toInt();
        operation.target.geometryFingerprint = toUtf8(
            target.value(QStringLiteral("geometry_fingerprint")));
        operation.enabled =
            item.value(QStringLiteral("enabled")).toBool(true);
        operation.sequence =
            item.value(QStringLiteral("sequence")).toInt();
        operation.state = static_cast<JobOperationState>(
            item.value(QStringLiteral("state")).toInt());
        operation.inputFingerprint = toUtf8(
            item.value(QStringLiteral("input_fingerprint")));
        result.pocketOperations.push_back(std::move(operation));
    }
    const QJsonArray profileOperations =
        object.value(QStringLiteral("profile_operations")).toArray();
    result.profileOperations.reserve(
        static_cast<std::size_t>(profileOperations.size()));
    for (const QJsonValue& value : profileOperations) {
        const QJsonObject item = value.toObject();
        ProfileOperationDefinition operation;
        operation.id = toUtf8(item.value(QStringLiteral("id")));
        operation.setupId = toUtf8(item.value(QStringLiteral("setup_id")));
        operation.toolControllerId = toUtf8(
            item.value(QStringLiteral("tool_controller_id")));
        operation.algorithmId = toUtf8(
            item.value(QStringLiteral("algorithm_id")));
        const QJsonObject target =
            item.value(QStringLiteral("target")).toObject();
        operation.target.axisLabel = toUtf8(
            target.value(QStringLiteral("axis_label")));
        operation.target.normalDirection = target.value(
            QStringLiteral("normal_direction")).toInt(1);
        operation.target.sourceFaceIds = stringsFromJson(
            target.value(QStringLiteral("source_face_ids")));
        operation.target.machiningFaceIds = stringsFromJson(
            target.value(QStringLiteral("machining_face_ids")));
        operation.target.terminalFaceIds = stringsFromJson(
            target.value(QStringLiteral("terminal_face_ids")));
        operation.target.mainOuterFaceIds = stringsFromJson(
            target.value(QStringLiteral("main_outer_face_ids")));
        operation.target.rootFaceId = toUtf8(
            target.value(QStringLiteral("root_face_id")));
        operation.target.rootWireId = toUtf8(
            target.value(QStringLiteral("root_wire_id")));
        operation.target.geometryFingerprint = toUtf8(
            target.value(QStringLiteral("geometry_fingerprint")));
        operation.restMachiningPreviousToolDiameter = item.value(
            QStringLiteral("rest_previous_tool_diameter")).toDouble(0.0);
        operation.finishPass = item.value(
            QStringLiteral("finish_pass")).toBool(true);
        operation.enabled =
            item.value(QStringLiteral("enabled")).toBool(true);
        operation.sequence =
            item.value(QStringLiteral("sequence")).toInt();
        operation.state = static_cast<JobOperationState>(
            item.value(QStringLiteral("state")).toInt());
        operation.inputFingerprint = toUtf8(
            item.value(QStringLiteral("input_fingerprint")));
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

bool parseDocument(const QByteArray& data,
                   QJsonObject& root,
                   QString* error)
{
    // 统一入口：体积上限 → JSON 语法 → 格式标识 → schema 版本区间 → job 节点。
    if (data.size() > kMaxJobBytes) {
        if (error) *error = QStringLiteral("加工项目超过 %1 MB 上限")
            .arg(kMaxJobBytes / (1024 * 1024));
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
        if (error) *error = QStringLiteral("加工项目 JSON 无效：%1")
            .arg(parseError.errorString());
        return false;
    }
    root = document.object();
    // 旧项目可显式打开；重新保存统一写入 OAP 格式标识。
    const QString format = root.value(QStringLiteral("format")).toString();
    if (format != QString::fromLatin1(kFormatId) &&
        format != QStringLiteral("faceface-machining-job")) {
        if (error) *error = QStringLiteral("不是 OpenAutoPath 加工项目文件");
        return false;
    }
    const int version = root.value(QStringLiteral("schema_version")).toInt();
    if (version < kMinimumSupportedMachiningJobSchemaVersion ||
        version > kMachiningJobSchemaVersion) {
        if (error) *error = QStringLiteral("不支持的加工项目版本：%1").arg(version);
        return false;
    }
    const QJsonValue jobValue = root.value(QStringLiteral("job"));
    if (!jobValue.isObject()) {
        if (error) *error = QStringLiteral("加工项目缺少 job 定义");
        return false;
    }
    const QJsonValue legacySurfaces = jobValue.toObject()
        .value(QStringLiteral("surface_operations"));
    if (!legacySurfaces.isUndefined() &&
        (!legacySurfaces.isArray() || !legacySurfaces.toArray().isEmpty())) {
        if (error) *error = QStringLiteral(
            "旧版曲面加工模块已移除，其工序与 OpenCAMLib 精加工定义不兼容，无法自动迁移。请使用旧版处理这些工序。");
        return false;
    }
    const int definitionVersion = jobValue.toObject()
        .value(QStringLiteral("schema_version")).toInt();
    if (definitionVersion != version) {
        if (error) {
            *error = QStringLiteral(
                "加工项目根版本与 job 定义版本不一致");
        }
        return false;
    }
    return true;
}

QString retiredManualInputError(const QJsonObject& job)
{
    const auto plan = job.value(QStringLiteral("manual_hole_plan"));
    if (!plan.isUndefined()) {
        if (!plan.isObject() || !plan.toObject().value(QStringLiteral("stages")).isArray() ||
            !plan.toObject().value(QStringLiteral("stages")).toArray().isEmpty())
            return QStringLiteral("项目包含已移除的人工孔段计划或无效人工计划；请使用旧版本处理原项目。");
    }
    for (const auto& value : job.value(QStringLiteral("hole_operations")).toArray()) {
        const auto operation = value.toObject();
        const auto overrides = operation.value(QStringLiteral("operation_overrides"));
        const auto local = operation.value(QStringLiteral("local_batch"));
        const auto overrideObject = overrides.toObject();
        const auto retract = overrideObject.value(QStringLiteral("retract_preference"));
        // V4-V10 wrote the inherited retract preference even without any override.
        const bool inheritedOnly = overrideObject.size() == 1 && retract.isDouble() && retract.toDouble() == 0.0;
        if ((!overrides.isUndefined() && (!overrides.isObject() || (!overrideObject.isEmpty() && !inheritedOnly))) ||
            (!local.isUndefined() && (!local.isBool() || local.toBool())))
            return QStringLiteral("项目包含已移除的手动孔工艺覆盖或局部人工计划；请使用旧版本处理原项目。");
    }
    for (const auto& value : job.value(QStringLiteral("tool_controllers")).toArray()) {
        const auto controller = value.toObject();
        const auto manual = controller.value(QStringLiteral("manual"));
        if (!manual.isUndefined() && (!manual.isBool() || manual.toBool()))
            return QStringLiteral("项目包含已移除的手工刀具；请使用旧版本将工序绑定到刀库后再迁移。");
    }
    return {};
}

QString validateCurrentShape(const QJsonObject& job)
{
    if (!job.value(QStringLiteral("stock")).isObject() ||
        !job.value(QStringLiteral("stock")).toObject().value(QStringLiteral("source_fingerprint")).isString())
        return QStringLiteral("V12 planning stock content identity must be a string");
    // 当前 schema 的快速结构预检（不含语义校验）。
    const QJsonValue recognizedValue =
        job.value(QStringLiteral("recognized_holes"));
    if (!recognizedValue.isArray())
        return QStringLiteral("V4 recognized_holes is not an array");
    for (const QJsonValue& value : recognizedValue.toArray()) {
        const QJsonObject hole = value.toObject();
        if (!value.isObject() ||
            !hole.value(QStringLiteral("manual")).isBool() ||
            !hole.value(QStringLiteral("stage_wall_face_ids")).isArray() ||
            !hole.value(QStringLiteral("stage_role")).isDouble()) {
            return QStringLiteral("V4 recognized hole fields are incomplete");
        }
    }
    const QJsonValue operationsValue =
        job.value(QStringLiteral("hole_operations"));
    if (!operationsValue.isArray())
        return QStringLiteral("V4 hole_operations is not an array");
    for (const QJsonValue& value : operationsValue.toArray()) {
        const QJsonObject operation = value.toObject();
        if (!value.isObject()) {
            return QStringLiteral("V4 hole operation fields are incomplete");
        }
    }
    if (!job.value(QStringLiteral("profile_parameters")).isObject() ||
        !job.value(QStringLiteral("profile_operations")).isArray()) {
        return QStringLiteral("V5 profile fields are incomplete");
    }
    for (const QJsonValue& value :
         job.value(QStringLiteral("profile_operations")).toArray()) {
        const QJsonObject operation = value.toObject();
        if (!value.isObject() ||
            !operation.value(
                QStringLiteral("rest_previous_tool_diameter")).isDouble() ||
            !operation.value(QStringLiteral("finish_pass")).isBool()) {
            return QStringLiteral(
                "V6 profile rest-machining fields are incomplete");
        }
    }
    return {};
}

QJsonArray executionOperations(QJsonArray operations)
{
    // 归一化操作数组用于执行指纹：去掉运行期 state，并按 sequence+id 排序，
    // 使声明顺序等非执行因素不影响指纹。
    std::vector<QJsonObject> normalized;
    normalized.reserve(static_cast<std::size_t>(operations.size()));
    for (qsizetype index = 0; index < operations.size(); ++index) {
        QJsonObject operation = operations.at(index).toObject();
        operation.remove(QStringLiteral("state"));
        normalized.push_back(std::move(operation));
    }
    std::sort(
        normalized.begin(), normalized.end(),
        [](const QJsonObject& left, const QJsonObject& right) {
            const int leftSequence =
                left.value(QStringLiteral("sequence")).toInt();
            const int rightSequence =
                right.value(QStringLiteral("sequence")).toInt();
            if (leftSequence != rightSequence)
                return leftSequence < rightSequence;
            return left.value(QStringLiteral("id")).toString() <
                right.value(QStringLiteral("id")).toString();
        });
    QJsonArray result;
    for (const QJsonObject& operation : normalized)
        result.append(operation);
    return result;
}

QJsonArray referencedResources(
    const QJsonArray& resources,
    const QJsonArray& operations,
    const QString& referenceKey)
{
    // 只保留执行指纹真正依赖的资源；Setup 还剔除确认/指令等非执行字段。
    QSet<QString> referenced;
    for (const QJsonValue& value : operations) {
        referenced.insert(
            value.toObject().value(referenceKey).toString());
    }
    std::vector<QJsonObject> normalized;
    for (const QJsonValue& value : resources) {
        QJsonObject item = value.toObject();
        if (!referenced.contains(item.value(QStringLiteral("id")).toString()))
            continue;
        if (referenceKey == QStringLiteral("setup_id")) {
            item.remove(QStringLiteral("confirmation"));
            item.remove(QStringLiteral("operator_instruction"));
        }
        normalized.push_back(std::move(item));
    }
    std::sort(
        normalized.begin(), normalized.end(),
        [](const QJsonObject& left, const QJsonObject& right) {
            return left.value(QStringLiteral("id")).toString() <
                right.value(QStringLiteral("id")).toString();
        });
    QJsonArray result;
    for (const QJsonObject& item : normalized)
        result.append(item);
    return result;
}



enum class ExecutionDomain {
    Hole,
    Pocket,
    Profile,
    SurfaceFinishing,
};

QJsonObject domainExecutionJson(
    const MachiningJobDefinition& definition, ExecutionDomain domain)
{
    // 按领域（孔/槽腔）裁剪定义生成执行 JSON，作为各自执行指纹的输入；
    // 移除 id/名称/时间等元数据与另一领域的参数和工序。
    QJsonObject result = definitionToJson(definition);
    result.remove(QStringLiteral("id"));
    result.remove(QStringLiteral("name"));
    result.remove(QStringLiteral("created_at"));
    result.remove(QStringLiteral("modified_at"));
    result.remove(QStringLiteral("post"));
    result.remove(QStringLiteral("active_tool_library_id"));

    const QString operationsKey =
        domain == ExecutionDomain::Hole
            ? QStringLiteral("hole_operations")
            : domain == ExecutionDomain::Pocket
                ? QStringLiteral("pocket_operations")
                : domain == ExecutionDomain::Profile
                    ? QStringLiteral("profile_operations") : QStringLiteral("surface_finishing_operations");
    const QJsonArray operations = executionOperations(
        result.value(operationsKey).toArray());
    result.insert(operationsKey, operations);
    result.insert(
        QStringLiteral("setups"),
        referencedResources(
            result.value(QStringLiteral("setups")).toArray(),
            operations, QStringLiteral("setup_id")));
    result.insert(
        QStringLiteral("tool_controllers"),
        referencedResources(
            result.value(QStringLiteral("tool_controllers")).toArray(),
            operations, QStringLiteral("tool_controller_id")));
    if (domain != ExecutionDomain::SurfaceFinishing)
        result.remove(QStringLiteral("surface_finishing_operations"));
    if (domain == ExecutionDomain::SurfaceFinishing) {
        for (const char* key : {"algorithm_id", "hole_parameters", "recognized_holes", "recognized_physical_hole_count",
             "hole_operations", "manual_hole_plan", "pocket_parameters", "pocket_operations", "profile_parameters", "profile_operations"})
            result.remove(QLatin1String(key));
    }
    if (domain == ExecutionDomain::Hole) {
        result.remove(QStringLiteral("pocket_parameters"));
        result.remove(QStringLiteral("pocket_operations"));
        result.remove(QStringLiteral("profile_parameters"));
        result.remove(QStringLiteral("profile_operations"));
    } else if (domain == ExecutionDomain::Pocket) {
        result.remove(QStringLiteral("algorithm_id"));
        result.remove(QStringLiteral("hole_parameters"));
        result.remove(QStringLiteral("recognized_holes"));
        result.remove(QStringLiteral("recognized_physical_hole_count"));
        result.remove(QStringLiteral("hole_operations"));
        result.remove(QStringLiteral("manual_hole_plan"));
        result.remove(QStringLiteral("profile_parameters"));
        result.remove(QStringLiteral("profile_operations"));
    } else if (domain == ExecutionDomain::Profile) {
        result.remove(QStringLiteral("algorithm_id"));
        result.remove(QStringLiteral("hole_parameters"));
        result.remove(QStringLiteral("recognized_holes"));
        result.remove(QStringLiteral("recognized_physical_hole_count"));
        result.remove(QStringLiteral("hole_operations"));
        result.remove(QStringLiteral("manual_hole_plan"));
        result.remove(QStringLiteral("pocket_parameters"));
        result.remove(QStringLiteral("pocket_operations"));
    }
    return result;
}

QString jsonFingerprint(const QJsonObject& value)
{
    return QString::fromLatin1(QCryptographicHash::hash(
        QJsonDocument(value).toJson(QJsonDocument::Compact),
        QCryptographicHash::Sha256).toHex());
}

} // namespace

QByteArray MachiningJobStorage::serializeDefinition(
    const MachiningJobDefinition& definition)
{
    // 输出 {format, schema_version, job} 根结构，job 内嵌定义 JSON；
    // 定义指纹直接以此完整序列化结果为输入。
    const QJsonObject root{
        {QStringLiteral("format"), QString::fromLatin1(kFormatId)},
        {QStringLiteral("schema_version"), kMachiningJobSchemaVersion},
        {QStringLiteral("job"), definitionToJson(definition)},
    };
    return QJsonDocument(root).toJson(QJsonDocument::Indented);
}

bool MachiningJobStorage::deserializeDefinition(
    const QByteArray& data,
    MachiningJobDefinition& definition,
    QString* error)
{
    // 加载流水线：解析 → 当前结构预检 → 反序列化 → 旧版本迁移 →
    // Setup 确认失效校验 → 领域校验，全部通过后才提交到调用方定义。
    QJsonObject root;
    if (!parseDocument(data, root, error))
        return false;
    const QJsonObject jobObject =
        root.value(QStringLiteral("job")).toObject();
    const int sourceVersion =
        root.value(QStringLiteral("schema_version")).toInt();
    const QString manualError = retiredManualInputError(jobObject);
    if (!manualError.isEmpty()) {
        if (error) *error = manualError;
        return false;
    }
    const QString finishingShapeError = detail::validateFinishingJson(jobObject, sourceVersion >= 10);
    if (!finishingShapeError.isEmpty()) {
        if (error) *error = finishingShapeError;
        return false;
    }
    if (sourceVersion == kMachiningJobSchemaVersion) {
        const QString shapeError = validateCurrentShape(jobObject);
        if (!shapeError.isEmpty()) {
            if (error)
                *error = QStringLiteral("加工项目结构无效：%1")
                    .arg(shapeError);
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
            parsed.part, setup).toStdString();
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
        if (error) *error = QStringLiteral("加工项目结构无效：%1")
            .arg(QString::fromStdString(errors.front()));
        return false;
    }
    definition = std::move(parsed);
    return true;
}

bool MachiningJobStorage::save(
    const QString& filePath,
    const MachiningJobDefinition& definition,
    QString* error)
{
    // 用 QSaveFile 先写临时文件再原子提交，避免中途失败损坏既有项目。
    QSaveFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        if (error) *error = file.errorString();
        return false;
    }
    const QByteArray data = serializeDefinition(definition);
    if (file.write(data) != data.size() || !file.commit()) {
        if (error) *error = file.errorString();
        return false;
    }
    return true;
}

bool MachiningJobStorage::load(
    const QString& filePath,
    MachiningJobDefinition& definition,
    QString* error)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) *error = file.errorString();
        return false;
    }
    if (file.size() < 0 || file.size() > kMaxJobBytes) {
        if (error) *error = QStringLiteral("加工项目文件大小无效或超过上限");
        return false;
    }
    return deserializeDefinition(file.readAll(), definition, error);
}

QString MachiningJobStorage::definitionFingerprint(
    const MachiningJobDefinition& definition)
{
    // 整个定义（含未启用内容）的规范化序列化哈希，任何字段变化都会改变它。
    return QString::fromLatin1(QCryptographicHash::hash(
        serializeDefinition(definition), QCryptographicHash::Sha256).toHex());
}

QString MachiningJobStorage::holeExecutionFingerprint(
    const MachiningJobDefinition& definition)
{
    // 孔执行指纹基于孔领域执行 JSON；其他工序族不参与其身份。
    return jsonFingerprint(domainExecutionJson(
        definition, ExecutionDomain::Hole));
}

QString MachiningJobStorage::pocketExecutionFingerprint(
    const MachiningJobDefinition& definition)
{
    // 槽腔执行指纹基于槽腔领域执行 JSON；孔相关字段不参与其身份。
    return jsonFingerprint(domainExecutionJson(
        definition, ExecutionDomain::Pocket));
}

QString MachiningJobStorage::profileExecutionFingerprint(
    const MachiningJobDefinition& definition)
{
    return jsonFingerprint(domainExecutionJson(
        definition, ExecutionDomain::Profile));
}

QString MachiningJobStorage::finishingExecutionFingerprint(const MachiningJobDefinition& definition)
{
    return jsonFingerprint(domainExecutionJson(definition, ExecutionDomain::SurfaceFinishing));
}

QString MachiningJobStorage::setupInputFingerprint(
    const PartDefinition& part,
    const SetupDefinition& setup)
{
    // 零件指纹 + 装夹关键值构成 Setup 的输入身份，用于确认失效与操作指纹。
    const QJsonObject input{
        {QStringLiteral("part_fingerprint"),
         fromUtf8(part.sourceFingerprint)},
        {QStringLiteral("setup_id"), fromUtf8(setup.id)},
        {QStringLiteral("setup_index"), setup.setupIndex},
        {QStringLiteral("face"), static_cast<int>(setup.face)},
        {QStringLiteral("work_offset"),
         static_cast<int>(setup.workOffset)},
        {QStringLiteral("origin"), vectorToJson(setup.origin)},
        {QStringLiteral("x_axis"), vectorToJson(setup.xAxis)},
        {QStringLiteral("y_axis"), vectorToJson(setup.yAxis)},
        {QStringLiteral("z_axis"), vectorToJson(setup.zAxis)},
        {QStringLiteral("axis_mapping"), fromUtf8(setup.axisMapping)},
    };
    return QString::fromLatin1(QCryptographicHash::hash(
        QJsonDocument(input).toJson(QJsonDocument::Compact),
        QCryptographicHash::Sha256).toHex());
}

QString MachiningJobStorage::sourceFileFingerprint(
    const QString& filePath,
    QString* error)
{
    // 对源文件做分块流式 SHA-256，避免一次性读入大模型文件。
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) *error = file.errorString();
        return {};
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    while (!file.atEnd()) {
        const QByteArray chunk = file.read(1024 * 1024);
        if (chunk.isEmpty() && file.error() != QFileDevice::NoError) {
            if (error) *error = file.errorString();
            return {};
        }
        hash.addData(chunk);
    }
    return QString::fromLatin1(hash.result().toHex());
}

} // namespace oap::manufacturing

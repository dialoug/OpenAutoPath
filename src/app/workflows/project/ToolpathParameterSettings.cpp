// ToolpathParameterSettings 实现（oap_app_workflow_cam）。
// 当前刀路工艺参数的桌面端持久化，见 ToolpathParameterSettings.h。
#include "app/workflows/project/ToolpathParameterSettings.h"

#include <cmath>

#include <QMetaType>
#include <QSettings>
#include <QStringList>
#include <QVariant>

namespace oap::app {
namespace {

constexpr int kSchemaVersion = 5;
constexpr int kLegacySchemaVersion = 1;
constexpr int kCombinedPocketProfileSchemaVersion = 2;
constexpr int kSeparateProfileSchemaVersion = 3;
constexpr int kProfileToolSelectionSchemaVersion = 4;
constexpr auto kSettingsGroup = "toolpath/current_parameters";
constexpr auto kHoleParametersGroup = "hole_parameters";
constexpr auto kPocketParametersGroup = "pocket_parameters";
constexpr auto kProfileParametersGroup = "profile_parameters";
constexpr auto kOrganization = "OAP";
constexpr auto kApplication = "OpenAutoPath";

// 把 QSettings 状态码映射为面向用户的中文提示。
QString settingsStatusText(QSettings::Status status)
{
    switch (status) {
        case QSettings::NoError:
            return {};
        case QSettings::AccessError:
            return QStringLiteral("没有权限读写本地刀路参数配置");
        case QSettings::FormatError:
            return QStringLiteral("本地刀路参数配置格式无效");
    }
    return QStringLiteral("本地刀路参数配置发生未知错误");
}

// 以下 read* 均为字段级读取:键缺失时静默保留调用方现值,
// 值不合法(类型错误/越界/非有限数)时记入 invalidKeys 供上层汇总。
void readDouble(QSettings& settings, const QString& key, double& target,
                QStringList& invalidKeys)
{
    if (!settings.contains(key))
        return;
    bool ok = false;
    const double value = settings.value(key).toDouble(&ok);
    if (!ok || !std::isfinite(value)) {
        invalidKeys.push_back(key);
        return;
    }
    target = value;
}

void readInt(QSettings& settings, const QString& key, int& target,
             QStringList& invalidKeys)
{
    if (!settings.contains(key))
        return;
    bool ok = false;
    const int value = settings.value(key).toInt(&ok);
    if (!ok) {
        invalidKeys.push_back(key);
        return;
    }
    target = value;
}

void readBool(QSettings& settings, const QString& key, bool& target,
              QStringList& invalidKeys)
{
    if (!settings.contains(key))
        return;
    const QVariant value = settings.value(key);
    if (value.metaType().id() == QMetaType::Bool) {
        target = value.toBool();
        return;
    }
    // 兼容旧版本以字符串 "true"/"1"/"false"/"0" 存储的布尔值。
    const QString text = value.toString().trimmed().toLower();
    if (text == QStringLiteral("true") || text == QStringLiteral("1")) {
        target = true;
        return;
    }
    if (text == QStringLiteral("false") || text == QStringLiteral("0")) {
        target = false;
        return;
    }
    invalidKeys.push_back(key);
}

template <typename Enum>
void readEnum(QSettings& settings, const QString& key, Enum& target,
              int minimum, int maximum, QStringList& invalidKeys)
{
    if (!settings.contains(key))
        return;
    bool ok = false;
    const int value = settings.value(key).toInt(&ok);
    if (!ok || value < minimum || value > maximum) {
        invalidKeys.push_back(key);
        return;
    }
    target = static_cast<Enum>(value);
}

// 读写辅助按工具/深度/清角/孔四个命名空间分组组织,
// 键名与 .oapjob 中的字段一一对应,保证跨版本迁移可追溯。
void writeTool(QSettings& settings, const oap::toolpath::ToolDef& tool)
{
    settings.setValue(QStringLiteral("tool/diameter"), tool.diameter);
    settings.setValue(QStringLiteral("tool/corner_radius"), tool.cornerRadius);
    settings.setValue(QStringLiteral("tool/horizontal_feed"), tool.horizFeed);
    settings.setValue(QStringLiteral("tool/vertical_feed"), tool.vertFeed);
    settings.setValue(QStringLiteral("tool/horizontal_rapid"), tool.horizRapid);
    settings.setValue(QStringLiteral("tool/vertical_rapid"), tool.vertRapid);
    settings.setValue(QStringLiteral("tool/spindle_speed"), tool.spindleSpeed);
    settings.setValue(QStringLiteral("tool/cutting_edge_length"),
                      tool.cuttingEdgeLength);
    settings.setValue(QStringLiteral("tool/overall_length"), tool.overallLength);
    settings.setValue(QStringLiteral("tool/shank_diameter"), tool.shankDiameter);
    settings.setValue(QStringLiteral("tool/holder_diameter"), tool.holderDiameter);
    settings.setValue(QStringLiteral("tool/holder_length"), tool.holderLength);
    settings.setValue(QStringLiteral("tool/spindle_envelope_diameter"),
                      tool.spindleEnvelopeDiameter);
    settings.setValue(QStringLiteral("tool/spindle_envelope_length"),
                      tool.spindleEnvelopeLength);
    settings.setValue(QStringLiteral("tool/tip_compensation"),
                      tool.tipCompensation);
}

void readTool(QSettings& settings, oap::toolpath::ToolDef& tool,
              QStringList& invalidKeys)
{
    readDouble(settings, QStringLiteral("tool/diameter"), tool.diameter,
               invalidKeys);
    readDouble(settings, QStringLiteral("tool/corner_radius"),
               tool.cornerRadius, invalidKeys);
    readDouble(settings, QStringLiteral("tool/horizontal_feed"),
               tool.horizFeed, invalidKeys);
    readDouble(settings, QStringLiteral("tool/vertical_feed"),
               tool.vertFeed, invalidKeys);
    readDouble(settings, QStringLiteral("tool/horizontal_rapid"),
               tool.horizRapid, invalidKeys);
    readDouble(settings, QStringLiteral("tool/vertical_rapid"),
               tool.vertRapid, invalidKeys);
    readDouble(settings, QStringLiteral("tool/spindle_speed"),
               tool.spindleSpeed, invalidKeys);
    readDouble(settings, QStringLiteral("tool/cutting_edge_length"),
               tool.cuttingEdgeLength, invalidKeys);
    readDouble(settings, QStringLiteral("tool/overall_length"),
               tool.overallLength, invalidKeys);
    readDouble(settings, QStringLiteral("tool/shank_diameter"),
               tool.shankDiameter, invalidKeys);
    readDouble(settings, QStringLiteral("tool/holder_diameter"),
               tool.holderDiameter, invalidKeys);
    readDouble(settings, QStringLiteral("tool/holder_length"),
               tool.holderLength, invalidKeys);
    readDouble(settings, QStringLiteral("tool/spindle_envelope_diameter"),
               tool.spindleEnvelopeDiameter, invalidKeys);
    readDouble(settings, QStringLiteral("tool/spindle_envelope_length"),
               tool.spindleEnvelopeLength, invalidKeys);
    readDouble(settings, QStringLiteral("tool/tip_compensation"),
               tool.tipCompensation, invalidKeys);
}

void writeDepth(QSettings& settings, const oap::toolpath::DepthParams& depth)
{
    settings.setValue(QStringLiteral("depth/clearance_height"),
                      depth.clearanceHeight);
    settings.setValue(QStringLiteral("depth/safe_height"), depth.safeHeight);
    settings.setValue(QStringLiteral("depth/use_feature_depth"),
                      depth.useFeatureDepth);
    settings.setValue(QStringLiteral("depth/start_depth"), depth.startDepth);
    settings.setValue(QStringLiteral("depth/final_depth"), depth.finalDepth);
    settings.setValue(QStringLiteral("depth/step_down"), depth.stepDown);
    settings.setValue(QStringLiteral("depth/finish_depth"), depth.finishDepth);
}

void readDepth(QSettings& settings, oap::toolpath::DepthParams& depth,
               QStringList& invalidKeys)
{
    readDouble(settings, QStringLiteral("depth/clearance_height"),
               depth.clearanceHeight, invalidKeys);
    readDouble(settings, QStringLiteral("depth/safe_height"),
               depth.safeHeight, invalidKeys);
    readBool(settings, QStringLiteral("depth/use_feature_depth"),
             depth.useFeatureDepth, invalidKeys);
    readDouble(settings, QStringLiteral("depth/start_depth"),
               depth.startDepth, invalidKeys);
    readDouble(settings, QStringLiteral("depth/final_depth"),
               depth.finalDepth, invalidKeys);
    readDouble(settings, QStringLiteral("depth/step_down"), depth.stepDown,
               invalidKeys);
    readDouble(settings, QStringLiteral("depth/finish_depth"),
               depth.finishDepth, invalidKeys);
}

void writeClearing(QSettings& settings,
                   const oap::toolpath::ClearingParams& clearing)
{
    settings.setValue(QStringLiteral("clearing/pattern"),
                      static_cast<int>(clearing.pattern));
    settings.setValue(QStringLiteral("clearing/expansion_direction"),
                      static_cast<int>(
                          oap::toolpath::effectivePocketClearingDirection(
                              clearing.pattern,
                              clearing.expansionDirection)));
    settings.setValue(QStringLiteral("clearing/cut_mode"),
                      static_cast<int>(clearing.cutMode));
    settings.setValue(QStringLiteral("clearing/entry_mode"),
                      static_cast<int>(clearing.entryMode));
    settings.setValue(QStringLiteral("clearing/ramp_start_reference"),
                      static_cast<int>(clearing.rampStartReference));
    settings.setValue(QStringLiteral("clearing/ramp_angle_deg"),
                      clearing.rampAngleDeg);
    settings.setValue(QStringLiteral("clearing/maximum_ramp_length"),
                      clearing.maximumRampLength);
    settings.setValue(QStringLiteral("clearing/helix_diameter"),
                      clearing.helixDiameter);
    settings.setValue(QStringLiteral("clearing/zigzag_ramp_width"),
                      clearing.zigzagRampWidth);
    settings.setValue(QStringLiteral("clearing/ramp_feed"),
                      clearing.rampFeed);
    settings.setValue(QStringLiteral("clearing/step_over_percent"),
                      clearing.stepOverPercent);
    settings.setValue(QStringLiteral("clearing/extra_offset"),
                      clearing.extraOffset);
    settings.setValue(QStringLiteral("clearing/precision"), clearing.precision);
    settings.setValue(QStringLiteral("clearing/arc_tolerance"),
                      clearing.arcTolerance);
    settings.setValue(QStringLiteral("clearing/zigzag_angle"),
                      clearing.zigzagAngle);
    settings.setValue(QStringLiteral("clearing/perimeter_pass"),
                      clearing.perimeterPass);
    settings.setValue(QStringLiteral("clearing/adaptive/target_engagement"),
                      clearing.adaptive.targetEngagement);
    settings.setValue(QStringLiteral("clearing/adaptive/target_angle_deg"),
                      clearing.adaptive.targetEngagementAngleDeg);
    settings.setValue(QStringLiteral("clearing/adaptive/maximum_angle_deg"),
                      clearing.adaptive.maximumEngagementAngleDeg);
    settings.setValue(QStringLiteral("clearing/adaptive/angle_samples"),
                      clearing.adaptive.engagementAngleSamples);
    settings.setValue(QStringLiteral("clearing/adaptive/minimum_turn_radius_factor"),
                      clearing.adaptive.minimumTurnRadiusFactor);
    settings.setValue(QStringLiteral("clearing/adaptive/remaining_tolerance"),
                      clearing.adaptive.remainingTolerance);
    settings.setValue(QStringLiteral("clearing/adaptive/material_cell_size"),
                      clearing.adaptive.materialCellSize);
    settings.setValue(QStringLiteral("clearing/adaptive/maximum_passes"),
                      clearing.adaptive.maximumAdaptivePasses);
    settings.setValue(QStringLiteral("clearing/adaptive/lookahead_depth"),
                      clearing.adaptive.lookaheadDepth);
    settings.setValue(QStringLiteral("clearing/adaptive/beam_width"),
                      clearing.adaptive.beamWidth);
}

void readClearing(QSettings& settings,
                  oap::toolpath::ClearingParams& clearing,
                  QStringList& invalidKeys)
{
    readEnum(settings, QStringLiteral("clearing/pattern"), clearing.pattern,
             0, 3, invalidKeys);
    readEnum(settings, QStringLiteral("clearing/expansion_direction"),
             clearing.expansionDirection, 0, 1, invalidKeys);
    clearing.expansionDirection =
        oap::toolpath::effectivePocketClearingDirection(
            clearing.pattern, clearing.expansionDirection);
    readEnum(settings, QStringLiteral("clearing/cut_mode"), clearing.cutMode,
             0, 1, invalidKeys);
    readEnum(settings, QStringLiteral("clearing/entry_mode"),
             clearing.entryMode, 0, 4, invalidKeys);
    readEnum(settings, QStringLiteral("clearing/ramp_start_reference"),
             clearing.rampStartReference, 0, 2, invalidKeys);
    readDouble(settings, QStringLiteral("clearing/ramp_angle_deg"),
               clearing.rampAngleDeg, invalidKeys);
    readDouble(settings, QStringLiteral("clearing/maximum_ramp_length"),
               clearing.maximumRampLength, invalidKeys);
    readDouble(settings, QStringLiteral("clearing/helix_diameter"),
               clearing.helixDiameter, invalidKeys);
    readDouble(settings, QStringLiteral("clearing/zigzag_ramp_width"),
               clearing.zigzagRampWidth, invalidKeys);
    readDouble(settings, QStringLiteral("clearing/ramp_feed"),
               clearing.rampFeed, invalidKeys);
    readDouble(settings, QStringLiteral("clearing/step_over_percent"),
               clearing.stepOverPercent, invalidKeys);
    readDouble(settings, QStringLiteral("clearing/extra_offset"),
               clearing.extraOffset, invalidKeys);
    readInt(settings, QStringLiteral("clearing/precision"),
            clearing.precision, invalidKeys);
    readDouble(settings, QStringLiteral("clearing/arc_tolerance"),
               clearing.arcTolerance, invalidKeys);
    readDouble(settings, QStringLiteral("clearing/zigzag_angle"),
               clearing.zigzagAngle, invalidKeys);
    readBool(settings, QStringLiteral("clearing/perimeter_pass"),
             clearing.perimeterPass, invalidKeys);
    readDouble(settings, QStringLiteral("clearing/adaptive/target_engagement"),
               clearing.adaptive.targetEngagement, invalidKeys);
    readDouble(settings, QStringLiteral("clearing/adaptive/target_angle_deg"),
               clearing.adaptive.targetEngagementAngleDeg, invalidKeys);
    readDouble(settings, QStringLiteral("clearing/adaptive/maximum_angle_deg"),
               clearing.adaptive.maximumEngagementAngleDeg, invalidKeys);
    readInt(settings, QStringLiteral("clearing/adaptive/angle_samples"),
            clearing.adaptive.engagementAngleSamples, invalidKeys);
    readDouble(settings,
               QStringLiteral("clearing/adaptive/minimum_turn_radius_factor"),
               clearing.adaptive.minimumTurnRadiusFactor, invalidKeys);
    readDouble(settings, QStringLiteral("clearing/adaptive/remaining_tolerance"),
               clearing.adaptive.remainingTolerance, invalidKeys);
    readDouble(settings, QStringLiteral("clearing/adaptive/material_cell_size"),
               clearing.adaptive.materialCellSize, invalidKeys);
    readInt(settings, QStringLiteral("clearing/adaptive/maximum_passes"),
            clearing.adaptive.maximumAdaptivePasses, invalidKeys);
    readInt(settings, QStringLiteral("clearing/adaptive/lookahead_depth"),
            clearing.adaptive.lookaheadDepth, invalidKeys);
    readInt(settings, QStringLiteral("clearing/adaptive/beam_width"),
            clearing.adaptive.beamWidth, invalidKeys);
}

void writeProfile(QSettings& settings,
                  const oap::toolpath::ProfileParams& profile)
{
    settings.setValue(QStringLiteral("profile/machining_mode"),
                      static_cast<int>(profile.machiningMode));
    settings.setValue(QStringLiteral("profile/cut_mode"),
                      static_cast<int>(profile.cutMode));
    settings.setValue(QStringLiteral("profile/side_allowance"),
                      profile.sideAllowance);
    settings.setValue(QStringLiteral("profile/roughing_allowance"),
                      profile.roughingAllowance);
    settings.setValue(QStringLiteral("profile/finish_pass"),
                      profile.finishPass);
    settings.setValue(QStringLiteral("profile/radial_pass_count"),
                      profile.radialPassCount);
    settings.setValue(QStringLiteral("profile/radial_step_over"),
                      profile.radialStepOver);
    settings.setValue(QStringLiteral("profile/lead_in_length"),
                      profile.leadInLength);
    settings.setValue(QStringLiteral("profile/lead_out_length"),
                      profile.leadOutLength);
    settings.setValue(QStringLiteral("profile/precision"),
                      profile.precision);
    settings.setValue(QStringLiteral("profile/arc_tolerance"),
                      profile.arcTolerance);
}

void readProfile(QSettings& settings,
                 oap::toolpath::ProfileParams& profile,
                 QStringList& invalidKeys)
{
    readEnum(settings, QStringLiteral("profile/machining_mode"),
             profile.machiningMode, 0, 1, invalidKeys);
    readEnum(settings, QStringLiteral("profile/cut_mode"), profile.cutMode,
             0, 1, invalidKeys);
    readDouble(settings, QStringLiteral("profile/side_allowance"),
               profile.sideAllowance, invalidKeys);
    readDouble(settings, QStringLiteral("profile/roughing_allowance"),
               profile.roughingAllowance, invalidKeys);
    readBool(settings, QStringLiteral("profile/finish_pass"),
             profile.finishPass, invalidKeys);
    readInt(settings, QStringLiteral("profile/radial_pass_count"),
            profile.radialPassCount, invalidKeys);
    readDouble(settings, QStringLiteral("profile/radial_step_over"),
               profile.radialStepOver, invalidKeys);
    readDouble(settings, QStringLiteral("profile/lead_in_length"),
               profile.leadInLength, invalidKeys);
    readDouble(settings, QStringLiteral("profile/lead_out_length"),
               profile.leadOutLength, invalidKeys);
    readInt(settings, QStringLiteral("profile/precision"),
            profile.precision, invalidKeys);
    readDouble(settings, QStringLiteral("profile/arc_tolerance"),
               profile.arcTolerance, invalidKeys);
}

void writeHole(QSettings& settings,
               const oap::toolpath::HoleMachiningParams& hole)
{
    settings.setValue(QStringLiteral("hole/start_corner"),
                      static_cast<int>(hole.startCorner));
    settings.setValue(QStringLiteral("hole/allowance"), hole.allowance);
    settings.setValue(QStringLiteral("hole/small_hole_diameter_tolerance"),
                      hole.smallHoleDiameterTolerance);
    settings.setValue(QStringLiteral("hole/center_spot_depth"),
                      hole.centerSpotDepth);
    settings.setValue(QStringLiteral("hole/through_hole_breakthrough"),
                      hole.throughHoleBreakthrough);
    settings.setValue(QStringLiteral("hole/medium_hole_ratio"),
                      hole.mediumHoleRadiusToolDiameterRatio);
    settings.setValue(QStringLiteral("hole/drill_cycle_mode"),
                      static_cast<int>(hole.drillCycleMode));
    settings.setValue(QStringLiteral("hole/drill_retract_mode"),
                      static_cast<int>(hole.drillRetractMode));
    settings.setValue(QStringLiteral("hole/peck_depth"), hole.peckDepth);
    settings.setValue(QStringLiteral("hole/dwell_seconds"), hole.dwellSeconds);
    settings.setValue(QStringLiteral("hole/drill_tip_angle_deg"),
                      hole.drillTipAngleDeg);
    settings.setValue(QStringLiteral("hole/g81_max_ratio"),
                      hole.g81MaxDepthDiameterRatio);
    settings.setValue(QStringLiteral("hole/g73_max_ratio"),
                      hole.g73MaxDepthDiameterRatio);
    settings.setValue(QStringLiteral("hole/g73_peck_depth"),
                      hole.g73PeckDepth);
    settings.setValue(QStringLiteral("hole/g83_peck_depth"),
                      hole.g83PeckDepth);
    settings.setValue(QStringLiteral("hole/g73_chip_break_retract"),
                      hole.g73ChipBreakRetract);
    settings.setValue(QStringLiteral("hole/g83_reentry_clearance"),
                      hole.g83ReentryClearance);
    settings.setValue(QStringLiteral("hole/collision_envelope_mode"),
                      static_cast<int>(hole.collisionEnvelopeMode));
    settings.setValue(QStringLiteral("hole/collision_sample_step"),
                      hole.collisionEnvelopeSampleStep);
    settings.setValue(QStringLiteral("hole/helix_pitch"), hole.helixPitch);
    settings.setValue(QStringLiteral("hole/radial_step"), hole.radialStep);
    settings.setValue(QStringLiteral("hole/sample_angle_deg"),
                      hole.sampleAngleDeg);
    settings.setValue(QStringLiteral("hole/helix_chord_tolerance"),
                      hole.helixChordTolerance);
    settings.setValue(QStringLiteral("hole/min_sample_angle_deg"),
                      hole.minSampleAngleDeg);
    settings.setValue(QStringLiteral("hole/max_sample_angle_deg"),
                      hole.maxSampleAngleDeg);
    settings.setValue(QStringLiteral("hole/max_sample_points"),
                      hole.maxSamplePoints);
}

void readHole(QSettings& settings, oap::toolpath::HoleMachiningParams& hole,
              QStringList& invalidKeys)
{
    readEnum(settings, QStringLiteral("hole/start_corner"), hole.startCorner,
             0, 3, invalidKeys);
    readDouble(settings, QStringLiteral("hole/allowance"), hole.allowance,
               invalidKeys);
    readDouble(settings, QStringLiteral("hole/small_hole_diameter_tolerance"),
               hole.smallHoleDiameterTolerance, invalidKeys);
    readDouble(settings, QStringLiteral("hole/center_spot_depth"),
               hole.centerSpotDepth, invalidKeys);
    readDouble(settings, QStringLiteral("hole/through_hole_breakthrough"),
               hole.throughHoleBreakthrough, invalidKeys);
    readDouble(settings, QStringLiteral("hole/medium_hole_ratio"),
               hole.mediumHoleRadiusToolDiameterRatio, invalidKeys);
    readEnum(settings, QStringLiteral("hole/drill_cycle_mode"),
             hole.drillCycleMode, 0, 3, invalidKeys);
    readEnum(settings, QStringLiteral("hole/drill_retract_mode"),
             hole.drillRetractMode, 0, 1, invalidKeys);
    readDouble(settings, QStringLiteral("hole/peck_depth"), hole.peckDepth,
               invalidKeys);
    readDouble(settings, QStringLiteral("hole/dwell_seconds"),
               hole.dwellSeconds, invalidKeys);
    readDouble(settings, QStringLiteral("hole/drill_tip_angle_deg"),
               hole.drillTipAngleDeg, invalidKeys);
    readDouble(settings, QStringLiteral("hole/g81_max_ratio"),
               hole.g81MaxDepthDiameterRatio, invalidKeys);
    readDouble(settings, QStringLiteral("hole/g73_max_ratio"),
               hole.g73MaxDepthDiameterRatio, invalidKeys);
    readDouble(settings, QStringLiteral("hole/g73_peck_depth"),
               hole.g73PeckDepth, invalidKeys);
    readDouble(settings, QStringLiteral("hole/g83_peck_depth"),
               hole.g83PeckDepth, invalidKeys);
    readDouble(settings, QStringLiteral("hole/g73_chip_break_retract"),
               hole.g73ChipBreakRetract, invalidKeys);
    readDouble(settings, QStringLiteral("hole/g83_reentry_clearance"),
               hole.g83ReentryClearance, invalidKeys);
    readEnum(settings, QStringLiteral("hole/collision_envelope_mode"),
             hole.collisionEnvelopeMode, 0, 2, invalidKeys);
    readDouble(settings, QStringLiteral("hole/collision_sample_step"),
               hole.collisionEnvelopeSampleStep, invalidKeys);
    readDouble(settings, QStringLiteral("hole/helix_pitch"),
               hole.helixPitch, invalidKeys);
    readDouble(settings, QStringLiteral("hole/helix_chord_tolerance"),
               hole.helixChordTolerance, invalidKeys);
    readDouble(settings, QStringLiteral("hole/radial_step"),
               hole.radialStep, invalidKeys);
    readDouble(settings, QStringLiteral("hole/sample_angle_deg"),
               hole.sampleAngleDeg, invalidKeys);
    readDouble(settings, QStringLiteral("hole/min_sample_angle_deg"),
               hole.minSampleAngleDeg, invalidKeys);
    readDouble(settings, QStringLiteral("hole/max_sample_angle_deg"),
               hole.maxSampleAngleDeg, invalidKeys);
    readInt(settings, QStringLiteral("hole/max_sample_points"),
            hole.maxSamplePoints, invalidKeys);
}

} // namespace

bool ToolpathParameterSettingsStorage::load(
    QSettings& settings, oap::toolpath::ToolpathParams& holeParams,
    oap::toolpath::ToolpathParams& pocketParams,
    oap::toolpath::ToolpathParams& profileParams,
    HoleToolSelectionMode& holeToolSelectionMode,
    PocketToolSelectionMode& pocketToolSelectionMode,
    ProfileToolSelectionMode& profileToolSelectionMode,
    bool& coolant, QString* warningOrError)
{
    if (warningOrError)
        warningOrError->clear();
    settings.sync();
    if (settings.status() != QSettings::NoError) {
        if (warningOrError)
            *warningOrError = settingsStatusText(settings.status());
        return false;
    }

    settings.beginGroup(QString::fromLatin1(kSettingsGroup));
    if (!settings.contains(QStringLiteral("schema_version"))) {
        settings.endGroup();
        return true;
    }

    bool versionOk = false;
    const int version =
        settings.value(QStringLiteral("schema_version")).toInt(&versionOk);
    if (!versionOk ||
        (version != kLegacySchemaVersion &&
         version != kCombinedPocketProfileSchemaVersion &&
         version != kSeparateProfileSchemaVersion &&
         version != kProfileToolSelectionSchemaVersion &&
         version != kSchemaVersion)) {
        settings.endGroup();
        if (warningOrError) {
            *warningOrError = QStringLiteral(
                "本地刀路参数版本不受支持（读取到 %1，当前支持 %2），已使用程序默认值")
                .arg(versionOk ? QString::number(version)
                               : QStringLiteral("无效值"))
                .arg(kSchemaVersion);
        }
        return false;
    }

    // 事务式读取：版本错误不会污染调用方；单个异常字段回退到调用方已有值。
    oap::toolpath::ToolpathParams loadedHole = holeParams;
    oap::toolpath::ToolpathParams loadedPocket = pocketParams;
    oap::toolpath::ToolpathParams loadedProfile = profileParams;
    HoleToolSelectionMode loadedHoleToolSelectionMode =
        HoleToolSelectionMode::AutomaticLibrary;
    PocketToolSelectionMode loadedPocketToolSelectionMode =
        PocketToolSelectionMode::AutomaticLibrary;
    ProfileToolSelectionMode loadedProfileToolSelectionMode =
        ProfileToolSelectionMode::AutomaticLibrary;
    bool loadedCoolant = coolant;
    QStringList invalidKeys;
    if (version == kLegacySchemaVersion) {
        oap::toolpath::ToolpathParams legacy = loadedHole;
        readTool(settings, legacy.tool, invalidKeys);
        readDepth(settings, legacy.depth, invalidKeys);
        readClearing(settings, legacy.clearing, invalidKeys);
        readHole(settings, legacy.hole, invalidKeys);
        loadedHole = legacy;
        loadedPocket = legacy;
        loadedProfile = legacy;
    } else {
        settings.beginGroup(QString::fromLatin1(kHoleParametersGroup));
        if (version >= kSchemaVersion) {
            readEnum(
                settings, QStringLiteral("tool_selection_mode"),
                loadedHoleToolSelectionMode,
                static_cast<int>(
                    HoleToolSelectionMode::AutomaticLibrary),
                static_cast<int>(
                    HoleToolSelectionMode::SingleTool),
                invalidKeys);
        }
        readTool(settings, loadedHole.tool, invalidKeys);
        readDepth(settings, loadedHole.depth, invalidKeys);
        readHole(settings, loadedHole.hole, invalidKeys);
        settings.endGroup();

        settings.beginGroup(QString::fromLatin1(kPocketParametersGroup));
        readEnum(
            settings, QStringLiteral("tool_selection_mode"),
            loadedPocketToolSelectionMode,
            static_cast<int>(PocketToolSelectionMode::AutomaticLibrary),
            static_cast<int>(PocketToolSelectionMode::SingleTool),
            invalidKeys);
        readTool(settings, loadedPocket.tool, invalidKeys);
        readDepth(settings, loadedPocket.depth, invalidKeys);
        readClearing(settings, loadedPocket.clearing, invalidKeys);
        if (version == kCombinedPocketProfileSchemaVersion)
            readProfile(settings, loadedPocket.profile, invalidKeys);
        settings.endGroup();
        if (version == kCombinedPocketProfileSchemaVersion) {
            // Schema 2 stored pocket and profile settings together. Preserve
            // the user's previous profile values when introducing its own page.
            loadedProfile = loadedPocket;
        } else {
            settings.beginGroup(QString::fromLatin1(kProfileParametersGroup));
            if (version >= kProfileToolSelectionSchemaVersion) {
                readEnum(
                    settings, QStringLiteral("tool_selection_mode"),
                    loadedProfileToolSelectionMode,
                    static_cast<int>(
                        ProfileToolSelectionMode::AutomaticLibrary),
                    static_cast<int>(
                        ProfileToolSelectionMode::SingleTool),
                    invalidKeys);
            }
            readTool(settings, loadedProfile.tool, invalidKeys);
            readDepth(settings, loadedProfile.depth, invalidKeys);
            readClearing(settings, loadedProfile.clearing, invalidKeys);
            readProfile(settings, loadedProfile.profile, invalidKeys);
            settings.endGroup();
        }
    }
    readBool(settings, QStringLiteral("program/coolant"), loadedCoolant,
             invalidKeys);
    settings.endGroup();

    holeParams = loadedHole;
    pocketParams = loadedPocket;
    profileParams = loadedProfile;
    holeToolSelectionMode = loadedHoleToolSelectionMode;
    pocketToolSelectionMode = loadedPocketToolSelectionMode;
    profileToolSelectionMode = loadedProfileToolSelectionMode;
    coolant = loadedCoolant;
    if (!invalidKeys.isEmpty() && warningOrError) {
        *warningOrError = QStringLiteral(
            "以下本地刀路参数无效，已分别回退到程序默认值：%1")
            .arg(invalidKeys.join(QStringLiteral(", ")));
    }
    return true;
}

// 保存前先整体清空该组,避免旧版本遗留键造成脏数据;
// 写入后显式 sync() 以立即暴露权限/格式错误。
bool ToolpathParameterSettingsStorage::save(
    QSettings& settings,
    const oap::toolpath::ToolpathParams& holeParams,
    const oap::toolpath::ToolpathParams& pocketParams,
    const oap::toolpath::ToolpathParams& profileParams,
    HoleToolSelectionMode holeToolSelectionMode,
    PocketToolSelectionMode pocketToolSelectionMode,
    ProfileToolSelectionMode profileToolSelectionMode,
    bool coolant, QString* error)
{
    if (error)
        error->clear();
    settings.beginGroup(QString::fromLatin1(kSettingsGroup));
    settings.remove(QString());
    settings.setValue(QStringLiteral("schema_version"), kSchemaVersion);
    settings.beginGroup(QString::fromLatin1(kHoleParametersGroup));
    settings.setValue(
        QStringLiteral("tool_selection_mode"),
        static_cast<int>(holeToolSelectionMode));
    writeTool(settings, holeParams.tool);
    writeDepth(settings, holeParams.depth);
    writeHole(settings, holeParams.hole);
    settings.endGroup();
    settings.beginGroup(QString::fromLatin1(kPocketParametersGroup));
    settings.setValue(
        QStringLiteral("tool_selection_mode"),
        static_cast<int>(pocketToolSelectionMode));
    writeTool(settings, pocketParams.tool);
    writeDepth(settings, pocketParams.depth);
    writeClearing(settings, pocketParams.clearing);
    settings.endGroup();
    settings.beginGroup(QString::fromLatin1(kProfileParametersGroup));
    settings.setValue(
        QStringLiteral("tool_selection_mode"),
        static_cast<int>(profileToolSelectionMode));
    writeTool(settings, profileParams.tool);
    writeDepth(settings, profileParams.depth);
    writeClearing(settings, profileParams.clearing);
    writeProfile(settings, profileParams.profile);
    settings.endGroup();
    settings.setValue(QStringLiteral("program/coolant"), coolant);
    settings.endGroup();
    settings.sync();
    if (settings.status() == QSettings::NoError)
        return true;
    if (error)
        *error = settingsStatusText(settings.status());
    return false;
}

// 用户级默认存储入口:定位到应用专属 QSettings 后转交 load/save。
bool ToolpathParameterSettingsStorage::loadDefault(
    oap::toolpath::ToolpathParams& holeParams,
    oap::toolpath::ToolpathParams& pocketParams,
    oap::toolpath::ToolpathParams& profileParams,
    HoleToolSelectionMode& holeToolSelectionMode,
    PocketToolSelectionMode& pocketToolSelectionMode,
    ProfileToolSelectionMode& profileToolSelectionMode,
    bool& coolant,
    QString* warningOrError)
{
    QSettings settings(QString::fromLatin1(kOrganization),
                       QString::fromLatin1(kApplication));
    return load(
        settings, holeParams, pocketParams, profileParams,
        holeToolSelectionMode,
        pocketToolSelectionMode, profileToolSelectionMode,
        coolant, warningOrError);
}

bool ToolpathParameterSettingsStorage::saveDefault(
    const oap::toolpath::ToolpathParams& holeParams,
    const oap::toolpath::ToolpathParams& pocketParams,
    const oap::toolpath::ToolpathParams& profileParams,
    HoleToolSelectionMode holeToolSelectionMode,
    PocketToolSelectionMode pocketToolSelectionMode,
    ProfileToolSelectionMode profileToolSelectionMode,
    bool coolant,
    QString* error)
{
    QSettings settings(QString::fromLatin1(kOrganization),
                       QString::fromLatin1(kApplication));
    return save(
        settings, holeParams, pocketParams, profileParams,
        holeToolSelectionMode,
        pocketToolSelectionMode, profileToolSelectionMode,
        coolant, error);
}

} // namespace oap::app

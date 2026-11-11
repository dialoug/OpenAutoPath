// MachiningConfigurationSession 实现（oap_app_workflow_cam）。
// 桌面加工参数与程序级配置，见 MachiningConfigurationSession.h。
#include "MachiningConfigurationSession.h"
#include <QCoreApplication>

#include <utility>

#include "app/workflows/project/ToolpathParameterSettings.h"
#include "app/workflows/project/EngravingSettingsStorage.h"
#include "app/workflows/project/DirectToolSettingsStorage.h"

namespace oap::app {

// 从本地存储恢复默认刀路参数;加载失败且未给出具体警告时,
// 生成一条通用的中文提示作为返回值。
QString MachiningConfigurationSession::loadDefaults()
{
    QString warning;
    if (!ToolpathParameterSettingsStorage::loadDefault(
            m_holeParams, m_pocketParams, m_profileParams,
            m_holeToolSelectionMode,
            m_pocketToolSelectionMode, m_profileToolSelectionMode,
            m_gcodeOptions.coolant, &warning) && warning.isEmpty()) {
        warning = QCoreApplication::translate("OpenAutoPath", "无法读取本地刀路参数配置");
    }
    QString engravingWarning;
    EngravingSettingsStorage::loadDefault(m_engraving, &engravingWarning);
    if (!engravingWarning.isEmpty()) {
        if (!warning.isEmpty()) warning += '\n';
        warning += engravingWarning;
    }
    QString directWarning;
    DirectToolSettingsStorage::loadDefault(m_holeSingleTool, m_pocketSingleTool, m_profileSingleTool, &directWarning);
    if (!directWarning.isEmpty()) {
        if (!warning.isEmpty()) warning += '\n';
        warning += directWarning;
    }
    return warning;
}

// 内部状态 + 外部传入的工位信息合并为一份"编辑器快照"。
ToolpathParameterEditorState
MachiningConfigurationSession::editorState(
    const toolpath::MoldSetupWorkFrames& workFrames,
    double moldLength,
    double moldWidth) const
{
    ToolpathParameterEditorState result;
    result.holeParams = m_holeParams;
    result.pocketParams = m_pocketParams;
    result.profileParams = m_profileParams;
    result.engraving = m_engraving;
    result.holeSingleTool = m_holeSingleTool;
    result.pocketSingleTool = m_pocketSingleTool;
    result.profileSingleTool = m_profileSingleTool;
    result.holeToolSelectionMode =
        m_holeToolSelectionMode;
    result.pocketToolSelectionMode =
        m_pocketToolSelectionMode;
    result.profileToolSelectionMode =
        m_profileToolSelectionMode;
    result.machiningSetup = m_machiningSetup;
    result.moldSetupWorkFrames = workFrames;
    result.gcodeProgramOptions = m_gcodeOptions;
    result.moldLength = moldLength;
    result.moldWidth = moldWidth;
    return result;
}

// apply 分三步:先在旧值上计算"哪些变了"的摘要,
// 再接管编辑结果并写入内部状态,最后持久化默认配置。
MachiningConfigurationApplyResult
MachiningConfigurationSession::apply(
    ToolpathParameterEditorState edited)
{
    if (edited.holeToolSelectionMode != ToolSelectionMode::SingleTool)
        edited.holeToolSelectionMode = ToolSelectionMode::AutomaticLibrary;
    if (edited.pocketToolSelectionMode != ToolSelectionMode::SingleTool)
        edited.pocketToolSelectionMode = ToolSelectionMode::AutomaticLibrary;
    if (edited.profileToolSelectionMode != ToolSelectionMode::SingleTool)
        edited.profileToolSelectionMode = ToolSelectionMode::AutomaticLibrary;
    MachiningConfigurationApplyResult result;
    result.engravingParametersChanged =
        !EngravingSettingsStorage::equal(m_engraving, edited.engraving);
    result.holeToolSelectionModeChanged =
        m_holeToolSelectionMode != edited.holeToolSelectionMode;
    result.holeParametersChanged =
        !parametersEqual(m_holeParams, edited.holeParams) ||
        result.holeToolSelectionModeChanged ||
        (edited.holeToolSelectionMode == ToolSelectionMode::SingleTool &&
         toolpath::toolpathToolContextKey(m_holeSingleTool) != toolpath::toolpathToolContextKey(edited.holeSingleTool));
    result.pocketToolSelectionModeChanged =
        m_pocketToolSelectionMode !=
        edited.pocketToolSelectionMode;
    result.pocketParametersChanged =
        !parametersEqual(m_pocketParams, edited.pocketParams) ||
        result.pocketToolSelectionModeChanged ||
        (edited.pocketToolSelectionMode == ToolSelectionMode::SingleTool &&
         toolpath::toolpathToolContextKey(m_pocketSingleTool) != toolpath::toolpathToolContextKey(edited.pocketSingleTool));
    result.profileToolSelectionModeChanged =
        m_profileToolSelectionMode !=
        edited.profileToolSelectionMode;
    result.profileParametersChanged =
        !parametersEqual(m_profileParams, edited.profileParams) ||
        result.profileToolSelectionModeChanged ||
        (edited.profileToolSelectionMode == ToolSelectionMode::SingleTool &&
         toolpath::toolpathToolContextKey(m_profileSingleTool) != toolpath::toolpathToolContextKey(edited.profileSingleTool));
    result.toolIdentityCleared =
        toolChanged(m_holeParams.tool, edited.holeParams.tool) ||
        toolChanged(m_pocketParams.tool, edited.pocketParams.tool) ||
        toolChanged(m_profileParams.tool, edited.profileParams.tool) ||
        m_gcodeOptions.coolant != edited.gcodeProgramOptions.coolant;

    // 刀具特征或冷却方式变化意味着上一把刀的后处理身份不再可信,
    // 因此强制清空 toolId/toolName,避免生成带错误刀具号的程序。
    m_holeSingleTool = std::move(edited.holeSingleTool);
    m_holeParams = std::move(edited.holeParams);
    m_pocketSingleTool = std::move(edited.pocketSingleTool);
    m_pocketParams = std::move(edited.pocketParams);
    m_profileSingleTool = std::move(edited.profileSingleTool);
    m_profileParams = std::move(edited.profileParams);
    m_engraving = std::move(edited.engraving);
    m_holeToolSelectionMode =
        edited.holeToolSelectionMode;
    m_pocketToolSelectionMode =
        edited.pocketToolSelectionMode;
    m_profileToolSelectionMode =
        edited.profileToolSelectionMode;
    m_machiningSetup = std::move(edited.machiningSetup);
    result.setupWorkFrames = std::move(edited.moldSetupWorkFrames);
    m_gcodeOptions = std::move(edited.gcodeProgramOptions);
    if (result.toolIdentityCleared) {
        m_gcodeOptions.toolId.clear();
        m_gcodeOptions.toolName.clear();
    }

    ToolpathParameterSettingsStorage::saveDefault(
        m_holeParams, m_pocketParams, m_profileParams,
        m_holeToolSelectionMode,
        m_pocketToolSelectionMode, m_profileToolSelectionMode,
        m_gcodeOptions.coolant,
        &result.persistenceError);
    QString engravingError;
    EngravingSettingsStorage::saveDefault(m_engraving, &engravingError);
    if (!engravingError.isEmpty()) {
        if (!result.persistenceError.isEmpty()) result.persistenceError += '\n';
        result.persistenceError += engravingError;
    }
    QString directError;
    DirectToolSettingsStorage::saveDefault(m_holeSingleTool, m_pocketSingleTool, m_profileSingleTool, &directError);
    if (!directError.isEmpty()) {
        if (!result.persistenceError.isEmpty()) result.persistenceError += '\n';
        result.persistenceError += directError;
    }
    return result;
}

// 只比较影响切削效果的几何/工艺字段,忽略刀号等身份字段。
bool MachiningConfigurationSession::toolChanged(
    const toolpath::ToolDef& first,
    const toolpath::ToolDef& second) noexcept
{
    return first.diameter != second.diameter ||
           first.cornerRadius != second.cornerRadius ||
           first.tipCompensation != second.tipCompensation ||
           first.spindleSpeed != second.spindleSpeed ||
           first.horizFeed != second.horizFeed ||
           first.vertFeed != second.vertFeed;
}

// 三组大字段全等才认为参数未变;用于在 apply 中生成变更摘要。
bool MachiningConfigurationSession::parametersEqual(
    const toolpath::ToolpathParams& a,
    const toolpath::ToolpathParams& b) noexcept
{
    return a.tool.diameter == b.tool.diameter &&
           a.tool.cornerRadius == b.tool.cornerRadius &&
           a.tool.tipCompensation == b.tool.tipCompensation &&
           a.tool.horizFeed == b.tool.horizFeed &&
           a.tool.vertFeed == b.tool.vertFeed &&
           a.tool.horizRapid == b.tool.horizRapid &&
           a.tool.vertRapid == b.tool.vertRapid &&
           a.tool.spindleSpeed == b.tool.spindleSpeed &&
           a.tool.cuttingEdgeLength == b.tool.cuttingEdgeLength &&
           a.tool.overallLength == b.tool.overallLength &&
           a.tool.shankDiameter == b.tool.shankDiameter &&
           a.tool.holderDiameter == b.tool.holderDiameter &&
           a.tool.holderLength == b.tool.holderLength &&
           a.tool.spindleEnvelopeDiameter ==
               b.tool.spindleEnvelopeDiameter &&
           a.tool.spindleEnvelopeLength ==
               b.tool.spindleEnvelopeLength &&
           a.depth.clearanceHeight == b.depth.clearanceHeight &&
           a.depth.safeHeight == b.depth.safeHeight &&
           a.depth.useFeatureDepth == b.depth.useFeatureDepth &&
           a.depth.startDepth == b.depth.startDepth &&
           a.depth.finalDepth == b.depth.finalDepth &&
           a.depth.stepDown == b.depth.stepDown &&
           a.depth.finishDepth == b.depth.finishDepth &&
           a.clearing.pattern == b.clearing.pattern &&
           toolpath::effectivePocketClearingDirection(
               a.clearing.pattern,
               a.clearing.expansionDirection) ==
               toolpath::effectivePocketClearingDirection(
                   b.clearing.pattern,
                   b.clearing.expansionDirection) &&
           a.clearing.cutMode == b.clearing.cutMode &&
           a.clearing.entryMode == b.clearing.entryMode &&
           a.clearing.rampAngleDeg == b.clearing.rampAngleDeg &&
           a.clearing.maximumRampLength ==
               b.clearing.maximumRampLength &&
           a.clearing.helixDiameter == b.clearing.helixDiameter &&
           a.clearing.stepOverPercent == b.clearing.stepOverPercent &&
           a.clearing.extraOffset == b.clearing.extraOffset &&
           a.clearing.precision == b.clearing.precision &&
           a.clearing.arcTolerance == b.clearing.arcTolerance &&
           a.clearing.zigzagAngle == b.clearing.zigzagAngle &&
           a.clearing.perimeterPass == b.clearing.perimeterPass &&
           a.clearing.adaptive.targetEngagement ==
               b.clearing.adaptive.targetEngagement &&
           a.clearing.adaptive.targetEngagementAngleDeg ==
               b.clearing.adaptive.targetEngagementAngleDeg &&
           a.clearing.adaptive.maximumEngagementAngleDeg ==
               b.clearing.adaptive.maximumEngagementAngleDeg &&
           a.clearing.adaptive.engagementAngleSamples ==
               b.clearing.adaptive.engagementAngleSamples &&
           a.clearing.adaptive.minimumTurnRadiusFactor ==
               b.clearing.adaptive.minimumTurnRadiusFactor &&
           a.clearing.adaptive.remainingTolerance ==
               b.clearing.adaptive.remainingTolerance &&
           a.clearing.adaptive.materialCellSize ==
               b.clearing.adaptive.materialCellSize &&
           a.clearing.adaptive.maximumAdaptivePasses ==
               b.clearing.adaptive.maximumAdaptivePasses &&
           a.clearing.adaptive.lookaheadDepth ==
               b.clearing.adaptive.lookaheadDepth &&
           a.clearing.adaptive.beamWidth ==
               b.clearing.adaptive.beamWidth &&
           a.profile.machiningMode == b.profile.machiningMode &&
           a.profile.cutMode == b.profile.cutMode &&
           a.profile.sideAllowance == b.profile.sideAllowance &&
           a.profile.roughingAllowance == b.profile.roughingAllowance &&
           a.profile.finishPass == b.profile.finishPass &&
           a.profile.radialPassCount == b.profile.radialPassCount &&
           a.profile.radialStepOver == b.profile.radialStepOver &&
           a.profile.leadInLength == b.profile.leadInLength &&
           a.profile.leadOutLength == b.profile.leadOutLength &&
           a.profile.precision == b.profile.precision &&
           a.profile.arcTolerance == b.profile.arcTolerance &&
           a.hole.smallHoleDiameterTolerance ==
               b.hole.smallHoleDiameterTolerance &&
           a.hole.useCenterSpot == b.hole.useCenterSpot &&
           a.hole.centerSpotDepth == b.hole.centerSpotDepth &&
           a.hole.throughHoleBreakthrough ==
               b.hole.throughHoleBreakthrough &&
           a.hole.mediumHoleRadiusToolDiameterRatio ==
               b.hole.mediumHoleRadiusToolDiameterRatio &&
           a.hole.drillCycleMode == b.hole.drillCycleMode &&
           a.hole.drillRetractMode == b.hole.drillRetractMode &&
           a.hole.allowG99RetractOptimization ==
               b.hole.allowG99RetractOptimization &&
           a.hole.peckDepth == b.hole.peckDepth &&
           a.hole.dwellSeconds == b.hole.dwellSeconds &&
           a.hole.drillTipAngleDeg == b.hole.drillTipAngleDeg &&
           a.hole.g81MaxDepthDiameterRatio ==
               b.hole.g81MaxDepthDiameterRatio &&
           a.hole.g73MaxDepthDiameterRatio ==
               b.hole.g73MaxDepthDiameterRatio &&
           a.hole.g73PeckDepth == b.hole.g73PeckDepth &&
           a.hole.g83PeckDepth == b.hole.g83PeckDepth &&
           a.hole.g73ChipBreakRetract == b.hole.g73ChipBreakRetract &&
           a.hole.g83ReentryClearance == b.hole.g83ReentryClearance &&
           a.hole.collisionEnvelopeMode == b.hole.collisionEnvelopeMode &&
           a.hole.collisionEnvelopeSampleStep ==
               b.hole.collisionEnvelopeSampleStep &&
           a.hole.collisionClearance == b.hole.collisionClearance &&
           a.hole.startCorner == b.hole.startCorner;
}

} // namespace oap::app

// PocketToolpathGenerationUseCase 实现（oap_app_workflow_cam）。
// 不依赖 MainWindow 状态的完整桌面槽/腔刀路候选生成，见 PocketToolpathGenerationUseCase.h。
#include "app/workflows/pocket/PocketToolpathGenerationUseCase.h"
#include "app/workflows/project/DirectToolSource.h"

#include "app/workflows/pocket/detail/PocketToolpathRunner.h"
#include "app/workflows/shared/detail/ToolpathRequestBuilder.h"

#include <algorithm>
#include <cmath>

#include <QDateTime>
#include <QElapsedTimer>

namespace oap::app {
namespace {

QString featureKindText(FeatureKind kind)
{
    switch (kind) {
        case FeatureKind::ThroughHole: return QStringLiteral("通孔");
        case FeatureKind::BlindHole: return QStringLiteral("盲孔");
        case FeatureKind::CountersinkHole: return QStringLiteral("沉孔");
        case FeatureKind::Slot: return QStringLiteral("槽");
        case FeatureKind::Pocket: return QStringLiteral("腔");
    }
    return QStringLiteral("特征");
}

QString clearingPatternText(toolpath::ClearingPattern pattern)
{
    switch (pattern) {
        case toolpath::ClearingPattern::Offset:
            return QStringLiteral("回字形");
        case toolpath::ClearingPattern::ZigZag:
            return QStringLiteral("Z字形");
        case toolpath::ClearingPattern::Spiral:
            return QStringLiteral("螺旋形");
        case toolpath::ClearingPattern::FreePath:
            return QStringLiteral("自适应清腔（FreePath）");
    }
    return QStringLiteral("回字形");
}

QString expansionDirectionText(
    toolpath::PocketClearingDirection direction)
{
    switch (direction) {
        case toolpath::PocketClearingDirection::OutsideToInside:
            return QStringLiteral("外圈向中心");
        case toolpath::PocketClearingDirection::InsideToOutside:
            return QStringLiteral("中心向外圈");
    }
    return QStringLiteral("外圈向中心");
}

QString entryModeText(toolpath::PocketEntryMode mode)
{
    switch (mode) {
        case toolpath::PocketEntryMode::VerticalPlunge:
            return QStringLiteral("垂直下刀");
        case toolpath::PocketEntryMode::RampOnShape:
            return QStringLiteral("沿轮廓斜坡");
        case toolpath::PocketEntryMode::LinearRamp:
            return QStringLiteral("直线斜坡");
        case toolpath::PocketEntryMode::HelicalRamp:
            return QStringLiteral("螺旋斜坡");
        case toolpath::PocketEntryMode::ZigZagRamp:
            return QStringLiteral("往复斜坡");
    }
    return QStringLiteral("垂直下刀");
}

QString diagnosticReasonText(
    const toolpath::ToolpathDiagnostic& diagnostic)
{
    using Code = toolpath::ToolpathDiagnosticCode;
    if (diagnostic.code == Code::MissingPlanarFloor)
        return QStringLiteral("底面非平面（暂不支持斜底/曲底）");
    if (diagnostic.code == Code::ManualFloorDisconnected)
        return QStringLiteral("人工底面没有可验证的拓扑邻接侧壁");
    if (diagnostic.code == Code::DuplicateRegion) {
        return QStringLiteral(
            "自动识别区域与已有区域共用同一底面，源面已合并并去重");
    }
    return QString::fromUtf8(
        toolpath::toolpathDiagnosticDefaultMessageZh(diagnostic.code));
}

QString validationMessage(
    const toolpath::ToolpathValidationReport& report,
    int maxIssues = 5)
{
    QString message = QString::fromStdString(report.summary());
    const int issueCount = static_cast<int>(report.issues.size());
    const int limit = std::min(maxIssues, issueCount);
    int appended = 0;
    const auto appendSeverity =
        [&](toolpath::ToolpathIssueSeverity severity) {
            for (const toolpath::ToolpathIssue& issue : report.issues) {
                if (appended >= limit || issue.severity != severity)
                    continue;
                message += issue.severity ==
                                   toolpath::ToolpathIssueSeverity::Error
                    ? QStringLiteral("\nERROR ")
                    : QStringLiteral("\nWARN ");
                message += QString::fromStdString(issue.code);
                if (!issue.operationId.empty()) {
                    message += QStringLiteral(" [%1]")
                        .arg(QString::fromStdString(issue.operationId));
                }
                if (issue.moveIndex >= 0) {
                    message += QStringLiteral(" move %1")
                        .arg(issue.moveIndex);
                }
                message += QStringLiteral(": %1")
                    .arg(QString::fromStdString(issue.message));
                ++appended;
            }
        };
    appendSeverity(toolpath::ToolpathIssueSeverity::Error);
    appendSeverity(toolpath::ToolpathIssueSeverity::Warning);
    if (issueCount > appended) {
        message += QStringLiteral("\n... %1 more issue(s)")
            .arg(issueCount - appended);
    }
    return message;
}

QString failureCodeText(toolpath::PocketClearingFailureCode code)
{
    using Code = toolpath::PocketClearingFailureCode;
    switch (code) {
        case Code::None: return QStringLiteral("无");
        case Code::AlgorithmUnavailable:
            return QStringLiteral("算法不可用");
        case Code::AlgorithmExecutionFailed:
            return QStringLiteral("算法执行失败");
        case Code::InvalidRegion: return QStringLiteral("区域无效");
        case Code::LayerSectionFailed:
            return QStringLiteral("分层截面算法失败");
        case Code::LayerNoEdges: return QStringLiteral("该层没有截交边");
        case Code::LayerOpenContour:
            return QStringLiteral("该层轮廓未闭合");
        case Code::LayerNoValidArea:
            return QStringLiteral("该层没有有效区域");
        case Code::ToolDoesNotFit: return QStringLiteral("刀具不可达");
        case Code::NoClearableArea:
            return QStringLiteral("没有可清除区域");
    }
    return QStringLiteral("未知失败");
}

struct FailureStats {
    int sectionAlgorithm = 0;
    int noEdges = 0;
    int openContour = 0;
    int noValidArea = 0;
    int toolDoesNotFit = 0;
    int other = 0;

    void add(toolpath::PocketClearingFailureCode code)
    {
        using Code = toolpath::PocketClearingFailureCode;
        switch (code) {
            case Code::LayerSectionFailed: ++sectionAlgorithm; break;
            case Code::LayerNoEdges: ++noEdges; break;
            case Code::LayerOpenContour: ++openContour; break;
            case Code::LayerNoValidArea: ++noValidArea; break;
            case Code::ToolDoesNotFit: ++toolDoesNotFit; break;
            case Code::None:
            case Code::AlgorithmUnavailable:
            case Code::AlgorithmExecutionFailed:
            case Code::InvalidRegion:
            case Code::NoClearableArea: ++other; break;
        }
    }

    QString summary() const
    {
        return QStringLiteral(
            "截面算法失败 %1,无截交边 %2,开放轮廓 %3,"
            "无有效区域 %4,刀具不可达 %5")
            .arg(sectionAlgorithm)
            .arg(noEdges)
            .arg(openContour)
            .arg(noValidArea)
            .arg(toolDoesNotFit);
    }
};

void accumulateOperationMetrics(
    const toolpath::PocketToolpathOperation& operation,
    PocketToolpathGenerationMetrics& metrics)
{
    const toolpath::PocketClearingResult& clearing =
        operation.clearing;
    metrics.algorithmMilliseconds += clearing.totalMilliseconds;
    metrics.sectionMilliseconds += clearing.sectionMilliseconds;
    metrics.patternMilliseconds += clearing.patternMilliseconds;
    metrics.moveAssemblyMilliseconds +=
        clearing.moveAssemblyMilliseconds;
    metrics.movePreparationMilliseconds +=
        clearing.movePreparationMilliseconds;
    metrics.moveLinkingMilliseconds +=
        clearing.moveLinkingMilliseconds;
    metrics.collinearMergedMoveCount +=
        clearing.collinearMergedMoveCount;
    metrics.restTravelReorderedLoopCount +=
        clearing.travelReorderedLoopCount;
    metrics.restTravelAdjustedStartCount +=
        clearing.travelAdjustedStartCount;
    metrics.travelChoiceParallelBatchCount +=
        clearing.travelChoiceParallelBatchCount;
    metrics.travelChoiceParallelTaskCount +=
        clearing.travelChoiceParallelTaskCount;
    metrics.travelChoicePeakParallelism = std::max(
        metrics.travelChoicePeakParallelism,
        clearing.travelChoicePeakParallelism);
    metrics.sectionWireParallelBatchCount +=
        clearing.sectionWireParallelBatchCount;
    metrics.sectionWireParallelTaskCount +=
        clearing.sectionWireParallelTaskCount;
    metrics.sectionWirePeakParallelism = std::max(
        metrics.sectionWirePeakParallelism,
        clearing.sectionWirePeakParallelism);
    metrics.occtInternalParallelCount +=
        clearing.occtInternalParallelCount;
    metrics.occtSerialFallbackCount +=
        clearing.occtSerialFallbackCount;
    if (clearing.usedParallelLayerProcessing)
        ++metrics.parallelLayerRegionCount;
    if (clearing.usedParallelPatternProcessing)
        ++metrics.parallelPatternRegionCount;
    metrics.patternParallelBatchCount +=
        clearing.patternParallelBatchCount;
    metrics.patternParallelTaskCount +=
        clearing.patternParallelTaskCount;
    metrics.patternPeakParallelism = std::max(
        metrics.patternPeakParallelism,
        clearing.patternPeakParallelism);
    int& loopCount = operation.kind == FeatureKind::Slot
        ? metrics.slotLoopCount
        : metrics.pocketLoopCount;
    loopCount += static_cast<int>(clearing.loops.size());
    if (!clearing.warning.empty()) {
        metrics.generationWarnings <<
            QString::fromStdString(clearing.warning);
    }
    for (const toolpath::ToolpathMove& move : clearing.moves) {
        switch (move.type) {
            case toolpath::ToolpathMoveType::Cut:
                ++metrics.fullCutMoveCount;
                break;
            case toolpath::ToolpathMoveType::Rapid:
                ++metrics.fullRapidMoveCount;
                break;
            case toolpath::ToolpathMoveType::Plunge:
            case toolpath::ToolpathMoveType::Retract:
                ++metrics.fullEntryRetractMoveCount;
                break;
        }
    }
    metrics.sectionLayerTotal += clearing.sectionLayerCount;
    if (clearing.usedProtectedMaterialSections) {
        ++metrics.protectedMaterialRegionCount;
        metrics.protectedMaterialLayerTotal +=
            static_cast<int>(
                clearing.protectedMaterialLayers.size());
    }
    metrics.openSectionLayerTotal += clearing.openSectionLayerCount;
    metrics.virtualOpenBoundaryTotal +=
        clearing.virtualOpenBoundaryCount;
    metrics.verticalEntryTotal += clearing.verticalEntryCount;
    metrics.rampOnShapeEntryTotal += clearing.rampOnShapeEntryCount;
    metrics.linearRampEntryTotal += clearing.linearRampEntryCount;
    metrics.zigzagRampEntryTotal += clearing.zigzagRampEntryCount;
    metrics.helicalRampEntryTotal += clearing.helicalRampEntryCount;
    metrics.entryFallbackTotal += clearing.entryFallbackCount;
    metrics.depthLayerTotal += clearing.depthLayerCount;
    metrics.maxDepthLayerCount = std::max(
        metrics.maxDepthLayerCount, clearing.depthLayerCount);
}

std::string pocketInstanceKey(
    const std::vector<std::string>& sourceFaceIds)
{
    std::vector<std::string> sorted = sourceFaceIds;
    std::sort(sorted.begin(), sorted.end());
    sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
    std::string key;
    for (const std::string& faceId : sorted) {
        if (!key.empty())
            key += '|';
        key += faceId;
    }
    return key;
}

QString buildSkipSummary(
    const std::vector<toolpath::SkippedRegion>& skipped)
{
    if (skipped.empty())
        return {};
    QString summary = QStringLiteral(" 另有 %1 个内环结构未生成:")
        .arg(static_cast<int>(skipped.size()));
    const int shown = std::min(3, static_cast<int>(skipped.size()));
    for (int index = 0; index < shown; ++index) {
        if (index > 0)
            summary += QStringLiteral(";");
        summary += diagnosticReasonText(
            skipped[static_cast<std::size_t>(index)].diagnostic);
    }
    if (static_cast<int>(skipped.size()) > shown)
        summary += QStringLiteral(" 等");
    summary += QStringLiteral("。");
    return summary;
}

void appendInitialReport(
    QStringList& report,
    const PocketToolpathGenerationRequest& request)
{
    const toolpath::ToolpathParams& params = *request.params;
    report
        << QStringLiteral("OpenAutoPath 腔清腔诊断日志")
        << QStringLiteral("时间: %1")
               .arg(QDateTime::currentDateTime().toString(Qt::ISODateWithMs))
        << QStringLiteral("识别来源: 主外表面内环追踪结构")
        << QStringLiteral("候选主外表面内环结构数: %1")
               .arg(request.candidateCount)
        << QString()
        << QStringLiteral("参数快照:")
        << QStringLiteral("  清腔方式: %1")
               .arg(clearingPatternText(params.clearing.pattern))
        << QStringLiteral("  清腔扩展方向: %1")
               .arg(expansionDirectionText(
                   toolpath::effectivePocketClearingDirection(
                       params.clearing.pattern,
                       params.clearing.expansionDirection)))
        << QStringLiteral("  下刀方式: %1")
               .arg(entryModeText(params.clearing.entryMode))
        << QStringLiteral("  最大斜坡角: %1°")
               .arg(params.clearing.rampAngleDeg, 0, 'f', 2)
        << QStringLiteral("  最大斜坡长度: %1 mm")
               .arg(params.clearing.maximumRampLength, 0, 'f', 3)
        << QStringLiteral("  刀心螺旋直径: %1 mm")
               .arg(params.clearing.helixDiameter, 0, 'f', 3)
        << QStringLiteral("  刀具直径: %1 mm")
               .arg(params.tool.diameter, 0, 'f', 3)
        << QStringLiteral("  步距: %1 %")
               .arg(params.clearing.stepOverPercent, 0, 'f', 1)
        << QStringLiteral("  清腔额外偏置: %1 mm")
               .arg(params.clearing.extraOffset, 0, 'f', 3)
        << QStringLiteral("  精修留量: %1 mm")
               .arg(params.depth.finishDepth, 0, 'f', 3)
        << QStringLiteral("  每层切深: %1 mm")
               .arg(params.depth.stepDown, 0, 'f', 3)
        << QStringLiteral("  安全偏移（相对下刀面）: %1 mm")
               .arg(params.depth.safeHeight, 0, 'f', 3)
        << QStringLiteral("  抬刀间隙（相对下刀面）: %1 mm")
               .arg(params.depth.clearanceHeight, 0, 'f', 3);
}

void appendRegionReport(
    QStringList& report,
    const PocketToolpathGenerationRequest& request)
{
    report << QString()
           << QStringLiteral("区域构建阶段:")
           << QStringLiteral("  耗时: %1 ms (%2)")
                  .arg(request.regionBuildMilliseconds)
                  .arg(request.regionCacheHit
                           ? QStringLiteral("命中缓存")
                           : QStringLiteral("重新构建"))
           << QStringLiteral("  成功区域: %1")
                  .arg(static_cast<int>(request.regions->size()))
           << QStringLiteral("  跳过实例: %1")
                  .arg(static_cast<int>(request.buildSkipped->size()));
    for (int index = 0;
         index < static_cast<int>(request.buildSkipped->size()); ++index) {
        const toolpath::SkippedRegion& skipped =
            (*request.buildSkipped)[static_cast<std::size_t>(index)];
        QStringList faces;
        for (const std::string& faceId : skipped.sourceFaceIds)
            faces << QString::fromStdString(faceId);
        report << QStringLiteral(
                      "  跳过 %1: 类型=%2, 面=[%3], 原因=%4, 原始信息=%5")
                      .arg(index + 1)
                      .arg(featureKindText(skipped.kind))
                      .arg(faces.join(QStringLiteral(",")))
                      .arg(diagnosticReasonText(skipped.diagnostic))
                      .arg(QString::fromStdString(skipped.reason));
    }
    for (int index = 0;
         index < static_cast<int>(request.regions->size()); ++index) {
        const toolpath::PocketRegion& region =
            (*request.regions)[static_cast<std::size_t>(index)];
        report << QStringLiteral(
                      "  区域 %1: 主外表面=%2, 截面结构面数=%3, "
                      "边界方式=%4, 连通通结构面=%5")
                      .arg(index + 1)
                      .arg(QString::fromStdString(region.floorFaceId))
                      .arg(region.sectionFaceCount)
                      .arg(region.allowOpenSectionContours
                               ? QStringLiteral(
                                     "每层真实截面，通槽出口虚拟封口")
                               : QStringLiteral("每层真实截面内环"))
                      .arg(region.openBoundaryFaceCount);
    }
}

void appendAutomaticPlanningAudit(
    QStringList& report,
    const tooling::PocketAutomaticToolpathResult& automatic)
{
    if (automatic.stageAudits.empty())
        return;
    const int usableCount = static_cast<int>(std::count_if(
        automatic.stageAudits.begin(), automatic.stageAudits.end(),
        [](const tooling::PocketAutomaticToolpathStageAudit& audit) {
            return audit.usable;
        }));
    const int reusedCount = static_cast<int>(std::count_if(
        automatic.stageAudits.begin(), automatic.stageAudits.end(),
        [](const tooling::PocketAutomaticToolpathStageAudit& audit) {
            return audit.reusedPlanningEvaluation;
        }));
    std::size_t patternParallelBatchCount = 0u;
    std::size_t patternParallelTaskCount = 0u;
    std::size_t patternPeakParallelism = 1u;
    std::size_t travelChoiceParallelBatchCount = 0u;
    std::size_t travelChoiceParallelTaskCount = 0u;
    std::size_t travelChoicePeakParallelism = 1u;
    std::size_t sectionWireParallelBatchCount = 0u;
    std::size_t sectionWireParallelTaskCount = 0u;
    std::size_t sectionWirePeakParallelism = 1u;
    std::size_t occtInternalParallelCount = 0u;
    std::size_t occtSerialFallbackCount = 0u;
    for (const tooling::PocketAutomaticToolpathStageAudit& audit :
         automatic.stageAudits) {
        patternParallelBatchCount +=
            audit.patternParallelBatchCount;
        patternParallelTaskCount +=
            audit.patternParallelTaskCount;
        patternPeakParallelism = std::max(
            patternPeakParallelism,
            audit.patternPeakParallelism);
        travelChoiceParallelBatchCount +=
            audit.travelChoiceParallelBatchCount;
        travelChoiceParallelTaskCount +=
            audit.travelChoiceParallelTaskCount;
        travelChoicePeakParallelism = std::max(
            travelChoicePeakParallelism,
            audit.travelChoicePeakParallelism);
        sectionWireParallelBatchCount +=
            audit.sectionWireParallelBatchCount;
        sectionWireParallelTaskCount +=
            audit.sectionWireParallelTaskCount;
        sectionWirePeakParallelism = std::max(
            sectionWirePeakParallelism,
            audit.sectionWirePeakParallelism);
        occtInternalParallelCount +=
            audit.occtInternalParallelCount;
        occtSerialFallbackCount +=
            audit.occtSerialFallbackCount;
    }
    report << QString()
           << QStringLiteral("自动多刀候选阶段审计:")
           << QStringLiteral("  已计算阶段: %1，可用 %2，淘汰 %3")
                  .arg(static_cast<int>(automatic.stageAudits.size()))
                  .arg(usableCount)
                  .arg(static_cast<int>(automatic.stageAudits.size()) -
                       usableCount)
           << QStringLiteral("  重规划轻量复用阶段: %1，按需物化 %2")
                  .arg(reusedCount)
                  .arg(automatic.selectedStageMaterializationCount)
           << QStringLiteral(
                  "  Offset 圈并行: 批次 %1，圈任务 %2，峰值并行 %3")
                  .arg(static_cast<qulonglong>(
                      patternParallelBatchCount))
                  .arg(static_cast<qulonglong>(
                      patternParallelTaskCount))
                  .arg(static_cast<qulonglong>(
                      patternPeakParallelism))
           << QStringLiteral(
                  "  余料访问候选并行: 批次 %1，候选任务 %2，峰值并行 %3")
                  .arg(static_cast<qulonglong>(
                      travelChoiceParallelBatchCount))
                  .arg(static_cast<qulonglong>(
                      travelChoiceParallelTaskCount))
                   .arg(static_cast<qulonglong>(
                       travelChoicePeakParallelism))
           << QStringLiteral(
                  "  截面 wire 并行: 批次 %1，wire 任务 %2，峰值并行 %3")
                  .arg(static_cast<qulonglong>(
                      sectionWireParallelBatchCount))
                  .arg(static_cast<qulonglong>(
                      sectionWireParallelTaskCount))
                  .arg(static_cast<qulonglong>(
                      sectionWirePeakParallelism))
           << QStringLiteral(
                  "  OCCT 单截面内部并行: %1 次，串行回退 %2 次")
                  .arg(static_cast<qulonglong>(
                      occtInternalParallelCount))
                  .arg(static_cast<qulonglong>(
                      occtSerialFallbackCount))
           << QStringLiteral(
                  "  候选累计耗时: 全量 %1 ms，余料 %2 ms；清腔 %3 ms"
                  "（截面 %4，图案 %5，移动 %6 [准备 %7，连接 %8]）；"
                  "结构校验 %9 ms，安全校验 %10 ms")
                  .arg(automatic.fullStageGenerationMilliseconds,
                       0, 'f', 1)
                  .arg(automatic.transitionStageGenerationMilliseconds,
                       0, 'f', 1)
                  .arg(automatic.candidateClearingMilliseconds,
                       0, 'f', 1)
                  .arg(automatic.candidateSectionMilliseconds,
                       0, 'f', 1)
                  .arg(automatic.candidatePatternMilliseconds,
                       0, 'f', 1)
                  .arg(automatic.candidateMoveAssemblyMilliseconds,
                       0, 'f', 1)
                  .arg(automatic.candidateMovePreparationMilliseconds,
                       0, 'f', 1)
                  .arg(automatic.candidateMoveLinkingMilliseconds,
                       0, 'f', 1)
                  .arg(automatic.candidateStructuralValidationMilliseconds,
                       0, 'f', 1)
                  .arg(automatic.planningSafetyValidationMilliseconds,
                       0, 'f', 1)
           << QStringLiteral(
                  "  单阶段长尾: 最慢候选 %1 ms，最慢安全校验 %2 ms")
                  .arg(automatic.maximumStageGenerationMilliseconds,
                       0, 'f', 1)
                  .arg(automatic.maximumSafetyValidationMilliseconds,
                       0, 'f', 1);
    for (const tooling::PocketAutomaticToolpathStageAudit& audit :
         automatic.stageAudits) {
        QStringList auditedRegions;
        for (const int regionIndex : audit.regionIndices) {
            auditedRegions.push_back(
                QString::number(regionIndex + 1));
        }
        const bool rest = audit.previousToolDiameter >
            audit.tool.diameter + 1.0e-6;
        report << QStringLiteral(
                      "  T%1 D%2 / %3 / %4: 工序 %5，失败区域 %6，"
                      "无需余料 %7，验证 %8E/%9W，移动 %10 mm；"
                      "耗时 %11 ms（清腔 %12：截面 %13，图案 %14，"
                      "移动 %15；结构校验 %16，安全 %17）")
                      .arg(audit.tool.toolNumber)
                      .arg(audit.tool.diameter, 0, 'f', 3)
                      .arg(rest
                               ? QStringLiteral("D%1 后余料")
                                     .arg(audit.previousToolDiameter,
                                          0, 'f', 3)
                               : QStringLiteral("单刀全量"))
                      .arg(audit.usable
                               ? (audit.reusedPlanningEvaluation
                                      ? QStringLiteral("可用/复用")
                                      : QStringLiteral("可用"))
                               : (audit.reusedPlanningEvaluation
                                      ? QStringLiteral("淘汰/复用")
                                      : QStringLiteral("淘汰")))
                      .arg(audit.operationCount)
                      .arg(audit.failedRegionCount)
                      .arg(audit.noRestMaterialRegionCount)
                      .arg(audit.validationErrorCount)
                      .arg(audit.validationWarningCount)
                      .arg(audit.movementDistance, 0, 'f', 3)
                      .arg(audit.generationMilliseconds, 0, 'f', 1)
                      .arg(audit.clearingMilliseconds, 0, 'f', 1)
                      .arg(audit.sectionMilliseconds, 0, 'f', 1)
                      .arg(audit.patternMilliseconds, 0, 'f', 1)
                      .arg(audit.moveAssemblyMilliseconds, 0, 'f', 1)
                      .arg(
                          audit.structuralValidationMilliseconds,
                          0, 'f', 1)
                      .arg(audit.safetyValidationMilliseconds, 0, 'f', 1);
        if (!auditedRegions.empty()) {
            report << QStringLiteral("    区域: %1").arg(
                auditedRegions.join(QStringLiteral(", ")));
        }
        if (audit.usable)
            continue;
        const std::size_t detailLimit = std::min<std::size_t>(
            3u, audit.failureDetails.size());
        for (std::size_t index = 0; index < detailLimit; ++index) {
            report << QStringLiteral("    - %1").arg(
                QString::fromStdString(audit.failureDetails[index]));
        }
        if (audit.failureDetails.size() > detailLimit) {
            report << QStringLiteral("    - 其余 %1 项省略")
                          .arg(static_cast<qulonglong>(
                              audit.failureDetails.size() - detailLimit));
        }
    }

    std::vector<const tooling::PocketAutomaticToolpathStageAudit*>
        slowStages;
    slowStages.reserve(automatic.stageAudits.size());
    for (const auto& audit : automatic.stageAudits)
        slowStages.push_back(&audit);
    std::stable_sort(
        slowStages.begin(), slowStages.end(),
        [](const auto* left, const auto* right) {
            return left->generationMilliseconds >
                right->generationMilliseconds;
        });
    report << QStringLiteral("  最慢候选阶段（最多 8 项）:");
    const std::size_t slowStageLimit = std::min<std::size_t>(
        8u, slowStages.size());
    for (std::size_t index = 0; index < slowStageLimit; ++index) {
        const auto& audit = *slowStages[index];
        QStringList auditedRegions;
        for (const int regionIndex : audit.regionIndices)
            auditedRegions.push_back(QString::number(regionIndex + 1));
        const bool rest = audit.previousToolDiameter >
            audit.tool.diameter + 1.0e-6;
        report << QStringLiteral(
                      "    %1. 区域 %2 / T%3 D%4 / %5：总 %6 ms，"
                      "截面 %7，图案 %8，移动 %9，安全 %10 ms")
                      .arg(static_cast<qulonglong>(index + 1u))
                      .arg(auditedRegions.join(QStringLiteral(", ")))
                      .arg(audit.tool.toolNumber)
                      .arg(audit.tool.diameter, 0, 'f', 3)
                      .arg(rest
                               ? QStringLiteral("D%1 后余料")
                                     .arg(audit.previousToolDiameter,
                                          0, 'f', 3)
                               : QStringLiteral("单刀全量"))
                      .arg(audit.generationMilliseconds, 0, 'f', 1)
                      .arg(audit.sectionMilliseconds, 0, 'f', 1)
                      .arg(audit.patternMilliseconds, 0, 'f', 1)
                      .arg(audit.moveAssemblyMilliseconds, 0, 'f', 1)
                      .arg(audit.safetyValidationMilliseconds, 0, 'f', 1);
    }
}

bool generatePocketBatch(const PocketToolpathGenerationRequest& request,
                         PocketToolpathGenerationResult& result)
{
    QElapsedTimer timer;
    timer.start();
    if (!request.automaticLibrary) {
        result.error = QStringLiteral("槽腔自动加工需要活动刀库。");
        return false;
    }
    result.automatic = request.singleTool == nullptr;
    if (request.automaticLibrary) {
        result.automaticSelection =
            generateAutomaticPocketsWithProgress(
                request.interaction.showProgress,
                *request.automaticLibrary,
                *request.regions, *request.params, "VIEW");
        result.batch = result.automaticSelection.batch;
    }
    result.metrics.batchMilliseconds = timer.elapsed();
    if (request.automaticLibrary) {
        appendAutomaticPlanningAudit(
            result.reportLines, result.automaticSelection);
    }
    if (result.batch.cancelled ||
        result.automaticSelection.cancelled) {
        result.failure = PocketToolpathGenerationFailure::Cancelled;
        result.error = QStringLiteral("槽/腔刀路生成已取消。");
        result.reportLines
            << QStringLiteral("刀路生成耗时: %1 ms")
                   .arg(result.metrics.batchMilliseconds)
            << QStringLiteral("结果: 用户取消了刀路生成。");
        return false;
    }
    if (request.automaticLibrary &&
        !result.automaticSelection.error.empty()) {
        result.failure = PocketToolpathGenerationFailure::EmptyProgram;
        result.error = QString::fromStdString(result.automaticSelection.error);
        result.reportLines << QStringLiteral("刀具规划失败: %1").arg(result.error);
        return false;
    }

    return true;
}

void appendAutomaticSelectionReport(const PocketToolpathGenerationRequest& request,
                                    PocketToolpathGenerationResult& result)
{
    if (request.automaticLibrary) {
        result.reportLines
            << QStringLiteral(
                   "刀库多刀开粗: %1，候选铣刀 %2 把，并发预算 %3，"
                   "计算转换 %4 次，剪枝 %5 次，区域并行启动 %6 次，"
                   "转换并行批次 %7 次")
                   .arg(request.automaticLibrary->name())
                   .arg(result.automaticSelection.candidateToolCount)
                   .arg(result.automaticSelection.planningParallelism)
                   .arg(result.automaticSelection
                            .evaluatedTransitionCount)
                   .arg(result.automaticSelection
                            .prunedTransitionCount)
                    .arg(result.automaticSelection
                             .parallelRegionBatchCount)
                     .arg(result.automaticSelection
                              .parallelTransitionBatchCount)
            << QStringLiteral(
                   "  自适应并发: 初始 %1，峰值 %2，结束 %3，调整 %4 次；"
                   "CPU 上限 %5，内存初始上限 %6，启动时可用内存 %7 MiB")
                   .arg(result.automaticSelection.planningParallelism)
                   .arg(result.automaticSelection
                            .planningPeakParallelism)
                   .arg(result.automaticSelection
                            .planningFinalParallelism)
                   .arg(result.automaticSelection
                            .planningConcurrencyAdjustmentCount)
                   .arg(result.automaticSelection
                            .planningCpuParallelism)
                   .arg(result.automaticSelection
                            .planningMemoryParallelism)
                   .arg(static_cast<qulonglong>(
                       result.automaticSelection
                           .planningAvailableMemoryMiB))
            << QStringLiteral(
                   "  槽腔内存估算: 平滑值 %1 MiB，最大观测增量 %2 MiB")
                   .arg(static_cast<qulonglong>(
                       result.automaticSelection
                           .planningEstimatedRegionMemoryMiB))
                   .arg(static_cast<qulonglong>(
                       result.automaticSelection
                           .planningObservedPeakRegionMemoryMiB))
            << QStringLiteral(
                   "  共享令牌: 总峰值 %1，区域峰值 %2，累计借用 %3 个")
                   .arg(result.automaticSelection
                            .planningPeakParallelism)
                   .arg(result.automaticSelection
                            .planningPeakRegionParallelism)
                   .arg(result.automaticSelection
                            .planningBorrowedTokenCount)
            << QStringLiteral(
                   "  候选阶段: 实际生成 %1，重规划轻量复用 %2，"
                   "按需物化 %3；全量并行批次 %4，预取 %5，"
                   "转换并行批次 %6，安全校验 %7，安全拒绝 %8，重规划 %9")
                   .arg(result.automaticSelection.generatedStageCount)
                   .arg(result.automaticSelection
                            .reusedStageEvaluationCount)
                   .arg(result.automaticSelection
                            .selectedStageMaterializationCount)
                   .arg(result.automaticSelection
                            .parallelFullStageBatchCount)
                   .arg(result.automaticSelection
                            .prefetchedFullStageCount)
                   .arg(result.automaticSelection
                            .parallelTransitionBatchCount)
                   .arg(result.automaticSelection
                            .safetyValidatedStageCount)
                   .arg(result.automaticSelection
                            .safetyRejectedStageCount)
                   .arg(result.automaticSelection.planningReplanCount)
            << QStringLiteral(
                   "  最小刀单独加工切削距离: %1 mm")
                   .arg(result.automaticSelection
                            .baselineSmallToolCuttingDistance,
                        0, 'f', 3)
            << QStringLiteral(
                   "  入选组合切削距离: %1 mm")
                   .arg(result.automaticSelection
                            .selectedCuttingDistance,
                        0, 'f', 3);
        for (const tooling::PocketAutomaticToolpathStage& stage :
             result.automaticSelection.stages) {
            result.reportLines << QStringLiteral(
                "  T%1 %2 / D%3 / 上一刀 D%4 / 移动 %5 mm / 切削 %6 mm / 工序 %7")
                .arg(stage.tool.toolNumber)
                .arg(QString::fromStdString(stage.tool.toolName))
                .arg(stage.tool.diameter, 0, 'f', 3)
                .arg(stage.previousToolDiameter, 0, 'f', 3)
                .arg(stage.movementDistance, 0, 'f', 3)
                .arg(stage.cuttingDistance, 0, 'f', 3)
                .arg(stage.operationCount);
        }
    }
}

FailureStats collectGenerationMetrics(const PocketToolpathGenerationRequest& request,
                                      PocketToolpathGenerationResult& result)
{
    FailureStats failures;
    for (int regionIndex = 0;
         regionIndex < static_cast<int>(request.regions->size());
         ++regionIndex) {
        const toolpath::PocketRegion& region =
            (*request.regions)[static_cast<std::size_t>(regionIndex)];
        const toolpath::PocketClearingResult* clearing =
            result.batch.clearingForRegion(
                static_cast<std::size_t>(regionIndex));
        if (!clearing) {
            ++result.metrics.skippedRegions;
            ++failures.other;
            result.metrics.generationDetails
                << QStringLiteral(
                       "  区域 %1 失败: 批处理未返回对应结果")
                       .arg(regionIndex + 1);
            continue;
        }
        if (!clearing->ok || clearing->moves.empty()) {
            if (clearing->ok && clearing->noRestMaterial) {
                ++result.metrics.generatedRegions;
                continue;
            }
            ++result.metrics.skippedRegions;
            if (clearing->firstLayerPreflightRejected)
                ++result.metrics.firstLayerPreflightRejectedCount;
            failures.add(clearing->failureCode);
            QStringList sourceFaces;
            for (const std::string& faceId : region.sourceFaceIds)
                sourceFaces << QString::fromStdString(faceId);
            QString detail = QStringLiteral(
                "  区域 %1 失败: 类型=%2, floor=%3, 源面=[%4], 代码=%5")
                .arg(regionIndex + 1)
                .arg(featureKindText(region.kind))
                .arg(QString::fromStdString(region.floorFaceId))
                .arg(sourceFaces.join(QStringLiteral(",")))
                .arg(failureCodeText(clearing->failureCode));
            detail += QStringLiteral(
                ", 截面面数=%1, 已过滤非连通面=%2")
                .arg(region.sectionFaceCount)
                .arg(region.filteredDisconnectedFaces
                         ? QStringLiteral("是")
                         : QStringLiteral("否"));
            if (clearing->failedDepthIndex >= 0) {
                detail += QStringLiteral(", 层=%1, 深度偏移=%2 mm")
                    .arg(clearing->failedDepthIndex + 1)
                    .arg(clearing->failedDepthOffset, 0, 'f', 3);
            }
            detail += QStringLiteral(", 原始信息=%1")
                .arg(QString::fromStdString(clearing->error));
            result.metrics.generationDetails << detail;
            continue;
        }

        ++result.metrics.generatedRegions;
    }
    for (const toolpath::PocketToolpathOperation& operation :
         result.batch.operations) {
        accumulateOperationMetrics(operation, result.metrics);
    }
    result.metrics.failureSummary = failures.summary();

    return failures;
}

void appendGenerationMetricsReport(PocketToolpathGenerationResult& result)
{
    result.reportLines
        << QString()
        << QStringLiteral("刀路生成阶段:")
        << QStringLiteral("  批处理墙钟耗时: %1 ms")
               .arg(result.metrics.batchMilliseconds)
        << QStringLiteral(
               "  算法累计耗时: %1 ms (截面 %2, 图案 %3, 移动组装 %4)")
               .arg(result.metrics.algorithmMilliseconds, 0, 'f', 1)
               .arg(result.metrics.sectionMilliseconds, 0, 'f', 1)
               .arg(result.metrics.patternMilliseconds, 0, 'f', 1)
               .arg(result.metrics.moveAssemblyMilliseconds, 0, 'f', 1)
        << QStringLiteral(
               "  移动组装明细: 曲线准备 %1 ms, 连接/写入 %2 ms, "
               "严格共线合并 %3 段")
               .arg(result.metrics.movePreparationMilliseconds, 0, 'f', 1)
               .arg(result.metrics.moveLinkingMilliseconds, 0, 'f', 1)
               .arg(static_cast<qulonglong>(
                   result.metrics.collinearMergedMoveCount))
        << QStringLiteral(
               "  余料抬跳刀优化: 重排路径 %1 条, 调整闭环起点 %2 条")
               .arg(static_cast<qulonglong>(
                   result.metrics.restTravelReorderedLoopCount))
               .arg(static_cast<qulonglong>(
                   result.metrics.restTravelAdjustedStartCount))
        << QStringLiteral(
               "  余料访问候选并行: 批次 %1, 候选任务 %2, 峰值并行 %3")
               .arg(static_cast<qulonglong>(
                   result.metrics.travelChoiceParallelBatchCount))
               .arg(static_cast<qulonglong>(
                   result.metrics.travelChoiceParallelTaskCount))
               .arg(static_cast<qulonglong>(
                   result.metrics.travelChoicePeakParallelism))
        << QStringLiteral(
               "  截面 wire 并行: 批次 %1, wire 任务 %2, 峰值并行 %3")
               .arg(static_cast<qulonglong>(
                   result.metrics.sectionWireParallelBatchCount))
               .arg(static_cast<qulonglong>(
                   result.metrics.sectionWireParallelTaskCount))
               .arg(static_cast<qulonglong>(
                   result.metrics.sectionWirePeakParallelism))
        << QStringLiteral(
               "  OCCT 单截面内部并行: %1 次, 串行回退 %2 次")
               .arg(static_cast<qulonglong>(
                   result.metrics.occtInternalParallelCount))
               .arg(static_cast<qulonglong>(
                   result.metrics.occtSerialFallbackCount))
        << QStringLiteral("  验证耗时: 结构 %1 ms, 安全 %2 ms")
               .arg(result.batch.structuralValidationMilliseconds, 0, 'f', 1)
               .arg(result.batch.safetyValidationMilliseconds, 0, 'f', 1)
        << QStringLiteral("  启用逐层并行的区域: %1")
               .arg(result.metrics.parallelLayerRegionCount)
        << QStringLiteral(
               "  Offset 圈并行: 区域 %1, 批次 %2, 圈任务 %3, 峰值并行 %4")
               .arg(result.metrics.parallelPatternRegionCount)
               .arg(static_cast<qulonglong>(
                   result.metrics.patternParallelBatchCount))
               .arg(static_cast<qulonglong>(
                   result.metrics.patternParallelTaskCount))
               .arg(static_cast<qulonglong>(
                   result.metrics.patternPeakParallelism))
        << QStringLiteral("  成功区域: %1")
               .arg(result.metrics.generatedRegions)
        << QStringLiteral("  失败区域: %1")
               .arg(result.metrics.skippedRegions)
        << QStringLiteral("  %1").arg(result.metrics.failureSummary)
        << QStringLiteral("  环数量: %1 (槽 %2, 腔 %3)")
               .arg(result.metrics.totalLoops())
               .arg(result.metrics.slotLoopCount)
               .arg(result.metrics.pocketLoopCount)
        << QStringLiteral(
               "  深度层累计: %1, 最大层数: %2, 真实截面层: %3")
               .arg(result.metrics.depthLayerTotal)
               .arg(result.metrics.maxDepthLayerCount)
               .arg(result.metrics.sectionLayerTotal)
        << QStringLiteral(
               "  实体材料保护: 区域 %1, 轴向材料阴影层 %2")
               .arg(result.metrics.protectedMaterialRegionCount)
               .arg(result.metrics.protectedMaterialLayerTotal)
        << QStringLiteral("  首层预检提前淘汰: %1")
               .arg(result.metrics.firstLayerPreflightRejectedCount)
        << QStringLiteral("  实体截面缓存: 命中 %1, 未命中 %2, 条目 %3")
               .arg(result.batch.materialSectionCacheHits)
               .arg(result.batch.materialSectionCacheMisses)
               .arg(result.batch.materialSectionCacheEntries)
        << QStringLiteral(
               "  通槽开口截面层: %1, 虚拟出口边界累计: %2")
               .arg(result.metrics.openSectionLayerTotal)
               .arg(result.metrics.virtualOpenBoundaryTotal)
        << QStringLiteral(
               "  下刀实际使用: 垂直 %1, 沿轮廓 %2, 直线 %3, "
               "往复 %4, 螺旋 %5, 安全回退 %6")
               .arg(result.metrics.verticalEntryTotal)
               .arg(result.metrics.rampOnShapeEntryTotal)
               .arg(result.metrics.linearRampEntryTotal)
               .arg(result.metrics.zigzagRampEntryTotal)
               .arg(result.metrics.helicalRampEntryTotal)
               .arg(result.metrics.entryFallbackTotal)
        << QStringLiteral(
               "  完整移动段: %1 (切削 %2, Rapid %3, 进退刀 %4)")
               .arg(result.metrics.fullTotalMoves())
               .arg(result.metrics.fullCutMoveCount)
               .arg(result.metrics.fullRapidMoveCount)
               .arg(result.metrics.fullEntryRetractMoveCount);
    if (result.metrics.generationDetails.isEmpty())
        result.reportLines << QStringLiteral("  失败/警告明细: 无");
    else
        result.reportLines << QStringLiteral("  失败/警告明细:")
                           << result.metrics.generationDetails;

}

bool validateGeneratedCandidate(PocketToolpathGenerationResult& result)
{
    result.program = result.batch.toProgram();
    result.validation = validateToolpathCandidate(
        result.batch.validation, result.program,
        result.batch.effectiveParams);
    const toolpath::ToolpathValidationReport validation =
        result.validation.combined();
    result.reportLines << QString()
                       << QStringLiteral("验证阶段:")
                       << QStringLiteral("  错误: %1, 警告: %2")
                              .arg(validation.errorCount)
                              .arg(validation.warningCount);
    if (validation.issues.empty())
        result.reportLines << QStringLiteral("  验证问题: 无");
    else
        result.reportLines << validationMessage(
            validation, static_cast<int>(validation.issues.size()));
    if (!validation.ok()) {
        result.failure =
            PocketToolpathGenerationFailure::ValidationFailed;
        result.error = QStringLiteral("腔清腔刀路验证失败:\n") +
            validationMessage(validation);
        result.reportLines << QStringLiteral(
            "结果: 腔清腔刀路验证失败。");
        return false;
    }

    return true;
}

bool prepareCandidateJob(const PocketToolpathGenerationRequest& request,
                         PocketToolpathGenerationResult& result)
{
    PocketCandidateJobRequest candidateRequest;
    candidateRequest.currentJob = request.currentJob;
    candidateRequest.definition = request.definition;
    candidateRequest.batch = &result.batch;
    candidateRequest.rootSources = request.rootSources;
    candidateRequest.setupFrames = request.setupFrames;
    result.candidateJob = PocketToolpathWorkflowController::prepareCandidateJob(candidateRequest);
    if (!result.candidateJob.ok) {
        result.failure = PocketToolpathGenerationFailure::CommitFailed;
        result.error = QStringLiteral("槽腔加工项目提交失败：\n%1")
            .arg(result.candidateJob.error);
        result.reportLines << QStringLiteral("结果: %1").arg(result.error);
        return false;
    }

    return true;
}

bool prepareCandidatePreview(const PocketToolpathGenerationRequest& request,
                             const PocketPreviewPublisher& previewPublisher,
                             PocketToolpathGenerationResult& result)
{
    QElapsedTimer timer;
    timer.start();
    if (!PocketToolpathGenerationUseCase::publishPreview(
            result.candidateJob.job.pocketResult()->operations, result.batch.effectiveParams,
            request.previewCutMoveBudget, previewPublisher,
            result.previewEntries, result.metrics)) {
        result.metrics.previewMilliseconds = timer.elapsed();
        result.failure = PocketToolpathGenerationFailure::PreviewFailed;
        result.error = QStringLiteral(
            "刀路已生成并验证通过，但预览未完整构造，未提交新加工任务。");
        result.reportLines
            << QString()
            << QStringLiteral("预览阶段:")
            << QStringLiteral("  耗时: %1 ms")
                   .arg(result.metrics.previewMilliseconds)
            << QStringLiteral("  预览构造失败或没有有效预览段。")
            << QStringLiteral(
                   "结果: 刀路生成并验证通过，但预览构造失败。");
        return false;
    }
    result.metrics.previewMilliseconds = timer.elapsed();

    return true;
}

void appendSuccessfulGenerationReport(const PocketToolpathGenerationRequest& request,
                                      PocketToolpathGenerationResult& result)
{
    const auto validation = result.validation.combined();
    const toolpath::ToolpathParams& params = result.batch.effectiveParams;
    result.summary = QStringLiteral(
        "已生成槽/腔分层%1清腔(主外表面内环):区域 %2/%3,"
        "环 %4 个(槽 %5,腔 %6)。")
        .arg(clearingPatternText(params.clearing.pattern))
        .arg(result.metrics.generatedRegions)
        .arg(static_cast<int>(request.regions->size()))
        .arg(result.metrics.totalLoops())
        .arg(result.metrics.slotLoopCount)
        .arg(result.metrics.pocketLoopCount);
    result.summary += QStringLiteral(
        " 完整移动段 %1:切削 %2,快速 %3,进退刀 %4。")
        .arg(result.metrics.fullTotalMoves())
        .arg(result.metrics.fullCutMoveCount)
        .arg(result.metrics.fullRapidMoveCount)
        .arg(result.metrics.fullEntryRetractMoveCount);
    result.summary += QStringLiteral(
        " 下刀:垂直 %1,沿轮廓 %2,直线 %3,往复 %4,螺旋 %5,回退 %6。")
        .arg(result.metrics.verticalEntryTotal)
        .arg(result.metrics.rampOnShapeEntryTotal)
        .arg(result.metrics.linearRampEntryTotal)
        .arg(result.metrics.zigzagRampEntryTotal)
        .arg(result.metrics.helicalRampEntryTotal)
        .arg(result.metrics.entryFallbackTotal);
    if (result.metrics.previewLimited) {
        result.summary += QStringLiteral(
            " 螺旋预览已抽稀:显示 %1/%2 个切削段。")
            .arg(result.metrics.cutMoveCount)
            .arg(result.metrics.fullCutMoveCount);
    }
    if (result.metrics.skippedDisplaySegments > 0) {
        result.summary += QStringLiteral(" 跳过无效显示段 %1。")
            .arg(result.metrics.skippedDisplaySegments);
    }
    result.summary += QStringLiteral(" 深度层累计 %1,最大 %2。")
        .arg(result.metrics.depthLayerTotal)
        .arg(result.metrics.maxDepthLayerCount);
    result.summary += QStringLiteral(" 真实截面层 %1;%2。")
        .arg(result.metrics.sectionLayerTotal)
        .arg(result.metrics.failureSummary);
    result.summary += QStringLiteral(
        " 刀具直径 %1mm,步距 %2%,step-down %3mm。")
        .arg(params.tool.diameter, 0, 'f', 3)
        .arg(params.clearing.stepOverPercent, 0, 'f', 1)
        .arg(params.depth.stepDown, 0, 'f', 3);
    if (request.automaticLibrary) {
        QStringList tools;
        for (const tooling::PocketAutomaticToolpathStage& stage :
             result.automaticSelection.stages) {
            tools << QStringLiteral("T%1(D%2)")
                         .arg(stage.tool.toolNumber)
                         .arg(stage.tool.diameter, 0, 'f', 3);
        }
        result.summary += QStringLiteral(
            " 自动开粗组合 %1，总切削 %2mm（最小刀单独 %3mm）。")
            .arg(tools.join(QStringLiteral("→")))
            .arg(result.automaticSelection.selectedCuttingDistance,
                 0, 'f', 3)
            .arg(result.automaticSelection
                     .baselineSmallToolCuttingDistance,
                 0, 'f', 3);
    }
    if (validation.warningCount > 0) {
        result.summary += QStringLiteral(" 验证警告 %1。")
            .arg(validation.warningCount);
    }
    if (!result.metrics.generationWarnings.isEmpty()) {
        result.summary += QStringLiteral(" 生成警告 %1。")
            .arg(result.metrics.generationWarnings.size());
    }
    if (std::any_of(result.batch.operations.begin(), result.batch.operations.end(),
            [](const auto& operation) { return operation.tool.cornerRadius > 0.0; }))
        result.summary += QStringLiteral(" 球刀或圆鼻刀可能在槽底、根部留下余料，请使用 IPW 检查。");
    result.summary += result.metrics.skipSummary;

    result.reportLines
        << QString()
        << QStringLiteral("预览阶段:")
        << QStringLiteral("  耗时: %1 ms")
               .arg(result.metrics.previewMilliseconds)
        << QStringLiteral(
               "  显示移动段: %1 (切削 %2, Rapid %3, 进退刀 %4)")
               .arg(result.metrics.displayTotalMoves())
               .arg(result.metrics.cutMoveCount)
               .arg(result.metrics.rapidMoveCount)
               .arg(result.metrics.entryRetractMoveCount)
        << QStringLiteral("  预览抽稀切削段: %1")
               .arg(result.metrics.skippedPreviewCutMoves)
        << QStringLiteral("  无效显示段: %1")
               .arg(result.metrics.skippedDisplaySegments)
        << QStringLiteral("结果: 腔清腔刀路生成成功。");
}

} // namespace

bool PocketToolpathGenerationUseCase::publishPreview(
    const std::vector<toolpath::PocketToolpathOperation>& operations,
    const toolpath::ToolpathParams& params,
    int cutMoveBudget,
    const PocketPreviewPublisher& publisher,
    std::vector<PocketPreviewEntry>& entries,
    PocketToolpathGenerationMetrics& metrics)
{
    (void)params;
    // 预算控制适用于所有口袋路径模式。步长取「完整切削段/预算」
    // 向上取整；仅影响显示统计，不改变完整工序移动流。
    cutMoveBudget = std::max(1, cutMoveBudget);
    metrics.previewLimited = metrics.fullCutMoveCount > cutMoveBudget;
    metrics.cutPreviewStride = metrics.previewLimited
        ? std::max(2, static_cast<int>(std::ceil(
                          static_cast<double>(metrics.fullCutMoveCount) /
                          static_cast<double>(cutMoveBudget))))
        : 1;

    // 逐工序构造预览条目：任一发布失败则整批失败，避免按显示子集提交工序；
    // 切削段按全局序号抽稀，快速/进退刀段始终保留。
    entries.clear();
    entries.reserve(operations.size());
    int cutOrdinal = 0;
    for (const toolpath::PocketToolpathOperation& operation : operations) {
        PocketPreviewEntry entry;
        entry.key = pocketInstanceKey(operation.sourceFaceIds);
        entry.operation = operation;
        if (publisher && !publisher(entry)) {
            metrics.skippedDisplaySegments +=
                static_cast<int>(operation.clearing.moves.size());
            entries.clear();
            return false;
        }
        for (const toolpath::ToolpathMove& move : operation.clearing.moves) {
            switch (move.type) {
                case toolpath::ToolpathMoveType::Cut: {
                    const int ordinal = cutOrdinal++;
                    if (metrics.previewLimited &&
                        ordinal % metrics.cutPreviewStride != 0) {
                        ++metrics.skippedPreviewCutMoves;
                        break;
                    }
                    ++metrics.cutMoveCount;
                    if (operation.kind == FeatureKind::Slot)
                        ++metrics.slotCutMoveCount;
                    else
                        ++metrics.pocketCutMoveCount;
                    break;
                }
                case toolpath::ToolpathMoveType::Rapid:
                    ++metrics.rapidMoveCount;
                    break;
                case toolpath::ToolpathMoveType::Plunge:
                case toolpath::ToolpathMoveType::Retract:
                    ++metrics.entryRetractMoveCount;
                    break;
            }
        }
        entries.push_back(std::move(entry));
    }
    return metrics.displayTotalMoves() > 0 && !entries.empty();
}

PocketToolpathGenerationResult
PocketToolpathGenerationUseCase::execute(
    const PocketToolpathGenerationRequest& input,
    const PocketPreviewPublisher& previewPublisher)
{
    PocketToolpathGenerationResult result;
    auto request = input;
    tooling::ToolLibrary directLibrary;
    if (request.singleTool) {
        if (!makeDirectToolLibrary(*request.singleTool, DirectToolProcess::Pocket, directLibrary, result.error))
            return result;
        request.automaticLibrary = &directLibrary;
        request.definition.operationToolLibraryId = std::string{};
    }
    // 请求完整性校验：任一必填输入缺失即失败。
    if (!request.regions || !request.buildSkipped || !request.params ||
        !request.moldFrame || !request.currentJob || !request.setupFrames || !request.automaticLibrary) {
        result.error = QStringLiteral("槽腔刀路生成需要完整请求及活动刀库。");
        result.reportLines << QStringLiteral(
            "结果: 槽腔刀路生成需要完整请求及活动刀库。");
        return result;
    }

    appendInitialReport(result.reportLines, request);
    result.metrics.skipSummary = buildSkipSummary(*request.buildSkipped);
    if (request.candidateCount <= 0) {
        result.failure = PocketToolpathGenerationFailure::EmptyProgram;
        result.error = QStringLiteral(
            "没有识别到由主外表面内环产生的结构。"
            "外环分支和内部根面分支不会生成清腔刀路。");
        result.reportLines << QString()
                           << QStringLiteral(
                                  "结果: 没有主外表面内环产生的可清腔结构。");
        return result;
    }

    appendRegionReport(result.reportLines, request);
    if (request.regions->empty()) {
        result.failure = PocketToolpathGenerationFailure::EmptyProgram;
        result.error = QStringLiteral(
            "没有主外表面内环结构能建立有效的逐层截面区域。") +
            result.metrics.skipSummary;
        result.reportLines << QStringLiteral(
            "结果: 区域构建后没有可加工区域。");
        return result;
    }

    // 生成：按活动刀库组合选刀。
    if (!generatePocketBatch(request, result))
        return result;

    // 为每个工序分配装夹方向上下文（六面判定）。
    assignPocketSetupContexts(
        result.batch, *request.regions, *request.moldFrame);
    appendAutomaticSelectionReport(request, result);
    // 逐区域核对批处理结果并汇总失败统计（分类计数 + 明细文本）。
    const FailureStats failures = collectGenerationMetrics(request, result);

    appendGenerationMetricsReport(result);

    if (result.batch.operations.empty() ||
        result.metrics.fullTotalMoves() == 0) {
        result.failure = PocketToolpathGenerationFailure::EmptyProgram;
        const int otherSkipped =
            static_cast<int>(request.buildSkipped->size()) + failures.other;
        result.error = QStringLiteral(
            "未生成腔清腔刀路:候选主外表面内环结构 %1,%2,"
            "其他原因跳过 %3。")
            .arg(request.candidateCount)
            .arg(result.metrics.failureSummary)
            .arg(otherSkipped);
        result.error += result.metrics.skipSummary;
        result.reportLines << QStringLiteral(
            "结果: 未生成任何可验证的腔清腔移动。");
        return result;
    }

    // 验证：结构验证与安全验证均通过才允许进入预览与提交。
    if (!validateGeneratedCandidate(result))
        return result;

    // 先构造候选 Job，取得正式工序 ID；此处尚未安装到工作区。
    if (!prepareCandidateJob(request, result))
        return result;

    // 预览失败由 UI 恢复旧显示，候选 Job 不会安装到工作区。
    if (!prepareCandidatePreview(request, previewPublisher, result))
        return result;

    appendSuccessfulGenerationReport(request, result);
    result.failure = PocketToolpathGenerationFailure::None;
    return result;
}

} // namespace oap::app

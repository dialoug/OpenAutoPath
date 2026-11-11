// HoleToolpathGenerationUseCase 实现（oap_app_workflow_cam）。
// 不依赖 MainWindow 状态的完整桌面孔刀路候选生成，见 HoleToolpathGenerationUseCase.h。
#include "app/workflows/hole/HoleToolpathGenerationUseCase.h"
#include <QCoreApplication>

#include "app/workflows/hole/HoleToolpathRunner.h"
#include "app/workflows/project/DirectToolSource.h"

#include <algorithm>

#include <QDateTime>
#include <QElapsedTimer>

namespace oap::app {
namespace {

QString validationMessage(
    const toolpath::ToolpathValidationReport& report,
    int maxIssues = 20)
{
    QString message = QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(report.summary()).toUtf8().constData());
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
                    .arg(QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(issue.message).toUtf8().constData()));
                ++appended;
            }
        };
    appendSeverity(toolpath::ToolpathIssueSeverity::Error);
    appendSeverity(toolpath::ToolpathIssueSeverity::Warning);
    if (issueCount > appended) {
        message += QCoreApplication::translate("OpenAutoPath", "\n……另有 %1 个问题")
            .arg(issueCount - appended);
    }
    return message;
}

QString diagnosticReasonZh(
    const toolpath::ToolpathDiagnostic& diagnostic)
{
    using Code = toolpath::ToolpathDiagnosticCode;
    if (diagnostic.code == Code::MissingPlanarFloor)
        return QCoreApplication::translate("OpenAutoPath", "底面非平面（暂不支持斜底/曲底）");
    if (diagnostic.code == Code::ManualFloorDisconnected)
        return QCoreApplication::translate("OpenAutoPath", "人工底面没有可验证的拓扑邻接侧壁");
    if (diagnostic.code == Code::DuplicateRegion) {
        return QCoreApplication::translate("OpenAutoPath", "自动识别区域与已有区域共用同一底面，源面已合并并去重");
    }
    return QCoreApplication::translate("OpenAutoPath",
        toolpath::toolpathDiagnosticDefaultMessageZh(diagnostic.code));
}

QString setupDecisionText(MoldSetupFaceDecision decision)
{
    switch (decision) {
        case MoldSetupFaceDecision::NearestDistance:
            return QCoreApplication::translate("OpenAutoPath", "端点到外包络面的最近距离");
        case MoldSetupFaceDecision::TopologyTieBreak:
            return QCoreApplication::translate("OpenAutoPath", "等距时采用拓扑外开口");
        case MoldSetupFaceDecision::StableFaceOrder:
            return QCoreApplication::translate("OpenAutoPath", "等距时采用固定六面顺序");
        case MoldSetupFaceDecision::None:
            return QCoreApplication::translate("OpenAutoPath", "未记录");
    }
    return QCoreApplication::translate("OpenAutoPath", "未记录");
}

QString pointText(const gp_Pnt& point)
{
    return QStringLiteral("(%1, %2, %3)")
        .arg(point.X(), 0, 'f', 3)
        .arg(point.Y(), 0, 'f', 3)
        .arg(point.Z(), 0, 'f', 3);
}

} // namespace

HoleToolpathGenerationResult
HoleToolpathGenerationUseCase::execute(
    const HoleToolpathGenerationRequest& input)
{
    HoleToolpathGenerationResult result;
    auto request = input;
    tooling::ToolLibrary directLibrary;
    if (request.singleTool) {
        if (!makeDirectToolLibrary(*request.singleTool, DirectToolProcess::Hole, directLibrary, result.error))
            return result;
        request.automaticLibrary = &directLibrary;
        request.definition.operationToolLibraryId = std::string{};
    }
    // 请求完整性校验：任一必填输入缺失即失败。
    if (!request.recognized || request.recognized->empty() ||
        !request.model || !request.indexer || !request.moldFrame ||
        !request.params || !request.currentJob || !request.setupFrames || !request.automaticLibrary) {
        result.error = QCoreApplication::translate("OpenAutoPath", "孔刀路生成需要完整请求及活动刀库。");
        return result;
    }

    // 几何准备：外包络入口恢复 + 障碍体快速避障上下文，单独计时。
    QElapsedTimer timer;
    timer.start();
    result.geometry = HoleToolpathWorkflowController::prepareGeometry(
        *request.recognized, *request.model, *request.indexer,
        request.moldBounds, *request.moldFrame);
    result.envelopeEntryMilliseconds = timer.elapsed();
    const auto& inputs = result.geometry.envelope.holes;

    // 按活动刀库自动选刀生成，带进度发布。
    timer.restart();
    result.automatic = request.singleTool == nullptr;
    if (request.automaticLibrary) {
        result.automaticLibraryName = request.automaticLibrary->name();
        tooling::HoleAutomaticToolpathResult automatic =
            generateAutomaticHolesWithProgress(
                request.interaction.showProgress, *request.automaticLibrary,
                inputs, *request.params, result.geometry.rapidContext);
        result.automaticSelection = std::move(automatic.selection);
        result.batch = std::move(automatic.batch);
    }
    result.generationMilliseconds = timer.elapsed();

    if (result.batch.cancelled) {
        result.failure = HoleToolpathGenerationFailure::Cancelled;
        return result;
    }
    const auto& machining = result.batch.machining;
    if (machining.operations.empty()) {
        result.failure = HoleToolpathGenerationFailure::EmptyProgram;
        result.error = QCoreApplication::translate("OpenAutoPath", "未生成有效的孔加工移动：识别孔 %1，跳过 %2。")
            .arg(inputs.size())
            .arg(machining.skippedCount);
        return result;
    }
    result.program = result.batch.toProgram();
    result.validation = validateToolpathCandidate(
        result.batch.validation, result.program,
        result.batch.effectiveParams);
    if (!result.validation.ok()) {
        const toolpath::ToolpathValidationReport validation =
            result.validation.combined();
        result.failure = HoleToolpathGenerationFailure::ValidationFailed;
        result.error = QCoreApplication::translate("OpenAutoPath", "孔刀路验证失败:\n%1")
            .arg(validationMessage(validation, 5));
        return result;
    }

    // 统计各移动类型数量，供状态栏摘要与日志使用。
    for (const toolpath::HoleMachiningOperation& operation :
         machining.operations) {
        for (const toolpath::ToolpathMove& move : operation.moves) {
            switch (move.type) {
                case toolpath::ToolpathMoveType::Cut:
                    ++result.cutMoveCount;
                    break;
                case toolpath::ToolpathMoveType::Rapid:
                    ++result.rapidMoveCount;
                    break;
                case toolpath::ToolpathMoveType::Plunge:
                case toolpath::ToolpathMoveType::Retract:
                    ++result.verticalMoveCount;
                    break;
            }
        }
    }
    result.summary = QCoreApplication::translate("OpenAutoPath", "已绘制孔刀路：小孔 %1，中孔 %2，大孔 %3，跳过 %4。\n"
        "移动段 %5：切削 %6，快速 %7，进退刀 %8。")
        .arg(machining.smallCount)
        .arg(machining.mediumCount)
        .arg(machining.largeCount)
        .arg(machining.skippedCount)
        .arg(result.cutMoveCount + result.rapidMoveCount +
             result.verticalMoveCount)
        .arg(result.cutMoveCount)
        .arg(result.rapidMoveCount)
        .arg(result.verticalMoveCount);
    result.summary += QCoreApplication::translate("OpenAutoPath", "\n装夹组 %1，孔间直连 %2，自动抬高 %3，避障失败 %4。")
        .arg(machining.setupCount)
        .arg(machining.directRapidLinkCount)
        .arg(machining.liftedRapidLinkCount)
        .arg(machining.unsafeRapidLinkCount);
    result.summary += QCoreApplication::translate("OpenAutoPath", " 路径 %1→%2mm，2-opt/重定位/3-opt=%3/%4/%5；"
        "精检 %6，粗筛 %7。")
        .arg(machining.nearestNeighborRouteLength, 0, 'f', 1)
        .arg(machining.optimizedRouteLength, 0, 'f', 1)
        .arg(machining.twoOptImprovementCount)
        .arg(machining.relocationImprovementCount)
        .arg(machining.threeOptImprovementCount)
        .arg(machining.exactCollisionCheckCount)
        .arg(machining.broadPhaseSafeSegmentCount);
    if (!result.geometry.envelope.adjustments.empty()) {
        result.summary += QCoreApplication::translate("OpenAutoPath", " 已恢复 %1 个物理孔组到模具外包络面。")
            .arg(result.geometry.envelope.adjustments.size());
    }

    // 组装提交请求并原子提交：失败则整个生成流程报 CommitFailed。
    HoleToolpathCommitRequest commit;
    commit.currentJob = request.currentJob;
    commit.definition = request.definition;
    commit.inputs = &inputs;
    commit.automaticSelection = &result.automaticSelection;
    commit.generated = &result.batch;
    commit.setupFrames = request.setupFrames;
    result.committed = HoleToolpathWorkflowController::commit(commit);
    if (!result.committed.ok) {
        result.failure = HoleToolpathGenerationFailure::CommitFailed;
        result.error = QCoreApplication::translate("OpenAutoPath", "孔加工项目提交失败：\n%1")
            .arg(result.committed.error);
        return result;
    }
    result.failure = HoleToolpathGenerationFailure::None;
    return result;
}

QStringList HoleToolpathGenerationUseCase::buildReport(
    const HoleToolpathGenerationResult& result,
    const HoleToolpathGenerationReportContext& context)
{
    const auto& envelope = result.geometry.envelope;
    const auto& inputs = envelope.holes;
    const auto& machining = result.batch.machining;
    QStringList lines;
    lines << QCoreApplication::translate("OpenAutoPath", "OpenAutoPath 孔刀路避障日志")
          << QCoreApplication::translate("OpenAutoPath", "时间: %1")
                 .arg(QDateTime::currentDateTime().toString(Qt::ISODateWithMs))
          << QCoreApplication::translate("OpenAutoPath", "日志文件: %1").arg(context.logPath)
          << QString()
          << QCoreApplication::translate("OpenAutoPath", "识别孔数: %1").arg(inputs.size())
          << QCoreApplication::translate("OpenAutoPath", "识别来源: %1").arg(context.recognitionSource)
          << QCoreApplication::translate("OpenAutoPath", "准备耗时:识别/缓存 %1 ms,外包络入口恢复 %2 ms")
                 .arg(context.recognitionMilliseconds)
                 .arg(result.envelopeEntryMilliseconds)
          << QCoreApplication::translate("OpenAutoPath", "孔专用识别分段:AAG %1 ms,圆柱孔分类 %2 ms,"
                 "刀路几何 %3 ms")
                 .arg(context.aagBuildMilliseconds)
                 .arg(context.holeClassificationMilliseconds)
                 .arg(context.geometryBuildMilliseconds)
          << QCoreApplication::translate("OpenAutoPath", "孔加工规划与验证耗时: %1 ms")
                 .arg(result.generationMilliseconds)
          << QCoreApplication::translate("OpenAutoPath", "选刀模式: %1")
                 .arg(result.automatic ? QCoreApplication::translate("OpenAutoPath", "活动刀库六面自动选刀（%1）").arg(result.automaticLibraryName)
                                       : QCoreApplication::translate("OpenAutoPath", "单刀，按识别几何自动规划"))
          << QCoreApplication::translate("OpenAutoPath", "物理孔组数: %1")
                 .arg(envelope.physicalGroupCount)
          << QCoreApplication::translate("OpenAutoPath", "恢复到外包络面的孔组数: %1")
                 .arg(envelope.adjustments.size())
          << QCoreApplication::translate("OpenAutoPath", "障碍实体数: %1")
                 .arg(result.geometry.obstacleShapeCount)
          << QCoreApplication::translate("OpenAutoPath", "生成操作数: %1")
                 .arg(machining.operations.size())
          << QCoreApplication::translate("OpenAutoPath", "跳过孔数: %1").arg(machining.skippedCount)
          << QCoreApplication::translate("OpenAutoPath", "装夹组数: %1").arg(machining.setupCount)
          << QCoreApplication::translate("OpenAutoPath", "孔间直接 Rapid: %1")
                 .arg(machining.directRapidLinkCount)
          << QCoreApplication::translate("OpenAutoPath", "自动抬高 Rapid: %1")
                 .arg(machining.liftedRapidLinkCount)
          << QCoreApplication::translate("OpenAutoPath", "避障失败连接: %1")
                 .arg(machining.unsafeRapidLinkCount)
          << QCoreApplication::translate("OpenAutoPath", "包围盒粗筛安全段: %1")
                 .arg(machining.broadPhaseSafeSegmentCount)
          << QCoreApplication::translate("OpenAutoPath", "OCCT 精确碰撞检查: %1")
                 .arg(machining.exactCollisionCheckCount)
          << QCoreApplication::translate("OpenAutoPath", "实体扫掠逻辑检查/实际构造: %1 / %2")
                 .arg(machining.solidSweepCheckCount)
                 .arg(machining.solidSweepConstructedPartCount)
          << QCoreApplication::translate("OpenAutoPath", "圆弧/螺旋扫掠原语: %1")
                 .arg(machining.arcSweepPrimitiveCount)
          << QCoreApplication::translate("OpenAutoPath", "切削/非切削碰撞: %1 / %2")
                 .arg(machining.cuttingMoveCollisionCount)
                 .arg(machining.nonCuttingMoveCollisionCount)
          << QCoreApplication::translate("OpenAutoPath", "路径长度(最近邻/2-opt/重定位/最终): %1 / %2 / %3 / %4 mm")
                 .arg(machining.nearestNeighborRouteLength, 0, 'f', 3)
                 .arg(machining.twoOptRouteLength, 0, 'f', 3)
                 .arg(machining.relocationRouteLength, 0, 'f', 3)
                 .arg(machining.optimizedRouteLength, 0, 'f', 3)
          << QCoreApplication::translate("OpenAutoPath", "路径改进次数(2-opt/重定位/3-opt): %1 / %2 / %3")
                 .arg(machining.twoOptImprovementCount)
                 .arg(machining.relocationImprovementCount)
                 .arg(machining.threeOptImprovementCount)
          << QCoreApplication::translate("OpenAutoPath", "避障重排: %1 次，恢复连接 %2 条")
                 .arg(machining.rapidRepairRelocationCount)
                 .arg(machining.rapidRepairRecoveredLinkCount)
          << QString();

    if (result.automatic) {
        lines << QCoreApplication::translate("OpenAutoPath", "自动选刀汇总:物理孔 %1，跳过 %2，非空装夹面 %3，"
                     "刀具批次 %4")
                     .arg(result.automaticSelection.physicalHoleCount)
                     .arg(result.automaticSelection.skippedPhysicalHoleCount)
                     .arg(result.automaticSelection.setupCount)
                     .arg(result.automaticSelection.batches.size());
        for (const tooling::HoleToolBatch& batch :
             result.automaticSelection.batches) {
            lines << QCoreApplication::translate("OpenAutoPath", "  装夹面 %1 / %2 / T%3 %4 / 直径 %5 mm / %6 个目标")
                         .arg(QString::fromLatin1(
                             moldSetupFaceName(batch.setupFace)))
                         .arg(QString::fromLatin1(
                             toolpath::holeMachiningPhaseName(batch.phase)))
                         .arg(batch.tool.toolNumber)
                         .arg(QString::fromStdString(batch.tool.toolName))
                         .arg(batch.tool.diameter, 0, 'f', 3)
                         .arg(batch.tasks.size());
        }
        lines << QString();
    }

    if (!envelope.adjustments.empty()) {
        lines << QCoreApplication::translate("OpenAutoPath", "模具外包络面入口恢复明细:");
        for (const toolpath::HoleMoldEnvelopeEntryAdjustment& adjustment :
             envelope.adjustments) {
            lines << QCoreApplication::translate("OpenAutoPath", "  孔 %1: 物理孔组=%2, 装夹面=%3, 延长=%4 mm")
                         .arg(adjustment.holeIndex + 1)
                         .arg(adjustment.physicalGroupIndex + 1)
                         .arg(QString::fromLatin1(
                             moldSetupFaceName(adjustment.setupFace)))
                         .arg(adjustment.extensionLength, 0, 'f', 3)
                  << QCoreApplication::translate("OpenAutoPath", "    原入口=%1, 外包络入口=%2")
                         .arg(pointText(adjustment.originalEntry))
                         .arg(pointText(adjustment.envelopeEntry));
        }
        lines << QString();
    }

    const bool hasCountersink = std::any_of(
        inputs.begin(), inputs.end(),
        [](const toolpath::HoleMachiningInput& hole) {
            return hole.kind == FeatureKind::CountersinkHole &&
                   hole.setupFace != MoldSetupFace::Unresolved;
        });
    if (hasCountersink) {
        lines << QCoreApplication::translate("OpenAutoPath", "沉头孔六面装夹归类明细:");
        for (int index = 0; index < static_cast<int>(inputs.size()); ++index) {
            const toolpath::HoleMachiningInput& hole = inputs[index];
            if (hole.kind != FeatureKind::CountersinkHole ||
                hole.setupFace == MoldSetupFace::Unresolved) {
                continue;
            }
            lines << QCoreApplication::translate("OpenAutoPath", "  孔 %1: 半径=%2 mm, 装夹面=%3, "
                         "入口端距外表面=%4 mm, 另一端最近距离=%5 mm")
                         .arg(index + 1)
                         .arg(hole.radius, 0, 'f', 3)
                         .arg(QString::fromLatin1(
                             moldSetupFaceName(hole.setupFace)))
                         .arg(hole.setupFaceEntryDistance, 0, 'f', 6)
                         .arg(hole.setupFaceOtherEndpointDistance, 0, 'f', 6)
                  << QCoreApplication::translate("OpenAutoPath", "    判定来源: %1 (%2)")
                         .arg(hole.entryResolution ==
                                      HoleEntryResolution::CountersinkLargestStage
                                  ? QCoreApplication::translate("OpenAutoPath", "继承沉头孔最大直径段装夹面")
                                  : setupDecisionText(hole.setupFaceDecision))
                         .arg(QString::fromLatin1(
                             hole.entryResolution ==
                                     HoleEntryResolution::CountersinkLargestStage
                                 ? "countersink_largest_stage"
                                 : moldSetupFaceDecisionName(
                                       hole.setupFaceDecision)));
        }
        lines << QString();
    }

    const bool hasCounterboreChild = std::any_of(
        inputs.begin(), inputs.end(),
        [](const toolpath::HoleMachiningInput& hole) {
            return hole.parentRelation ==
                HoleParentRelation::CounterboreFloorChild;
        });
    if (hasCounterboreChild) {
        lines << QCoreApplication::translate("OpenAutoPath", "沉孔平底子孔装夹归属明细:");
        for (int index = 0; index < static_cast<int>(inputs.size()); ++index) {
            const toolpath::HoleMachiningInput& hole = inputs[index];
            if (hole.parentRelation !=
                HoleParentRelation::CounterboreFloorChild) {
                continue;
            }
            lines << QCoreApplication::translate("OpenAutoPath", "  孔 %1: 类型=%2, 半径=%3 mm, 装夹面=%4")
                         .arg(index + 1)
                         .arg(QString::fromLatin1(
                             holeParentRelationName(hole.parentRelation)))
                         .arg(hole.radius, 0, 'f', 3)
                         .arg(QString::fromLatin1(
                             moldSetupFaceName(hole.setupFace)))
                  << QCoreApplication::translate("OpenAutoPath", "    父沉孔圆柱=%1, 共享平底面=%2, 父沉孔半径=%3 mm")
                         .arg(QString::fromStdString(
                             hole.parentCounterboreFaceId))
                         .arg(QString::fromStdString(
                             hole.parentCounterboreFloorFaceId))
                         .arg(hole.parentCounterboreRadius, 0, 'f', 3);
        }
        lines << QString();
    }

    std::vector<toolpath::ToolpathDiagnostic> diagnostics =
        envelope.diagnostics;
    diagnostics.insert(
        diagnostics.end(), result.batch.diagnostics.begin(),
        result.batch.diagnostics.end());
    diagnostics.insert(
        diagnostics.end(), machining.diagnostics.begin(),
        machining.diagnostics.end());
    if (diagnostics.empty()) {
        lines << QCoreApplication::translate("OpenAutoPath", "失败原因: 无避障或孔刀路生成警告。");
    } else {
        lines << QCoreApplication::translate("OpenAutoPath", "失败/警告明细:");
        for (int index = 0;
             index < static_cast<int>(diagnostics.size()); ++index) {
            const toolpath::ToolpathDiagnostic& diagnostic =
                diagnostics[index];
            lines << QCoreApplication::translate("OpenAutoPath", "%1. [%2] 原因: %3")
                         .arg(index + 1)
                         .arg(QString::fromLatin1(
                             toolpath::toolpathDiagnosticCodeName(
                                 diagnostic.code)))
                         .arg(diagnosticReasonZh(diagnostic))
                  << QCoreApplication::translate("OpenAutoPath", "   原始信息: %1")
                         .arg(QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(diagnostic.message).toUtf8().constData()));
        }
    }
    lines << QString()
          << QCoreApplication::translate("OpenAutoPath", "最终刀路验证:")
          << validationMessage(result.validation.combined(), 20);
    return lines;
}

} // namespace oap::app

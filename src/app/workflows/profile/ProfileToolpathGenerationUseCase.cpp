// ProfileToolpathGenerationUseCase 实现（oap_app_workflow_cam）。
// 不依赖 MainWindow 状态的完整桌面外轮廓刀路候选生成，见 ProfileToolpathGenerationUseCase.h。
#include "app/workflows/profile/ProfileToolpathGenerationUseCase.h"
#include <QCoreApplication>
#include "app/workflows/project/DirectToolSource.h"
#include "analysis/MoldSurfaceClassifier.h"

#include <algorithm>

#include <QDateTime>

#include "app/workflows/profile/ProfileSetupContextAssigner.h"
#include "app/workflows/profile/ProfileToolpathRunner.h"
#include "toolpath/profile/geometry/ProfileRegionBuilder.h"

namespace oap::app {
namespace {

QString validationMessage(const toolpath::ToolpathValidationReport &report,
                          int maxIssues = 5) {
  QString message = QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(report.summary()).toUtf8().constData());
  const int visibleCount =
      std::min(maxIssues, static_cast<int>(report.issues.size()));
  for (int index = 0; index < visibleCount; ++index) {
    const toolpath::ToolpathIssue &issue = report.issues[index];
    message += issue.severity == toolpath::ToolpathIssueSeverity::Error
                   ? QStringLiteral("\nERROR ")
                   : QStringLiteral("\nWARN ");
    message += QString::fromStdString(issue.code);
    if (!issue.operationId.empty()) {
      message += QStringLiteral(" [%1]").arg(
          QString::fromStdString(issue.operationId));
    }
    if (issue.moveIndex >= 0)
      message += QStringLiteral(" move %1").arg(issue.moveIndex);
    message +=
        QStringLiteral(": %1").arg(QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(issue.message).toUtf8().constData()));
  }
  if (static_cast<int>(report.issues.size()) > visibleCount) {
    message += QCoreApplication::translate("OpenAutoPath", "\n……另有 %1 个问题")
                   .arg(static_cast<int>(report.issues.size()) - visibleCount);
  }
  return message;
}

double maximumAxialEngagement(const toolpath::ProfileRegion &region,
                              const toolpath::ProfileContourResult &contour) {
  double previousOffset = region.depth;
  double maximum = 0.0;
  for (const toolpath::ProfileLoop &loop : contour.loops) {
    const double offset = std::clamp(loop.depthOffset, 0.0, region.depth);
    if (offset + 1.0e-9 < previousOffset) {
      maximum = std::max(maximum, previousOffset - offset);
      previousOffset = offset;
    }
  }
  return maximum > 1.0e-9 ? maximum : region.depth;
}

QString failureMessage(const toolpath::ProfileToolpathBatch &batch,
                       const std::vector<toolpath::ProfileRegion> &regions) {
  const toolpath::ToolpathDiagnostic *error = nullptr;
  for (const toolpath::ToolpathDiagnostic &diagnostic : batch.diagnostics) {
    if (diagnostic.severity == toolpath::ToolpathDiagnosticSeverity::Error) {
      error = &diagnostic;
      break;
    }
  }

  int regionIndex = error ? error->sourceIndex : -1;
  const toolpath::ProfileGenerationFailure *failure = nullptr;
  if (regionIndex >= 0) {
    const auto found =
        std::find_if(batch.failures.begin(), batch.failures.end(),
                     [regionIndex](const auto &item) {
                       return item.regionIndex == regionIndex;
                     });
    if (found != batch.failures.end())
      failure = &*found;
  }
  if (!failure && !batch.failures.empty()) {
    failure = &batch.failures.front();
    regionIndex = failure->regionIndex;
  }

  const toolpath::ProfileRegion *region =
      regionIndex >= 0 && regionIndex < static_cast<int>(regions.size())
          ? &regions[static_cast<std::size_t>(regionIndex)]
          : nullptr;
  const QString location =
      region ? QCoreApplication::translate("OpenAutoPath", "区域 %1（根面 %2，识别总深度 %3 mm）")
                   .arg(regionIndex + 1)
                   .arg(QString::fromStdString(region->source.rootFaceId))
                   .arg(region->depth, 0, 'f', 3)
             : QCoreApplication::translate("OpenAutoPath", "外轮廓区域");

  if (error) {
    using Code = toolpath::ToolpathDiagnosticCode;
    switch (error->code) {
    case Code::ProfileCuttingEdgeReachInsufficient: {
      const double engagement =
          region && failure ? maximumAxialEngagement(*region, failure->contour)
                            : batch.effectiveParams.depth.stepDown;
      return QCoreApplication::translate("OpenAutoPath", "%1：单层最大轴向切深 %2 mm 超过刀刃长度 %3 mm。\n"
                 "请减小“槽腔/外轮廓 → 每层切深”，或选择更长刃长的刀具。")
          .arg(location)
          .arg(engagement, 0, 'f', 3)
          .arg(batch.effectiveParams.tool.cuttingEdgeLength, 0, 'f', 3);
    }
    case Code::ProfileToolReachInsufficient:
      return QCoreApplication::translate("OpenAutoPath", "%1：刀具可达长度 %2 mm 不足。\n"
                 "请增大刀具伸出/总长，或改用能够到达该侧壁深度的刀具。")
          .arg(location)
          .arg(batch.effectiveParams.tool.overallLength, 0, 'f', 3);
    case Code::ProfileFinishedPartIntrusion:
      return QCoreApplication::translate("OpenAutoPath", "%1：第 %2 个移动段的刀刃包络侵入成品轮廓。\n"
                            "请检查侧壁余量、刀径以及切入/切出长度。")
          .arg(location)
          .arg(error->moveIndex >= 0 ? error->moveIndex + 1 : 0);
    case Code::NonCuttingMoveCollision:
    case Code::CuttingMoveCollision:
      return QCoreApplication::translate("OpenAutoPath", "%1：第 %2 个移动段发生%3包络干涉。\n"
                            "请检查安全高度、刀具伸出以及刀柄/主轴尺寸。")
          .arg(location)
          .arg(error->moveIndex >= 0 ? error->moveIndex + 1 : 0)
          .arg(error->code == Code::NonCuttingMoveCollision
                   ? QCoreApplication::translate("OpenAutoPath", "非切削移动")
                   : QCoreApplication::translate("OpenAutoPath", "切削移动"));
    default:
      return QCoreApplication::translate("OpenAutoPath", "%1：%2\n详细信息：%3")
          .arg(location)
          .arg(QCoreApplication::translate("OpenAutoPath",
              toolpath::toolpathDiagnosticDefaultMessageZh(error->code)))
          .arg(QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(error->message).toUtf8().constData()));
    }
  }
  if (failure && !failure->contour.error.empty()) {
    return QStringLiteral("%1：%2").arg(location).arg(
        QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(failure->contour.error).toUtf8().constData()));
  }
  return QCoreApplication::translate("OpenAutoPath", "未生成任何可执行的外轮廓工序。");
}

QString
automaticSummary(const tooling::ProfileAutomaticToolpathResult &selected) {
  QStringList stages;
  int restStageCount = 0;
  for (const auto &stage : selected.stages) {
    QString text = QStringLiteral("T%1 %2（D%3）")
                       .arg(stage.tool.toolNumber)
                       .arg(QString::fromStdString(stage.tool.toolName))
                       .arg(stage.tool.diameter, 0, 'f', 3);
    if (stage.previousToolDiameter > stage.tool.diameter) {
      ++restStageCount;
      text += QCoreApplication::translate("OpenAutoPath", " 余料");
    }
    if (stage.finishPass)
      text += QCoreApplication::translate("OpenAutoPath", " 精修");
    stages.push_back(text);
  }
  return QCoreApplication::translate("OpenAutoPath", "；自动选刀 %1 阶段（%2 个余料阶段）：%3")
      .arg(selected.stages.size())
      .arg(restStageCount)
      .arg(stages.join(QStringLiteral(" → ")));
}

} // namespace

ProfileToolpathGenerationResult ProfileToolpathGenerationUseCase::execute(
    const ProfileToolpathGenerationRequest &input,
    const ProfilePreviewPublisher &previewPublisher) {
  ProfileToolpathGenerationResult result;
  auto request = input;
  tooling::ToolLibrary directLibrary;
  if (request.singleTool) {
    if (!makeDirectToolLibrary(*request.singleTool, DirectToolProcess::Profile, directLibrary, result.error))
      return result;
    request.automaticLibrary = &directLibrary;
    request.definition.operationToolLibraryId = std::string{};
  }
  if (!request.recognizedRegions || !request.params || !request.moldFrame ||
      !request.currentJob || !request.setupFrames || !request.automaticLibrary) {
    result.error = QCoreApplication::translate("OpenAutoPath", "外轮廓刀路生成需要完整请求及活动刀库。");
    return result;
  }
  if (request.recognizedRegions->empty()) {
    result.failure = ProfileToolpathGenerationFailure::EmptyProgram;
    result.error = QCoreApplication::translate("OpenAutoPath", "没有可生成刀路的外轮廓区域。");
    return result;
  }

  result.regions = orderProfileRegionsBySetup(*request.recognizedRegions,
                                              *request.moldFrame);
  if (request.automaticSetupDirections && request.selectedSetupFace) {
    result.error = QCoreApplication::translate("OpenAutoPath", "自动规划方向与指定方向不能同时启用。");
    return result;
  }
  if (request.selectedSetupFace) {
    if (*request.selectedSetupFace == MoldSetupFace::Unresolved) {
      result.error = QCoreApplication::translate("OpenAutoPath", "外轮廓加工方向无效。");
      return result;
    }
    std::erase_if(result.regions, [&](const auto& region) {
      return classifyMoldSetupFace(*request.moldFrame, gp_Vec(region.openDir)) !=
          *request.selectedSetupFace;
    });
    if (result.regions.empty()) {
      result.error = QCoreApplication::translate("OpenAutoPath", "所选方向 %1 没有可加工的外轮廓区域。")
          .arg(QString::fromLatin1(moldSetupFaceName(*request.selectedSetupFace)));
      return result;
    }
  }
  toolpath::ProfileRegionBuildOptions stockOptions;
  stockOptions.stockMarginX = request.definition.stock.marginX;
  stockOptions.stockMarginY = request.definition.stock.marginY;
  stockOptions.stockMarginZ = request.definition.stock.marginZ;
  for (toolpath::ProfileRegion &region : result.regions) {
    toolpath::ProfileRegionBuilder::updateStockBoundary(region, stockOptions);
  }

  result.automatic = request.singleTool == nullptr;
  if (request.automaticLibrary) {
    toolpath::ProfileDirectionPlanningOptions directionOptions;
    directionOptions.allowReduction = request.params->depth.useFeatureDepth &&
        request.definition.stock.sourceKind == manufacturing::StockSourceKind::PerUnitBoxes;
    directionOptions.stockMargins = gp_Vec(request.definition.stock.marginX,
        request.definition.stock.marginY, request.definition.stock.marginZ);
    toolpath::ProfileDirectionPlan directionPlan;
    result.automaticSelection = generateProfilesWithProgress(
        request.interaction.showProgress, *request.automaticLibrary,
        result.regions, *request.params,
        request.singleTool ? "PROFILE_SINGLE_" : "PROFILE_AUTO_",
        request.singleTool && request.params->profile.machiningMode ==
            toolpath::ProfileMachiningMode::ContourOnly,
        request.automaticSetupDirections ? &directionOptions : nullptr,
        &directionPlan);
    if (request.automaticSetupDirections && directionPlan.complete) {
      for (const auto& coverage : directionPlan.coverage) {
        const auto& selected = result.regions[coverage.selectedRegion];
        result.directionReport << QCoreApplication::translate("OpenAutoPath", "单元 %1：%2（深度 %3 mm）已证明覆盖 %4 个外轮廓来源；刀路仍须通过完整安全验证。")
            .arg(QString::fromStdString(selected.source.unitId))
            .arg(QString::fromLatin1(moldSetupFaceName(classifyMoldSetupFace(*request.moldFrame, gp_Vec(selected.openDir)))))
            .arg(selected.depth, 0, 'f', 3).arg(coverage.coveredRegions.size());
      }
      for (const auto& unit : directionPlan.retainedUnitIds)
        result.directionReport << QCoreApplication::translate("OpenAutoPath", "单元 %1：几何、毛坯或刀具条件不足以证明单方向完整覆盖，保留全部来源。")
            .arg(QString::fromStdString(unit));
      std::vector<toolpath::ProfileRegion> selected;
      for (const auto index : directionPlan.selectedRegionIndices)
        selected.push_back(result.regions[index]);
      result.regions = std::move(selected);
      result.directionReport.prepend(QCoreApplication::translate("OpenAutoPath", "自动方向规划：%1 个来源保留 %2 个；无法证明覆盖的来源全部保留。")
          .arg(request.recognizedRegions->size()).arg(result.regions.size()));
    }
    if (result.automaticSelection.cancelled) {
      result.failure = ProfileToolpathGenerationFailure::Cancelled;
      result.error = QCoreApplication::translate("OpenAutoPath", "外轮廓刀路生成已取消。");
      return result;
    }
    if (!result.automaticSelection.ok()) {
      result.failure = ProfileToolpathGenerationFailure::EmptyProgram;
      const QString reason = result.automaticSelection.error.empty()
                                 ? failureMessage(
                                       result.automaticSelection.batch,
                                       result.regions)
                                 : QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(
                                       result.automaticSelection.error).toUtf8().constData());
      result.error =
          QCoreApplication::translate("OpenAutoPath", "外轮廓刀路生成失败：\n%1")
              .arg(reason);
      return result;
    }
    result.batch = std::move(result.automaticSelection.batch);
  }

  if (result.batch.cancelled) {
    result.failure = ProfileToolpathGenerationFailure::Cancelled;
    result.error = QCoreApplication::translate("OpenAutoPath", "外轮廓刀路生成已取消。");
    return result;
  }

  assignProfileSetupContexts(result.batch, result.regions, *request.moldFrame);
  std::stable_sort(result.batch.operations.begin(),
                   result.batch.operations.end(),
                   [](const auto &left, const auto &right) {
                     return left.setupIndex < right.setupIndex;
                   });
  if (result.batch.operations.empty()) {
    result.failure = ProfileToolpathGenerationFailure::EmptyProgram;
    result.error = failureMessage(result.batch, result.regions);
    return result;
  }

  result.program = result.batch.toProgram();
  result.validation = validateToolpathCandidate(
      result.batch.safetyValidation, result.program,
      result.batch.effectiveParams);
  if (!result.validation.ok()) {
    const toolpath::ToolpathValidationReport validation =
        result.validation.combined();
    result.failure = ProfileToolpathGenerationFailure::ValidationFailed;
    result.error = QCoreApplication::translate("OpenAutoPath", "外轮廓刀路验证失败：\n%1")
                       .arg(validationMessage(validation));
    return result;
  }

  // Build the preview from the validated candidate before capturing the Job
  // copy. This matches the pocket transaction boundary: a preview failure
  // never produces a committed candidate.
  if (previewPublisher && !previewPublisher(result.program)) {
    result.failure = ProfileToolpathGenerationFailure::PreviewFailed;
    result.error =
        QCoreApplication::translate("OpenAutoPath", "刀路已生成并验证通过，但无法创建外轮廓显示对象。");
    return result;
  }

  ProfileToolpathCommitRequest commit;
  commit.currentJob = request.currentJob;
  commit.definition = request.definition;
  commit.operations = &result.batch.operations;
  commit.rootSources = request.rootSources;
  commit.effectiveParams = &result.batch.effectiveParams;
  commit.setupFrames = request.setupFrames;
  result.committed = ProfileToolpathWorkflowController::commit(commit);
  if (!result.committed.ok) {
    result.failure = ProfileToolpathGenerationFailure::CommitFailed;
    result.error = QCoreApplication::translate("OpenAutoPath", "外轮廓加工项目提交失败：\n%1")
                       .arg(result.committed.error);
    return result;
  }

  const toolpath::ProfileToolpathBatch *committedBatch =
      result.committed.job.profileResult();
  if (!committedBatch || committedBatch->operations.empty()) {
    result.failure = ProfileToolpathGenerationFailure::CommitFailed;
    result.error =
        QCoreApplication::translate("OpenAutoPath", "外轮廓加工项目提交后没有可安装的运行期程序。");
    return result;
  }
  result.batch.operations = committedBatch->operations;
  result.program = committedBatch->toProgram();

  result.summary =
      QCoreApplication::translate("OpenAutoPath", "已生成 %1 道外轮廓工序（%2 个来源跳过）%3；"
                     "刀具中心按半径和侧壁余量位于工件外侧。")
          .arg(result.program.operations.size())
          .arg(result.batch.failures.size())
          .arg(result.automatic ? automaticSummary(result.automaticSelection)
                                : QString());
  if (std::any_of(result.batch.operations.begin(), result.batch.operations.end(),
          [](const auto& operation) { return operation.tool.cornerRadius > 0.0; }))
    result.summary += QCoreApplication::translate("OpenAutoPath", " 球刀或圆鼻刀可能在底部、根部留下余料，请使用 IPW 检查。");
  if (!result.directionReport.isEmpty())
    result.summary += QStringLiteral("\n") + result.directionReport.join(QStringLiteral("\n"));
  result.failure = ProfileToolpathGenerationFailure::None;
  return result;
}

QStringList ProfileToolpathGenerationUseCase::buildReport(
    const ProfileToolpathGenerationResult &result) {
  QStringList lines;
  lines << QCoreApplication::translate("OpenAutoPath", "OpenAutoPath 外轮廓刀路日志")
        << QCoreApplication::translate("OpenAutoPath", "时间: %1")
               .arg(QDateTime::currentDateTime().toString(Qt::ISODateWithMs))
        << QCoreApplication::translate("OpenAutoPath", "选刀模式: %1")
               .arg(result.automatic ? QCoreApplication::translate("OpenAutoPath", "活动刀库自动选刀") : QCoreApplication::translate("OpenAutoPath", "单刀"))
        << QCoreApplication::translate("OpenAutoPath", "输入区域: %1").arg(result.regions.size())
        << QCoreApplication::translate("OpenAutoPath", "生成工序: %1").arg(result.batch.operations.size())
        << QCoreApplication::translate("OpenAutoPath", "跳过区域: %1").arg(result.batch.failures.size())
        << QCoreApplication::translate("OpenAutoPath", "算法/物化/安全/总耗时: %1 / %2 / %3 / %4 ms")
               .arg(result.batch.metrics.algorithmMilliseconds, 0, 'f', 1)
               .arg(result.batch.metrics.materializationMilliseconds, 0, 'f', 1)
               .arg(result.batch.metrics.safetyValidationMilliseconds, 0, 'f',
                    1)
               .arg(result.batch.metrics.totalMilliseconds, 0, 'f', 1)
        << QCoreApplication::translate("OpenAutoPath", "Profile 安全验证: %1")
               .arg(QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(
                   result.batch.safetyValidation.summary()).toUtf8().constData()))
        << QCoreApplication::translate("OpenAutoPath", "统一程序验证: %1")
               .arg(QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(
                   result.validation.program.summary()).toUtf8().constData()));
  if (result.automatic) {
    lines << QCoreApplication::translate("OpenAutoPath", "自动候选刀具: %1")
                 .arg(result.automaticSelection.candidateToolCount)
          << QCoreApplication::translate("OpenAutoPath", "自动阶段: %1")
                 .arg(result.automaticSelection.stages.size());
  }
  lines << result.directionReport;
  lines << QCoreApplication::translate("OpenAutoPath", "结果: %1")
               .arg(result.ok() ? result.summary : result.error);
  return lines;
}

} // namespace oap::app

// MainWindow 加工与刀路命令编排。
#include "MainWindow.h"

#include "app/orchestration/DesktopWorkspaceCoordinator.h"
#include "app/recognition/RecognitionWorkspaceCoordinator.h"
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"
#include "app/ui/dialogs/ToolLibraryDialog.h"
#include "app/ui/dialogs/GCodeReviewDialog.h"
#include "app/ui/panels/HoleToolTreePanel.h"
#include "app/ui/playback/PlaybackUiController.h"
#include "app/workflows/hole/HoleToolpathGenerationUseCase.h"
#include "app/workflows/hole/HoleToolpathRunner.h"
#include "app/workflows/pocket/PocketToolpathGenerationUseCase.h"
#include "app/workflows/profile/ProfileToolpathGenerationUseCase.h"
#include "analysis/MoldSurfaceClassifier.h"
#include "app/dialogs/QtWorkflowInteraction.h"
#include "app/workflows/shared/ReferenceNcExportService.h"
#include "app/workflows/shared/ToolpathExportController.h"
#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/dialogs/ToolpathParameterDialog.h"
#include "toolpath/pocket/planning/PocketClearing.h"
#include "toolpath/pocket/program/PocketToolpathExport.h"
#include "toolpath/program/validation/ToolpathProgramValidator.h"
#include "toolpath/profile/program/ProfileProgramAdapter.h"

#include <algorithm>
#include <array>
#include <optional>
#include <set>
#include <sstream>
#include <tuple>

#include <QAction>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QInputDialog>
#include <QMessageBox>
#include <QSaveFile>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QStringList>

namespace {
constexpr int kPocketPreviewCutMoveBudget = 50000;

QString clearingPatternText(oap::toolpath::ClearingPattern pattern)
{
    switch (pattern) {
        case oap::toolpath::ClearingPattern::Offset:
            return QCoreApplication::translate("OpenAutoPath", "回字形");
        case oap::toolpath::ClearingPattern::ZigZag:
            return QCoreApplication::translate("OpenAutoPath", "Z字形");
        case oap::toolpath::ClearingPattern::Spiral:
            return QCoreApplication::translate("OpenAutoPath", "螺旋形");
        case oap::toolpath::ClearingPattern::FreePath:
            return QCoreApplication::translate("OpenAutoPath", "自适应清腔（FreePath）");
    }
    return QCoreApplication::translate("OpenAutoPath", "回字形");
}

QString validationMessage(
    const oap::toolpath::ToolpathValidationReport& report,
    int maxIssues = 5)
{
    QString message = QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(report.summary()).toUtf8().constData());
    const int issueCount = static_cast<int>(report.issues.size());
    const int visibleCount = std::min(maxIssues, issueCount);
    int appended = 0;
    const auto appendSeverity =
        [&](oap::toolpath::ToolpathIssueSeverity severity) {
        for (const oap::toolpath::ToolpathIssue& issue : report.issues) {
            if (appended >= visibleCount || issue.severity != severity)
                continue;
            message += QLatin1Char('\n');
            message += issue.severity ==
                    oap::toolpath::ToolpathIssueSeverity::Error
                ? QStringLiteral("ERROR ")
                : QStringLiteral("WARN ");
            message += QString::fromStdString(issue.code);
            if (!issue.operationId.empty()) {
                message += QStringLiteral(" [") +
                    QString::fromStdString(issue.operationId) +
                    QLatin1Char(']');
            }
            if (issue.moveIndex >= 0) {
                message += QStringLiteral(" move %1")
                    .arg(issue.moveIndex);
            }
            message += QStringLiteral(": ") +
                QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(issue.message).toUtf8().constData());
            ++appended;
        }
    };
    appendSeverity(oap::toolpath::ToolpathIssueSeverity::Error);
    appendSeverity(oap::toolpath::ToolpathIssueSeverity::Warning);
    if (issueCount > appended) {
        message += QCoreApplication::translate("OpenAutoPath", "\n……另有 %1 个问题")
            .arg(issueCount - appended);
    }
    return message;
}

QString projectRootPath()
{
    const QStringList seeds{
        QDir::currentPath(), QCoreApplication::applicationDirPath()};
    for (const QString& seed : seeds) {
        QDir directory(seed);
        for (int level = 0; level < 8; ++level) {
            if (directory.exists(QStringLiteral("src")))
                return directory.absolutePath();
            if (!directory.cdUp())
                break;
        }
    }
    return QDir::currentPath();
}

bool writeProjectLog(
    const QString& fileName,
    const QStringList& lines,
    QString* writtenPath = nullptr)
{
    const QString path = QDir(projectRootPath()).filePath(fileName);
    if (writtenPath)
        *writtenPath = path;
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return false;
    const QByteArray utf8 =
        (lines.join(QLatin1Char('\n')) + QLatin1Char('\n')).toUtf8();
    return file.write(utf8) == utf8.size() && file.commit();
}

} // namespace

const oap::toolpath::PocketRegionWorkflowResult&
MainWindow::cachedMainOuterInnerWirePocketRegions()
{
    return m_recognitionWorkspace->ensurePocketRegionCache(
        stepIndexer());
}

void MainWindow::showHolePaths()
{
    if (m_uiBusy)
        return;
    if (model().faces.empty()) {
        if (m_actHolePaths) {
            const QSignalBlocker blocker(m_actHolePaths);
            m_actHolePaths->setChecked(false);
        }
        QMessageBox::information(
            this, QCoreApplication::translate("OpenAutoPath", "显示孔刀路"),
            QCoreApplication::translate("OpenAutoPath", "请先打开并分析一个 STEP 文件。"));
        return;
    }
    if (m_actHolePaths && !m_actHolePaths->isChecked()) {
        clearOverlay();
        if (!m_view->view().IsNull())
            m_view->view()->Redraw();
        statusBar()->showMessage(QCoreApplication::translate("OpenAutoPath", "孔刀路已隐藏"));
        return;
    }
    if (m_machiningWorkspace->hasHoleProgram() &&
        !m_machiningWorkspace->holeToolpathNeedsRegeneration() &&
        m_machiningWorkspace->hasRecognizedHoleSources()) {
        clearOverlay(false);
        m_machiningWorkspace->activateInstalledHoleProgram();
        rebuildHolePreviewDisplay();
        rebuildHolePlaybackMoves();
        if (!m_view->view().IsNull())
            m_view->view()->Redraw();
        statusBar()->showMessage(
            QCoreApplication::translate("OpenAutoPath", "已恢复当前活动孔程序显示。"));
        return;
    }

    QElapsedTimer recognitionTimer;
    recognitionTimer.start();
    oap::Bbox moldBounds;
    for (const oap::Unit& unit : model().units)
        moldBounds.merge(unit.bbox);

    const std::vector<oap::toolpath::HoleMachiningInput>*
        recognizedPaths = nullptr;
    QString recognitionSource;
    if (m_recognitionWorkspace->hasResult()) {
        recognizedPaths =
            &m_recognitionWorkspace->holeToolpathInputs();
        recognitionSource = QCoreApplication::translate("OpenAutoPath", "完整特征识别缓存");
    } else {
        if (!m_recognitionWorkspace->hasHoleToolpathInputs()) {
            std::string recognitionError;
            std::optional<oap::HoleGeometryRecognitionResult> recognition =
                oap::app::recognizeHolesWithProgress(
                    makeWorkflowInteraction().showProgress,
                    model(), stepIndexer(), &recognitionError);
            if (!recognition) {
                if (m_actHolePaths) {
                    const QSignalBlocker blocker(m_actHolePaths);
                    m_actHolePaths->setChecked(false);
                }
                statusBar()->showMessage(
                    recognitionError.empty() ? QCoreApplication::translate("OpenAutoPath", "孔识别已取消。")
                                             : QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(recognitionError).toUtf8().constData()));
                return;
            }
            recognizedPaths =
                &m_recognitionWorkspace->installHoleGeometryRecognition(
                    std::move(*recognition), model(), moldFrame());
            recognitionSource = QCoreApplication::translate("OpenAutoPath", "孔专用识别");
        } else {
            recognitionSource = QCoreApplication::translate("OpenAutoPath", "孔专用缓存");
        }
        if (!recognizedPaths) {
            recognizedPaths =
                &m_recognitionWorkspace->holeToolpathInputs();
        }
    }
    const qint64 recognitionMilliseconds =
        recognitionTimer.elapsed();
    if (!recognizedPaths || recognizedPaths->empty()) {
        if (m_actHolePaths) {
            const QSignalBlocker blocker(m_actHolePaths);
            m_actHolePaths->setChecked(false);
        }
        m_view->view()->Redraw();
        QMessageBox::information(
            this, QCoreApplication::translate("OpenAutoPath", "显示孔刀路"),
            QCoreApplication::translate("OpenAutoPath", "此模型中未识别到孔。"));
        return;
    }

    const bool singleTool = m_machiningWorkspace->holeToolSelectionMode() == oap::app::ToolSelectionMode::SingleTool;
    const oap::tooling::ToolLibrary* library = singleTool ? nullptr : activeToolLibrary();
    if (!singleTool && (!library || library->assignedTools().empty())) {
        if (m_actHolePaths) {
            const QSignalBlocker blocker(m_actHolePaths);
            m_actHolePaths->setChecked(false);
        }
        QMessageBox::information(
            this, QCoreApplication::translate("OpenAutoPath", "显示孔刀路"),
            QCoreApplication::translate("OpenAutoPath", "孔刀路当前使用活动刀库自动选刀。请先选择刀库并分配至少一把适用刀具。"));
        return;
    }

    const bool hadInstalledHoleProgram =
        m_machiningWorkspace->hasHoleProgram();
    oap::app::HoleToolpathGenerationRequest request;
    request.interaction = makeWorkflowInteraction();
    request.recognized = recognizedPaths;
    request.model = &model();
    request.indexer = &stepIndexer();
    request.moldBounds = moldBounds;
    request.moldFrame = &moldFrame();
    request.params = &m_machiningWorkspace->holeParams();
    request.automaticLibrary = library;
    request.singleTool = singleTool ? &m_machiningWorkspace->holeSingleTool() : nullptr;
    request.definition = machiningProjectDefinitionContext();
    request.setupFrames = &moldSetupWorkFrames();
    oap::app::HoleToolpathGenerationResult generated =
        m_machiningWorkspace->generateHoleToolpath(request);

    const QString rapidLogPath =
        QDir(projectRootPath()).filePath(
            QCoreApplication::translate("OpenAutoPath", "孔刀路避障日志.txt"));
    oap::app::HoleToolpathGenerationReportContext reportContext;
    reportContext.logPath = rapidLogPath;
    reportContext.recognitionSource = recognitionSource;
    reportContext.recognitionMilliseconds =
        recognitionMilliseconds;
    reportContext.aagBuildMilliseconds =
        m_recognitionWorkspace->holeGeometryProfile().aagBuildMs;
    reportContext.holeClassificationMilliseconds =
        m_recognitionWorkspace->holeGeometryProfile()
            .holeClassificationMs;
    reportContext.geometryBuildMilliseconds =
        m_recognitionWorkspace->holeGeometryProfile().geometryBuildMs;
    if (!writeProjectLog(
            QCoreApplication::translate("OpenAutoPath", "孔刀路避障日志.txt"),
            oap::app::HoleToolpathGenerationUseCase::buildReport(
                generated, reportContext))) {
        qWarning().noquote()
            << "[OpenAutoPath] cannot write hole rapid log:"
            << rapidLogPath;
    }

    for (const std::string& warning :
         generated.batch.machining.warnings) {
        qDebug().noquote()
            << "[OpenAutoPath] hole machining:"
            << QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(warning).toUtf8().constData());
    }
    using Failure = oap::app::HoleToolpathGenerationFailure;
    if (generated.failure == Failure::Cancelled) {
        if (!hadInstalledHoleProgram && m_actHolePaths) {
            const QSignalBlocker blocker(m_actHolePaths);
            m_actHolePaths->setChecked(false);
        }
        statusBar()->showMessage(
            hadInstalledHoleProgram
                ? QCoreApplication::translate("OpenAutoPath", "孔刀路生成已取消，保留原孔程序。")
                : QCoreApplication::translate("OpenAutoPath", "孔刀路生成已取消。"));
        return;
    }
    if (!generated.ok()) {
        if (!hadInstalledHoleProgram &&
            (generated.failure == Failure::EmptyProgram ||
             generated.failure == Failure::ValidationFailed)) {
            if (m_actHolePaths) {
                const QSignalBlocker blocker(m_actHolePaths);
                m_actHolePaths->setChecked(false);
            }
            m_view->view()->Redraw();
        }
        if (generated.failure == Failure::ValidationFailed ||
            generated.failure == Failure::CommitFailed ||
            generated.failure == Failure::InvalidRequest) {
            QMessageBox::warning(
                this, QCoreApplication::translate("OpenAutoPath", "显示孔刀路"),
                generated.error);
        } else {
            statusBar()->showMessage(generated.error);
            QMessageBox::information(
                this, QCoreApplication::translate("OpenAutoPath", "显示孔刀路"),
                generated.error);
        }
        return;
    }

    m_stockSimulationWorkspace->clearHole();
    clearOverlay(false);
    m_machiningWorkspace->installHoleProgram(
        std::move(generated.committed.job),
        std::move(generated.geometry.sources),
        generated.batch.effectiveParams);
    rebuildHolePreviewDisplay();
    m_view->view()->Redraw();

    QFile previewLog(rapidLogPath);
    if (previewLog.open(
            QIODevice::WriteOnly | QIODevice::Append |
                QIODevice::Text)) {
        previewLog.write(
            QCoreApplication::translate("OpenAutoPath", "孔预览批量构建与显示耗时: %1 ms\n")
                .arg(m_lastHolePreviewBuildMs).toUtf8());
    }

    QString status = generated.summary;
    const auto& validation = generated.batch.validation;
    const auto& machining = generated.batch.machining;
    if (validation.warningCount > 0) {
        qDebug().noquote()
            << "[OpenAutoPath] hole toolpath validation:"
            << validationMessage(validation);
        status += QCoreApplication::translate("OpenAutoPath", " 验证警告 %1。")
                      .arg(validation.warningCount);
    } else if (!machining.warnings.empty()) {
        status += QCoreApplication::translate("OpenAutoPath", " 生成警告 %1。")
                      .arg(machining.warnings.size());
    }
    statusBar()->showMessage(status);
    for (const std::string& error :
         m_machiningWorkspace->validationErrors()) {
        qWarning().noquote()
            << "[OpenAutoPath] machining job:"
            << QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(error).toUtf8().constData());
    }
    rebuildHolePlaybackMoves();
}

void MainWindow::openToolLibrary()
{
    if (m_uiBusy)
        return;
    const QString previousActiveId =
        m_machiningWorkspace->activeToolLibraryId();
    ToolLibraryDialog dialog(
        m_machiningWorkspace->toolCatalog(),
        previousActiveId, this);
    if (dialog.exec() != QDialog::Accepted)
        return;

    QString activeId = previousActiveId;
    const bool activeStillExists = std::any_of(
        dialog.libraries().cbegin(), dialog.libraries().cend(),
        [&activeId](const oap::tooling::ToolLibrary& library) {
            return library.id() == activeId;
        });
    if (!activeStillExists) {
        activeId = dialog.selectedLibraryId();
        const bool selectedIsLibrary = std::any_of(
            dialog.libraries().cbegin(), dialog.libraries().cend(),
            [&activeId](const oap::tooling::ToolLibrary& library) {
                return library.id() == activeId;
            });
        if (!selectedIsLibrary && !dialog.libraries().isEmpty())
            activeId = dialog.libraries().front().id();
    }
    const oap::app::ToolLibraryApplyResult applied =
        m_machiningWorkspace->applyToolLibraries(
            dialog.catalog(), activeId);
    if (!applied.ok) {
        QMessageBox::warning(this, QCoreApplication::translate("OpenAutoPath", "刀库"),
                             applied.error);
        return;
    }
    if (!applied.warning.isEmpty()) {
        QMessageBox::warning(this, QCoreApplication::translate("OpenAutoPath", "保存刀库"),
                             QCoreApplication::translate("OpenAutoPath", "刀库目录保存失败：\n%1")
                                 .arg(applied.warning));
    }

    synchronizeInvalidatedToolpathPreview();
    const oap::tooling::ToolLibrary* selectedLibrary = activeToolLibrary();
    if (!selectedLibrary)
        return;
    statusBar()->showMessage(
        QCoreApplication::translate("OpenAutoPath", "刀库目录已保存；活动刀库保持为“%1”（%2 把已分配 T 刀具）。%3")
            .arg(selectedLibrary->name())
            .arg(selectedLibrary->assignedTools().size())
            .arg(m_machiningWorkspace->hasPendingRegeneration()
                ? QCoreApplication::translate("OpenAutoPath", "依赖变更刀库的工序已标记为待重新生成。")
                : QString()));
    refreshCamWorkflowPanel();
}

void MainWindow::selectToolLibrary()
{
    if (m_uiBusy)
        return;
    ToolLibrarySelectionDialog dialog(
        m_machiningWorkspace->toolLibraries(),
        m_machiningWorkspace->activeToolLibraryId(), this);
    if (dialog.exec() != QDialog::Accepted)
        return;

    const oap::app::ToolLibraryApplyResult applied =
        m_machiningWorkspace->applyToolLibraries(
            m_machiningWorkspace->toolCatalog(),
            dialog.selectedLibraryId());
    if (!applied.ok) {
        QMessageBox::warning(
            this, QCoreApplication::translate("OpenAutoPath", "选择活动刀库"), applied.error);
        return;
    }
    if (!applied.warning.isEmpty()) {
        QMessageBox::warning(
            this, QCoreApplication::translate("OpenAutoPath", "保存活动刀库"), applied.warning);
    }
    synchronizeInvalidatedToolpathPreview();
    const oap::tooling::ToolLibrary* selectedLibrary = activeToolLibrary();
    if (!selectedLibrary)
        return;
    statusBar()->showMessage(
        QCoreApplication::translate("OpenAutoPath", "已选择 CAM 活动刀库“%1”：%2 把已分配 T 刀具。%3")
            .arg(selectedLibrary->name())
            .arg(selectedLibrary->assignedTools().size())
            .arg(m_machiningWorkspace->hasPendingRegeneration()
                ? QCoreApplication::translate("OpenAutoPath", "依赖变更刀库的工序已标记为待重新生成。")
                : QCoreApplication::translate("OpenAutoPath", "后续自动选刀将使用该刀库。")));
    refreshCamWorkflowPanel();
}

void MainWindow::configureToolpathParams()
{
    configureToolpathParamsForPage(oap::app::ToolpathParameterEditorPage::Hole);
}

void MainWindow::synchronizeInvalidatedToolpathPreview()
{
    if (m_machiningWorkspace->hasPendingRegeneration()) {
        m_stockSimulationWorkspace->clearHole();
        m_stockSimulationWorkspace->clearPocket();
    }
    if (m_machiningWorkspace->activePreviewNeedsRegeneration()) {
        if (m_actHolePaths) {
            const QSignalBlocker blocker(m_actHolePaths);
            m_actHolePaths->setChecked(false);
        }
        if (m_playbackUi)
            m_playbackUi->clearPanel();
        m_playbackWorkspace->clearProgram();
    }
    rebuildHoleToolTree();
    refreshFeatureAllowanceView();
    refreshCamWorkflowPanel();
    if (m_view && !m_view->view().IsNull())
        m_view->view()->Redraw();
}

void MainWindow::configureEngravingParams()
{
    configureToolpathParamsForPage(oap::app::ToolpathParameterEditorPage::Engraving);
}

void MainWindow::configureToolpathParamsForPage(oap::app::ToolpathParameterEditorPage page)
{
    if (m_uiBusy)
        return;
    const double moldLength = moldFrame().isValid
        ? moldFrame().length : 0.0;
    const double moldWidth = moldFrame().isValid
        ? moldFrame().width : 0.0;
    const oap::app::ToolpathParameterEditorState editorState =
        m_machiningWorkspace->configurationEditorState(
            moldSetupWorkFrames(), moldLength, moldWidth);

    std::optional<oap::app::ToolpathParameterEditorState> edited =
        oap::app::editToolpathParameters(this, editorState, page);
    if (!edited)
        return;

    const oap::app::MachiningConfigurationApplyResult applied =
        m_machiningWorkspace->applyConfiguration(
            std::move(*edited), *m_documentWorkspace);
    // 工作区统一处理失效，UI只同步受影响的显示和仿真证据。
    if (applied.holeParametersChanged || applied.pocketParametersChanged ||
        applied.profileParametersChanged || applied.engravingParametersChanged)
        synchronizeInvalidatedToolpathPreview();
    if (!applied.persistenceError.isEmpty()) {
        QMessageBox::warning(
            this, QCoreApplication::translate("OpenAutoPath", "保存刀路参数"),
            QCoreApplication::translate("OpenAutoPath", "当前参数已应用，但无法保存到本地：\n%1")
                .arg(applied.persistenceError));
    }

    refreshCamWorkflowPanel();
    statusBar()->showMessage(QCoreApplication::translate("OpenAutoPath", "刀路参数已更新，请重新生成受影响的加工工序。"));
}

oap::app::MachiningProjectLoadRequest
MainWindow::currentMachiningRegenerationRequest() const
{
    oap::app::MachiningProjectLoadRequest request;
    request.currentSourcePath = currentPath();
    request.model = &model();
    request.indexer = &stepIndexer();
    request.moldFrame = &moldFrame();
    request.setupFrames = &moldSetupWorkFrames();
    return request;
}

oap::app::MachiningProjectDefinitionContext
MainWindow::machiningProjectDefinitionContext() const
{
    oap::app::MachiningProjectDefinitionContext context;
    context.sourcePath = currentPath();
    context.activeToolLibraryId =
        m_machiningWorkspace->activeToolLibraryId();
    context.gcodeOptions = m_machiningWorkspace->gcodeOptions();
    context.stock = m_stockSimulationWorkspace->holeStockDefinition();
    return context;
}

bool MainWindow::ensureMachiningJobDefinition(QString& error)
{
    return m_machiningWorkspace->ensureProjectDefinition(
        machiningProjectDefinitionContext(), &error);
}

bool MainWindow::synchronizePocketJobFromResult(QString& error)
{
    oap::app::MachiningProjectPocketCaptureRequest request;
    request.definition = machiningProjectDefinitionContext();
    request.rootSources = &m_recognitionWorkspace->pocketSources();
    request.setupFrames = &moldSetupWorkFrames();
    return m_machiningWorkspace->capturePocketProjectOperations(
        request, &error);
}

void MainWindow::showPocketClearing()
{
    if (m_uiBusy)
        return;
    if (model().faces.empty()) {
        QMessageBox::information(
            this, QCoreApplication::translate("OpenAutoPath", "显示腔清腔"),
            QCoreApplication::translate("OpenAutoPath", "请先打开并分析一个 STEP 文件。"));
        return;
    }
    if (m_view->context().IsNull())
        return;

    if (!m_recognitionWorkspace->hasResult()) {
        QMessageBox::information(
            this, QCoreApplication::translate("OpenAutoPath", "显示腔清腔"),
            QCoreApplication::translate("OpenAutoPath", "请先执行内外环结构识别。"));
        return;
    }

    // 候选生成会清空共享的刀路展示,以便在隔离状态下构建新的 OCCT 预览。
    // 保留一份值快照,在所有失败/取消路径上恢复它;已装载的 Job 在生成
    // 用例成功提交之前不会被改动。
    const oap::app::PocketPreviewSnapshot previousPreview =
        m_machiningWorkspace->capturePocketPreview();
    const auto restorePreviousPreview = [&]() {
        clearToolpathOverlay();
        m_machiningWorkspace->restorePocketPreview(previousPreview);
        if (previousPreview.activePreview ==
                ActiveToolpathPreview::Pocket &&
            previousPreview.previewActive &&
            !previousPreview.entries.empty()) {
            for (const PocketPreviewEntry& entry :
                 m_machiningWorkspace->pocketPreviewEntries()) {
                displayPocketPreviewEntry(entry);
            }
            rebuildPocketPlaybackMoves();
        } else if (previousPreview.activePreview ==
                       ActiveToolpathPreview::Hole &&
                   m_machiningWorkspace->hasHoleProgram()) {
            rebuildHolePreviewDisplay();
            rebuildHolePlaybackMoves();
        } else if (previousPreview.activePreview ==
                       ActiveToolpathPreview::Profile &&
                   m_machiningWorkspace->hasProfilePreview()) {
            const oap::toolpath::ToolpathProgram program =
                m_machiningWorkspace->profilePreviewProgram();
            m_playbackWorkspace->replaceProgramPreview(
                program, sceneController().modelTransform());
            rebuildPlaybackMovesFromProgram(program, false);
        } else if (previousPreview.activePreview == ActiveToolpathPreview::SurfaceFinishing &&
                   m_machiningWorkspace->hasFinishingPreview()) {
            const auto program = m_machiningWorkspace->finishingPreviewProgram();
            m_playbackWorkspace->replaceProgramPreview(program, sceneController().modelTransform());
            rebuildPlaybackMovesFromProgram(program, false);
        }
    };
    clearOverlay();

    const int candidateCount = static_cast<int>(
        m_recognitionWorkspace->pocketSources().size());
    const std::vector<oap::toolpath::PocketRegion> emptyRegions;
    const std::vector<oap::toolpath::SkippedRegion> emptySkipped;
    const std::vector<oap::toolpath::PocketRegion>* regions =
        &emptyRegions;
    const std::vector<oap::toolpath::SkippedRegion>* buildSkipped =
        &emptySkipped;
    bool regionCacheHit = false;
    qint64 regionBuildMilliseconds = 0;
    if (candidateCount > 0) {
        regionCacheHit = m_recognitionWorkspace->hasPocketRegionCache();
        QElapsedTimer timer;
        timer.start();
        const oap::toolpath::PocketRegionWorkflowResult& regionResult =
            cachedMainOuterInnerWirePocketRegions();
        regionBuildMilliseconds = timer.elapsed();
        regions = &regionResult.regions;
        buildSkipped = &regionResult.skippedRegions;
    }

    oap::app::PocketToolpathGenerationRequest request;
    request.interaction = makeWorkflowInteraction();
    request.candidateCount = candidateCount;
    request.regions = regions;
    request.buildSkipped = buildSkipped;
    request.regionCacheHit = regionCacheHit;
    request.regionBuildMilliseconds = regionBuildMilliseconds;
    request.params = &m_machiningWorkspace->pocketParams();
    if (m_machiningWorkspace->pocketToolSelectionMode() == oap::app::ToolSelectionMode::SingleTool)
        request.singleTool = &m_machiningWorkspace->pocketSingleTool();
    else
        request.automaticLibrary = activeToolLibrary();
    request.moldFrame = &moldFrame();
    request.definition = machiningProjectDefinitionContext();
    request.rootSources = &m_recognitionWorkspace->pocketSources();
    request.setupFrames = &moldSetupWorkFrames();
    request.previewCutMoveBudget = kPocketPreviewCutMoveBudget;

    oap::app::PocketToolpathGenerationResult generated =
        m_machiningWorkspace->generatePocketToolpath(
            request,
            [this](PocketPreviewEntry& entry) {
                return displayPocketPreviewEntry(entry);
            });

    QString logPath;
    if (!writeProjectLog(
            QCoreApplication::translate("OpenAutoPath", "腔清腔日志.txt"),
            generated.reportLines, &logPath)) {
        qWarning().noquote()
            << "[OpenAutoPath] cannot write pocket clearing log:"
            << logPath;
    }

    if (!generated.ok()) {
        using Failure = oap::app::PocketToolpathGenerationFailure;
        restorePreviousPreview();
        m_view->view()->Redraw();
        if (generated.failure == Failure::Cancelled) {
            statusBar()->showMessage(generated.error);
            return;
        }
        statusBar()->showMessage(generated.error);
        if (generated.failure == Failure::ValidationFailed ||
            generated.failure == Failure::CommitFailed) {
            QMessageBox::warning(
                this, QCoreApplication::translate("OpenAutoPath", "显示腔清腔"), generated.error);
        } else {
            QMessageBox::information(
                this, QCoreApplication::translate("OpenAutoPath", "显示腔清腔"), generated.error);
        }
        return;
    }

    m_workspaceCoordinator->publishPocketProgram(
        std::move(generated.candidateJob.job),
        std::move(generated.previewEntries),
        m_machiningWorkspace->pocketParams(),
        generated.batch.effectiveParams);

    if (generated.metrics.skippedPreviewCutMoves > 0) {
        qDebug().nospace()
            << "[OpenAutoPath] pocket spiral preview skipped cut moves="
            << generated.metrics.skippedPreviewCutMoves
            << " stride=" << generated.metrics.cutPreviewStride;
    }
    if (generated.metrics.skippedRegions > 0) {
        qDebug().nospace()
            << "[OpenAutoPath] pocket clearing skipped regions="
            << generated.metrics.skippedRegions;
    }
    if (!generated.metrics.generationWarnings.isEmpty()) {
        qDebug().noquote()
            << "[OpenAutoPath] pocket clearing warnings:"
            << generated.metrics.generationWarnings.join(
                   QStringLiteral(" | "));
    }
    if (generated.batch.validation.warningCount > 0) {
        qDebug().noquote()
            << "[OpenAutoPath] pocket toolpath validation:"
            << validationMessage(generated.batch.validation);
    }

    m_view->view()->Redraw();
    statusBar()->showMessage(generated.summary);
    rebuildPocketPlaybackMoves();
}

void MainWindow::showProfileContour()
{
    const QString title = QCoreApplication::translate("OpenAutoPath", "外轮廓刀路");
    if (m_uiBusy)
        return;
    if (model().faces.empty()) {
        QMessageBox::information(
            this, title, QCoreApplication::translate("OpenAutoPath", "请先打开并分析一个 STEP 文件。"));
        return;
    }
    if (!m_recognitionWorkspace->hasResult()) {
        QMessageBox::information(
            this, title,
            QCoreApplication::translate("OpenAutoPath", "请先执行“识别 CAM 特征”。"));
        return;
    }
    if (m_recognitionWorkspace->profileSources().empty()) {
        QMessageBox::information(
            this, title,
            QCoreApplication::translate("OpenAutoPath", "未识别到可加工的闭合外轮廓。当前版本仅接受主外平面的真实 outer wire。"));
        return;
    }

    const std::vector<oap::toolpath::ProfileRegion>& recognizedRegions =
        m_recognitionWorkspace->ensureProfileRegionCache(stepIndexer());
    if (recognizedRegions.empty()) {
        QMessageBox::information(
            this, title,
            QCoreApplication::translate("OpenAutoPath", "外轮廓来源未能构建为可加工区域；请检查侧壁是否为直壁，且不存在锥面、自由曲面或倒扣。"));
        return;
    }

    const bool singleTool = m_machiningWorkspace->profileToolSelectionMode() == oap::app::ToolSelectionMode::SingleTool;
    std::set<oap::MoldSetupFace> profileFaces;
    for (const auto& region : recognizedRegions) {
        const auto face = oap::classifyMoldSetupFace(moldFrame(), gp_Vec(region.openDir));
        if (face != oap::MoldSetupFace::Unresolved) profileFaces.insert(face);
    }
    QStringList scopeLabels{QCoreApplication::translate("OpenAutoPath", "自动规划方向（仅合并已证明覆盖的重复方向）"),
                            QCoreApplication::translate("OpenAutoPath", "全部方向（所选范围须全部生成成功）")};
    std::vector<oap::MoldSetupFace> scopeFaces(profileFaces.begin(), profileFaces.end());
    for (const auto face : scopeFaces) {
        int count = 0;
        double maximumDepth = 0.0;
        for (const auto& region : recognizedRegions) {
            if (oap::classifyMoldSetupFace(moldFrame(), gp_Vec(region.openDir)) != face) continue;
            ++count;
            maximumDepth = std::max(maximumDepth, region.depth);
        }
        scopeLabels.push_back(QCoreApplication::translate("OpenAutoPath", "%1 · %2 个区域 · 最大深度 %3 mm")
            .arg(QString::fromLatin1(oap::moldSetupFaceName(face))).arg(count).arg(maximumDepth, 0, 'f', 2));
    }
    bool scopeAccepted = false;
    const QString scope = QInputDialog::getItem(this, QCoreApplication::translate("OpenAutoPath", "外轮廓加工方向"),
        QCoreApplication::translate("OpenAutoPath", "选择本次生成方向（按装夹坐标系）。\n成功后替换现有外轮廓工序；其他类型工序保留。"),
        scopeLabels, 0, false, &scopeAccepted);
    if (!scopeAccepted) return;
    const int selectedScope = scopeLabels.indexOf(scope);
    const oap::tooling::ToolLibrary* library = singleTool ? nullptr : activeToolLibrary();
    {
        if (!singleTool && (!library || library->assignedTools().empty())) {
            QMessageBox::information(
                this, title,
                QCoreApplication::translate("OpenAutoPath", "外轮廓当前使用活动刀库自动选刀。请先选择刀库并分配至少一把平底铣刀、球刀或圆鼻刀。"));
            return;
        }
    }

    const oap::app::PocketPreviewSnapshot previousPreview =
        m_machiningWorkspace->capturePocketPreview();
    const auto restorePreviousPreview = [&]() {
        clearToolpathOverlay();
        m_machiningWorkspace->restorePocketPreview(previousPreview);
        if (previousPreview.activePreview ==
                ActiveToolpathPreview::Pocket &&
            previousPreview.previewActive &&
            !previousPreview.entries.empty()) {
            for (const PocketPreviewEntry& entry :
                 m_machiningWorkspace->pocketPreviewEntries()) {
                displayPocketPreviewEntry(entry);
            }
            rebuildPocketPlaybackMoves();
        } else if (previousPreview.activePreview ==
                       ActiveToolpathPreview::Hole &&
                   m_machiningWorkspace->hasHoleProgram()) {
            rebuildHolePreviewDisplay();
            rebuildHolePlaybackMoves();
        } else if (previousPreview.activePreview ==
                       ActiveToolpathPreview::Profile &&
                   m_machiningWorkspace->hasProfilePreview()) {
            const oap::toolpath::ToolpathProgram previousProgram =
                m_machiningWorkspace->profilePreviewProgram();
            m_playbackWorkspace->replaceProgramPreview(
                previousProgram, sceneController().modelTransform());
            rebuildPlaybackMovesFromProgram(previousProgram, false);
        } else if (previousPreview.activePreview == ActiveToolpathPreview::SurfaceFinishing &&
                   m_machiningWorkspace->hasFinishingPreview()) {
            const auto program = m_machiningWorkspace->finishingPreviewProgram();
            m_playbackWorkspace->replaceProgramPreview(program, sceneController().modelTransform());
            rebuildPlaybackMovesFromProgram(program, false);
        }
    };

    oap::app::ProfileToolpathGenerationRequest request;
    request.interaction = makeWorkflowInteraction();
    request.recognizedRegions = &recognizedRegions;
    request.automaticSetupDirections = selectedScope == 0;
    if (selectedScope > 1)
        request.selectedSetupFace = scopeFaces[static_cast<std::size_t>(selectedScope - 2)];
    request.params = &m_machiningWorkspace->profileParams();
    request.automaticLibrary = library;
    request.singleTool = singleTool ? &m_machiningWorkspace->profileSingleTool() : nullptr;
    request.moldFrame = &moldFrame();
    request.definition = machiningProjectDefinitionContext();
    request.rootSources = &m_recognitionWorkspace->profileSources();
    request.setupFrames = &moldSetupWorkFrames();

    bool previewMutated = false;
    oap::app::ProfileToolpathGenerationResult generated =
        m_machiningWorkspace->generateProfileToolpath(
            request,
            [this, &previewMutated](
                const oap::toolpath::ToolpathProgram& program) {
                previewMutated = true;
                clearOverlay();
                const oap::app::ToolpathPreviewBuildStats preview =
                    m_playbackWorkspace->replaceProgramPreview(
                        program, sceneController().modelTransform());
                return preview.objectCount > 0;
            });

    QString logPath;
    if (!writeProjectLog(
            QCoreApplication::translate("OpenAutoPath", "外轮廓刀路日志.txt"),
            oap::app::ProfileToolpathGenerationUseCase::buildReport(
                generated),
            &logPath)) {
        qWarning().noquote()
            << "[OpenAutoPath] cannot write profile toolpath log:"
            << logPath;
    }

    if (!generated.ok()) {
        if (previewMutated) {
            restorePreviousPreview();
            m_view->view()->Redraw();
        }
        using Failure = oap::app::ProfileToolpathGenerationFailure;
        statusBar()->showMessage(generated.error);
        if (generated.failure == Failure::Cancelled)
            return;
        if (generated.failure == Failure::ValidationFailed ||
            generated.failure == Failure::CommitFailed ||
            generated.failure == Failure::PreviewFailed ||
            generated.failure == Failure::InvalidRequest) {
            QMessageBox::warning(this, title, generated.error);
        } else {
            QMessageBox::information(this, title, generated.error);
        }
        return;
    }

    m_workspaceCoordinator->publishProfileProgram(
        std::move(generated.committed.job),
        m_machiningWorkspace->profileParams(),
        generated.batch.effectiveParams);
    if (m_actHolePaths) {
        const QSignalBlocker blocker(m_actHolePaths);
        m_actHolePaths->setChecked(false);
    }
    rebuildPlaybackMovesFromProgram(generated.program, false);
    m_view->view()->Redraw();
    statusBar()->showMessage(generated.summary);
    refreshCamWorkflowPanel();
}

void MainWindow::saveMachiningJob()
{
    const QString title = QCoreApplication::translate("OpenAutoPath", "保存加工项目");
    if (m_uiBusy)
        return;
    if (m_machiningWorkspace->hasPendingRegeneration()) {
        QMessageBox::information(this, title,
            QCoreApplication::translate("OpenAutoPath", "参数或刀具来源已变化，请先成功重新生成受影响工序，再保存加工项目。"));
        return;
    }
    QString initializationError;
    if (m_machiningWorkspace->hasPocketPreview() &&
        m_machiningWorkspace->hasPocketPreviewParams() &&
        !synchronizePocketJobFromResult(
            initializationError)) {
        QMessageBox::warning(
            this, title,
            QCoreApplication::translate("OpenAutoPath", "槽腔 Job 捕获失败：\n%1")
                .arg(initializationError));
        return;
    }
    if (m_machiningWorkspace->hasProfilePreview() &&
        m_machiningWorkspace->hasProfilePreviewParams()) {
        oap::app::MachiningProjectProfileCaptureRequest request;
        request.definition = machiningProjectDefinitionContext();
        request.rootSources =
            &m_recognitionWorkspace->profileSources();
        request.setupFrames = &moldSetupWorkFrames();
        if (!m_machiningWorkspace->captureProfileProjectPreview(
                request, &initializationError)) {
            QMessageBox::warning(
                this, title,
                QCoreApplication::translate("OpenAutoPath", "外轮廓 Job 捕获失败：\n%1")
                    .arg(initializationError));
            return;
        }
    }
    if (!m_machiningWorkspace->hasProjectContent()) {
        QMessageBox::information(this, title,
            QCoreApplication::translate("OpenAutoPath", "当前没有可保存的加工工序，请先生成所需工序。"));
        return;
    }
    oap::app::MachiningProjectSaveRequest synchronization;
    synchronization.definition = machiningProjectDefinitionContext();
    synchronization.setupFrames = &moldSetupWorkFrames();
    QString synchronizationError;
    if (!m_machiningWorkspace->synchronizeProjectForSave(
            synchronization, &synchronizationError)) {
        QMessageBox::warning(
            this, title,
            QCoreApplication::translate("OpenAutoPath", "加工项目结构无效：\n%1")
                .arg(synchronizationError));
        return;
    }
    QString suggested = QStringLiteral("machining.oapjob");
    if (!currentPath().isEmpty()) {
        const QFileInfo source(currentPath());
        suggested = source.absolutePath() + QLatin1Char('/') +
                    source.completeBaseName() + QStringLiteral(".oapjob");
    }
    QString path = QFileDialog::getSaveFileName(
        this, title, suggested,
        QCoreApplication::translate("OpenAutoPath", "OpenAutoPath 加工项目 (*.oapjob);;JSON 文件 (*.json);;所有文件 (*.*)"));
    if (path.isEmpty())
        return;
    if (QFileInfo(path).suffix().isEmpty())
        path += QStringLiteral(".oapjob");
    QString error;
    if (!m_machiningWorkspace->saveProject(path, &error)) {
        QMessageBox::warning(
            this, title, QCoreApplication::translate("OpenAutoPath", "保存失败：\n%1").arg(error));
        return;
    }
    statusBar()->showMessage(
        QCoreApplication::translate("OpenAutoPath", "加工项目已保存：%1").arg(path));
    m_currentMachiningJobPath = path;
    refreshCamWorkflowPanel();
}

void MainWindow::openMachiningJob()
{
    const QString title = QCoreApplication::translate("OpenAutoPath", "打开加工项目");
    if (m_uiBusy)
        return;
    const QString path = QFileDialog::getOpenFileName(
        this, title,
        currentPath().isEmpty()
            ? QString()
            : QFileInfo(currentPath()).absolutePath(),
        QCoreApplication::translate("OpenAutoPath", "OpenAutoPath 加工项目 (*.oapjob *.ffjob *.json);;所有文件 (*.*)"));
    if (path.isEmpty())
        return;

    oap::app::MachiningProjectLoadRequest request;
    request.filePath = path;
    request.currentSourcePath = currentPath();
    request.model = &model();
    request.indexer = &stepIndexer();
    request.moldFrame = &moldFrame();
    request.setupFrames = &moldSetupWorkFrames();
    oap::app::MachiningProjectLoadResult loaded =
        oap::app::MachiningProjectController::loadAndRegenerate(request);
    if (!loaded.ok()) {
        if (loaded.failure ==
            oap::app::MachiningProjectLoadFailure::PartRequired) {
            QMessageBox::information(this, title, loaded.error);
        } else {
            QMessageBox::warning(
                this, title,
                loaded.failure ==
                        oap::app::MachiningProjectLoadFailure::File
                    ? QCoreApplication::translate("OpenAutoPath", "加载失败：\n%1").arg(loaded.error)
                    : loaded.error);
        }
        return;
    }
    clearOverlay(false);
    const auto installed =
        m_workspaceCoordinator->installProject(std::move(loaded));
    for (const std::string& warning : installed.warnings) {
        qWarning().noquote()
            << "[OpenAutoPath] project restore:"
            << QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(warning).toUtf8().constData());
    }
    const auto activePreview = m_machiningWorkspace->activePreview();
    if (activePreview == ActiveToolpathPreview::SurfaceFinishing) {
        const auto program = m_machiningWorkspace->finishingPreviewProgram();
        m_playbackWorkspace->replaceProgramPreview(program, sceneController().modelTransform());
        if (m_actHolePaths) {
            const QSignalBlocker block(m_actHolePaths);
            m_actHolePaths->setChecked(false);
        }
        rebuildPlaybackMovesFromProgram(program, false);
    } else if (activePreview == ActiveToolpathPreview::Profile) {
        const oap::toolpath::ToolpathProgram program =
            m_machiningWorkspace->profilePreviewProgram();
        const oap::app::ToolpathPreviewBuildStats preview =
            m_playbackWorkspace->replaceProgramPreview(
                program, sceneController().modelTransform());
        if (preview.objectCount <= 0) {
            QMessageBox::warning(
                this, title,
                QCoreApplication::translate("OpenAutoPath", "外轮廓刀路已重建，但无法创建显示对象。"));
            return;
        }
        if (m_actHolePaths) {
            QSignalBlocker block(m_actHolePaths);
            m_actHolePaths->setChecked(false);
        }
        rebuildPlaybackMovesFromProgram(program, false);
    } else if (activePreview == ActiveToolpathPreview::Pocket) {
        const int displayedEntryCount =
            m_machiningWorkspace->retainDisplayablePocketPreviewEntries(
                [this](PocketPreviewEntry& entry) {
                    return displayPocketPreviewEntry(entry);
                });
        if (displayedEntryCount == 0) {
            QMessageBox::warning(
                this, title,
                QCoreApplication::translate("OpenAutoPath", "槽腔刀路已重建，但无法创建显示对象。"));
            return;
        }
        if (m_actHolePaths) {
            QSignalBlocker block(m_actHolePaths);
            m_actHolePaths->setChecked(false);
        }
        rebuildPocketPlaybackMoves();
    } else if (activePreview == ActiveToolpathPreview::Hole) {
        if (m_actHolePaths) {
            QSignalBlocker block(m_actHolePaths);
            m_actHolePaths->setChecked(true);
        }
        rebuildHolePreviewDisplay();
        rebuildHolePlaybackMoves();
    } else {
        if (m_actHolePaths) {
            const QSignalBlocker block(m_actHolePaths);
            m_actHolePaths->setChecked(false);
        }
        rebuildHoleToolTree();
    }
    m_view->view()->Redraw();
    m_undoHistory.clear();
    m_currentMachiningJobPath = path;
    statusBar()->showMessage(
        QCoreApplication::translate("OpenAutoPath", "加工项目已加载并重建：孔 %1 道，槽腔 %2 道，外轮廓 %3 道，雕刻 %4 道，%5 个装夹")
            .arg(installed.holeOperationCount)
            .arg(installed.pocketOperationCount)
            .arg(installed.profileOperationCount)
            .arg(installed.finishingOperationCount)
            .arg(installed.setupCount));
    refreshCamWorkflowPanel();
}

void MainWindow::exportCurrentToolpathPreview()
{
    const QString title = QCoreApplication::translate("OpenAutoPath", "导出当前刀路");
    if (m_uiBusy)
        return;

    if (m_machiningWorkspace->activePreview() == ActiveToolpathPreview::None) {
        QMessageBox::information(this, title,
            QCoreApplication::translate("OpenAutoPath", "当前没有活动刀路，请先生成或显示需要导出的工序。"));
        return;
    }

    const oap::app::ToolpathExportPreparation prepared =
        m_machiningWorkspace->prepareToolpathExport();
    if (!prepared.ok) {
        if (prepared.validation.errorCount > 0) {
            QMessageBox::warning(
                this, title,
                QCoreApplication::translate("OpenAutoPath", "当前刀路程序验证失败:\n") +
                    validationMessage(prepared.validation));
        } else {
            QMessageBox::information(this, title, prepared.error);
        }
        return;
    }
    const oap::toolpath::ToolpathProgram& program = prepared.program;
    const std::vector<oap::toolpath::PocketToolpathOperation>&
        pocketOperations = prepared.pocketOperations;
    const oap::toolpath::ToolpathParams& params = prepared.params;
    const oap::toolpath::ToolpathValidationReport& validation =
        prepared.validation;

    QString suggested = prepared.holeProgram
        ? QStringLiteral("hole_toolpath.nc")
        : QStringLiteral("pocket_toolpath.nc");
    if (!currentPath().isEmpty()) {
        const QFileInfo sourceInfo(currentPath());
        suggested = sourceInfo.absolutePath() + "/" +
                    sourceInfo.completeBaseName() + prepared.suggestedSuffix;
    }

    QString selectedFilter;
    const QString filters = prepared.supportsJsonl
        ? QCoreApplication::translate("OpenAutoPath", "G 代码 (*.nc *.gcode *.tap);;刀路 JSONL (*.jsonl);;所有文件 (*.*)")
        : QCoreApplication::translate("OpenAutoPath", "G 代码 (*.nc *.gcode *.tap);;所有文件 (*.*)");
    QString outPath = QFileDialog::getSaveFileName(
        this, title, suggested, filters, &selectedFilter);
    if (outPath.isEmpty())
        return;

    QFileInfo outInfo(outPath);
    const bool wantsJsonl = prepared.supportsJsonl &&
        (selectedFilter.contains("JSONL", Qt::CaseInsensitive) ||
         outInfo.suffix().compare("jsonl", Qt::CaseInsensitive) == 0);
    if (outInfo.suffix().isEmpty()) {
        outPath += wantsJsonl ? ".jsonl" : ".nc";
        outInfo = QFileInfo(outPath);
    }

    oap::toolpath::GCodeProgramOptions programOptions = m_machiningWorkspace->gcodeOptions();
    if (!currentPath().isEmpty())
        programOptions.programName = QFileInfo(currentPath())
            .completeBaseName().toUtf8().toStdString();

    std::string error;
    QStringList exportedPaths;
    bool ok = false;
    if (wantsJsonl) {
        ok = oap::toolpath::PocketToolpathExport{}.writeJsonl(
            pocketOperations, params,
            outPath.toUtf8().toStdString(), error);
        if (ok)
            exportedPaths << outPath;
    } else {
        // 先把所有单装夹程序在内存中后处理一遍，确保混合刀具、中心出水
        // 代码等错误不会在已经写出部分文件后才被发现；内存结果随后进入
        // 逐行 NC 审查窗口，操作者确认前不创建任何目标文件。
        const oap::app::ReferenceNcExportService exportService;
        const oap::app::ReferenceNcExportPlan exportPlan =
            exportService.prepare(
                program, params, moldSetupWorkFrames(),
                programOptions, outPath);
        ok = exportPlan.ok();
        error = exportPlan.error;
        QVector<oap::app::GCodeReviewDocument> reviewDocuments;
        reviewDocuments.reserve(
            static_cast<qsizetype>(exportPlan.files.size()));
        for (const oap::app::ReferenceNcExportFile& file :
             exportPlan.files) {
            oap::app::GCodeReviewDocument document;
            document.title = QFileInfo(file.outputPath).fileName();
            document.outputPath =
                QFileInfo(file.outputPath).absoluteFilePath();
            document.text = file.ncText;
            reviewDocuments.push_back(std::move(document));
        }
        if (ok && !oap::app::confirmGCodeReview(this, reviewDocuments)) {
            statusBar()->showMessage(
                QCoreApplication::translate("OpenAutoPath", "NC 导出已取消，未写入文件。"));
            return;
        }
        if (ok) {
            ok = exportService.write(exportPlan, params, error);
            if (ok) {
                for (const oap::app::ReferenceNcExportFile& file :
                     exportPlan.files) {
                    exportedPaths << file.outputPath;
                }
            }
        }
    }
    if (!ok) {
        QMessageBox::critical(this, title,
                              QCoreApplication::translate("OpenAutoPath", "导出失败:\n") +
                                  QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(error).toUtf8().constData()));
        return;
    }

    QString message =
        QCoreApplication::translate("OpenAutoPath", "已导出当前%1刀路 %2：%3 个工序，%4 条移动。")
            .arg(prepared.programKind)
            .arg(wantsJsonl ? QStringLiteral("JSONL")
                            : QCoreApplication::translate("OpenAutoPath", "G 代码"))
            .arg(static_cast<int>(program.operations.size()))
            .arg(static_cast<qulonglong>(program.moveCount()));
    if (prepared.setupCount > 0) {
        message += QCoreApplication::translate("OpenAutoPath", " 装夹组 %1。").arg(
            prepared.setupCount);
    }
    if (validation.warningCount > 0)
        message += QCoreApplication::translate("OpenAutoPath", " 验证警告 %1。").arg(validation.warningCount);
    if (!wantsJsonl) {
        message += QCoreApplication::translate("OpenAutoPath", " 当前为参考 NC 文件，未包含现场投产参数，不可直接上机执行。");
    }
    if (!prepared.referenceWarnings.isEmpty()) {
        message += QCoreApplication::translate("OpenAutoPath", " 参考格式已移除非必要生产门禁：%1")
            .arg(prepared.referenceWarnings.join(QStringLiteral("；")));
    }
    statusBar()->showMessage(message);
    if (exportedPaths.size() > 1) {
        message += QCoreApplication::translate("OpenAutoPath", " 已按装夹面、刀具和加工阶段拆分为 %1 个独立文件。")
                       .arg(exportedPaths.size());
    }
    QMessageBox::information(
        this, title,
        message + QStringLiteral("\n") +
            exportedPaths.join(QStringLiteral("\n")));
}

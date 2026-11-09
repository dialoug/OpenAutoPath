// MainWindow 已分析文档的加载、缓存与导出适配器。
#include "MainWindow.h"
#include "app/ui/playback/PlaybackUiController.h"

#include "app/orchestration/DesktopWorkspaceCoordinator.h"
#include "app/ui/shell/MainWindowUiBuilder.h"
#include "app/ui/view/OcctView.h"
#include "app/panels/ResultPanel.h"
#include "app/panels/FeaturePanel.h"
#include "app/panels/GroupPanel.h"
#include "app/ui/panels/FileListPanel.h"
#include "app/ui/dialogs/ShortcutSettingsDialog.h"
#include "app/ui/panels/HoleToolTreePanel.h"
#include "app/ui/panels/HoleCoverageAuditPanel.h"
#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/recognition/RecognitionWorkspaceCoordinator.h"
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"
#include "app/workflows/hole/HoleToolpathWorkbenchCoordinator.h"
#include "app/workflows/hole/HoleJobSourceSynchronizer.h"
#include "app/presentation/OverlayController.h"

#include "recognition/FeatureRecognizer.h"
#include "analysis/AabbBoundaryFace.h"
#include "analysis/MoldSurfaceClassifier.h"
#include "toolpath/hole/HoleMachining.h"
#include "toolpath/hole/HoleMoldEnvelopeEntryResolver.h"
#include "toolpath/pocket/planning/PocketClearing.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/pocket/simulation/PocketZMapComparison.h"

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepPrimAPI_MakeBox.hxx>

#include <set>
#include <tuple>
#include <iterator>
#include <sstream>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <array>
#include <optional>
#include <unordered_set>

#include <QFileDialog>
#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QSettings>
#include <QDir>
#include <QDateTime>
#include <QMenuBar>
#include <QMessageBox>
#include <QToolBar>
#include <QLabel>
#include <QSlider>
#include <QDockWidget>
#include <QListWidget>
#include <QStatusBar>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QElapsedTimer>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QTabWidget>
#include <QScreen>
#include <QDebug>
#include <QMenu>
#include <QAction>
#include <QActionGroup>
#include <QSignalBlocker>
#include <QApplication>
#include <QProgressDialog>
#include <QStringList>
#include <QMetaObject>
#include <QTimer>
#include <QHash>

#include <STEPCAFControl_Reader.hxx>
#include <Interface_Static.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <QCloseEvent>
#include <XCAFPrs_AISObject.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Line.hxx>
#include <AIS_Point.hxx>
#include <Aspect_TypeOfMarker.hxx>
#include <Geom_CartesianPoint.hxx>
#include <Graphic3d_DisplayPriority.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <Quantity_Color.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Prs3d_PointAspect.hxx>
#include <Standard_Failure.hxx>
#include <gp_Trsf.hxx>
#include <gp_Ax1.hxx>
#include <gp.hxx>
#include <gp_Vec.hxx>
#include <TDF_LabelSequence.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <OSD.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <Standard_Failure.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Poly_Triangle.hxx>
#include <V3d.hxx>

void MainWindow::loadFile(const QString &path)
{
    if (path.isEmpty())
        return;
    if (m_view->context().IsNull()) {
        m_pendingViewerLoadPath = path;
        statusBar()->showMessage(
            QStringLiteral("三维视图初始化后将自动加载 STEP 文件。"));
        return;
    }
    m_pendingViewerLoadPath.clear();
    // 加载 controller 保持单飞：当前文件跑完前不接受新加载。
    if (m_uiBusy)
        return;
    if (!m_documentWorkspace ||
        !m_documentWorkspace->beginLoad(path)) {
        statusBar()->showMessage(
            QStringLiteral("无法提交 STEP 后台分析请求。"));
        return;
    }

    // 在文件列表中高亮当前文件(若它属于已打开的文件夹)。
    m_documentLoadTimer.start();
    if (m_fileList)
        m_fileList->setCurrentFile(path);
    updateFileNavigationActions();

    // 进入忙碌态并把计算半部派发到工作线程。读取/缓存/索引/配对/求解都在 worker
    // 线程进行;完成后经 loadFinished 信号回到 onAnalysisFinished 做显示。
    setLoadingState(true);
}

void MainWindow::setLoadingState(bool loading)
{
    // 预览期间旋转相机，不改变仍保留的上一文档模型变换。
    if (loading && !m_loadRotationMode) {
        m_loadRotationMode = m_view->rotationMode();
        m_view->setRotationMode(OcctView::RotationMode::Camera);
    } else if (!loading && m_loadRotationMode) {
        m_view->setRotationMode(*m_loadRotationMode);
        m_loadRotationMode.reset();
    }
    m_uiBusy = loading;
    if (menuBar()) menuBar()->setEnabled(!loading);
    if (m_recognitionToolbar) m_recognitionToolbar->setEnabled(!loading);
    if (m_visibilityToolbar) m_visibilityToolbar->setEnabled(!loading);
    if (m_modeToolbar) m_modeToolbar->setEnabled(!loading);
    if (m_resultPanel) m_resultPanel->setEnabled(!loading);
    if (m_groupPanel) m_groupPanel->setEnabled(!loading);
    if (m_featurePanel) m_featurePanel->setEnabled(!loading);
    m_view->setFaceSelectionEnabled(!loading && !model().faces.empty());
    if (loading) {
        QApplication::setOverrideCursor(Qt::WaitCursor);
        if (m_fileList) m_fileList->setEnabled(false);
        const QString pending = m_documentWorkspace
            ? m_documentWorkspace->pendingPath() : QString{};
        const QString path = pending.isEmpty() ? currentPath() : pending;
        statusBar()->showMessage(QStringLiteral("正在加载 %1 ...").arg(path));
    } else {
        QApplication::restoreOverrideCursor();
        if (m_fileList) m_fileList->setEnabled(true);
    }
    updateFileNavigationActions();
    refreshCamWorkflowPanel();
}

void MainWindow::finishLoadingState(const QString& requestedPath)
{
    m_loadPreview.clear(true);
    if (documentBounds().valid()) {
        const auto center = documentBounds().center();
        m_view->setModelCenter(center[0], center[1], center[2]);
    } else {
        m_view->clearModelCenter();
    }
    if (m_fileList && !requestedPath.isEmpty() &&
        requestedPath != currentPath() && !currentPath().isEmpty())
        m_fileList->setCurrentFile(currentPath());
    setLoadingState(false);
    updateFileNavigationActions();
    if (m_closeWhenIdle) {
        m_closeWhenIdle = false;
        QTimer::singleShot(0, this, &QWidget::close);
    }
}

void MainWindow::onAnalysisPreview(std::shared_ptr<AnalysisLoadPreview> preview)
{
    if (!preview || !m_documentWorkspace || !m_documentWorkspace->acceptsPreview(*preview)) return;
    try {
        m_loadPreview.display(m_view->context(), m_view->view(), preview->shape, preview->meshDeflection);
        if (preview->bounds.valid()) {
            const auto center = preview->bounds.center();
            m_view->setModelCenter(center[0], center[1], center[2]);
        }
        statusBar()->showMessage(QStringLiteral("模型预览已显示，正在分析精确几何；完成后可加工。"));
        qDebug() << "[OpenAutoPath] preview_visible_ms=" << m_documentLoadTimer.elapsed()
                 << " worker_preview_ready_ms=" << preview->elapsedMillis;
    } catch (...) {
        m_loadPreview.clear(true);
        qWarning() << "[OpenAutoPath] preview display failed; exact analysis continues";
    }
}

void MainWindow::onAnalysisFinished(
    std::shared_ptr<AnalysisLoadResult> result)
{
    const QString requestedPath = m_documentWorkspace
        ? m_documentWorkspace->pendingPath() : QString{};
    try {
        OCC_CATCH_SIGNALS
        handleAnalysisFinished(std::move(result));
    } catch (const Standard_Failure& failure) {
        finishLoadingState(requestedPath);
        const char* message = failure.GetMessageString();
        QMessageBox::critical(
            this, QStringLiteral("加载失败"),
            QStringLiteral("显示分析结果时发生 OCCT 异常：\n") +
                QString::fromLocal8Bit(message ? message : "OCCT failure"));
    } catch (const std::exception& failure) {
        finishLoadingState(requestedPath);
        QMessageBox::critical(
            this, QStringLiteral("加载失败"),
            QStringLiteral("显示分析结果时发生异常：\n") +
                QString::fromLocal8Bit(failure.what()));
    } catch (...) {
        finishLoadingState(requestedPath);
        QMessageBox::critical(this, QStringLiteral("加载失败"),
                              QStringLiteral("显示分析结果时发生未知异常。"));
    }
}

void MainWindow::resetForDocumentReplacement()
{
    m_loadPreview.clear(false);
    m_view->resetModelTransform();
    m_view->clearModelCenter();
    sceneController().clear();
    if (m_overlayController)
        m_overlayController->clearAllTransient();
    m_view->context()->RemoveAll(Standard_False);
    m_workspaceCoordinator->resetForModel();
    m_currentMachiningJobPath.clear();
    m_undoHistory.clear();
    refreshHoleCoverageAudit();
    if (m_playbackUi)
        m_playbackUi->stop();
    m_resultPanel->clear();
    if (m_groupPanel)
        m_groupPanel->clear();
    clearOverlay();
    m_lastHolePreviewBuildMs = 0;
    if (m_actFeatureColors) {
        QSignalBlocker block(m_actFeatureColors);
        m_actFeatureColors->setChecked(false);
        m_actFeatureColors->setEnabled(false);
    }
    refreshFeatureLegend();
    if (m_featurePanel)
        m_featurePanel->clear();
    m_sceneInteraction.resetSelection();
    refreshCamWorkflowPanel();
}

void MainWindow::handleAnalysisFinished(
    std::shared_ptr<AnalysisLoadResult> result)
{
    if (!m_documentWorkspace)
        return;
    const oap::app::DocumentLoadCompletion completion =
        m_documentWorkspace->completeLoad(
            result, [this] { resetForDocumentReplacement(); });
    if (completion.status == oap::app::DocumentLoadStatus::Ignored)
        return;
    if (completion.status == oap::app::DocumentLoadStatus::Cancelled) {
        finishLoadingState(completion.requestedPath);
        return;
    }
    if (completion.status == oap::app::DocumentLoadStatus::Failed) {
        if (!completion.message.isEmpty()) {
            QMessageBox::critical(
                this, QStringLiteral("错误"), completion.message);
        }
        finishLoadingState(completion.requestedPath);
        return;
    }
    if (completion.status == oap::app::DocumentLoadStatus::Warning) {
        if (!completion.message.isEmpty()) {
            QMessageBox::warning(
                this, QStringLiteral("分析"), completion.message);
        }
        finishLoadingState(completion.requestedPath);
        return;
    }

    const QString path = completion.sourcePath;
    const oap::Bbox& setupBounds = documentBounds();
    if (setupBounds.valid()) {
        const auto center = setupBounds.center();
        m_view->setModelCenter(center[0], center[1], center[2]);
    }
    setWindowTitle(completion.fromCache
        ? QStringLiteral("OpenAutoPath — ") + path + QStringLiteral("  [缓存]")
        : QStringLiteral("OpenAutoPath — ") + path);

    // 按面建立 AIS 对象(逐面回查 XDE 颜色),之后即可按 face/unit 粒度隐藏。
    Handle(XCAFDoc_ColorTool) colorTool =
        XCAFDoc_DocumentTool::ColorTool(document()->Main());
    QElapsedTimer uitimer; uitimer.start();
    sceneController().displayModel(
        m_view->context(), model(),
        stepIndexer(), colorTool);
    m_view->setMoldCoordinateFrame(moldFrame());
    const qint64 tDisplay = uitimer.restart();
    fitModelInView();
    m_view->view()->Redraw();
    const qint64 tFitRedraw = uitimer.elapsed();
    qDebug().nospace()
        << "[OpenAutoPath] UI display" << (completion.fromCache ? " [cached]" : "")
        << "  ms: displayModel=" << tDisplay
        << " fitall+redraw=" << tFitRedraw
        << " ui_total=" << (tDisplay + tFitRedraw);
    qDebug() << "[OpenAutoPath] exact_model_visible_ms=" << m_documentLoadTimer.elapsed();

    m_resultPanel->setModel(
        &model(), &stepIndexer());
    if (m_groupPanel) m_groupPanel->setModel(&model());
    m_view->setFaceSelectionEnabled(true);
    if (completion.fromCache) {
        statusBar()->showMessage(
            QStringLiteral("已从缓存加载:%1 个单元,%2 个面")
                .arg(model().units.size()).arg(model().faces.size()));
    } else {
        statusBar()->showMessage(
            QStringLiteral("已加载:%1 个单元,%2 个面")
                .arg(model().units.size()).arg(model().faces.size()));
    }

    // 模型进入场景后立即执行统一内外环结构识别并染色；自动流程只使用
    // 状态栏汇报，避免每次切换文件都弹出完成对话框。
    runInnerOuterStructureRecognition(false);
    qDebug() << "[OpenAutoPath] machining_ready_ms=" << m_documentLoadTimer.elapsed();
    finishLoadingState(completion.requestedPath);
}

void MainWindow::openStep()
{
    QString path = QFileDialog::getOpenFileName(
        this, QStringLiteral("打开 STEP 文件"), QString(),
        QStringLiteral("STEP 文件 (*.stp *.step *.STP *.STEP);;所有文件 (*.*)"));
    loadFile(path);
}

void MainWindow::openFolder()
{
    const QString dir = QFileDialog::getExistingDirectory(this, QStringLiteral("打开 STEP 文件所在文件夹"));
    if (dir.isEmpty()) return;

    const int count = m_fileList->setFolder(dir);
    updateFileNavigationActions();
    if (count == 0) {
        QMessageBox::information(this, QStringLiteral("打开文件夹"),
            QStringLiteral("该文件夹中未找到 .stp/.step 文件:\n") + dir);
        return;
    }
    statusBar()->showMessage(QStringLiteral("在 %2 中找到 %1 个 STEP 文件").arg(count).arg(dir));

    // 询问是否批量建立缓存。已缓存文件由 worker 自动跳过。
    if (m_uiBusy) return;
    const QMessageBox::StandardButton ans = QMessageBox::question(
        this, QStringLiteral("打开文件夹"),
        QStringLiteral("在该文件夹找到 %1 个 STEP 文件。\n现在为它们建立缓存吗?已缓存的文件会自动跳过。")
            .arg(count),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if (ans != QMessageBox::Yes) return;

    const QStringList paths = m_fileList->filePaths();
    if (paths.isEmpty()) return;

    // 模态进度对话框:运行期间阻塞主界面,可随时取消(在文件边界生效)。
    m_cacheBuildDialog = new QProgressDialog(
        QStringLiteral("准备建立缓存…"), QStringLiteral("取消"), 0, paths.size(), this);
    m_cacheBuildDialog->setWindowTitle(
        QStringLiteral("建立缓存"));
    m_cacheBuildDialog->setWindowModality(
        Qt::ApplicationModal);
    m_cacheBuildDialog->setAutoClose(false);
    m_cacheBuildDialog->setAutoReset(false);
    m_cacheBuildDialog->setMinimumDuration(0);
    m_cacheBuildDialog->setValue(0);
    connect(
        m_cacheBuildDialog,
        &QProgressDialog::canceled,
        this, [this] {
        if (m_documentWorkspace)
            m_documentWorkspace->requestCancelCacheBuild();
        if (m_cacheBuildDialog) {
            m_cacheBuildDialog->setLabelText(
                QStringLiteral("正在取消…"));
        }
    });

    // 复用加载忙碌态，防止缓存构建期间触发单文件加载或识别。
    if (!m_documentWorkspace ||
        !m_documentWorkspace->beginCacheBuild(paths)) {
        m_cacheBuildDialog->close();
        m_cacheBuildDialog->deleteLater();
        m_cacheBuildDialog = nullptr;
        statusBar()->showMessage(
            QStringLiteral("无法提交 STEP 缓存构建请求。"));
        return;
    }
    setLoadingState(true);
    statusBar()->showMessage(QStringLiteral("正在建立缓存…"));
}

void MainWindow::onCacheBuildProgress(
    int done, int total,
    const QString& currentFile)
{
    if (!m_cacheBuildDialog ||
        m_inCacheBuildProgress) {
        return;
    }
    m_inCacheBuildProgress = true;
    // 注意顺序:setValue 在 ApplicationModal 下会内部 processEvents,可能重入派发已
    // 排队的 cacheBuildFinished，后者会销毁 m_cacheBuildDialog。
    // 全部命中缓存时进度信号瞬间堆满队列,这种重入必然发生。故标签更新放在 setValue 之
    // 前，setValue 返回后不再访问对话框。
    if (!currentFile.isEmpty()) {
        const QString name = QFileInfo(currentFile).fileName();
        m_cacheBuildDialog->setLabelText(
            QStringLiteral("正在处理 (%1/%2): %3")
                .arg(done + 1).arg(total).arg(name));
    }
    m_cacheBuildDialog->setValue(done);
    m_inCacheBuildProgress = false;
}

void MainWindow::onCacheBuildFinished(
    int built, int skipped,
    int failed, bool cancelled)
{
    if (m_cacheBuildDialog) {
        m_cacheBuildDialog->close();
        m_cacheBuildDialog->deleteLater();
        m_cacheBuildDialog = nullptr;
    }
    const bool closing = m_closeWhenIdle;
    finishLoadingState();
    if (closing)
        return;

    QString msg = QStringLiteral("新建 %1 个,跳过 %2 个(已缓存)").arg(built).arg(skipped);
    if (failed > 0) msg += QStringLiteral(",失败 %1 个").arg(failed);
    if (cancelled) msg += QStringLiteral("。\n(已取消,剩余文件未处理)");
    QMessageBox::information(this, QStringLiteral("建立缓存"), msg);
    statusBar()->showMessage(msg);
}

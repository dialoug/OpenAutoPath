// MainWindow 组合根与生命周期装配。
#include "MainWindow.h"

#include "app/orchestration/DesktopWorkspaceCoordinator.h"
#include "app/ui/playback/PlaybackUiController.h"
#include "app/ui/recognition/RecognitionSelectionUiController.h"
#include "app/ui/document/DocumentUiController.h"
#include "app/ui/cam/CamUiController.h"
#include "app/ui/simulation/SimulationUiController.h"
#include "app/ui/shell/MainWindowUiBuilder.h"
#include "app/ui/view/OcctView.h"
#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/recognition/RecognitionWorkspaceCoordinator.h"
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"
#include "app/presentation/OverlayController.h"
#include "app/dialogs/QtWorkflowInteraction.h"

#include <algorithm>
#include <cstddef>
#include <memory>

#include <QCloseEvent>
#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QMessageBox>
#include <QSettings>
#include <QScreen>
#include <QStatusBar>
#include <QStringList>
#include <QTimer>

namespace {
constexpr int kWindowWidth = 1360;
constexpr int kWindowHeight = 860;
constexpr int kDefaultViewLayoutStateVersion = 1;

QString applicationRootPath()
{
    const QString configuredRoot = qEnvironmentVariable("OAP_APP_ROOT").trimmed();
    if (!configuredRoot.isEmpty())
        return QDir(configuredRoot).absolutePath();

    QDir directory(QCoreApplication::applicationDirPath());
    if (directory.dirName().compare(QStringLiteral("bin"),
                                    Qt::CaseInsensitive) == 0 &&
        directory.cdUp())
        return directory.absolutePath();
    return QCoreApplication::applicationDirPath();
}
} // namespace

oap::SceneController& MainWindow::sceneController() noexcept
{
    return m_sceneInteraction.scene();
}

const oap::SceneController& MainWindow::sceneController() const noexcept
{
    return m_sceneInteraction.scene();
}

// 构造函数只完成两件事:创建并绑定各 WorkspaceCoordinator(它们持有各自
// Session 与 worker 线程),再委托 MainWindowUiBuilder 装配界面。其余信号接线
// 在本函数末尾完成,避免构造期间出现半初始化状态。
MainWindow::MainWindow()
{
    m_view = new OcctView(this);
    setCentralWidget(m_view);
    // 所有工作区共用同一个 OverlayController 发布叠加显示(拾取高亮、刀路、
    // 孔段高亮等),确保互不覆盖、可被统一清理。
    m_overlayController =
        std::make_unique<oap::app::presentation::OverlayController>(
            std::make_shared<
                oap::app::presentation::OcctPresentationSink>(
                    m_view->context()));
    m_recognitionWorkspace =
        std::make_unique<oap::app::RecognitionWorkspaceCoordinator>(
            *m_overlayController);
    m_playbackWorkspace =
        std::make_unique<oap::app::PlaybackWorkspaceCoordinator>(
            *m_overlayController, m_view->context(), m_view->view());
    m_stockSimulationWorkspace =
        std::make_unique<oap::app::StockSimulationWorkspaceCoordinator>(
            *m_overlayController, m_view->context(), m_view->view());
    connect(m_view, &OcctView::viewerInitialized,
            this, &MainWindow::bindViewerDependencies);
    if (!m_view->context().IsNull() && !m_view->view().IsNull())
        bindViewerDependencies();
    m_documentWorkspace =
        std::make_unique<oap::app::ModelDocumentWorkspaceCoordinator>(
            applicationRootPath().toUtf8().toStdString());
    m_machiningWorkspace =
        std::make_unique<oap::app::MachiningWorkspaceCoordinator>();
    m_workspaceCoordinator =
        std::make_unique<oap::app::DesktopWorkspaceCoordinator>(
            *m_machiningWorkspace, *m_stockSimulationWorkspace,
            *m_playbackWorkspace, *m_recognitionWorkspace,
            *m_documentWorkspace);
    setWindowTitle(QStringLiteral("OpenAutoPath — STEP / CAM (Qt6 + OpenCASCADE)"));
    // Use logical screen coordinates so the first window fits laptop and
    // high-DPI desktops, including their taskbar and window decorations.
    const QSize available = screen() ? screen()->availableGeometry().size()
                                     : QSize(kWindowWidth, kWindowHeight);
    resize(std::min(1600, available.width() * 94 / 100),
           std::min(1000, available.height() * 90 / 100));

    QSettings displaySettings(
        QStringLiteral("OAP"),
        QStringLiteral("OpenAutoPath"));
    m_stockSimulationWorkspace->setDisplayQuality(std::clamp(
        displaySettings.value(
            QStringLiteral("ipw/display_quality"),
            10).toInt(),
        1, 10));
    m_stockSimulationWorkspace->setTriangulationEnabled(
        displaySettings.value(
            QStringLiteral("ipw/triangulation_enabled"),
            true).toBool());
    m_stockSimulationWorkspace->setDisplayMode(
        static_cast<oap::app::IpwOcctDisplayMode>(
            std::clamp(
                displaySettings.value(
                    QStringLiteral("ipw/occt_display_mode"),
                    0).toInt(),
                0, 8)));
    m_playbackWorkspace->setDisplayLayer(std::clamp(
        displaySettings.value(
            QStringLiteral("toolpath/display_layer"),
            0).toInt(),
        0, 2));
    const int cacheBudgetMiB = std::clamp(
        displaySettings.value(
            QStringLiteral("ipw/display_cache_mib"),
            384).toInt(),
        64, 2048);
    m_stockSimulationWorkspace->setDisplayCacheBudgetBytes(
        static_cast<std::size_t>(cacheBudgetMiB) *
        1024u * 1024u);

    loadToolLibraries();
    const QString parameterSettingsWarning =
        m_machiningWorkspace->loadConfigurationDefaults();
    if (!parameterSettingsWarning.isEmpty()) {
        qWarning().noquote()
            << "[OpenAutoPath] load current toolpath parameters:"
            << parameterSettingsWarning;
    }

    oap::app::MainWindowUiBuilder::build(*this);
    // 等 Qt 完成首次几何分配后保存程序定义的默认 Dock/Toolbar 拓扑。
    QTimer::singleShot(0, this, [this] {
        m_defaultViewLayoutState = saveState(kDefaultViewLayoutStateVersion);
    });
    connect(m_documentWorkspace.get(),
            &oap::app::ModelDocumentWorkspaceCoordinator::analysisFinished,
            this, &MainWindow::onAnalysisFinished);
    connect(m_documentWorkspace.get(),
            &oap::app::ModelDocumentWorkspaceCoordinator::previewReady,
            this, &MainWindow::onAnalysisPreview);
    connect(m_documentWorkspace.get(),
            &oap::app::ModelDocumentWorkspaceCoordinator::loadProgress,
            this, [this](const QString& stage, int done, int total) {
        if (!m_uiBusy) return;
        const QString count = total > 0 ? QStringLiteral(" %1/%2").arg(done).arg(total) : QString{};
        statusBar()->showMessage(stage + count + (m_loadPreview.active()
            ? QStringLiteral("；模型预览可见，分析完成后可加工") : QString{}));
    });
    connect(m_documentWorkspace.get(),
            &oap::app::ModelDocumentWorkspaceCoordinator::cacheBuildProgress,
            this, &MainWindow::onCacheBuildProgress);
    connect(m_documentWorkspace.get(),
            &oap::app::ModelDocumentWorkspaceCoordinator::cacheBuildFinished,
            this, &MainWindow::onCacheBuildFinished);

    connect(
        m_stockSimulationWorkspace.get(),
        &oap::app::StockSimulationWorkspaceCoordinator::statusChanged,
        this, [this](const QString& status) {
            statusBar()->showMessage(status);
        });
    connect(
        m_stockSimulationWorkspace.get(),
        &oap::app::StockSimulationWorkspaceCoordinator::warningRequested,
        this, [this](const QString& title, const QString& message) {
            QMessageBox::warning(this, title, message);
        });
    connect(
        m_stockSimulationWorkspace.get(),
        &oap::app::StockSimulationWorkspaceCoordinator::
            featureAllowancesChanged,
        this, &MainWindow::refreshFeatureAllowanceView);
    connect(
        m_stockSimulationWorkspace.get(),
        &oap::app::StockSimulationWorkspaceCoordinator::
            asynchronousDisplayFinished,
        this, [this] {
            if (m_closeWhenIdle && !m_uiBusy &&
                !m_stockSimulationWorkspace->simulationRunning()) {
                QTimer::singleShot(0, this, &QWidget::close);
            }
        });

    setVisibilityActionsEnabled();
    updateStatusBar();
}

void MainWindow::bindViewerDependencies()
{
    const Handle(AIS_InteractiveContext)& context = m_view->context();
    const Handle(V3d_View)& view = m_view->view();
    if (context.IsNull() || view.IsNull()) {
        qWarning() << "[OpenAutoPath] OCCT viewer initialization emitted"
                      " without a valid context/view";
        return;
    }
    m_overlayController->bindContext(context);
    m_playbackWorkspace->bindViewer(context, view);
    m_stockSimulationWorkspace->bindViewer(context, view);

    if (!m_pendingViewerLoadPath.isEmpty()) {
        const QString pendingPath = m_pendingViewerLoadPath;
        m_pendingViewerLoadPath.clear();
        QTimer::singleShot(0, this, [this, pendingPath] {
            loadFile(pendingPath);
        });
    }
}

MainWindow::~MainWindow()
{
    if (m_playbackUi)
        m_playbackUi->stop();
    if (m_workspaceCoordinator)
        m_workspaceCoordinator->cancelAndWait();
}

oap::app::WorkflowInteraction MainWindow::makeWorkflowInteraction()
{
    return oap::app::makeQtWorkflowInteraction(this,
        [this](bool busy) {
            m_uiBusy = busy;
            refreshCamWorkflowPanel();
            updateFileNavigationActions();
            if (!busy && m_closeWhenIdle)
                QTimer::singleShot(0, this, &QWidget::close);
        },
        [this] { return m_closeWhenIdle; });
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    if (m_uiBusy ||
        (m_workspaceCoordinator &&
         m_workspaceCoordinator->backgroundBusy())) {
        m_closeWhenIdle = true;
        if (m_workspaceCoordinator)
            m_workspaceCoordinator->requestCancel();
        if (m_playbackUi)
            m_playbackUi->stop();
        statusBar()->showMessage(
            QStringLiteral(
                "正在取消后台任务，任务安全结束后将自动关闭…"));
        event->ignore();
        return;
    }
    event->accept();
}

void MainWindow::loadToolLibraries()
{
    for (const QString& warning :
         m_machiningWorkspace->loadToolLibraries()) {
        if (!warning.isEmpty()) {
            qWarning().noquote()
                << "[OpenAutoPath] tool library:" << warning;
        }
    }
}

const oap::tooling::ToolLibrary* MainWindow::activeToolLibrary() const noexcept
{
    return m_machiningWorkspace
        ? m_machiningWorkspace->activeToolLibrary() : nullptr;
}

// MainWindow UI 组装（OpenAutoPathApp）。
// setupUiSlices() 实例化各 UI 切片 controller（识别/文档/CAM/播放/仿真）并
// 提供回调端口；MainWindow 只保留 widget/action 指针与跨工作区的薄粘合。
#include "MainWindow.h"

#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/dialogs/ToolpathParameterDialog.h"
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"
#include "app/ui/cam/CamUiController.h"
#include "app/ui/document/DocumentUiController.h"
#include "app/ui/panels/CamWorkflowPanel.h"
#include "app/ui/panels/HoleToolTreePanel.h"
#include "app/ui/playback/PlaybackUiController.h"
#include "app/ui/recognition/RecognitionSelectionUiController.h"
#include "app/ui/simulation/SimulationUiController.h"

#include <QAction>
#include <QDockWidget>
#include <QMenu>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QStatusBar>

void MainWindow::setupUiSlices()
{
    auto addDockToggle = [this](QDockWidget* dock, const QString& text) {
        if (!m_panelMenu)
            return;
        QAction* action = dock->toggleViewAction();
        action->setText(text);
        m_panelMenu->addAction(action);
    };

    oap::app::RecognitionSelectionUiCallbacks recognitionCallbacks;
    recognitionCallbacks.faceSelected = [this](const QString& id) {
        onTreeFaceSelected(id);
    };
    recognitionCallbacks.featureSelectionChanged = [this] {
        onFeatureSelectionChanged();
    };
    recognitionCallbacks.groupSelectionChanged = [this] {
        onGroupSelectionChanged();
    };
    recognitionCallbacks.contextMenuRequested = [this](const QPoint& pos) {
        showContextMenu(pos);
    };
    m_recognitionSelectionUi =
        std::make_unique<oap::app::RecognitionSelectionUiController>(
            *this, std::move(recognitionCallbacks));
    m_resultPanel = &m_recognitionSelectionUi->resultPanel();
    m_featurePanel = &m_recognitionSelectionUi->featurePanel();
    m_groupPanel = &m_recognitionSelectionUi->groupPanel();
    m_analysisDock = &m_recognitionSelectionUi->analysisDock();
    m_featureDock = &m_recognitionSelectionUi->featureDock();
    m_groupDock = &m_recognitionSelectionUi->groupDock();

    m_documentUi = std::make_unique<oap::app::DocumentUiController>(
        *this, [this](const QString& path) {
            loadFile(path);
            updateFileNavigationActions();
        });
    m_fileList = &m_documentUi->fileList();
    m_fileDock = &m_documentUi->dock();

    oap::app::CamUiCallbacks camCallbacks;
    camCallbacks.openPart = [this] { openStep(); };
    camCallbacks.openProject = [this] { openMachiningJob(); };
    camCallbacks.saveProject = [this] { saveMachiningJob(); };
    camCallbacks.recognizeFeatures = [this] {
        recognizeInnerOuterStructures();
    };
    camCallbacks.configureSetup = [this] { configureToolpathParams(); };
    camCallbacks.selectTools = [this] { selectToolLibrary(); };
    camCallbacks.manageTools = [this] { openToolLibrary(); };
    camCallbacks.generateHole = [this] { generateOrUpdateHolePaths(); };
    camCallbacks.generatePocket = [this] { showPocketClearing(); };
    camCallbacks.generateProfile = [this] { showProfileContour(); };
    camCallbacks.generateFinishing = [this] { generateEngravingToolpaths(); };
    camCallbacks.simulateProject = [this] { simulateProjectStock(); };
    camCallbacks.simulateHole = [this] { simulateHoleStock(); };
    camCallbacks.simulatePocket = [this] { simulatePocketStock(); };
    camCallbacks.simulateProfile = [this] { simulateProfileStock(); };
    camCallbacks.simulateEngraving = [this] { simulateEngravingStock(); };
    camCallbacks.showAllowance = [this] { showFeatureAllowanceView(); };
    camCallbacks.exportProgram = [this] { exportCurrentToolpathPreview(); };
    camCallbacks.toolBatchActivated = [this](int index) {
        if (m_playbackUi)
            m_playbackUi->jumpToToolBatch(index);
    };
    camCallbacks.operationActivated = [this](int startMoveIndex) {
        if (m_playbackUi)
            m_playbackUi->seekToMoveStart(startMoveIndex);
    };
    camCallbacks.operationVisibilityChanged =
        [this](const QString& operationId, bool visible) {
            if (!m_playbackWorkspace)
                return;
            m_playbackWorkspace->setOperationVisible(
                operationId.toStdString(), visible);
            if (m_view && !m_view->view().IsNull())
                m_view->view()->Redraw();
        };
    camCallbacks.operationSoloRequested = [this](const QString& operationId) {
        if (!m_playbackWorkspace)
            return;
        m_playbackWorkspace->setSoloOperation(operationId.toStdString());
        if (m_view && !m_view->view().IsNull())
            m_view->view()->Redraw();
    };
    camCallbacks.operationVisibilitySummaryChanged =
        [this](int visibleCount, int totalCount, bool soloMode) {
            if (m_playbackUi) {
                m_playbackUi->setOperationVisibilitySummary(
                    visibleCount, totalCount, soloMode);
            }
            if (totalCount > 0 && visibleCount == 0) {
                statusBar()->showMessage(
                    QStringLiteral(
                        "全部刀路已由 CAM 工序树隐藏；可通过“显示全部工序”恢复。"),
                    8000);
            }
        };
    camCallbacks.editHoleParameters = [this] { configureToolpathParams(); };
    camCallbacks.editPocketOperation = [this] { configureToolpathParamsForPage(oap::app::ToolpathParameterEditorPage::Pocket); };
    camCallbacks.regeneratePocket = [this] { showPocketClearing(); };
    camCallbacks.editProfileOperation = [this] { configureToolpathParamsForPage(oap::app::ToolpathParameterEditorPage::Profile); };
    camCallbacks.regenerateProfile = [this] { showProfileContour(); };
    camCallbacks.editFinishingOperation = [this](const QString&) { configureEngravingParams(); };
    camCallbacks.regenerateFinishing = [this](const QString&) { generateEngravingToolpaths(); };
    camCallbacks.previewOperation = [this](const QString& formalOperationId) {
        if (m_uiBusy || !m_machiningWorkspace || !m_playbackWorkspace)
            return;
        oap::toolpath::ToolpathProgram program;
        QString error;
        if (!m_machiningWorkspace->activateOperationPreview(
                formalOperationId.toStdString(), program, &error)) {
            QMessageBox::information(this, QStringLiteral("显示此类刀路"),
                error.isEmpty() ? QStringLiteral("此类工序没有可用的刀路，请先重新生成。") : error);
            return;
        }
        // Only replace presentation/playback. clearOverlay() would also discard
        // the stored pocket preview and reset the selected machining family.
        if (m_playbackUi)
            m_playbackUi->stop();
        clearHoleStageHighlightDisplay(true);
        const auto family = m_machiningWorkspace->activePreview();
        const bool hole = family == ActiveToolpathPreview::Hole;
        if (m_actHolePaths) {
            const QSignalBlocker blocker(m_actHolePaths);
            m_actHolePaths->setChecked(hole);
        }
        if (hole) {
            rebuildHolePreviewDisplay();
            // The legacy hole helper synchronizes display entries back into
            // the Job. A preview switch consumes the validated result as-is.
            rebuildPlaybackMovesFromProgram(program, false);
        } else {
            m_playbackWorkspace->replaceProgramPreview(
                program, sceneController().modelTransform());
            rebuildPlaybackMovesFromProgram(
                program, family == ActiveToolpathPreview::Pocket);
        }
        if (m_holeToolTreePanel)
            m_holeToolTreePanel->selectFormalOperation(formalOperationId);
        refreshCamWorkflowPanel();
        if (m_view && !m_view->view().IsNull())
            m_view->view()->Redraw();
        statusBar()->showMessage(QStringLiteral("已显示此类已有刀路，可在工序树定位并播放。"), 6000);
    };
    camCallbacks.formalOperationSelected = [this](const QString& id) {
        synchronizeFormalOperationSelection(id);
    };

    m_camUi = std::make_unique<oap::app::CamUiController>(
        *this, *m_fileDock, std::move(camCallbacks));
    m_camWorkflowPanel = &m_camUi->panel();
    m_camWorkflowDock = &m_camUi->dock();
    m_holeToolTreePanel = &m_camUi->operationTree();
    m_holeCoverageAuditPanel = &m_camUi->coverageAudit();
    m_featureAllowancePanel = &m_camUi->featureAllowance();

    m_simulationUi = std::make_unique<oap::app::SimulationUiController>(
        *m_featureAllowancePanel, *m_stockSimulationWorkspace,
        [this] { return sceneController().modelTransform(); }, this);

    refreshCamWorkflowPanel();

    oap::app::PlaybackUiCallbacks playbackCallbacks;
    playbackCallbacks.modelTransform = [this] {
        return sceneController().modelTransform();
    };
    playbackCallbacks.context = [this] { return m_view->context(); };
    playbackCallbacks.view = [this] { return m_view->view(); };
    playbackCallbacks.rebuildHoleStageHighlight = [this] {
        rebuildHoleStageHighlightDisplay();
    };
    playbackCallbacks.setHoleTreePlaybackPosition = [this](int index) {
        if (m_holeToolTreePanel)
            m_holeToolTreePanel->setPlaybackPosition(index);
    };
    playbackCallbacks.clearHoleTree = [this] {
        if (m_holeToolTreePanel)
            m_holeToolTreePanel->clear();
    };
    m_playbackUi = std::make_unique<oap::app::PlaybackUiController>(
        *this, *m_playbackWorkspace, *m_machiningWorkspace,
        std::move(playbackCallbacks));

    if (!m_panelMenu)
        return;
    m_panelMenu->addSection(QStringLiteral("输入"));
    addDockToggle(m_fileDock, QStringLiteral("文件"));
    addDockToggle(m_camWorkflowDock, QStringLiteral("CAM 工作流"));
    m_panelMenu->addSection(QStringLiteral("模型"));
    addDockToggle(m_analysisDock, QStringLiteral("模型"));
    addDockToggle(m_groupDock, QStringLiteral("分组"));
    m_panelMenu->addSection(QStringLiteral("识别"));
    addDockToggle(m_featureDock, QStringLiteral("特征"));
    m_panelMenu->addSection(QStringLiteral("刀路"));
    addDockToggle(&m_playbackUi->dock(), QStringLiteral("刀路播放"));
}

// MainWindow 播放相关命令适配器（OpenAutoPathApp）。
// 保留跨多个工作区的播放命令适配：清除叠加层/孔段高亮显示等，把 UI 事件
// 翻译为 workspace coordinator 调用。
#include "MainWindow.h"

#include "app/orchestration/DesktopWorkspaceCoordinator.h"
#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/ui/playback/PlaybackUiController.h"
#include "app/ui/panels/HoleToolTreePanel.h"

#include <QAction>
#include <QDebug>
#include <QElapsedTimer>
#include <QSignalBlocker>
#include <QStatusBar>

void MainWindow::clearOverlay(bool resetHolePathAction)
{
    if (resetHolePathAction && m_actHolePaths && m_actHolePaths->isChecked()) {
        const QSignalBlocker blocker(m_actHolePaths);
        m_actHolePaths->setChecked(false);
    }
    clearTransientOverlay();
    clearToolpathOverlay();
    if (m_playbackUi)
        m_playbackUi->clearPanel();
    m_workspaceCoordinator->clearPlaybackProgram();
    if (m_holeToolTreePanel)
        m_holeToolTreePanel->clear();
}

void MainWindow::clearHoleStageHighlightDisplay(bool resetSources)
{
    m_playbackWorkspace->clearHoleStageHighlight();
    if (resetSources)
        m_machiningWorkspace->clearHighlightedHoleStages();
}

void MainWindow::clearTransientOverlay(bool resetHoleStageHighlight)
{
    m_workspaceCoordinator->clearTransientState(resetHoleStageHighlight);
}

void MainWindow::clearToolpathOverlay()
{
    m_workspaceCoordinator->clearToolpathState();
}

bool MainWindow::displayPocketPreviewEntry(const PocketPreviewEntry& entry)
{
    if (!m_view || m_view->context().IsNull())
        return false;
    const oap::app::ToolpathPreviewBuildStats stats =
        m_playbackWorkspace->appendPocketPreview(
            entry,
            sceneController().modelTransform());
    return stats.objectCount > 0;
}

void MainWindow::rebuildPocketPlaybackMoves()
{
    rebuildPlaybackMovesFromProgram(
        m_machiningWorkspace->pocketPreviewProgram(), true);
}

void MainWindow::rebuildPlaybackMovesFromProgram(
    const oap::toolpath::ToolpathProgram& program,
    bool limitPocketCutMoves)
{
    if (m_playbackUi)
        m_playbackUi->stop();
    if (m_holeToolTreePanel)
        m_holeToolTreePanel->resetOperationVisibility();
    m_playbackWorkspace->replaceProgram(
        program, limitPocketCutMoves);
    rebuildHoleToolTree();
    if (m_playbackUi) {
        m_playbackUi->setProgramCounts(
            m_playbackWorkspace->moveCount(),
            m_playbackWorkspace->staticToolpathSegmentCount());
    }
}

void MainWindow::rebuildHolePlaybackMoves()
{
    m_machiningWorkspace->synchronizeHolePreviewBatch();
    const oap::toolpath::HoleToolpathBatch* holeResult =
        m_machiningWorkspace->holeProgram();
    rebuildPlaybackMovesFromProgram(
        holeResult ? holeResult->toProgram()
                   : oap::toolpath::ToolpathProgram{},
        false);
}

void MainWindow::rebuildHolePreviewDisplay()
{
    QElapsedTimer previewTimer;
    previewTimer.start();
    if (!m_view || m_view->context().IsNull()) {
        m_lastHolePreviewBuildMs = previewTimer.elapsed();
        return;
    }
    const oap::app::ToolpathPreviewBuildStats stats =
        m_playbackWorkspace->replaceHolePreview(
            m_machiningWorkspace->holePreviewEntries(),
            sceneController().modelTransform());
    const oap::app::PlaybackUiPosition playbackPosition = m_playbackUi
        ? m_playbackUi->position()
        : oap::app::PlaybackUiPosition{};
    m_playbackWorkspace->applyStaticOverlayVisibility(
        playbackPosition.completedMoveCount,
        playbackPosition.currentMoveFraction);
    m_lastHolePreviewBuildMs = previewTimer.elapsed();
    qDebug().noquote() << "[OpenAutoPath] hole preview batch:"
                       << stats.displayedSegmentCount << "segments,"
                       << m_lastHolePreviewBuildMs << "ms";
}

oap::app::HoleStageHighlightStats
MainWindow::rebuildHoleStageHighlightDisplay()
{
    if (!m_view || m_view->context().IsNull())
        return {};
    const oap::app::PlaybackUiPosition playbackPosition = m_playbackUi
        ? m_playbackUi->position()
        : oap::app::PlaybackUiPosition{};
    return m_playbackWorkspace->replaceHoleStageHighlight(
        m_machiningWorkspace->holePreviewEntries(),
        m_machiningWorkspace->recognizedHoleSources(),
        m_machiningWorkspace->highlightedHoleStageSources(),
        stepIndexer(),
        [this](const std::string& faceId) {
            return sceneController().isHidden(faceId);
        },
        sceneController().modelTransform(),
        m_playbackWorkspace->staticOverlaysVisible(
            playbackPosition.completedMoveCount,
            playbackPosition.currentMoveFraction));
}

void MainWindow::highlightHoleStageToolpathForFace(
    const std::string& faceId)
{
    if (m_machiningWorkspace->activePreview() != ActiveToolpathPreview::Hole)
        return;
    const int sourceIndex = m_playbackWorkspace->holeStageIndexForWallFace(
        faceId, m_machiningWorkspace->recognizedHoleSources());
    if (sourceIndex < 0 || !m_view || m_view->context().IsNull())
        return;
    if (!m_machiningWorkspace->toggleHighlightedHoleStage(sourceIndex)) {
        rebuildHoleStageHighlightDisplay();
        m_view->context()->ClearSelected(Standard_False);
        m_view->view()->Redraw();
        statusBar()->showMessage(
            QStringLiteral("已取消当前孔段高亮；仍保留 %1 个。")
                .arg(m_machiningWorkspace
                    ->highlightedHoleStageSources().size()));
        return;
    }
    if (!m_playbackWorkspace->hasHighlightableHoleStagePath(
            sourceIndex, m_machiningWorkspace->holePreviewEntries())) {
        statusBar()->showMessage(
            m_playbackWorkspace->currentToolOnly()
                ? QStringLiteral("当前孔段不属于当前刀具批次。")
                : QStringLiteral("当前孔段没有可高亮刀路，请先生成孔刀路。"));
        return;
    }
    m_machiningWorkspace->toggleHighlightedHoleStage(sourceIndex);
    const oap::app::HoleStageHighlightStats stats =
        rebuildHoleStageHighlightDisplay();
    if (stats.moveCount == 0) {
        m_machiningWorkspace->removeHighlightedHoleStage(sourceIndex);
        rebuildHoleStageHighlightDisplay();
        statusBar()->showMessage(QStringLiteral("孔段高亮显示构建失败。"));
        return;
    }
    m_view->view()->Redraw();
    statusBar()->showMessage(
        QStringLiteral("已加入孔段高亮：%1 个孔段，%2 个孔壁，%3 个工序。")
            .arg(m_machiningWorkspace
                ->highlightedHoleStageSources().size())
            .arg(stats.wallCount)
            .arg(stats.operationCount));
}

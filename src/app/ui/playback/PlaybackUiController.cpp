// PlaybackUiController 实现（oap_app_playback_ui）。
// 播放 dock 所有权与信号路由，见 PlaybackUiController.h。
#include "PlaybackUiController.h"
#include <QCoreApplication>

#include <algorithm>
#include <utility>

#include <QApplication>
#include <QDockWidget>
#include <QMainWindow>

#include <Quantity_Color.hxx>

#include "app/machining/MachiningWorkspaceCoordinator.h"
#include "app/playback/PlaybackCameraFollower.h"
#include "app/playback/PlaybackPanelViewModel.h"
#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/ui/panels/ToolpathPlayerPanel.h"
#include "app/workflows/playback/ToolpathPlaybackWorker.h"

namespace {
const Quantity_Color kToolMarkerColor(
    1.0, 0.15, 0.15, Quantity_TOC_sRGB);
constexpr double kToolMarkerPointSize = 12.0;

bool isHoleKind(oap::FeatureKind kind)
{
    return kind == oap::FeatureKind::ThroughHole ||
           kind == oap::FeatureKind::BlindHole ||
           kind == oap::FeatureKind::CountersinkHole;
}
}

namespace oap::app {

PlaybackUiController::PlaybackUiController(
    QMainWindow& window,
    PlaybackWorkspaceCoordinator& playback,
    MachiningWorkspaceCoordinator& machining,
    PlaybackUiCallbacks callbacks)
    : m_playback(playback),
      m_machining(machining),
      m_callbacks(std::move(callbacks))
{
    m_panel = new ToolpathPlayerPanel(&window);
    m_dock = new QDockWidget(QCoreApplication::translate("OpenAutoPath", "刀路播放"), &window);
    m_dock->setObjectName(QStringLiteral("toolpathPlayerDock"));
    m_dock->setWidget(m_panel);
    m_dock->setAllowedAreas(
        Qt::TopDockWidgetArea | Qt::BottomDockWidgetArea);
    window.addDockWidget(Qt::BottomDockWidgetArea, m_dock);

    connect(m_panel, &ToolpathPlayerPanel::frameChanged,
            this, [this](int index, double fraction) {
                updatePosition(index, fraction);
            });
    connect(m_panel, &ToolpathPlayerPanel::playbackReset,
            this, [this] { resetPlayback(); });
    connect(m_panel, &ToolpathPlayerPanel::previousToolRequested,
            this, [this] { jumpToAdjacentTool(-1); });
    connect(m_panel, &ToolpathPlayerPanel::nextToolRequested,
            this, [this] { jumpToAdjacentTool(1); });
    connect(m_panel, &ToolpathPlayerPanel::currentToolPathOnlyChanged,
            this, [this](bool enabled) {
                setCurrentToolPathOnly(enabled);
            });
    connect(m_panel, &ToolpathPlayerPanel::staticToolpathDisplayChanged,
            this, [this](int firstIndex, int count, bool showNodes) {
                m_playback.setStaticToolpathDisplay(
                    firstIndex, count, showNodes);
                const Handle(V3d_View) activeView = view();
                if (!activeView.IsNull())
                    activeView->Redraw();
            });
    connect(&m_playback, &PlaybackWorkspaceCoordinator::frameReady,
            this, [this](const ::ToolpathPlaybackFrame& frame) {
                applyFrame(frame);
            }, Qt::QueuedConnection);
}

PlaybackUiController::~PlaybackUiController() = default;

QDockWidget& PlaybackUiController::dock() noexcept
{
    return *m_dock;
}

void PlaybackUiController::stop()
{
    m_panel->stop();
}

void PlaybackUiController::clearPanel()
{
    m_panel->stop();
    m_panel->setMoveCount(0);
    m_panel->setStaticToolpathSegmentCount(0);
    m_panel->setToolNavigationState(false, false);
}

void PlaybackUiController::setProgramCounts(
    int moveCount, int staticSegmentCount)
{
    m_panel->setMoveCount(moveCount);
    m_panel->setStaticToolpathSegmentCount(staticSegmentCount);
}

PlaybackUiPosition PlaybackUiController::position() const noexcept
{
    return {m_panel->completedMoveCount(),
            m_panel->currentMoveFraction()};
}

void PlaybackUiController::seekToMoveStart(int moveIndex)
{
    m_panel->seekToMoveStart(moveIndex);
}

void PlaybackUiController::jumpToToolBatch(int batchIndex)
{
    if (batchIndex < 0 ||
        batchIndex >= static_cast<int>(m_playback.toolBatchStarts().size())) {
        return;
    }
    m_panel->seekToMoveStart(m_playback.toolBatchStarts()[batchIndex]);
}

void PlaybackUiController::setOperationVisibilitySummary(
    int visibleCount, int totalCount, bool soloMode)
{
    m_panel->setOperationVisibilitySummary(
        visibleCount, totalCount, soloMode);
}

void PlaybackUiController::applyDisplayLayer()
{
    m_playback.applyDisplayLayer();
    const Handle(V3d_View) activeView = view();
    if (!activeView.IsNull())
        activeView->Redraw();
}

void PlaybackUiController::resetPlayback()
{
    m_playback.resetPlayback();
    if (m_callbacks.rebuildHoleStageHighlight)
        m_callbacks.rebuildHoleStageHighlight();
    if (m_callbacks.clearHoleTree)
        m_callbacks.clearHoleTree();
    m_panel->setToolNavigationState(false, false);
    const Handle(V3d_View) activeView = view();
    if (!context().IsNull() && !activeView.IsNull())
        activeView->Redraw();
}

void PlaybackUiController::updatePosition(int index, double fraction)
{
    if (m_callbacks.setHoleTreePlaybackPosition)
        m_callbacks.setHoleTreePlaybackPosition(index);

    PlaybackPanelViewModelInput input;
    input.moves = &m_playback.moves();
    input.toolBatchStarts = &m_playback.toolBatchStarts();
    input.index = index;
    input.fraction = fraction;
    const PlaybackPanelViewModel panelModel =
        PlaybackPanelViewModelBuilder::build(input);
    m_panel->setToolNavigationState(
        panelModel.previousToolEnabled,
        panelModel.nextToolEnabled);
    m_panel->setMachiningContext(
        panelModel.phaseText, panelModel.toolText);
    m_panel->setInfoText(panelModel.infoText);
    updateCurrentToolFilter(panelModel.navigationMoveIndex);

    const bool modalOrPopup =
        QApplication::activeModalWidget() != nullptr ||
        QApplication::activePopupWidget() != nullptr;
    const bool mouseDragging =
        QApplication::mouseButtons() != Qt::NoButton;
    const bool requestCursorFrame = m_playback.shouldRenderFrame(
        index, m_panel->isPlaying(), modalOrPopup, mouseDragging);
    m_playback.publishPosition(
        index, fraction, modelTransform(), requestCursorFrame);
}

void PlaybackUiController::jumpToAdjacentTool(int direction)
{
    if (direction == 0 || m_playback.moves().empty() ||
        m_playback.toolBatchStarts().empty()) {
        return;
    }
    const int total = m_playback.moveCount();
    const int moveIndex = std::clamp(
        m_panel->completedMoveCount(), 0, total - 1);
    const int currentBatch = m_playback.moves()[moveIndex].toolBatchIndex;
    const int targetBatch = currentBatch + (direction < 0 ? -1 : 1);
    jumpToToolBatch(targetBatch);
}

void PlaybackUiController::setCurrentToolPathOnly(bool enabled)
{
    m_playback.setCurrentToolOnly(enabled);
    const int total = m_playback.moveCount();
    const int index = m_panel->completedMoveCount();
    const int moveIndex = total > 0 ? std::clamp(index, 0, total - 1) : -1;
    updateCurrentToolFilter(moveIndex, true);
    if (total > 0) {
        updatePosition(
            m_panel->completedMoveCount(),
            m_panel->currentMoveFraction());
    } else {
        const Handle(V3d_View) activeView = view();
        if (!context().IsNull() && !activeView.IsNull())
            activeView->Redraw();
    }
}

void PlaybackUiController::updateCurrentToolFilter(int moveIndex, bool force)
{
    const PlaybackToolFilterChange change =
        m_playback.updateCurrentToolFilterPresentation(
            moveIndex, force,
            m_machining.activePreview() == ActiveToolpathPreview::Hole,
            modelTransform());
    if (change.kind != PlaybackToolFilterChangeKind::None &&
        m_callbacks.rebuildHoleStageHighlight) {
        m_callbacks.rebuildHoleStageHighlight();
    }
}

void PlaybackUiController::applyFrame(const ::ToolpathPlaybackFrame& frame)
{
    if (context().IsNull())
        return;
    const int total = m_playback.moveCount();
    const int committed = std::clamp(frame.toIndex, 0, total);
    const bool hasCurrent = frame.fraction < 1.0 && committed < total;
    if (frame.kind == ToolpathPlaybackFrameKind::TrailChunk) {
        m_playback.scheduleRedraw();
        return;
    }
    if (frame.hasTip) {
        const PlaybackMove* current = nullptr;
        if (hasCurrent)
            current = &m_playback.moves()[committed];
        else if (committed > 0)
            current = &m_playback.moves()[committed - 1];

        double toolDiameter =
            m_machining.activePreview() == ActiveToolpathPreview::Hole
                ? m_machining.holeParams().tool.diameter
                : m_machining.activePreview() == ActiveToolpathPreview::Profile
                    ? m_machining.profileParams().tool.diameter
                    : m_machining.activePreview() == ActiveToolpathPreview::SurfaceFinishing
                        ? m_machining.finishingPreviewParams().tool.diameter
                    : m_machining.pocketParams().tool.diameter;
        if (current) {
            if (current->tool.valid && current->tool.diameter > 0.0) {
                toolDiameter = current->tool.diameter;
            } else if (isHoleKind(current->kind)) {
                toolDiameter =
                    m_machining.holePreviewParams().tool.diameter;
            } else if (
                m_machining.activePreview() == ActiveToolpathPreview::SurfaceFinishing) {
                toolDiameter = m_machining.finishingPreviewParams().tool.diameter;
            } else if (
                m_machining.activePreview() == ActiveToolpathPreview::Profile) {
                toolDiameter = m_machining
                    .profilePreviewEffectiveParams().tool.diameter;
            } else {
                toolDiameter = m_machining
                    .pocketPreviewEffectiveParams().tool.diameter;
            }
        }
        m_playback.updateTipMarker(
            frame.tip, toolDiameter, modelTransform(),
            kToolMarkerColor, kToolMarkerPointSize);
        PlaybackCameraFollower::follow(
            view(), frame.tip,
            m_playback.shouldFollowCamera(
                m_panel->isPlaying(), m_panel->speedMultiplier()));
    } else {
        m_playback.clearCursor();
    }
    m_playback.scheduleRedraw();
}

Handle(AIS_InteractiveContext) PlaybackUiController::context() const
{
    return m_callbacks.context
        ? m_callbacks.context() : Handle(AIS_InteractiveContext){};
}

Handle(V3d_View) PlaybackUiController::view() const
{
    return m_callbacks.view ? m_callbacks.view() : Handle(V3d_View){};
}

gp_Trsf PlaybackUiController::modelTransform() const
{
    return m_callbacks.modelTransform
        ? m_callbacks.modelTransform() : gp_Trsf{};
}

} // namespace oap::app

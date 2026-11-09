// 播放 dock 所有权、信号路由与帧呈现（oap_app_playback_ui）。
// 拥有刀路播放面板，把播放位置/刀具过滤等命令经 PlaybackUiCallbacks 端口
// 转交 PlaybackWorkspaceCoordinator，并把帧与位置投影回面板显示。
#pragma once

#include <functional>
#include <memory>

#include <QObject>

#include <AIS_InteractiveContext.hxx>
#include <V3d_View.hxx>
#include <gp_Trsf.hxx>

class QDockWidget;
class QMainWindow;
class ToolpathPlayerPanel;
struct ToolpathPlaybackFrame;

namespace oap::app {
class MachiningWorkspaceCoordinator;
class PlaybackWorkspaceCoordinator;

// 播放 UI 回调端口：把播放命令转交应用层。
struct PlaybackUiCallbacks {
    std::function<gp_Trsf()> modelTransform;
    std::function<Handle(AIS_InteractiveContext)()> context;
    std::function<Handle(V3d_View)()> view;
    std::function<void()> rebuildHoleStageHighlight;
    std::function<void(int)> setHoleTreePlaybackPosition;
    std::function<void()> clearHoleTree;
};

// 播放 UI 位置：播放进度到面板值的投影。
struct PlaybackUiPosition {
    int completedMoveCount = 0;
    double currentMoveFraction = 1.0;
};

// Owns the playback dock, player widget and all playback-specific UI routing.
class PlaybackUiController final : public QObject {
public:
    PlaybackUiController(
        QMainWindow& window,
        PlaybackWorkspaceCoordinator& playback,
        MachiningWorkspaceCoordinator& machining,
        PlaybackUiCallbacks callbacks);
    ~PlaybackUiController() override;

    PlaybackUiController(const PlaybackUiController&) = delete;
    PlaybackUiController& operator=(const PlaybackUiController&) = delete;

    QDockWidget& dock() noexcept;
    void stop();
    void clearPanel();
    void setProgramCounts(int moveCount, int staticSegmentCount);
    PlaybackUiPosition position() const noexcept;
    void seekToMoveStart(int moveIndex);
    void jumpToToolBatch(int batchIndex);
    void setOperationVisibilitySummary(
        int visibleCount, int totalCount, bool soloMode);
    void applyDisplayLayer();

private:
    void resetPlayback();
    void updatePosition(int index, double fraction);
    void jumpToAdjacentTool(int direction);
    void setCurrentToolPathOnly(bool enabled);
    void updateCurrentToolFilter(int moveIndex, bool force = false);
    void applyFrame(const ::ToolpathPlaybackFrame& frame);
    Handle(AIS_InteractiveContext) context() const;
    Handle(V3d_View) view() const;
    gp_Trsf modelTransform() const;

    PlaybackWorkspaceCoordinator& m_playback;
    MachiningWorkspaceCoordinator& m_machining;
    PlaybackUiCallbacks m_callbacks;
    ToolpathPlayerPanel* m_panel = nullptr;
    QDockWidget* m_dock = nullptr;
};

} // namespace oap::app

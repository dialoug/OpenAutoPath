// 应用级播放边界:拥有会话、worker 与 OCCT 播放呈现。
// 职责:管理播放会话(移动/刀具批次/显示层/播放请求状态)、后台播放
//   worker 的帧/轨迹请求,以及 OCCT 播放呈现(静态路径/剩余路径/光标)。
// 模块归属:src/app(playback 聚焦工作区协调器),由 MainWindow 持有。
// 协作:ToolpathPlaybackController 在后台派生播放帧;PlaybackPresenter
//   负责播放叠加层;ToolpathPreviewPresenter/HoleStageHighlightPresenter
//   负责完整刀路预览与孔阶段高亮;程序装配由 PlaybackProgramAssembler 完成。
#pragma once

#include <functional>
#include <memory>
#include <span>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <QObject>

#include <AIS_InteractiveContext.hxx>
#include <Quantity_Color.hxx>
#include <V3d_View.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include "app/presentation/PlaybackOverlayPolicy.h"
#include "app/workflows/playback/PlaybackSession.h"
#include "app/workflows/playback/ToolpathPlaybackWorker.h"

namespace oap::toolpath {
struct ToolpathProgram;
}

namespace oap {
class StepIndexer;
}

class ToolpathPlaybackController;

namespace oap::app {

struct PlaybackProgramBuildOptions;
struct HolePreviewEntry;
struct PocketPreviewEntry;
struct RecognizedHoleSource;

namespace presentation {
class OverlayController;
class HoleStageHighlightPresenter;
class PlaybackPresenter;
class ToolpathPreviewPresenter;
}

// 当前刀具过滤器变化类型:无变化/已清除/需要重建批次预览。
enum class PlaybackToolFilterChangeKind {
    None,
    Cleared,
    RebuildBatch,
};

// 过滤器变化:类型与涉及的批次号。
struct PlaybackToolFilterChange {
    PlaybackToolFilterChangeKind kind =
        PlaybackToolFilterChangeKind::None;
    int batchIndex = -1;
};

// 播放位置上的叠加层可见性状态:静态叠加层(完整路径/当前刀具/孔阶段
// 高亮)与剩余路径是否可见。
struct PlaybackOverlayVisibilityState {
    presentation::PlaybackStaticOverlayVisibility staticOverlays;
    bool remainingPath = false;

    bool hasVisibleToolpath() const noexcept
    {
        return staticOverlays.fullToolpath ||
            staticOverlays.currentToolpath ||
            staticOverlays.holeStageHighlight || remainingPath;
    }
};

// 刀路预览构建统计:源/显示段数、失败段数与呈现对象数。
struct ToolpathPreviewBuildStats {
    std::size_t sourceSegmentCount = 0;
    std::size_t displayedSegmentCount = 0;
    int failedSegmentCount = 0;
    int objectCount = 0;
};

// 孔阶段高亮统计:墙/工序/移动计数。
struct HoleStageHighlightStats {
    int wallCount = 0;
    int operationCount = 0;
    int moveCount = 0;
};

// 播放工作区协调器:聚合播放会话、后台播放控制器与各呈现器。
// 状态所有权:PlaybackSession 保存移动/刀具批次/显示层与请求状态;
//   ToolpathPlaybackController 拥有后台 worker;呈现器负责播放叠加层、
//   刀路预览与孔阶段高亮。帧请求以分页方式异步推进(见 requestTrailFlush)。
class PlaybackWorkspaceCoordinator final : public QObject {
    Q_OBJECT
public:
    PlaybackWorkspaceCoordinator(
        presentation::OverlayController& overlays,
        const Handle(AIS_InteractiveContext)& context,
        const Handle(V3d_View)& view,
        QObject* parent = nullptr);
    ~PlaybackWorkspaceCoordinator() override;

    PlaybackWorkspaceCoordinator(
        const PlaybackWorkspaceCoordinator&) = delete;
    PlaybackWorkspaceCoordinator& operator=(
        const PlaybackWorkspaceCoordinator&) = delete;

    // 绑定/切换视图(呈现器同步更新)。
    void bindViewer(
        const Handle(AIS_InteractiveContext)& context,
        const Handle(V3d_View)& view);
    // 播放数据只读查询:移动列表、刀具批次起点、总步数与某批次的步范围。
    const std::vector<PlaybackMove>& moves() const noexcept
    {
        return m_session.moves();
    }
    const std::vector<int>& toolBatchStarts() const noexcept
    {
        return m_session.toolBatchStarts();
    }
    int moveCount() const noexcept
    {
        return static_cast<int>(m_session.moves().size());
    }
    std::pair<int, int> toolBatchRange(int toolBatchIndex) const;

    // 用程序替换播放内容:装配会话、清空旧呈现,并把移动集推给后台
    // 控制器；重载版本构造构建选项（含通用口袋预算与各段颜色）。
    bool replaceProgram(
        const toolpath::ToolpathProgram& program,
        const PlaybackProgramBuildOptions& options);
    bool replaceProgram(
        const toolpath::ToolpathProgram& program,
        bool limitPocketCutMoves);
    // 清除程序/呈现/各类预览与高亮,以及重置播放位置。
    void clearProgram();
    void clearPresentation();
    void clearFullToolpathPreview();
    void clearCurrentToolpathPreview();
    void clearHoleStageHighlight();
    void clearStaticToolpathPresentation();
    void resetPlayback();

    // 构建完整刀路预览:槽预览逐条追加;孔预览整体替换(采样削减 + 颜色
    // 重叠解析);当前刀具预览仅重建当前批次。
    ToolpathPreviewBuildStats appendPocketPreview(
        const PocketPreviewEntry& entry,
        const gp_Trsf& modelTransform);
    ToolpathPreviewBuildStats replaceProgramPreview(
        const toolpath::ToolpathProgram& program,
        const gp_Trsf& modelTransform);
    ToolpathPreviewBuildStats replaceHolePreview(
        std::span<const HolePreviewEntry> entries,
        const gp_Trsf& modelTransform);
    ToolpathPreviewBuildStats replaceCurrentToolPreview(
        bool holePreview,
        const gp_Trsf& modelTransform);
    int staticToolpathSegmentCount() const;
    void setStaticToolpathDisplay(
        int firstIndex, int count, bool showNodes);
    void setOperationVisible(
        const std::string& operationId, bool visible);
    void setSoloOperation(const std::string& operationId);
    bool operationVisible(const std::string& operationId) const noexcept;
    bool remainingSegmentVisible(int moveIndex) const;
    const std::string& soloOperationId() const noexcept
    {
        return m_soloOperationId;
    }

    // 孔阶段高亮:面→孔源映射(歧义返回 -1)、是否有可高亮路径,
    // 以及按选中孔源重建高亮(墙 + 切削/进刀路径)。
    int holeStageIndexForWallFace(
        const std::string& faceId,
        std::span<const RecognizedHoleSource> sources) const;
    bool hasHighlightableHoleStagePath(
        int sourceIndex,
        std::span<const HolePreviewEntry> entries) const;
    HoleStageHighlightStats replaceHoleStageHighlight(
        std::span<const HolePreviewEntry> entries,
        std::span<const RecognizedHoleSource> sources,
        const std::unordered_set<int>& selectedSources,
        const oap::StepIndexer& indexer,
        const std::function<bool(const std::string&)>& isFaceHidden,
        const gp_Trsf& modelTransform,
        bool visible);

    // 后台任务控制:请求取消/同步等待/运行状态。
    void requestCancel() noexcept;
    void cancelAndWait();
    bool running() const noexcept;

    // 显示层:会话内 0..2,换算为 OCCT Z 图层后应用;模型变换应用于
    // 播放呈现。
    int displayLayer() const noexcept { return m_session.displayLayer(); }
    void setDisplayLayer(int layer) noexcept;
    int displayZLayer() const noexcept;
    void applyDisplayLayer();
    void applyModelTransform(const gp_Trsf& transform);

    // 当前刀具过滤:是否只播当前刀具、当前批次号、重置批次,以及按
    // 移动更新过滤器(变化类型含清除/重建批次)。
    bool currentToolOnly() const noexcept
    {
        return m_session.currentToolOnly();
    }
    void setCurrentToolOnly(bool enabled) noexcept
    {
        m_session.currentToolOnly() = enabled;
    }
    int currentToolBatchIndex() const noexcept
    {
        return m_session.currentToolBatchIndex();
    }
    void resetCurrentToolBatch() noexcept
    {
        m_session.currentToolBatchIndex() = -1;
    }
    PlaybackToolFilterChange updateCurrentToolFilter(
        int moveIndex, bool force = false);
    PlaybackToolFilterChange updateCurrentToolFilterPresentation(
        int moveIndex,
        bool force,
        bool holePreview,
        const gp_Trsf& modelTransform);

    // 播放位置驱动:计算/应用叠加层可见性,发布位置(更新剩余路径,
    // 可选请求光标帧)。
    PlaybackOverlayVisibilityState visibilityForPosition(
        int index, double fraction) const noexcept;
    bool staticOverlaysVisible(
        int index, double fraction) const noexcept;
    void applyStaticOverlayVisibility(int index, double fraction);
    void publishPosition(
        int index, double fraction,
        const gp_Trsf& modelTransform,
        bool requestCursorFrame);

    // 播放节流与相机:是否渲染本帧/跟随相机,以及合并重绘请求。
    bool shouldRenderFrame(
        int index, bool panelPlaying,
        bool modalOrPopup, bool mouseDragging);
    bool shouldFollowCamera(
        bool panelPlaying, double speedMultiplier);
    void scheduleRedraw();

    // 刀具尖端标记与播放光标清除。
    void updateTipMarker(
        const gp_Pnt& tip,
        double toolDiameter,
        const gp_Trsf& modelTransform,
        const Quantity_Color& color,
        double pointSize);
    void clearCursor();

signals:
    // 后台播放帧就绪(光标帧与轨迹帧)。
    void frameReady(const ToolpathPlaybackFrame& frame);

private:
    // 更新剩余路径叠加层(懒构建段列表,按当前/批次范围裁剪)。
    void updateRemainingPath(
        int index, double fraction,
        const gp_Trsf& modelTransform,
        bool visible);
    // 重置轨迹播放对象与请求状态(从指定步开始)。
    void resetTrailRange(int startIndex);
    void refreshRemainingPathOperationFilter();
    // 请求单帧光标,必要时先倒带并作废旧请求。
    void requestFrame(int index, double fraction);
    // 分页推进轨迹帧请求(带合并/挂起语义)。
    void requestTrailFlush(int toIndex, bool force);
    // 把光标/轨迹请求派发给后台控制器。
    void dispatchCursorRequest(int index, double fraction);
    void dispatchTrailRequest(int fromIndex, int toIndex);
    // 消费后台帧:光标帧转发光标并继续下一请求;轨迹帧推进刷指针并
    // 处理挂起的下一页。
    void handleWorkerFrame(const ToolpathPlaybackFrame& frame);

    PlaybackSession m_session;      // 播放会话:移动/批次/请求状态。
    presentation::OverlayController* m_overlays = nullptr;  // 叠加层控制器。
    std::unique_ptr<ToolpathPlaybackController> m_controller;   // 后台播放控制器。
    std::unique_ptr<presentation::PlaybackPresenter> m_presenter;   // 播放叠加层呈现。
    std::unique_ptr<presentation::ToolpathPreviewPresenter>
        m_previewPresenter;        // 完整/当前刀路预览呈现。
    std::unique_ptr<presentation::HoleStageHighlightPresenter>
        m_holeStageHighlightPresenter;   // 孔阶段高亮呈现。
    Handle(V3d_View) m_view;       // 目标视图。
    int m_staticToolpathFirstIndex = 0;
    int m_staticToolpathCount = -1;
    bool m_staticToolpathShowNodes = false;
    std::unordered_set<std::string> m_hiddenOperationIds;
    std::string m_soloOperationId;
};

} // namespace oap::app

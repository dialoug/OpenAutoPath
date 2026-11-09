// 模块归属：app/workflows/playback（oap_app_workflow_playback）刀路播放工作流。
// 职责：保存单个刀路程序的播放状态：展平移动序列、换刀批次、
//       光标/轨迹请求合并与节流、相机跟随与显示层状态。
// 线程：仅由 UI 线程使用；后台 worker 只持有只读快照（ToolpathPlaybackFrame）。
// 陈旧结果门禁：generation/requestId 由 controller 维护，本会话只记录
//       飞行中的请求与待派发位置。
// 每个程序的刀路播放状态。不包含 QWidget 或 AIS 所有权。
#pragma once

#include <utility>
#include <vector>
#include <optional>
#include <string>

#include <QElapsedTimer>

#include <Quantity_Color.hxx>

#include "recognition/FeatureRecognizer.h"
#include "toolpath/core/ToolpathMoves.h"
#include "toolpath/core/ToolpathPrimitiveMapping.h"
#include "toolpath/core/ToolpathProgram.h"
#include "toolpath/hole/HoleMachining.h"

namespace oap::app {

// 一条展平后的播放移动：携带刀路移动数据、显示配色、特征/装夹面/
// 工序阶段上下文与换刀批次索引。
struct PlaybackMove {
    toolpath::ToolpathMove move;
    toolpath::ToolpathMoveSemantic semantic;
    Quantity_Color color;
    std::string operationId;
    int operationMoveIndex = -1;
    toolpath::ToolpathOperationType operationType =
        toolpath::ToolpathOperationType::Generic;
    FeatureKind kind = FeatureKind::Pocket;
    toolpath::HoleMachiningStrategy holeStrategy =
        toolpath::HoleMachiningStrategy::SmallDrill;
    int sourceIndex = -1;
    int setupIndex = -1;
    int setupCount = 0;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    toolpath::HoleMachiningPhase phase =
        toolpath::HoleMachiningPhase::Standard;
    toolpath::ToolpathToolContext tool;
    int toolBatchIndex = -1;
};

// 播放会话：扁平化 move、批处理、过滤与节流。
class PlaybackSession {
public:
    // 光标请求：index 为目标移动下标，fraction 为段内进度（0..1）。
    // dispatch 存在表示本次立即派发请求；rewound 表示发生回退，
    // 需要让工作区失效对应的工作线程请求。
    struct CursorRequest {
        int index = 0;
        double fraction = 1.0;
    };
    // 光标决策：播放位置到刀具/阶段/步骤的选择。
    struct CursorDecision {
        std::optional<CursorRequest> dispatch;
        bool rewound = false;
    };

    // 清空整个程序并复位请求状态。
    void clearProgram();
    // 复位全部请求与节流状态；trailStartIndex 为轨迹起点（默认 0）。
    void resetRequestState(int trailStartIndex = 0);
    // 返回指定换刀批次在 moves 中的半开区间 [start, end)。
    std::pair<int, int> toolBatchRange(int toolBatchIndex) const;
    // 合并光标请求：若已有帧请求在飞行则暂存为待派发；回退时
    // 立即清空飞行状态并派发所请求位置。
    CursorDecision requestCursor(int index, double fraction);
    // 完成一次光标请求：若期间累积了新请求则立即派发并置为飞行中。
    std::optional<CursorRequest> completeCursorRequest();
    // 派发失败：清除飞行状态，等待下一次请求。
    void cursorDispatchFailed() noexcept;
    // 渲染节流：模态/弹窗或拖拽时限制帧率，回退或首尾强制渲染。
    bool shouldRenderFrame(
        int index, bool panelPlaying,
        bool modalOrPopup, bool mouseDragging);
    // 相机跟随节流：高倍速播放时降低跟随频率。
    bool shouldFollowCamera(bool panelPlaying, double speedMultiplier);
    // 重绘节流：合并高频重绘请求；返回距最小间隔的延迟毫秒数
    // （-1 表示已有重绘在等待）。
    int beginRedrawRequest(qint64 minimumIntervalMs);
    void completeRedrawRequest();

    std::vector<PlaybackMove>& moves() noexcept { return m_moves; }
    const std::vector<PlaybackMove>& moves() const noexcept { return m_moves; }
    std::vector<int>& toolBatchStarts() noexcept { return m_toolBatchStarts; }
    const std::vector<int>& toolBatchStarts() const noexcept
    {
        return m_toolBatchStarts;
    }

    bool& frameInFlight() noexcept { return m_frameInFlight; }
    bool& pendingFrame() noexcept { return m_pendingFrame; }
    int& pendingIndex() noexcept { return m_pendingIndex; }
    double& pendingFraction() noexcept { return m_pendingFraction; }
    bool& trailInFlight() noexcept { return m_trailInFlight; }
    bool& trailPending() noexcept { return m_trailPending; }
    bool& trailPendingForce() noexcept { return m_trailPendingForce; }
    int& trailPendingTo() noexcept { return m_trailPendingTo; }
    int& trailFlushIndex() noexcept { return m_trailFlushIndex; }

    QElapsedTimer& renderThrottleTimer() noexcept
    {
        return m_renderThrottleTimer;
    }
    QElapsedTimer& redrawThrottleTimer() noexcept
    {
        return m_redrawThrottleTimer;
    }
    QElapsedTimer& cameraFollowTimer() noexcept
    {
        return m_cameraFollowTimer;
    }
    bool& redrawPending() noexcept { return m_redrawPending; }

    int& displayLayer() noexcept { return m_displayLayer; }
    int displayLayer() const noexcept { return m_displayLayer; }
    bool& currentToolOnly() noexcept { return m_currentToolOnly; }
    bool currentToolOnly() const noexcept { return m_currentToolOnly; }
    int& currentToolBatchIndex() noexcept { return m_currentToolBatchIndex; }
    int currentToolBatchIndex() const noexcept
    {
        return m_currentToolBatchIndex;
    }
    int& trailPageStartIndex() noexcept { return m_trailPageStartIndex; }
    int& trailPageEndIndex() noexcept { return m_trailPageEndIndex; }

private:
    // 帧请求状态机：m_frameInFlight 表示光标帧请求在飞行，
    // m_pendingFrame 记录飞行期间累积的最新请求；
    // trail 系列为轨迹块请求的对应状态。
    std::vector<PlaybackMove> m_moves;
    std::vector<int> m_toolBatchStarts;

    bool m_frameInFlight = false;
    bool m_pendingFrame = false;
    int m_pendingIndex = 0;
    double m_pendingFraction = 1.0;
    bool m_trailInFlight = false;
    bool m_trailPending = false;
    bool m_trailPendingForce = false;
    int m_trailPendingTo = 0;
    int m_trailFlushIndex = 0;
    QElapsedTimer m_renderThrottleTimer;
    QElapsedTimer m_redrawThrottleTimer;
    QElapsedTimer m_cameraFollowTimer;
    bool m_redrawPending = false;

    int m_displayLayer = 0;
    bool m_currentToolOnly = false;
    int m_currentToolBatchIndex = -1;
    int m_trailPageStartIndex = 0;
    int m_trailPageEndIndex = 0;
};

} // namespace oap::app

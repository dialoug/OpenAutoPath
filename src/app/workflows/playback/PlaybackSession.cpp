// PlaybackSession 实现：光标/轨迹请求合并、渲染与重绘节流、相机跟随节流。
#include "app/workflows/playback/PlaybackSession.h"

#include <algorithm>

namespace oap::app {

// 清空程序内容并复位请求状态。
void PlaybackSession::clearProgram()
{
    m_moves.clear();
    m_toolBatchStarts.clear();
    m_currentToolBatchIndex = -1;
    resetRequestState();
}

// 复位所有请求与节流状态；trailStartIndex 被夹取到合法范围后
// 作为轨迹与页面范围的起点。
void PlaybackSession::resetRequestState(int trailStartIndex)
{
    const int clamped = std::clamp(
        trailStartIndex, 0, static_cast<int>(m_moves.size()));
    m_frameInFlight = false;
    m_pendingFrame = false;
    m_pendingIndex = clamped;
    m_pendingFraction = 1.0;
    m_trailInFlight = false;
    m_trailPending = false;
    m_trailPendingForce = false;
    m_trailPendingTo = 0;
    m_trailFlushIndex = clamped;
    m_trailPageStartIndex = clamped;
    m_trailPageEndIndex = clamped;
    m_renderThrottleTimer.invalidate();
    m_redrawThrottleTimer.invalidate();
    m_cameraFollowTimer.invalidate();
    m_redrawPending = false;
}

// 返回指定换刀批次在 moves 中的半开区间；索引越界时返回空区间。
std::pair<int, int> PlaybackSession::toolBatchRange(
    int toolBatchIndex) const
{
    const int total = static_cast<int>(m_moves.size());
    if (toolBatchIndex < 0 ||
        toolBatchIndex >= static_cast<int>(m_toolBatchStarts.size())) {
        return {0, 0};
    }
    const int start = m_toolBatchStarts[toolBatchIndex];
    const int end = toolBatchIndex + 1 <
            static_cast<int>(m_toolBatchStarts.size())
        ? m_toolBatchStarts[toolBatchIndex + 1]
        : total;
    return {std::clamp(start, 0, total), std::clamp(end, 0, total)};
}

PlaybackSession::CursorDecision PlaybackSession::requestCursor(
    int index, double fraction)
{
    const int total = static_cast<int>(m_moves.size());
    const int clamped = std::clamp(index, 0, total);
    CursorDecision decision;
    // 回退检测：目标位置早于已刷新的轨迹末尾即视为回退。
    decision.rewound = clamped < m_trailFlushIndex;
    if (decision.rewound) {
        // 回退会取代任何仍在飞行中的前向光标帧。工作区使对应的
        // 工作线程请求 id 失效，本会话立即派发所请求的回退位置。
        m_frameInFlight = false;
        m_pendingFrame = false;
        m_trailInFlight = false;
        m_trailPending = false;
        m_trailPendingForce = false;
        m_trailPendingTo = 0;
        m_trailFlushIndex = clamped;
    }

    if (m_frameInFlight) {
        m_pendingFrame = true;
        m_pendingIndex = clamped;
        m_pendingFraction = fraction;
        return decision;
    }

    m_frameInFlight = true;
    decision.dispatch = CursorRequest{clamped, fraction};
    return decision;
}

std::optional<PlaybackSession::CursorRequest>
PlaybackSession::completeCursorRequest()
{
    m_frameInFlight = false;
    if (!m_pendingFrame)
        return std::nullopt;
    m_pendingFrame = false;
    m_frameInFlight = true;
    return CursorRequest{m_pendingIndex, m_pendingFraction};
}

void PlaybackSession::cursorDispatchFailed() noexcept
{
    m_frameInFlight = false;
}

bool PlaybackSession::shouldRenderFrame(
    int index, bool panelPlaying,
    bool modalOrPopup, bool mouseDragging)
{
    const int total = static_cast<int>(m_moves.size());
    // 未播放或处于首尾边界时强制渲染，不做节流。
    const bool force = !panelPlaying || index <= 0 || index >= total;
    if (force) {
        m_renderThrottleTimer.restart();
        return true;
    }

    const int minimumIntervalMs = modalOrPopup ? 120
        : mouseDragging ? 30 : 0;
    if (minimumIntervalMs <= 0)
        return true;
    if (!m_renderThrottleTimer.isValid()) {
        m_renderThrottleTimer.start();
        return true;
    }
    if (m_renderThrottleTimer.elapsed() < minimumIntervalMs)
        return false;
    m_renderThrottleTimer.restart();
    return true;
}

bool PlaybackSession::shouldFollowCamera(
    bool panelPlaying, double speedMultiplier)
{
    if (!panelPlaying)
        return false;
    const qint64 minimumIntervalMs = speedMultiplier >= 8.0 ? 500
        : speedMultiplier >= 4.0 ? 250 : 0;
    if (minimumIntervalMs <= 0)
        return true;
    if (!m_cameraFollowTimer.isValid()) {
        m_cameraFollowTimer.start();
        return true;
    }
    if (m_cameraFollowTimer.elapsed() < minimumIntervalMs)
        return false;
    m_cameraFollowTimer.restart();
    return true;
}

int PlaybackSession::beginRedrawRequest(qint64 minimumIntervalMs)
{
    if (m_redrawPending)
        return -1;
    int delayMs = 0;
    if (m_redrawThrottleTimer.isValid()) {
        const qint64 elapsed = m_redrawThrottleTimer.elapsed();
        if (elapsed < minimumIntervalMs) {
            delayMs = static_cast<int>(minimumIntervalMs - elapsed);
        }
    }
    m_redrawPending = true;
    return delayMs;
}

void PlaybackSession::completeRedrawRequest()
{
    m_redrawPending = false;
    m_redrawThrottleTimer.restart();
}

} // namespace oap::app

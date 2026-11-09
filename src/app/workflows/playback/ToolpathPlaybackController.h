// 模块归属：app/workflows/playback（oap_app_workflow_playback）刀路播放工作流。
// 职责：把光标/轨迹请求转投后台 worker，通过 generation/requestId 双门禁
//       丢弃过期帧后转发给界面线程。由播放面板/工作区持有。
// 线程：本对象在 UI 线程；worker 槽在工作线程执行，结果经队列信号回来。
// 拥有播放几何工作线程，并过滤过期的帧请求。
#pragma once

#include "app/workflows/playback/ToolpathPlaybackWorker.h"

#include <vector>

#include <QObject>

class QThread;

// 刀路播放控制器：协调播放会话与呈现。
class ToolpathPlaybackController final : public QObject {
    Q_OBJECT
public:
    explicit ToolpathPlaybackController(QObject* parent = nullptr);
    ~ToolpathPlaybackController() override;

    ToolpathPlaybackController(const ToolpathPlaybackController&) = delete;
    ToolpathPlaybackController& operator=(
        const ToolpathPlaybackController&) = delete;

    // 更换播放数据：令旧 generation 失效并重投新数据；
    // 返回 false 表示线程不可用。
    bool setMoves(std::vector<oap::toolpath::ToolpathMove> moves);
    // 清空播放数据并使所有在途请求失效。
    bool clear();
    // 投递光标帧请求；成功后自增光标 requestId 用于发布门禁。
    bool requestCursor(int index, double fraction);
    // 投递轨迹块请求（半开区间）；成功后自增轨迹 requestId。
    bool requestTrail(int fromIndex, int toIndex);
    // 使所有在途请求过期（不取消线程工作，仅丢弃其结果）。
    void invalidateRequests() noexcept;
    // 请求取消并提升 generation，令在途结果全部过期。
    void requestCancel() noexcept;
    // 取消并等待工作线程退出（析构与关闭时调用）。
    void cancelAndWait();
    bool running() const noexcept;

signals:
    void frameReady(const ToolpathPlaybackFrame& frame);

private:
    void handleFrame(const ToolpathPlaybackFrame& frame);

    QThread* m_thread = nullptr;
    ToolpathPlaybackWorker* m_worker = nullptr;
    int m_generation = 0;
    quint64 m_cursorRequestId = 0u;
    quint64 m_trailRequestId = 0u;
};

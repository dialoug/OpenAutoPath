// 模块归属：app/workflows/playback（oap_app_workflow_playback）刀路播放工作流。
// 职责：在后台线程为播放会话构建帧几何（已完成轨迹、当前段、刀尖），
//       由 ToolpathPlaybackController 持有 QThread 并驱动其生命期。
// 线程模型：所有槽都在工作线程排队执行；信号把只读帧送回 UI 线程。
// ToolpathPlaybackWorker —— 在 UI 线程之外构建播放帧几何。
// 它不会访问 AIS 对象、控件或 OCCT 查看器/上下文。
#pragma once

#include <atomic>
#include <vector>

#include <QObject>
#include <QtGlobal>

#include <gp_Pnt.hxx>
#include <TopoDS_Compound.hxx>

#include "toolpath/core/ToolpathMoves.h"

// 帧类型：Cursor 为光标位置（含当前段与刀尖），TrailChunk 为轨迹块。
enum class ToolpathPlaybackFrameKind {
    Cursor,
    TrailChunk,
};

// 一次后台构建的只读帧：generation/requestId 用于在 UI 线程过滤过期结果；
// 同帧最多携带一种有效几何（completedDelta、current 或 tip）。
struct ToolpathPlaybackFrame {
    ToolpathPlaybackFrameKind kind = ToolpathPlaybackFrameKind::Cursor;
    int generation = 0;
    quint64 requestId = 0;
    int fromIndex = 0;
    int toIndex = 0;
    double fraction = 1.0;

    bool hasCompletedDelta = false;
    TopoDS_Compound completedDelta;

    bool hasCurrent = false;
    TopoDS_Compound current;

    bool hasTip = false;
    gp_Pnt tip;
};

Q_DECLARE_METATYPE(ToolpathPlaybackFrame)

// 刀路播放工作线程：后台组装播放帧。
class ToolpathPlaybackWorker : public QObject
{
    Q_OBJECT
public:
    using QObject::QObject;
    void prepareWork() noexcept { m_cancelRequested.store(false); }
    void requestCancel() noexcept { m_cancelRequested.store(true); }

public slots:
    void setMoves(int generation, std::vector<oap::toolpath::ToolpathMove> moves);
    void clear(int generation);
    void buildCursorFrame(int generation, quint64 requestId,
                          int index, double fraction);
    void buildTrailChunk(int generation, quint64 requestId,
                         int fromIndex, int toIndex);

signals:
    void frameReady(const ToolpathPlaybackFrame& frame);

private:
    int m_generation = 0;
    std::vector<oap::toolpath::ToolpathMove> m_moves;
    std::atomic_bool m_cancelRequested{false};
};

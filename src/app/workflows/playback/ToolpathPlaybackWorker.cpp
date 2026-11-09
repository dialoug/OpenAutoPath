// 后台回放工作线程：按时间轴推进移动采样，并通过信号把只读帧发送给界面线程。
// 后台回放工作线程实现：移动采样、帧几何构建与取消检查。
#include "app/workflows/playback/ToolpathPlaybackWorker.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>

namespace {

// 播放帧几何的端点容差：低于该长度的线段被丢弃，避免退化边。
constexpr double kPlaybackMoveTolerance = 1.0e-9;

bool finitePoint(const gp_Pnt& p)
{
    return std::isfinite(p.X()) && std::isfinite(p.Y()) && std::isfinite(p.Z());
}

// 把线段加入复合体；非有限点或长度过短时跳过，
// OCCT 构边失败或异常时也安全返回 false。
bool addSegmentToCompound(const gp_Pnt& a,
                          const gp_Pnt& b,
                          BRep_Builder& builder,
                          TopoDS_Compound& compound)
{
    if (!finitePoint(a) || !finitePoint(b))
        return false;
    if (a.Distance(b) < kPlaybackMoveTolerance)
        return false;

    try {
        BRepBuilderAPI_MakeEdge edge(a, b);
        if (!edge.IsDone())
            return false;
        const TopoDS_Shape& shape = edge.Edge();
        builder.Add(compound, shape);
    } catch (const Standard_Failure&) {
        return false;
    } catch (...) {
        return false;
    }
    return true;
}

// 把一条刀路移动（from→to）作为线段加入复合体。
bool addMoveToCompound(const oap::toolpath::ToolpathMove& move,
                       BRep_Builder& builder,
                       TopoDS_Compound& compound)
{
    return addSegmentToCompound(move.from, move.to, builder, compound);
}

} // namespace

void ToolpathPlaybackWorker::setMoves(
    int generation,
    std::vector<oap::toolpath::ToolpathMove> moves)
{
    m_generation = generation;
    m_moves = std::move(moves);
}

void ToolpathPlaybackWorker::clear(int generation)
{
    m_generation = generation;
    m_moves.clear();
}

// 构建光标帧：按 fraction 对移动 [index] 线性插值得到刀尖位置，
// 并生成「当前段」线段；fraction >= 1 时只发布已完成的段与刀尖。
void ToolpathPlaybackWorker::buildCursorFrame(int generation,
                                              quint64 requestId,
                                              int index,
                                              double fraction)
{
    try {
        OCC_CATCH_SIGNALS
        OSD::SetThreadLocalSignal(OSD_SignalMode_Set, Standard_False);
    if (m_cancelRequested.load(std::memory_order_relaxed))
        return;
    if (generation != m_generation)
        return;

    ToolpathPlaybackFrame frame;
    frame.kind = ToolpathPlaybackFrameKind::Cursor;
    frame.generation = generation;
    frame.requestId = requestId;
    frame.fromIndex = index;
    frame.toIndex = index;
    frame.fraction = fraction;

    const int total = static_cast<int>(m_moves.size());
    const int committed = std::clamp(index, 0, total);
    // 仅当尚未走完整段移动时才有「当前段」；否则刀尖停在段末。
    const bool hasCurrent = fraction < 1.0 && committed < total;

    BRep_Builder builder;
    builder.MakeCompound(frame.completedDelta);
    builder.MakeCompound(frame.current);

    if (hasCurrent || committed > 0) {
        const oap::toolpath::ToolpathMove& cur =
            hasCurrent ? m_moves[static_cast<size_t>(committed)]
                       : m_moves[static_cast<size_t>(committed - 1)];
        const gp_Pnt a = cur.from;
        const gp_Pnt b = cur.to;
        const double t = hasCurrent ? std::clamp(fraction, 0.0, 1.0) : 1.0;
        frame.tip = gp_Pnt(a.X() + (b.X() - a.X()) * t,
                           a.Y() + (b.Y() - a.Y()) * t,
                           a.Z() + (b.Z() - a.Z()) * t);
        frame.hasTip = finitePoint(frame.tip);

        if (hasCurrent)
            frame.hasCurrent = addSegmentToCompound(a, frame.tip, builder, frame.current);
    }

    emit frameReady(frame);
    } catch (const Standard_Failure&) {
        return;
    } catch (...) {
        return;
    }
}

// 构建轨迹块：把 [fromIndex, toIndex) 半开区间内的移动逐段加入
// completedDelta 复合体；循环内支持取消。
void ToolpathPlaybackWorker::buildTrailChunk(int generation,
                                             quint64 requestId,
                                             int fromIndex,
                                             int toIndex)
{
    try {
        OCC_CATCH_SIGNALS
        OSD::SetThreadLocalSignal(OSD_SignalMode_Set, Standard_False);
    if (m_cancelRequested.load(std::memory_order_relaxed))
        return;
    if (generation != m_generation)
        return;

    ToolpathPlaybackFrame frame;
    frame.kind = ToolpathPlaybackFrameKind::TrailChunk;
    frame.generation = generation;
    frame.requestId = requestId;
    frame.fromIndex = fromIndex;
    frame.toIndex = toIndex;
    frame.fraction = 1.0;

    const int total = static_cast<int>(m_moves.size());
    const int from = std::clamp(fromIndex, 0, total);
    const int to = std::clamp(toIndex, 0, total);

    BRep_Builder builder;
    builder.MakeCompound(frame.completedDelta);
    builder.MakeCompound(frame.current);

    for (int i = from; i < to; ++i) {
        if (m_cancelRequested.load(std::memory_order_relaxed))
            return;
        if (addMoveToCompound(m_moves[static_cast<size_t>(i)], builder, frame.completedDelta))
            frame.hasCompletedDelta = true;
    }

    emit frameReady(frame);
    } catch (const Standard_Failure&) {
        return;
    } catch (...) {
        return;
    }
}

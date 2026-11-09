// ToolpathPlaybackController 实现：后台线程托管与帧发布门禁。
#include "app/workflows/playback/ToolpathPlaybackController.h"

#include <QMetaObject>
#include <QThread>

#include <utility>

// 创建后台线程与 worker，并建立队列连接：帧结果经 QueuedConnection
// 回到本对象（UI 线程）后再转发。
ToolpathPlaybackController::ToolpathPlaybackController(QObject* parent)
    : QObject(parent)
{
    qRegisterMetaType<ToolpathPlaybackFrame>("ToolpathPlaybackFrame");
    m_thread = new QThread(this);
    m_worker = new ToolpathPlaybackWorker;
    m_worker->moveToThread(m_thread);
    connect(m_thread, &QThread::finished,
            m_worker, &QObject::deleteLater);
    connect(m_worker, &ToolpathPlaybackWorker::frameReady,
            this, &ToolpathPlaybackController::handleFrame,
            Qt::QueuedConnection);
    m_thread->start();
}

ToolpathPlaybackController::~ToolpathPlaybackController()
{
    cancelAndWait();
}

// 更换播放数据：先取消旧工作，再以新 generation 重投；
// cursor/trail 请求 id 同步自增，使在途结果全部过期。
bool ToolpathPlaybackController::setMoves(
    std::vector<oap::toolpath::ToolpathMove> moves)
{
    if (!running() || !m_worker)
        return false;
    m_worker->requestCancel();
    const int generation = ++m_generation;
    ++m_cursorRequestId;
    ++m_trailRequestId;
    ToolpathPlaybackWorker* worker = m_worker;
    return QMetaObject::invokeMethod(
        worker,
        [worker, generation, moves = std::move(moves)]() mutable {
            worker->prepareWork();
            worker->setMoves(generation, std::move(moves));
        },
        Qt::QueuedConnection);
}

bool ToolpathPlaybackController::clear()
{
    if (!running() || !m_worker)
        return false;
    m_worker->requestCancel();
    const int generation = ++m_generation;
    ++m_cursorRequestId;
    ++m_trailRequestId;
    ToolpathPlaybackWorker* worker = m_worker;
    return QMetaObject::invokeMethod(
        worker,
        [worker, generation]() {
            worker->prepareWork();
            worker->clear(generation);
        },
        Qt::QueuedConnection);
}

bool ToolpathPlaybackController::requestCursor(int index,
                                               double fraction)
{
    if (!running() || !m_worker)
        return false;
    const int generation = m_generation;
    const quint64 requestId = ++m_cursorRequestId;
    ToolpathPlaybackWorker* worker = m_worker;
    return QMetaObject::invokeMethod(
        worker,
        [worker, generation, requestId, index, fraction]() {
            worker->buildCursorFrame(
                generation, requestId, index, fraction);
        },
        Qt::QueuedConnection);
}

bool ToolpathPlaybackController::requestTrail(int fromIndex,
                                              int toIndex)
{
    if (!running() || !m_worker || toIndex <= fromIndex)
        return false;
    const int generation = m_generation;
    const quint64 requestId = ++m_trailRequestId;
    ToolpathPlaybackWorker* worker = m_worker;
    return QMetaObject::invokeMethod(
        worker,
        [worker, generation, requestId, fromIndex, toIndex]() {
            worker->buildTrailChunk(
                generation, requestId, fromIndex, toIndex);
        },
        Qt::QueuedConnection);
}

void ToolpathPlaybackController::invalidateRequests() noexcept
{
    ++m_cursorRequestId;
    ++m_trailRequestId;
}

void ToolpathPlaybackController::requestCancel() noexcept
{
    ++m_generation;
    ++m_cursorRequestId;
    ++m_trailRequestId;
    if (m_worker)
        m_worker->requestCancel();
}

void ToolpathPlaybackController::cancelAndWait()
{
    requestCancel();
    if (m_thread && m_thread->isRunning()) {
        m_thread->quit();
        m_thread->wait();
    }
    m_worker = nullptr;
}

bool ToolpathPlaybackController::running() const noexcept
{
    return m_thread && m_thread->isRunning();
}

// 发布门禁：generation 或请求 id 任一不匹配的过期帧直接丢弃。
void ToolpathPlaybackController::handleFrame(
    const ToolpathPlaybackFrame& frame)
{
    if (frame.generation != m_generation)
        return;
    // 光标与轨迹各自维护请求 id，仅转发最新一帧。
    if (frame.kind == ToolpathPlaybackFrameKind::Cursor) {
        if (frame.requestId != m_cursorRequestId)
            return;
    } else if (frame.requestId != m_trailRequestId) {
        return;
    }
    emit frameReady(frame);
}

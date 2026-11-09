// IpwDisplayMeshController 实现：后台线程托管与结果发布门禁。
#include "app/workflows/simulation/IpwDisplayMeshController.h"

#include <QMetaObject>
#include <QThread>

#include <utility>

namespace oap::app {

toolpath::DexelDisplayMeshOptions IpwDisplayMeshController::meshOptionsForQuality(
    int level, double gridResolution)
{
    return ipwDisplayQualityProfile(level, gridResolution).meshOptions;
}

// 创建后台线程与 worker，帧结果经 QueuedConnection 回到 UI 线程。
IpwDisplayMeshController::IpwDisplayMeshController(QObject* parent)
    : QObject(parent)
{
    qRegisterMetaType<oap::app::IpwDisplayMeshResult>(
        "oap::app::IpwDisplayMeshResult");
    m_thread = new QThread(this);
    m_worker = new IpwDisplayMeshWorker;
    m_worker->moveToThread(m_thread);
    connect(m_thread, &QThread::finished,
            m_worker, &QObject::deleteLater);
    connect(m_worker, &IpwDisplayMeshWorker::resultReady,
            this, &IpwDisplayMeshController::handleResult,
            Qt::QueuedConnection);
    m_thread->start();
}

IpwDisplayMeshController::~IpwDisplayMeshController()
{
    cancelAndWait();
}

bool IpwDisplayMeshController::submit(IpwDisplayMeshRequest request)
{
    cancel(false);
    if (!running() || !m_worker)
        return false;

    request.generation = m_generation;
    request.requestId = ++m_requestId;
    request.cancellation =
        std::make_shared<std::atomic_bool>(false);
    m_cancellation = request.cancellation;
    m_inFlight = true;

    IpwDisplayMeshWorker* worker = m_worker;
    // 以队列方式投递构建请求，保证在 worker 线程执行。
    const bool queued = QMetaObject::invokeMethod(
        worker,
        [worker, request = std::move(request)]() mutable {
            worker->build(std::move(request));
        },
        Qt::QueuedConnection);
    if (!queued)
        cancel(false);
    return queued;
}

// 取消在途请求：置位取消标志并使请求 id 失效；
// invalidateGeneration 为 true 时额外提升代次。
void IpwDisplayMeshController::cancel(bool invalidateGeneration)
{
    if (m_cancellation) {
        m_cancellation->store(true, std::memory_order_relaxed);
    }
    m_cancellation.reset();
    m_inFlight = false;
    ++m_requestId;
    if (invalidateGeneration)
        ++m_generation;
}

void IpwDisplayMeshController::requestCancelCurrent() noexcept
{
    if (m_cancellation) {
        m_cancellation->store(true, std::memory_order_relaxed);
    }
}

void IpwDisplayMeshController::cancelAndWait()
{
    cancel(true);
    if (m_thread && m_thread->isRunning()) {
        m_thread->quit();
        m_thread->wait();
    }
    m_worker = nullptr;
}

bool IpwDisplayMeshController::running() const noexcept
{
    return m_thread && m_thread->isRunning();
}

// 发布门禁：generation 或 requestId 任一不匹配的过期结果直接丢弃，
// 匹配则复位在途状态并转发给所有者。
void IpwDisplayMeshController::handleResult(
    const IpwDisplayMeshResult& result)
{
    if (result.generation != m_generation ||
        result.requestId != m_requestId) {
        return;
    }
    m_inFlight = false;
    m_cancellation.reset();
    emit resultReady(result);
}

} // namespace oap::app


// AnalysisLoadController 实现:把 STEP 加载 / 缓存构建任务以队列方式投递到
// 独立 QThread,避免长任务阻塞 UI 线程;取消与等待逻辑集中在控制器上。
#include "app/analysis/AnalysisLoadController.h"

#include <QMetaObject>
#include <QThread>

#include <utility>

namespace oap::app {

// 构造函数:注册跨线程传递所需元类型,创建后台线程与 worker 并建立
// 队列连接(worker 的信号全部投递回 UI 线程),随后启动线程。
AnalysisLoadController::AnalysisLoadController(QObject* parent)
    : QObject(parent)
{
    qRegisterMetaType<std::shared_ptr<AnalysisLoadResult>>(
        "std::shared_ptr<AnalysisLoadResult>");
    qRegisterMetaType<std::shared_ptr<AnalysisLoadPreview>>(
        "std::shared_ptr<AnalysisLoadPreview>");
    qRegisterMetaType<oap::StepAnalysisCache*>(
        "oap::StepAnalysisCache*");

    m_thread = new QThread(this);
    m_worker = new AnalysisWorker;
    m_worker->moveToThread(m_thread);
    connect(m_thread, &QThread::finished,
            m_worker, &QObject::deleteLater);
    connect(m_worker, &AnalysisWorker::loadFinished,
            this, &AnalysisLoadController::loadFinished,
            Qt::QueuedConnection);
    connect(m_worker, &AnalysisWorker::previewReady,
            this, &AnalysisLoadController::previewReady, Qt::QueuedConnection);
    connect(m_worker, &AnalysisWorker::loadProgress,
            this, &AnalysisLoadController::loadProgress, Qt::QueuedConnection);
    connect(m_worker, &AnalysisWorker::cacheBuildProgress,
            this, &AnalysisLoadController::cacheBuildProgress,
            Qt::QueuedConnection);
    connect(m_worker, &AnalysisWorker::cacheBuildFinished,
            this, &AnalysisLoadController::cacheBuildFinished,
            Qt::QueuedConnection);
    m_thread->start();
}

// 析构:取消并等待后台线程退出,确保不再有信号回用到本对象。
AnalysisLoadController::~AnalysisLoadController()
{
    cancelAndWait();
}

// loadStepFile:提交前重置取消标志，再经 queued invokeMethod 执行加载。
// 闭包按值捕获移动后的参数并持裸 worker 指针,生命周期由本控制器保证。
bool AnalysisLoadController::loadStepFile(
    QString sourcePath,
    std::shared_ptr<StepAnalysisCache> analysisCache)
{
    if (!running() || !m_worker)
        return false;
    AnalysisWorker* worker = m_worker;
    // owner 保证单飞；必须先重置再入队，避免覆盖入队后发出的取消请求。
    worker->prepareRun();
    const quint64 requestId = ++m_requestId;
    return QMetaObject::invokeMethod(
        worker,
        [worker, requestId,
         sourcePath = std::move(sourcePath),
         analysisCache = std::move(analysisCache)]() mutable {
            worker->loadStepFile(
                sourcePath, analysisCache.get(), requestId);
        },
        Qt::QueuedConnection);
}

// buildStepCaches:同 loadStepFile，在入队前 prepareCacheBuild。
bool AnalysisLoadController::buildStepCaches(
    QStringList sourcePaths,
    std::shared_ptr<StepAnalysisCache> analysisCache)
{
    if (!running() || !m_worker)
        return false;
    AnalysisWorker* worker = m_worker;
    worker->prepareCacheBuild();
    return QMetaObject::invokeMethod(
        worker,
        [worker,
         sourcePaths = std::move(sourcePaths),
         analysisCache = std::move(analysisCache)]() mutable {
            worker->buildStepCaches(
                sourcePaths, analysisCache.get());
        },
        Qt::QueuedConnection);
}

void AnalysisLoadController::requestCancelLoad() noexcept
{
    if (m_worker)
        m_worker->requestCancel();
}

void AnalysisLoadController::requestCancelCacheBuild() noexcept
{
    if (m_worker)
        m_worker->requestCancelCacheBuild();
}

void AnalysisLoadController::requestCancelAll() noexcept
{
    if (m_worker)
        m_worker->requestCancel();
}

// cancelAndWait:先请求全部取消,再 quit+wait 等待线程退出,最后释放 worker。
void AnalysisLoadController::cancelAndWait()
{
    requestCancelAll();
    if (m_thread && m_thread->isRunning()) {
        m_thread->quit();
        m_thread->wait();
    }
    m_worker = nullptr;
}

bool AnalysisLoadController::running() const noexcept
{
    return m_thread && m_thread->isRunning();
}

} // namespace oap::app


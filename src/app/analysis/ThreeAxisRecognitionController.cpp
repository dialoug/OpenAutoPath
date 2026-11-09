// ThreeAxisRecognitionController 实现：不捕获 MainWindow，不从后台发布 UI。
#include "app/analysis/ThreeAxisRecognitionController.h"

#include "analysis/Model.h"
#include "analysis/ModelIndex.h"
#include "analysis/StepIndexer.h"
#include "toolpath/core/HeavyJobScheduler.h"
#include "toolpath/core/ToolpathParallel.h"

#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>

#include <exception>
#include <unordered_set>
#include <utility>

namespace oap::app {

// 构造函数:记录执行器(runner 为空则用默认 runDefault),不做线程启动。
ThreeAxisRecognitionController::ThreeAxisRecognitionController(Runner runner)
    : m_runner(runner ? std::move(runner) : &runDefault)
{
}

// 析构:取消并等待后台任务结束,避免任务在控制器销毁后仍引用其状态。
ThreeAxisRecognitionController::~ThreeAxisRecognitionController()
{
    cancelAndWait();
}

std::uint64_t ThreeAxisRecognitionController::start(
    ThreeAxisRecognitionRequest request)
{
    // 先撤销旧代次；排队任务取消直接完成，执行中的任务协作退出。
    cancelAndWait();
    const std::uint64_t requestId = ++m_generation;
    m_activeRequestId = requestId;
    m_state = std::make_shared<TaskState>();
    const std::shared_ptr<TaskState> state = m_state;
    const Runner runner = m_runner;
    // Freeze records and topology lookups before deferred execution. ModelIndex
    // stores pointers into Model, so rebuild it against the owned snapshot.
    auto model = request.model ? std::make_shared<Model>(*request.model) : nullptr;
    auto indexer = request.stepIndexer
        ? std::make_shared<StepIndexer>(*request.stepIndexer) : nullptr;
    std::shared_ptr<ModelIndex> index;
    if (request.modelIndex && model) {
        index = std::make_shared<ModelIndex>();
        index->build(*model);
    }
    request.model = model.get();
    request.modelIndex = index.get();
    request.stepIndexer = indexer.get();
    auto cancellation = std::shared_ptr<std::atomic_bool>(state, &state->cancelled);
    m_future = toolpath::submitHeavyJob(
        cancellation,
        [requestId, request, state, runner, model, indexer, index] {
            state->phase.store(ThreeAxisRecognitionPhase::RecognizingHoles,
                               std::memory_order_relaxed);
            ThreeAxisRecognitionTaskControl control;
            control.isCancellationRequested = [state] {
                return state->cancelled.load(std::memory_order_relaxed);
            };
            control.reportProgress = [state](
                                         ThreeAxisRecognitionPhase phase,
                                         std::size_t completed,
                                         std::size_t total) {
                state->phase.store(phase, std::memory_order_relaxed);
                state->completed.store(completed, std::memory_order_relaxed);
                state->total.store(total, std::memory_order_relaxed);
            };

            ThreeAxisRecognitionTaskResult result;
            result.requestId = requestId;
            try {
                result = runner(request, control);
                result.requestId = requestId;
            } catch (const Standard_Failure& failure) {
                const char* message = failure.GetMessageString();
                result.error = message ? message :
                    "OCCT recognition controller failure";
            } catch (const std::exception& failure) {
                result.error = failure.what();
            } catch (...) {
                result.error = "unknown recognition controller failure";
            }
            result.cancelled = result.cancelled || control.cancelled();
            state->phase.store(
                ThreeAxisRecognitionPhase::Finished,
                std::memory_order_relaxed);
            return result;
        }, [requestId, state](toolpath::HeavyJobRejection reason) {
            ThreeAxisRecognitionTaskResult result;
            result.requestId = requestId;
            result.cancelled = reason == toolpath::HeavyJobRejection::Cancelled;
            if (!result.cancelled)
                result.error = toolpath::heavyJobRejectionMessage(reason);
            state->phase.store(ThreeAxisRecognitionPhase::Finished,
                               std::memory_order_relaxed);
            return result;
        });
    return requestId;
}

// cancel:仅原子置位取消标志,不阻塞。requestId 非 0 且不是当前代次时忽略。
void ThreeAxisRecognitionController::cancel(std::uint64_t requestId)
{
    if (!m_state ||
        (requestId != 0 && requestId != m_activeRequestId)) {
        return;
    }
    toolpath::cancelHeavyJob(
        std::shared_ptr<std::atomic_bool>(m_state, &m_state->cancelled));
}

// cancelAndWait:置位取消并阻塞等待任务结束,随后清理 future/state。
// 析构与 start 都会调用,保证同一时刻至多一个活动代次。
void ThreeAxisRecognitionController::cancelAndWait()
{
    cancel();
    if (m_future.valid())
        m_future.wait();
    m_future = {};
    m_state.reset();
    m_activeRequestId = 0;
}

// progress:从共享原子状态读取快照;代次不匹配时返回仅含 requestId 的空值。
ThreeAxisRecognitionProgress ThreeAxisRecognitionController::progress(
    std::uint64_t requestId) const
{
    ThreeAxisRecognitionProgress value;
    value.requestId = requestId;
    if (requestId == 0 || requestId != m_activeRequestId || !m_state)
        return value;
    value.phase = m_state->phase.load(std::memory_order_relaxed);
    value.completed = m_state->completed.load(std::memory_order_relaxed);
    value.total = m_state->total.load(std::memory_order_relaxed);
    value.ready = ready(requestId);
    return value;
}

// ready:非阻塞检查结果是否已就绪(valid 且 future 已 ready)。
bool ThreeAxisRecognitionController::ready(std::uint64_t requestId) const
{
    return requestId != 0 && requestId == m_activeRequestId &&
        m_future.valid() &&
        m_future.wait_for(std::chrono::milliseconds(0)) ==
            std::future_status::ready;
}

// take:取走已就绪结果并重置内部状态;取走前必须 ready() 通过。
std::optional<ThreeAxisRecognitionTaskResult>
ThreeAxisRecognitionController::take(std::uint64_t requestId)
{
    if (!ready(requestId))
        return std::nullopt;
    ThreeAxisRecognitionTaskResult result = m_future.get();
    m_state.reset();
    m_activeRequestId = 0;
    return result;
}

// runDefault:默认执行器 —— 先做孔/槽/腔特征识别,再把识别到的孔面集合
// 传给三向线框追踪;两个阶段之间及内部均检查取消标志。
ThreeAxisRecognitionTaskResult
ThreeAxisRecognitionController::runDefault(
    const ThreeAxisRecognitionRequest& request,
    const ThreeAxisRecognitionTaskControl& control)
{
    ThreeAxisRecognitionTaskResult result;
    if (!request.model || !request.modelIndex || !request.stepIndexer) {
        result.error = "recognition controller requires model and topology indexes";
        return result;
    }

    OCC_CATCH_SIGNALS
    OSD::SetThreadLocalSignal(OSD_SignalMode_Set, Standard_False);
    control.reportProgress(
        ThreeAxisRecognitionPhase::RecognizingHoles, 0, 0);
    result.recognition = FeatureRecognizer().recognize(
        *request.model, *request.stepIndexer);
    if (control.cancelled()) {
        result.cancelled = true;
        return result;
    }

    std::unordered_set<std::string> recognizedHoleFaces;
    for (const auto& [faceId, kind] : result.recognition.faceKind) {
        if (kind == FeatureKind::ThroughHole ||
            kind == FeatureKind::BlindHole ||
            kind == FeatureKind::CountersinkHole) {
            recognizedHoleFaces.insert(faceId);
        }
    }

    ThreeAxisWireTraceRequest traceRequest;
    traceRequest.model = request.model;
    traceRequest.modelIndex = request.modelIndex;
    traceRequest.stepIndexer = request.stepIndexer;
    traceRequest.recognizedHoleFaceIds = std::move(recognizedHoleFaces);
    ThreeAxisWireTraceControl traceControl;
    traceControl.isCancellationRequested =
        control.isCancellationRequested;
    traceControl.executeRange = [](
        std::size_t count, const std::function<void(std::size_t)>& task) {
        toolpath::toolpathParallelForBudgeted(count, task);
    };
    traceControl.reportProgress = [&control](
                                      std::size_t completed,
                                      std::size_t total) {
        control.reportProgress(
            ThreeAxisRecognitionPhase::TracingStructures,
            completed, total);
    };
    result.wireTrace = ThreeAxisWireTraceUseCase().execute(
        traceRequest, &traceControl);
    result.cancelled = result.wireTrace.cancelled || control.cancelled();
    result.error = result.wireTrace.error;
    return result;
}

} // namespace oap::app

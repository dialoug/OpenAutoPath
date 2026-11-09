// 后台 worker/controller 的代次、请求身份、取消、异常和析构特征测试。
#include "app/analysis/ThreeAxisRecognitionController.h"
#include "app/analysis/AnalysisLoadController.h"
#include "app/workflows/playback/ToolpathPlaybackController.h"
#include "analysis/Model.h"
#include "analysis/ModelIndex.h"
#include "analysis/StepIndexer.h"
#include "toolpath/core/HeavyJobScheduler.h"
#include "toolpath/workflow/ToolpathAsyncExecutor.h"

#include <QCoreApplication>
#include <QObject>
#include <QSemaphore>
#include <QThread>

#include <BRepPrimAPI_MakeBox.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

oap::toolpath::ToolpathMove move(double x0, double x1)
{
    oap::toolpath::ToolpathMove result;
    result.type = oap::toolpath::ToolpathMoveType::Cut;
    result.from = gp_Pnt(x0, 0.0, 0.0);
    result.to = gp_Pnt(x1, 0.0, 0.0);
    return result;
}

bool generationAndCancellationGateResults()
{
    ToolpathPlaybackWorker worker;
    std::vector<ToolpathPlaybackFrame> frames;
    QObject::connect(
        &worker, &ToolpathPlaybackWorker::frameReady,
        [&frames](const ToolpathPlaybackFrame& frame) {
            frames.push_back(frame);
        });

    worker.setMoves(7, {move(0.0, 5.0), move(5.0, 10.0)});
    worker.buildCursorFrame(6, 1u, 0, 0.5);
    if (!expect(frames.empty(),
                "a stale generation must not emit a playback frame")) {
        return false;
    }

    worker.buildCursorFrame(7, 41u, 0, 0.5);
    if (!expect(frames.size() == 1 &&
                    frames.back().generation == 7 &&
                    frames.back().requestId == 41u &&
                    frames.back().hasCurrent && frames.back().hasTip &&
                    frames.back().tip.Distance(gp_Pnt(2.5, 0.0, 0.0)) <
                        1.0e-9,
                "the current generation must echo request identity and geometry")) {
        return false;
    }

    worker.requestCancel();
    worker.buildTrailChunk(7, 42u, 0, 2);
    if (!expect(frames.size() == 1,
                "a cancelled worker must not emit a late result")) {
        return false;
    }

    worker.prepareWork();
    worker.clear(8);
    worker.buildCursorFrame(7, 43u, 0, 1.0);
    worker.buildCursorFrame(8, 44u, 0, 1.0);
    return expect(frames.size() == 2 &&
                      frames.back().generation == 8 &&
                      frames.back().requestId == 44u &&
                      !frames.back().hasCurrent && !frames.back().hasTip,
                  "clear must advance generation and permit only the new empty state");
}

template <typename Predicate>
bool waitFor(Predicate&& predicate)
{
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(2);
    while (!predicate() &&
           std::chrono::steady_clock::now() < deadline) {
        QCoreApplication::processEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    QCoreApplication::processEvents();
    return predicate();
}

bool playbackControllerPublishesOnlyCurrentRequests()
{
    ToolpathPlaybackController controller;
    std::vector<ToolpathPlaybackFrame> frames;
    QObject::connect(
        &controller, &ToolpathPlaybackController::frameReady,
        [&frames](const ToolpathPlaybackFrame& frame) {
            frames.push_back(frame);
        });

    if (!expect(controller.running(),
                "the playback controller should own a running thread") ||
        !expect(controller.setMoves(
                    {move(0.0, 5.0), move(5.0, 10.0)}),
                "the playback controller should accept moves") ||
        !expect(controller.requestCursor(0, 0.25) &&
                    controller.requestCursor(1, 0.5),
                "the playback controller should queue cursor requests") ||
        !expect(waitFor([&frames] { return !frames.empty(); }),
                "the current cursor request should complete")) {
        return false;
    }

    if (!expect(frames.size() == 1 && frames.back().hasTip &&
                    frames.back().tip.Distance(
                        gp_Pnt(7.5, 0.0, 0.0)) < 1.0e-9,
                "a replaced cursor request must not be published")) {
        return false;
    }

    frames.clear();
    if (!expect(controller.clear() &&
                    controller.requestCursor(0, 1.0),
                "clear should establish a new empty generation") ||
        !expect(waitFor([&frames] { return !frames.empty(); }),
                "the empty generation should still produce a value frame")) {
        return false;
    }
    controller.cancelAndWait();
    controller.cancelAndWait();
    return expect(frames.size() == 1 && !frames.back().hasTip &&
                      !frames.back().hasCurrent && !controller.running(),
                  "controller shutdown must be repeatable and join the worker");
}

bool analysisControllerOwnsRepeatableShutdown()
{
    oap::app::AnalysisLoadController controller;
    if (!expect(controller.running(),
                "the analysis controller should own a running thread")) {
        return false;
    }
    controller.cancelAndWait();
    controller.cancelAndWait();
    return expect(!controller.running(),
                  "analysis controller shutdown must be repeatable");
}

// Hold the worker event loop before submission so cancellation always happens
// after enqueueing but before execution, without relying on sleep timing.
bool analysisCancellationBeforeExecution(bool cacheBuild)
{
    oap::app::AnalysisLoadController controller;
    QThread* thread = controller.findChild<QThread*>();
    if (!expect(thread != nullptr, "analysis worker thread must exist"))
        return false;
    auto* blocker = new QObject;
    blocker->moveToThread(thread);
    QObject::connect(thread, &QThread::finished, blocker, &QObject::deleteLater);
    auto entered = std::make_shared<QSemaphore>();
    auto release = std::make_shared<QSemaphore>();
    if (!expect(QMetaObject::invokeMethod(blocker, [entered, release] {
                    entered->release();
                    release->acquire();
                }, Qt::QueuedConnection),
                "worker barrier must be queued")) {
        return false;
    }
    if (!entered->tryAcquire(1, 2000)) {
        release->release();
        return expect(false, "worker barrier must start");
    }

    int completions = 0;
    bool cancelled = false;
    bool didWork = false;
    QObject::connect(&controller, &oap::app::AnalysisLoadController::loadFinished,
                     &controller, [&](std::shared_ptr<AnalysisLoadResult> result) {
        ++completions;
        cancelled = result && result->cancelled;
        didWork = !result || result->ok || !result->errorMessage.isEmpty();
    });
    QObject::connect(&controller, &oap::app::AnalysisLoadController::cacheBuildFinished,
                     &controller, [&](int built, int skipped, int failed, bool stopped) {
        ++completions;
        cancelled = stopped;
        didWork = built != 0 || skipped != 0 || failed != 0;
    });
    const auto submit = [&] {
        // Empty paths fail normal loading, making cancellation distinguishable
        // from an attempted load without requiring an external STEP fixture.
        return cacheBuild
            ? controller.buildStepCaches({QString{}}, {})
            : controller.loadStepFile({}, {});
    };
    const bool accepted = submit();
    if (cacheBuild)
        controller.requestCancelCacheBuild();
    else
        controller.requestCancelLoad();
    release->release();
    const bool finished = waitFor([&] { return completions == 1; });
    if (!expect(accepted && finished && cancelled && !didWork,
                "queued cancellation must survive task startup and skip I/O")) {
        return false;
    }

    // A subsequent accepted task must reset the previous cancellation.
    if (!expect(submit() && waitFor([&] { return completions == 2; }) &&
                    !cancelled && didWork,
                "a new task must not inherit previous cancellation")) {
        return false;
    }
    controller.cancelAndWait();
    return true;
}

bool waitUntilReady(oap::app::ThreeAxisRecognitionController& controller,
                    std::uint64_t requestId)
{
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(2);
    while (!controller.ready(requestId) &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return controller.ready(requestId);
}

bool recognitionControllerReplacesAndStructuresFailures()
{
    std::atomic_int callCount{0};
    oap::app::ThreeAxisRecognitionController controller(
        [&callCount](const oap::app::ThreeAxisRecognitionRequest&,
                     const oap::app::ThreeAxisRecognitionTaskControl& control) {
            oap::app::ThreeAxisRecognitionTaskResult result;
            const int call = ++callCount;
            if (call == 1) {
                while (!control.cancelled())
                    std::this_thread::yield();
                result.cancelled = true;
                return result;
            }
            result.recognition.holeCount = 23;
            return result;
        });

    const std::uint64_t stale = controller.start({});
    // Replacement of a running request and cancellation while still queued
    // are separate contracts. Ensure this test exercises the former.
    if (!expect(waitFor([&] { return callCount.load() == 1; }),
                "the first recognition request must enter its runner"))
        return false;
    const std::uint64_t current = controller.start({});
    if (!expect(current > stale && controller.currentRequestId() == current,
                "a replacement request must advance the generation") ||
        !expect(!controller.take(stale).has_value(),
                "a replaced request must never be takeable") ||
        !expect(waitUntilReady(controller, current),
                "the replacement request should finish")) {
        return false;
    }
    auto currentResult = controller.take(current);
    if (!expect(currentResult && currentResult->requestId == current &&
                    currentResult->ok() &&
                    currentResult->recognition.holeCount == 23,
                "only the current generation should publish its value result")) {
        return false;
    }

    oap::app::ThreeAxisRecognitionController failing(
        [](const oap::app::ThreeAxisRecognitionRequest&,
           const oap::app::ThreeAxisRecognitionTaskControl&)
            -> oap::app::ThreeAxisRecognitionTaskResult {
            throw std::runtime_error("injected recognition failure");
        });
    const std::uint64_t failedId = failing.start({});
    if (!expect(waitUntilReady(failing, failedId),
                "an exceptional request should still become ready")) {
        return false;
    }
    const auto failed = failing.take(failedId);
    return expect(failed && !failed->ok() && !failed->cancelled &&
                      failed->error == "injected recognition failure",
                  "runner exceptions must become structured failure results");
}

bool recognitionControllerDestructorCancelsAndWaits()
{
    std::atomic_bool workerEntered{false};
    std::atomic_bool workerExited{false};
    {
        oap::app::ThreeAxisRecognitionController controller(
            [&workerEntered, &workerExited](
                const oap::app::ThreeAxisRecognitionRequest&,
                const oap::app::ThreeAxisRecognitionTaskControl& control) {
                workerEntered.store(true, std::memory_order_relaxed);
                while (!control.cancelled())
                    std::this_thread::yield();
                workerExited.store(true, std::memory_order_relaxed);
                oap::app::ThreeAxisRecognitionTaskResult result;
                result.cancelled = true;
                return result;
            });
        controller.start({});
        if (!expect(waitFor([&] { return workerEntered.load(); }),
                    "the running-destructor fixture must enter its runner"))
            return false;
    }
    return expect(workerExited.load(std::memory_order_relaxed),
                  "controller destruction must cancel and join its worker");
}

// Occupy the process-wide scheduler before submitting through real public
// controllers/services. The timeout and destructor release prevent a broken
// queued-cancellation implementation from hanging this test indefinitely.
class HeavyJobBlocker {
public:
    HeavyJobBlocker()
    {
        const auto state = m_state;
        m_future = oap::toolpath::submitHeavyJob(
            std::make_shared<std::atomic_bool>(false),
            [state] {
                std::unique_lock lock(state->mutex);
                state->entered = true;
                state->changed.notify_all();
                state->changed.wait_for(lock, std::chrono::seconds(5),
                                        [&] { return state->released; });
                state->finished = true;
            },
            [state](oap::toolpath::HeavyJobRejection) {
                std::lock_guard lock(state->mutex);
                state->finished = true;
                state->changed.notify_all();
            });
    }

    ~HeavyJobBlocker()
    {
        release();
        m_future.wait();
    }

    void release()
    {
        {
            std::lock_guard lock(m_state->mutex);
            m_state->released = true;
        }
        m_state->changed.notify_all();
    }

    bool entered()
    {
        std::unique_lock lock(m_state->mutex);
        return m_state->changed.wait_for(lock, std::chrono::seconds(2),
                                        [&] { return m_state->entered || m_state->finished; }) &&
            m_state->entered && !m_state->finished;
    }

    bool stillBlocked()
    {
        std::lock_guard lock(m_state->mutex);
        return m_state->entered && !m_state->finished;
    }

private:
    struct State {
        std::mutex mutex;
        std::condition_variable changed;
        bool entered = false;
        bool released = false;
        bool finished = false;
    };
    std::shared_ptr<State> m_state = std::make_shared<State>();
    std::future<void> m_future;
};

bool recognitionQueuedCancellationSkipsRunner()
{
    HeavyJobBlocker blocker;
    if (!expect(blocker.entered(), "shared heavy-job blocker must start"))
        return false;
    std::atomic_int calls{0};
    const auto runner = [&calls](
                            const oap::app::ThreeAxisRecognitionRequest&,
                            const oap::app::ThreeAxisRecognitionTaskControl&) {
        ++calls;
        return oap::app::ThreeAxisRecognitionTaskResult{};
    };
    oap::app::ThreeAxisRecognitionController controller(runner);
    const auto requestId = controller.start({});
    if (!expect(!controller.ready(requestId) &&
                    oap::toolpath::heavyJobScheduler().pendingCount() == 1u,
                "recognition must queue behind an unrelated heavy job"))
        return false;
    controller.cancel(requestId);
    if (!expect(waitUntilReady(controller, requestId) && blocker.stillBlocked(),
                "queued recognition cancellation must finish without waiting for the active job"))
        return false;
    const auto result = controller.take(requestId);
    if (!expect(result && result->requestId == requestId && result->cancelled &&
                    calls.load() == 0 && oap::toolpath::heavyJobScheduler().pendingCount() == 0u,
                "queued cancellation must preserve identity and never call the runner"))
        return false;

    {
        oap::app::ThreeAxisRecognitionController destroyedWhileQueued(runner);
        destroyedWhileQueued.start({});
    }
    return expect(blocker.stillBlocked() && calls.load() == 0 &&
                      oap::toolpath::heavyJobScheduler().pendingCount() == 0u,
                  "destroying queued recognition must release it before the active job finishes");
}

bool toolpathQueuedCancellationCompletesAllBatchKinds()
{
    HeavyJobBlocker blocker;
    if (!expect(blocker.entered(), "toolpath admission blocker must start"))
        return false;
    oap::toolpath::ToolpathAsyncExecutor executor;
    std::atomic_int progressCalls{0};
    const auto progress = [&](std::size_t, std::size_t) { ++progressCalls; };
    auto pockets = executor.generatePockets({}, {}, "QUEUED_POCKET", progress);
    auto holes = executor.generateHoles({}, {}, std::nullopt, {}, progress);
    auto profiles = executor.generateProfiles({}, {}, "QUEUED_PROFILE", progress);
    if (!expect(!pockets.ready() && !holes.ready() && !profiles.ready() &&
                    oap::toolpath::heavyJobScheduler().pendingCount() == 3u,
                "all three public toolpath APIs must share heavy-job admission"))
        return false;
    pockets.cancel();
    holes.cancel();
    profiles.cancel();
    if (!expect(waitFor([&] { return pockets.ready() && holes.ready() && profiles.ready(); }) &&
                    blocker.stillBlocked(),
                "cancelled queued toolpaths must become ready before the active job ends"))
        return false;
    const auto pocketResult = pockets.get();
    const auto holeResult = holes.get();
    const auto profileResult = profiles.get();
    return expect(pocketResult.cancelled && holeResult.cancelled && profileResult.cancelled &&
                      progressCalls.load() == 0 &&
                      oap::toolpath::heavyJobScheduler().pendingCount() == 0u,
                  "queued toolpaths must return cancelled batches without running services or progress");
}

bool queuedRecognitionOwnsItsModelAndTopologySnapshot()
{
    auto model = std::make_unique<oap::Model>();
    auto indexer = std::make_unique<oap::StepIndexer>();
    auto index = std::make_unique<oap::ModelIndex>();
    const auto application = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) document;
    application->NewDocument("MDTV-XCAF", document);
    XCAFDoc_DocumentTool::ShapeTool(document->Main())->AddShape(
        BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape(), Standard_False);
    const bool built = indexer->build(document, *model);
    application->Close(document);
    if (!expect(built && !model->faces.empty() && !model->edges.empty() && !model->wires.empty(),
                "snapshot fixture should contain indexed model records and topology"))
        return false;
    index->build(*model);
    const std::string faceId = model->faces.front().face_id;
    const std::string edgeId = model->edges.front().edge_id;
    const std::string wireId = model->wires.front().wire_id;
    const double faceArea = model->faces.front().area;
    const auto faceCount = model->faces.size();
    const auto edgeCount = model->edges.size();
    const auto wireCount = model->wires.size();

    HeavyJobBlocker blocker;
    if (!expect(blocker.entered(), "snapshot admission blocker must start"))
        return false;
    oap::app::ThreeAxisRecognitionController controller(
        [=](const oap::app::ThreeAxisRecognitionRequest& request,
            const oap::app::ThreeAxisRecognitionTaskControl&) {
            oap::app::ThreeAxisRecognitionTaskResult result;
            if (!request.model || !request.modelIndex || !request.stepIndexer ||
                request.model->faces.size() != faceCount ||
                request.model->edges.size() != edgeCount ||
                request.model->wires.size() != wireCount ||
                request.model->faces.front().face_id != faceId ||
                request.model->faces.front().area != faceArea ||
                request.modelIndex->face(faceId) != &request.model->faces.front() ||
                request.modelIndex->edge(edgeId) != &request.model->edges.front() ||
                request.modelIndex->wire(wireId) != &request.model->wires.front()) {
                result.error = "queued recognition lost its owned model/index snapshot";
                return result;
            }
            const auto* face = request.stepIndexer->faceForId(faceId);
            const auto* edge = request.stepIndexer->edgeForId(edgeId);
            const auto* wire = request.stepIndexer->wireForId(wireId);
            if (!face || face->IsNull() || !edge || edge->IsNull() ||
                !wire || wire->IsNull() || request.stepIndexer->idForFace(*face) != faceId ||
                request.stepIndexer->idForEdge(*edge) != edgeId ||
                request.stepIndexer->idForWire(*wire) != wireId) {
                result.error = "queued recognition lost its copied topology maps";
                return result;
            }
            result.recognition.holeCount = 29;
            return result;
        });
    const auto requestId = controller.start({model.get(), index.get(), indexer.get()});
    if (!expect(!controller.ready(requestId) && blocker.stillBlocked() &&
                    oap::toolpath::heavyJobScheduler().pendingCount() == 1u,
                "snapshot verification must stay queued until original inputs are destroyed"))
        return false;

    // Mutate all three original inputs, then destroy them before work begins.
    // ModelIndex's copied maps must point into the owned Model, not these vectors.
    model->faces.front().area = -999.0;
    model->faces.front().face_id = "REPLACED_FACE";
    index->clear();
    *indexer = oap::StepIndexer{};
    model.reset();
    index.reset();
    indexer.reset();
    blocker.release();
    if (!expect(waitUntilReady(controller, requestId),
                "queued recognition should execute after its original inputs are destroyed"))
        return false;
    const auto result = controller.take(requestId);
    return expect(result && result->ok() && result->requestId == requestId &&
                      result->recognition.holeCount == 29,
                  "recognition must observe the submitted model, rebuilt indexes and copied topology");
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    if (!generationAndCancellationGateResults() ||
        !playbackControllerPublishesOnlyCurrentRequests() ||
        !analysisControllerOwnsRepeatableShutdown() ||
        !analysisCancellationBeforeExecution(false) ||
        !analysisCancellationBeforeExecution(true) ||
        !recognitionControllerReplacesAndStructuresFailures() ||
        !recognitionControllerDestructorCancelsAndWaits() ||
        !recognitionQueuedCancellationSkipsRunner() ||
        !toolpathQueuedCancellationCompletesAllBatchKinds() ||
        !queuedRecognitionOwnsItsModelAndTopologySnapshot())
        return 1;
    std::cout << "task lifecycle smoke tests passed\n";
    return 0;
}

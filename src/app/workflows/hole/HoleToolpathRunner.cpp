// 孔刀路运行器：调用无界面工作流，并通过交互端口发布进度和取消。
#include "app/workflows/hole/HoleToolpathRunner.h"
#include <QCoreApplication>

#include <atomic>
#include <exception>
#include <future>
#include <memory>

#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>

#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/core/HeavyJobScheduler.h"
#include "analysis/Model.h"
#include "analysis/StepIndexer.h"

#include "toolpath/workflow/ToolpathAsyncExecutor.h"

namespace oap::app {
namespace {
tooling::HoleAutomaticToolpathResult holeJobNotStarted(toolpath::HeavyJobRejection reason)
{
    tooling::HoleAutomaticToolpathResult result;
    result.batch.cancelled = reason == toolpath::HeavyJobRejection::Cancelled;
    result.batch.diagnostics.push_back({
        result.batch.cancelled ? toolpath::ToolpathDiagnosticCode::GenerationCancelled
                               : toolpath::ToolpathDiagnosticCode::AlgorithmExecutionFailed,
        result.batch.cancelled ? toolpath::ToolpathDiagnosticSeverity::Info
                               : toolpath::ToolpathDiagnosticSeverity::Error,
        toolpath::heavyJobRejectionMessage(reason)});
    return result;
}
} // namespace

std::optional<HoleGeometryRecognitionResult> recognizeHolesWithProgress(
    const WorkflowProgressPresenter& progress,
    const Model& model,
    const StepIndexer& indexer,
    std::string* error)
{
    if (error) error->clear();
    std::string rejectionError;
    std::atomic_size_t completed{0};
    std::atomic_size_t total{1};
    auto cancellation = std::make_shared<std::atomic_bool>(false);
    // 后台线程执行识别；屏蔽 OCCT 信号，异常时返回空识别结果。
    auto future = toolpath::submitHeavyJob(
        cancellation,
        [model, indexer, cancellation, &completed]() {
            try {
                OCC_CATCH_SIGNALS
                OSD::SetThreadLocalSignal(
                    OSD_SignalMode_Set, Standard_False);
                HoleGeometryRecognitionResult result =
                    FeatureRecognizer().recognizeHoleGeometries(model, indexer);
                completed.store(1, std::memory_order_relaxed);
                return result;
            } catch (const Standard_Failure&) {
                completed.store(1, std::memory_order_relaxed);
                return HoleGeometryRecognitionResult{};
            } catch (...) {
                completed.store(1, std::memory_order_relaxed);
                return HoleGeometryRecognitionResult{};
            }
        }, [&rejectionError](toolpath::HeavyJobRejection reason) {
            if (reason != toolpath::HeavyJobRejection::Cancelled)
                rejectionError = toolpath::heavyJobRejectionMessage(reason);
            return HoleGeometryRecognitionResult{};
        });
    toolpath::ToolpathAsyncTask<HoleGeometryRecognitionResult> task(
        std::move(future), cancellation);
    WorkflowProgressRequest progressRequest;
    progressRequest.title = QCoreApplication::translate("OpenAutoPath", "孔特征识别");
    progressRequest.label = QCoreApplication::translate("OpenAutoPath", "正在后台识别孔特征…");
    progressRequest.cancellingLabel =
        QCoreApplication::translate("OpenAutoPath", "正在取消，等待当前拓扑识别阶段结束…");
    progressRequest.initialTotal = 1;
    progressRequest.snapshot = [&] {
        return WorkflowProgressSnapshot{
            completed.load(std::memory_order_relaxed),
            total.load(std::memory_order_relaxed), {}};
    };
    progressRequest.queued = [&task] { return task.queued(); };
    progressRequest.ready = [&task] { return task.ready(); };
    progressRequest.cancel = [&task] { task.cancel(); };
    progressRequest.cancellationRequested = [cancellation] {
        return cancellation->load(std::memory_order_relaxed);
    };
    waitForWorkflowTask(progress, progressRequest);
    HoleGeometryRecognitionResult result = task.get();
    if (!rejectionError.empty()) {
        if (error) *error = std::move(rejectionError);
        return std::nullopt;
    }
    if (cancellation->load(std::memory_order_relaxed))
        return std::nullopt;
    return result;
}

tooling::HoleAutomaticToolpathResult generateAutomaticHolesWithProgress(
    const WorkflowProgressPresenter& progress,
    const tooling::ToolLibrary& library,
    const std::vector<toolpath::HoleMachiningInput>& inputs,
    const toolpath::ToolpathParams& params,
    const toolpath::HoleRapidPlanningContext& rapidContext)
{
    std::atomic_size_t completed{0};
    std::atomic_size_t total{inputs.size()};
    // 自动模式：后台线程内用 ToolpathGenerationControl 上报进度与取消。
    auto cancellation = std::make_shared<std::atomic_bool>(false);
    auto future = toolpath::submitHeavyJob(
        cancellation,
        [library, inputs, params, rapidContext, cancellation,
         &completed, &total]() mutable {
            try {
                OCC_CATCH_SIGNALS
                OSD::SetThreadLocalSignal(OSD_SignalMode_Set, Standard_False);
                toolpath::ToolpathGenerationControl control;
                toolpath::attachGlobalToolpathParallelBudget(control);
                control.isCancellationRequested = [cancellation] {
                    return cancellation->load(std::memory_order_relaxed);
                };
                control.reportProgress = [&completed, &total](
                                             std::size_t done,
                                             std::size_t count) {
                    completed.store(done, std::memory_order_relaxed);
                    total.store(count, std::memory_order_relaxed);
                };
                return tooling::HoleAutomaticToolpath().generate(
                    library, inputs, params, &rapidContext,
                    &control);
            } catch (const Standard_Failure& failure) {
                tooling::HoleAutomaticToolpathResult result;
                const char* message = failure.GetMessageString();
                result.batch.diagnostics.push_back({
                    toolpath::ToolpathDiagnosticCode::AlgorithmExecutionFailed,
                    toolpath::ToolpathDiagnosticSeverity::Error,
                    std::string("OCCT automatic hole toolpath failure: ") +
                        (message ? message : "OCCT failure")});
                return result;
            } catch (const std::exception& failure) {
                tooling::HoleAutomaticToolpathResult result;
                result.batch.diagnostics.push_back({
                    toolpath::ToolpathDiagnosticCode::AlgorithmExecutionFailed,
                    toolpath::ToolpathDiagnosticSeverity::Error,
                    std::string("Automatic hole toolpath failure: ") + failure.what()});
                return result;
            } catch (...) {
                tooling::HoleAutomaticToolpathResult result;
                result.batch.diagnostics.push_back({
                    toolpath::ToolpathDiagnosticCode::AlgorithmExecutionFailed,
                    toolpath::ToolpathDiagnosticSeverity::Error,
                    "Unknown automatic hole toolpath failure"});
                return result;
            }
        }, &holeJobNotStarted);
    toolpath::ToolpathAsyncTask<tooling::HoleAutomaticToolpathResult> task(
        std::move(future), cancellation);
    WorkflowProgressRequest progressRequest;
    progressRequest.title = QCoreApplication::translate("OpenAutoPath", "孔自动选刀");
    progressRequest.label =
        QCoreApplication::translate("OpenAutoPath", "正在按六面和刀库自动规划孔刀路…");
    progressRequest.cancellingLabel =
        QCoreApplication::translate("OpenAutoPath", "正在取消，等待当前刀具批次结束…");
    progressRequest.initialTotal = inputs.size();
    progressRequest.snapshot = [&] {
        return WorkflowProgressSnapshot{
            completed.load(std::memory_order_relaxed),
            total.load(std::memory_order_relaxed), {}};
    };
    progressRequest.queued = [&task] { return task.queued(); };
    progressRequest.ready = [&task] { return task.ready(); };
    progressRequest.cancel = [&task] { task.cancel(); };
    progressRequest.cancellationRequested = [cancellation] {
        return cancellation->load(std::memory_order_relaxed);
    };
    waitForWorkflowTask(progress, progressRequest);
    return task.get();
}

} // namespace oap::app

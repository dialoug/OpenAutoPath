// 槽腔刀路运行器：把识别选择转换为请求并协调进度和取消。
#include "app/workflows/pocket/detail/PocketToolpathRunner.h"

#include <atomic>
#include <chrono>
#include <exception>
#include <future>
#include <memory>

#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>

#include "toolpath/workflow/ToolpathAsyncExecutor.h"
#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/core/HeavyJobScheduler.h"

namespace oap::app {

tooling::PocketAutomaticToolpathResult
generateAutomaticPocketsWithProgress(
    const WorkflowProgressPresenter& progress,
    const tooling::ToolLibrary& library,
    const std::vector<toolpath::PocketRegion>& regions,
    const toolpath::ToolpathParams& params,
    const std::string& operationPrefix)
{
    // 自动模式：后台线程内用 ToolpathGenerationControl 上报进度与取消。
    auto cancellation =
        std::make_shared<std::atomic_bool>(false);
    auto future = toolpath::submitHeavyJob(
        cancellation,
        [library, regions, params, operationPrefix,
         cancellation]() mutable {
            try {
                OCC_CATCH_SIGNALS
                OSD::SetThreadLocalSignal(
                    OSD_SignalMode_Set, Standard_False);
                toolpath::ToolpathGenerationControl control;
                toolpath::attachGlobalToolpathParallelBudget(control);
                control.isCancellationRequested = [cancellation] {
                    return cancellation->load(
                        std::memory_order_relaxed);
                };
                return tooling::PocketAutomaticToolpath().generate(
                    library, regions, params,
                    operationPrefix, &control);
            } catch (const Standard_Failure& failure) {
                tooling::PocketAutomaticToolpathResult result;
                const char* message = failure.GetMessageString();
                result.error = std::string(
                    "OCCT automatic pocket toolpath failure: ") +
                    (message ? message : "OCCT failure");
                return result;
            } catch (const std::exception& failure) {
                tooling::PocketAutomaticToolpathResult result;
                result.error = std::string(
                    "automatic pocket toolpath failure: ") +
                    failure.what();
                return result;
            } catch (...) {
                tooling::PocketAutomaticToolpathResult result;
                result.error =
                    "unknown automatic pocket toolpath failure";
                return result;
            }
        }, [](toolpath::HeavyJobRejection reason) {
            tooling::PocketAutomaticToolpathResult result;
            result.cancelled = reason == toolpath::HeavyJobRejection::Cancelled;
            if (!result.cancelled)
                result.error = toolpath::heavyJobRejectionMessage(reason);
            return result;
        });
    toolpath::HeavyJobWaitGuard waitGuard(future, cancellation);

    WorkflowProgressRequest progressRequest;
    progressRequest.title = QStringLiteral("槽/腔自动选刀");
    progressRequest.label = QStringLiteral(
        "正在比较铣刀组合并生成余料刀路…");
    progressRequest.cancellingLabel = QStringLiteral(
        "正在取消，等待当前组合计算结束…");
    progressRequest.initialTotal = 1u;
    progressRequest.snapshot = [] {
        return WorkflowProgressSnapshot{0u, 1u, {}};
    };
    progressRequest.queued = [cancellation] { return toolpath::heavyJobScheduler().isPending(cancellation); };
    progressRequest.ready = [&future] {
        return future.wait_for(std::chrono::milliseconds(0)) ==
            std::future_status::ready;
    };
    progressRequest.cancel = [cancellation] {
        toolpath::cancelHeavyJob(cancellation);
    };
    progressRequest.cancellationRequested = [cancellation] {
        return cancellation->load(std::memory_order_relaxed);
    };
    waitForWorkflowTask(progress, progressRequest);
    return future.get();
}

} // namespace oap::app

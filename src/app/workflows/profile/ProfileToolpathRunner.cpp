// ProfileToolpathRunner 实现（oap_app_workflow_cam）。
// 外轮廓刀路后台调用器：协调刀库自动选刀、进度和取消，见 ProfileToolpathRunner.h。
#include "app/workflows/profile/ProfileToolpathRunner.h"

#include <atomic>
#include <chrono>
#include <exception>
#include <future>
#include <memory>
#include <optional>

#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>

#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/core/HeavyJobScheduler.h"
#include "toolpath/core/PocketToolBinding.h"
#include "tooling/adapter/AssignedToolContext.h"
#include "toolpath/workflow/profile/ProfileToolpathService.h"
#include "toolpath/workflow/ToolpathAsyncExecutor.h"

namespace oap::app {

tooling::ProfileAutomaticToolpathResult
generateProfilesWithProgress(
    const WorkflowProgressPresenter& progress,
    const tooling::ToolLibrary& library,
    const std::vector<toolpath::ProfileRegion>& regions,
    const toolpath::ToolpathParams& params,
    const std::string& operationPrefix,
    bool directContour,
    const toolpath::ProfileDirectionPlanningOptions* directionOptions,
    toolpath::ProfileDirectionPlan* directionPlan)
{
    auto cancellation = std::make_shared<std::atomic_bool>(false);
    auto plannedDirections = std::make_shared<toolpath::ProfileDirectionPlan>();
    const auto planningOptions = directionOptions
        ? std::optional<toolpath::ProfileDirectionPlanningOptions>(*directionOptions)
        : std::nullopt;
    auto future = toolpath::submitHeavyJob(
        cancellation,
        [library, regions = std::vector<toolpath::ProfileRegion>(regions), params, operationPrefix, directContour,
         cancellation, plannedDirections, planningOptions]() mutable {
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
                if (planningOptions) {
                    auto options = *planningOptions;
                    options.cancellationRequested = control.isCancellationRequested;
                    options.hasCandidateTool = [&](const auto& region) {
                        return tooling::ProfileAutomaticToolpath::hasCandidateTool(library, region, params);
                    };
                    *plannedDirections = toolpath::ProfileDirectionPlanner().plan(regions, options);
                    if (plannedDirections->cancelled) {
                        tooling::ProfileAutomaticToolpathResult result;
                        result.cancelled = true;
                        return result;
                    }
                    std::vector<toolpath::ProfileRegion> selected;
                    for (const auto index : plannedDirections->selectedRegionIndices)
                        selected.push_back(regions[index]);
                    regions = std::move(selected);
                }
                if (directContour) {
                    tooling::ProfileAutomaticToolpathResult result;
                    const auto tools = library.assignedTools();
                    if (tools.size() != 1 || params.profile.machiningMode !=
                            toolpath::ProfileMachiningMode::ContourOnly) {
                        result.error = "single-tool contour generation requires one validated tool";
                        return result;
                    }
                    const auto tool = tooling::makeToolpathToolContext(tools.front());
                    const auto effective = toolpath::bindPocketToolParameters(params, tool);
                    result.batch = toolpath::ProfileToolpathService().generate(
                        regions, effective, operationPrefix, &control);
                    result.cancelled = result.batch.cancelled ||
                        cancellation->load(std::memory_order_relaxed);
                    result.validation = result.batch.safetyValidation;
                    for (auto& operation : result.batch.operations) {
                        operation.tool = tool;
                        operation.restMachiningPreviousToolDiameter =
                            effective.clearing.restMachiningPreviousToolDiameter;
                        operation.finishPass = effective.profile.finishPass;
                    }
                    if (!result.batch.failures.empty()) {
                        result.error = result.batch.failures.front().contour.error;
                        if (result.error.empty())
                            result.error = "single-tool contour generation failed for a source region";
                    } else if (!result.batch.safetyValidationPerformed && !result.cancelled) {
                        result.error = "single-tool contour safety validation was not completed";
                    }
                    return result;
                }
                return tooling::ProfileAutomaticToolpath().generate(
                    library, regions, params,
                    operationPrefix, &control);
            } catch (const Standard_Failure& failure) {
                tooling::ProfileAutomaticToolpathResult result;
                const char* message = failure.GetMessageString();
                result.error = std::string(
                    "OCCT automatic profile toolpath failure: ") +
                    (message ? message : "OCCT failure");
                return result;
            } catch (const std::exception& failure) {
                tooling::ProfileAutomaticToolpathResult result;
                result.error = std::string(
                    "automatic profile toolpath failure: ") +
                    failure.what();
                return result;
            } catch (...) {
                tooling::ProfileAutomaticToolpathResult result;
                result.error =
                    "unknown automatic profile toolpath failure";
                return result;
            }
        }, [](toolpath::HeavyJobRejection reason) {
            tooling::ProfileAutomaticToolpathResult result;
            result.cancelled = reason == toolpath::HeavyJobRejection::Cancelled;
            if (!result.cancelled)
                result.error = toolpath::heavyJobRejectionMessage(reason);
            return result;
        });
    toolpath::HeavyJobWaitGuard waitGuard(future, cancellation);

    WorkflowProgressRequest request;
    request.title = directionOptions ? QStringLiteral("外轮廓方向规划与刀路生成")
                        : directContour ? QStringLiteral("外轮廓单刀生成")
                                  : QStringLiteral("外轮廓自动选刀");
    request.label = directionOptions ? QStringLiteral("正在验证加工方向覆盖、选择刀具并生成刀路…")
                        : directContour ? QStringLiteral("正在生成单刀外轮廓刀路并进行安全验证…")
                                  : QStringLiteral("正在比较铣刀组合并生成多刀余料刀路…");
    request.cancellingLabel = QStringLiteral(
        "正在取消，等待当前候选刀具计算结束…");
    request.initialTotal = 1u;
    request.snapshot = [] {
        return WorkflowProgressSnapshot{0u, 1u, {}};
    };
    request.queued = [cancellation] { return toolpath::heavyJobScheduler().isPending(cancellation); };
    request.ready = [&future] {
        return future.wait_for(std::chrono::milliseconds(0)) ==
            std::future_status::ready;
    };
    request.cancel = [cancellation] {
        toolpath::cancelHeavyJob(cancellation);
    };
    request.cancellationRequested = [cancellation] {
        return cancellation->load(std::memory_order_relaxed);
    };
    waitForWorkflowTask(progress, request);
    auto result = future.get();
    if (directionPlan) *directionPlan = std::move(*plannedDirections);
    return result;
}

} // namespace oap::app

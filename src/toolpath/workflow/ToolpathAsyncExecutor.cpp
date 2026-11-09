// 通过标准异步任务执行刀路服务，并把取消标志安全传递给领域算法。
//
// 模块归属:oap_toolpath_workflow(无 UI 的刀路用例编排)。
// 实现要点:捕获 OCCT 信号并将其转译为 C++ 异常,统一翻译为批内诊断
// (AlgorithmExecutionFailed),保证 get() 永不因领域异常向外抛出。
#include "toolpath/workflow/ToolpathAsyncExecutor.h"

#include <exception>

#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>

#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/core/HeavyJobScheduler.h"

namespace oap::toolpath {
namespace {

template <typename Batch>
Batch jobNotStarted(HeavyJobRejection reason)
{
    Batch result;
    result.cancelled = reason == HeavyJobRejection::Cancelled;
    result.diagnostics.push_back({
        result.cancelled ? ToolpathDiagnosticCode::GenerationCancelled
                         : ToolpathDiagnosticCode::AlgorithmExecutionFailed,
        result.cancelled ? ToolpathDiagnosticSeverity::Info
                         : ToolpathDiagnosticSeverity::Error,
        heavyJobRejectionMessage(reason)});
    return result;
}

// 提取 OCCT 异常的可读消息(为空时回退为固定文案)。
std::string occtFailureText(const Standard_Failure& failure)
{
    const char* message = failure.GetMessageString();
    return message ? message : "OCCT failure";
}

ToolpathDiagnostic asyncFailureDiagnostic(std::string message)
{
    return {ToolpathDiagnosticCode::AlgorithmExecutionFailed,
            ToolpathDiagnosticSeverity::Error, std::move(message)};
}

// 构造携带失败诊断的空槽腔批次:让异常以诊断形式沿正常结果通道返回。
PocketToolpathBatch pocketFailure(std::string message)
{
    PocketToolpathBatch result;
    result.diagnostics.push_back(asyncFailureDiagnostic(std::move(message)));
    return result;
}

// 构造携带失败诊断的空孔批次,语义同 pocketFailure。
HoleToolpathBatch holeFailure(std::string message)
{
    HoleToolpathBatch result;
    result.diagnostics.push_back(asyncFailureDiagnostic(std::move(message)));
    return result;
}

// 构造携带失败诊断的空外轮廓批次。
ProfileToolpathBatch profileFailure(std::string message)
{
    ProfileToolpathBatch result;
    result.diagnostics.push_back(asyncFailureDiagnostic(std::move(message)));
    return result;
}

} // namespace

ToolpathAsyncExecutor::ToolpathAsyncExecutor() = default;

ToolpathAsyncExecutor::ToolpathAsyncExecutor(
    std::shared_ptr<const ToolpathAlgorithmRegistry> algorithms,
    ToolpathAlgorithmSelection selection)
    : m_pocketService(algorithms, std::move(selection.pocketAlgorithmId)),
      m_holeService(algorithms, std::move(selection.holeAlgorithmId)),
      m_profileService(algorithms,
                       std::move(selection.profileAlgorithmId))
{
}

PocketToolpathAsyncTask ToolpathAsyncExecutor::generatePockets(
    std::vector<PocketRegion> regions,
    ToolpathParams params,
    std::string operationPrefix,
    ToolpathProgressCallback progress) const
{
    // 共享取消标志 + 服务副本一并转移进后台线程;control 的取消/进度
    // 回调直接绑定该共享标志,使 cancel() 能影响正在运行的算法。
    auto cancellation = std::make_shared<std::atomic_bool>(false);
    PocketToolpathService service = m_pocketService;
    auto future = submitHeavyJob(
        cancellation,
        [regions = std::move(regions), params,
         operationPrefix = std::move(operationPrefix),
          progress = std::move(progress), cancellation,
          service = std::move(service)]() mutable {
            try {
                OCC_CATCH_SIGNALS
                OSD::SetThreadLocalSignal(OSD_SignalMode_Set, Standard_False);
                ToolpathGenerationControl control;
                attachGlobalToolpathParallelBudget(control);
                control.isCancellationRequested = [cancellation] {
                    return cancellation->load(std::memory_order_relaxed);
                };
                control.reportProgress = std::move(progress);
                return service.generate(
                    regions, params, operationPrefix, &control);
            } catch (const Standard_Failure& failure) {
                return pocketFailure("OCCT pocket toolpath failure: " +
                                     occtFailureText(failure));
            } catch (const std::exception& failure) {
                return pocketFailure("Pocket toolpath failure: " +
                                     std::string(failure.what()));
            } catch (...) {
                return pocketFailure("Unknown pocket toolpath failure");
            }
        }, &jobNotStarted<PocketToolpathBatch>);
    return {std::move(future), std::move(cancellation)};
}

HoleToolpathAsyncTask ToolpathAsyncExecutor::generateHoles(
    std::vector<HoleMachiningInput> inputs,
    ToolpathParams params,
    std::optional<HoleRapidPlanningContext> rapidContext,
    std::vector<HoleMachiningStrategy> strategyOverrides,
    ToolpathProgressCallback progress) const
{
    // 与槽腔版本同构:孔输入/快速规划上下文/策略覆盖按值转移,
    // 可选参数在此展开为指针交给领域接口。
    auto cancellation = std::make_shared<std::atomic_bool>(false);
    HoleToolpathService service = m_holeService;
    auto future = submitHeavyJob(
        cancellation,
        [inputs = std::move(inputs), params,
         rapidContext = std::move(rapidContext),
         strategyOverrides = std::move(strategyOverrides),
         progress = std::move(progress), cancellation,
         service = std::move(service)]() mutable {
            try {
                OCC_CATCH_SIGNALS
                OSD::SetThreadLocalSignal(OSD_SignalMode_Set, Standard_False);
                ToolpathGenerationControl control;
                attachGlobalToolpathParallelBudget(control);
                control.isCancellationRequested = [cancellation] {
                    return cancellation->load(std::memory_order_relaxed);
                };
                control.reportProgress = std::move(progress);
                const HoleRapidPlanningContext* context = rapidContext
                    ? &*rapidContext
                    : nullptr;
                const std::vector<HoleMachiningStrategy>* overrides =
                    strategyOverrides.empty() ? nullptr : &strategyOverrides;
                return service.generate(
                    inputs, params, context, overrides, &control);
            } catch (const Standard_Failure& failure) {
                return holeFailure("OCCT hole toolpath failure: " +
                                   occtFailureText(failure));
            } catch (const std::exception& failure) {
                return holeFailure("Hole toolpath failure: " +
                                   std::string(failure.what()));
            } catch (...) {
                return holeFailure("Unknown hole toolpath failure");
            }
        }, &jobNotStarted<HoleToolpathBatch>);
    return {std::move(future), std::move(cancellation)};
}

ProfileToolpathAsyncTask ToolpathAsyncExecutor::generateProfiles(
    std::vector<ProfileRegion> regions,
    ToolpathParams params,
    std::string operationPrefix,
    ToolpathProgressCallback progress) const
{
    auto cancellation = std::make_shared<std::atomic_bool>(false);
    ProfileToolpathService service = m_profileService;
    auto future = submitHeavyJob(
        cancellation,
        [regions = std::move(regions), params,
         operationPrefix = std::move(operationPrefix),
         progress = std::move(progress), cancellation,
         service = std::move(service)]() mutable {
            try {
                OCC_CATCH_SIGNALS
                OSD::SetThreadLocalSignal(OSD_SignalMode_Set, Standard_False);
                ToolpathGenerationControl control;
                attachGlobalToolpathParallelBudget(control);
                control.isCancellationRequested = [cancellation] {
                    return cancellation->load(std::memory_order_relaxed);
                };
                control.reportProgress = std::move(progress);
                return service.generate(
                    regions, params, operationPrefix, &control);
            } catch (const Standard_Failure& failure) {
                return profileFailure("OCCT profile toolpath failure: " +
                                      occtFailureText(failure));
            } catch (const std::exception& failure) {
                return profileFailure("Profile toolpath failure: " +
                                      std::string(failure.what()));
            } catch (...) {
                return profileFailure("Unknown profile toolpath failure");
            }
        }, &jobNotStarted<ProfileToolpathBatch>);
    return {std::move(future), std::move(cancellation)};
}

} // namespace oap::toolpath

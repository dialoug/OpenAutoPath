// 模块归属:oap_toolpath_workflow(无 UI 的刀路用例编排)。
// 职责:实现孔批次并行/串行执行循环。并行度受 OCCT 精确校验的 CPU/内存
// 开销约束,通过进程共享预算分配工作线程;取消与失败用原子
// 标志 + 异常指针跨线程传递,保证要么完整返回全部批次、要么全无。
#include "toolpath/workflow/HoleBatchExecutor.h"
#include "toolpath/core/ToolpathParallel.h"

#include <algorithm>
#include <atomic>
#include <exception>
#include <mutex>
#include <thread>

#include <OSD.hxx>
#include <Standard_ErrorHandler.hxx>

namespace oap::toolpath {
HoleBatchExecutorResult HoleBatchExecutor::execute(
    const HoleBatchExecutorRequest& request,
    const ToolpathGenerationControl* control) const
{
    ToolpathGenerationControl boundedControl =
        control ? *control : ToolpathGenerationControl{};
    ensureToolpathParallelBudget(boundedControl);
    // 汇总全部孔数作为总进度,先上报 0,再在入口处响应一次取消。
    HoleBatchExecutorResult result;
    const std::size_t batchCount = request.batches.size();
    std::size_t totalTasks = 0;
    for (const HoleBatchExecutorItem& batch : request.batches)
        totalTasks += batch.inputs.size();
    if (control)
        control->progress(0, totalTasks);

    if (control && control->cancelled()) {
        result.cancelled = true;
        result.machining.cancelled = true;
        result.cancelledBatchIndex = 0;
        return result;
    }

    // —— 跨线程状态 ——
    // generated 按批次索引预分配;nextBatch 负责工作窃取式分派;
    // firstCancelled 记录首个被取消的批次(compare_exchange 只写一次);
    // failure 经互斥保护保存首个异常,供主线程重新抛出。
    std::vector<HoleToolpathBatch> generated(batchCount);
    std::vector<std::size_t> batchProgress(batchCount, 0);
    std::atomic_size_t nextBatch{0};
    std::atomic_size_t completedBatches{0};
    std::atomic_size_t firstCancelled{static_cast<std::size_t>(-1)};
    std::atomic_bool failed{false};
    std::exception_ptr failure;
    std::mutex failureMutex;
    std::mutex progressMutex;
    std::mutex callbackMutex;

    // 取消查询加锁串行化,保证 control->cancelled() 自身线程安全。
    const auto cancellationRequested = [&] {
        if (!control)
            return false;
        std::lock_guard<std::mutex> lock(callbackMutex);
        return control->cancelled();
    };
    // 单批执行:为该批构造带取消/进度回调的生成控制,调用 HoleToolpathService
    // 生成刀路;结果按索引写入 generated,并记录取消/完成计数。
    const auto runBatch = [&](std::size_t batchIndex) {
        const HoleBatchExecutorItem& batch = request.batches[batchIndex];
        ToolpathGenerationControl batchControl;
        batchControl.holeVisitOrderMode = batch.visitOrderMode;
        batchControl.holeRapidLinkMode = batch.rapidLinkMode;
        if (control) {
            batchControl.isCancellationRequested = cancellationRequested;
            batchControl.reportProgress = [&, batchIndex](
                                                std::size_t done,
                                                std::size_t /*batchTotal*/) {
                std::size_t aggregate = 0;
                {
                    std::lock_guard<std::mutex> lock(progressMutex);
                    batchProgress[batchIndex] = std::max(
                        batchProgress[batchIndex],
                        std::min(done, request.batches[batchIndex].inputs.size()));
                    for (const std::size_t value : batchProgress)
                        aggregate += value;
                }
                std::lock_guard<std::mutex> callbackLock(callbackMutex);
                control->progress(aggregate, totalTasks);
            };
        }
        generated[batchIndex] = HoleToolpathService().generate(
            batch.inputs,
            bindHoleBatchParameters(
                request.baseParams, batch.tool),
            request.rapidContext, &batch.strategies,
            control || batch.visitOrderMode ==
                           HoleVisitOrderMode::PreserveInput ||
                    batch.rapidLinkMode ==
                           HoleRapidLinkMode::CommonClearancePlane
                ? &batchControl : nullptr);
        if (generated[batchIndex].cancelled) {
            std::size_t expected = static_cast<std::size_t>(-1);
            firstCancelled.compare_exchange_strong(expected, batchIndex);
        } else {
            completedBatches.fetch_add(1, std::memory_order_relaxed);
        }
    };
    // 工作线程主体:循环从 nextBatch 取批次执行,直到取完、被取消或
    // 出现失败;OCCT 信号被转译为 C++ 异常,统一在 catch 中记录。
    const auto worker = [&] {
        ToolpathParallelScope parallelScope(&boundedControl);
        try {
            OCC_CATCH_SIGNALS
            OSD::SetThreadLocalSignal(
                OSD_SignalMode_Set, Standard_False);
            while (!failed.load(std::memory_order_relaxed)) {
                if (cancellationRequested())
                    return;
                const std::size_t batchIndex =
                    nextBatch.fetch_add(1, std::memory_order_relaxed);
                if (batchIndex >= batchCount)
                    return;
                runBatch(batchIndex);
            }
        } catch (...) {
            std::lock_guard<std::mutex> lock(failureMutex);
            if (!failure)
                failure = std::current_exception();
            failed.store(true, std::memory_order_relaxed);
        }
    };

    // 并行模式:按批次数与硬件并发度取小,并受上限约束;串行模式直接
    // 在主线程执行,避免额外线程开销。
    const std::size_t desired = request.parallel
        ? toolpathParallelismFor(batchCount) : 1u;
    ToolpathParallelLease lease(
        &boundedControl, desired > 0u ? desired - 1u : 0u, 0u);
    {
        // Reuse the caller as one worker. jthread joins already-started workers
        // if thread creation throws, before the shared lease is released.
        std::vector<std::jthread> workers;
        workers.reserve(lease.count());
        for (std::size_t index = 0; index < lease.count(); ++index)
            workers.emplace_back(worker);
        worker();
    }

    // 汇总:读取完成/取消计数;若失败则从异常指针还原错误消息。
    result.completedBatchCount =
        completedBatches.load(std::memory_order_relaxed);
    result.cancelledBatchIndex =
        firstCancelled.load(std::memory_order_relaxed);
    result.cancelled = cancellationRequested() ||
        result.cancelledBatchIndex != static_cast<std::size_t>(-1);
    if (failure) {
        try {
            std::rethrow_exception(failure);
        } catch (const std::exception& value) {
            result.error = value.what();
        } catch (...) {
            result.error = "unknown hole batch execution failure";
        }
    }
    // 全有或全无:一旦取消/出错,丢弃所有已生成的批次与诊断。
    if (result.cancelled || !result.error.empty()) {
        result.batches.clear();
        result.machining = {};
        result.machining.cancelled = result.cancelled;
        result.diagnostics.clear();
        return result;
    }

    // 成功路径:按批次索引顺序组装结果,并聚合加工摘要与诊断。
    result.batches.reserve(batchCount);
    for (std::size_t index = 0; index < batchCount; ++index) {
        HoleBatchExecutorItemResult item;
        item.batchIndex = index;
        item.logicalId = request.batches[index].logicalId;
        item.generated = std::move(generated[index]);
        appendHoleMachiningSummary(
            result.machining, item.generated.machining);
        result.diagnostics.insert(
            result.diagnostics.end(),
            item.generated.diagnostics.begin(),
            item.generated.diagnostics.end());
        result.batches.push_back(std::move(item));
    }
    return result;
}

} // namespace oap::toolpath

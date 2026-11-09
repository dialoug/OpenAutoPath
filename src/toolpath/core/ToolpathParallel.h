// 槽腔刀路共享调度的并行工具（oap_toolpath_core）。
// 提供 CPU 并行度上限 toolpathHardwareParallelism、进程级并行预算
// ToolpathParallelBudget 与租约 ToolpathParallelLease，以及封装 OSD_Parallel
// 的 toolpathParallelFor / toolpathParallelForWorkers 稳定索引并行入口。
// 嵌套的候选/区域/层任务统一从全局预算借用额外工作槽，避免多个同时运行
// 的 CAM 作业各自占满全部逻辑 CPU。
#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <cerrno>
#include <exception>
#include <mutex>
#include <thread>
#include <utility>

#include <OSD_Parallel.hxx>

#include "toolpath/core/ToolpathExecution.h"

namespace oap::toolpath {

// 槽腔刀路共享调度的 CPU 上限。hardware_concurrency() 返回当前进程可见的
// 逻辑处理器数；平台无法报告时仍保留两个执行槽，避免完全失去并行能力。
inline std::size_t toolpathHardwareParallelism() noexcept
{
    static const std::size_t parallelism = [] {
        const unsigned int hardwareThreads =
            std::thread::hardware_concurrency();
        const std::size_t hardware = std::max<std::size_t>(
            1u, hardwareThreads == 0u ? 2u : hardwareThreads);
        // Optional process-level cap for benchmarks or machines shared with
        // other workloads. The default uses every available logical CPU.
        const char* setting = std::getenv("OAP_CAM_MAX_THREADS");
        if (setting && *setting >= '0' && *setting <= '9') {
            char* end = nullptr;
            errno = 0;
            const unsigned long limit = std::strtoul(setting, &end, 10);
            if (errno == 0 && end && *end == '\0' && limit > 0)
                return std::min(hardware, static_cast<std::size_t>(limit));
        }
        return hardware;
    }();
    return parallelism;
}

inline std::size_t toolpathParallelismFor(std::size_t taskCount) noexcept
{
    return std::min(taskCount, toolpathHardwareParallelism());
}

// Process-wide budget for additional CAM workers. Root workflow threads are
// not leased; every nested candidate/region/layer task must borrow from this
// shared pool so simultaneous CAM jobs cannot each claim all logical CPUs.
class ToolpathParallelBudget {
public:
    explicit ToolpathParallelBudget(std::size_t capacity) noexcept
        : m_capacity(capacity), m_available(capacity)
    {
    }

    std::size_t tryAcquire(std::size_t requested) noexcept
    {
        std::size_t available =
            m_available.load(std::memory_order_relaxed);
        while (available > 0u && requested > 0u) {
            const std::size_t granted = std::min(requested, available);
            if (m_available.compare_exchange_weak(
                    available, available - granted,
                    std::memory_order_acq_rel,
                    std::memory_order_relaxed)) {
                return granted;
            }
        }
        return 0u;
    }

    void release(std::size_t count) noexcept
    {
        std::size_t available =
            m_available.load(std::memory_order_relaxed);
        while (true) {
            const std::size_t restored = std::min(
                m_capacity, available + count);
            if (m_available.compare_exchange_weak(
                    available, restored,
                    std::memory_order_release,
                    std::memory_order_relaxed)) {
                return;
            }
        }
    }

    std::size_t capacity() const noexcept { return m_capacity; }
    std::size_t available() const noexcept
    {
        return m_available.load(std::memory_order_acquire);
    }

private:
    std::size_t m_capacity = 0u;
    std::atomic_size_t m_available{0u};
};

inline ToolpathParallelBudget& globalToolpathParallelBudget() noexcept
{
    static ToolpathParallelBudget budget(
        toolpathHardwareParallelism() > 1u
            ? toolpathHardwareParallelism() - 1u : 0u);
    return budget;
}

inline void attachGlobalToolpathParallelBudget(
    ToolpathGenerationControl& control)
{
    ToolpathParallelBudget* budget = &globalToolpathParallelBudget();
    control.tryAcquireParallelWorkers =
        [budget](std::size_t requested) {
            return budget->tryAcquire(requested);
        };
    control.releaseParallelWorkers =
        [budget](std::size_t count) { budget->release(count); };
}

// The synchronous call tree may include legacy validators without a control
// argument. Carry only the current budget through that tree; never retain this
// borrowed pointer beyond the scope. Parallel helpers install it on each worker
// and restore the worker's previous value, including on exceptions.
namespace parallel_detail {
inline const ToolpathGenerationControl*& currentControlSlot() noexcept
{
    static thread_local const ToolpathGenerationControl* control = nullptr;
    return control;
}
} // namespace parallel_detail

inline const ToolpathGenerationControl* currentToolpathParallelControl() noexcept
{
    return parallel_detail::currentControlSlot();
}

inline bool hasToolpathParallelBudget(const ToolpathGenerationControl* control) noexcept
{
    return control && control->tryAcquireParallelWorkers && control->releaseParallelWorkers;
}

class ToolpathParallelScope {
public:
    explicit ToolpathParallelScope(const ToolpathGenerationControl* control) noexcept
        : m_previous(currentToolpathParallelControl())
    {
        if (hasToolpathParallelBudget(control))
            parallel_detail::currentControlSlot() = control;
    }

    ~ToolpathParallelScope() { parallel_detail::currentControlSlot() = m_previous; }
    ToolpathParallelScope(const ToolpathParallelScope&) = delete;
    ToolpathParallelScope& operator=(const ToolpathParallelScope&) = delete;

private:
    const ToolpathGenerationControl* m_previous;
};

// Library and test callers receive the same process-wide limit as the UI.
// Explicit caller budgets (including a zero-worker budget) remain authoritative.
inline void ensureToolpathParallelBudget(ToolpathGenerationControl& control)
{
    if (hasToolpathParallelBudget(&control))
        return;
    const auto* inherited = currentToolpathParallelControl();
    if (hasToolpathParallelBudget(inherited)) {
        control.tryAcquireParallelWorkers = inherited->tryAcquireParallelWorkers;
        control.releaseParallelWorkers = inherited->releaseParallelWorkers;
    } else {
        attachGlobalToolpathParallelBudget(control);
    }
}

// 从上层共享调度器借用额外工作槽。当前调用线程不计入借用数量，
// 因此可用并行度始终为 1 + count()。没有安装预算回调的独立调用者
// 先继承同步调用树中的预算；独立调用者按 fallbackAdditionalWorkers 上限
// 向进程共享池借用，不再产生未计费的本地工作槽。
class ToolpathParallelLease {
public:
    ToolpathParallelLease(
        const ToolpathGenerationControl* control,
        std::size_t requestedAdditionalWorkers,
        std::size_t fallbackAdditionalWorkers)
        : m_control(hasToolpathParallelBudget(control)
                        ? control : currentToolpathParallelControl())
    {
        if (requestedAdditionalWorkers == 0u)
            return;
        if (m_control && m_control->tryAcquireParallelWorkers &&
            m_control->releaseParallelWorkers) {
            m_count = std::min(
                requestedAdditionalWorkers,
                m_control->tryAcquireParallelWorkers(
                    requestedAdditionalWorkers));
        } else {
            m_globalBudget = &globalToolpathParallelBudget();
            m_count = m_globalBudget->tryAcquire(std::min(
                requestedAdditionalWorkers, fallbackAdditionalWorkers));
        }
    }

    ToolpathParallelLease(const ToolpathParallelLease&) = delete;
    ToolpathParallelLease& operator=(const ToolpathParallelLease&) = delete;

    ~ToolpathParallelLease()
    {
        release();
    }

    void retain(std::size_t count)
    {
        if (count >= m_count)
            return;
        const std::size_t returned = m_count - count;
        m_count = count;
        if (m_globalBudget)
            m_globalBudget->release(returned);
        else if (m_control)
            m_control->releaseParallelWorkers(returned);
    }

    void release() { retain(0u); }

    std::size_t count() const noexcept { return m_count; }
    std::size_t parallelism() const noexcept { return 1u + m_count; }

private:
    const ToolpathGenerationControl* m_control = nullptr;
    ToolpathParallelBudget* m_globalBudget = nullptr;
    std::size_t m_count = 0u;
};

// 对一个已受 ToolpathParallelLease 限制的小批次执行稳定索引并行。
// 结果必须由调用方写入按 index 分配的独立槽，并在返回后顺序合并。
template <typename Function>
void toolpathParallelFor(std::size_t count, Function&& function)
{
    if (count <= 1u) {
        if (count == 1u)
            function(0u);
        return;
    }
    const auto* inherited = currentToolpathParallelControl();
    std::mutex failureMutex;
    std::exception_ptr failure;
    OSD_Parallel::For(
        0, static_cast<Standard_Integer>(count),
        [&](Standard_Integer index) {
            ToolpathParallelScope scope(inherited);
            // An external pool may execute us on a thread currently running an
            // unrelated scope. Propagation of an absent parent must clear that
            // scope for this invocation, then restore it on return.
            parallel_detail::currentControlSlot() = inherited;
            try {
                function(static_cast<std::size_t>(index));
            } catch (...) {
                // Join all work before propagating: callers may own borrowed
                // callback contexts and partially populated indexed results.
                std::lock_guard<std::mutex> lock(failureMutex);
                if (!failure)
                    failure = std::current_exception();
            }
        });
    if (failure)
        std::rethrow_exception(failure);
}

// 用固定数量的 worker 消费任意数量的索引，避免把任务总数误当成
// 允许并发度。每个索引仍只写自己的结果槽，最终合并顺序由调用方决定。
template <typename Function>
void toolpathParallelForWorkers(
    std::size_t count,
    std::size_t workerCount,
    Function&& function)
{
    workerCount = std::max<std::size_t>(
        1u, std::min(workerCount, count));
    if (workerCount == 1u) {
        for (std::size_t index = 0u; index < count; ++index)
            function(index);
        return;
    }

    std::atomic_size_t next{0u};
    toolpathParallelFor(workerCount, [&](std::size_t) {
        while (true) {
            const std::size_t index =
                next.fetch_add(1u, std::memory_order_relaxed);
            if (index >= count)
                return;
            function(index);
        }
    });
}

// Admission and stable-index execution for callers that do not already hold a
// lease. Explicit control wins over the synchronous scope, then the process pool.
// Failure to obtain slots is an immediate serial fallback, never a wait.
template <typename Function>
void toolpathParallelForBudgeted(
    std::size_t count,
    Function&& function,
    const ToolpathGenerationControl* control = nullptr)
{
    if (count == 0u)
        return;
    ToolpathParallelScope scope(control);
    const std::size_t requested = toolpathParallelismFor(count) - 1u;
    ToolpathParallelLease lease(control, requested, requested);
    toolpathParallelForWorkers(count, lease.parallelism(),
                              std::forward<Function>(function));
}

} // namespace oap::toolpath

#include "PocketPlanningScheduler.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#endif

#include "toolpath/core/ToolpathParallel.h"

namespace oap::tooling::detail {
namespace {

// Kept as implementation constants so benchmark builds can compare the
// policies without adding application settings or changing geometry inputs.
constexpr bool kDynamicMemoryFeedback = true;
constexpr bool kTrackRegionMemory = true;
constexpr bool kBorrowNestedWorkers = true;
constexpr std::uint64_t kPlanningMemoryReserveBytes = 2ull * 1024ull * 1024ull * 1024ull;
constexpr std::uint64_t kEstimatedParallelRegionBytes = 1ull * 1024ull * 1024ull * 1024ull;
constexpr std::uint64_t kMinimumObservedRegionBytes = 256ull * 1024ull * 1024ull;
constexpr std::uint64_t kMaximumObservedRegionBytes = 4ull * 1024ull * 1024ull * 1024ull;
constexpr std::uint64_t kMemorySchedulingSafetyNumerator = 9u;
constexpr std::uint64_t kMemorySchedulingSafetyDenominator = 8u;

std::uint64_t availablePhysicalMemoryBytes()
{
#ifdef _WIN32
    MEMORYSTATUSEX status{};
    status.dwLength = sizeof(status);
    if (GlobalMemoryStatusEx(&status) != FALSE)
        return static_cast<std::uint64_t>(status.ullAvailPhys);
#endif
    return 0u;
}

std::uint64_t currentProcessWorkingSetBytes()
{
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS counters{};
    counters.cb = sizeof(counters);
    if (K32GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)) != FALSE)
        return static_cast<std::uint64_t>(counters.WorkingSetSize);
#endif
    return 0u;
}

oap::toolpath::ToolpathGenerationControl parentControl(
    const oap::toolpath::ToolpathGenerationControl* control)
{
    auto result = control ? *control : oap::toolpath::ToolpathGenerationControl{};
    oap::toolpath::ensureToolpathParallelBudget(result);
    return result;
}

} // namespace

struct RegionMemoryTracker::Impl {
    const std::uint64_t startBytes = kTrackRegionMemory ? currentProcessWorkingSetBytes() : 0u;
    std::atomic<std::uint64_t> peakBytes{startBytes};
    std::atomic_bool stopped{false};
    std::mutex mutex;
    std::condition_variable stopRequested;
    std::thread sampler;

    Impl()
    {
        if (startBytes == 0u)
            return;
        sampler = std::thread([this] {
            while (!stopped.load(std::memory_order_relaxed)) {
                sample();
                std::unique_lock<std::mutex> lock(mutex);
                stopRequested.wait_for(lock, std::chrono::milliseconds(100), [this] {
                    return stopped.load(std::memory_order_relaxed);
                });
            }
            sample();
        });
    }

    ~Impl()
    {
        stopped.store(true, std::memory_order_relaxed);
        stopRequested.notify_one();
        if (sampler.joinable())
            sampler.join();
    }

    void sample()
    {
        if (!kTrackRegionMemory)
            return;
        const auto current = currentProcessWorkingSetBytes();
        auto peak = peakBytes.load(std::memory_order_relaxed);
        while (current > peak && !peakBytes.compare_exchange_weak(
                   peak, current, std::memory_order_relaxed)) {}
    }
};

RegionMemoryTracker::RegionMemoryTracker() : m_impl(std::make_unique<Impl>()) {}
RegionMemoryTracker::~RegionMemoryTracker() = default;

std::uint64_t RegionMemoryTracker::peakGrowthBytes()
{
    m_impl->sample();
    const auto peak = m_impl->peakBytes.load(std::memory_order_relaxed);
    return peak > m_impl->startBytes ? peak - m_impl->startBytes : 0u;
}

struct PocketPlanningScheduler::Impl {
    oap::toolpath::ToolpathGenerationControl parent;
    oap::toolpath::ToolpathParallelLease lease;
    const std::size_t grantedParallelism;
    oap::toolpath::ToolpathGenerationControl local;
    PocketPlanningScheduleAudit audit;
    std::mutex callbackMutex;
    std::mutex mutex;
    std::condition_variable ready;
    std::size_t next = 0u;
    std::size_t count = 0u;
    std::size_t activeRegions = 0u;
    std::size_t tokensInUse = 0u;
    bool started = false;
    std::exception_ptr failure;

    explicit Impl(const oap::toolpath::ToolpathGenerationControl* control)
        : parent(parentControl(control)),
          lease(&parent, oap::toolpath::toolpathHardwareParallelism() - 1u, 0u),
          grantedParallelism(lease.parallelism()), local(parent)
    {
        audit.cpuParallelism = oap::toolpath::toolpathHardwareParallelism();
        audit.availableMemoryBytes = availablePhysicalMemoryBytes();
        audit.memoryParallelism = audit.cpuParallelism;
        if (audit.availableMemoryBytes > 0u) {
            const auto usable = audit.availableMemoryBytes > kPlanningMemoryReserveBytes
                ? audit.availableMemoryBytes - kPlanningMemoryReserveBytes : 0u;
            audit.memoryParallelism = std::max<std::size_t>(1u, std::min<std::size_t>(
                audit.cpuParallelism, static_cast<std::size_t>(usable / kEstimatedParallelRegionBytes)));
        }
        audit.initialParallelism = std::min(grantedParallelism, audit.memoryParallelism);
        audit.finalParallelism = audit.initialParallelism;
        audit.estimatedRegionBytes = kEstimatedParallelRegionBytes;
        if (local.reportProgress) {
            const auto report = local.reportProgress;
            local.reportProgress = [this, report](std::size_t completed, std::size_t total) {
                std::lock_guard<std::mutex> lock(callbackMutex);
                report(completed, total);
            };
        }
        if (local.reportStatus) {
            const auto report = local.reportStatus;
            local.reportStatus = [this, report](const oap::toolpath::ToolpathExecutionStatus& status) {
                std::lock_guard<std::mutex> lock(callbackMutex);
                report(status);
            };
        }
        local.tryAcquireParallelWorkers = [this](std::size_t requested) {
            std::lock_guard<std::mutex> lock(mutex);
            // Queued regions have priority; nested work borrows only after
            // every region has started, and never exceeds the parent grant.
            if (!kBorrowNestedWorkers || next < count || tokensInUse >= audit.finalParallelism)
                return std::size_t{0u};
            const auto acquired = std::min(requested, audit.finalParallelism - tokensInUse);
            tokensInUse += acquired;
            audit.borrowedTokenCount += acquired;
            audit.peakTokenConcurrency = std::max(audit.peakTokenConcurrency, tokensInUse);
            return acquired;
        };
        local.releaseParallelWorkers = [this](std::size_t released) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                tokensInUse -= std::min(released, tokensInUse);
            }
            ready.notify_all();
        };
    }

    // Called while holding mutex. Working-set growth is an estimate across
    // overlapping regions, not an attribution of exact region allocations.
    void updateConcurrency(std::uint64_t peakGrowthBytes, std::size_t overlap)
    {
        if (!kDynamicMemoryFeedback)
            return;
        if (peakGrowthBytes > 0u) {
            const auto observed = std::clamp(peakGrowthBytes / std::max<std::size_t>(1u, overlap),
                kMinimumObservedRegionBytes, kMaximumObservedRegionBytes);
            audit.maximumObservedRegionBytes = std::max(audit.maximumObservedRegionBytes, observed);
            audit.estimatedRegionBytes = (audit.estimatedRegionBytes * 3u + observed) / 4u;
        }
        const auto available = availablePhysicalMemoryBytes();
        std::size_t target = audit.finalParallelism;
        if (available > 0u) {
            const auto headroom = available > kPlanningMemoryReserveBytes
                ? available - kPlanningMemoryReserveBytes : 0u;
            const auto guardedEstimate = audit.estimatedRegionBytes *
                kMemorySchedulingSafetyNumerator / kMemorySchedulingSafetyDenominator;
            const auto additional = guardedEstimate > 0u
                ? static_cast<std::size_t>(headroom / guardedEstimate) : 0u;
            target = std::max<std::size_t>(1u,
                std::min(grantedParallelism, activeRegions + additional));
        }
        if (target != audit.finalParallelism) {
            audit.finalParallelism = target;
            ++audit.adjustmentCount;
        }
    }

    void worker(const RegionTask& task)
    {
        oap::toolpath::ToolpathParallelScope scope(&local);
        while (true) {
            std::size_t index, overlap;
            {
                std::unique_lock<std::mutex> lock(mutex);
                ready.wait(lock, [this] {
                    return next >= count || tokensInUse < audit.finalParallelism;
                });
                if (next >= count)
                    return;
                index = next++;
                ++activeRegions;
                ++tokensInUse;
                overlap = activeRegions;
                audit.peakRegionConcurrency = std::max(audit.peakRegionConcurrency, activeRegions);
                audit.peakTokenConcurrency = std::max(audit.peakTokenConcurrency, tokensInUse);
                if (activeRegions > 1u)
                    ++audit.parallelStarts;
            }
            std::uint64_t observed = 0u;
            std::exception_ptr error;
            try {
                observed = task(index, local, grantedParallelism);
            } catch (...) {
                error = std::current_exception();
            }
            {
                std::lock_guard<std::mutex> lock(mutex);
                --activeRegions;
                --tokensInUse;
                updateConcurrency(observed, overlap);
                if (error) {
                    if (!failure) failure = error;
                    next = count;
                }
            }
            ready.notify_all();
        }
    }

    void run(std::size_t regionCount, const RegionTask& task)
    {
        if (started)
            throw std::logic_error("pocket planning scheduler may only run once");
        started = true;
        count = regionCount;
        const auto workersNeeded = std::min(grantedParallelism, count);
        std::vector<std::thread> workers;
        workers.reserve(workersNeeded > 0u ? workersNeeded - 1u : 0u);
        try {
            for (std::size_t index = 1; index < workersNeeded; ++index)
                workers.emplace_back([this, &task] { worker(task); });
            worker(task);
        } catch (...) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                next = count;
            }
            ready.notify_all();
            for (auto& thread : workers) thread.join();
            throw;
        }
        for (auto& thread : workers) thread.join();
        // No region or nested task can reference the subdivision now. Return
        // its parent slots before ordered merge/final validation, which can
        // borrow directly from the caller's budget in the restored root scope.
        lease.release();
        if (failure) std::rethrow_exception(failure);
    }
};

PocketPlanningScheduler::PocketPlanningScheduler(
    const oap::toolpath::ToolpathGenerationControl* control)
    : m_impl(std::make_unique<Impl>(control)) {}
PocketPlanningScheduler::~PocketPlanningScheduler() = default;

void PocketPlanningScheduler::run(std::size_t count, const RegionTask& task)
{
    m_impl->run(count, task);
}

const PocketPlanningScheduleAudit& PocketPlanningScheduler::audit() const noexcept
{
    return m_impl->audit;
}

} // namespace oap::tooling::detail

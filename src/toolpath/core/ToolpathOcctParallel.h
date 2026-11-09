// Admission for OCCT algorithms exposing only a boolean internal-parallel flag.
// No process-wide backend or thread-pool configuration is changed here.
#pragma once

#include <atomic>
#include <cstddef>
#include <mutex>
#include <optional>

#include <OSD_Parallel.hxx>
#include <OSD_ThreadPool.hxx>

#include "toolpath/core/ToolpathParallel.h"

namespace oap::toolpath {

// Zero means that internal parallelism cannot be safely admitted. The native
// OCCT backend's pool size is a conservative bound (including the caller).
// External/TBB backends expose no equivalent per-operation limit through this
// API, so boolean-only operations remain serial on those backends.
inline std::size_t toolpathOcctParallelism()
{
    if (toolpathHardwareParallelism() <= 1u || !OSD_Parallel::ToUseOcctThreads())
        return 0u;
    return static_cast<std::size_t>(
        std::max(1, OSD_ThreadPool::DefaultPool()->NbThreads()));
}

struct ToolpathOcctParallelStatistics {
    std::size_t requested = 0u;
    std::size_t admitted = 0u;
    std::size_t backendDenied = 0u;
    std::size_t budgetDenied = 0u;
    std::size_t busy = 0u;
};

namespace detail {
struct OcctParallelAdmissionState {
    std::mutex gate;
    std::atomic_size_t requested{0u}, admitted{0u}, backendDenied{0u},
        budgetDenied{0u}, busy{0u};
};
inline OcctParallelAdmissionState& occtParallelAdmissionState()
{
    static OcctParallelAdmissionState state;
    return state;
}
inline bool& occtParallelAdmissionActive()
{
    static thread_local bool active = false;
    return active;
}
} // namespace detail

// Cumulative diagnostics; snapshots/deltas do not reset another job's counters.
inline ToolpathOcctParallelStatistics toolpathOcctParallelStatistics()
{
    const auto& state = detail::occtParallelAdmissionState();
    return {state.requested.load(std::memory_order_relaxed),
            state.admitted.load(std::memory_order_relaxed),
            state.backendDenied.load(std::memory_order_relaxed),
            state.budgetDenied.load(std::memory_order_relaxed),
            state.busy.load(std::memory_order_relaxed)};
}

class ToolpathOcctParallelLease {
public:
    explicit ToolpathOcctParallelLease(
        bool requested, const ToolpathGenerationControl* control = nullptr)
        : m_gate(detail::occtParallelAdmissionState().gate, std::defer_lock)
    {
        if (!requested)
            return;
        auto& state = detail::occtParallelAdmissionState();
        state.requested.fetch_add(1u, std::memory_order_relaxed);
        const std::size_t parallelism = toolpathOcctParallelism();
        if (parallelism <= 1u || parallelism > toolpathHardwareParallelism()) {
            state.backendDenied.fetch_add(1u, std::memory_order_relaxed);
            return;
        }
        // Do not recursively try_lock a mutex owned by this thread.
        if (detail::occtParallelAdmissionActive() || !m_gate.try_lock()) {
            state.busy.fetch_add(1u, std::memory_order_relaxed);
            return;
        }
        // Budget callbacks are caller-owned and may synchronously request
        // another OCCT operation. Guard before invoking either acquisition or
        // the release callback for a partial grant, not just during Build().
        detail::occtParallelAdmissionActive() = true;
        const std::size_t additional = parallelism - 1u;
        try {
            m_lease.emplace(control, additional, additional);
        } catch (...) {
            // This object's destructor will not run if construction fails.
            m_gate.unlock();
            detail::occtParallelAdmissionActive() = false;
            throw;
        }
        if (m_lease->count() != additional) {
            state.budgetDenied.fetch_add(1u, std::memory_order_relaxed);
            m_lease.reset();
            m_gate.unlock();
            detail::occtParallelAdmissionActive() = false;
            return;
        }
        m_enabled = true;
        state.admitted.fetch_add(1u, std::memory_order_relaxed);
    }

    ~ToolpathOcctParallelLease()
    {
        if (m_enabled) {
            // Keep the reentrancy guard set while returning caller-owned slots.
            m_lease.reset();
            m_gate.unlock();
            detail::occtParallelAdmissionActive() = false;
        }
    }
    ToolpathOcctParallelLease(const ToolpathOcctParallelLease&) = delete;
    ToolpathOcctParallelLease& operator=(const ToolpathOcctParallelLease&) = delete;
    bool enabled() const noexcept { return m_enabled; }

private:
    std::unique_lock<std::mutex> m_gate;
    std::optional<ToolpathParallelLease> m_lease;
    bool m_enabled = false;
};

} // namespace oap::toolpath

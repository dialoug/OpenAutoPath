// Target-private execution policy for automatic pocket planning. Geometry and
// tool-selection decisions stay in PocketAutomaticToolpath.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

#include "toolpath/core/ToolpathExecution.h"

namespace oap::tooling::detail {

class RegionMemoryTracker {
public:
    RegionMemoryTracker();
    ~RegionMemoryTracker();
    RegionMemoryTracker(const RegionMemoryTracker&) = delete;
    RegionMemoryTracker& operator=(const RegionMemoryTracker&) = delete;
    std::uint64_t peakGrowthBytes();

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

struct PocketPlanningScheduleAudit {
    std::size_t cpuParallelism = 1u;
    std::size_t memoryParallelism = 1u;
    std::size_t initialParallelism = 1u;
    std::uint64_t availableMemoryBytes = 0u;
    std::size_t peakRegionConcurrency = 0u;
    std::size_t peakTokenConcurrency = 0u;
    std::size_t finalParallelism = 1u;
    std::size_t adjustmentCount = 0u;
    std::size_t parallelStarts = 0u;
    std::size_t borrowedTokenCount = 0u;
    std::uint64_t estimatedRegionBytes = 0u;
    std::uint64_t maximumObservedRegionBytes = 0u;
};

// A single invocation owns one parent lease. Region workers and their nested
// candidates/layers subdivide only that grant; the caller supplies one slot.
// run() joins every worker before returning or rethrowing an unexpected failure.
class PocketPlanningScheduler {
public:
    explicit PocketPlanningScheduler(
        const oap::toolpath::ToolpathGenerationControl* control);
    ~PocketPlanningScheduler();
    PocketPlanningScheduler(const PocketPlanningScheduler&) = delete;
    PocketPlanningScheduler& operator=(const PocketPlanningScheduler&) = delete;

    // The callback writes only its indexed output and returns its observed
    // process-working-set growth. Domain failures remain the caller's concern.
    using RegionTask = std::function<std::uint64_t(
        std::size_t index,
        const oap::toolpath::ToolpathGenerationControl& control,
        std::size_t maximumParallelStages)>;
    void run(std::size_t count, const RegionTask& task);

    // Read before run() or after all workers have joined, never concurrently.
    const PocketPlanningScheduleAudit& audit() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace oap::tooling::detail

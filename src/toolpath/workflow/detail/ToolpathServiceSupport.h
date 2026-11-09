// Private service lifecycle helpers. Domain filtering, cancellation policy and
// validation ordering stay in the individual workflow services.
#pragma once

#include <algorithm>
#include <chrono>
#include <mutex>
#include <string>
#include <utility>

#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/core/ToolpathParamNormalizer.h"

namespace oap::toolpath::workflow_detail {

using ServiceClock = std::chrono::steady_clock;

inline double elapsedMilliseconds(ServiceClock::time_point start)
{
    return std::chrono::duration<double, std::milli>(
               ServiceClock::now() - start).count();
}

// Evaluate validity before transferring the diagnostics: ok() reads that
// vector, so consulting the moved-from normalization loses blocking errors.
template <typename Batch>
bool normalizeBatchParameters(Batch& batch, const ToolpathParams& params,
                              bool warnRoundedResidual = false)
{
    ToolpathParamNormalization normalization =
        ToolpathParamNormalizer().normalize(params);
    const bool valid = normalization.ok();
    batch.effectiveParams = normalization.effective;
    batch.diagnostics = std::move(normalization.diagnostics);
    if (valid && warnRoundedResidual && batch.effectiveParams.tool.cornerRadius > 0.0) {
        batch.diagnostics.push_back({ToolpathDiagnosticCode::RoundedCutterResidual,
            ToolpathDiagnosticSeverity::Warning,
            toolpathDiagnosticDefaultMessageZh(ToolpathDiagnosticCode::RoundedCutterResidual)});
    }
    return valid;
}

inline ToolpathDiagnostic unavailableAlgorithmDiagnostic(
    const char* domain, const std::string& algorithmId)
{
    return {ToolpathDiagnosticCode::AlgorithmUnavailable,
            ToolpathDiagnosticSeverity::Error,
            std::string(domain) + " algorithm is not registered: " + algorithmId};
}

inline ToolpathDiagnostic failedAlgorithmDiagnostic(
    const char* domain, const char* error, int sourceIndex = -1)
{
    return {ToolpathDiagnosticCode::AlgorithmExecutionFailed,
            ToolpathDiagnosticSeverity::Error,
            std::string(domain) + (error
                ? " algorithm failed: " + std::string(error)
                : " algorithm failed with an unknown exception"),
            {}, sourceIndex};
}

// Own the worker lease and serialized callback wrappers for one synchronous
// indexed region pass. The caller's control must outlive this context; all
// workers must join before it is destroyed. No budget or callback is detached.
class RegionExecutionContext {
public:
    RegionExecutionContext(
        const ToolpathGenerationControl& control, std::size_t taskCount)
        : m_control(&control),
          m_lease(&control, additionalWorkers(taskCount),
                  additionalWorkers(taskCount)),
          m_workerCount(std::min(toolpathParallelismFor(taskCount),
                                 m_lease.parallelism())),
          m_parallelControl(control)
    {
        if (m_parallelControl.reportProgress) {
            const auto reportProgress = m_parallelControl.reportProgress;
            m_parallelControl.reportProgress =
                [this, reportProgress](std::size_t completed, std::size_t total) {
                    std::lock_guard<std::mutex> lock(m_callbackMutex);
                    reportProgress(completed, total);
                };
        }
        if (m_parallelControl.reportStatus) {
            const auto reportStatus = m_parallelControl.reportStatus;
            m_parallelControl.reportStatus =
                [this, reportStatus](const ToolpathExecutionStatus& status) {
                    std::lock_guard<std::mutex> lock(m_callbackMutex);
                    reportStatus(status);
                };
        }
    }

    RegionExecutionContext(const RegionExecutionContext&) = delete;
    RegionExecutionContext& operator=(const RegionExecutionContext&) = delete;

    std::size_t workerCount() const noexcept { return m_workerCount; }

    const ToolpathGenerationControl* control() const noexcept
    {
        return m_workerCount > 1u ? &m_parallelControl : m_control;
    }

private:
    static std::size_t additionalWorkers(std::size_t taskCount) noexcept
    {
        const std::size_t desired = toolpathParallelismFor(taskCount);
        return desired > 0u ? desired - 1u : 0u;
    }

    const ToolpathGenerationControl* m_control;
    ToolpathParallelLease m_lease;
    const std::size_t m_workerCount;
    std::mutex m_callbackMutex;
    ToolpathGenerationControl m_parallelControl;
};

} // namespace oap::toolpath::workflow_detail

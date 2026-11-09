// ProfileToolpathService 实现（oap_toolpath_workflow）。
// 区域级并行生成、安全校验与批次物化，见 ProfileToolpathService.h。
#include "toolpath/workflow/profile/ProfileToolpathService.h"

#include <algorithm>
#include <exception>
#include <iterator>
#include <optional>
#include <utility>

#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/pocket/planning/PocketLayerSlicer.h"
#include "toolpath/profile/program/ProfileProgramAdapter.h"
#include "toolpath/profile/validation/ProfileSafetyValidator.h"
#include "toolpath/workflow/detail/ToolpathServiceSupport.h"

namespace oap::toolpath {
namespace {

using Clock = workflow_detail::ServiceClock;
using workflow_detail::elapsedMilliseconds;

// Both direct generation and evaluation materialization publish one indexed
// result only after safety validation. The caller merges these slots in order.
struct ProfileRegionResult {
    std::optional<ProfileToolpathOperation> operation;
    std::optional<ProfileGenerationFailure> failure;
    std::vector<ToolpathDiagnostic> diagnostics;
    ToolpathValidationReport safetyValidation;
    double algorithmMilliseconds = 0.0;
    double safetyValidationMilliseconds = 0.0;
    std::size_t candidateGeometryCacheHitCount = 0u;
    std::size_t candidateGeometryCacheMissCount = 0u;
    int parallelLayerBatchCount = 0;
    int parallelLayerCount = 0;
    std::size_t layerPeakParallelism = 1u;
    bool cancelled = false;
};

// Cancellation retains elapsed time but does not publish the validator's
// partial diagnostics/report. Successful validation consumes the operation;
// failed validation consumes its contour into the per-region failure slot.
void validateRegionOperation(
    const ProfileRegion& region,
    ProfileToolpathOperation& operation,
    const ToolpathParams& params,
    const ToolpathGenerationControl& control,
    ProfileRegionResult& result)
{
    const int sourceIndex = operation.sourceRegionIndex;
    const Clock::time_point safetyStart = Clock::now();
    ProfileSafetyValidationResult safety = ProfileSafetyValidator().validate(
        region, operation, params, control.isCancellationRequested);
    result.safetyValidationMilliseconds = elapsedMilliseconds(safetyStart);
    for (ToolpathDiagnostic& diagnostic : safety.diagnostics)
        diagnostic.sourceIndex = sourceIndex;
    if (safety.cancelled || control.cancelled()) {
        result.cancelled = true;
        return;
    }
    const bool safetyOk = safety.report.ok();
    result.safetyValidation = std::move(safety.report);
    result.diagnostics.insert(
        result.diagnostics.end(),
        std::make_move_iterator(safety.diagnostics.begin()),
        std::make_move_iterator(safety.diagnostics.end()));
    if (!safetyOk) {
        operation.contour.ok = false;
        operation.contour.failureCode =
            ProfileContourFailureCode::SafetyValidationFailed;
        const auto error = std::find_if(
            result.diagnostics.rbegin(), result.diagnostics.rend(),
            [sourceIndex](const ToolpathDiagnostic& diagnostic) {
                return diagnostic.sourceIndex == sourceIndex &&
                    diagnostic.severity == ToolpathDiagnosticSeverity::Error;
            });
        operation.contour.error = error != result.diagnostics.rend()
            ? error->message : "profile toolpath failed safety validation";
        result.failure.emplace(ProfileGenerationFailure{
            sourceIndex, std::move(operation.contour)});
        return;
    }
    result.operation.emplace(std::move(operation));
}

} // namespace

ProfileToolpathService::ProfileToolpathService()
    : ProfileToolpathService(defaultToolpathAlgorithmRegistry())
{
}

ProfileToolpathService::ProfileToolpathService(
    std::shared_ptr<const ToolpathAlgorithmRegistry> algorithms,
    std::string algorithmId)
    : m_algorithms(algorithms ? std::move(algorithms)
                              : defaultToolpathAlgorithmRegistry()),
      m_algorithmId(std::move(algorithmId))
{
}

ToolpathProgram ProfileToolpathBatch::toProgram() const
{
    return makeToolpathProgram(operations);
}

ProfileToolpathBatch ProfileToolpathService::generate(
    const std::vector<ProfileRegion>& regions,
    const ToolpathParams& params,
    const std::string& operationPrefix,
    const ToolpathGenerationControl* control) const
{
    ProfileToolpathBatch batch;
    const Clock::time_point totalStart = Clock::now();
    ToolpathGenerationControl batchControl =
        control ? *control : ToolpathGenerationControl{};
    ensureToolpathParallelBudget(batchControl);
    ToolpathParallelScope parallelScope(&batchControl);
    if (!batchControl.pocketMaterialSectionCache) {
        batchControl.pocketMaterialSectionCache =
            std::make_shared<PocketMaterialSectionCache>();
    }
    const ToolpathGenerationControl* effectiveControl = &batchControl;
    PocketMaterialSectionCache& sectionCache =
        *batchControl.pocketMaterialSectionCache;
    const std::size_t cacheHitsBefore = sectionCache.hitCount();
    const std::size_t cacheMissesBefore = sectionCache.missCount();
    const auto finish = [&]() -> ProfileToolpathBatch {
        batch.metrics.materialSectionCacheHitCount =
            sectionCache.hitCount() - cacheHitsBefore;
        batch.metrics.materialSectionCacheMissCount =
            sectionCache.missCount() - cacheMissesBefore;
        batch.metrics.totalMilliseconds =
            elapsedMilliseconds(totalStart);
        return std::move(batch);
    };
    batch.algorithmId = m_algorithmId;
    const bool parametersValid =
        workflow_detail::normalizeBatchParameters(batch, params, true);
    batch.operations.reserve(regions.size());
    batch.failures.reserve(regions.size());
    batch.safetyValidationPerformed =
        !effectiveControl->profileEvaluationOnly;
    if (!parametersValid)
        return finish();

    const IProfileToolpathAlgorithm* algorithm =
        m_algorithms->profileAlgorithm(batch.algorithmId);
    if (!algorithm) {
        batch.diagnostics.push_back(workflow_detail::unavailableAlgorithmDiagnostic(
            "profile", batch.algorithmId));
        return finish();
    }

    std::vector<ProfileRegionResult> generated(regions.size());
    workflow_detail::RegionExecutionContext regionExecution(
        *effectiveControl, regions.size());
    const std::size_t workerCount = regionExecution.workerCount();
    batch.metrics.regionPeakParallelism = std::max(
        batch.metrics.regionPeakParallelism, workerCount);
    if (workerCount > 1u) {
        ++batch.metrics.parallelRegionBatchCount;
        batch.metrics.parallelRegionOperationCount +=
            static_cast<int>(regions.size());
    }
    // Nested layer work can borrow unused slots from the same budget.
    const ToolpathGenerationControl* regionControl = regionExecution.control();

    toolpathParallelForWorkers(
        regions.size(), workerCount,
        [&](std::size_t index) {
        ProfileRegionResult& regionResult = generated[index];
        if (regionControl->cancelled()) {
            regionResult.cancelled = true;
            return;
        }
        ProfileContourResult contour;
        const Clock::time_point algorithmStart = Clock::now();
        try {
            contour = algorithm->generate(
                regions[index], batch.effectiveParams,
                regionControl);
        } catch (const std::exception& exception) {
            contour.error = exception.what();
            contour.failureCode =
                ProfileContourFailureCode::OffsetFailed;
        } catch (...) {
            contour.error = "unknown profile algorithm exception";
            contour.failureCode =
                ProfileContourFailureCode::OffsetFailed;
        }
        regionResult.algorithmMilliseconds =
            elapsedMilliseconds(algorithmStart);
        regionResult.candidateGeometryCacheHitCount =
            contour.candidateGeometryCacheHitCount;
        regionResult.candidateGeometryCacheMissCount =
            contour.candidateGeometryCacheMissCount;
        regionResult.parallelLayerBatchCount =
            contour.parallelLayerBatchCount;
        regionResult.parallelLayerCount = contour.parallelLayerCount;
        regionResult.layerPeakParallelism =
            contour.layerPeakParallelism;
        if (contour.cancelled || regionControl->cancelled()) {
            regionResult.cancelled = true;
            return;
        }
        if (!contour.ok) {
            regionResult.failure.emplace(ProfileGenerationFailure{
                static_cast<int>(index), std::move(contour)});
            return;
        }
        if (contour.restMachining && contour.noRestMaterial) {
            regionResult.failure.emplace(ProfileGenerationFailure{
                static_cast<int>(index), std::move(contour)});
            return;
        }

        const ProfileRegion& region = regions[index];
        ProfileToolpathOperation operation;
        operation.operationId = operationPrefix +
            std::to_string(index + 1);
        operation.algorithmId = batch.algorithmId;
        operation.sourceRegionIndex = static_cast<int>(index);
        operation.sourceFaceIds = region.source.sourceFaceIds;
        operation.rootFaceId = region.source.rootFaceId;
        operation.rootWireId = region.source.rootWireId;
        operation.outwardToolAxis = gp_Vec(region.openDir);
        operation.restMachiningPreviousToolDiameter =
            batch.effectiveParams.clearing
                .restMachiningPreviousToolDiameter;
        operation.finishPass = batch.effectiveParams.profile.finishPass;
        operation.contour = std::move(contour);

        if (regionControl->profileEvaluationOnly) {
            regionResult.operation.emplace(std::move(operation));
            return;
        }

        validateRegionOperation(
            region, operation, batch.effectiveParams, *regionControl,
            regionResult);
    });

    for (ProfileRegionResult& regionResult : generated) {
        batch.cancelled = batch.cancelled || regionResult.cancelled;
        batch.metrics.algorithmMilliseconds +=
            regionResult.algorithmMilliseconds;
        batch.metrics.safetyValidationMilliseconds +=
            regionResult.safetyValidationMilliseconds;
        batch.metrics.candidateGeometryCacheHitCount +=
            regionResult.candidateGeometryCacheHitCount;
        batch.metrics.candidateGeometryCacheMissCount +=
            regionResult.candidateGeometryCacheMissCount;
        batch.metrics.parallelLayerBatchCount +=
            regionResult.parallelLayerBatchCount;
        batch.metrics.parallelLayerCount +=
            regionResult.parallelLayerCount;
        batch.metrics.layerPeakParallelism = std::max(
            batch.metrics.layerPeakParallelism,
            regionResult.layerPeakParallelism);
        batch.safetyValidation.append(
            std::move(regionResult.safetyValidation));
        batch.diagnostics.insert(
            batch.diagnostics.end(),
            std::make_move_iterator(regionResult.diagnostics.begin()),
            std::make_move_iterator(regionResult.diagnostics.end()));
        if (regionResult.failure)
            batch.failures.push_back(std::move(*regionResult.failure));
        if (regionResult.operation)
            batch.operations.push_back(std::move(*regionResult.operation));
    }
    if (batch.cancelled) {
        batch.operations.clear();
        batch.failures.clear();
    }
    return finish();
}

ProfileToolpathBatch ProfileToolpathService::materializeEvaluated(
    const std::vector<ProfileRegion>& regions,
    ProfileToolpathBatch evaluation,
    const ToolpathGenerationControl* control) const
{
    ProfileToolpathBatch batch;
    const Clock::time_point totalStart = Clock::now();
    const bool evaluationCancelled = evaluation.cancelled;
    batch.algorithmId = std::move(evaluation.algorithmId);
    batch.effectiveParams = evaluation.effectiveParams;
    batch.diagnostics = std::move(evaluation.diagnostics);
    batch.failures = std::move(evaluation.failures);
    batch.metrics = evaluation.metrics;
    batch.safetyValidationPerformed = true;
    ToolpathGenerationControl materializationControl =
        control ? *control : ToolpathGenerationControl{};
    ensureToolpathParallelBudget(materializationControl);
    ToolpathParallelScope parallelScope(&materializationControl);
    materializationControl.profileEvaluationOnly = false;
    const ToolpathGenerationControl* effectiveControl =
        &materializationControl;
    const auto finish = [&]() -> ProfileToolpathBatch {
        const double milliseconds = elapsedMilliseconds(totalStart);
        batch.metrics.materializationMilliseconds += milliseconds;
        batch.metrics.totalMilliseconds += milliseconds;
        return std::move(batch);
    };

    if (evaluationCancelled || effectiveControl->cancelled()) {
        batch.cancelled = true;
        batch.failures.clear();
        return finish();
    }
    batch.operations.reserve(evaluation.operations.size());
    batch.failures.reserve(
        evaluation.failures.size() + evaluation.operations.size());
    std::vector<ProfileRegionResult> materialized(evaluation.operations.size());
    workflow_detail::RegionExecutionContext regionExecution(
        *effectiveControl, evaluation.operations.size());
    const std::size_t workerCount = regionExecution.workerCount();
    batch.metrics.regionPeakParallelism = std::max(
        batch.metrics.regionPeakParallelism, workerCount);
    if (workerCount > 1u) {
        ++batch.metrics.parallelRegionBatchCount;
        batch.metrics.parallelRegionOperationCount +=
            static_cast<int>(evaluation.operations.size());
    }
    // Nested layer work can borrow unused slots from the same budget.
    const ToolpathGenerationControl* regionControl = regionExecution.control();

    toolpathParallelForWorkers(
        evaluation.operations.size(), workerCount,
        [&](std::size_t operationIndex) {
        ProfileRegionResult& regionResult =
            materialized[operationIndex];
        if (regionControl->cancelled()) {
            regionResult.cancelled = true;
            return;
        }
        ProfileToolpathOperation operation = std::move(
            evaluation.operations[operationIndex]);
        const int sourceIndex = operation.sourceRegionIndex;
        if (sourceIndex < 0 ||
            static_cast<std::size_t>(sourceIndex) >= regions.size()) {
            operation.contour.ok = false;
            operation.contour.failureCode =
                ProfileContourFailureCode::InvalidRegion;
            operation.contour.error =
                "evaluated profile operation has an invalid source region";
            regionResult.failure.emplace(ProfileGenerationFailure{
                sourceIndex, std::move(operation.contour)});
            return;
        }

        const Clock::time_point algorithmStart = Clock::now();
        operation.contour = ProfileContour().materializeEvaluation(
            regions[static_cast<std::size_t>(sourceIndex)],
            batch.effectiveParams, std::move(operation.contour),
            regionControl);
        regionResult.algorithmMilliseconds =
            elapsedMilliseconds(algorithmStart);
        if (operation.contour.cancelled ||
            regionControl->cancelled()) {
            regionResult.cancelled = true;
            return;
        }
        if (!operation.contour.ok) {
            regionResult.failure.emplace(ProfileGenerationFailure{
                sourceIndex, std::move(operation.contour)});
            return;
        }

        validateRegionOperation(
            regions[static_cast<std::size_t>(sourceIndex)], operation,
            batch.effectiveParams, *regionControl, regionResult);
    });

    for (ProfileRegionResult& regionResult : materialized) {
        batch.cancelled = batch.cancelled || regionResult.cancelled;
        batch.metrics.algorithmMilliseconds +=
            regionResult.algorithmMilliseconds;
        batch.metrics.safetyValidationMilliseconds +=
            regionResult.safetyValidationMilliseconds;
        batch.safetyValidation.append(
            std::move(regionResult.safetyValidation));
        batch.diagnostics.insert(
            batch.diagnostics.end(),
            std::make_move_iterator(regionResult.diagnostics.begin()),
            std::make_move_iterator(regionResult.diagnostics.end()));
        if (regionResult.failure)
            batch.failures.push_back(std::move(*regionResult.failure));
        if (regionResult.operation)
            batch.operations.push_back(std::move(*regionResult.operation));
    }
    if (batch.cancelled) {
        batch.operations.clear();
        batch.failures.clear();
    }
    return finish();
}

} // namespace oap::toolpath

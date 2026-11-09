// Regenerate hole Job operations and publish validated runtime results.
#include "manufacturing/hole/HoleMachiningJobService.h"
#include <algorithm>
#include <set>
#include <utility>
#include <QDateTime>
#include "manufacturing/hole/HoleJobRebuildReview.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "toolpath/hole/HoleToolpathValidator.h"
#include "toolpath/workflow/HoleBatchExecutor.h"

#include "manufacturing/hole/detail/HoleJobTypes.h"
#include "manufacturing/hole/detail/HoleJobEnablement.h"
#include "manufacturing/hole/detail/HoleJobActiveIpw.h"

namespace oap::manufacturing {
using namespace hole_job_detail;

// 把局部索引映射回全局孔段索引，并在成功后恢复可复用的 IPW 前缀。

toolpath::HoleToolpathBatch HoleMachiningJobService::regenerate(
    MachiningJob& job,
    const toolpath::HoleRapidPlanningContext* rapidContext,
    const toolpath::ToolpathGenerationControl* control) const
{
    // 在副本上重建，只有成功时才提交回调用方 Job；当前 IPW 前缀先行保存。
    toolpath::HoleToolpathBatch result;
    const ActiveIpwPlan activeIpw = captureActiveIpwPlan(job);
    MachiningJob candidate = job;
    MachiningJobDefinition& definition = candidate.definition();
    result.algorithmId = definition.algorithmId;
    result.effectiveParams = definition.holeParameters;
    const std::vector<toolpath::HoleMachiningInput>& machiningInputs =
        definition.recognizedHoles;
    candidate.clearHoleResult();
    if (!candidate.validationErrors().empty())
        return result;

    std::vector<HoleOperationDefinition*> operations;
    for (HoleOperationDefinition& operation : definition.holeOperations) {
        if (operation.enabled)
            operations.push_back(&operation);
    }
    std::stable_sort(
        operations.begin(), operations.end(),
        [](const HoleOperationDefinition* lhs,
           const HoleOperationDefinition* rhs) {
            return lhs->sequence < rhs->sequence;
        });
    // 按全局 sequence 稳定排序后，把每个工序打包成一个批执行项：
    // 目标未就绪（入口/装夹未解析）的工序整体置为 Error 并跳过。

    struct JobBatchBinding {
        HoleOperationDefinition* operation = nullptr;
        const SetupDefinition* setup = nullptr;
        toolpath::ToolpathToolContext tool;
    };
    std::vector<JobBatchBinding> bindings;
    toolpath::HoleBatchExecutorRequest executionRequest;
    executionRequest.baseParams = definition.holeParameters;
    executionRequest.rapidContext = rapidContext;
    executionRequest.parallel = false;
    executionRequest.batches.reserve(operations.size());
    bindings.reserve(operations.size());

    for (HoleOperationDefinition* operationDefinition : operations) {
        const SetupDefinition* setup =
            candidate.setupById(operationDefinition->setupId);
        const JobToolController* controller =
            candidate.toolControllerById(
                operationDefinition->toolControllerId);
        if (!setup || !controller) {
            operationDefinition->state = JobOperationState::Error;
            continue;
        }

        toolpath::ToolpathToolContext operationTool = controller->tool;
        toolpath::HoleBatchExecutorItem item;
        item.logicalId = operationDefinition->id;
        item.tool = operationTool;
        item.inputs.reserve(operationDefinition->targets.size());
        item.strategies.reserve(operationDefinition->targets.size());
        bool unresolvedTarget = false;
        for (const HoleTargetReference& target :
             operationDefinition->targets) {
            const toolpath::HoleMachiningInput& input = target.hole;
            if (!input.ready()) {
                unresolvedTarget = true;
                toolpath::ToolpathDiagnostic diagnostic;
                diagnostic.code = input.mappingState ==
                        toolpath::HoleMachiningMappingState::UnresolvedEntry
                    ? toolpath::ToolpathDiagnosticCode::HoleOpeningUnresolved
                    : toolpath::ToolpathDiagnosticCode::HoleOutsideSixFace;
                diagnostic.severity =
                    toolpath::ToolpathDiagnosticSeverity::Error;
                diagnostic.operationId = operationDefinition->id;
                diagnostic.sourceIndex = target.sourceHoleIndex;
                diagnostic.message =
                    "unresolved Job target cannot be regenerated";
                result.diagnostics.push_back(std::move(diagnostic));
                break;
            }
            item.inputs.push_back(input);
            item.strategies.push_back(target.strategy);
        }
        if (unresolvedTarget) {
            operationDefinition->state = JobOperationState::Error;
            continue;
        }
        executionRequest.batches.push_back(std::move(item));
        bindings.push_back(
            JobBatchBinding{operationDefinition, setup, operationTool});
    }

    // 批执行：整批失败/取消时按错误码统一标记状态；
    // 单个工序的成功结果还要经过重建审查与局部索引重映射。

    toolpath::HoleBatchExecutorResult execution =
        toolpath::HoleBatchExecutor().execute(
            executionRequest, control);
    std::set<std::string> generatedSetups;
    toolpath::ToolpathValidationReport reviewWarnings;
    if (!execution.error.empty()) {
        toolpath::addToolpathIssue(
            reviewWarnings,
            toolpath::ToolpathIssueSeverity::Error,
            "hole_batch_execution_failed",
            execution.error);
        for (JobBatchBinding& binding : bindings)
            binding.operation->state = JobOperationState::Error;
    }
    if (execution.cancelled) {
        result.cancelled = true;
        result.machining.cancelled = true;
        if (execution.cancelledBatchIndex < bindings.size()) {
            bindings[execution.cancelledBatchIndex]
                .operation->state = JobOperationState::Dirty;
        }
    } else if (execution.error.empty()) {
        result.machining = std::move(execution.machining);
        result.diagnostics.insert(
            result.diagnostics.end(),
            execution.diagnostics.begin(), execution.diagnostics.end());
        for (oap::toolpath::HoleBatchExecutorItemResult& item :
             execution.batches) {
            if (item.batchIndex >= bindings.size())
                continue;
            JobBatchBinding& binding = bindings[item.batchIndex];
            HoleOperationDefinition* operationDefinition =
                binding.operation;
            const SetupDefinition* setup = binding.setup;
            toolpath::HoleToolpathBatch& generated = item.generated;

            HoleJobRebuildReview rebuildReview = reviewHoleJobRebuild(
                generated, operationDefinition->id);
            operationDefinition->state =
                generated.machining.operations.empty()
                ? JobOperationState::Error
                : (rebuildReview.needsReview()
                       ? JobOperationState::NeedsReview
                       : generatedState(generated));
            if (operationDefinition->state != JobOperationState::Error &&
                (!setup->valid || !setup->confirmation.confirmed)) {
                operationDefinition->state =
                    JobOperationState::NeedsReview;
            }

            reviewWarnings.append(std::move(rebuildReview.warnings));
            const std::size_t acceptedOperationCountBefore =
                result.machining.operations.size();
            int omittedOperationCount = 0;
            for (std::size_t localIndex = 0;
                 localIndex < generated.machining.operations.size();
                 ++localIndex) {
                toolpath::HoleMachiningOperation generatedOperation =
                    generated.machining.operations[localIndex];
                if (rebuildReview.omits(generatedOperation.operationId)) {
                    ++omittedOperationCount;
                    continue;
                }
                const int localSource =
                    generatedOperation.sourceHoleIndex;
                if (localSource >= 0 &&
                    localSource < static_cast<int>(
                        operationDefinition->targets.size())) {
                    // 把生成结果的局部目标索引重映射为 Job 全局孔段索引，
                    // 使覆盖审计与后续诊断指向稳定来源。
                    const HoleTargetReference& target =
                        operationDefinition->targets[
                            static_cast<std::size_t>(localSource)];
                    generatedOperation.sourceHoleIndex =
                        target.sourceHoleIndex;
                    std::vector<int> localCovered =
                        generatedOperation.coveredSourceHoleIndices;
                    localCovered.push_back(localSource);
                    generatedOperation.coveredSourceHoleIndices.clear();
                    for (const int coveredLocalIndex : localCovered) {
                        if (coveredLocalIndex < 0 ||
                            coveredLocalIndex >= static_cast<int>(
                                operationDefinition->targets.size())) {
                            continue;
                        }
                        const HoleTargetReference& coveredTarget =
                            operationDefinition->targets[
                                static_cast<std::size_t>(
                                    coveredLocalIndex)];
                        generatedOperation.coveredSourceHoleIndices.insert(
                            generatedOperation.coveredSourceHoleIndices.end(),
                            coveredTarget.coveredSourceHoleIndices.begin(),
                            coveredTarget.coveredSourceHoleIndices.end());
                        generatedOperation.coveredSourceHoleIndices.push_back(
                            coveredTarget.sourceHoleIndex);
                    }
                    std::sort(
                        generatedOperation.coveredSourceHoleIndices.begin(),
                        generatedOperation.coveredSourceHoleIndices.end());
                    generatedOperation.coveredSourceHoleIndices.erase(
                        std::unique(
                            generatedOperation
                                .coveredSourceHoleIndices.begin(),
                            generatedOperation
                                .coveredSourceHoleIndices.end()),
                        generatedOperation
                            .coveredSourceHoleIndices.end());
                }
                generatedOperation.operationId =
                    operationDefinition->id + "-" +
                    std::to_string(localIndex + 1);
                generatedOperation.algorithmId =
                    definition.algorithmId;
                generatedOperation.setupIndex = setup->setupIndex;
                generatedOperation.setupFace = setup->face;
                generatedOperation.phase = operationDefinition->phase;
                generatedOperation.tool = binding.tool;
                result.machining.operations.push_back(
                    std::move(generatedOperation));
            }
            result.machining.skippedCount += omittedOperationCount;
            if (result.machining.operations.size() >
                acceptedOperationCountBefore) {
                generatedSetups.insert(setup->id);
            }
        }
    }

    result.machining.setupCount = static_cast<int>(generatedSetups.size());
    // 重建完成后统一校验、做覆盖审计并构建生产安全证据；
    if (!result.cancelled) {
        result.validation = toolpath::HoleToolpathValidator().validate(
            result.machining.operations, result.effectiveParams);
        result.validation.append(std::move(reviewWarnings));
        if (!definition.recognizedHoles.empty()) {
            result.coverage = toolpath::auditHoleCoverage(
                machiningInputs,
                result.machining.operations,
                definition.recognizedPhysicalHoleCount,
                result.machining.skippedCount,
                result.diagnostics,
                result.machining.diagnostics);
            if (result.machining.operations.empty() &&
                result.coverage.complete()) {
                result.validation = {};
            }
        }
        result.productionSafety =
            toolpath::buildHoleProductionSafetyEvidence(
                result.effectiveParams, rapidContext, result.machining);
    }
    definition.modifiedAt =
        QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)
            .toStdString();
    const std::string fingerprint = MachiningJobStorage::
        holeExecutionFingerprint(definition).toStdString();
    candidate.setHoleResult(result, fingerprint);
    if (!result.cancelled) {
        // 重建成功：恢复保存的 IPW 前缀；失败则保持原 Job 不动。
        if (restoreActiveIpwPlan(candidate, activeIpw))
            job = std::move(candidate);
    }
    return result;
}

} // namespace oap::manufacturing

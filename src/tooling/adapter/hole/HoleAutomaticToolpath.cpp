// 自动孔加工门面：把选刀/装夹规划结果逐批交给孔刀路服务并合并诊断。
// 属于 oap_toolpath_adapter 对外自动入口;依赖 toolpath/workflow 的
// HoleBatchExecutor 实际生成刀路,并负责把批次内局部 source 索引重映射回
// 识别数组的全局孔段索引(供覆盖审计与 UI 使用)。
#include "tooling/adapter/hole/HoleAutomaticToolpath.h"

#include <algorithm>
#include <exception>
#include <iterator>
#include <sstream>
#include <unordered_map>

#include "toolpath/core/ToolpathParamNormalizer.h"
#include "toolpath/hole/HoleBatchExecution.h"
#include "toolpath/hole/HoleToolpathValidator.h"
#include "toolpath/workflow/HoleBatchExecutor.h"

namespace oap::tooling {
namespace {

std::string operationPrefix(const HoleToolBatch& batch)
{
    std::ostringstream value;
    value << "HOLE_S" << batch.setupIndex + 1 << '_'
          << oap::toolpath::holeMachiningPhaseName(batch.phase)
          << "_T" << batch.tool.toolNumber;
    return value.str();
}

} // namespace

HoleAutomaticToolpathResult HoleAutomaticToolpath::generate(
    const ToolLibrary& library,
    const std::vector<oap::toolpath::HoleMachiningInput>& inputs,
    const oap::toolpath::ToolpathParams& params,
    const oap::toolpath::HoleRapidPlanningContext* rapidContext,
    const oap::toolpath::ToolpathGenerationControl* control) const
{
    // 先归一化参数得到 effective(含诊断),再做自动选刀;归一化诊断放在
    // 结果诊断最前面,保证用户先看到参数层问题。
    oap::toolpath::ToolpathParamNormalization normalization =
        oap::toolpath::ToolpathParamNormalizer().normalize(params);
    HoleToolSelectionPlan selection = HoleToolSelectionPlanner().plan(
        library, inputs, normalization.effective);
    HoleAutomaticToolpathResult result = generateFromPlan(
        std::move(selection), inputs, normalization.effective,
        rapidContext, control);
    result.batch.diagnostics.insert(
        result.batch.diagnostics.begin(),
        normalization.diagnostics.begin(), normalization.diagnostics.end());
    return result;
}

HoleAutomaticToolpathResult HoleAutomaticToolpath::generateFromPlan(
    HoleToolSelectionPlan selection,
    const std::vector<oap::toolpath::HoleMachiningInput>& inputs,
    const oap::toolpath::ToolpathParams& params,
    const oap::toolpath::HoleRapidPlanningContext* rapidContext,
    const oap::toolpath::ToolpathGenerationControl* control) const
{
    HoleAutomaticToolpathResult result;
    result.batch.algorithmId = "hole.auto_tool_library";
    oap::toolpath::ToolpathParamNormalization normalization =
        oap::toolpath::ToolpathParamNormalizer().normalize(params);
    result.batch.effectiveParams = normalization.effective;
    result.batch.diagnostics = std::move(normalization.diagnostics);
    result.selection = std::move(selection);
    result.batch.diagnostics.insert(
        result.batch.diagnostics.end(),
        result.selection.diagnostics.begin(),
        result.selection.diagnostics.end());

    result.batch.machining.skippedCount =
        result.selection.skippedPhysicalHoleCount;
    result.batch.machining.setupCount = result.selection.setupCount;

    // 把选刀批次翻译为执行请求:每个 HoleToolBatch 对应一个
    // HoleBatchExecutorItem,其 inputs/strategies 来自 batch 内各任务。
    oap::toolpath::HoleBatchExecutorRequest executionRequest;
    executionRequest.baseParams = result.batch.effectiveParams;
    executionRequest.rapidContext = rapidContext;
    executionRequest.parallel = true;
    executionRequest.batches.reserve(result.selection.batches.size());
    for (std::size_t batchIndex = 0;
         batchIndex < result.selection.batches.size(); ++batchIndex) {
        const HoleToolBatch& plannedBatch =
            result.selection.batches[batchIndex];
        oap::toolpath::HoleBatchExecutorItem item;
        item.logicalId = operationPrefix(plannedBatch);
        item.tool = plannedBatch.tool;
        item.visitOrderMode = plannedBatch.visitOrderMode;
        item.rapidLinkMode = plannedBatch.rapidLinkMode;
        item.inputs.reserve(plannedBatch.tasks.size());
        item.strategies.reserve(plannedBatch.tasks.size());
        for (const HoleToolTask& task : plannedBatch.tasks) {
            item.inputs.push_back(task.hole);
            item.strategies.push_back(task.strategy);
        }
        executionRequest.batches.push_back(std::move(item));
    }

    oap::toolpath::HoleBatchExecutorResult execution =
        oap::toolpath::HoleBatchExecutor().execute(
            executionRequest, control);
    if (!execution.error.empty()) {
        oap::toolpath::addToolpathIssue(
            result.batch.validation,
            oap::toolpath::ToolpathIssueSeverity::Error,
            "hole_batch_execution_failed",
            execution.error);
        return result;
    }

    const std::size_t batchCount = result.selection.batches.size();
    std::vector<oap::toolpath::HoleToolpathBatch> generatedBatches(batchCount);
    for (oap::toolpath::HoleBatchExecutorItemResult& item :
         execution.batches) {
        if (item.batchIndex < generatedBatches.size())
            generatedBatches[item.batchIndex] = std::move(item.generated);
    }
    result.batch.machining = std::move(execution.machining);
    result.batch.diagnostics.insert(
        result.batch.diagnostics.end(),
        execution.diagnostics.begin(), execution.diagnostics.end());
    result.batch.machining.skippedCount +=
        result.selection.skippedPhysicalHoleCount;
    result.batch.machining.setupCount = result.selection.setupCount;

    // 逐批回填生成的刀路操作:每个操作带上前缀 Operation ID,并把批次局部
    // sourceHoleIndex 映射回全局孔段索引;同前缀批次出现两次时追加 _B 序号
    // 消歧(例如同一 T 号刀具复用的自动批次)。
    bool cancelled = execution.cancelled;
    std::unordered_map<std::string, int> prefixOccurrences;
    for (std::size_t batchIndex = 0; batchIndex < batchCount; ++batchIndex) {
        const HoleToolBatch& plannedBatch =
            result.selection.batches[batchIndex];
        const oap::toolpath::HoleToolpathBatch& generated =
            generatedBatches[batchIndex];
        cancelled = cancelled || generated.cancelled;

        std::string prefix = operationPrefix(plannedBatch);
        const int occurrence = ++prefixOccurrences[prefix];
        if (occurrence > 1)
            prefix += "_B" + std::to_string(occurrence);
        for (std::size_t operationIndex = 0;
             operationIndex < generated.machining.operations.size(); ++operationIndex) {
            oap::toolpath::HoleMachiningOperation operation =
                generated.machining.operations[operationIndex];
            const int localSource = operation.sourceHoleIndex;
            if (localSource >= 0 &&
                localSource < static_cast<int>(plannedBatch.tasks.size())) {
                const HoleToolTask& task =
                    plannedBatch.tasks[static_cast<std::size_t>(localSource)];
                operation.sourceHoleIndex =
                    static_cast<int>(task.sourceHoleIndex);
                operation.coveredSourceHoleIndices.clear();
                operation.coveredSourceHoleIndices.reserve(
                    task.coveredSourceHoleIndices.size());
                for (const std::size_t sourceIndex :
                     task.coveredSourceHoleIndices) {
                    operation.coveredSourceHoleIndices.push_back(
                        static_cast<int>(sourceIndex));
                }
            }
            operation.operationId = prefix + "_" +
                std::to_string(operationIndex + 1);
            operation.algorithmId = result.batch.algorithmId;
            operation.setupIndex = plannedBatch.setupIndex;
            operation.setupFace = plannedBatch.setupFace;
            operation.phase = plannedBatch.phase;
            if (operation.phase ==
                oap::toolpath::HoleMachiningPhase::CenterSpot) {
                operation.minimumCuttingEdgeLength = 0.0;
            }
            operation.tool = plannedBatch.tool;
            result.batch.machining.operations.push_back(std::move(operation));
        }
    }
    if (cancelled) {
        result.batch.cancelled = true;
        result.batch.machining.cancelled = true;
        result.batch.machining.operations.clear();
    }

    // appendHoleMachiningSummary 同时复制临时批次操作之外的统计；操作由上面的
    // source 索引重映射流程单独追加，因此此处移除错误的临时追加风险。
    // （appendHoleMachiningSummary 本身不复制 operations。）
    result.batch.machining.setupCount = result.selection.setupCount;
    result.batch.machining.cancelled = result.batch.cancelled;
    if (!result.batch.cancelled) {
        result.batch.validation = oap::toolpath::HoleToolpathValidator()
            .validate(result.batch.machining.operations,
                      result.batch.effectiveParams);
    }
    result.batch.coverage = oap::toolpath::auditHoleCoverage(
        inputs, result.batch.machining.operations,
        result.selection.physicalHoleCount,
        result.batch.machining.skippedCount,
        result.batch.diagnostics,
        result.batch.machining.diagnostics);
    // 无待加工输入且覆盖完整时允许空结果。
    if (result.batch.machining.operations.empty() &&
        result.batch.coverage.complete()) {
        result.batch.validation = {};
    }
    result.batch.productionSafety =
        oap::toolpath::buildHoleProductionSafetyEvidence(
            result.batch.effectiveParams, rapidContext,
            result.batch.machining);
    return result;
}

} // namespace oap::tooling

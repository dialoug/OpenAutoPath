// 槽腔刀路应用服务：归一化参数、选择算法、处理取消并执行槽腔专用验证。
//
// 模块归属:oap_toolpath_workflow 的槽腔服务子目录。编排链路为:参数归一化
// → 逐区域调用算法(异常/取消兜底,失败区域单独记录)→ 结构验证 →
// 并行安全验证 → 汇总统计。材料截面缓存跨区域共享以加速重复几何求交。
#include "toolpath/workflow/pocket/PocketToolpathService.h"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <iterator>
#include <utility>

#include "toolpath/core/ToolpathParallel.h"

#include "toolpath/pocket/planning/PocketLayerSlicer.h"
#include "toolpath/pocket/program/PocketProgramAdapter.h"
#include "toolpath/pocket/validation/PocketSafetyValidator.h"
#include "toolpath/pocket/validation/PocketToolpathValidator.h"
#include "toolpath/workflow/detail/ToolpathServiceSupport.h"

namespace oap::toolpath {

namespace {

using Clock = workflow_detail::ServiceClock;
using workflow_detail::elapsedMilliseconds;

} // namespace

PocketToolpathService::PocketToolpathService()
    : PocketToolpathService(defaultToolpathAlgorithmRegistry())
{
}

PocketToolpathService::PocketToolpathService(
    std::shared_ptr<const ToolpathAlgorithmRegistry> algorithms,
    std::string algorithmId)
    : m_algorithms(algorithms ? std::move(algorithms)
                              : defaultToolpathAlgorithmRegistry()),
      m_algorithmId(std::move(algorithmId))
{
}

// 通过 regionOutcomes 把区域索引映射到成功操作;越界或失败返回 nullptr。
const PocketToolpathOperation* PocketToolpathBatch::operationForRegion(
    std::size_t regionIndex) const noexcept
{
    if (regionIndex >= regionOutcomes.size())
        return nullptr;
    const int operationIndex = regionOutcomes[regionIndex].operationIndex;
    if (operationIndex < 0 ||
        operationIndex >= static_cast<int>(operations.size()))
        return nullptr;
    return &operations[operationIndex];
}

// 区域清料结果查询:先查成功操作,失败则回退到 failures 中对应记录。
const PocketClearingResult* PocketToolpathBatch::clearingForRegion(
    std::size_t regionIndex) const noexcept
{
    if (const PocketToolpathOperation* operation =
            operationForRegion(regionIndex))
        return &operation->clearing;
    if (regionIndex >= regionOutcomes.size())
        return nullptr;
    const int failureIndex = regionOutcomes[regionIndex].failureIndex;
    if (failureIndex < 0 || failureIndex >= static_cast<int>(failures.size()))
        return nullptr;
    return &failures[failureIndex].clearing;
}

ToolpathProgram PocketToolpathBatch::toProgram() const
{
    return makeToolpathProgram(operations);
}

PocketToolpathBatch PocketToolpathService::generate(
    const std::vector<PocketRegion>& regions,
    const ToolpathParams& params,
    const std::string& operationPrefix,
    const ToolpathGenerationControl* control) const
{
    return generateImpl(
        regions, params, operationPrefix, control, true);
}

PocketToolpathBatch PocketToolpathService::generateCandidate(
    const std::vector<PocketRegion>& regions,
    const ToolpathParams& params,
    const std::string& operationPrefix,
    const ToolpathGenerationControl* control) const
{
    return generateImpl(
        regions, params, operationPrefix, control, false);
}

PocketToolpathBatch PocketToolpathService::generateImpl(
    const std::vector<PocketRegion>& regions,
    const ToolpathParams& params,
    const std::string& operationPrefix,
    const ToolpathGenerationControl* control,
    bool validateSafety) const
{
    // Legacy structural/safety validators inherit this synchronous budget.
    ToolpathParallelScope parallelScope(control);
    // 先归一化参数:失败时把全部区域记为 InvalidRegion 失败并返回,
    // 不再进入算法阶段。
    PocketToolpathBatch batch;
    batch.algorithmId = m_algorithmId;
    const bool parametersValid =
        workflow_detail::normalizeBatchParameters(batch, params, true);
    batch.operations.reserve(regions.size());
    batch.failures.reserve(regions.size());
    batch.regionOutcomes.reserve(regions.size());

    const std::size_t total = regions.size();
    const std::size_t totalWork = total + 2u;
    if (control)
        control->progress(0, totalWork);

    if (!parametersValid) {
        for (std::size_t regionIndex = 0;
             regionIndex < regions.size(); ++regionIndex) {
            PocketClearingResult clearing;
            clearing.failureCode = PocketClearingFailureCode::InvalidRegion;
            clearing.error = "toolpath parameters are invalid";
            const int failureIndex = static_cast<int>(batch.failures.size());
            batch.failures.push_back(
                {static_cast<int>(regionIndex), std::move(clearing)});
            batch.regionOutcomes.push_back({-1, failureIndex});
        }
        batch.validation = PocketToolpathValidator().validate(
            batch.operations, batch.effectiveParams);
        return batch;
    }

    // 从注册表解析算法;未注册时报 Error 诊断并为每个区域记失败。
    const IPocketToolpathAlgorithm* algorithm =
        m_algorithms->pocketAlgorithm(batch.algorithmId);
    if (!algorithm) {
        batch.diagnostics.push_back(workflow_detail::unavailableAlgorithmDiagnostic(
            "pocket", batch.algorithmId));
        for (std::size_t regionIndex = 0;
             regionIndex < regions.size(); ++regionIndex) {
            PocketClearingResult clearing;
            clearing.failureCode =
                PocketClearingFailureCode::AlgorithmUnavailable;
            clearing.error = "pocket algorithm is not registered";
            const int failureIndex = static_cast<int>(batch.failures.size());
            batch.failures.push_back(
                {static_cast<int>(regionIndex), std::move(clearing)});
            batch.regionOutcomes.push_back({-1, failureIndex});
        }
        batch.validation = PocketToolpathValidator().validate(
            batch.operations, batch.effectiveParams);
        return batch;
    }

    // 工作副本控制对象:若无外部分享的材料截面缓存则创建一个,使
    // 同一批内多个区域能复用截面几何求交结果。
    ToolpathGenerationControl batchControl =
        control ? *control : ToolpathGenerationControl{};
    if (!batchControl.pocketMaterialSectionCache) {
        batchControl.pocketMaterialSectionCache =
            std::make_shared<
                PocketMaterialSectionCache>();
    }
    const ToolpathGenerationControl* effectiveControl =
        &batchControl;

    // 主循环:逐区域生成。每个区域独立 try/catch,单区失败只进 failures
    // 不中断整批;循环前后与区域内均响应协作取消。
    for (std::size_t regionIndex = 0;
         regionIndex < regions.size(); ++regionIndex) {
        if (effectiveControl->cancelled()) {
            batch.cancelled = true;
            batch.diagnostics.push_back(
                {ToolpathDiagnosticCode::GenerationCancelled,
                 ToolpathDiagnosticSeverity::Info,
                 "pocket generation was cancelled",
                 {}, static_cast<int>(regionIndex)});
            break;
        }

        const PocketRegion& region = regions[regionIndex];
        PocketClearingResult clearing;
        try {
            clearing = algorithm->generate(
                region, batch.effectiveParams,
                effectiveControl);
        } catch (const std::exception& error) {
            clearing.failureCode =
                PocketClearingFailureCode::AlgorithmExecutionFailed;
            clearing.error = error.what();
            batch.diagnostics.push_back(workflow_detail::failedAlgorithmDiagnostic(
                "pocket", clearing.error.c_str(), static_cast<int>(regionIndex)));
        } catch (...) {
            clearing.failureCode =
                PocketClearingFailureCode::AlgorithmExecutionFailed;
            clearing.error = "unknown exception";
            batch.diagnostics.push_back(workflow_detail::failedAlgorithmDiagnostic(
                "pocket", nullptr, static_cast<int>(regionIndex)));
        }

        if (clearing.cancelled ||
            effectiveControl->cancelled()) {
            batch.cancelled = true;
            batch.diagnostics.push_back(
                {ToolpathDiagnosticCode::GenerationCancelled,
                 ToolpathDiagnosticSeverity::Info,
                 "pocket generation was cancelled during depth-layer processing",
                 {}, static_cast<int>(regionIndex)});
            break;
        }

        if (!clearing.ok || clearing.moves.empty()) {
            const int failureIndex = static_cast<int>(batch.failures.size());
            batch.failures.push_back(
                {static_cast<int>(regionIndex), std::move(clearing)});
            batch.regionOutcomes.push_back({-1, failureIndex});
            effectiveControl->progress(
                regionIndex + 1, totalWork);
            continue;
        }

        // 成功路径:把清料结果封装为操作,记录区域→操作的映射与元数据
        // (工序 id、算法 id、来源区域、精加工前刀具直径等)。
        const int operationIndex = static_cast<int>(batch.operations.size());
        PocketToolpathOperation operation;
        operation.operationId =
            operationPrefix + std::to_string(batch.operations.size() + 1);
        operation.algorithmId = batch.algorithmId;
        operation.sourceRegionIndex =
            static_cast<int>(regionIndex);
        operation.restMachiningPreviousToolDiameter =
            batch.effectiveParams.clearing
                .restMachiningPreviousToolDiameter;
        operation.kind = region.kind;
        operation.outwardToolAxis = gp_Vec(region.openDir);
        operation.sourceFaceIds = region.sourceFaceIds;
        operation.floorFaceId = region.floorFaceId;
        operation.featureBoundaryShape = region.sectionShape;
        operation.clearing = std::move(clearing);
        batch.operations.push_back(std::move(operation));
        batch.regionOutcomes.push_back({operationIndex, -1});
        effectiveControl->progress(
            regionIndex + 1, totalWork);
    }

    // 取消时丢弃本轮全部结果,保持「全有或全无」。
    if (batch.cancelled) {
        batch.operations.clear();
        batch.failures.clear();
        batch.regionOutcomes.clear();
    }

    // 结构验证:对成功操作整体执行,并记录耗时供 UI 展示。
    const Clock::time_point structuralValidationStart =
        Clock::now();
    batch.validation =
        PocketToolpathValidator().validate(
            batch.operations, batch.effectiveParams);
    batch.structuralValidationMilliseconds =
        elapsedMilliseconds(
            structuralValidationStart);
    effectiveControl->progress(total + 1u, totalWork);

    // 安全验证:对每个有操作的区域并行执行碰撞/安全距离校验,结果先按
    // 区域索引存槽,之后统一合并进验证报告与诊断。
    if (validateSafety)
        validateCandidateSafety(regions, batch, effectiveControl);
    // 收尾:读取材料截面缓存统计并返回。
    batch.materialSectionCacheHits =
        batchControl.pocketMaterialSectionCache
            ->hitCount();
    batch.materialSectionCacheMisses =
        batchControl.pocketMaterialSectionCache
            ->missCount();
    batch.materialSectionCacheEntries =
        batchControl.pocketMaterialSectionCache
            ->entryCount();
    return batch;
}

bool PocketToolpathService::validateCandidateSafety(
    const std::vector<PocketRegion>& regions,
    PocketToolpathBatch& batch,
    const ToolpathGenerationControl* control) const
{
    ToolpathParallelScope parallelScope(control);
    if (batch.safetyValidationPerformed)
        return !batch.cancelled && batch.validation.ok();
    batch.safetyValidationPerformed = true;

    if (batch.cancelled || !batch.validation.ok())
        return false;
    if (regions.size() < batch.regionOutcomes.size()) {
        addToolpathIssue(
            batch.validation, ToolpathIssueSeverity::Error,
            "pocket.safety.region_mapping",
            "pocket safety validation is missing source regions");
        return false;
    }

    const Clock::time_point safetyValidationStart = Clock::now();
    std::vector<PocketSafetyValidationResult>
        safetyResults(batch.regionOutcomes.size());
    std::vector<std::uint8_t> hasSafetyResult(
        batch.regionOutcomes.size(), 0u);
    toolpathParallelForBudgeted(
        batch.regionOutcomes.size(),
        [&](std::size_t regionIndex) {
            const PocketToolpathOperation* operation =
                batch.operationForRegion(regionIndex);
            if (!operation)
                return;
            safetyResults[regionIndex] =
                PocketSafetyValidator().validate(
                    regions[regionIndex], *operation,
                    batch.effectiveParams,
                    control ? control->isCancellationRequested
                            : std::function<bool()>{});
            hasSafetyResult[regionIndex] = 1u;
        });
    batch.safetyValidationMilliseconds +=
        elapsedMilliseconds(safetyValidationStart);
    const bool safetyCancelled =
        (control && control->cancelled()) ||
        std::any_of(
            safetyResults.begin(), safetyResults.end(),
            [](const PocketSafetyValidationResult& safety) {
                return safety.cancelled;
            });
    if (safetyCancelled) {
        batch.cancelled = true;
        batch.operations.clear();
        batch.failures.clear();
        batch.regionOutcomes.clear();
        batch.diagnostics.push_back(
            {ToolpathDiagnosticCode::GenerationCancelled,
             ToolpathDiagnosticSeverity::Info,
             "pocket generation was cancelled during safety validation"});
        return false;
    }
    if (control)
        control->progress(regions.size() + 2u, regions.size() + 2u);
    for (std::size_t regionIndex = 0;
         regionIndex < safetyResults.size(); ++regionIndex) {
        if (hasSafetyResult[regionIndex] == 0u)
            continue;
        PocketSafetyValidationResult& safety = safetyResults[regionIndex];
        batch.safetyValidation.append(safety.report);
        batch.validation.append(std::move(safety.report));
        batch.diagnostics.insert(
            batch.diagnostics.end(),
            std::make_move_iterator(safety.diagnostics.begin()),
            std::make_move_iterator(safety.diagnostics.end()));
    }
    return batch.validation.ok();
}

} // namespace oap::toolpath

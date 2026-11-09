// 活动刀库槽腔多刀开粗：以最小铣刀的可达结果为目标，使用最短路径
// 动态规划选择从大刀到小刀的递减刀具组合。
//
// 模块归属：oap_toolpath_adapter（刀具库决策 -> CAM 输入适配）。
// 调用方只需提供 ToolLibrary 与槽腔区域，结果含可执行工序与逐阶段审计。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "tooling/library/ToolLibrary.h"
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"
#include "toolpath/workflow/pocket/PocketToolpathService.h"

namespace oap::tooling {

// 选中链路上单个阶段的汇总：刀具、前序刀具直径与移动/切削距离。
struct PocketAutomaticToolpathStage {
    // tool 为本阶段所用刀具（含 T 号与切削参数）；previousToolDiameter 为
    // 余料开粗的前序刀具直径，0 表示全量开粗。
    oap::toolpath::ToolpathToolContext tool;
    double previousToolDiameter = 0.0;
    double movementDistance = 0.0;
    double cuttingDistance = 0.0;
    int operationCount = 0;
};

// 单刀全量或大刀后余料候选阶段的稳定审计结果。即使阶段被规划器
// 淘汰也保留覆盖和验证原因，供日志与界面诊断使用。
struct PocketAutomaticToolpathStageAudit {
    oap::toolpath::ToolpathToolContext tool;
    double previousToolDiameter = 0.0;
    double movementDistance = 0.0;
    int operationCount = 0;
    int failedRegionCount = 0;
    int noRestMaterialRegionCount = 0;
    int validationErrorCount = 0;
    int validationWarningCount = 0;
    bool usable = false;
    bool safetyValidated = false;
    bool reusedPlanningEvaluation = false;
    // 候选入口墙钟耗时，以及其内部清腔/结构/安全子阶段耗时。清腔子项
    // 来自对应区域 PocketClearingResult 的既有低开销计时。
    double generationMilliseconds = 0.0;
    double clearingMilliseconds = 0.0;
    double sectionMilliseconds = 0.0;
    double patternMilliseconds = 0.0;
    double moveAssemblyMilliseconds = 0.0;
    double movePreparationMilliseconds = 0.0;
    double moveLinkingMilliseconds = 0.0;
    std::size_t patternParallelBatchCount = 0u;
    std::size_t patternParallelTaskCount = 0u;
    std::size_t patternPeakParallelism = 1u;
    std::size_t travelChoiceParallelBatchCount = 0u;
    std::size_t travelChoiceParallelTaskCount = 0u;
    std::size_t travelChoicePeakParallelism = 1u;
    std::size_t sectionWireParallelBatchCount = 0u;
    std::size_t sectionWireParallelTaskCount = 0u;
    std::size_t sectionWirePeakParallelism = 1u;
    std::size_t occtInternalParallelCount = 0u;
    std::size_t occtSerialFallbackCount = 0u;
    double structuralValidationMilliseconds = 0.0;
    double safetyValidationMilliseconds = 0.0;
    std::vector<int> regionIndices;
    std::vector<std::string> failureDetails;
};

// 自动规划的完整结果：工序批 + 选中阶段 + 全部候选审计 + 规划统计。
struct PocketAutomaticToolpathResult {
    oap::toolpath::PocketToolpathBatch batch;
    std::vector<PocketAutomaticToolpathStage> stages;
    std::vector<PocketAutomaticToolpathStageAudit> stageAudits;
    std::string libraryId;
    std::string libraryName;
    std::string error;
    int candidateToolCount = 0;
    int planningParallelism = 1;
    int planningCpuParallelism = 1;
    int planningMemoryParallelism = 1;
    std::uint64_t planningAvailableMemoryMiB = 0;
    int planningPeakParallelism = 1;
    int planningPeakRegionParallelism = 1;
    int planningFinalParallelism = 1;
    int planningConcurrencyAdjustmentCount = 0;
    std::uint64_t planningEstimatedRegionMemoryMiB = 0;
    std::uint64_t planningObservedPeakRegionMemoryMiB = 0;
    int planningBorrowedTokenCount = 0;
    int evaluatedTransitionCount = 0;
    int prunedTransitionCount = 0;
    int generatedStageCount = 0;
    int reusedStageEvaluationCount = 0;
    int selectedStageMaterializationCount = 0;
    int safetyValidatedStageCount = 0;
    int safetyRejectedStageCount = 0;
    int planningReplanCount = 0;
    int parallelRegionBatchCount = 0;
    int parallelFullStageBatchCount = 0;
    int prefetchedFullStageCount = 0;
    int parallelTransitionBatchCount = 0;
    // 下列值为所有候选审计的累计工作耗时；候选可并行，因此不等于批次墙钟
    // 时间。maximum* 用于快速定位串行长尾。
    double fullStageGenerationMilliseconds = 0.0;
    double transitionStageGenerationMilliseconds = 0.0;
    double candidateClearingMilliseconds = 0.0;
    double candidateSectionMilliseconds = 0.0;
    double candidatePatternMilliseconds = 0.0;
    double candidateMoveAssemblyMilliseconds = 0.0;
    double candidateMovePreparationMilliseconds = 0.0;
    double candidateMoveLinkingMilliseconds = 0.0;
    double candidateStructuralValidationMilliseconds = 0.0;
    double planningSafetyValidationMilliseconds = 0.0;
    double maximumStageGenerationMilliseconds = 0.0;
    double maximumSafetyValidationMilliseconds = 0.0;
    double baselineSmallToolCuttingDistance = 0.0;
    double selectedCuttingDistance = 0.0;
    bool cancelled = false;

    bool ok() const noexcept
    {
        return !cancelled && error.empty() &&
               !batch.operations.empty() && batch.validation.ok();
    }
};

// 槽腔多刀自动粗加工规划器：无状态，generate 每次独立规划，可多线程调用。
class PocketAutomaticToolpath {
public:
    // 生成自动粗加工：返回结果含可执行工序与审计；失败原因见 result.error。
    PocketAutomaticToolpathResult generate(
        const ToolLibrary& library,
        const std::vector<oap::toolpath::PocketRegion>& regions,
        const oap::toolpath::ToolpathParams& params,
        const std::string& operationPrefix = "POCKET_AUTO",
        const oap::toolpath::ToolpathGenerationControl* control = nullptr) const;
};

} // namespace oap::tooling

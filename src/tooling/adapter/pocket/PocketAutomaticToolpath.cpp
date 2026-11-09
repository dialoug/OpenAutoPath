// 活动刀库槽腔多刀自动粗加工实现：候选铣刀筛选 → 兼容区域分组 →
// 逐组最短路径动态规划选刀（全量/余料阶段）→ 汇总工序与审计。
#include "tooling/adapter/pocket/PocketAutomaticToolpath.h"
#include "tooling/adapter/AssignedToolContext.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <utility>

#include "detail/PocketPlanningScheduler.h"

#include "toolpath/core/PocketToolBinding.h"
#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/pocket/planning/PocketLayerSlicer.h"
#include "toolpath/pocket/validation/PocketToolpathValidator.h"

namespace oap::tooling {
namespace {

// 直径/距离比较容差，用于刀具去重、排序与 DP 距离判优。
constexpr double kDiameterTolerance = 1.0e-4;
constexpr double kDistanceTolerance = 1.0e-6;
constexpr std::size_t kMaximumFullStagePrefetch = 4u;

using detail::RegionMemoryTracker;

// 候选铣刀筛选的统计审计，用于无可用刀具时的诊断信息。
struct MillingToolAudit {
    double requiredCuttingEdgeLength = 0.0;
    double requiredReach = 0.0;
    int assignedToolCount = 0;
    int assignedEndMillCount = 0;
    int invalidGeometryCount = 0;
    int unsupportedGeometryCount = 0;
    int cuttingLengthTooShortCount = 0;
    int overallLengthTooShortCount = 0;
};

// 槽腔区域对刀具的工艺要求：单层最大切深与刀具可达深度。
struct PocketRegionRequirements {
    double cuttingEdgeLength = 0.0;
    double reach = 0.0;
};

// 计算深度方向单次开粗所需的最大单层切深：由 stepDown 与 finishDepth
// 约束出最大有效切深，作为刀具刃长下限的依据。
double maximumLayerEngagement(double travel,
                              const oap::toolpath::DepthParams& depth)
{
    constexpr double tolerance = 1.0e-9;
    if (!std::isfinite(travel) || travel <= tolerance)
        return 0.0;

    double stepDown = depth.stepDown;
    if (!std::isfinite(stepDown) || stepDown <= tolerance)
        stepDown = travel;

    double finishDepth = depth.finishDepth;
    if (!std::isfinite(finishDepth) || finishDepth < 0.0 ||
        finishDepth + tolerance >= travel) {
        finishDepth = 0.0;
    }
    return std::max(
        std::min(stepDown, travel - finishDepth), finishDepth);
}

// 由区域深度与深度参数推导刀具要求；useFeatureDepth 时按特征深度，
// 否则按用户 startDepth/finalDepth 的范围与最深点计算。
PocketRegionRequirements pocketRequirements(
    const oap::toolpath::PocketRegion& region,
    const oap::toolpath::ToolpathParams& params)
{
    const double regionDepth =
        std::isfinite(region.depth) ? std::max(0.0, region.depth) : 0.0;
    double travel = regionDepth;
    double deepestOffset = 0.0;
    if (!params.depth.useFeatureDepth) {
        if (std::isfinite(params.depth.startDepth) &&
            std::isfinite(params.depth.finalDepth)) {
            travel = std::abs(
                params.depth.startDepth - params.depth.finalDepth);
            deepestOffset = std::min(
                params.depth.startDepth, params.depth.finalDepth);
        } else {
            travel = 0.0;
        }
    }
    PocketRegionRequirements result;
    result.cuttingEdgeLength =
        maximumLayerEngagement(travel, params.depth);
    result.reach = std::max(0.0, regionDepth - deepestOffset);
    return result;
}

// 判断刀具是否满足区域要求：刀刃足够长（粗柄按可达深度）且总长够深。
bool toolSupportsRegion(
    const oap::toolpath::ToolpathToolContext& tool,
    const PocketRegionRequirements& requirements,
    bool* cuttingLengthTooShort = nullptr,
    bool* overallLengthTooShort = nullptr)
{
    const double shankDiameter = tool.shankDiameter > 0.0
        ? tool.shankDiameter : tool.diameter;
    const double requiredCuttingEdgeLength =
        shankDiameter > tool.diameter + kDiameterTolerance
        ? requirements.reach : requirements.cuttingEdgeLength;
    const bool cuttingTooShort =
        tool.cuttingEdgeLength > 0.0 &&
        tool.cuttingEdgeLength + kDiameterTolerance <
            requiredCuttingEdgeLength;
    const bool overallTooShort =
        tool.overallLength > 0.0 &&
        tool.overallLength + kDiameterTolerance < requirements.reach;
    if (cuttingLengthTooShort)
        *cuttingLengthTooShort = cuttingTooShort;
    if (overallLengthTooShort)
        *overallLengthTooShort = overallTooShort;
    return !cuttingTooShort && !overallTooShort;
}

// 汇总全部区域的最大刃长/可达要求，供候选筛选与诊断使用。
void auditPocketRequirements(
    const std::vector<oap::toolpath::PocketRegion>& regions,
    const oap::toolpath::ToolpathParams& params,
    MillingToolAudit& audit)
{
    for (const oap::toolpath::PocketRegion& region : regions) {
        const PocketRegionRequirements requirements =
            pocketRequirements(region, params);
        audit.requiredCuttingEdgeLength = std::max(
            audit.requiredCuttingEdgeLength,
            requirements.cuttingEdgeLength);
        audit.requiredReach = std::max(
            audit.requiredReach, requirements.reach);
    }
}

// 筛选可用铣刀：几何有效、支持平底/球头/圆角、至少支持一个区域；随后按直径降序、
// 同直径优先较小圆角，再按最小 T 号；区域可达性筛选后才按直径去重。
std::vector<oap::toolpath::ToolpathToolContext> millingTools(
    const ToolLibrary& library,
    const std::vector<oap::toolpath::PocketRegion>& regions,
    const oap::toolpath::ToolpathParams& params,
    MillingToolAudit& audit)
{
    auditPocketRequirements(regions, params, audit);

    std::vector<oap::toolpath::ToolpathToolContext> result;
    for (const AssignedTool& assigned : library.assignedTools()) {
        ++audit.assignedToolCount;
        if (!isSupportedMillingToolType(assigned.tool.type))
            continue;
        ++audit.assignedEndMillCount;
        oap::toolpath::ToolpathToolContext candidate =
            makeToolpathToolContext(assigned);
        const bool invalidGeometry = !candidate.valid;
        const bool unsupportedGeometry = !hasSupportedMillingGeometry(assigned.tool);
        bool aggregateCuttingLengthTooShort = false;
        bool aggregateOverallLengthTooShort = false;
        const PocketRegionRequirements aggregateRequirements{
            audit.requiredCuttingEdgeLength, audit.requiredReach};
        toolSupportsRegion(
            candidate, aggregateRequirements,
            &aggregateCuttingLengthTooShort,
            &aggregateOverallLengthTooShort);
        const bool supportsAnyRegion = std::any_of(
            regions.begin(), regions.end(),
            [&](const oap::toolpath::PocketRegion& region) {
                return toolSupportsRegion(
                    candidate, pocketRequirements(region, params));
            });
        audit.invalidGeometryCount += invalidGeometry ? 1 : 0;
        audit.unsupportedGeometryCount += unsupportedGeometry ? 1 : 0;
        audit.cuttingLengthTooShortCount +=
            (!supportsAnyRegion && aggregateCuttingLengthTooShort) ? 1 : 0;
        audit.overallLengthTooShortCount +=
            (!supportsAnyRegion && aggregateOverallLengthTooShort) ? 1 : 0;
        if (invalidGeometry || unsupportedGeometry || !supportsAnyRegion) {
            continue;
        }
        result.push_back(std::move(candidate));
    }
    std::stable_sort(
        result.begin(), result.end(),
        [](const auto& left, const auto& right) {
            if (std::abs(left.diameter - right.diameter) >
                kDiameterTolerance) {
                return left.diameter > right.diameter;
            }
            if (left.cornerRadius != right.cornerRadius)
                return left.cornerRadius < right.cornerRadius;
            return left.toolNumber < right.toolNumber;
        });
    return result;
}

// 统计批内所有工序的移动总距离；cuttingOnly 时只计切削移动。
double moveDistance(
    const oap::toolpath::PocketToolpathBatch& batch,
    bool cuttingOnly)
{
    double result = 0.0;
    for (const auto& operation : batch.operations) {
        for (const auto& move : operation.clearing.moves) {
            if (cuttingOnly &&
                move.type != oap::toolpath::ToolpathMoveType::Cut) {
                continue;
            }
            result += move.from.Distance(move.to);
        }
    }
    return result;
}

// 判断失败是否全部为“余料开粗但无剩余材料”的良性失败（可接受）。
bool onlyEmptyRestFailures(
    const oap::toolpath::PocketToolpathBatch& batch)
{
    return !batch.failures.empty() && std::all_of(
        batch.failures.begin(), batch.failures.end(),
        [](const oap::toolpath::PocketGenerationFailure& failure) {
            return failure.clearing.ok &&
                   failure.clearing.restMachining &&
                   failure.clearing.noRestMaterial;
        });
}

// 单次生成结果：原始批次 + 阶段汇总 + 可用性判定。
struct GeneratedStage {
    oap::toolpath::PocketToolpathBatch batch;
    PocketAutomaticToolpathStage summary;
    std::optional<PocketAutomaticToolpathStageAudit> cachedAudit;
    double generationMilliseconds = 0.0;
    bool materialized = true;
    bool reusedPlanningEvaluation = false;
    bool usable = false;
};

using PlanningClock = std::chrono::steady_clock;

double planningElapsedMilliseconds(
    const PlanningClock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(
               PlanningClock::now() - start)
        .count();
}

void appendClearingTiming(
    PocketAutomaticToolpathStageAudit& audit,
    const oap::toolpath::PocketClearingResult& clearing)
{
    audit.clearingMilliseconds += clearing.totalMilliseconds;
    audit.sectionMilliseconds += clearing.sectionMilliseconds;
    audit.patternMilliseconds += clearing.patternMilliseconds;
    audit.moveAssemblyMilliseconds += clearing.moveAssemblyMilliseconds;
    audit.movePreparationMilliseconds +=
        clearing.movePreparationMilliseconds;
    audit.moveLinkingMilliseconds += clearing.moveLinkingMilliseconds;
    audit.patternParallelBatchCount +=
        clearing.patternParallelBatchCount;
    audit.patternParallelTaskCount +=
        clearing.patternParallelTaskCount;
    audit.patternPeakParallelism = std::max(
        audit.patternPeakParallelism,
        clearing.patternPeakParallelism);
    audit.travelChoiceParallelBatchCount +=
        clearing.travelChoiceParallelBatchCount;
    audit.travelChoiceParallelTaskCount +=
        clearing.travelChoiceParallelTaskCount;
    audit.travelChoicePeakParallelism = std::max(
        audit.travelChoicePeakParallelism,
        clearing.travelChoicePeakParallelism);
    audit.sectionWireParallelBatchCount +=
        clearing.sectionWireParallelBatchCount;
    audit.sectionWireParallelTaskCount +=
        clearing.sectionWireParallelTaskCount;
    audit.sectionWirePeakParallelism = std::max(
        audit.sectionWirePeakParallelism,
        clearing.sectionWirePeakParallelism);
    audit.occtInternalParallelCount +=
        clearing.occtInternalParallelCount;
    audit.occtSerialFallbackCount +=
        clearing.occtSerialFallbackCount;
}

// 清腔失败码 -> 稳定英文名，用于审计与日志的紧凑描述。
const char* failureCodeName(
    oap::toolpath::PocketClearingFailureCode code) noexcept
{
    using Code = oap::toolpath::PocketClearingFailureCode;
    switch (code) {
        case Code::None: return "None";
        case Code::AlgorithmUnavailable: return "AlgorithmUnavailable";
        case Code::AlgorithmExecutionFailed: return "AlgorithmExecutionFailed";
        case Code::InvalidRegion: return "InvalidRegion";
        case Code::LayerSectionFailed: return "LayerSectionFailed";
        case Code::LayerNoEdges: return "LayerNoEdges";
        case Code::LayerOpenContour: return "LayerOpenContour";
        case Code::LayerNoValidArea: return "LayerNoValidArea";
        case Code::ToolDoesNotFit: return "ToolDoesNotFit";
        case Code::NoClearableArea: return "NoClearableArea";
    }
    return "Unknown";
}

// 把一次生成结果转成稳定审计记录：保留失败区域、验证错误与原因明细。
PocketAutomaticToolpathStageAudit stageAudit(
    const GeneratedStage& stage,
    const std::vector<int>& sourceRegionIndices)
{
    if (stage.cachedAudit.has_value()) {
        PocketAutomaticToolpathStageAudit audit = *stage.cachedAudit;
        audit.reusedPlanningEvaluation =
            stage.reusedPlanningEvaluation;
        audit.generationMilliseconds = 0.0;
        audit.clearingMilliseconds = 0.0;
        audit.sectionMilliseconds = 0.0;
        audit.patternMilliseconds = 0.0;
        audit.moveAssemblyMilliseconds = 0.0;
        audit.movePreparationMilliseconds = 0.0;
        audit.moveLinkingMilliseconds = 0.0;
        audit.patternParallelBatchCount = 0u;
        audit.patternParallelTaskCount = 0u;
        audit.patternPeakParallelism = 1u;
        audit.travelChoiceParallelBatchCount = 0u;
        audit.travelChoiceParallelTaskCount = 0u;
        audit.travelChoicePeakParallelism = 1u;
        audit.sectionWireParallelBatchCount = 0u;
        audit.sectionWireParallelTaskCount = 0u;
        audit.sectionWirePeakParallelism = 1u;
        audit.occtInternalParallelCount = 0u;
        audit.occtSerialFallbackCount = 0u;
        audit.structuralValidationMilliseconds = 0.0;
        audit.safetyValidationMilliseconds = 0.0;
        return audit;
    }
    PocketAutomaticToolpathStageAudit audit;
    audit.tool = stage.summary.tool;
    audit.previousToolDiameter = stage.summary.previousToolDiameter;
    audit.movementDistance = stage.summary.movementDistance;
    audit.operationCount = stage.summary.operationCount;
    audit.validationErrorCount = stage.batch.validation.errorCount;
    audit.validationWarningCount = stage.batch.validation.warningCount;
    audit.usable = stage.usable;
    audit.safetyValidated = stage.batch.safetyValidationPerformed;
    audit.reusedPlanningEvaluation =
        stage.reusedPlanningEvaluation;
    audit.generationMilliseconds = stage.generationMilliseconds;
    audit.structuralValidationMilliseconds =
        stage.batch.structuralValidationMilliseconds;
    audit.safetyValidationMilliseconds =
        stage.batch.safetyValidationMilliseconds;
    audit.regionIndices = sourceRegionIndices;
    for (const auto& operation : stage.batch.operations)
        appendClearingTiming(audit, operation.clearing);
    for (const oap::toolpath::PocketGenerationFailure& failure :
         stage.batch.failures) {
        appendClearingTiming(audit, failure.clearing);
        if (failure.clearing.ok && failure.clearing.restMachining &&
            failure.clearing.noRestMaterial) {
            ++audit.noRestMaterialRegionCount;
            continue;
        }
        ++audit.failedRegionCount;
        const int sourceRegionIndex =
            failure.regionIndex >= 0 &&
                failure.regionIndex <
                    static_cast<int>(sourceRegionIndices.size())
            ? sourceRegionIndices[
                  static_cast<std::size_t>(failure.regionIndex)]
            : failure.regionIndex;
        std::ostringstream detail;
        detail << "region " << sourceRegionIndex + 1
               << " [" << failureCodeName(failure.clearing.failureCode)
               << ']';
        if (failure.clearing.failedDepthIndex >= 0) {
            detail << " depth " << failure.clearing.failedDepthIndex + 1
                   << " (offset=" << failure.clearing.failedDepthOffset
                   << " mm)";
        }
        if (!failure.clearing.error.empty())
            detail << ": " << failure.clearing.error;
        audit.failureDetails.push_back(detail.str());
    }
    for (const oap::toolpath::ToolpathIssue& issue :
         stage.batch.validation.issues) {
        if (issue.severity !=
            oap::toolpath::ToolpathIssueSeverity::Error) {
            continue;
        }
        std::ostringstream detail;
        detail << "validation [" << issue.code << ']';
        if (!issue.operationId.empty())
            detail << " operation " << issue.operationId;
        detail << ": " << issue.message;
        audit.failureDetails.push_back(detail.str());
    }
    return audit;
}

// 最小刀仍不可达时的失败消息：附最小刀审计统计与最多 5 条原因明细。
std::string finalPlanningFailure(
    const std::vector<oap::toolpath::ToolpathToolContext>& candidates,
    std::size_t regionCount,
    const std::vector<PocketAutomaticToolpathStageAudit>& audits)
{
    if (candidates.empty())
        return "automatic pocket planning has no candidate tool";
    const oap::toolpath::ToolpathToolContext& smallest = candidates.back();
    const auto found = std::find_if(
        audits.begin(), audits.end(),
        [&smallest](const PocketAutomaticToolpathStageAudit& audit) {
            return audit.tool.toolNumber == smallest.toolNumber &&
                   audit.previousToolDiameter <= kDiameterTolerance;
        });
    QString message = QStringLiteral(
        "最小铣刀 T%1（%2，D%3 mm）无法完成全部 %4 个槽腔区域")
        .arg(smallest.toolNumber)
        .arg(smallest.toolName.empty()
                 ? QStringLiteral("未命名刀具")
                 : QString::fromStdString(smallest.toolName))
        .arg(smallest.diameter, 0, 'f', 3)
        .arg(static_cast<qulonglong>(regionCount));
    if (found == audits.end())
        return message.toUtf8().toStdString();

    message += QStringLiteral(
        "：生成工序 %1，失败区域 %2，无需余料区域 %3，验证错误 %4。")
        .arg(found->operationCount)
        .arg(found->failedRegionCount)
        .arg(found->noRestMaterialRegionCount)
        .arg(found->validationErrorCount);
    const std::size_t detailLimit = std::min<std::size_t>(
        5u, found->failureDetails.size());
    for (std::size_t index = 0; index < detailLimit; ++index) {
        message += QStringLiteral("\n- %1").arg(
            QString::fromStdString(found->failureDetails[index]));
    }
    if (found->failureDetails.size() > detailLimit) {
        message += QStringLiteral("\n- 其余 %1 项见腔清腔日志")
            .arg(static_cast<qulonglong>(
                found->failureDetails.size() - detailLimit));
    }
    return message.toUtf8().toStdString();
}

// 用指定刀具（可带前序直径做余料开粗）生成一批工序并汇总；判定是否可用。
GeneratedStage generateStage(
    const oap::toolpath::ToolpathToolContext& tool,
    double previousDiameter,
    const std::vector<oap::toolpath::PocketRegion>& regions,
    const oap::toolpath::ToolpathParams& base,
    const std::string& prefix,
    const oap::toolpath::ToolpathGenerationControl* control)
{
    const PlanningClock::time_point generationStart = PlanningClock::now();
    GeneratedStage result;
    oap::toolpath::ToolpathParams effective =
        oap::toolpath::bindPocketToolParameters(base, tool);
    effective.clearing.restMachiningPreviousToolDiameter =
        previousDiameter;
    result.batch = oap::toolpath::PocketToolpathService().generateCandidate(
        regions, effective, prefix, control);
    result.summary.tool = tool;
    result.summary.previousToolDiameter = previousDiameter;
    result.summary.movementDistance =
        moveDistance(result.batch, false);
    result.summary.cuttingDistance =
        moveDistance(result.batch, true);
    result.summary.operationCount =
        static_cast<int>(result.batch.operations.size());
    for (auto& operation : result.batch.operations) {
        operation.tool = tool;
        operation.restMachiningPreviousToolDiameter =
            previousDiameter;
    }
    const bool emptyRestFailures =
        previousDiameter > tool.diameter &&
        onlyEmptyRestFailures(result.batch);
    const bool acceptableFailures =
        result.batch.failures.empty() || emptyRestFailures;
    result.usable = !result.batch.cancelled &&
        acceptableFailures &&
        ((!result.batch.operations.empty() &&
          result.batch.validation.ok()) ||
         (previousDiameter > tool.diameter &&
          result.batch.operations.empty() &&
          emptyRestFailures));
    result.generationMilliseconds =
        planningElapsedMilliseconds(generationStart);
    return result;
}

void updatePlanningTimingSummary(PocketAutomaticToolpathResult& result)
{
    result.fullStageGenerationMilliseconds = 0.0;
    result.transitionStageGenerationMilliseconds = 0.0;
    result.candidateClearingMilliseconds = 0.0;
    result.candidateSectionMilliseconds = 0.0;
    result.candidatePatternMilliseconds = 0.0;
    result.candidateMoveAssemblyMilliseconds = 0.0;
    result.candidateMovePreparationMilliseconds = 0.0;
    result.candidateMoveLinkingMilliseconds = 0.0;
    result.candidateStructuralValidationMilliseconds = 0.0;
    result.planningSafetyValidationMilliseconds = 0.0;
    result.maximumStageGenerationMilliseconds = 0.0;
    result.maximumSafetyValidationMilliseconds = 0.0;
    for (const PocketAutomaticToolpathStageAudit& audit :
         result.stageAudits) {
        if (audit.previousToolDiameter >
            audit.tool.diameter + kDiameterTolerance) {
            result.transitionStageGenerationMilliseconds +=
                audit.generationMilliseconds;
        } else {
            result.fullStageGenerationMilliseconds +=
                audit.generationMilliseconds;
        }
        result.candidateClearingMilliseconds += audit.clearingMilliseconds;
        result.candidateSectionMilliseconds += audit.sectionMilliseconds;
        result.candidatePatternMilliseconds += audit.patternMilliseconds;
        result.candidateMoveAssemblyMilliseconds +=
            audit.moveAssemblyMilliseconds;
        result.candidateMovePreparationMilliseconds +=
            audit.movePreparationMilliseconds;
        result.candidateMoveLinkingMilliseconds +=
            audit.moveLinkingMilliseconds;
        result.candidateStructuralValidationMilliseconds +=
            audit.structuralValidationMilliseconds;
        result.planningSafetyValidationMilliseconds +=
            audit.safetyValidationMilliseconds;
        result.maximumStageGenerationMilliseconds = std::max(
            result.maximumStageGenerationMilliseconds,
            audit.generationMilliseconds);
        result.maximumSafetyValidationMilliseconds = std::max(
            result.maximumSafetyValidationMilliseconds,
            audit.safetyValidationMilliseconds);
    }
}

// 把选中的阶段（诊断/校验耗时/工序）合并进最终批次输出。
void appendSelectedStage(
    oap::toolpath::PocketToolpathBatch& output,
    GeneratedStage&& stage)
{
    output.diagnostics.insert(
        output.diagnostics.end(),
        std::make_move_iterator(stage.batch.diagnostics.begin()),
        std::make_move_iterator(stage.batch.diagnostics.end()));
    output.structuralValidationMilliseconds +=
        stage.batch.structuralValidationMilliseconds;
    output.safetyValidationMilliseconds +=
        stage.batch.safetyValidationMilliseconds;
    output.safetyValidation.append(
        std::move(stage.batch.safetyValidation));
    for (auto& operation : stage.batch.operations)
        output.operations.push_back(std::move(operation));
}

// 单个槽腔区域及其候选刀具。每个区域独立规划刀具链，避免某个窄槽的
// ToolDoesNotFit 失败连带淘汰同组宽腔中本可使用的大刀。
struct CompatibleRegionGroup {
    std::vector<oap::toolpath::ToolpathToolContext> candidates;
    std::vector<oap::toolpath::PocketRegion> regions;
    std::vector<int> sourceRegionIndices;
};

// 单个区域组的规划结果：选中批次、全量审计与 DP 统计。
struct RegionGroupPlan {
    oap::toolpath::PocketToolpathBatch batch;
    std::vector<PocketAutomaticToolpathStageAudit> audits;
    std::string error;
    int evaluatedTransitionCount = 0;
    int prunedTransitionCount = 0;
    int generatedStageCount = 0;
    int reusedStageEvaluationCount = 0;
    int selectedStageMaterializationCount = 0;
    int safetyValidatedStageCount = 0;
    int safetyRejectedStageCount = 0;
    int planningReplanCount = 0;
    int parallelFullStageBatchCount = 0;
    int prefetchedFullStageCount = 0;
    int parallelTransitionBatchCount = 0;
    double baselineSmallToolCuttingDistance = 0.0;
    double selectedCuttingDistance = 0.0;
    std::uint64_t peakWorkingSetGrowthBytes = 0u;
    bool cancelled = false;
};

// 重规划只缓存 DP 所需的轻量结果与审计，不保留候选 moves/primitives。
// 这样被安全黑名单影响的路径可快速重算，而不会让全部候选几何常驻内存。
struct CachedStageEvaluation {
    PocketAutomaticToolpathStage summary;
    PocketAutomaticToolpathStageAudit audit;
    bool usable = false;
};

class RegionReplanEvaluationCache {
public:
    explicit RegionReplanEvaluationCache(std::size_t candidateCount)
        : m_candidateCount(candidateCount),
          m_entries((candidateCount + 1u) * candidateCount)
    {
    }

    const CachedStageEvaluation* find(
        int previous, std::size_t current) const
    {
        const std::size_t index = entryIndex(previous, current);
        return m_entries[index].has_value()
            ? &m_entries[index].value()
            : nullptr;
    }

    void store(
        int previous,
        std::size_t current,
        const GeneratedStage& stage,
        const std::vector<int>& sourceRegionIndices)
    {
        if (stage.batch.cancelled)
            return;
        CachedStageEvaluation cached;
        cached.summary = stage.summary;
        cached.audit = stageAudit(stage, sourceRegionIndices);
        cached.usable = stage.usable;
        m_entries[entryIndex(previous, current)] = std::move(cached);
    }

private:
    std::size_t entryIndex(int previous, std::size_t current) const
    {
        const std::size_t row = static_cast<std::size_t>(previous + 1);
        return row * m_candidateCount + current;
    }

    std::size_t m_candidateCount = 0u;
    std::vector<std::optional<CachedStageEvaluation>> m_entries;
};

GeneratedStage reusedStage(const CachedStageEvaluation& cached)
{
    GeneratedStage result;
    result.summary = cached.summary;
    result.cachedAudit = cached.audit;
    result.materialized = false;
    result.reusedPlanningEvaluation = true;
    result.usable = cached.usable;
    return result;
}

GeneratedStage lightweightStage(
    const GeneratedStage& stage,
    const PocketAutomaticToolpathStageAudit& audit)
{
    GeneratedStage result;
    result.summary = stage.summary;
    result.cachedAudit = audit;
    result.materialized = false;
    result.reusedPlanningEvaluation =
        stage.reusedPlanningEvaluation;
    result.usable = stage.usable;
    return result;
}

struct RejectedStageEdge {
    int previous = -1;
    int current = -1;
};

bool isRejectedStageEdge(
    const std::vector<RejectedStageEdge>& rejectedEdges,
    int previous,
    int current)
{
    return std::any_of(
        rejectedEdges.begin(), rejectedEdges.end(),
        [&](const RejectedStageEdge& edge) {
            return edge.previous == previous && edge.current == current;
        });
}

// 源区域号（1 基）的逗号分隔列表，用于错误消息。
std::string regionList(const std::vector<int>& sourceRegionIndices)
{
    std::ostringstream result;
    for (std::size_t index = 0; index < sourceRegionIndices.size(); ++index) {
        if (index > 0)
            result << ", ";
        result << sourceRegionIndices[index] + 1;
    }
    return result.str();
}

// 把组内局部区域下标映射回全局源区域下标，写入工序供输出使用。
void remapOperationRegions(
    GeneratedStage& stage,
    const std::vector<int>& sourceRegionIndices)
{
    for (auto& operation : stage.batch.operations) {
        if (operation.sourceRegionIndex < 0 ||
            operation.sourceRegionIndex >=
                static_cast<int>(sourceRegionIndices.size())) {
            continue;
        }
        operation.sourceRegionIndex = sourceRegionIndices[
            static_cast<std::size_t>(operation.sourceRegionIndex)];
    }
}

// 核心选刀规划：对候选序列做最短路径动态规划。每个状态代表“用某把刀
// 完成此前全部区域（可含余料阶段）”的最优方案；状态距离 = 累计切削距离，
// 转移即“用更小的刀对更大刀留下的余料开粗”。全量阶段作初始状态，
// 转移按距离升序批量评估，在共享租约允许的额度内并行生成。
RegionGroupPlan planRegionGroup(
    const CompatibleRegionGroup& group,
    const oap::toolpath::ToolpathParams& params,
    const std::string& operationPrefix,
    const oap::toolpath::ToolpathGenerationControl& sharedControl,
    std::size_t maximumParallelStages,
    const std::vector<RejectedStageEdge>& rejectedEdges = {},
    std::shared_ptr<oap::toolpath::PocketMaterialSectionCache> regionCache = {},
    std::shared_ptr<RegionMemoryTracker> memoryTracker = {},
    std::shared_ptr<RegionReplanEvaluationCache> evaluationCache = {})
{
    RegionGroupPlan result;
    if (!regionCache) {
        regionCache =
            std::make_shared<oap::toolpath::PocketMaterialSectionCache>();
    }
    if (!memoryTracker)
        memoryTracker = std::make_shared<RegionMemoryTracker>();
    if (!evaluationCache) {
        evaluationCache =
            std::make_shared<RegionReplanEvaluationCache>(
                group.candidates.size());
    }
    oap::toolpath::ToolpathGenerationControl scopedControl = sharedControl;
    scopedControl.pocketMaterialSectionCache = regionCache;
    oap::toolpath::ToolpathParallelScope parallelScope(&scopedControl);
    const auto finish = [&]() {
        result.batch.materialSectionCacheHits = regionCache->hitCount();
        result.batch.materialSectionCacheMisses = regionCache->missCount();
        result.batch.materialSectionCacheEntries = regionCache->entryCount();
        result.peakWorkingSetGrowthBytes =
            memoryTracker->peakGrowthBytes();
        return std::move(result);
    };
    const auto& candidates = group.candidates;

    // 一个候选刀具索引对应一个 DP 状态；distance 是到达该状态的最小切削距离。
    struct State {
        // 自动组合的优化成本只包含 Cut；快移、下刀和抬刀不参与权重。
        double distance = std::numeric_limits<double>::infinity();
        int stageCount = std::numeric_limits<int>::max();
        int parent = -2;
        int incomingAuditIndex = -1;
        GeneratedStage incoming;
        bool reachable = false;
    };
    std::vector<State> states(candidates.size());
    std::vector<std::optional<GeneratedStage>> fullStages(
        candidates.size());

    // The scheduler already serializes progress/status across every region and
    // candidate, including recursive replanning; no second callback lock needed.
    const auto& parallelControl = scopedControl;

    const auto candidatePrefix =
        [&](int previous, std::size_t current) {
        const auto& tool = candidates[current];
        if (previous < 0) {
            return operationPrefix + "T" +
                std::to_string(tool.toolNumber) + "_FULL_";
        }
        return operationPrefix + "T" +
            std::to_string(tool.toolNumber) + "_REST_FROM_T" +
            std::to_string(
                candidates[static_cast<std::size_t>(previous)].toolNumber) +
            "_";
    };

    const auto evaluateStage =
        [&](int previous, std::size_t current) {
        if (const CachedStageEvaluation* cached =
                evaluationCache->find(previous, current)) {
            return reusedStage(*cached);
        }
        const double previousDiameter = (previous < 0 ||
                candidates[static_cast<std::size_t>(previous)].cornerRadius > 0.0)
            ? 0.0
            : candidates[static_cast<std::size_t>(previous)].diameter;
        GeneratedStage generated = generateStage(
            candidates[current], previousDiameter, group.regions, params,
            candidatePrefix(previous, current), &parallelControl);
        evaluationCache->store(
            previous, current, generated, group.sourceRegionIndices);
        return generated;
    };

    const auto prefetchFullStages = [&](std::size_t current) {
        if (fullStages[current].has_value())
            return true;

        std::vector<std::size_t> stageIndices;
        stageIndices.reserve(kMaximumFullStagePrefetch);
        for (std::size_t index = current;
             index < candidates.size() &&
             stageIndices.size() < kMaximumFullStagePrefetch;
             ++index) {
            if (!fullStages[index].has_value() &&
                !isRejectedStageEdge(
                    rejectedEdges, -1, static_cast<int>(index))) {
                stageIndices.push_back(index);
            }
        }
        if (stageIndices.empty())
            return true;

        const std::size_t maximumBorrow = std::min(
            maximumParallelStages - 1u,
            stageIndices.size() - 1u);
        oap::toolpath::ToolpathParallelLease tokenLease(
            &scopedControl, maximumBorrow, 0u);
        stageIndices.resize(std::min(
            stageIndices.size(), 1u + tokenLease.count()));

        std::vector<GeneratedStage> generated(stageIndices.size());
        const auto generateFullStage = [&](std::size_t batchIndex) {
            const std::size_t index = stageIndices[batchIndex];
            generated[batchIndex] = evaluateStage(-1, index);
        };
        if (stageIndices.size() == 1u) {
            generateFullStage(0u);
        } else {
            ++result.parallelFullStageBatchCount;
            oap::toolpath::toolpathParallelFor(
                stageIndices.size(), generateFullStage);
        }
        tokenLease.release();
        result.generatedStageCount += static_cast<int>(std::count_if(
            generated.begin(), generated.end(),
            [](const GeneratedStage& stage) {
                return !stage.reusedPlanningEvaluation;
            }));
        result.reusedStageEvaluationCount +=
            static_cast<int>(std::count_if(
                generated.begin(), generated.end(),
                [](const GeneratedStage& stage) {
                    return stage.reusedPlanningEvaluation;
                }));
        result.prefetchedFullStageCount +=
            static_cast<int>(stageIndices.size() - 1u);
        const bool cancelled = std::any_of(
            generated.begin(), generated.end(),
            [](const GeneratedStage& stage) {
                return stage.batch.cancelled;
            }) || scopedControl.cancelled();
        if (cancelled) {
            // 保留已返回候选（包括协作取消的当前长任务）的耗时，便于取消后
            // 仍能定位最后一个昂贵阶段；本路径随后立即退出，不会重复审计。
            for (const GeneratedStage& stage : generated) {
                result.audits.push_back(
                    stageAudit(stage, group.sourceRegionIndices));
            }
            return false;
        }
        for (std::size_t batchIndex = 0;
             batchIndex < stageIndices.size(); ++batchIndex) {
            fullStages[stageIndices[batchIndex]].emplace(
                std::move(generated[batchIndex]));
        }
        return true;
    };

    const auto transitionCannotImprove =
        [&](std::size_t previous, std::size_t current) {
            const State& predecessor = states[previous];
            const State& incumbent = states[current];
            if (!incumbent.reachable)
                return false;
            if (predecessor.distance >
                incumbent.distance + kDistanceTolerance) {
                return true;
            }
            return std::abs(
                       predecessor.distance - incumbent.distance) <=
                       kDistanceTolerance &&
                   predecessor.stageCount >= incumbent.stageCount;
        };

    const auto applyTransition =
        [&](std::size_t previous, std::size_t current,
            GeneratedStage&& rest, int auditIndex) {
            if (!rest.usable)
                return;
            const double candidateDistance =
                states[previous].distance +
                rest.summary.cuttingDistance;
            const int candidateStages =
                states[previous].stageCount +
                (rest.summary.operationCount == 0 ? 0 : 1);
            const bool better = !states[current].reachable ||
                candidateDistance <
                    states[current].distance - kDistanceTolerance ||
                (std::abs(
                     candidateDistance - states[current].distance) <=
                     kDistanceTolerance &&
                 candidateStages < states[current].stageCount);
            if (!better)
                return;
            states[current].distance = candidateDistance;
            states[current].stageCount = candidateStages;
            states[current].parent = static_cast<int>(previous);
            states[current].incomingAuditIndex = auditIndex;
            states[current].incoming = lightweightStage(
                rest,
                result.audits[static_cast<std::size_t>(auditIndex)]);
            states[current].reachable = true;
        };

    for (std::size_t index = 0; index < candidates.size(); ++index) {
        if (scopedControl.cancelled()) {
            result.cancelled = true;
            return finish();
        }
        if (!isRejectedStageEdge(
                rejectedEdges, -1, static_cast<int>(index))) {
            if (!prefetchFullStages(index)) {
                result.cancelled = true;
                return finish();
            }
            GeneratedStage full =
                std::move(fullStages[index].value());
            fullStages[index].reset();
            if (index + 1 == candidates.size()) {
                result.baselineSmallToolCuttingDistance =
                    full.summary.cuttingDistance;
            }
            result.audits.push_back(
                stageAudit(full, group.sourceRegionIndices));
            const int auditIndex =
                static_cast<int>(result.audits.size()) - 1;
            if (full.usable && full.summary.operationCount > 0) {
                states[index].distance = full.summary.cuttingDistance;
                states[index].stageCount = 1;
                states[index].parent = -1;
                states[index].incomingAuditIndex = auditIndex;
                states[index].incoming = lightweightStage(
                    full,
                    result.audits[static_cast<std::size_t>(auditIndex)]);
                states[index].reachable = true;
            }
        }

        std::vector<std::size_t> predecessors;
        predecessors.reserve(index);
        for (std::size_t previous = 0; previous < index; ++previous) {
            if (states[previous].reachable)
                predecessors.push_back(previous);
        }
        std::stable_sort(
            predecessors.begin(), predecessors.end(),
            [&](std::size_t left, std::size_t right) {
                if (std::abs(
                        states[left].distance - states[right].distance) >
                    kDistanceTolerance) {
                    return states[left].distance < states[right].distance;
                }
                return states[left].stageCount < states[right].stageCount;
            });

        std::size_t predecessorCursor = 0;
        bool firstBatch = true;
        while (predecessorCursor < predecessors.size()) {
            const std::size_t remaining =
                predecessors.size() - predecessorCursor;
            const std::size_t requestedTokens = firstBatch
                ? 0u
                : std::min(
                      maximumParallelStages - 1u,
                      remaining > 0u ? remaining - 1u : 0u);
            oap::toolpath::ToolpathParallelLease tokenLease(
                &scopedControl, requestedTokens, 0u);
            const std::size_t batchLimit = 1u + tokenLease.count();
            firstBatch = false;
            std::vector<std::size_t> batch;
            batch.reserve(batchLimit);
            while (predecessorCursor < predecessors.size() &&
                   batch.size() < batchLimit) {
                const std::size_t previous =
                    predecessors[predecessorCursor++];
                if (isRejectedStageEdge(
                        rejectedEdges, static_cast<int>(previous),
                        static_cast<int>(index))) {
                    ++result.prunedTransitionCount;
                    continue;
                }
                if (transitionCannotImprove(previous, index)) {
                    ++result.prunedTransitionCount;
                    continue;
                }
                batch.push_back(previous);
            }
            tokenLease.retain(
                batch.size() > 1u ? batch.size() - 1u : 0u);
            if (batch.empty())
                continue;

            std::vector<GeneratedStage> generated(batch.size());
            const auto generateTransition = [&](std::size_t batchIndex) {
                const std::size_t previous = batch[batchIndex];
                generated[batchIndex] = evaluateStage(
                    static_cast<int>(previous), index);
            };
            if (batch.size() == 1u) {
                generateTransition(0u);
            } else {
                ++result.parallelTransitionBatchCount;
                oap::toolpath::toolpathParallelFor(
                    batch.size(), generateTransition);
            }
            tokenLease.release();
            result.evaluatedTransitionCount +=
                static_cast<int>(batch.size());
            result.generatedStageCount += static_cast<int>(std::count_if(
                generated.begin(), generated.end(),
                [](const GeneratedStage& stage) {
                    return !stage.reusedPlanningEvaluation;
                }));
            result.reusedStageEvaluationCount +=
                static_cast<int>(std::count_if(
                    generated.begin(), generated.end(),
                    [](const GeneratedStage& stage) {
                        return stage.reusedPlanningEvaluation;
                    }));
            std::vector<int> auditIndices(batch.size(), -1);
            for (std::size_t batchIndex = 0;
                 batchIndex < batch.size(); ++batchIndex) {
                result.audits.push_back(
                    stageAudit(
                        generated[batchIndex],
                        group.sourceRegionIndices));
                auditIndices[batchIndex] =
                    static_cast<int>(result.audits.size()) - 1;
            }
            const bool cancelled = std::any_of(
                generated.begin(), generated.end(),
                [](const GeneratedStage& stage) {
                    return stage.batch.cancelled;
                }) || scopedControl.cancelled();
            if (cancelled) {
                result.cancelled = true;
                return finish();
            }
            for (std::size_t batchIndex = 0;
                 batchIndex < batch.size(); ++batchIndex) {
                applyTransition(
                    batch[batchIndex], index,
                    std::move(generated[batchIndex]),
                    auditIndices[batchIndex]);
            }
        }
    }

    const int finalIndex = static_cast<int>(candidates.size()) - 1;
    if (!states[static_cast<std::size_t>(finalIndex)].reachable) {
        result.error = finalPlanningFailure(
            candidates, group.regions.size(), result.audits);
        return finish();
    }

    std::vector<int> chain;
    for (int index = finalIndex; index >= 0;) {
        chain.push_back(index);
        index = states[static_cast<std::size_t>(index)].parent;
    }
    std::reverse(chain.begin(), chain.end());

    // Only the selected chain pays for full geometric safety validation. If a
    // selected edge is unsafe, blacklist that exact transition and rerun DP so
    // another full/rest-machining path can be chosen without weakening safety.
    oap::toolpath::PocketToolpathService service;
    for (const int index : chain) {
        State& state = states[static_cast<std::size_t>(index)];
        if (state.incoming.summary.operationCount == 0)
            continue;
        if (!state.incoming.materialized) {
            PocketAutomaticToolpathStageAudit evaluationAudit;
            if (state.incomingAuditIndex >= 0 &&
                state.incomingAuditIndex <
                    static_cast<int>(result.audits.size())) {
                evaluationAudit = result.audits[
                    static_cast<std::size_t>(state.incomingAuditIndex)];
            }
            const bool reusedEvaluation =
                state.incoming.reusedPlanningEvaluation;
            const int previous = state.parent;
            const double previousDiameter = (previous < 0 ||
                candidates[static_cast<std::size_t>(previous)].cornerRadius > 0.0)
                ? 0.0
                : candidates[static_cast<std::size_t>(previous)].diameter;
            GeneratedStage materialized = generateStage(
                candidates[static_cast<std::size_t>(index)],
                previousDiameter, group.regions, params,
                candidatePrefix(previous, static_cast<std::size_t>(index)),
                &scopedControl);
            materialized.reusedPlanningEvaluation = reusedEvaluation;
            state.incoming = std::move(materialized);
            ++result.generatedStageCount;
            ++result.selectedStageMaterializationCount;
            if (state.incomingAuditIndex >= 0 &&
                state.incomingAuditIndex <
                    static_cast<int>(result.audits.size())) {
                PocketAutomaticToolpathStageAudit materializedAudit =
                    stageAudit(
                        state.incoming, group.sourceRegionIndices);
                materializedAudit.generationMilliseconds +=
                    evaluationAudit.generationMilliseconds;
                materializedAudit.clearingMilliseconds +=
                    evaluationAudit.clearingMilliseconds;
                materializedAudit.sectionMilliseconds +=
                    evaluationAudit.sectionMilliseconds;
                materializedAudit.patternMilliseconds +=
                    evaluationAudit.patternMilliseconds;
                materializedAudit.moveAssemblyMilliseconds +=
                    evaluationAudit.moveAssemblyMilliseconds;
                materializedAudit.movePreparationMilliseconds +=
                    evaluationAudit.movePreparationMilliseconds;
                materializedAudit.moveLinkingMilliseconds +=
                    evaluationAudit.moveLinkingMilliseconds;
                materializedAudit.patternParallelBatchCount +=
                    evaluationAudit.patternParallelBatchCount;
                materializedAudit.patternParallelTaskCount +=
                    evaluationAudit.patternParallelTaskCount;
                materializedAudit.patternPeakParallelism = std::max(
                    materializedAudit.patternPeakParallelism,
                    evaluationAudit.patternPeakParallelism);
                materializedAudit.travelChoiceParallelBatchCount +=
                    evaluationAudit.travelChoiceParallelBatchCount;
                materializedAudit.travelChoiceParallelTaskCount +=
                    evaluationAudit.travelChoiceParallelTaskCount;
                materializedAudit.travelChoicePeakParallelism = std::max(
                    materializedAudit.travelChoicePeakParallelism,
                    evaluationAudit.travelChoicePeakParallelism);
                materializedAudit.sectionWireParallelBatchCount +=
                    evaluationAudit.sectionWireParallelBatchCount;
                materializedAudit.sectionWireParallelTaskCount +=
                    evaluationAudit.sectionWireParallelTaskCount;
                materializedAudit.sectionWirePeakParallelism = std::max(
                    materializedAudit.sectionWirePeakParallelism,
                    evaluationAudit.sectionWirePeakParallelism);
                materializedAudit.occtInternalParallelCount +=
                    evaluationAudit.occtInternalParallelCount;
                materializedAudit.occtSerialFallbackCount +=
                    evaluationAudit.occtSerialFallbackCount;
                materializedAudit.structuralValidationMilliseconds +=
                    evaluationAudit.structuralValidationMilliseconds;
                materializedAudit.safetyValidationMilliseconds +=
                    evaluationAudit.safetyValidationMilliseconds;
                result.audits[
                    static_cast<std::size_t>(state.incomingAuditIndex)] =
                    std::move(materializedAudit);
            }
            if (state.incoming.batch.cancelled ||
                scopedControl.cancelled()) {
                result.cancelled = true;
                return finish();
            }
        }
        ++result.safetyValidatedStageCount;
        const bool safetyPassed = service.validateCandidateSafety(
            group.regions, state.incoming.batch, &scopedControl);
        state.incoming.usable = state.incoming.usable && safetyPassed;
        if (state.incomingAuditIndex >= 0 &&
            state.incomingAuditIndex <
                static_cast<int>(result.audits.size())) {
            const std::size_t auditIndex =
                static_cast<std::size_t>(state.incomingAuditIndex);
            const PocketAutomaticToolpathStageAudit accumulatedAudit =
                result.audits[auditIndex];
            PocketAutomaticToolpathStageAudit validatedAudit =
                stageAudit(state.incoming, group.sourceRegionIndices);
            validatedAudit.generationMilliseconds =
                accumulatedAudit.generationMilliseconds;
            validatedAudit.clearingMilliseconds =
                accumulatedAudit.clearingMilliseconds;
            validatedAudit.sectionMilliseconds =
                accumulatedAudit.sectionMilliseconds;
            validatedAudit.patternMilliseconds =
                accumulatedAudit.patternMilliseconds;
            validatedAudit.moveAssemblyMilliseconds =
                accumulatedAudit.moveAssemblyMilliseconds;
            validatedAudit.movePreparationMilliseconds =
                accumulatedAudit.movePreparationMilliseconds;
            validatedAudit.moveLinkingMilliseconds =
                accumulatedAudit.moveLinkingMilliseconds;
            validatedAudit.patternParallelBatchCount =
                accumulatedAudit.patternParallelBatchCount;
            validatedAudit.patternParallelTaskCount =
                accumulatedAudit.patternParallelTaskCount;
            validatedAudit.patternPeakParallelism =
                accumulatedAudit.patternPeakParallelism;
            validatedAudit.travelChoiceParallelBatchCount =
                accumulatedAudit.travelChoiceParallelBatchCount;
            validatedAudit.travelChoiceParallelTaskCount =
                accumulatedAudit.travelChoiceParallelTaskCount;
            validatedAudit.travelChoicePeakParallelism =
                accumulatedAudit.travelChoicePeakParallelism;
            validatedAudit.sectionWireParallelBatchCount =
                accumulatedAudit.sectionWireParallelBatchCount;
            validatedAudit.sectionWireParallelTaskCount =
                accumulatedAudit.sectionWireParallelTaskCount;
            validatedAudit.sectionWirePeakParallelism =
                accumulatedAudit.sectionWirePeakParallelism;
            validatedAudit.occtInternalParallelCount =
                accumulatedAudit.occtInternalParallelCount;
            validatedAudit.occtSerialFallbackCount =
                accumulatedAudit.occtSerialFallbackCount;
            validatedAudit.structuralValidationMilliseconds =
                accumulatedAudit.structuralValidationMilliseconds;
            validatedAudit.safetyValidationMilliseconds +=
                accumulatedAudit.safetyValidationMilliseconds;
            result.audits[auditIndex] = std::move(validatedAudit);
        }
        if (state.incoming.batch.cancelled || scopedControl.cancelled()) {
            result.cancelled = true;
            return finish();
        }
        if (safetyPassed)
            continue;

        ++result.safetyRejectedStageCount;
        std::vector<RejectedStageEdge> nextRejectedEdges = rejectedEdges;
        nextRejectedEdges.push_back({state.parent, index});
        RegionGroupPlan replanned = planRegionGroup(
            group, params, operationPrefix, sharedControl,
            maximumParallelStages, nextRejectedEdges, regionCache,
            memoryTracker, evaluationCache);
        replanned.audits.insert(
            replanned.audits.begin(),
            std::make_move_iterator(result.audits.begin()),
            std::make_move_iterator(result.audits.end()));
        replanned.evaluatedTransitionCount +=
            result.evaluatedTransitionCount;
        replanned.prunedTransitionCount += result.prunedTransitionCount;
        replanned.generatedStageCount += result.generatedStageCount;
        replanned.reusedStageEvaluationCount +=
            result.reusedStageEvaluationCount;
        replanned.selectedStageMaterializationCount +=
            result.selectedStageMaterializationCount;
        replanned.safetyValidatedStageCount +=
            result.safetyValidatedStageCount;
        replanned.safetyRejectedStageCount +=
            result.safetyRejectedStageCount;
        replanned.planningReplanCount += result.planningReplanCount + 1;
        replanned.parallelFullStageBatchCount +=
            result.parallelFullStageBatchCount;
        replanned.prefetchedFullStageCount +=
            result.prefetchedFullStageCount;
        replanned.parallelTransitionBatchCount +=
            result.parallelTransitionBatchCount;
        if (result.baselineSmallToolCuttingDistance > 0.0) {
            replanned.baselineSmallToolCuttingDistance =
                result.baselineSmallToolCuttingDistance;
        }
        return replanned;
    }

    for (const int index : chain) {
        State& state = states[static_cast<std::size_t>(index)];
        if (state.incoming.batch.operations.empty())
            continue;
        remapOperationRegions(
            state.incoming, group.sourceRegionIndices);
        appendSelectedStage(result.batch, std::move(state.incoming));
    }
    result.selectedCuttingDistance =
        states[static_cast<std::size_t>(finalIndex)].distance;
    return finish();
}

// 单工序移动距离统计；cuttingOnly 时只计切削移动。
double operationDistance(
    const oap::toolpath::PocketToolpathOperation& operation,
    bool cuttingOnly)
{
    double result = 0.0;
    for (const auto& move : operation.clearing.moves) {
        if (cuttingOnly &&
            move.type != oap::toolpath::ToolpathMoveType::Cut) {
            continue;
        }
        result += move.from.Distance(move.to);
    }
    return result;
}

// 依据最终工序流（按刀具换刀点切分）重建面向界面的阶段汇总。
void rebuildStageSummaries(
    PocketAutomaticToolpathResult& result)
{
    result.stages.clear();
    for (const auto& operation : result.batch.operations) {
        const bool startsStage = result.stages.empty() ||
            result.stages.back().tool.toolNumber !=
                operation.tool.toolNumber;
        if (startsStage) {
            PocketAutomaticToolpathStage stage;
            stage.tool = operation.tool;
            stage.previousToolDiameter =
                operation.restMachiningPreviousToolDiameter;
            result.stages.push_back(std::move(stage));
        }
        PocketAutomaticToolpathStage& stage = result.stages.back();
        stage.previousToolDiameter = std::max(
            stage.previousToolDiameter,
            operation.restMachiningPreviousToolDiameter);
        stage.movementDistance += operationDistance(operation, false);
        stage.cuttingDistance += operationDistance(operation, true);
        ++stage.operationCount;
    }
}

} // namespace

// 对外入口：候选筛选 → 分组 → 逐组 DP 规划 → 合并排序 → 阶段汇总与总体验证。
PocketAutomaticToolpathResult PocketAutomaticToolpath::generate(
    const ToolLibrary& library,
    const std::vector<oap::toolpath::PocketRegion>& regions,
    const oap::toolpath::ToolpathParams& params,
    const std::string& operationPrefix,
    const oap::toolpath::ToolpathGenerationControl* control) const
{
    // Preserve the caller's budget for the final synchronous validation too,
    // after the region scheduler has restored its nested local scope.
    oap::toolpath::ToolpathParallelScope parallelScope(control);
    PocketAutomaticToolpathResult result;
    result.libraryId = library.id().toUtf8().toStdString();
    result.libraryName = library.name().toUtf8().toStdString();
    if (regions.empty()) {
        result.error = "automatic pocket roughing requires at least one region";
        return result;
    }

    MillingToolAudit candidateAudit;
    const auto candidates = millingTools(
        library, regions, params, candidateAudit);
    result.candidateToolCount =
        static_cast<int>(candidates.size());
    if (candidates.empty()) {
        result.error = QStringLiteral(
            "活动刀库“%1”没有可用的已分配铣刀：最大单层切深 %2 mm，"
            "最深刀具可达距离 %3 mm；已分配刀具 %4 把，其中铣刀 %5 把；"
            "几何无效 %6 把，不支持的切削几何 %7 把，刃长不足 %8 把，"
            "总长不足 %9 把。")
            .arg(library.name())
            .arg(candidateAudit.requiredCuttingEdgeLength, 0, 'f', 3)
            .arg(candidateAudit.requiredReach, 0, 'f', 3)
            .arg(candidateAudit.assignedToolCount)
            .arg(candidateAudit.assignedEndMillCount)
            .arg(candidateAudit.invalidGeometryCount)
            .arg(candidateAudit.unsupportedGeometryCount)
            .arg(candidateAudit.cuttingLengthTooShortCount)
            .arg(candidateAudit.overallLengthTooShortCount)
            .toUtf8().toStdString();
        return result;
    }

    detail::PocketPlanningScheduler scheduler(control);
    const auto& schedule = scheduler.audit();
    result.planningParallelism = static_cast<int>(schedule.initialParallelism);
    result.planningCpuParallelism = static_cast<int>(schedule.cpuParallelism);
    result.planningMemoryParallelism = static_cast<int>(schedule.memoryParallelism);
    result.planningAvailableMemoryMiB = schedule.availableMemoryBytes / (1024ull * 1024ull);
    std::vector<CompatibleRegionGroup> groups;
    groups.reserve(regions.size());
    for (std::size_t regionIndex = 0;
         regionIndex < regions.size(); ++regionIndex) {
        CompatibleRegionGroup group;
        const PocketRegionRequirements requirements =
            pocketRequirements(regions[regionIndex], params);
        group.candidates.reserve(candidates.size());
        for (const auto& candidate : candidates) {
            if (toolSupportsRegion(candidate, requirements) &&
                (group.candidates.empty() ||
                 std::abs(group.candidates.back().diameter - candidate.diameter) > kDiameterTolerance))
                group.candidates.push_back(candidate);
        }
        if (group.candidates.empty()) {
            result.error = QStringLiteral(
                "槽腔区域 %1 没有满足刃长和总长要求的已分配铣刀"
                "（单层切深 %2 mm，可达距离 %3 mm）。")
                .arg(static_cast<qulonglong>(regionIndex + 1))
                .arg(requirements.cuttingEdgeLength, 0, 'f', 3)
                .arg(requirements.reach, 0, 'f', 3)
                .toUtf8().toStdString();
            return result;
        }

        group.regions.push_back(regions[regionIndex]);
        group.sourceRegionIndices.push_back(
            static_cast<int>(regionIndex));
        groups.push_back(std::move(group));
    }

    result.batch.algorithmId = "pocket.auto_tool_library";
    result.batch.effectiveParams =
        oap::toolpath::bindPocketToolParameters(
            params, candidates.back());
    result.batch.effectiveParams.clearing
        .restMachiningPreviousToolDiameter = 0.0;

    // The scheduler owns resource admission and worker lifetime. Tool selection
    // writes indexed results; the fixed-order merge below remains independent
    // of task completion order.
    std::vector<RegionGroupPlan> groupPlans(groups.size());
    scheduler.run(groups.size(), [&](std::size_t groupIndex,
                                     const oap::toolpath::ToolpathGenerationControl& regionControl,
                                     std::size_t maximumParallelStages) {
        try {
            groupPlans[groupIndex] = planRegionGroup(
                groups[groupIndex], params,
                operationPrefix + "_G" + std::to_string(groupIndex + 1) + "_",
                regionControl, maximumParallelStages);
        } catch (const std::exception& exception) {
            groupPlans[groupIndex].error = exception.what();
        } catch (...) {
            groupPlans[groupIndex].error = "unknown exception while planning pocket region";
        }
        return groupPlans[groupIndex].peakWorkingSetGrowthBytes;
    });
    result.parallelRegionBatchCount = static_cast<int>(schedule.parallelStarts);
    result.planningPeakParallelism = static_cast<int>(schedule.peakTokenConcurrency);
    result.planningPeakRegionParallelism = static_cast<int>(schedule.peakRegionConcurrency);
    result.planningFinalParallelism = static_cast<int>(schedule.finalParallelism);
    result.planningConcurrencyAdjustmentCount = static_cast<int>(schedule.adjustmentCount);
    result.planningEstimatedRegionMemoryMiB = schedule.estimatedRegionBytes / (1024ull * 1024ull);
    result.planningObservedPeakRegionMemoryMiB = schedule.maximumObservedRegionBytes / (1024ull * 1024ull);
    result.planningBorrowedTokenCount = static_cast<int>(schedule.borrowedTokenCount);

    for (std::size_t groupIndex = 0;
         groupIndex < groups.size(); ++groupIndex) {
        RegionGroupPlan& groupPlan = groupPlans[groupIndex];
        result.evaluatedTransitionCount +=
            groupPlan.evaluatedTransitionCount;
        result.prunedTransitionCount +=
            groupPlan.prunedTransitionCount;
        result.generatedStageCount +=
            groupPlan.generatedStageCount;
        result.reusedStageEvaluationCount +=
            groupPlan.reusedStageEvaluationCount;
        result.selectedStageMaterializationCount +=
            groupPlan.selectedStageMaterializationCount;
        result.safetyValidatedStageCount +=
            groupPlan.safetyValidatedStageCount;
        result.safetyRejectedStageCount +=
            groupPlan.safetyRejectedStageCount;
        result.planningReplanCount +=
            groupPlan.planningReplanCount;
        result.parallelFullStageBatchCount +=
            groupPlan.parallelFullStageBatchCount;
        result.prefetchedFullStageCount +=
            groupPlan.prefetchedFullStageCount;
        result.parallelTransitionBatchCount +=
            groupPlan.parallelTransitionBatchCount;
        result.batch.materialSectionCacheHits +=
            groupPlan.batch.materialSectionCacheHits;
        result.batch.materialSectionCacheMisses +=
            groupPlan.batch.materialSectionCacheMisses;
        result.batch.materialSectionCacheEntries +=
            groupPlan.batch.materialSectionCacheEntries;
        result.baselineSmallToolCuttingDistance +=
            groupPlan.baselineSmallToolCuttingDistance;
        result.selectedCuttingDistance +=
            groupPlan.selectedCuttingDistance;
        result.stageAudits.insert(
            result.stageAudits.end(),
            std::make_move_iterator(groupPlan.audits.begin()),
            std::make_move_iterator(groupPlan.audits.end()));

        if (groupPlan.cancelled) {
            updatePlanningTimingSummary(result);
            result.cancelled = true;
            return result;
        }
        if (!groupPlan.error.empty()) {
            updatePlanningTimingSummary(result);
            result.error =
                "兼容区域组 [" +
                regionList(groups[groupIndex].sourceRegionIndices) +
                "] 规划失败：" + groupPlan.error;
            return result;
        }

        result.batch.diagnostics.insert(
            result.batch.diagnostics.end(),
            std::make_move_iterator(
                groupPlan.batch.diagnostics.begin()),
            std::make_move_iterator(
                groupPlan.batch.diagnostics.end()));
        result.batch.structuralValidationMilliseconds +=
            groupPlan.batch.structuralValidationMilliseconds;
        result.batch.safetyValidationMilliseconds +=
            groupPlan.batch.safetyValidationMilliseconds;
        result.batch.safetyValidation.append(
            std::move(groupPlan.batch.safetyValidation));
        result.batch.operations.insert(
            result.batch.operations.end(),
            std::make_move_iterator(
                groupPlan.batch.operations.begin()),
            std::make_move_iterator(
                groupPlan.batch.operations.end()));
    }
    updatePlanningTimingSummary(result);

    std::stable_sort(
        result.batch.operations.begin(),
        result.batch.operations.end(),
        [](const auto& left, const auto& right) {
            if (std::abs(
                    left.tool.diameter - right.tool.diameter) >
                kDiameterTolerance) {
                return left.tool.diameter > right.tool.diameter;
            }
            if (left.tool.toolNumber != right.tool.toolNumber)
                return left.tool.toolNumber < right.tool.toolNumber;
            if (std::abs(
                    left.restMachiningPreviousToolDiameter -
                    right.restMachiningPreviousToolDiameter) >
                kDiameterTolerance) {
                return left.restMachiningPreviousToolDiameter >
                    right.restMachiningPreviousToolDiameter;
            }
            return left.sourceRegionIndex <
                right.sourceRegionIndex;
        });
    rebuildStageSummaries(result);

    result.batch.regionOutcomes.resize(regions.size());
    for (auto& outcome : result.batch.regionOutcomes) {
        outcome.operationIndex = -1;
        outcome.failureIndex = -1;
    }
    for (std::size_t operationIndex = 0;
         operationIndex < result.batch.operations.size();
         ++operationIndex) {
        const int regionIndex =
            result.batch.operations[operationIndex]
                .sourceRegionIndex;
        if (regionIndex >= 0 &&
            regionIndex < static_cast<int>(regions.size())) {
            result.batch.regionOutcomes[
                static_cast<std::size_t>(regionIndex)]
                .operationIndex =
                static_cast<int>(operationIndex);
        }
    }
    result.batch.validation =
        oap::toolpath::PocketToolpathValidator().validate(
            result.batch.operations,
            result.batch.effectiveParams);
    result.batch.validation.append(result.batch.safetyValidation);
    result.batch.safetyValidationPerformed = true;
    return result;
}

} // namespace oap::tooling

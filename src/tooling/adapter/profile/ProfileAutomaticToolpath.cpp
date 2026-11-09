// ProfileAutomaticToolpath 实现（oap_toolpath_adapter）。
// 外轮廓多刀开粗：筛选可达铣刀、按全量/余料阶段最短切削距离规划，
// 输出从大刀到小刀的可执行工序，见 ProfileAutomaticToolpath.h。
#include "tooling/adapter/profile/ProfileAutomaticToolpath.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iterator>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <utility>

#include "tooling/adapter/AssignedToolContext.h"
#include "toolpath/core/PocketToolBinding.h"
#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/pocket/planning/PocketLayerSlicer.h"
#include "toolpath/profile/planning/ProfileContour.h"

namespace oap::tooling {
namespace {

constexpr double kDiameterTolerance = 1.0e-4;
constexpr double kDistanceTolerance = 1.0e-6;
constexpr std::size_t kMinimumParallelEvaluationCount = 2u;
using Clock = std::chrono::steady_clock;

double elapsedMilliseconds(const Clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(
               Clock::now() - start)
        .count();
}

struct ProfileRequirements {
    double cuttingEdgeLength = 0.0;
    double reach = 0.0;
};

double maximumLayerEngagement(
    double travel,
    const oap::toolpath::DepthParams& depth)
{
    if (!std::isfinite(travel) || travel <= 1.0e-9)
        return 0.0;
    double stepDown = depth.stepDown;
    if (!std::isfinite(stepDown) || stepDown <= 1.0e-9)
        stepDown = travel;
    return std::min(stepDown, travel);
}

ProfileRequirements profileRequirements(
    const std::vector<oap::toolpath::ProfileRegion>& regions,
    const oap::toolpath::ToolpathParams& params)
{
    ProfileRequirements result;
    for (const oap::toolpath::ProfileRegion& region : regions) {
        const double depth = std::isfinite(region.depth)
            ? std::max(0.0, region.depth) : 0.0;
        const double travel = params.depth.useFeatureDepth
            ? depth
            : std::abs(
                  params.depth.startDepth - params.depth.finalDepth);
        result.cuttingEdgeLength = std::max(
            result.cuttingEdgeLength,
            maximumLayerEngagement(travel, params.depth));
        result.reach = std::max(result.reach, depth);
    }
    return result;
}

bool supportsProfile(
    const oap::toolpath::ToolpathToolContext& tool,
    const ProfileRequirements& requirements)
{
    const double shankDiameter = tool.shankDiameter > 0.0
        ? tool.shankDiameter : tool.diameter;
    const double requiredCuttingLength =
        shankDiameter > tool.diameter + kDiameterTolerance
        ? requirements.reach
        : requirements.cuttingEdgeLength;
    return !(tool.cuttingEdgeLength > 0.0 &&
             tool.cuttingEdgeLength + kDiameterTolerance <
                 requiredCuttingLength) &&
           !(tool.overallLength > 0.0 &&
             tool.overallLength + kDiameterTolerance <
                 requirements.reach);
}

std::vector<oap::toolpath::ToolpathToolContext> candidateTools(
    const ToolLibrary& library,
    const ProfileRequirements& requirements,
    int& assignedCount,
    int& endMillCount)
{
    std::vector<oap::toolpath::ToolpathToolContext> result;
    for (const AssignedTool& assigned : library.assignedTools()) {
        ++assignedCount;
        if (!isSupportedMillingToolType(assigned.tool.type))
            continue;
        ++endMillCount;
        oap::toolpath::ToolpathToolContext tool =
            makeToolpathToolContext(assigned);
        if (!tool.valid || !hasSupportedMillingGeometry(assigned.tool) ||
            !supportsProfile(tool, requirements)) {
            continue;
        }
        result.push_back(std::move(tool));
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
    result.erase(
        std::unique(
            result.begin(), result.end(),
            [](const auto& left, const auto& right) {
                return std::abs(left.diameter - right.diameter) <=
                    kDiameterTolerance;
            }),
        result.end());
    return result;
}

double moveDistance(
    const oap::toolpath::ProfileToolpathBatch& batch,
    bool cuttingOnly)
{
    double result = 0.0;
    for (const auto& operation : batch.operations) {
        if (operation.contour.evaluationOnly) {
            result += operation.contour.estimatedCuttingDistance;
            continue;
        }
        for (const auto& move : operation.contour.moves) {
            if (cuttingOnly &&
                move.type != oap::toolpath::ToolpathMoveType::Cut) {
                continue;
            }
            result += move.from.Distance(move.to);
        }
    }
    return result;
}

double mandatoryFinishDistanceLowerBound(
    const oap::toolpath::ProfileToolpathBatch& batch)
{
    double result = 0.0;
    for (const auto& operation : batch.operations) {
        for (const auto& loop : operation.contour.loops) {
            if (loop.passKind != oap::toolpath::ProfilePassKind::Finish ||
                loop.world.size() < 2u) {
                continue;
            }
            for (std::size_t index = 1u;
                 index < loop.world.size(); ++index) {
                result += loop.world[index - 1u].Distance(
                    loop.world[index]);
            }
            if (loop.closed) {
                result += loop.world.back().Distance(
                    loop.world.front());
            }
        }
    }
    return result;
}

std::size_t retainedStringBytes(const std::string& value) noexcept
{
    return value.capacity() + 1u;
}

std::size_t retainedContourBytes(
    const oap::toolpath::ProfileContourResult& contour) noexcept
{
    std::size_t result = sizeof(contour);
    result += contour.loops.capacity() * sizeof(oap::toolpath::ProfileLoop);
    result += contour.moves.capacity() * sizeof(oap::toolpath::ToolpathMove);
    result += contour.primitives.capacity() *
        sizeof(oap::toolpath::ToolpathPrimitive);
    result += retainedStringBytes(contour.warning) +
        retainedStringBytes(contour.error);
    for (const auto& loop : contour.loops) {
        result += loop.points.capacity() * sizeof(gp_Pnt2d);
        result += loop.world.capacity() * sizeof(gp_Pnt);
    }
    for (const auto& primitive : contour.primitives) {
        result += primitive.fallbackMoves.capacity() *
            sizeof(oap::toolpath::ToolpathMove);
    }
    return result;
}

std::size_t retainedEvaluationBytes(
    const oap::toolpath::ProfileToolpathBatch& batch) noexcept
{
    std::size_t result = sizeof(batch) +
        retainedStringBytes(batch.algorithmId);
    result += batch.operations.capacity() *
        sizeof(oap::toolpath::ProfileToolpathOperation);
    result += batch.failures.capacity() *
        sizeof(oap::toolpath::ProfileGenerationFailure);
    result += batch.diagnostics.capacity() *
        sizeof(oap::toolpath::ToolpathDiagnostic);
    result += batch.safetyValidation.issues.capacity() *
        sizeof(oap::toolpath::ToolpathIssue);
    for (const auto& operation : batch.operations) {
        result += retainedStringBytes(operation.operationId) +
            retainedStringBytes(operation.algorithmId) +
            retainedStringBytes(operation.rootFaceId) +
            retainedStringBytes(operation.rootWireId);
        result += retainedContourBytes(operation.contour);
    }
    for (const auto& failure : batch.failures)
        result += retainedContourBytes(failure.contour);
    for (const auto& diagnostic : batch.diagnostics) {
        result += retainedStringBytes(diagnostic.message) +
            retainedStringBytes(diagnostic.operationId);
    }
    for (const auto& issue : batch.safetyValidation.issues) {
        result += retainedStringBytes(issue.code) +
            retainedStringBytes(issue.message) +
            retainedStringBytes(issue.operationId);
    }
    return result;
}

std::size_t evaluationLoopCount(
    const oap::toolpath::ProfileToolpathBatch& batch) noexcept
{
    std::size_t result = 0u;
    for (const auto& operation : batch.operations)
        result += operation.contour.loops.size();
    return result;
}

bool benignNoRestFailure(
    const oap::toolpath::ProfileGenerationFailure& failure)
{
    return failure.contour.ok && failure.contour.restMachining &&
           failure.contour.noRestMaterial;
}

struct GeneratedStage {
    oap::toolpath::ProfileToolpathBatch batch;
    oap::toolpath::ToolpathValidationReport validation;
    ProfileAutomaticToolpathStage summary;
    ProfileAutomaticToolpathStageAudit audit;
    bool usable = false;
};

GeneratedStage summarizeStage(
    const oap::toolpath::ToolpathToolContext& tool,
    double previousDiameter,
    bool finishPass,
    oap::toolpath::ProfileToolpathBatch batch,
    bool evaluationOnly)
{
    GeneratedStage result;
    result.batch = std::move(batch);
    for (oap::toolpath::ProfileToolpathOperation& operation :
         result.batch.operations) {
        operation.tool = tool;
        operation.restMachiningPreviousToolDiameter =
            previousDiameter;
        operation.finishPass = finishPass;
    }
    // ProfileToolpathService 已对每道候选工序执行几何安全验证；完整程序的
    // 结构验证由桌面生成用例在提交预览前统一执行。
    // Evaluation batches deliberately carry no moves and no safety report.
    // Materialized selected stages retain the service's complete validation.
    result.validation = result.batch.safetyValidation;

    result.summary.tool = tool;
    result.summary.previousToolDiameter = previousDiameter;
    result.summary.finishPass = finishPass;
    result.summary.operationCount =
        static_cast<int>(result.batch.operations.size());
    result.summary.generationMilliseconds =
        result.batch.metrics.totalMilliseconds;
    result.summary.movementDistance = moveDistance(result.batch, false);
    result.summary.cuttingDistance = moveDistance(result.batch, true);
    result.summary.mandatoryCuttingDistanceLowerBound =
        mandatoryFinishDistanceLowerBound(result.batch);
    result.audit.stage = result.summary;
    result.audit.evaluationOnly = evaluationOnly;
    result.audit.failedRegionCount =
        static_cast<int>(result.batch.failures.size());
    result.audit.validationErrorCount = result.validation.errorCount;
    for (const auto& failure : result.batch.failures) {
        if (benignNoRestFailure(failure)) {
            ++result.summary.noRestMaterialRegionCount;
            ++result.audit.stage.noRestMaterialRegionCount;
            continue;
        }
        std::ostringstream detail;
        detail << "region " << failure.regionIndex + 1 << ": "
               << failure.contour.error;
        result.audit.failureDetails.push_back(detail.str());
    }
    const bool failuresAcceptable = std::all_of(
        result.batch.failures.begin(), result.batch.failures.end(),
        benignNoRestFailure);
    const bool hasWork = !result.batch.operations.empty();
    const bool emptyBenignRest =
        previousDiameter > tool.diameter + kDiameterTolerance &&
        !hasWork && failuresAcceptable && !result.batch.failures.empty();
    result.usable = !result.batch.cancelled && failuresAcceptable &&
        (!result.batch.safetyValidationPerformed ||
         result.validation.ok()) &&
        (hasWork || emptyBenignRest);
    result.audit.usable = result.usable;
    return result;
}

GeneratedStage generateStage(
    const oap::toolpath::ToolpathToolContext& tool,
    double previousDiameter,
    bool finishPass,
    const std::vector<oap::toolpath::ProfileRegion>& regions,
    const oap::toolpath::ToolpathParams& base,
    const std::string& prefix,
    const oap::toolpath::ToolpathGenerationControl* control,
    bool evaluationOnly)
{
    oap::toolpath::ToolpathParams effective =
        oap::toolpath::bindPocketToolParameters(base, tool);
    effective.clearing.restMachiningPreviousToolDiameter =
        previousDiameter;
    effective.profile.finishPass = finishPass;
    oap::toolpath::ToolpathGenerationControl stageControl =
        control ? *control :
                  oap::toolpath::ToolpathGenerationControl{};
    stageControl.profileEvaluationOnly = evaluationOnly;
    return summarizeStage(
        tool, previousDiameter, finishPass,
        oap::toolpath::ProfileToolpathService().generate(
            regions, effective, prefix, &stageControl),
        evaluationOnly);
}

GeneratedStage materializeStage(
    const oap::toolpath::ToolpathToolContext& tool,
    double previousDiameter,
    bool finishPass,
    const std::vector<oap::toolpath::ProfileRegion>& regions,
    oap::toolpath::ProfileToolpathBatch evaluation,
    const oap::toolpath::ToolpathGenerationControl* control)
{
    return summarizeStage(
        tool, previousDiameter, finishPass,
        oap::toolpath::ProfileToolpathService().materializeEvaluated(
            regions, std::move(evaluation), control),
        false);
}

void appendSelectedStage(
    oap::toolpath::ProfileToolpathBatch& output,
    GeneratedStage& stage)
{
    output.diagnostics.insert(
        output.diagnostics.end(),
        std::make_move_iterator(stage.batch.diagnostics.begin()),
        std::make_move_iterator(stage.batch.diagnostics.end()));
    output.safetyValidation.append(
        std::move(stage.batch.safetyValidation));
    output.metrics.append(stage.batch.metrics);
    output.operations.insert(
        output.operations.end(),
        std::make_move_iterator(stage.batch.operations.begin()),
        std::make_move_iterator(stage.batch.operations.end()));
}

struct ProfilePlanningGroup {
    ProfileRequirements requirements;
    std::vector<std::size_t> indices;
    std::vector<oap::toolpath::ProfileRegion> regions;
};

std::vector<ProfilePlanningGroup> planningGroups(
    const std::vector<oap::toolpath::ProfileRegion>& regions,
    const oap::toolpath::ToolpathParams& params)
{
    std::vector<ProfilePlanningGroup> groups;
    for (std::size_t index = 0; index < regions.size(); ++index) {
        const auto requirements = profileRequirements({regions[index]}, params);
        auto group = std::find_if(groups.begin(), groups.end(), [&](const auto& existing) {
            return existing.regions.front().openDir.IsEqual(regions[index].openDir, 1.0e-6) &&
                std::abs(existing.requirements.reach - requirements.reach) <= kDistanceTolerance &&
                std::abs(existing.requirements.cuttingEdgeLength - requirements.cuttingEdgeLength) <= kDistanceTolerance;
        });
        if (group == groups.end()) {
            groups.push_back({requirements, {}, {}});
            group = std::prev(groups.end());
        }
        group->indices.push_back(index);
        group->regions.push_back(regions[index]);
    }
    return groups;
}

std::string groupDescription(const ProfilePlanningGroup& group)
{
    const auto& first = group.regions.front();
    std::ostringstream text;
    text << "region " << group.indices.front() + 1 << " [" << first.source.rootFaceId
         << "] axis=(" << first.openDir.X() << ',' << first.openDir.Y() << ',' << first.openDir.Z()
         << "), depth=" << group.requirements.reach << " mm";
    return text.str();
}

void appendPlanningGroup(ProfileAutomaticToolpathResult& output,
                         ProfileAutomaticToolpathResult& group)
{
    // Counts cover (group, candidate) pairs; independent groups are planned
    // sequentially and each retains the existing parallel evaluation budget.
    output.candidateToolCount += group.candidateToolCount;
    output.evaluatedTransitionCount += group.evaluatedTransitionCount;
    output.costBoundPrunedTransitionCount += group.costBoundPrunedTransitionCount;
    output.evaluatedStageCount += group.evaluatedStageCount;
    output.materializedStageCount += group.materializedStageCount;
    output.promotedStageCount += group.promotedStageCount;
    output.cachedEvaluationStageCount += group.cachedEvaluationStageCount;
    output.releasedEvaluationStageCount += group.releasedEvaluationStageCount;
    output.evictedEvaluationStageCount += group.evictedEvaluationStageCount;
    output.reevaluatedEvictedStageCount += group.reevaluatedEvictedStageCount;
    output.transferredEvaluationStageCount += group.transferredEvaluationStageCount;
    output.rejectedTransitionCount += group.rejectedTransitionCount;
    output.parallelEvaluationBatchCount += group.parallelEvaluationBatchCount;
    output.parallelEvaluatedStageCount += group.parallelEvaluatedStageCount;
    output.evaluationPeakParallelism = std::max(output.evaluationPeakParallelism, group.evaluationPeakParallelism);
    output.peakRetainedEvaluationLoopCount = std::max(output.peakRetainedEvaluationLoopCount, group.peakRetainedEvaluationLoopCount);
    output.peakRetainedEvaluationBytes = std::max(output.peakRetainedEvaluationBytes, group.peakRetainedEvaluationBytes);
    output.evaluationMilliseconds += group.evaluationMilliseconds;
    output.materializationMilliseconds += group.materializationMilliseconds;
    output.candidateGeometryCacheHitCount += group.candidateGeometryCacheHitCount;
    output.candidateGeometryCacheMissCount += group.candidateGeometryCacheMissCount;
    output.candidateGeometryCacheEntryCount += group.candidateGeometryCacheEntryCount;
    output.candidateGeometryCacheEstimatedBytes = std::max(output.candidateGeometryCacheEstimatedBytes, group.candidateGeometryCacheEstimatedBytes);
    output.stages.insert(output.stages.end(), std::make_move_iterator(group.stages.begin()), std::make_move_iterator(group.stages.end()));
    output.stageAudits.insert(output.stageAudits.end(), std::make_move_iterator(group.stageAudits.begin()), std::make_move_iterator(group.stageAudits.end()));
    if (output.batch.operations.empty() || group.batch.effectiveParams.tool.diameter < output.batch.effectiveParams.tool.diameter)
        output.batch.effectiveParams = group.batch.effectiveParams;
    GeneratedStage selected;
    selected.batch = std::move(group.batch);
    appendSelectedStage(output.batch, selected);
}

} // namespace

bool ProfileAutomaticToolpath::hasCandidateTool(
    const ToolLibrary& library,
    const oap::toolpath::ProfileRegion& region,
    const oap::toolpath::ToolpathParams& params)
{
    int assigned = 0, endMills = 0;
    const auto candidates = candidateTools(library, profileRequirements({region}, params),
                                           assigned, endMills);
    // Unknown lengths may retain legacy generation diagnostics, but cannot
    // justify eliminating another setup during automatic direction planning.
    return std::any_of(candidates.begin(), candidates.end(), [](const auto& tool) {
        return std::isfinite(tool.cuttingEdgeLength) && tool.cuttingEdgeLength > 0 &&
               std::isfinite(tool.overallLength) && tool.overallLength > 0;
    });
}

ProfileAutomaticToolpathResult ProfileAutomaticToolpath::generate(
    const ToolLibrary& library,
    const std::vector<oap::toolpath::ProfileRegion>& regions,
    const oap::toolpath::ToolpathParams& params,
    const std::string& operationPrefix,
    const oap::toolpath::ToolpathGenerationControl* control) const
{
    ProfileAutomaticToolpathResult result;
    const Clock::time_point totalStart = Clock::now();
    oap::toolpath::ToolpathGenerationControl automaticControl =
        control ? *control :
                  oap::toolpath::ToolpathGenerationControl{};
    oap::toolpath::ensureToolpathParallelBudget(automaticControl);
    oap::toolpath::ToolpathParallelScope parallelScope(&automaticControl);
    if (!automaticControl.pocketMaterialSectionCache) {
        automaticControl.pocketMaterialSectionCache =
            std::make_shared<
                oap::toolpath::PocketMaterialSectionCache>();
    }
    automaticControl.profileCandidateGeometryCache.reset();
    const oap::toolpath::ToolpathGenerationControl* effectiveControl =
        &automaticControl;
    oap::toolpath::PocketMaterialSectionCache& sectionCache =
        *automaticControl.pocketMaterialSectionCache;
    oap::toolpath::ProfileCandidateGeometryCache* geometryCache = nullptr;
    const std::size_t cacheHitsBefore = sectionCache.hitCount();
    const std::size_t cacheMissesBefore = sectionCache.missCount();
    const auto finish = [&]() -> ProfileAutomaticToolpathResult {
        result.materialSectionCacheHitCount =
            sectionCache.hitCount() - cacheHitsBefore;
        result.materialSectionCacheMissCount =
            sectionCache.missCount() - cacheMissesBefore;
        if (geometryCache) {
            result.candidateGeometryCacheHitCount =
                geometryCache->hitCount();
            result.candidateGeometryCacheMissCount =
                geometryCache->missCount();
            result.candidateGeometryCacheEntryCount =
                geometryCache->entryCount();
            result.candidateGeometryCacheEstimatedBytes =
                geometryCache->estimatedByteCount();
        }
        result.totalMilliseconds = elapsedMilliseconds(totalStart);
        return std::move(result);
    };
    result.libraryId = library.id().toUtf8().toStdString();
    result.libraryName = library.name().toUtf8().toStdString();
    result.evaluationCacheByteLimit =
        automaticControl.profileEvaluationCacheByteLimit;
    if (regions.empty()) {
        result.error = "automatic profile planning requires at least one region";
        return finish();
    }
    if (params.profile.machiningMode !=
        oap::toolpath::ProfileMachiningMode::ExternalClearing) {
        result.error =
            "automatic profile tool selection requires external-clearing mode";
        return finish();
    }
    if (std::any_of(
            regions.begin(), regions.end(),
            [](const oap::toolpath::ProfileRegion& region) {
                return !region.stockBoundary.closed ||
                       region.stockBoundary.points.size() < 3;
            })) {
        result.error =
            "automatic profile rest machining requires a valid stock boundary for every region";
        return finish();
    }

    const auto groups = planningGroups(regions, params);
    if (groups.size() > 1u) {
        // Fail the selected scope atomically. Preflight every group so a long
        // side profile reports its own requirement rather than filtering the
        // tools for otherwise machinable top profiles.
        for (const auto& group : groups) {
            if (effectiveControl->cancelled()) {
                result.cancelled = true;
                return finish();
            }
            int assigned = 0, endMills = 0;
            if (candidateTools(library, group.requirements, assigned, endMills).empty()) {
                result.error += groupDescription(group) + ": no assigned milling cutter (flat, ball or bull nose) satisfies " +
                    std::to_string(group.requirements.cuttingEdgeLength) + " mm cutting-edge engagement and " +
                    std::to_string(group.requirements.reach) + " mm reach\n";
            }
        }
        if (!result.error.empty()) return finish();
        for (std::size_t index = 0; index < groups.size(); ++index) {
            const auto& group = groups[index];
            auto generated = generate(library, group.regions, params,
                operationPrefix + "G" + std::to_string(index + 1) + "_", effectiveControl);
            if (!generated.ok()) {
                result.cancelled = generated.cancelled;
                result.error = groupDescription(group) + ": " + generated.error;
                result.batch = {};
                result.stages.clear();
                return finish();
            }
            for (auto& operation : generated.batch.operations) {
                const auto local = operation.sourceRegionIndex;
                if (local < 0 || static_cast<std::size_t>(local) >= group.indices.size()) {
                    result.error = "automatic profile group lost its source region mapping";
                    result.batch = {};
                    result.stages.clear();
                    return finish();
                }
                operation.sourceRegionIndex = static_cast<int>(group.indices[static_cast<std::size_t>(local)]);
            }
            appendPlanningGroup(result, generated);
        }
        result.batch.algorithmId = "profile.auto_tool_library";
        result.batch.safetyValidationPerformed = true;
        result.validation = result.batch.safetyValidation;
        return finish();
    }

    int assignedCount = 0;
    int endMillCount = 0;
    const ProfileRequirements requirements =
        profileRequirements(regions, params);
    std::vector<oap::toolpath::ToolpathToolContext> candidates =
        candidateTools(
            library, requirements,
            assignedCount, endMillCount);
    result.candidateToolCount = static_cast<int>(candidates.size());
    if (candidates.empty()) {
        std::ostringstream message;
        message << groupDescription(groups.front()) << ": active library has no assigned milling cutter (flat, ball or bull nose) satisfying "
                << requirements.cuttingEdgeLength
                << " mm cutting-edge engagement and "
                << requirements.reach << " mm reach"
                << " (assigned=" << assignedCount
                << ", end mills=" << endMillCount << ')';
        result.error = message.str();
        return finish();
    }
    if (candidates.size() > 1u) {
        // Keys use stable ProfileRegion addresses. Keep this cache scoped to
        // one automatic invocation; a single-tool request avoids its setup
        // and sweep-precomputation overhead entirely.
        automaticControl.profileCandidateGeometryCache =
            std::make_shared<
                oap::toolpath::ProfileCandidateGeometryCache>();
        geometryCache =
            automaticControl.profileCandidateGeometryCache.get();
    }

    struct CachedEvaluation {
        ProfileAutomaticToolpathStage summary;
        ProfileAutomaticToolpathStageAudit audit;
        std::optional<oap::toolpath::ProfileToolpathBatch> batch;
        std::size_t retainedBytes = 0u;
        std::size_t retainedLoopCount = 0u;
        bool usable = false;
        bool cancelled = false;
    };
    struct State {
        double cuttingDistance =
            std::numeric_limits<double>::infinity();
        int stageCount = std::numeric_limits<int>::max();
        int parent = -2;
        bool reachable = false;
    };
    std::size_t finalIndex = candidates.size() - 1u;
    const std::size_t evaluationColumnCount = candidates.size();
    std::vector<std::optional<CachedEvaluation>> evaluationCache(
        (candidates.size() + 1u) * evaluationColumnCount);
    std::vector<unsigned char> costBoundPruned(
        evaluationCache.size(), 0u);
    std::size_t retainedEvaluationLoopCount = 0u;
    std::size_t retainedEvaluationByteCount = 0u;
    std::size_t evaluationEvictionCursor = 0u;
    const auto evaluationIndex = [&](int previous, std::size_t current) {
        return static_cast<std::size_t>(previous + 1) *
            evaluationColumnCount + current;
    };
    const auto stagePrefix = [&](int previous, std::size_t current) {
        std::string prefix = operationPrefix + "T" +
            std::to_string(candidates[current].toolNumber);
        if (previous < 0)
            return prefix + "_FULL_";
        return prefix + "_REST_FROM_T" +
            std::to_string(
                candidates[static_cast<std::size_t>(previous)].toolNumber) +
            "_";
    };
    struct EvaluationEdge {
        int previous = -1;
        std::size_t current = 0u;
    };

    std::mutex callbackMutex;
    oap::toolpath::ToolpathGenerationControl parallelEvaluationControl =
        automaticControl;
    if (parallelEvaluationControl.reportProgress) {
        const auto reportProgress =
            parallelEvaluationControl.reportProgress;
        parallelEvaluationControl.reportProgress =
            [reportProgress, &callbackMutex](std::size_t completed,
                                             std::size_t total) {
                std::lock_guard<std::mutex> lock(callbackMutex);
                reportProgress(completed, total);
            };
    }
    if (parallelEvaluationControl.reportStatus) {
        const auto reportStatus = parallelEvaluationControl.reportStatus;
        parallelEvaluationControl.reportStatus =
            [reportStatus, &callbackMutex](
                const oap::toolpath::ToolpathExecutionStatus& status) {
                std::lock_guard<std::mutex> lock(callbackMutex);
                reportStatus(status);
            };
    }
    // Candidate, region and layer work share one process-wide worker budget.
    // Small candidate batches can lend the remaining slots to their regions.

    const auto evaluateUncached =
        [&](const EvaluationEdge& edge,
            const oap::toolpath::ToolpathGenerationControl* stageControl) {
        const int previous = edge.previous;
        const std::size_t current = edge.current;
        const double previousDiameter = (previous < 0 ||
                candidates[static_cast<std::size_t>(previous)].cornerRadius > 0.0)
            ? 0.0
            : candidates[static_cast<std::size_t>(previous)].diameter;
        const bool finishPass =
            current == finalIndex && params.profile.finishPass;
        CachedEvaluation cached;
        try {
            GeneratedStage stage = generateStage(
                candidates[current], previousDiameter, finishPass,
                regions, params, stagePrefix(previous, current),
                stageControl, true);
            cached.summary = stage.summary;
            cached.audit = stage.audit;
            cached.usable = stage.usable;
            cached.cancelled = stage.batch.cancelled;
            if (stage.usable) {
                cached.batch.emplace(std::move(stage.batch));
                cached.retainedBytes = retainedEvaluationBytes(
                    *cached.batch);
                cached.retainedLoopCount = evaluationLoopCount(
                    *cached.batch);
            }
        } catch (const std::exception& exception) {
            cached.summary.tool = candidates[current];
            cached.summary.previousToolDiameter = previousDiameter;
            cached.summary.finishPass = finishPass;
            cached.audit.stage = cached.summary;
            cached.audit.evaluationOnly = true;
            cached.audit.failedRegionCount =
                static_cast<int>(regions.size());
            cached.audit.failureDetails.push_back(exception.what());
        } catch (...) {
            cached.summary.tool = candidates[current];
            cached.summary.previousToolDiameter = previousDiameter;
            cached.summary.finishPass = finishPass;
            cached.audit.stage = cached.summary;
            cached.audit.evaluationOnly = true;
            cached.audit.failedRegionCount =
                static_cast<int>(regions.size());
            cached.audit.failureDetails.push_back(
                "unknown profile candidate evaluation exception");
        }
        return cached;
    };
    const auto releaseCachedBatch =
        [&](CachedEvaluation& cached, bool evicted) {
        if (!cached.batch)
            return;
        retainedEvaluationLoopCount -= cached.retainedLoopCount;
        retainedEvaluationByteCount -= cached.retainedBytes;
        cached.batch.reset();
        cached.retainedLoopCount = 0u;
        cached.retainedBytes = 0u;
        ++result.releasedEvaluationStageCount;
        if (evicted)
            ++result.evictedEvaluationStageCount;
    };
    const auto enforceEvaluationCacheLimit = [&] {
        const std::size_t limit =
            automaticControl.profileEvaluationCacheByteLimit;
        if (limit == 0u || retainedEvaluationByteCount <= limit)
            return;
        while (retainedEvaluationByteCount > limit) {
            std::size_t residentCount = 0u;
            for (const auto& cached : evaluationCache) {
                if (cached && cached->batch)
                    ++residentCount;
            }
            // Keep one oversize batch so a very small diagnostic limit does
            // not create an immediate evaluate/evict loop.
            if (residentCount <= 1u)
                return;
            bool released = false;
            for (std::size_t offset = 0u;
                 offset < evaluationCache.size(); ++offset) {
                const std::size_t index =
                    (evaluationEvictionCursor + offset) %
                    evaluationCache.size();
                auto& cached = evaluationCache[index];
                if (!cached || !cached->batch)
                    continue;
                releaseCachedBatch(*cached, true);
                evaluationEvictionCursor =
                    (index + 1u) % evaluationCache.size();
                released = true;
                break;
            }
            if (!released)
                return;
        }
    };
    const auto ensureEvaluated =
        [&](const std::vector<EvaluationEdge>& requestedEdges) {
        std::vector<EvaluationEdge> missingEdges;
        missingEdges.reserve(requestedEdges.size());
        for (const EvaluationEdge& edge : requestedEdges) {
            if (!evaluationCache[
                    evaluationIndex(edge.previous, edge.current)]) {
                missingEdges.push_back(edge);
            }
        }
        if (missingEdges.empty())
            return true;

        const Clock::time_point evaluationStart = Clock::now();
        const std::size_t desiredParallelism =
            missingEdges.size() >= kMinimumParallelEvaluationCount
            ? oap::toolpath::toolpathParallelismFor(missingEdges.size())
            : 1u;
        oap::toolpath::ToolpathParallelLease lease(
            effectiveControl, desiredParallelism - 1u,
            desiredParallelism - 1u);
        const std::size_t workerCount = std::min(
            desiredParallelism, lease.parallelism());
        const oap::toolpath::ToolpathGenerationControl* stageControl =
            workerCount > 1u
            ? &parallelEvaluationControl : effectiveControl;
        std::vector<CachedEvaluation> generated(missingEdges.size());
        oap::toolpath::toolpathParallelForWorkers(
            missingEdges.size(), workerCount,
            [&](std::size_t index) {
                generated[index] = evaluateUncached(
                    missingEdges[index], stageControl);
            });
        result.evaluationMilliseconds +=
            elapsedMilliseconds(evaluationStart);
        result.evaluationPeakParallelism = std::max(
            result.evaluationPeakParallelism, workerCount);
        if (workerCount > 1u) {
            ++result.parallelEvaluationBatchCount;
            result.parallelEvaluatedStageCount +=
                static_cast<int>(missingEdges.size());
        }

        bool cancelled = false;
        // Merge by the stable DP edge order, never by worker completion order.
        for (std::size_t index = 0u;
             index < missingEdges.size(); ++index) {
            const EvaluationEdge& edge = missingEdges[index];
            const std::size_t cacheIndex =
                evaluationIndex(edge.previous, edge.current);
            if (costBoundPruned[cacheIndex] != 0u) {
                costBoundPruned[cacheIndex] = 0u;
                --result.costBoundPrunedTransitionCount;
            }
            evaluationCache[cacheIndex] =
                std::move(generated[index]);
            if (evaluationCache[cacheIndex]->batch) {
                ++result.cachedEvaluationStageCount;
                retainedEvaluationLoopCount +=
                    evaluationCache[cacheIndex]->retainedLoopCount;
                retainedEvaluationByteCount +=
                    evaluationCache[cacheIndex]->retainedBytes;
                result.peakRetainedEvaluationLoopCount = std::max(
                    result.peakRetainedEvaluationLoopCount,
                    retainedEvaluationLoopCount);
                result.peakRetainedEvaluationBytes = std::max(
                    result.peakRetainedEvaluationBytes,
                    retainedEvaluationByteCount);
            }
            ++result.evaluatedStageCount;
            if (edge.previous >= 0)
                ++result.evaluatedTransitionCount;
            result.stageAudits.push_back(
                evaluationCache[cacheIndex]->audit);
            cancelled = cancelled ||
                evaluationCache[cacheIndex]->cancelled;
        }
        enforceEvaluationCacheLimit();
        return !cancelled && !effectiveControl->cancelled();
    };
    const auto cachedEvaluation =
        [&](int previous, std::size_t current)
            -> CachedEvaluation* {
        auto& cached =
            evaluationCache[evaluationIndex(previous, current)];
        return cached ? &*cached : nullptr;
    };

    std::vector<std::pair<int, std::size_t>> rejectedEdges;
    const auto edgeRejected =
        [&](int previous, std::size_t current) {
        return std::find(
                   rejectedEdges.begin(), rejectedEdges.end(),
                   std::make_pair(previous, current)) !=
            rejectedEdges.end();
    };
    std::vector<GeneratedStage> selectedStages;
    bool selectedChainMaterialized = false;
    const std::size_t maximumPlanningAttempts =
        2u * evaluationCache.size() + candidates.size() + 1u;
    for (std::size_t attempt = 0u;
         attempt < maximumPlanningAttempts; ++attempt) {
        if (effectiveControl->cancelled()) {
            result.cancelled = true;
            return finish();
        }
        std::vector<State> states(candidates.size());
        // Every cutter can be a valid starting point. Warm the shared section
        // cache with the largest cutter, then evaluate the remaining full
        // stages as one deterministic bounded-parallel batch.
        if (!edgeRejected(-1, 0u) &&
            !ensureEvaluated({EvaluationEdge{-1, 0u}})) {
            result.cancelled = true;
            return finish();
        }
        std::vector<EvaluationEdge> remainingFullEdges;
        remainingFullEdges.reserve(finalIndex);
        for (std::size_t current = 1u;
             current <= finalIndex; ++current) {
            if (!edgeRejected(-1, current))
                remainingFullEdges.push_back({-1, current});
        }
        if (!ensureEvaluated(remainingFullEdges)) {
            result.cancelled = true;
            return finish();
        }

        const auto applyEvaluation =
            [&](State& target, int previous, std::size_t current) {
            const CachedEvaluation* evaluation =
                cachedEvaluation(previous, current);
            if (!evaluation || !evaluation->usable)
                return;
            double distance = evaluation->summary.cuttingDistance;
            int stageCount =
                evaluation->summary.operationCount == 0 ? 0 : 1;
            if (previous >= 0) {
                const State& parent =
                    states[static_cast<std::size_t>(previous)];
                if (!parent.reachable)
                    return;
                distance += parent.cuttingDistance;
                stageCount += parent.stageCount;
            }
            if (!target.reachable ||
                distance < target.cuttingDistance -
                    kDistanceTolerance ||
                (std::abs(distance - target.cuttingDistance) <=
                     kDistanceTolerance &&
                 stageCount < target.stageCount)) {
                target.cuttingDistance = distance;
                target.stageCount = stageCount;
                target.parent = previous;
                target.reachable = true;
            }
        };
        const auto cannotImprove =
            [&](const State& parent, const State& incumbent,
                double transitionDistanceLowerBound,
                int transitionStageLowerBound) {
            if (automaticControl.profileExhaustiveAutomaticEvaluation)
                return false;
            if (!incumbent.reachable)
                return false;
            // Every generated transition has non-negative cutting distance
            // and adds either zero or one stage. The parent state is therefore
            // an admissible lower bound for the complete transition.
            const double distanceLowerBound =
                parent.cuttingDistance +
                transitionDistanceLowerBound;
            const int stageLowerBound =
                parent.stageCount + transitionStageLowerBound;
            if (distanceLowerBound >
                incumbent.cuttingDistance + kDistanceTolerance) {
                return true;
            }
            return std::abs(
                       distanceLowerBound -
                       incumbent.cuttingDistance) <=
                    kDistanceTolerance &&
                stageLowerBound >= incumbent.stageCount;
        };
        const auto markCostBoundPruned =
            [&](int previous, std::size_t current) {
            const std::size_t index =
                evaluationIndex(previous, current);
            if (!evaluationCache[index] &&
                costBoundPruned[index] == 0u) {
                costBoundPruned[index] = 1u;
                ++result.costBoundPrunedTransitionCount;
            }
        };

        for (std::size_t current = 0u;
             current <= finalIndex; ++current) {
            const CachedEvaluation* fullEvaluation =
                cachedEvaluation(-1, current);
            const double transitionDistanceLowerBound = fullEvaluation
                ? fullEvaluation->summary
                      .mandatoryCuttingDistanceLowerBound
                : 0.0;
            const int transitionStageLowerBound =
                transitionDistanceLowerBound > kDistanceTolerance
                ? 1 : 0;
            State provisional;
            if (!edgeRejected(-1, current))
                applyEvaluation(provisional, -1, current);
            std::vector<EvaluationEdge> missingRestEdges;
            missingRestEdges.reserve(current);
            for (std::size_t previous = 0u;
                 previous < current; ++previous) {
                if (states[previous].reachable &&
                    !edgeRejected(
                        static_cast<int>(previous), current)) {
                    const int previousIndex =
                        static_cast<int>(previous);
                    if (cachedEvaluation(previousIndex, current)) {
                        applyEvaluation(
                            provisional, previousIndex, current);
                    } else {
                        missingRestEdges.push_back(
                            {previousIndex, current});
                    }
                }
            }

            std::stable_sort(
                missingRestEdges.begin(), missingRestEdges.end(),
                [&](const EvaluationEdge& left,
                    const EvaluationEdge& right) {
                    const State& a = states[
                        static_cast<std::size_t>(left.previous)];
                    const State& b = states[
                        static_cast<std::size_t>(right.previous)];
                    if (std::abs(
                            a.cuttingDistance - b.cuttingDistance) >
                        kDistanceTolerance) {
                        return a.cuttingDistance < b.cuttingDistance;
                    }
                    if (a.stageCount != b.stageCount)
                        return a.stageCount < b.stageCount;
                    return left.previous < right.previous;
                });

            std::size_t nextMissing = 0u;
            for (; nextMissing < missingRestEdges.size();
                 ++nextMissing) {
                const EvaluationEdge& edge =
                    missingRestEdges[nextMissing];
                const State& parent = states[
                    static_cast<std::size_t>(edge.previous)];
                if (cannotImprove(
                        parent, provisional,
                        transitionDistanceLowerBound,
                        transitionStageLowerBound)) {
                    markCostBoundPruned(edge.previous, current);
                    continue;
                }
                if (!ensureEvaluated({edge})) {
                    result.cancelled = true;
                    return finish();
                }
                applyEvaluation(
                    provisional, edge.previous, current);
                ++nextMissing;
                break;
            }

            std::vector<EvaluationEdge> boundedRestEdges;
            boundedRestEdges.reserve(
                missingRestEdges.size() - nextMissing);
            for (; nextMissing < missingRestEdges.size();
                 ++nextMissing) {
                const EvaluationEdge& edge =
                    missingRestEdges[nextMissing];
                const State& parent = states[
                    static_cast<std::size_t>(edge.previous)];
                if (cannotImprove(
                        parent, provisional,
                        transitionDistanceLowerBound,
                        transitionStageLowerBound)) {
                    markCostBoundPruned(edge.previous, current);
                } else {
                    boundedRestEdges.push_back(edge);
                }
            }
            if (!ensureEvaluated(boundedRestEdges)) {
                result.cancelled = true;
                return finish();
            }

            // Evaluation order is cost-directed, but final DP tie-breaking
            // remains the original stable diameter order.
            State deterministic;
            if (!edgeRejected(-1, current))
                applyEvaluation(deterministic, -1, current);
            for (std::size_t previous = 0u;
                 previous < current; ++previous) {
                if (states[previous].reachable &&
                    !edgeRejected(
                        static_cast<int>(previous), current) &&
                    cachedEvaluation(
                        static_cast<int>(previous), current)) {
                    applyEvaluation(
                        deterministic,
                        static_cast<int>(previous), current);
                }
            }
            states[current] = deterministic;
        }
        if (!states[finalIndex].reachable) {
            if (finalIndex == 0u) break;
            --finalIndex;
            // A larger terminal tool must be evaluated again WITH its finish
            // pass. Reusing its former rough-only batch would omit finishing.
            // Preserve the failed audits, but discard cache/edge decisions for
            // the changed terminal and all now-excluded smaller cutters.
            for (std::size_t current = finalIndex; current < candidates.size(); ++current) {
                for (int previous = -1; previous < static_cast<int>(current); ++previous) {
                    auto& cached = evaluationCache[evaluationIndex(previous, current)];
                    if (cached) releaseCachedBatch(*cached, false);
                    cached.reset();
                }
            }
            std::erase_if(rejectedEdges, [&](const auto& edge) { return edge.second >= finalIndex; });
            selectedStages.clear();
            continue;
        }

        std::vector<std::size_t> selected;
        for (int index = static_cast<int>(finalIndex);
             index >= 0;) {
            selected.push_back(static_cast<std::size_t>(index));
            index = states[static_cast<std::size_t>(index)].parent;
        }
        std::reverse(selected.begin(), selected.end());

        selectedStages.clear();
        selectedStages.reserve(selected.size());
        bool retryPlanning = false;
        for (const std::size_t current : selected) {
            const int previous = states[current].parent;
            const double previousDiameter = (previous < 0 ||
                candidates[static_cast<std::size_t>(previous)].cornerRadius > 0.0)
                ? 0.0
                : candidates[static_cast<std::size_t>(previous)].diameter;
            const bool finishPass =
                current == finalIndex && params.profile.finishPass;
            CachedEvaluation* evaluated =
                cachedEvaluation(previous, current);
            if (!evaluated) {
                rejectedEdges.emplace_back(previous, current);
                ++result.rejectedTransitionCount;
                retryPlanning = true;
                break;
            }
            oap::toolpath::ProfileToolpathBatch lightweightBatch;
            if (evaluated->batch) {
                lightweightBatch = std::move(*evaluated->batch);
                evaluated->batch.reset();
                retainedEvaluationLoopCount -=
                    evaluated->retainedLoopCount;
                retainedEvaluationByteCount -=
                    evaluated->retainedBytes;
                evaluated->retainedLoopCount = 0u;
                evaluated->retainedBytes = 0u;
                ++result.releasedEvaluationStageCount;
                ++result.transferredEvaluationStageCount;
            } else {
                const Clock::time_point reevaluationStart = Clock::now();
                GeneratedStage regenerated = generateStage(
                    candidates[current], previousDiameter, finishPass,
                    regions, params, stagePrefix(previous, current),
                    effectiveControl, true);
                result.evaluationMilliseconds +=
                    elapsedMilliseconds(reevaluationStart);
                ++result.reevaluatedEvictedStageCount;
                if (regenerated.batch.cancelled) {
                    result.cancelled = true;
                    return finish();
                }
                if (!regenerated.usable) {
                    result.stageAudits.push_back(regenerated.audit);
                    rejectedEdges.emplace_back(previous, current);
                    ++result.rejectedTransitionCount;
                    retryPlanning = true;
                    break;
                }
                lightweightBatch = std::move(regenerated.batch);
            }
            const Clock::time_point materializationStart = Clock::now();
            GeneratedStage materialized = materializeStage(
                candidates[current], previousDiameter, finishPass,
                regions, std::move(lightweightBatch), effectiveControl);
            result.materializationMilliseconds +=
                elapsedMilliseconds(materializationStart);
            ++result.materializedStageCount;
            ++result.promotedStageCount;
            result.stageAudits.push_back(materialized.audit);
            if (materialized.batch.cancelled) {
                result.cancelled = true;
                return finish();
            }
            if (!materialized.usable) {
                rejectedEdges.emplace_back(previous, current);
                ++result.rejectedTransitionCount;
                retryPlanning = true;
                break;
            }
            selectedStages.push_back(std::move(materialized));
        }
        if (retryPlanning)
            continue;
        selectedChainMaterialized = true;
        break;
    }

    if (!selectedChainMaterialized) {
        const auto fullAudit = std::find_if(
            result.stageAudits.rbegin(), result.stageAudits.rend(),
            [&](const ProfileAutomaticToolpathStageAudit& audit) {
                return audit.stage.tool.toolNumber ==
                           candidates[finalIndex].toolNumber &&
                       !audit.usable;
            });
        std::ostringstream message;
        message << "no validated full/rest profile tool combination reaches "
                << "the smallest usable cutter T"
                << candidates[finalIndex].toolNumber << " D"
                << candidates[finalIndex].diameter;
        if (fullAudit != result.stageAudits.rend()) {
            message << "; its "
                    << (fullAudit->stage.previousToolDiameter <=
                                kDiameterTolerance
                            ? "full" : "rest")
                    << " stage generated "
                    << fullAudit->stage.operationCount
                    << " operation(s) and failed "
                    << fullAudit->failedRegionCount << " region(s)";
            if (!fullAudit->failureDetails.empty())
                message << ": " << fullAudit->failureDetails.front();
            else if (fullAudit->validationErrorCount > 0)
                message << " with " << fullAudit->validationErrorCount
                        << " validation error(s)";
        }
        result.error = message.str();
        return finish();
    }

    // All replanning decisions are complete. Explicitly release every
    // unselected lightweight contour batch before assembling the final output.
    for (auto& cached : evaluationCache) {
        if (cached && cached->batch)
            releaseCachedBatch(*cached, false);
    }
    retainedEvaluationLoopCount = 0u;
    retainedEvaluationByteCount = 0u;

    result.batch.algorithmId = "profile.auto_tool_library";
    result.batch.effectiveParams =
        oap::toolpath::bindPocketToolParameters(
            params, candidates[finalIndex]);
    result.batch.effectiveParams.clearing
        .restMachiningPreviousToolDiameter = 0.0;
    result.batch.safetyValidationPerformed = true;
    for (GeneratedStage& stage : selectedStages) {
        if (!stage.batch.operations.empty()) {
            result.stages.push_back(stage.summary);
            appendSelectedStage(result.batch, stage);
        }
    }
    if (finalIndex + 1u < candidates.size()) {
        oap::toolpath::addToolpathIssue(result.batch.safetyValidation,
            oap::toolpath::ToolpathIssueSeverity::Warning, "profile.final_tool_fallback",
            "Smaller profile cutters failed planning or safety validation; the smallest validated final cutter is T" +
                std::to_string(candidates[finalIndex].toolNumber) + " D" + std::to_string(candidates[finalIndex].diameter));
    }
    result.validation = result.batch.safetyValidation;
    if (!result.validation.ok())
        result.error = "selected automatic profile program failed validation";
    return finish();
}

} // namespace oap::tooling

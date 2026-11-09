// 活动刀库外轮廓多刀开粗：筛选可达铣刀，通过全量/余料阶段的
// 最短切削距离规划，输出从大刀到小刀的可执行工序与候选审计。
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "tooling/library/ToolLibrary.h"
#include "toolpath/profile/geometry/ProfileRegion.h"
#include "toolpath/workflow/profile/ProfileToolpathService.h"

namespace oap::tooling {

// 外轮廓自动刀路的一个阶段（一把刀）：刀具、余料前刀径与切削距离。
struct ProfileAutomaticToolpathStage {
    oap::toolpath::ToolpathToolContext tool;
    double previousToolDiameter = 0.0;
    double movementDistance = 0.0;
    double cuttingDistance = 0.0;
    double mandatoryCuttingDistanceLowerBound = 0.0;
    int operationCount = 0;
    int noRestMaterialRegionCount = 0;
    bool finishPass = false;
    double generationMilliseconds = 0.0;
};

// 阶段候选审计：某把刀的可用性、失败区域与校验错误明细。
struct ProfileAutomaticToolpathStageAudit {
    ProfileAutomaticToolpathStage stage;
    int failedRegionCount = 0;
    int validationErrorCount = 0;
    bool evaluationOnly = false;
    bool usable = false;
    std::vector<std::string> failureDetails;
};

// 外轮廓自动刀路的完整结果：批次、校验、各阶段与评估/物化统计。
struct ProfileAutomaticToolpathResult {
    oap::toolpath::ProfileToolpathBatch batch;
    oap::toolpath::ToolpathValidationReport validation;
    std::vector<ProfileAutomaticToolpathStage> stages;
    std::vector<ProfileAutomaticToolpathStageAudit> stageAudits;
    std::string libraryId;
    std::string libraryName;
    std::string error;
    int candidateToolCount = 0;
    int evaluatedTransitionCount = 0;
    int costBoundPrunedTransitionCount = 0;
    int evaluatedStageCount = 0;
    int materializedStageCount = 0;
    int promotedStageCount = 0;
    int cachedEvaluationStageCount = 0;
    int releasedEvaluationStageCount = 0;
    int evictedEvaluationStageCount = 0;
    int reevaluatedEvictedStageCount = 0;
    int transferredEvaluationStageCount = 0;
    int rejectedTransitionCount = 0;
    int parallelEvaluationBatchCount = 0;
    int parallelEvaluatedStageCount = 0;
    std::size_t evaluationPeakParallelism = 1u;
    std::size_t peakRetainedEvaluationLoopCount = 0u;
    std::size_t evaluationCacheByteLimit = 0u;
    std::size_t peakRetainedEvaluationBytes = 0u;
    double evaluationMilliseconds = 0.0;
    double materializationMilliseconds = 0.0;
    double totalMilliseconds = 0.0;
    std::size_t materialSectionCacheHitCount = 0u;
    std::size_t materialSectionCacheMissCount = 0u;
    std::size_t candidateGeometryCacheHitCount = 0u;
    std::size_t candidateGeometryCacheMissCount = 0u;
    std::size_t candidateGeometryCacheEntryCount = 0u;
    std::size_t candidateGeometryCacheEstimatedBytes = 0u;
    bool cancelled = false;

    bool ok() const noexcept
    {
        return !cancelled && error.empty() &&
               !batch.operations.empty() && validation.ok();
    }
};

// 外轮廓自动刀路入口：从刀库筛选可达铣刀并按多刀阶段生成候选工序。
class ProfileAutomaticToolpath {
public:
    // Preliminary length/type screening only; generation still validates the path.
    static bool hasCandidateTool(
        const ToolLibrary& library,
        const oap::toolpath::ProfileRegion& region,
        const oap::toolpath::ToolpathParams& params);

    ProfileAutomaticToolpathResult generate(
        const ToolLibrary& library,
        const std::vector<oap::toolpath::ProfileRegion>& regions,
        const oap::toolpath::ToolpathParams& params,
        const std::string& operationPrefix = "PROFILE_AUTO_",
        const oap::toolpath::ToolpathGenerationControl* control = nullptr) const;
};

} // namespace oap::tooling

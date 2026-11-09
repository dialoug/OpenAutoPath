// 外轮廓刀路服务（oap_toolpath_workflow）。
// 对一批 ProfileRegion 并行生成外轮廓工序、执行安全校验并汇总批次
// （ProfileToolpathBatch / 指标 / 失败项），或把评估批次物化为可执行
// moves；UI 无关，由 app 层工作流调用。
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "toolpath/algorithms/ToolpathAlgorithm.h"
#include "toolpath/core/ToolpathDiagnostics.h"
#include "toolpath/core/ToolpathProgram.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"
#include "toolpath/profile/program/ProfileToolpathOperation.h"

namespace oap::toolpath {

// 轮廓生成失败项：区域索引与失败轮廓结果。
struct ProfileGenerationFailure {
    int regionIndex = -1;
    ProfileContourResult contour;
};

// 轮廓批次指标：算法/物化/校验耗时与缓存统计。
struct ProfileToolpathBatchMetrics {
    double algorithmMilliseconds = 0.0;
    double materializationMilliseconds = 0.0;
    double safetyValidationMilliseconds = 0.0;
    double totalMilliseconds = 0.0;
    std::size_t materialSectionCacheHitCount = 0u;
    std::size_t materialSectionCacheMissCount = 0u;
    std::size_t candidateGeometryCacheHitCount = 0u;
    std::size_t candidateGeometryCacheMissCount = 0u;
    int parallelRegionBatchCount = 0;
    int parallelRegionOperationCount = 0;
    std::size_t regionPeakParallelism = 1u;
    int parallelLayerBatchCount = 0;
    int parallelLayerCount = 0;
    std::size_t layerPeakParallelism = 1u;

    void append(const ProfileToolpathBatchMetrics& other) noexcept
    {
        algorithmMilliseconds += other.algorithmMilliseconds;
        materializationMilliseconds +=
            other.materializationMilliseconds;
        safetyValidationMilliseconds +=
            other.safetyValidationMilliseconds;
        totalMilliseconds += other.totalMilliseconds;
        materialSectionCacheHitCount +=
            other.materialSectionCacheHitCount;
        materialSectionCacheMissCount +=
            other.materialSectionCacheMissCount;
        candidateGeometryCacheHitCount +=
            other.candidateGeometryCacheHitCount;
        candidateGeometryCacheMissCount +=
            other.candidateGeometryCacheMissCount;
        parallelRegionBatchCount += other.parallelRegionBatchCount;
        parallelRegionOperationCount +=
            other.parallelRegionOperationCount;
        regionPeakParallelism = std::max(
            regionPeakParallelism, other.regionPeakParallelism);
        parallelLayerBatchCount += other.parallelLayerBatchCount;
        parallelLayerCount += other.parallelLayerCount;
        layerPeakParallelism = std::max(
            layerPeakParallelism, other.layerPeakParallelism);
    }
};

// 轮廓批次：算法标识、工序、失败项、诊断与指标。
struct ProfileToolpathBatch {
    std::string algorithmId;
    std::vector<ProfileToolpathOperation> operations;
    std::vector<ProfileGenerationFailure> failures;
    ToolpathParams effectiveParams;
    std::vector<ToolpathDiagnostic> diagnostics;
    ToolpathValidationReport safetyValidation;
    ProfileToolpathBatchMetrics metrics;
    bool safetyValidationPerformed = false;
    bool cancelled = false;

    ToolpathProgram toProgram() const;
};

// 外轮廓刀路服务：并行生成、安全校验与批次物化。
class ProfileToolpathService {
public:
    ProfileToolpathService();
    ProfileToolpathService(
        std::shared_ptr<const ToolpathAlgorithmRegistry> algorithms,
        std::string algorithmId = kBuiltinProfileAlgorithmId);

    ProfileToolpathBatch generate(
        const std::vector<ProfileRegion>& regions,
        const ToolpathParams& params,
        const std::string& operationPrefix = "PROFILE",
        const ToolpathGenerationControl* control = nullptr) const;

    // Promote the immutable loops retained by an evaluation-only batch into
    // executable moves, then perform the same safety validation as generate().
    ProfileToolpathBatch materializeEvaluated(
        const std::vector<ProfileRegion>& regions,
        ProfileToolpathBatch evaluation,
        const ToolpathGenerationControl* control = nullptr) const;

private:
    std::shared_ptr<const ToolpathAlgorithmRegistry> m_algorithms;
    std::string m_algorithmId;
};

} // namespace oap::toolpath

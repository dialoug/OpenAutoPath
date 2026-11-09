// 槽/腔刀路应用服务：独立负责参数归一化、算法调用、安全验证和来源映射。
//
// 模块归属:oap_toolpath_workflow(无 UI 的刀路用例编排)的槽腔服务子目录。
// 职责:对槽腔生成做端到端编排——归一化参数、按区域逐一调用算法、聚合
// 结构验证与安全验证(可并行)、并维护「区域 → 操作/失败」的稳定映射。
// 服务按值持有注册表与算法 id,无共享状态,线程安全。
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "toolpath/core/ToolpathDiagnostics.h"
#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/ToolpathProgram.h"
#include "toolpath/algorithms/ToolpathAlgorithm.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/core/validation/ToolpathValidationTypes.h"

namespace oap::toolpath {

// 单个区域的生成失败记录:失败来源区域索引与携带失败码/错误信息的清料结果。
struct PocketGenerationFailure {
    int regionIndex = -1;
    PocketClearingResult clearing;
};

// 输入区域到批处理存储位置的稳定映射。成功和失败结果分别保存在对应向量中。
struct PocketRegionOutcome {
    int operationIndex = -1;
    int failureIndex = -1;
};

// 槽腔刀路服务的一次完整输出:生成的操作、失败、区域映射、验证报告、
// 生效参数、诊断与若干缓存/耗时统计。cancelled 为真时操作已清空。
struct PocketToolpathBatch {
    std::string algorithmId;
    std::vector<PocketToolpathOperation> operations;
    std::vector<PocketGenerationFailure> failures;
    std::vector<PocketRegionOutcome> regionOutcomes;
    ToolpathValidationReport validation;
    ToolpathValidationReport safetyValidation;
    ToolpathParams effectiveParams;
    std::vector<ToolpathDiagnostic> diagnostics;
    bool cancelled = false;
    std::size_t materialSectionCacheHits = 0;
    std::size_t materialSectionCacheMisses = 0;
    std::size_t materialSectionCacheEntries = 0;
    double structuralValidationMilliseconds = 0.0;
    double safetyValidationMilliseconds = 0.0;
    bool safetyValidationPerformed = false;

    // 明确发布条件，不能仅凭结构报告没有 Error 就发布候选。
    bool readyForPublication() const noexcept
    {
        return !cancelled && !operations.empty() &&
               safetyValidationPerformed && validation.ok() &&
               safetyValidation.ok();
    }

    // 按区域索引查已生成操作;失败或无结果返回 nullptr。
    const PocketToolpathOperation* operationForRegion(
        std::size_t regionIndex) const noexcept;
    // 按区域索引查清料结果:优先取成功操作,否则回退到失败记录。
    const PocketClearingResult* clearingForRegion(
        std::size_t regionIndex) const noexcept;
    // 把批内操作转换为中立的 ToolpathProgram(供后处理/导出)。
    ToolpathProgram toProgram() const;
};

// 槽腔刀路应用服务:对调用方隐藏算法解析、异常处理与验证细节。
class PocketToolpathService {
public:
    // 默认构造:使用默认注册表与内置槽腔算法。
    PocketToolpathService();
    // 显式构造:注入注册表与算法 id(支持可替换实现)。
    PocketToolpathService(
        std::shared_ptr<const ToolpathAlgorithmRegistry> algorithms,
        std::string algorithmId = kBuiltinPocketAlgorithmId);

    // 生成一批槽腔刀路。
    // 参数:regions 已构建的槽腔区域;params 工艺参数;operationPrefix
    //       生成工序 ID 的前缀;control 可选取消/进度控制。
    // 返回:完整批结果;失败区域进入 failures 而非中断,取消时清空结果。
    PocketToolpathBatch generate(
        const std::vector<PocketRegion>& regions,
        const ToolpathParams& params,
        const std::string& operationPrefix = "POCKET",
        const ToolpathGenerationControl* control = nullptr) const;

    // Generate and structurally validate a planning candidate while deferring
    // the more expensive geometric safety validation until it is selected.
    PocketToolpathBatch generateCandidate(
        const std::vector<PocketRegion>& regions,
        const ToolpathParams& params,
        const std::string& operationPrefix = "POCKET_CANDIDATE",
        const ToolpathGenerationControl* control = nullptr) const;

    // Complete safety validation for a batch returned by generateCandidate().
    // Returns false when validation fails or is cancelled.
    bool validateCandidateSafety(
        const std::vector<PocketRegion>& regions,
        PocketToolpathBatch& batch,
        const ToolpathGenerationControl* control = nullptr) const;

private:
    PocketToolpathBatch generateImpl(
        const std::vector<PocketRegion>& regions,
        const ToolpathParams& params,
        const std::string& operationPrefix,
        const ToolpathGenerationControl* control,
        bool validateSafety) const;

    std::shared_ptr<const ToolpathAlgorithmRegistry> m_algorithms;
    std::string m_algorithmId;
};

} // namespace oap::toolpath

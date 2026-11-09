// 长耗时刀路任务的协作控制。核心层不依赖 Qt；桌面端或 CLI 均可通过
// 回调接入取消标志和进度显示。
#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

namespace oap::toolpath {

class PocketMaterialSectionCache;
class ProfileCandidateGeometryCache;

// 控制同一孔批次的访问次序；局部重生成保留旧自动批次时使用
// PreserveInput，新人工批次仍可在自身内部优化路线。
enum class HoleVisitOrderMode {
    Optimize,
    PreserveInput,
};

// 控制孔间 Rapid 的安全连接形态；局部人工批次使用共同安全平面，
// 避免不同局部高度之间出现斜向三轴连线。
enum class HoleRapidLinkMode {
    DirectWhenSafe,
    CommonClearancePlane,
};

// 长耗时几何任务的当前子阶段。百分比只能表示已完成的离散工作单元；
// 单次 OCCT 布尔内部没有稳定的连续进度，因此桌面端会结合该状态显示
// 忙碌动画和阶段耗时，避免把正在计算误显示成“卡在 0%”。
enum class ToolpathExecutionPhase {
    None,
    PreparingStock,
    BuildingRemovalVolume,
    ValidatingRemovalVolume,
    ApplyingBooleanRemoval,
    ValidatingRemovalResult,
    ValidatingComparisonInputs,
    ComputingInitialMissing,
    ComputingRemainingAllowance,
    IntersectingInitialTarget,
    ComputingOvercut,
    AttributingOvercut,
};

// 长任务当前子阶段与进度上下文，随 reportStatus 回调逐次派发。
// phase 用于 UI 显示忙碌动画；operationIndex/Count 定位当前工序。
struct ToolpathExecutionStatus {
    ToolpathExecutionPhase phase = ToolpathExecutionPhase::None;
    std::string operationId;
    std::size_t operationIndex = 0;
    std::size_t operationCount = 0;
    std::size_t workItemCount = 0;
};

// 生成控制句柄：取消查询、进度/状态回调和孔批次访问策略。
// 回调均为可选；缺省时退化为无取消、无上报。
struct ToolpathGenerationControl {
    std::function<bool()> isCancellationRequested;
    std::function<void(std::size_t completed, std::size_t total)> reportProgress;
    std::function<void(const ToolpathExecutionStatus& status)> reportStatus;
    HoleVisitOrderMode holeVisitOrderMode = HoleVisitOrderMode::Optimize;
    HoleRapidLinkMode holeRapidLinkMode =
        HoleRapidLinkMode::DirectWhenSafe;
    // 单次槽腔批处理共享；不会跨模型或跨任务保留几何。
    // Pocket/profile batches share immutable material sections within one
    // planning task; geometry is never retained across models or jobs.
    std::shared_ptr<PocketMaterialSectionCache>
        pocketMaterialSectionCache;
    // One automatic-profile planning batch reuses immutable layer envelopes,
    // cutter-center areas and sweep areas across full/rest candidates.
    std::shared_ptr<ProfileCandidateGeometryCache>
        profileCandidateGeometryCache;
    // Automatic profile planning can evaluate cutter transitions without
    // materializing moves/primitives or running full move safety validation.
    bool profileEvaluationOnly = false;
    // Regression/diagnostic mode: evaluate every reachable automatic-profile
    // transition instead of applying the admissible accumulated-cost bound.
    bool profileExhaustiveAutomaticEvaluation = false;
    // Soft cap for retained evaluation-only loops. Zero disables eviction;
    // one oversize batch may temporarily exceed the cap until consumed.
    std::size_t profileEvaluationCacheByteLimit =
        64u * 1024u * 1024u;
    // 可选的共享并行预算。上层调度器通过这两个回调把空闲工作槽借给
    // 当前长任务；子阶段拿不到槽时必须退化为串行，避免区域、候选、
    // 深度层和图案内部的多层并行造成过度订阅。
    std::function<std::size_t(std::size_t requested)>
        tryAcquireParallelWorkers;
    std::function<void(std::size_t count)>
        releaseParallelWorkers;

    bool cancelled() const
    {
        return isCancellationRequested && isCancellationRequested();
    }

    void progress(std::size_t completed, std::size_t total) const
    {
        if (reportProgress)
            reportProgress(completed, total);
    }

    void status(ToolpathExecutionStatus value) const
    {
        if (reportStatus)
            reportStatus(value);
    }
};

} // namespace oap::toolpath

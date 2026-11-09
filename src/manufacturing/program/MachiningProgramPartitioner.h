// 已组装正式项目程序的连续输出分包与最终 IPW 新鲜度门禁。
//
// Setup/WCS 是不可跨越的硬边界；当前参考输出策略还按不可变刀具控制器和
// Hole 阶段保守拆分。分包只合并全局 sequence 中连续且分组键相同的工序，
// 不会把后续同键工序提前，从而保持组间/组内执行顺序守恒。
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "manufacturing/program/MachiningProgramAssembler.h"

namespace oap::manufacturing {

enum class MachiningProgramPartitionIssueCode {
    AssemblyRejected,
    MissingIpwPlan,
    IpwPlanMismatch,
    IpwNodeNotReady,
    MissingIpwArtifact,
    StaleIpwArtifact,
    MissingIpwCheckpoint,
    StaleIpwCheckpoint,
    PartitionInvariantViolation,
};

const char* machiningProgramPartitionIssueCodeId(
    MachiningProgramPartitionIssueCode code) noexcept;

struct MachiningProgramPartitionIssue {
    MachiningProgramPartitionIssueCode code =
        MachiningProgramPartitionIssueCode::AssemblyRejected;
    std::string operationId;
    std::string message;
};

// 一个可独立交给当前保守后处理边界的连续程序分包。
struct MachiningProgramPartition {
    std::size_t index = 0u;
    std::size_t firstAssemblyOperation = 0u;
    std::size_t assemblyOperationCount = 0u;
    std::string setupId;
    int setupIndex = 0;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    toolpath::WorkCoordinateSystem workOffset =
        toolpath::WorkCoordinateSystem::None;
    std::string setupInputFingerprint;
    std::string toolControllerId;
    std::string toolContextFingerprint;
    int toolNumber = 0;
    std::string toolId;
    // false 表示本包不含 Hole 工序；此时 holePhase 固定为 Standard。
    bool holePhaseApplies = false;
    toolpath::HoleMachiningPhase holePhase =
        toolpath::HoleMachiningPhase::Standard;
    std::vector<std::string> formalOperationIds;
    std::vector<std::string> runtimeOperationIds;
    toolpath::ToolpathProgram program;
};

struct MachiningProgramPartitionResult {
    MachiningProgramAssemblyResult assembly;
    std::vector<MachiningProgramPartition> partitions;
    std::vector<MachiningProgramPartitionIssue> issues;
    std::string initialIpwStateKey;
    std::string finalIpwStateKey;
    std::string finalIpwContentFingerprint;
    std::string ipwEngineFingerprint;

    bool ok() const noexcept
    {
        return assembly.ok() && issues.empty() && !partitions.empty() &&
            !initialIpwStateKey.empty() && !finalIpwStateKey.empty() &&
            !finalIpwContentFingerprint.empty() &&
            !ipwEngineFingerprint.empty();
    }
};

class MachiningProgramPartitioner {
public:
    // 全有或全无：先执行正式项目组装，再证明 IPW 会话与当前全部启用工序
    // 一一对应且完整 Ready。任一门禁失败时不返回任何可输出分包。
    MachiningProgramPartitionResult partition(
        const MachiningJob& job) const;
};

} // namespace oap::manufacturing

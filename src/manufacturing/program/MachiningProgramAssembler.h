// 正式 MachiningJob 到项目级中立程序的纯值组装器。
//
// 该边界只消费已生成的 Job 定义/运行期快照，不读取桌面活动预览，也不做
// 文件输出。它按全局 sequence 把 Hole/Pocket/Profile 的运行期子工序
// 组装为一个 ToolpathProgram，并保留正式工序到运行期工序的追溯映射。
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "manufacturing/job/MachiningJob.h"
#include "manufacturing/job/MachiningOperationService.h"
#include "toolpath/core/ToolpathProgram.h"

namespace oap::manufacturing {

enum class MachiningProgramAssemblyIssueCode {
    InvalidJobDefinition,
    EmptyOperationSet,
    InvalidOperationState,
    MissingSetup,
    UnconfirmedSetup,
    MissingToolController,
    MissingRuntimeResult,
    StaleRuntimeResult,
    RuntimeProgramRejected,
    UnmappedRuntimeOperation,
    DisabledRuntimeOperation,
    DuplicateRuntimeOperationId,
    OperationKindMismatch,
    HolePhaseMismatch,
    SetupMismatch,
    ToolMismatch,
    EmptyRuntimeOperation,
    MissingRuntimeOperation,
};

const char* machiningProgramAssemblyIssueCodeId(
    MachiningProgramAssemblyIssueCode code) noexcept;

struct MachiningProgramAssemblyIssue {
    MachiningProgramAssemblyIssueCode code =
        MachiningProgramAssemblyIssueCode::InvalidJobDefinition;
    std::string operationId;
    std::string message;
};

// 一道正式 Job 工序在项目程序中的连续范围。一个正式工序可展开为多个
// 运行期子工序（例如孔的 -N 或区域展开的 -rN）。
struct MachiningProgramOperationMapping {
    std::string formalOperationId;
    JobOperationKind kind = JobOperationKind::Hole;
    int sequence = -1;
    std::string setupId;
    std::string toolControllerId;
    std::string inputFingerprint;
    int setupIndex = 0;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    toolpath::WorkCoordinateSystem workOffset =
        toolpath::WorkCoordinateSystem::None;
    toolpath::HoleMachiningPhase holePhase =
        toolpath::HoleMachiningPhase::Standard;
    std::size_t firstProgramOperation = 0u;
    std::size_t programOperationCount = 0u;
    std::vector<std::string> runtimeOperationIds;
};

struct MachiningProgramAssemblyResult {
    toolpath::ToolpathProgram program;
    std::vector<MachiningProgramOperationMapping> mappings;
    std::vector<MachiningProgramAssemblyIssue> issues;
    // 组装时的完整 Job 定义身份，供后续分包/导出新鲜度门禁消费。
    std::string jobDefinitionFingerprint;

    bool ok() const noexcept
    {
        return issues.empty() && !program.empty() && !mappings.empty();
    }
};

class MachiningProgramAssembler {
public:
    // 全有或全无：任何启用工序缺少正式运行期结果、执行指纹过期、资源
    // 快照不一致或运行期子工序无法映射时，返回空 program/mappings。
    // 非空 includedOperationIds 只组装指定正式工序，供 IPW 范围播放使用；
    // 不修改 Job，仍校验所选工序所属批次的执行指纹及完整资源快照。
    MachiningProgramAssemblyResult assemble(
        const MachiningJob& job,
        const std::vector<std::string>& includedOperationIds = {}) const;
};

} // namespace oap::manufacturing

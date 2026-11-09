// 加工 Job 工序查询：提供孔、槽腔和外轮廓工序的稳定顺序投影与运行期 ID 映射。
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "manufacturing/job/MachiningJob.h"

namespace oap::manufacturing {

enum class JobOperationKind {
    Hole,
    Pocket,
    Profile,
    SurfaceFinishing,
};

const char* jobOperationKindId(JobOperationKind kind) noexcept;

struct JobOperationDescriptor {
    std::string id;
    JobOperationKind kind = JobOperationKind::Hole;
    std::string setupId;
    std::string toolControllerId;
    std::string algorithmId;
    std::string inputFingerprint;
    // 仅 Hole 使用；其他领域保持 Standard，供项目级程序分包建立稳定阶段边界。
    toolpath::HoleMachiningPhase holePhase =
        toolpath::HoleMachiningPhase::Standard;
    int sequence = -1;
    bool enabled = false;
    JobOperationState state = JobOperationState::Planned;
};

class MachiningOperationService {
public:
    // 返回三类正式工序的稳定全局顺序投影。
    static std::vector<JobOperationDescriptor> operations(
        const MachiningJobDefinition& definition);

    // 把运行期展开 ID（例如 formal-2 / formal-r3）解析回正式工序。
    // 若多个正式 ID 是前缀，选择最长的合法匹配，避免短 ID 抢占。
    static std::optional<JobOperationDescriptor> operationForRuntimeId(
        const MachiningJobDefinition& definition,
        const std::string& runtimeOperationId);
    static std::string formalOperationIdForRuntime(
        const MachiningJobDefinition& definition,
        const std::string& runtimeOperationId);
    static bool runtimeOperationIdMatches(
        const std::string& formalOperationId,
        const std::string& runtimeOperationId);

};

} // namespace oap::manufacturing

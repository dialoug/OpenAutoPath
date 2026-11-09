// Private stage data shared only within this target.
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>
#include "manufacturing/hole/HoleMachiningJobService.h"

namespace oap::manufacturing::hole_job_detail {

//
// 职责:孔的"稳定操作身份"——在 Job 定义变更前后依据装夹面/相位/阶段键
// 而非临时 ID 匹配旧工序,用于恢复 ID、全局顺序与可复用 IPW 前缀;同时把
// 生成的运行期刀路工序 ID 重绑为可回溯到定义的稳定 ID。
struct StableHoleOperationIdentity {
    // 定义内索引(用于排序诊断)与稳定 ID;sequence 为全局加工序列。
    std::size_t operationIndex = 0;
    std::string id;
    int sequence = -1;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    toolpath::HoleMachiningPhase phase =
        toolpath::HoleMachiningPhase::Standard;
    std::vector<std::string> stageKeys;
};

//
// 职责:捕获"活动 IPW 计划"(当前已配置的初始毛坯指纹、引擎指纹与
// 纳入 IPW 的工序集合),并在 Job 重建后据此恢复可复用的检查点链。
// 协作关系:通过 MachiningJob::ipwSession()/configureIpwPlan() 与
// IpwSession、IpwCheckpointStore 交互;UI 无关,单线程使用。
// 活动 IPW 计划的稳定快照:记录配置指纹与纳入工序,供 Job 重建后
// 恢复相同前缀的检查点;configured 为 false 表示会话尚未配置 IPW。
struct ActiveIpwPlan {
    // 孔工序在 IPW 计划中的稳定作用域:装夹面 + 相位 + 已识别孔阶段键,
    // 用于重建时按语义而非临时 ID 匹配工序。
    struct SelectedHoleScope {
        MoldSetupFace setupFace = MoldSetupFace::Unresolved;
        toolpath::HoleMachiningPhase phase =
            toolpath::HoleMachiningPhase::Standard;
        std::vector<std::string> stageKeys;
    };

    bool configured = false;
    bool fullPlan = true;
    std::string stockFingerprint;
    std::string engineFingerprint;
    std::vector<std::string> includedOperationIds;
    std::vector<SelectedHoleScope> includedHoleScopes;
};

} // namespace oap::manufacturing::hole_job_detail

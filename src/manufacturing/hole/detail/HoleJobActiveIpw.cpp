// Snapshot and restore the active material-state plan across Job replacement.
#include "manufacturing/hole/HoleMachiningJobService.h"
#include <algorithm>
#include <set>
#include <unordered_set>

#include "manufacturing/hole/detail/HoleJobTypes.h"
#include "manufacturing/hole/detail/HoleJobIdentity.h"
#include "manufacturing/hole/detail/HoleJobActiveIpw.h"

namespace oap::manufacturing::hole_job_detail {

// 捕获当前 Job 的活动 IPW 计划:汇总会话指纹、全部已启用工序与
// 会话节点中的孔作用域,供 Job 持久化后重建使用。
ActiveIpwPlan captureActiveIpwPlan(const MachiningJob& job)
{
    ActiveIpwPlan result;
    const ipw::IpwSession& session = job.ipwSession();
    if (!session.configured())
        return result;
    result.configured = true;
    result.stockFingerprint = session.initialStockFingerprint();
    result.engineFingerprint = session.engineFingerprint();
    std::set<std::string> allEnabled;
    for (const HoleOperationDefinition& operation :
         job.definition().holeOperations) {
        if (operation.enabled)
            allEnabled.insert(operation.id);
    }
    for (const PocketOperationDefinition& operation :
         job.definition().pocketOperations) {
        if (operation.enabled)
            allEnabled.insert(operation.id);
    }
    for (const auto& operation : job.definition().profileOperations)
        if (operation.enabled) allEnabled.insert(operation.id);
    for (const auto& operation : job.definition().finishingOperations)
        if (operation.enabled) allEnabled.insert(operation.id);
    for (const ipw::IpwNode& node : session.nodes()) {
        result.includedOperationIds.push_back(node.operation.id);
        if (node.operation.kind != ipw::IpwOperationKind::Hole)
            continue;
        const auto operation = std::find_if(
            job.definition().holeOperations.begin(),
            job.definition().holeOperations.end(),
            [&](const HoleOperationDefinition& value) {
                return value.id == node.operation.id;
            });
        if (operation == job.definition().holeOperations.end())
            continue;
        result.includedHoleScopes.push_back({
            operationSetupFace(job.definition(), *operation),
            operation->phase,
            operationStageKeys(
                *operation, job.definition().recognizedHoles),
        });
    }
    // 仅当会话节点集合恰为"全部已启用工序"时视为完整计划;此时重建可按
    // 原指纹整体恢复,否则必须按作用域逐工序重新匹配。
    result.fullPlan = result.includedOperationIds.size() == allEnabled.size() &&
        std::all_of(
            result.includedOperationIds.begin(),
            result.includedOperationIds.end(),
            [&](const std::string& id) { return allEnabled.contains(id); });
    return result;
}

// 把保存的活动 IPW 计划恢复到 Job:先按稳定孔身份(装夹面/相位/阶段键
// 重叠)把保存的孔作用域匹配到当前启用的孔工序,再连同保存的指纹配置
// IPW 计划;没有任何可匹配工序时清空会话与检查点。
bool restoreActiveIpwPlan(MachiningJob& job, const ActiveIpwPlan& saved)
{
    if (!saved.configured)
        return true;
    std::vector<std::string> included;
    if (!saved.fullPlan) {
        std::set<std::string> enabled;
        for (const HoleOperationDefinition& operation :
             job.definition().holeOperations) {
            if (operation.enabled)
                enabled.insert(operation.id);
        }
        for (const PocketOperationDefinition& operation :
             job.definition().pocketOperations) {
            if (operation.enabled)
                enabled.insert(operation.id);
        }
        for (const auto& operation : job.definition().profileOperations)
            if (operation.enabled) enabled.insert(operation.id);
        for (const auto& operation : job.definition().finishingOperations)
            if (operation.enabled) enabled.insert(operation.id);
        for (const std::string& id : saved.includedOperationIds) {
            if (enabled.contains(id))
                included.push_back(id);
        }
        std::unordered_set<std::string> includedSet(
            included.begin(), included.end());
        // 部分计划:对未列入 included 的已启用孔工序,用保存的孔作用域做
        // 稳定身份匹配(装夹面、相位一致且阶段键有重叠)以补齐工序 ID。
        for (const HoleOperationDefinition& operation :
             job.definition().holeOperations) {
            if (!operation.enabled || includedSet.contains(operation.id))
                continue;
            StableHoleOperationIdentity current;
            current.setupFace = operationSetupFace(
                job.definition(), operation);
            current.phase = operation.phase;
            current.stageKeys = operationStageKeys(
                operation, job.definition().recognizedHoles);
            const bool selected = std::any_of(
                saved.includedHoleScopes.begin(),
                saved.includedHoleScopes.end(),
                [&](const ActiveIpwPlan::SelectedHoleScope& scope) {
                    if (scope.setupFace != current.setupFace ||
                        scope.phase != current.phase) {
                        return false;
                    }
                    StableHoleOperationIdentity previous;
                    previous.stageKeys = scope.stageKeys;
                    return identityOverlap(current, previous) > 0;
                });
            if (!selected)
                continue;
            included.push_back(operation.id);
            includedSet.insert(operation.id);
        }
        // 没有任何可恢复的工序时,清空 IPW 会话与检查点,使重建表现为空计划。
        if (included.empty()) {
            job.ipwSession().clear();
            job.runtime().ipwCheckpoints.clear();
            return true;
        }
    }
    return job.configureIpwPlan(
        saved.stockFingerprint, saved.engineFingerprint, included).ok();
}

} // namespace oap::manufacturing::hole_job_detail

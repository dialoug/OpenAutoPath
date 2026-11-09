// Stable operation matching and runtime identity binding.
#include "manufacturing/hole/HoleMachiningJobService.h"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <tuple>
#include <unordered_set>
#include "tooling/adapter/hole/HoleStageIdentity.h"

#include "manufacturing/hole/detail/HoleJobTypes.h"
#include "manufacturing/hole/detail/HoleJobFoundation.h"
#include "manufacturing/hole/detail/HoleJobIdentity.h"

namespace oap::manufacturing::hole_job_detail {

// 返回工序所属 Setup 的装夹面;Setup 缺失时返回 Unresolved。
MoldSetupFace operationSetupFace(
    const MachiningJobDefinition& definition,
    const HoleOperationDefinition& operation)
{
    const auto setup = std::find_if(
        definition.setups.begin(), definition.setups.end(),
        [&](const SetupDefinition& value) {
            return value.id == operation.setupId;
        });
    return setup == definition.setups.end()
        ? MoldSetupFace::Unresolved : setup->face;
}

// 按 ID 查找工序的 Setup 定义;缺失时返回 nullptr。
const SetupDefinition* operationSetup(
    const MachiningJobDefinition& definition,
    const HoleOperationDefinition& operation)
{
    const auto setup = std::find_if(
        definition.setups.begin(), definition.setups.end(),
        [&](const SetupDefinition& value) {
            return value.id == operation.setupId;
        });
    return setup == definition.setups.end() ? nullptr : &*setup;
}

// 按 ID 查找工序的刀具控制器定义;缺失时返回 nullptr。
const JobToolController* operationController(
    const MachiningJobDefinition& definition,
    const HoleOperationDefinition& operation)
{
    const auto controller = std::find_if(
        definition.toolControllers.begin(),
        definition.toolControllers.end(),
        [&](const JobToolController& value) {
            return value.id == operation.toolControllerId;
        });
    return controller == definition.toolControllers.end()
        ? nullptr : &*controller;
}

// 判断目标是否引用给定源孔(主引用或覆盖引用)。
bool targetReferencesSource(
    const HoleTargetReference& target,
    int sourceIndex)
{
    return sourceIndex >= 0 &&
           (target.sourceHoleIndex == sourceIndex ||
            std::find(
                target.coveredSourceHoleIndices.begin(),
                target.coveredSourceHoleIndices.end(),
                sourceIndex) != target.coveredSourceHoleIndices.end());
}

// 判断定义目标集合是否引用运行期工序的源孔(主或覆盖)。
bool operationReferencesRuntimeSource(
    const HoleOperationDefinition& definition,
    const toolpath::HoleMachiningOperation& runtime)
{
    const auto references = [&](int sourceIndex) {
        return std::any_of(
            definition.targets.begin(), definition.targets.end(),
            [sourceIndex](const HoleTargetReference& target) {
                return targetReferencesSource(target, sourceIndex);
            });
    };
    if (references(runtime.sourceHoleIndex))
        return true;
    return std::any_of(
        runtime.coveredSourceHoleIndices.begin(),
        runtime.coveredSourceHoleIndices.end(), references);
}

// 为运行期刀路工序寻找唯一匹配的定义候选:过滤条件为装夹面/索引、相位、
// 本地批标记与源孔引用一致,且刀具键匹配;多候选时优先稳定 ID 前缀匹配。
std::optional<std::size_t> capturedDefinitionForRuntime(
    const MachiningJobDefinition& definition,
    const toolpath::HoleMachiningOperation& runtime)
{
    std::vector<std::size_t> candidates;
    for (std::size_t index = 0;
         index < definition.holeOperations.size(); ++index) {
        const HoleOperationDefinition& operation =
            definition.holeOperations[index];
        const SetupDefinition* setup =
            operationSetup(definition, operation);
        if (setup &&
            runtime.setupFace != MoldSetupFace::Unresolved &&
            setup->face != runtime.setupFace) {
            continue;
        }
        if (setup && runtime.setupIndex >= 0 &&
            setup->setupIndex != runtime.setupIndex) {
            continue;
        }
        if (operation.phase != runtime.phase ||
            !operationReferencesRuntimeSource(operation, runtime)) {
            continue;
        }
        const JobToolController* controller =
            operationController(definition, operation);
        if (controller && controller->tool.valid && runtime.tool.valid &&
            controllerKey(controller->tool) != controllerKey(runtime.tool)) {
            continue;
        }
        candidates.push_back(index);
    }
    if (candidates.size() == 1)
        return candidates.front();

    // 重新捕获已安装的程序会保留其稳定 ID。这也能解决一种少见情况：
    // 两个定义引用了具有相同工艺元数据的同一物理来源。
    const auto stableMatch = std::find_if(
        candidates.begin(), candidates.end(),
        [&](std::size_t index) {
            const std::string& id =
                definition.holeOperations[index].id;
            return runtime.operationId == id ||
                   (runtime.operationId.starts_with(id) &&
                    runtime.operationId.size() > id.size() &&
                    runtime.operationId[id.size()] == '-');
        });
    return stableMatch == candidates.end()
        ? std::nullopt
        : std::optional<std::size_t>(*stableMatch);
}

// 把生成的运行期刀路工序 ID 重绑为"<定义ID>-<序号>"形式,使运行期产物
// 可回溯到稳定定义;同时把诊断/问题条目的 operationId 一并重绑。
toolpath::HoleToolpathBatch bindCapturedRuntimeIdentities(
    const MachiningJobDefinition& definition,
    const toolpath::HoleToolpathBatch& generated)
{
    toolpath::HoleToolpathBatch result = generated;
    std::vector<std::size_t> localIndices(
        definition.holeOperations.size(), 0);
    std::map<std::string, std::string> reboundIds;
    for (toolpath::HoleMachiningOperation& runtime :
         result.machining.operations) {
        const std::optional<std::size_t> definitionIndex =
            capturedDefinitionForRuntime(definition, runtime);
        if (!definitionIndex)
            continue;
        const std::string previousId = runtime.operationId;
        runtime.operationId =
            definition.holeOperations[*definitionIndex].id + "-" +
            std::to_string(++localIndices[*definitionIndex]);
        if (!previousId.empty())
            reboundIds.emplace(previousId, runtime.operationId);
    }

    // 诊断与校验问题也引用运行期操作 ID,需同步重绑到新 ID。
    const auto rebindDiagnostic = [&reboundIds](auto& item) {
        const auto rebound = reboundIds.find(item.operationId);
        if (rebound != reboundIds.end())
            item.operationId = rebound->second;
    };
    for (toolpath::ToolpathDiagnostic& diagnostic : result.diagnostics)
        rebindDiagnostic(diagnostic);
    for (toolpath::ToolpathDiagnostic& diagnostic :
         result.machining.diagnostics) {
        rebindDiagnostic(diagnostic);
    }
    for (toolpath::ToolpathIssue& issue : result.validation.issues)
        rebindDiagnostic(issue);
    return result;
}

// 判断运行期刀路的每个工序 ID 都已绑定为"<定义ID>-<数字后缀>"形式。
bool capturedRuntimeIdentitiesComplete(
    const MachiningJobDefinition& definition,
    const toolpath::HoleToolpathBatch& runtime)
{
    return std::all_of(
        runtime.machining.operations.begin(),
        runtime.machining.operations.end(),
        [&](const toolpath::HoleMachiningOperation& operation) {
            return std::any_of(
                definition.holeOperations.begin(),
                definition.holeOperations.end(),
                [&](const HoleOperationDefinition& candidate) {
                    const std::string prefix = candidate.id + "-";
                    return operation.operationId.starts_with(prefix) &&
                           operation.operationId.size() > prefix.size() &&
                           std::all_of(
                               operation.operationId.begin() +
                                   static_cast<std::ptrdiff_t>(prefix.size()),
                               operation.operationId.end(),
                               [](char value) {
                                   return value >= '0' && value <= '9';
                               });
                });
        });
}

// 计算工序的稳定阶段键集合:对每个目标取主源孔与覆盖源孔的孔阶段键;
// 无有效源孔时退化为 FEATURE|特征ID 或 GEOMETRY|几何指纹,最后去重排序。
std::vector<std::string> operationStageKeys(
    const HoleOperationDefinition& operation,
    const std::vector<toolpath::HoleMachiningInput>& holes)
{
    std::vector<std::string> result;
    const auto appendIndex = [&](int sourceIndex) {
        if (sourceIndex < 0 ||
            sourceIndex >= static_cast<int>(holes.size())) {
            return;
        }
        result.push_back(tooling::holeStageKey(
            holes[static_cast<std::size_t>(sourceIndex)]));
    };
    for (const HoleTargetReference& target : operation.targets) {
        const std::size_t before = result.size();
        appendIndex(target.sourceHoleIndex);
        for (const int sourceIndex : target.coveredSourceHoleIndices)
            appendIndex(sourceIndex);
        if (result.size() != before)
            continue;
        if (!target.featureId.empty())
            result.push_back("FEATURE|" + target.featureId);
        else if (!target.geometryFingerprint.empty())
            result.push_back("GEOMETRY|" + target.geometryFingerprint);
    }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

// 捕获定义当前全部孔工序的稳定身份快照,用于与旧身份做匹配。
std::vector<StableHoleOperationIdentity> captureStableHoleIdentities(
    const MachiningJobDefinition& definition,
    const std::vector<toolpath::HoleMachiningInput>& holes)
{
    std::vector<StableHoleOperationIdentity> result;
    result.reserve(definition.holeOperations.size());
    for (std::size_t index = 0;
         index < definition.holeOperations.size(); ++index) {
        const HoleOperationDefinition& operation =
            definition.holeOperations[index];
        result.push_back({
            index,
            operation.id,
            operation.sequence,
            operationSetupFace(definition, operation),
            operation.phase,
            operationStageKeys(operation, holes),
        });
    }
    return result;
}

// 计算两个稳定身份的公共阶段键数量(双指针扫描有序键列表)。
std::size_t identityOverlap(
    const StableHoleOperationIdentity& first,
    const StableHoleOperationIdentity& second)
{
    std::size_t result = 0;
    std::size_t left = 0;
    std::size_t right = 0;
    while (left < first.stageKeys.size() &&
           right < second.stageKeys.size()) {
        if (first.stageKeys[left] < second.stageKeys[right]) {
            ++left;
        } else if (second.stageKeys[right] < first.stageKeys[left]) {
            ++right;
        } else {
            ++result;
            ++left;
            ++right;
        }
    }
    return result;
}

// 候选匹配:新/旧索引、是否同来源(localBatch 一致)、阶段键完全相等、
// 重叠与差异数量、旧序列,供匹配排序与贪心分配使用。
struct StableIdentityMatchCandidate {
    std::size_t newIndex = 0;
    std::size_t oldIndex = 0;
    bool sameOrigin = false;
    bool exactStages = false;
    std::size_t overlap = 0;
    std::size_t difference = 0;
    int oldSequence = -1;
};

// 为当前孔工序匹配旧身份:仅比较同装夹面/同相位的候选,优先完全相等、
// 同来源与更大重叠,再按差异与旧序列取优;每个旧身份最多被匹配一次。
std::vector<int> matchStableHoleIdentities(
    const std::vector<StableHoleOperationIdentity>& previous,
    const std::vector<StableHoleOperationIdentity>& current)
{
    std::vector<StableIdentityMatchCandidate> candidates;
    for (std::size_t newIndex = 0;
         newIndex < current.size(); ++newIndex) {
        for (std::size_t oldIndex = 0;
             oldIndex < previous.size(); ++oldIndex) {
            const auto& next = current[newIndex];
            const auto& old = previous[oldIndex];
            if (next.setupFace != old.setupFace ||
                next.phase != old.phase) {
                continue;
            }
            const std::size_t overlap = identityOverlap(next, old);
            if (overlap == 0)
                continue;
            candidates.push_back({
                newIndex,
                oldIndex,
                true,
                next.stageKeys == old.stageKeys,
                overlap,
                next.stageKeys.size() + old.stageKeys.size() -
                    2 * overlap,
                old.sequence,
            });
        }
    }
    // 按 完全相等 > 同来源 > 重叠数 降序,再以差异/旧序列/新索引升序
    // 排序,使随后的贪心分配总是选中最可靠的匹配。
    std::stable_sort(
        candidates.begin(), candidates.end(),
        [](const StableIdentityMatchCandidate& left,
           const StableIdentityMatchCandidate& right) {
            return std::tie(
                       left.exactStages, left.sameOrigin, left.overlap) >
                    std::tie(
                       right.exactStages, right.sameOrigin, right.overlap) ||
                (std::tie(
                     left.exactStages, left.sameOrigin, left.overlap) ==
                 std::tie(
                     right.exactStages, right.sameOrigin, right.overlap) &&
                 std::tie(
                     left.difference, left.oldSequence, left.newIndex) <
                 std::tie(
                     right.difference, right.oldSequence, right.newIndex));
        });

    // 贪心分配:按排序后的候选依次占用新/旧索引,保证一一对应。
    std::vector<int> result(current.size(), -1);
    std::vector<bool> oldClaimed(previous.size(), false);
    for (const StableIdentityMatchCandidate& candidate : candidates) {
        if (result[candidate.newIndex] >= 0 ||
            oldClaimed[candidate.oldIndex]) {
            continue;
        }
        result[candidate.newIndex] =
            static_cast<int>(candidate.oldIndex);
        oldClaimed[candidate.oldIndex] = true;
    }
    return result;
}

// 为当前工序寻找放置锚点:在旧身份中挑出最匹配(同面优先、相位/阶段键
// 一致、重叠最大、序列最靠前)的身份,返回其旧序列用于全局顺序恢复。
std::optional<int> placementAnchor(
    const StableHoleOperationIdentity& current,
    const std::vector<StableHoleOperationIdentity>& previous)
{
    const StableHoleOperationIdentity* best = nullptr;
    std::tuple<
        bool, bool, std::size_t, std::size_t, std::int64_t> bestScore{};
    for (const StableHoleOperationIdentity& old : previous) {
        if (current.setupFace != old.setupFace)
            continue;
        const std::size_t overlap = identityOverlap(current, old);
        if (overlap == 0)
            continue;
        const std::size_t difference =
            current.stageKeys.size() + old.stageKeys.size() - 2 * overlap;
        const auto score = std::make_tuple(
            current.phase == old.phase,
            current.stageKeys == old.stageKeys,
            overlap,
            std::numeric_limits<std::size_t>::max() - difference,
            -static_cast<std::int64_t>(old.sequence));
        if (!best || score > bestScore) {
            best = &old;
            bestScore = score;
        }
    }
    return best && best->sequence >= 0
        ? std::optional<int>(best->sequence) : std::nullopt;
}

// 依据稳定身份生成确定性 ID(setupFace/phase/localBatch/stageKeys 的
// SHA-256 摘要),与已用集合冲突时追加数字后缀。
std::string stableHoleOperationId(
    const StableHoleOperationIdentity& identity,
    const std::unordered_set<std::string>& used)
{
    std::ostringstream value;
    value << static_cast<int>(identity.setupFace) << '|'
          << static_cast<int>(identity.phase) << '|'
          << 0;
    for (const std::string& key : identity.stageKeys)
        value << '|' << key;
    const std::string base =
        "hole-operation-" + sha256(value.str()).substr(0, 16);
    if (!used.contains(base))
        return base;
    for (std::size_t suffix = 2;; ++suffix) {
        const std::string candidate =
            base + '-' + std::to_string(suffix);
        if (!used.contains(candidate))
            return candidate;
    }
}

} // namespace oap::manufacturing::hole_job_detail

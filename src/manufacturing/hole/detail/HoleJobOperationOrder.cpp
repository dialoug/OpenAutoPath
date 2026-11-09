// Restore stable hole identities and mixed-feature global operation order.
#include "manufacturing/hole/HoleMachiningJobService.h"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <unordered_set>
#include <utility>

#include "manufacturing/hole/detail/HoleJobTypes.h"
#include "manufacturing/hole/detail/HoleJobIdentity.h"
#include "manufacturing/hole/detail/HoleJobOperationOrder.h"

namespace oap::manufacturing::hole_job_detail {

//
// 职责:在 Job 重建时恢复孔的稳定 ID 与"孔+槽腔"混合全局加工序列,使新定义
// 尽量沿用旧定义的顺序语义,便于保留 IPW 前缀与刀路一致性。
struct MixedOperationOrderEntry {
    // hole=true 表示孔工序索引,否则为槽腔工序索引;preferredSequence 为
    // 希望保留的旧序列(可能缺失)。
    bool hole = false;
    std::size_t index = 0;
    std::optional<int> preferredSequence;
};

// 检查首选序列能否被保留:序列必须非负、单调递增,且相邻锚点间必须有
// 足够空隙容纳中间未锚定工序(否则无法回填连续序列)。
bool canPreservePreferredSequences(
    const std::vector<MixedOperationOrderEntry>& order)
{
    std::optional<std::size_t> previousPosition;
    std::optional<int> previousSequence;
    for (std::size_t position = 0; position < order.size(); ++position) {
        if (!order[position].preferredSequence)
            continue;
        const int sequence = *order[position].preferredSequence;
        if (sequence < 0)
            return false;
        if (!previousPosition) {
            if (static_cast<std::size_t>(sequence) < position)
                return false;
        } else {
            const std::size_t distance = position - *previousPosition;
            if (sequence <= *previousSequence ||
                static_cast<std::uint64_t>(sequence - *previousSequence) <
                    distance) {
                return false;
            }
        }
        previousPosition = position;
        previousSequence = sequence;
    }
    return true;
}

// 为混合顺序分配最终全局序列。可行时以锚点序列为界回填连续序列;不可行时
// 退化为按索引步进,为后续手工插入预留空隙。
std::vector<int> assignMixedSequences(
    const std::vector<MixedOperationOrderEntry>& order)
{
    std::vector<int> result(order.size(), -1);
    if (canPreservePreferredSequences(order)) {
        std::optional<std::size_t> previousPosition;
        std::optional<int> previousSequence;
        for (std::size_t position = 0; position < order.size(); ++position) {
            if (!order[position].preferredSequence)
                continue;
            const int sequence = *order[position].preferredSequence;
            if (!previousPosition) {
                int current = sequence - static_cast<int>(position);
                for (std::size_t index = 0; index < position; ++index)
                    result[index] = current++;
            } else {
                int current = *previousSequence + 1;
                for (std::size_t index = *previousPosition + 1;
                     index < position; ++index) {
                    result[index] = current++;
                }
            }
            result[position] = sequence;
            previousPosition = position;
            previousSequence = sequence;
        }
        int current = previousSequence ? *previousSequence + 1 : 0;
        const std::size_t first = previousPosition
            ? *previousPosition + 1 : 0;
        for (std::size_t index = first; index < order.size(); ++index)
            result[index] = current++;
        return result;
    }

    // 退化路径:无法保留首选序列时,用大步进(1024)或逐 1 步进生成序列,
    // 保证唯一、有序且后续可在任意位置插入新工序。
    constexpr int kSequenceStride = 1024;
    const bool canUseStride = order.size() <=
        static_cast<std::size_t>(
            std::numeric_limits<int>::max() / kSequenceStride);
    for (std::size_t index = 0; index < order.size(); ++index) {
        result[index] = static_cast<int>(index) *
            (canUseStride ? kSequenceStride : 1);
    }
    return result;
}

// 恢复孔工序的稳定 ID,并按旧全局顺序重建孔+槽腔混合序列。流程:
// 1) 用稳定身份匹配旧工序并恢复/生成确定性 ID;
// 2) 为每个孔工序求放置锚点(旧序列),未锚定的用最近锚点插值;
// 3) 把旧全局槽(孔锚 + 槽腔)按序列排布,孔工序按锚点组填入;
// 4) assignMixedSequences 统一赋新序列。
void restoreStableHoleIdentityAndGlobalOrder(
    MachiningJobDefinition& definition,
    const std::vector<StableHoleOperationIdentity>& previous)
{
    std::vector<StableHoleOperationIdentity> current =
        captureStableHoleIdentities(
            definition, definition.recognizedHoles);
    const std::vector<int> matched =
        matchStableHoleIdentities(previous, current);

    // 先恢复稳定 ID:旧 ID 未与其他工序冲突时直接沿用,否则按稳定身份
    // 生成确定性新 ID;已用集合同时涵盖槽腔工序以避免冲突。
    std::unordered_set<std::string> usedIds;
    for (const PocketOperationDefinition& operation :
         definition.pocketOperations) {
        usedIds.insert(operation.id);
    }
    for (const ProfileOperationDefinition& operation :
         definition.profileOperations) {
        usedIds.insert(operation.id);
    }
    for (const auto& operation : definition.finishingOperations)
        usedIds.insert(operation.id);
    for (std::size_t index = 0; index < current.size(); ++index) {
        HoleOperationDefinition& operation =
            definition.holeOperations[index];
        const int oldIndex = matched[index];
        if (oldIndex >= 0 &&
            !previous[static_cast<std::size_t>(oldIndex)].id.empty() &&
            !usedIds.contains(
                previous[static_cast<std::size_t>(oldIndex)].id)) {
            operation.id =
                previous[static_cast<std::size_t>(oldIndex)].id;
        } else {
            operation.id = stableHoleOperationId(current[index], usedIds);
        }
        usedIds.insert(operation.id);
        current[index].id = operation.id;
    }

    // 每个孔工序从旧身份中取放置锚点;无直接锚点的工序通过最近的
    // 已锚定工序取序列,保持相对次序稳定。
    std::vector<std::optional<int>> anchors(current.size());
    for (std::size_t index = 0; index < current.size(); ++index) {
        anchors[index] = placementAnchor(current[index], previous);
    }
    for (std::size_t index = 0; index < anchors.size(); ++index) {
        if (anchors[index])
            continue;
        std::optional<std::pair<std::size_t, int>> before;
        std::optional<std::pair<std::size_t, int>> after;
        for (std::size_t distance = 1; distance < anchors.size(); ++distance) {
            if (!before && index >= distance && anchors[index - distance])
                before = {{distance, *anchors[index - distance]}};
            if (!after && index + distance < anchors.size() &&
                anchors[index + distance]) {
                after = {{distance, *anchors[index + distance]}};
            }
            if (before || after) {
                if (before && after && before->first == after->first)
                    anchors[index] = after->second;
                else if (before && (!after || before->first < after->first))
                    anchors[index] = before->second;
                else if (after)
                    anchors[index] = after->second;
                break;
            }
        }
    }

    // 按旧序列分组锚定的孔工序,并记录未锚定工序,便于后续插入全局槽。
    std::map<int, std::vector<std::size_t>> holesAtOldSequence;
    std::vector<std::size_t> unanchored;
    for (std::size_t index = 0; index < anchors.size(); ++index) {
        if (anchors[index])
            holesAtOldSequence[*anchors[index]].push_back(index);
        else
            unanchored.push_back(index);
    }

    struct OldGlobalSlot {
        int sequence = -1;
        bool hole = false;
        std::size_t index = 0;
    };
    std::vector<OldGlobalSlot> globalOrderSlots;
    globalOrderSlots.reserve(
        previous.size() + definition.pocketOperations.size());
    for (std::size_t index = 0; index < previous.size(); ++index)
        globalOrderSlots.push_back(
            {previous[index].sequence, true, index});
    for (std::size_t index = 0;
         index < definition.pocketOperations.size(); ++index) {
        globalOrderSlots.push_back({
            definition.pocketOperations[index].sequence, false, index});
    }
    std::stable_sort(
        globalOrderSlots.begin(), globalOrderSlots.end(),
        [](const OldGlobalSlot& left, const OldGlobalSlot& right) {
            return left.sequence < right.sequence;
        });

    std::vector<MixedOperationOrderEntry> order;
    order.reserve(
        definition.holeOperations.size() +
        definition.pocketOperations.size());
    // 构建混合顺序:先按旧序列遍历全局槽——槽腔直接入列,孔工序按锚点组
    // 整组入列(避免重复),最后把未锚定孔工序追加到末尾。
    std::set<int> appendedHoleAnchors;
    for (const OldGlobalSlot& slot : globalOrderSlots) {
        if (!slot.hole) {
            order.push_back({
                false, slot.index,
                definition.pocketOperations[slot.index].sequence,
            });
            continue;
        }
        if (!appendedHoleAnchors.insert(slot.sequence).second)
            continue;
        const auto group = holesAtOldSequence.find(slot.sequence);
        if (group == holesAtOldSequence.end())
            continue;
        for (const std::size_t newIndex : group->second) {
            const int oldIndex = matched[newIndex];
            order.push_back({
                true,
                newIndex,
                oldIndex >= 0
                    ? std::optional<int>(previous[
                          static_cast<std::size_t>(oldIndex)].sequence)
                    : std::nullopt,
            });
        }
    }
    for (const std::size_t index : unanchored)
        order.push_back({true, index, std::nullopt});

    // 按混合顺序生成新序列并回写孔/槽腔定义。
    const std::vector<int> sequences = assignMixedSequences(order);
    for (std::size_t position = 0; position < order.size(); ++position) {
        if (order[position].hole) {
            definition.holeOperations[order[position].index].sequence =
                sequences[position];
        } else {
            definition.pocketOperations[order[position].index].sequence =
                sequences[position];
        }
    }
    // Profile 是侧壁外轮廓收尾工序；孔重新捕获时保留其定义，并把它们按
    // 原相对顺序稳定地排在新孔/槽腔全局序列之后，避免 sequence 冲突。
    int nextSequence = 0;
    for (const HoleOperationDefinition& operation : definition.holeOperations)
        nextSequence = std::max(nextSequence, operation.sequence + 1);
    for (const PocketOperationDefinition& operation : definition.pocketOperations)
        nextSequence = std::max(nextSequence, operation.sequence + 1);
    std::stable_sort(
        definition.profileOperations.begin(),
        definition.profileOperations.end(),
        [](const ProfileOperationDefinition& left,
           const ProfileOperationDefinition& right) {
            return left.sequence < right.sequence;
        });
    for (ProfileOperationDefinition& operation : definition.profileOperations)
        operation.sequence = nextSequence++;
    std::stable_sort(definition.finishingOperations.begin(), definition.finishingOperations.end(),
        [](const auto& a, const auto& b) { return a.sequence < b.sequence; });
    for (auto& operation : definition.finishingOperations) operation.sequence = nextSequence++;
}

} // namespace oap::manufacturing::hole_job_detail

// ConnectedBlindPocketMerger 实现 — 参见 ConnectedBlindPocketMerger.h。
// 模块: oap_recognition。
#include "ConnectedBlindPocketMerger.h"

#include "analysis/ModelIndex.h"

#include <limits>
#include <unordered_map>
#include <utility>

namespace oap {
namespace {

// 并查集：按秩合并 + 路径压缩，用于管理盲腔区域之间的连通关系。
class DisjointRegionSet {
public:
    explicit DisjointRegionSet(std::size_t size)
        : m_parent(size), m_rank(size, 0)
    {
        for (std::size_t index = 0; index < size; ++index)
            m_parent[index] = index;
    }

    std::size_t find(std::size_t index)
    {
        if (m_parent[index] != index)
            m_parent[index] = find(m_parent[index]);
        return m_parent[index];
    }

    void unite(std::size_t first, std::size_t second)
    {
        first = find(first);
        second = find(second);
        if (first == second)
            return;
        if (m_rank[first] < m_rank[second])
            std::swap(first, second);
        m_parent[second] = first;
        if (m_rank[first] == m_rank[second])
            ++m_rank[first];
    }

private:
    std::vector<std::size_t> m_parent;
    std::vector<unsigned char> m_rank;
};

struct UnitEdgeKey {
    std::string unitId;
    std::string edgeId;

    bool operator==(const UnitEdgeKey&) const = default;
};

struct UnitEdgeKeyHash {
    std::size_t operator()(const UnitEdgeKey& key) const noexcept
    {
        const std::size_t unitHash =
            std::hash<std::string>{}(key.unitId);
        const std::size_t edgeHash =
            std::hash<std::string>{}(key.edgeId);
        return unitHash ^
               (edgeHash + 0x9e3779b9U +
                (unitHash << 6U) + (unitHash >> 2U));
    }
};

} // namespace

// 实现思路：遍历每个盲腔区域的每张面，用 (unitId, edgeId) 与 faceId 分别
// 登记“最先遇到该键的区域”；后续区域遇到相同键时与先到者并查合并。
// 盲槽区域不参与合并（单独成组），连通关系可传递。
std::vector<std::vector<std::size_t>>
GroupConnectedBlindPocketRegions(
    const std::vector<BlindPocketMergeRegion>& regions,
    const ModelIndex& modelIndex)
{
    DisjointRegionSet connected(regions.size());
    std::unordered_map<UnitEdgeKey, std::size_t, UnitEdgeKeyHash>
        firstPocketRegionByEdge;
    std::unordered_map<std::string, std::size_t>
        firstPocketRegionByFace;

    for (std::size_t regionIndex = 0;
         regionIndex < regions.size(); ++regionIndex) {
        const BlindPocketMergeRegion& region =
            regions[regionIndex];
        if (region.featureKind !=
            AxisWireTraceFeatureKind::BlindPocket) {
            continue;
        }

        const auto registerConnectivityFace =
            [&](const std::string& faceId) {
            const Face* face = modelIndex.face(faceId);
            if (!face || face->unit_id.empty())
                return;
            const auto [faceOwner, faceInserted] =
                firstPocketRegionByFace.emplace(
                    faceId, regionIndex);
            if (!faceInserted)
                connected.unite(
                    regionIndex, faceOwner->second);
            for (const std::string& edgeId : face->edge_ids) {
                if (edgeId.empty() || !modelIndex.edge(edgeId))
                    continue;
                const UnitEdgeKey key{face->unit_id, edgeId};
                const auto [owner, inserted] =
                    firstPocketRegionByEdge.emplace(
                        key, regionIndex);
                if (!inserted)
                    connected.unite(regionIndex, owner->second);
            }
        };
        for (const std::string& faceId : region.faceIds)
            registerConnectivityFace(faceId);
        for (const std::string& connectorFaceId :
             region.connectorFaceIds) {
            registerConnectivityFace(connectorFaceId);
        }
    }

    constexpr std::size_t kNoGroup =
        std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> outputIndexByRoot(
        regions.size(), kNoGroup);
    std::vector<std::vector<std::size_t>> groups;
    groups.reserve(regions.size());
    for (std::size_t regionIndex = 0;
         regionIndex < regions.size(); ++regionIndex) {
        const std::size_t root = connected.find(regionIndex);
        std::size_t& outputIndex = outputIndexByRoot[root];
        if (outputIndex == kNoGroup) {
            outputIndex = groups.size();
            groups.emplace_back();
        }
        groups[outputIndex].push_back(regionIndex);
    }
    return groups;
}

// 把来源面集合映射回合并后的连通组：先建 faceId -> 组下标索引，再统计每个
// 来源与各组的重叠面数；重叠最多的来源记为优先来源（重叠数相同时取更小
// 来源下标，保证结果稳定）。
BlindPocketSourceGroupMatches
MatchBlindPocketSourcesToConnectedGroups(
    const std::vector<std::vector<std::size_t>>& connectedGroups,
    const std::vector<BlindPocketMergeRegion>& regions,
    const std::vector<std::vector<std::string>>& sourceFaceIds)
{
    BlindPocketSourceGroupMatches matches;
    matches.sourceIndicesByGroup.resize(connectedGroups.size());
    matches.preferredSourceIndexByGroup.resize(
        connectedGroups.size());
    std::vector<std::size_t> preferredOverlapCount(
        connectedGroups.size(), 0);

    std::unordered_map<std::string, std::size_t>
        groupIndexByFace;
    for (std::size_t groupIndex = 0;
         groupIndex < connectedGroups.size();
         ++groupIndex) {
        for (const std::size_t regionIndex :
             connectedGroups[groupIndex]) {
            if (regionIndex >= regions.size())
                continue;
            for (const std::string& faceId :
                 regions[regionIndex].faceIds) {
                groupIndexByFace.emplace(faceId, groupIndex);
            }
        }
    }

    for (std::size_t sourceIndex = 0;
         sourceIndex < sourceFaceIds.size();
         ++sourceIndex) {
        std::unordered_map<std::size_t, std::size_t>
            overlapCountByGroup;
        for (const std::string& faceId :
             sourceFaceIds[sourceIndex]) {
            const auto group = groupIndexByFace.find(faceId);
            if (group != groupIndexByFace.end())
                ++overlapCountByGroup[group->second];
        }
        for (const auto& [groupIndex, overlapCount] :
             overlapCountByGroup) {
            if (groupIndex >= connectedGroups.size() ||
                overlapCount == 0) {
                continue;
            }
            matches.sourceIndicesByGroup[groupIndex]
                .push_back(sourceIndex);
            const std::optional<std::size_t>& preferred =
                matches.preferredSourceIndexByGroup[
                    groupIndex];
            if (overlapCount >
                    preferredOverlapCount[groupIndex] ||
                (overlapCount ==
                     preferredOverlapCount[groupIndex] &&
                 (!preferred || sourceIndex < *preferred))) {
                preferredOverlapCount[groupIndex] =
                    overlapCount;
                matches.preferredSourceIndexByGroup[
                    groupIndex] = sourceIndex;
            }
        }
    }

    return matches;
}

} // namespace oap

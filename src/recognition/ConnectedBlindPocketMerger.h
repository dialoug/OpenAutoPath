// 将最终面归属后的盲腔子集按共享拓扑边合并；盲槽始终保持独立。
// 模块: oap_recognition（依赖 oap_analysis 的 ModelIndex）。输入是三轴追踪得到
//       的盲腔/盲槽区域及其来源面，输出连通分组与来源面到分组的匹配关系。
// 消费方: ThreeAxisWireTraceUseCase 的盲腔合并阶段。
#pragma once

#include "AxisWireTraceTracker.h"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace oap {

class ModelIndex;

// 一个待合并的盲腔区域：faceIds 为该区域的实际结构面，connectorFaceIds
// 仅参与连通判定（桥接面），不会进入最终结构面集合。
struct BlindPocketMergeRegion {
    AxisWireTraceFeatureKind featureKind =
        AxisWireTraceFeatureKind::BlindPocket;
    std::vector<std::string> faceIds;
    // 仅参与连通判定，不会由本工具加入最终结构面集合。当前用于传入
    // 已确认不是主外表面的来源根面，以及非主外表面根面发出的外环
    // 盲腔终止面，使盲腔可以经内部平面桥接。
    std::vector<std::string> connectorFaceIds;
};

// 返回输入下标的分组。顺序以每组最早出现的输入为准，组内同样保持输入顺序。
// 只有同一 unit 内，通过结构面或 connectorFaceIds 共享有效拓扑边/连接面的
// 盲腔区域才会归入同一组；连通关系可传递。盲槽和无法验证拓扑的区域
// 均作为单独一组返回。
std::vector<std::vector<std::size_t>>
GroupConnectedBlindPocketRegions(
    const std::vector<BlindPocketMergeRegion>& regions,
    const ModelIndex& modelIndex);

// 来源面到连通组的匹配结果：sourceIndicesByGroup 列出每个连通组命中的
// 来源下标，preferredSourceIndexByGroup 给出重叠面最多的来源（无重叠为
// nullopt）。用于排他归属前的来源面重新关联。
struct BlindPocketSourceGroupMatches {
    // 每个连通组命中的来源下标；一个来源可与多个组有面重叠。
    std::vector<std::vector<std::size_t>> sourceIndicesByGroup;
    // 每组重叠面最多的来源；无重叠时为 nullopt。相同重叠数时取较小
    // 来源下标，保证结果稳定。
    std::vector<std::optional<std::size_t>>
        preferredSourceIndexByGroup;
};

// 将排他归属前的来源面集合重新关联到最终连通组。即使来源自身在
// 排他归属后没有剩余 region，仍可通过被其他 region 接管的同名面
// 找回其所属连通组。
BlindPocketSourceGroupMatches
MatchBlindPocketSourcesToConnectedGroups(
    const std::vector<std::vector<std::size_t>>& connectedGroups,
    const std::vector<BlindPocketMergeRegion>& regions,
    const std::vector<std::vector<std::string>>& sourceFaceIds);

} // namespace oap

// 已识别几何与加工决策的显式组合。未解析的输入不会静默产生刀路。
// 属于 oap_toolpath_hole 模块的输入契约：加工决策字段由人工计划或自动规划
// 解析阶段填写，刀路生成器只消费 ready() 为 true 的实例。
#pragma once

#include "analysis/HoleGeometry.h"

namespace oap::toolpath {

enum class HoleMachiningMappingState {
    Ready,
    UnresolvedEntry,
    UnresolvedSetup,
};

// 解析阶段填写的加工决策：入口如何确定、归属哪个装夹面、通孔方向是否
// 已被人工/包络恢复锁定。几何本身继承自 RecognizedHoleGeometry。
struct HoleMachiningDecision {
    // 入口确定方式；Unresolved 表示入口不可靠，生成阶段会跳过该孔。
    HoleEntryResolution entryResolution = HoleEntryResolution::Unresolved;
    // 孔所属的模具六面装夹面；Unresolved 表示尚未归入任何装夹。
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    // true 表示入口侧已由人工确认或包络恢复，通孔方向不再被自动翻转。
    bool preserveEntrySide = false;
    // true 表示人工确认过该孔段；自动重建不得覆盖人工计划。
    bool manuallyConfirmed = false;
    // 入口沿进刀轴到装夹面的距离；-1.0 表示未计算。
    double setupFaceEntryDistance = -1.0;
    // 孔另一端点沿进刀轴到装夹面的距离；-1.0 表示未计算。
    double setupFaceOtherEndpointDistance = -1.0;
    // 记录装夹面是依最近距离、拓扑破平还是稳定顺序选定，供 UI/诊断解释。
    MoldSetupFaceDecision setupFaceDecision =
        MoldSetupFaceDecision::None;
};

// 孔加工输入：孔几何 + 映射状态 + 工艺决策。
struct HoleMachiningInput : RecognizedHoleGeometry, HoleMachiningDecision {
    HoleMachiningMappingState mappingState =
        HoleMachiningMappingState::UnresolvedEntry;

    bool ready() const noexcept
    {
        return mappingState == HoleMachiningMappingState::Ready;
    }
};

} // namespace oap::toolpath

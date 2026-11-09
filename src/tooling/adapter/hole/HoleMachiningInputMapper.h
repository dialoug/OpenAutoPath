// tooling/toolpath 边界：为纯识别几何绑定显式装夹与入口决策。
// 模块归属:oap_toolpath_adapter;把识别层 RecognizedHoleGeometry 翻译为
// 刀路层 HoleMachiningInput(含 mappingState),是自动孔规划的前置输入。
#pragma once

#include "toolpath/hole/HoleMachiningInput.h"
#include "analysis/Model.h"
#include "analysis/MoldCoordinateFrame.h"

#include <vector>

namespace oap::tooling {

// 孔段映射器:把识别几何与决策/毛坯包络信息合并为刀路可消费的输入。
class HoleMachiningInputMapper {
public:
    // 按显式给出的决策绑定入口与装夹,并据决策完整性设置 mappingState。
    toolpath::HoleMachiningInput map(
        RecognizedHoleGeometry geometry,
        toolpath::HoleMachiningDecision decision) const;

    // 自动解析:先用模具表面分类定装夹面与入口,失败时退回轴向分类。
    toolpath::HoleMachiningInput resolve(
        RecognizedHoleGeometry geometry,
        const Bbox& bounds,
        const MoldCoordinateFrame& moldFrame) const;

    // 批量解析全部孔段,保持输入顺序。
    std::vector<toolpath::HoleMachiningInput> resolve(
        const std::vector<RecognizedHoleGeometry>& geometries,
        const Bbox& bounds,
        const MoldCoordinateFrame& moldFrame) const;

};

} // namespace oap::tooling

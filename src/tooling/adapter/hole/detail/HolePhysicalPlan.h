// 把识别层的圆柱段转换为自动选刀使用的物理孔模型。
// 属于 hole 模块 detail 私有实现:将同一轴线上的多段孔(沉孔+通孔等)
// 归并为一个物理孔,供中心钻/钻削/平底铣按整孔约束选刀。
#pragma once

#include <cstddef>
#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include "toolpath/hole/HoleMachiningInput.h"

namespace oap::tooling::detail {

// 一个物理孔(同轴多段归并):稳定编号、最小直径段及其深度约束,
// 以及加工入口(外端)与向内轴向;由 buildPhysicalHolePlan 创建。
struct PlannedPhysicalHole {
    std::size_t stableIndex = 0;
    oap::MoldSetupFace setupFace = oap::MoldSetupFace::Unresolved;
    std::vector<std::size_t> segmentIndices;
    std::vector<std::size_t> minimumDiameterSegmentIndices;
    std::size_t outerSegmentIndex = 0;
    std::size_t minimumSegmentIndex = 0;
    gp_Pnt outerEntry;
    gp_Pnt deepestBottom;
    gp_Vec inwardAxis;
    double minimumDiameter = 0.0;
};

// 按装夹面收集该面孔段并归并同轴组,为每组生成物理孔计划;
// nextStableIndex 跨面连续,保证物理孔编号全局唯一。
std::vector<PlannedPhysicalHole> buildPhysicalHolePlan(
    const std::vector<oap::toolpath::HoleMachiningInput>& holes,
    oap::MoldSetupFace face,
    std::size_t& nextStableIndex);

// 以最小直径段为基准构造“引导孔”:入口用物理孔外端,
// 深度取最深底,孔径取最小直径,供钻削/点钻阶段使用。
oap::toolpath::HoleMachiningInput makePilotHole(
    const PlannedPhysicalHole& physical,
    const std::vector<oap::toolpath::HoleMachiningInput>& holes);

} // namespace oap::tooling::detail

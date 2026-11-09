// 将同一装夹面上属于同一个台阶/沉头孔的圆柱段组合为一个访问节点。
// 属于 oap_toolpath_hole 模块（detail/ 目录下的实现为内部细节）；供入口
// 包络恢复与访问顺序优化消费，不依赖任何加工参数。
#pragma once

#include <cstddef>
#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace oap::toolpath {

// 参与物理孔归组的单个孔段输入：携带轴向信息以便做共轴/共线判定。
struct HolePhysicalSegmentInput {
    std::size_t localIndex = 0;      // 本组 segments 内的下标。
    std::size_t originalIndex = 0;   // 原始输入向量中的下标，用于稳定排序。
    gp_Pnt entry;
    gp_Pnt bottom;
    gp_Vec inwardAxis;               // 从入口指向孔底的单位轴方向。
    double radius = 0.0;
};

// 一个物理孔：由若干共轴相接的孔段组成，anchorSegmentIndex 指向代表段。
struct HolePhysicalGroup {
    std::vector<std::size_t> segmentIndices;   // 段下标，已按轴向来序稳定排序。
    std::size_t anchorSegmentIndex = 0;        // 最外侧代表段（轴向最浅）。
    std::size_t originalIndex = 0;             // 组内最小的原始段下标。
};

// 按线性公差合并共轴、轴向区间相接（或重叠）的孔段为物理孔组。
std::vector<HolePhysicalGroup> groupPhysicalHoleSegments(
    const std::vector<HolePhysicalSegmentInput>& segments,
    double linearTolerance);

} // namespace oap::toolpath

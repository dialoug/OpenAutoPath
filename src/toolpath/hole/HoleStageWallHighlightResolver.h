// HoleStageWallHighlightResolver —— 按孔段父子关系解析下刀路径需要高亮的孔壁。
// 属于 oap_toolpath_hole 模块；选中一个孔段时，除自身壁面外还包含其父级
// 沉孔段壁面，供 UI 高亮下刀入口附近的真实干涉区域。
#pragma once

#include "toolpath/hole/HoleMachiningInput.h"

#include <cstddef>
#include <string>
#include <vector>

#include "analysis/FeatureTypes.h"

namespace oap::toolpath {

// 解析单个选中孔段的高亮壁面 id 列表（含自身及父沉孔段壁面）。
std::vector<std::string> resolveHoleStageWallHighlightFaceIds(
    const std::vector<HoleMachiningInput>& holes,
    std::size_t selectedHoleIndex);

// 同上，批量解析多个选中孔段并去重合并壁面 id。
std::vector<std::string> resolveHoleStageWallHighlightFaceIds(
    const std::vector<HoleMachiningInput>& holes,
    const std::vector<std::size_t>& selectedHoleIndices);

} // namespace oap::toolpath

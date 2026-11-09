// 固定方向 Dexel 材料区间的通用集合运算。
//
// 材料列使用按 lower 递增、互不重叠的有限区间。这里的操作只更新一列材料，
// 不依赖槽腔、孔或具体刀具语义，供后续原生 Dexel 刀具扫掠共同复用。
//
// 属于 oap_toolpath_pocket 的毛坯仿真层：DexelInterval 定义于
// DexelDisplayMesh.h，本文件提供纯函数式区间差运算，由 MultiDexelStock
// 的减料内核（detail/MultiDexelTileUpdates.cpp）在每一根射线列上调用。
#pragma once

#include <vector>

#include "toolpath/pocket/simulation/DexelDisplayMesh.h"

namespace oap::toolpath {

// 从 material 中减去 removal，并保持材料区间的既有顺序和不重叠约束。
//
// tolerance 必须为有限非负数。无效、厚度不超过 tolerance 或未命中材料的
// removal 是无操作；返回 false 时 material 保持不变。
[[nodiscard]] bool subtractDexelInterval(
    std::vector<DexelInterval>& material,
    const DexelInterval& removal,
    double tolerance);

} // namespace oap::toolpath

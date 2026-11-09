// 内部孔几何与策略生成器。HoleMachining 负责批量校验和分组，
// 此处函数每次只生成一个加工操作。
// 属于 oap_toolpath_hole 模块 detail 私有部分；策略生成实现分散在
// detail/strategies/ 下，本文件是它们的统一定义点。
#pragma once

#include "toolpath/hole/HoleMachiningInput.h"

#include <string>
#include <vector>

#include <gp_Vec.hxx>

#include "HoleRapidPlanner.h"
#include "toolpath/hole/HoleMachining.h"

namespace oap::toolpath::detail {

inline constexpr double kHoleTolerance = 1.0e-7; // 孔策略共用的几何容差（mm）。

// 以孔入口为原点、孔轴为 z 的局部正交坐标系，供策略按轴/半径/角度采样。
struct HoleFrame {
    gp_Pnt origin;   // 孔入口点。
    gp_Vec axis;     // 从入口指向孔底的单位轴。
    gp_Vec u;        // 与 axis 垂直的第一个径向基向量。
    gp_Vec v;        // 与 axis、u 构成右手系的第二个径向基向量。
    double depth = 0.0;  // 入口到底点的孔深（mm）。
};

// 由孔输入构造 HoleFrame；无效深度/端点/轴时返回 false。
bool makeHoleFrame(const HoleMachiningInput& hole, HoleFrame& frame);

// 小孔钻削：中心线固定循环 + 等价展开移动，含钻尖深度补偿。
HoleMachiningOperation buildSmallDrill(
    const HoleMachiningInput& hole, const HoleFrame& frame, int operationIndex,
    const ToolpathParams& params, const gp_Pnt* previousClearance,
    HoleRapidPlanner* rapidPlanner);

// 中孔螺旋铣：按螺距下刀并在孔底追加一圈定径精加工。
HoleMachiningOperation buildMediumHelix(
    const HoleMachiningInput& hole, const HoleFrame& frame, double cutterRadius,
    int operationIndex, const ToolpathParams& params,
    HoleMachiningResult& result, const gp_Pnt* previousClearance,
    HoleRapidPlanner* rapidPlanner);

// 大孔分层螺旋清料：每层从中心向外扩展并执行外圆精加工。
HoleMachiningOperation buildLargeSpiral(
    const HoleMachiningInput& hole, const HoleFrame& frame, double cutterRadius,
    int operationIndex, const ToolpathParams& params,
    HoleMachiningResult& result, const gp_Pnt* previousClearance,
    HoleRapidPlanner* rapidPlanner);

} // namespace oap::toolpath::detail

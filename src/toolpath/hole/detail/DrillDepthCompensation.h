// 将孔圆柱终止平面换算为中心线钻削的实际钻尖目标。
// 属于 oap_toolpath_hole 模块 detail 私有部分，仅由小孔钻削策略使用。
#pragma once

#include "toolpath/hole/HoleMachiningInput.h"

#include <gp_Pnt.hxx>

#include "analysis/FeatureTypes.h"
#include "toolpath/core/ToolpathParams.h"

namespace oap::toolpath::detail {

struct HoleFrame;

// 钻尖深度目标：实际刀尖点以及组成它的钻尖长度与通孔突破量。
struct DrillDepthTarget {
    gp_Pnt point;          // 补偿后的刀尖目标点（深于或等于 hole.bottom）。
    double tipLength = 0.0;      // 圆柱刃口平面到钻尖的轴向高度（mm）。
    double breakthrough = 0.0;   // 通孔突破量（mm）。
};

// HoleMachiningInput::bottom 表示完整刀径应到达的圆柱终止平面。人工确认的通孔
// 始终使用钻尖和突破量；其余孔只在底部轮廓明确时补偿钻尖，避免盲孔过切。
DrillDepthTarget resolveDrillDepthTarget(
    const HoleMachiningInput& hole,
    const HoleFrame& frame,
    const ToolpathParams& params);

} // namespace oap::toolpath::detail

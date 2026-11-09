// DrillDepthCompensation —— 钻尖深度补偿的内部实现。属于 oap_toolpath_hole
// 模块 detail 私有部分：把孔圆柱终止平面换算为中心线钻削的实际钻尖目标，
// 只被 SmallDrillStrategy 消费，不构成稳定对外接口。
#include "toolpath/hole/detail/DrillDepthCompensation.h"

#include <algorithm>
#include <cmath>

#include "toolpath/hole/DrillTipParameterBinding.h"
#include "toolpath/hole/detail/HoleStrategyGenerators.h"

namespace oap::toolpath::detail {
namespace {

bool hasFullDiameterDepth(HoleBottomProfile profile)
{
    return profile == HoleBottomProfile::Open ||
           profile == HoleBottomProfile::Planar ||
           profile == HoleBottomProfile::Conical;
}

} // namespace

DrillDepthTarget resolveDrillDepthTarget(
    const HoleMachiningInput& hole,
    const HoleFrame& frame,
    const ToolpathParams& params)
{
    DrillDepthTarget result;
    result.point = hole.bottom;
    const bool throughHole =
        hole.kind == FeatureKind::ThroughHole ||
        (hole.kind == FeatureKind::CountersinkHole &&
         hole.bottomProfile == HoleBottomProfile::Open);
    if (!throughHole && !hasFullDiameterDepth(hole.bottomProfile))
        return result;

    if (std::isfinite(params.tool.tipCompensation) &&
        params.tool.tipCompensation > 0.0) {
        result.tipLength = params.tool.tipCompensation;
    } else {
        result.tipLength = oap::toolpath::drillTipLength(
            params.tool.diameter, params.hole.drillTipAngleDeg);
    }
    if (throughHole &&
        std::isfinite(params.hole.throughHoleBreakthrough)) {
        result.breakthrough =
            std::max(0.0, params.hole.throughHoleBreakthrough);
    }
    const double extension = result.tipLength + result.breakthrough;
    if (extension > 0.0) {
        result.point = hole.bottom.Translated(
            frame.axis.Multiplied(extension));
    }
    return result;
}

} // namespace oap::toolpath::detail

// 孔识别的纯几何/拓扑事实；不包含装夹面、刀具、策略、Operation 或 Job 状态。
// 属于 oap_analysis 模块的识别结果契约：recognition/ 产生本结构，人工计划、
// 刀路与显示层直接消费，不依赖识别实现细节。
#pragma once

#include "analysis/FeatureTypes.h"

#include <gp_Pnt.hxx>

#include <string>
#include <vector>

namespace oap {

// 一个已识别孔段的最小几何描述：入口/底部端点、半径与拓扑来源面集合。
// 值类型、无所有权；深度由入口与底部端点距离推导。
struct RecognizedHoleGeometry {
    FeatureKind kind = FeatureKind::BlindHole;
    gp_Pnt entry;          // 刀具进入侧端面中心点
    gp_Pnt bottom;         // 孔底中心点（通孔时为另一端面中心点）
    double radius = 0.0;   // 孔段半径（mm）
    HoleBottomProfile bottomProfile = HoleBottomProfile::Unknown;
    std::vector<std::string> sourceFaceIds;        // 参与识别的源面 id
    std::vector<std::string> stageWallFaceIds;     // 孔段壁面 id（供可视化/后处理）
    HoleStageRole stageRole = HoleStageRole::Standard;
    HoleParentRelation parentRelation = HoleParentRelation::None;
    std::string parentCounterboreFaceId;           // 父沉孔壁面 id
    std::string parentCounterboreFloorFaceId;      // 父沉孔平底面 id
    double parentCounterboreRadius = 0.0;          // 父沉孔半径（mm）

    // 孔深：入口与底部端点的直线距离（mm）。
    double depth() const noexcept
    {
        return entry.Distance(bottom);
    }
};

} // namespace oap

// 外轮廓装夹上下文分配（oap_app_workflow_cam）。
// 按 +Z,+X,+Y,-X,-Y,-Z 的稳定六面调度顺序排序外轮廓区域并分配装夹上下文，
// 与其它自动 CAM 工作流保持一致。
#pragma once

#include <vector>

#include "analysis/MoldCoordinateFrame.h"
#include "toolpath/profile/geometry/ProfileRegion.h"
#include "toolpath/workflow/profile/ProfileToolpathService.h"

namespace oap::app {

// Return profile regions in the same stable six-face scheduling order used by
// the other automatic CAM workflows: +Z, +X, +Y, -X, -Y, -Z. Regions whose
// opening direction cannot be classified are retained at the end. The sort is
// stable so recognition order is preserved within one setup face.
std::vector<toolpath::ProfileRegion> orderProfileRegionsBySetup(
    const std::vector<toolpath::ProfileRegion>& regions,
    const MoldCoordinateFrame& moldFrame);

// Classify each automatically recognized profile against the mold coordinate
// frame. Imported STEP world axes need not match the mold-local six-face frame.
void assignProfileSetupContexts(
    toolpath::ProfileToolpathBatch& batch,
    const std::vector<toolpath::ProfileRegion>& regions,
    const MoldCoordinateFrame& moldFrame);

} // namespace oap::app

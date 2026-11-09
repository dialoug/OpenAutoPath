// 外轮廓刀路后台调用器：协调刀库自动选刀、进度和取消。
#pragma once

#include <string>
#include <vector>

#include "app/workflows/shared/WorkflowInteraction.h"
#include "tooling/adapter/profile/ProfileAutomaticToolpath.h"
#include "toolpath/profile/planning/ProfileDirectionPlanner.h"

namespace oap::app {

tooling::ProfileAutomaticToolpathResult
generateProfilesWithProgress(
    const WorkflowProgressPresenter& progress,
    const tooling::ToolLibrary& library,
    const std::vector<toolpath::ProfileRegion>& regions,
    const toolpath::ToolpathParams& params,
    const std::string& operationPrefix,
    bool directContour = false,
    const toolpath::ProfileDirectionPlanningOptions* directionOptions = nullptr,
    toolpath::ProfileDirectionPlan* directionPlan = nullptr);

} // namespace oap::app

// 槽/腔刀路后台调用器：只协调生成、进度和取消。
#pragma once

#include <string>
#include <vector>

#include "app/workflows/shared/WorkflowInteraction.h"
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"
#include "toolpath/workflow/pocket/PocketToolpathService.h"
#include "tooling/adapter/pocket/PocketAutomaticToolpath.h"

namespace oap::app {

// 后台按刀库自动组合选刀生成槽/腔刀路（带进度与取消）。
tooling::PocketAutomaticToolpathResult
generateAutomaticPocketsWithProgress(
    const WorkflowProgressPresenter& progress,
    const tooling::ToolLibrary& library,
    const std::vector<toolpath::PocketRegion>& regions,
    const toolpath::ToolpathParams& params,
    const std::string& operationPrefix);

} // namespace oap::app

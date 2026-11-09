// 孔刀路后台调用器：通过交互端口发布进度，不依赖具体 UI 框架。
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "app/workflows/shared/WorkflowInteraction.h"
#include "recognition/FeatureRecognizer.h"
#include "toolpath/workflow/hole/HoleToolpathService.h"
#include "tooling/adapter/hole/HoleAutomaticToolpath.h"

namespace oap::app {

// 后台识别孔几何（带进度）：返回空表示取消或未获准启动；后者写入 error。
std::optional<HoleGeometryRecognitionResult> recognizeHolesWithProgress(
    const WorkflowProgressPresenter& progress,
    const Model& model,
    const StepIndexer& indexer,
    std::string* error = nullptr);

// 后台按刀库自动选刀生成孔刀路（带进度与取消）。
tooling::HoleAutomaticToolpathResult generateAutomaticHolesWithProgress(
    const WorkflowProgressPresenter& progress,
    const tooling::ToolLibrary& library,
    const std::vector<toolpath::HoleMachiningInput>& inputs,
    const toolpath::ToolpathParams& params,
    const toolpath::HoleRapidPlanningContext& rapidContext);

} // namespace oap::app

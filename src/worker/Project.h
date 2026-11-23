#pragma once
#include "support/Json.h"
#include <vector>
#include "analysis/StepIndexer.h"
#include "manufacturing/job/MachiningJob.h"
#include "tooling/adapter/hole/HoleAutomaticToolpath.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

namespace oap::worker
{
void captureProject(const oap::JsonObject &request, const tooling::HoleAutomaticToolpathResult &generated,
                    const std::vector<toolpath::HoleMachiningInput> &inputs,
                    const toolpath::MoldSetupWorkFrames &frames, const tooling::ToolLibrary &library);
oap::JsonObject processProject(const oap::JsonObject &request, const StepIndexer &index, const Model &model,
                           const tooling::ToolLibrary &library, const toolpath::MoldSetupWorkFrames &frames,
                           const toolpath::HoleRapidPlanningContext &context);
} // namespace oap::worker

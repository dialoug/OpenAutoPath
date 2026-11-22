#pragma once
#include "analysis/StepIndexer.h"
#include "manufacturing/job/MachiningJob.h"
#include "tooling/adapter/hole/HoleAutomaticToolpath.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"
#include <QJsonObject>

namespace oap::worker
{
void captureProject(const QJsonObject &request, const tooling::HoleAutomaticToolpathResult &generated,
                    const std::vector<toolpath::HoleMachiningInput> &inputs,
                    const toolpath::MoldSetupWorkFrames &frames, const tooling::ToolLibrary &library);
QJsonObject processProject(const QJsonObject &request, const StepIndexer &index, const Model &model,
                           const tooling::ToolLibrary &library, const toolpath::MoldSetupWorkFrames &frames,
                           const toolpath::HoleRapidPlanningContext &context);
} // namespace oap::worker

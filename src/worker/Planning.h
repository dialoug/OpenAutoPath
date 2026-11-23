#pragma once
#include "support/Json.h"
#include <vector>
#include "analysis/HoleGeometry.h"
#include "manufacturing/job/MachiningJob.h"
#include "recognition/ThreeAxisWireTraceUseCase.h"
#include "tooling/library/ToolLibrary.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

namespace oap::worker
{
ThreeAxisWireTraceUseCaseResult traceRegions(const Model &model, const StepIndexer &index,
                                             const std::vector<RecognizedHoleGeometry> &holes);
oap::JsonArray planningFeatures(const ThreeAxisWireTraceUseCaseResult &trace);
void planFeature(manufacturing::MachiningJob &job, const oap::JsonObject &request, const Model &model,
                 const StepIndexer &index, const tooling::ToolLibrary &library,
                 const toolpath::MoldSetupWorkFrames &frames, const TopoDS_Shape &part);
} // namespace oap::worker

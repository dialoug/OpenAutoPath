#pragma once
#include "analysis/HoleGeometry.h"
#include "manufacturing/job/MachiningJob.h"
#include "recognition/ThreeAxisWireTraceUseCase.h"
#include "tooling/library/ToolLibrary.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"
#include <QJsonArray>
#include <QJsonObject>

namespace oap::worker
{
ThreeAxisWireTraceUseCaseResult traceRegions(const Model &model, const StepIndexer &index,
                                             const std::vector<RecognizedHoleGeometry> &holes);
QJsonArray planningFeatures(const ThreeAxisWireTraceUseCaseResult &trace);
void planFeature(manufacturing::MachiningJob &job, const QJsonObject &request, const Model &model,
                 const StepIndexer &index, const tooling::ToolLibrary &library,
                 const toolpath::MoldSetupWorkFrames &frames, const TopoDS_Shape &part);
} // namespace oap::worker

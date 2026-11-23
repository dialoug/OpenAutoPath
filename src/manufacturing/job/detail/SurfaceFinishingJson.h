#pragma once
#include "support/Json.h"
#include <vector>
#include <string>
#include "manufacturing/job/MachiningJob.h"

namespace oap::manufacturing::detail {
oap::JsonArray finishingOperationsToJson(const std::vector<SurfaceFinishingOperationDefinition>& operations);
std::vector<SurfaceFinishingOperationDefinition> finishingOperationsFromJson(const oap::JsonArray& array);
std::string validateFinishingJson(const oap::JsonObject& job, bool required);
bool validFinishingParameters(const toolpath::SurfaceFinishingParams& parameters);
bool finishingCutterMatchesTool(toolpath::FinishingCutterKind kind, const toolpath::ToolpathToolContext& tool);
} // namespace oap::manufacturing::detail

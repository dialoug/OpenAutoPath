#pragma once
#include <QJsonArray>
#include <QJsonObject>
#include <QString>
#include "manufacturing/job/MachiningJob.h"

namespace oap::manufacturing::detail {
QJsonArray finishingOperationsToJson(const std::vector<SurfaceFinishingOperationDefinition>& operations);
std::vector<SurfaceFinishingOperationDefinition> finishingOperationsFromJson(const QJsonArray& array);
QString validateFinishingJson(const QJsonObject& job, bool required);
bool validFinishingParameters(const toolpath::SurfaceFinishingParams& parameters);
bool finishingCutterMatchesTool(toolpath::FinishingCutterKind kind, const toolpath::ToolpathToolContext& tool);
} // namespace oap::manufacturing::detail

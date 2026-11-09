#pragma once
#include <QJsonObject>
#include "toolpath/core/ToolpathToolContext.h"
namespace oap::app::detail {
QJsonObject encodeToolContext(const toolpath::ToolpathToolContext& snapshot);
bool decodeToolContext(const QJsonObject& object, toolpath::ToolpathToolContext& value);
}

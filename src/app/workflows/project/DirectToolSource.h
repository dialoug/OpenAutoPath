#pragma once

#include <QString>
#include "tooling/library/ToolLibrary.h"
#include "toolpath/core/ToolpathToolContext.h"

namespace oap::app {
enum class DirectToolProcess { Hole, Pocket, Profile };

// A private, one-tool planning source. Never inserts into the user catalog or
// changes the active library; formal capture stores an empty library ID.
bool makeDirectToolLibrary(const toolpath::ToolpathToolContext& tool,
    DirectToolProcess process, tooling::ToolLibrary& library, QString& error);
} // namespace oap::app

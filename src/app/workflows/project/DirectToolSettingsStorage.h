#pragma once

#include <QString>
#include "toolpath/core/ToolpathToolContext.h"
class QSettings;

namespace oap::app {
// Independent snapshots; no entries are added to the user's tool catalog.
class DirectToolSettingsStorage {
public:
    static bool load(QSettings&, toolpath::ToolpathToolContext& hole,
                     toolpath::ToolpathToolContext& pocket, toolpath::ToolpathToolContext& profile,
                     QString* error = nullptr);
    static bool save(QSettings&, const toolpath::ToolpathToolContext& hole,
                     const toolpath::ToolpathToolContext& pocket, const toolpath::ToolpathToolContext& profile,
                     QString* error = nullptr);
    static bool loadDefault(toolpath::ToolpathToolContext& hole, toolpath::ToolpathToolContext& pocket,
                            toolpath::ToolpathToolContext& profile, QString* error = nullptr);
    static bool saveDefault(const toolpath::ToolpathToolContext& hole, const toolpath::ToolpathToolContext& pocket,
                            const toolpath::ToolpathToolContext& profile, QString* error = nullptr);
};
}

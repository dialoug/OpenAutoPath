#pragma once

#include <QString>
#include "app/workflows/engraving/EngravingSettings.h"

class QSettings;

namespace oap::app {

// Separate process preferences: changing engraving tools never changes the
// active library or the hole/pocket/profile parameter groups.
class EngravingSettingsStorage {
public:
    static bool load(QSettings& settings, EngravingSettings& value, QString* error = nullptr);
    static bool save(QSettings& settings, const EngravingSettings& value, QString* error = nullptr);
    static bool loadDefault(EngravingSettings& value, QString* error = nullptr);
    static bool saveDefault(const EngravingSettings& value, QString* error = nullptr);
    static bool equal(const EngravingSettings& first, const EngravingSettings& second);
};

} // namespace oap::app

#pragma once

#include "app/workflows/project/ToolpathParameterEditorState.h"
#include <QString>

class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QSpinBox;
class QWidget;

namespace oap::app {
// Shared in-page source/geometry editor for hole, pocket and profile tools.
class DirectToolParameterSection {
public:
    DirectToolParameterSection(QFormLayout& form, QWidget& parent,
        const QString& prefix, bool allowDrill, ToolSelectionMode initialMode,
        const toolpath::ToolpathToolContext& initialTool);
    ToolSelectionMode mode() const;
    toolpath::ToolpathToolContext tool() const;
    QString validationError() const;
private:
    QString m_prefix;
    ToolSelectionMode m_initialMode;
    toolpath::ToolpathToolContext m_initial;
    bool m_changed = false;
    QComboBox *m_mode = nullptr, *m_type = nullptr;
    QSpinBox* m_number = nullptr;
    QDoubleSpinBox *m_diameter = nullptr, *m_length = nullptr, *m_spindle = nullptr,
        *m_feed = nullptr, *m_plunge = nullptr, *m_tip = nullptr, *m_corner = nullptr,
        *m_overall = nullptr, *m_shank = nullptr, *m_holderDiameter = nullptr,
        *m_holderLength = nullptr, *m_spindleDiameter = nullptr, *m_spindleLength = nullptr;
};
} // namespace oap::app

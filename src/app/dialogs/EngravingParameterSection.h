#pragma once

#include <QString>

#include "app/workflows/project/ToolpathParameterEditorState.h"

class QFormLayout;
class QWidget;
class QComboBox;
class QDoubleSpinBox;
class QSpinBox;

namespace oap::app {

// An embedded section of the shared process-parameter dialog, not a separate
// generation window. It edits only a configuration snapshot.
class EngravingParameterSection {
public:
    EngravingParameterSection(QFormLayout& form, QWidget& parent,
                             const ToolpathParameterEditorState& initial);
    QString validationError() const;
    EngravingSettings value() const;
private:
    EngravingSettings m_initial;
    QComboBox *m_scope, *m_mode, *m_machiningMode, *m_library, *m_cutter;
    QSpinBox* m_number;
    QDoubleSpinBox *m_diameter, *m_length, *m_corner, *m_spindle, *m_feed, *m_plunge;
    QDoubleSpinBox *m_maxDiameter, *m_stepOver, *m_sample, *m_mesh, *m_allowance, *m_clearance;
    QDoubleSpinBox *m_overall, *m_shank, *m_holderDiameter, *m_holderLength;
    QDoubleSpinBox *m_maxStepDown, *m_roughingAllowance;
};

} // namespace oap::app

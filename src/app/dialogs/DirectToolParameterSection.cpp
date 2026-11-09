#include "app/dialogs/DirectToolParameterSection.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QSpinBox>
#include <QWidget>

namespace oap::app {
DirectToolParameterSection::DirectToolParameterSection(QFormLayout& form, QWidget& parent,
    const QString& prefix, bool allowDrill, ToolSelectionMode initialMode,
    const toolpath::ToolpathToolContext& initialTool)
    : m_prefix(prefix), m_initialMode(initialMode), m_initial(initialTool)
{
    m_mode = new QComboBox(&parent);
    m_mode->setObjectName(prefix + "ToolMode");
    m_mode->addItem(QStringLiteral("活动刀库自动选刀"), static_cast<int>(ToolSelectionMode::AutomaticLibrary));
    m_mode->addItem(QStringLiteral("手动单刀（直接填写）"), static_cast<int>(ToolSelectionMode::SingleTool));
    m_mode->setCurrentIndex(m_mode->findData(static_cast<int>(initialMode)));
    form.addRow(QStringLiteral("刀具来源"), m_mode);
    m_type = new QComboBox(&parent);
    m_type->setObjectName(prefix + "ToolType");
    if (allowDrill) m_type->addItem(QStringLiteral("钻头"), "drill");
    m_type->addItem(QStringLiteral("平底立铣刀"), "end_mill");
    if (!allowDrill) {
        m_type->addItem(QStringLiteral("球刀"), "ball_end_mill");
        m_type->addItem(QStringLiteral("圆鼻刀"), "bull_nose");
        m_type->setToolTip(QStringLiteral("球刀和圆鼻刀可生成分层刀路；槽底、根部可能保留余料，请通过 IPW 检查。"));
    }
    const int typeIndex = m_type->findData(QString::fromStdString(initialTool.toolType));
    m_type->setCurrentIndex(typeIndex >= 0 ? typeIndex : 0);
    form.addRow(QStringLiteral("单刀类型"), m_type);
    m_number = new QSpinBox(&parent);
    m_number->setObjectName(prefix + "ToolNumber");
    m_number->setRange(1, 9999);
    m_number->setValue(initialTool.toolNumber > 0 ? initialTool.toolNumber : 1);
    form.addRow(QStringLiteral("单刀刀号 T"), m_number);
    const auto number = [&](const char* name, const QString& label, double value,
                            double minimum, double maximum, const QString& unit = QStringLiteral(" mm")) {
        auto* field = new QDoubleSpinBox(&parent);
        field->setObjectName(prefix + QString::fromLatin1(name));
        field->setRange(minimum, maximum);
        field->setDecimals(6);
        field->setSingleStep(0.1);
        field->setSuffix(unit);
        field->setValue(value);
        form.addRow(label, field);
        return field;
    };
    const auto positive = [](double value, double fallback) { return value > 0 ? value : fallback; };
    m_diameter = number("ToolDiameter", QStringLiteral("单刀直径"), positive(initialTool.diameter, 6), 0.001, 1000);
    m_corner = number("ToolCornerRadius", QStringLiteral("单刀圆角半径"), positive(initialTool.cornerRadius, 0.5), 0.001, 500);
    m_length = number("ToolLength", QStringLiteral("单刀刃长"), positive(initialTool.cuttingEdgeLength, 30), 0.001, 10000);
    m_spindle = number("ToolSpindle", QStringLiteral("单刀主轴转速"), positive(initialTool.spindleSpeed, 8000), 1, 1000000, " rpm");
    m_feed = number("ToolFeed", QStringLiteral("单刀切削进给"), positive(initialTool.horizontalFeed, 600), 1, 1000000, " mm/min");
    m_plunge = number("ToolPlunge", QStringLiteral("单刀进刀进给"), positive(initialTool.verticalFeed, 150), 1, 1000000, " mm/min");
    m_tip = number("ToolTipAngle", QStringLiteral("单刀钻尖角"), positive(initialTool.tipAngle, 118), 1, 179, QStringLiteral("°"));
    m_overall = number("ToolOverall", QStringLiteral("单刀总长 / 伸出（可选）"), initialTool.overallLength, 0, 10000);
    m_shank = number("ToolShank", QStringLiteral("单刀刀杆直径（可选）"), initialTool.shankDiameter, 0, 1000);
    m_holderDiameter = number("ToolHolderDiameter", QStringLiteral("单刀刀柄直径（可选）"), initialTool.holderDiameter, 0, 1000);
    m_holderLength = number("ToolHolderLength", QStringLiteral("单刀刀柄长度（可选）"), initialTool.holderLength, 0, 10000);
    m_spindleDiameter = number("ToolSpindleDiameter", QStringLiteral("单刀主轴包络直径（可选）"), initialTool.spindleEnvelopeDiameter, 0, 10000);
    m_spindleLength = number("ToolSpindleLength", QStringLiteral("单刀主轴包络长度（可选）"), initialTool.spindleEnvelopeLength, 0, 10000);
    const auto refresh = [this, &form] {
        const bool single = mode() == ToolSelectionMode::SingleTool;
        for (QWidget* widget : std::initializer_list<QWidget*>{m_type, m_number, m_diameter, m_length,
                m_spindle, m_feed, m_plunge, m_overall, m_shank, m_holderDiameter, m_holderLength,
                m_spindleDiameter, m_spindleLength})
            form.setRowVisible(widget, single);
        form.setRowVisible(m_tip, single && m_type->currentData().toString() == "drill");
        form.setRowVisible(m_corner, single && m_type->currentData().toString() == "bull_nose");
    };
    QObject::connect(m_mode, &QComboBox::currentIndexChanged, &parent, [this, refresh] { m_changed = true; refresh(); });
    QObject::connect(m_type, &QComboBox::currentIndexChanged, &parent, [this, refresh] { m_changed = true; refresh(); });
    QObject::connect(m_number, &QSpinBox::valueChanged, &parent, [this] { m_changed = true; });
    for (auto* field : {m_diameter, m_length, m_spindle, m_feed, m_plunge, m_tip, m_corner, m_overall,
            m_shank, m_holderDiameter, m_holderLength, m_spindleDiameter, m_spindleLength})
        QObject::connect(field, &QDoubleSpinBox::valueChanged, &parent, [this] { m_changed = true; });
    refresh();
}

ToolSelectionMode DirectToolParameterSection::mode() const
{
    return static_cast<ToolSelectionMode>(m_mode->currentData().toInt());
}

QString DirectToolParameterSection::validationError() const
{
    if (mode() != ToolSelectionMode::SingleTool) return {};
    const auto type = m_type->currentData().toString();
    const double radius = type == "ball_end_mill" ? m_diameter->value() * 0.5
        : type == "bull_nose" ? m_corner->value() : 0.0;
    if (type == "bull_nose" && !(radius > 0.0 && radius < m_diameter->value() * 0.5))
        return QStringLiteral("圆鼻刀圆角半径须大于零且小于刀具半径。");
    if (m_length->value() < radius)
        return QStringLiteral("单刀刃长不能小于球头或圆角高度。");
    if (m_overall->value() > 0 && m_overall->value() < m_length->value())
        return QStringLiteral("单刀总长 / 伸出不能小于刃长；未填写时可保持 0。");
    if ((m_holderDiameter->value() > 0) != (m_holderLength->value() > 0))
        return QStringLiteral("单刀刀柄直径与长度需同时填写，或同时保留 0。");
    if ((m_spindleDiameter->value() > 0) != (m_spindleLength->value() > 0))
        return QStringLiteral("单刀主轴包络直径与长度需同时填写，或同时保留 0。");
    return {};
}

toolpath::ToolpathToolContext DirectToolParameterSection::tool() const
{
    if (mode() != ToolSelectionMode::SingleTool || (!m_changed && m_initialMode == mode() && m_initial.valid)) return m_initial;
    auto result = m_initial;
    result.valid = true;
    if (result.toolId.empty()) result.toolId = (m_prefix + "-single-tool").toStdString();
    if (result.toolName.empty()) result.toolName = (m_prefix + " single tool").toStdString();
    result.toolType = m_type->currentData().toString().toStdString();
    result.toolNumber = m_number->value();
    result.diameter = m_diameter->value();
    result.cuttingEdgeLength = m_length->value();
    result.cornerRadius = result.toolType == "ball_end_mill" ? result.diameter * 0.5
        : result.toolType == "bull_nose" ? m_corner->value() : 0.0;
    result.tipAngle = result.toolType == "drill" ? m_tip->value() : 0;
    if (result.toolType != m_initial.toolType || result.diameter != m_initial.diameter ||
        result.tipAngle != m_initial.tipAngle)
        result.tipCompensation = 0;
    result.spindleSpeed = m_spindle->value();
    result.horizontalFeed = m_feed->value();
    result.verticalFeed = m_plunge->value();
    result.overallLength = m_overall->value();
    result.shankDiameter = m_shank->value();
    result.holderDiameter = m_holderDiameter->value();
    result.holderLength = m_holderLength->value();
    result.spindleEnvelopeDiameter = m_spindleDiameter->value();
    result.spindleEnvelopeLength = m_spindleLength->value();
    return result;
}
} // namespace oap::app

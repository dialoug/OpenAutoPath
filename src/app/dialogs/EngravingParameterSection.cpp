#include "app/dialogs/EngravingParameterSection.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QSpinBox>
#include <QWidget>

namespace oap::app {
namespace {
double positiveOr(double value, double fallback) { return value > 0 ? value : fallback; }
}

EngravingParameterSection::EngravingParameterSection(
    QFormLayout& form, QWidget& parent, const ToolpathParameterEditorState& initial)
    : m_initial(initial.engraving)
{
    const auto& params = m_initial.parameters;
    const auto& tool = m_initial.manualTool;
    auto* note = new QLabel(QStringLiteral(
        "直接从完整模型生成浮雕刀路，无需孔槽识别。默认六面分别生成固定刀轴刀路；"
        "也可单独选择任一面，只雕刻正面时选择 +Z。雕刻按项目顺序接在前序加工之后，可用项目 IPW 验证中间毛坯。"), &parent);
    note->setWordWrap(true);
    form.addRow(note);
    m_scope = new QComboBox(&parent);
    m_scope->setObjectName("engravingScope");
    m_scope->addItem(QStringLiteral("六面（默认）"), static_cast<int>(EngravingSetupScope::AllSixFaces));
    m_scope->addItem(QStringLiteral("仅 +Z"), static_cast<int>(EngravingSetupScope::PositiveZ));
    m_scope->addItem(QStringLiteral("仅 -X"), static_cast<int>(EngravingSetupScope::NegativeX));
    m_scope->addItem(QStringLiteral("仅 +X"), static_cast<int>(EngravingSetupScope::PositiveX));
    m_scope->addItem(QStringLiteral("仅 -Y"), static_cast<int>(EngravingSetupScope::NegativeY));
    m_scope->addItem(QStringLiteral("仅 +Y"), static_cast<int>(EngravingSetupScope::PositiveY));
    m_scope->addItem(QStringLiteral("仅 -Z"), static_cast<int>(EngravingSetupScope::NegativeZ));
    if (m_initial.scope == EngravingSetupScope::PreserveProjectFaces)
        m_scope->addItem(QStringLiteral("项目既有装夹方向"), static_cast<int>(EngravingSetupScope::PreserveProjectFaces));
    m_scope->setCurrentIndex(m_scope->findData(static_cast<int>(m_initial.scope)));
    form.addRow(QStringLiteral("雕刻方向"), m_scope);
    m_machiningMode = new QComboBox(&parent);
    m_machiningMode->setObjectName("engravingMachiningMode");
    m_machiningMode->addItem(QStringLiteral("仅精雕"), static_cast<int>(toolpath::EngravingMachiningMode::FinishOnly));
    m_machiningMode->addItem(QStringLiteral("从毛坯加工（分层开粗后精雕）"), static_cast<int>(toolpath::EngravingMachiningMode::StockRoughingAndFinishing));
    m_machiningMode->setCurrentIndex(m_machiningMode->findData(static_cast<int>(params.mode)));
    m_machiningMode->setToolTip(QStringLiteral(
        "从毛坯加工使用项目毛坯的保守包络范围，按最大切深分层开粗后精雕；"
        "不会按前序 IPW 残料精确裁剪刀路。仅精雕保持原有贴面刀路。"));
    form.addRow(QStringLiteral("雕刻加工模式"), m_machiningMode);
    m_mode = new QComboBox(&parent);
    m_mode->setObjectName("engravingToolMode");
    m_mode->addItem(QStringLiteral("雕刻刀库自动选刀"), static_cast<int>(EngravingToolMode::AutomaticLibrary));
    m_mode->addItem(QStringLiteral("指定单刀（直接填写）"), static_cast<int>(EngravingToolMode::SingleTool));
    m_mode->setCurrentIndex(m_mode->findData(static_cast<int>(m_initial.toolMode)));
    form.addRow(QStringLiteral("雕刻刀具来源"), m_mode);
    m_library = new QComboBox(&parent);
    m_library->setObjectName("engravingLibrary");
    m_library->addItem(QStringLiteral("跟随活动刀库"), QString());
    for (const auto& choice : initial.engravingLibraries)
        m_library->addItem(QString::fromStdString(choice.name), QString::fromStdString(choice.id));
    const QString selectedLibrary = QString::fromStdString(m_initial.libraryId);
    if (m_library->findData(selectedLibrary) < 0)
        m_library->addItem(QStringLiteral("不可用刀库：%1").arg(selectedLibrary), selectedLibrary);
    m_library->setCurrentIndex(m_library->findData(selectedLibrary));
    m_library->setToolTip(QStringLiteral("只影响雕刻，不更改孔、槽腔或外轮廓的活动刀库。"));
    form.addRow(QStringLiteral("雕刻专用刀库"), m_library);
    const auto number = [&](const char* name, const QString& label, double initialValue,
                            double minimum, double maximum, const QString& suffix = QStringLiteral(" mm")) {
        auto* field = new QDoubleSpinBox(&parent);
        field->setObjectName(QString::fromLatin1(name));
        field->setRange(minimum, maximum);
        field->setDecimals(3);
        field->setSingleStep(0.1);
        field->setSuffix(suffix);
        field->setValue(initialValue);
        form.addRow(label, field);
        return field;
    };
    m_maxDiameter = number("engravingMaxDiameter", QStringLiteral("自动选刀最大直径"), params.diameter, 0.01, 1000);
    m_maxDiameter->setToolTip(QStringLiteral("优先球头刀，其次圆鼻刀、平底刀；同类中选择满足行距和直径上限的较小刀具。"));
    m_cutter = new QComboBox(&parent);
    m_cutter->setObjectName("engravingCutterKind");
    m_cutter->addItem(QStringLiteral("球头刀"), "ball_end_mill");
    m_cutter->addItem(QStringLiteral("平底刀"), "end_mill");
    m_cutter->addItem(QStringLiteral("圆鼻刀"), "bull_nose");
    const int cutterIndex = m_cutter->findData(QString::fromStdString(tool.toolType));
    m_cutter->setCurrentIndex(cutterIndex >= 0 ? cutterIndex : 0);
    form.addRow(QStringLiteral("单刀类型"), m_cutter);
    m_number = new QSpinBox(&parent);
    m_number->setObjectName("engravingToolNumber");
    m_number->setRange(1, 9999);
    m_number->setValue(tool.toolNumber > 0 ? tool.toolNumber : 1);
    form.addRow(QStringLiteral("单刀刀号 T"), m_number);
    m_diameter = number("engravingDiameter", QStringLiteral("单刀直径"), positiveOr(tool.diameter, 2), 0.01, 1000);
    m_length = number("engravingLength", QStringLiteral("单刀刃长"), positiveOr(tool.cuttingEdgeLength, 30), 0.01, 10000);
    m_corner = number("engravingCornerRadius", QStringLiteral("单刀圆角半径"), positiveOr(tool.cornerRadius, 0.2), 0.001, 500);
    m_overall = number("engravingOverall", QStringLiteral("单刀总长 / 伸出"), tool.overallLength, 0, 10000);
    m_shank = number("engravingShank", QStringLiteral("单刀刀杆直径"), tool.shankDiameter, 0, 1000);
    m_holderDiameter = number("engravingHolderDiameter", QStringLiteral("单刀刀柄直径"), tool.holderDiameter, 0, 1000);
    m_holderLength = number("engravingHolderLength", QStringLiteral("单刀刀柄长度"), tool.holderLength, 0, 10000);
    m_spindle = number("engravingSpindle", QStringLiteral("单刀主轴转速"), positiveOr(tool.spindleSpeed, 8000), 1, 1000000, QStringLiteral(" rpm"));
    m_feed = number("engravingFeed", QStringLiteral("单刀切削进给"), positiveOr(tool.horizontalFeed, 600), 1, 1000000, QStringLiteral(" mm/min"));
    m_plunge = number("engravingPlunge", QStringLiteral("单刀进刀进给"), positiveOr(tool.verticalFeed, 150), 1, 1000000, QStringLiteral(" mm/min"));
    m_stepOver = number("engravingStepOver", QStringLiteral("雕刻行距"), params.stepOver, 0.001, 1000);
    m_sample = number("engravingSampleSpacing", QStringLiteral("沿行采样间距"), params.sampleSpacing, 0.001, 1000);
    m_mesh = number("engravingMeshDeflection", QStringLiteral("曲面离散偏差"), params.meshDeflection, 0.001, 100);
    m_allowance = number("engravingAllowance", QStringLiteral("沿刀轴余量"), params.allowance, 0, 1000);
    m_clearance = number("engravingClearance", QStringLiteral("模型 / 毛坯上方抬刀"), params.safeClearance, 0.001, 1000);
    m_maxStepDown = number("engravingMaxStepDown", QStringLiteral("分层最大切深"), params.maxStepDown, 0.001, 1000);
    m_maxStepDown->setToolTip(QStringLiteral("从项目毛坯顶面向下分层；每层切深不能超过所选刀具刃长。"));
    m_roughingAllowance = number("engravingRoughingAllowance", QStringLiteral("开粗保留余量"), params.roughingAllowance, 0, 1000);
    m_roughingAllowance->setToolTip(QStringLiteral("开粗在精雕路径上方沿刀轴保留的额外余量，不能大于分层最大切深。"));
    const auto refresh = [this, &form] {
        const bool single = m_mode->currentData().toInt() == static_cast<int>(EngravingToolMode::SingleTool);
        form.setRowVisible(m_library, !single);
        form.setRowVisible(m_maxDiameter, !single);
        for (QWidget* field : std::initializer_list<QWidget*>{m_cutter, m_number, m_diameter, m_length,
                 m_corner, m_overall, m_shank, m_holderDiameter, m_holderLength, m_spindle, m_feed, m_plunge})
            form.setRowVisible(field, single);
        form.setRowVisible(m_corner, single && m_cutter->currentData().toString() == "bull_nose");
        const bool roughing = m_machiningMode->currentData().toInt() ==
            static_cast<int>(toolpath::EngravingMachiningMode::StockRoughingAndFinishing);
        form.setRowVisible(m_maxStepDown, roughing);
        form.setRowVisible(m_roughingAllowance, roughing);
    };
    QObject::connect(m_mode, &QComboBox::currentIndexChanged, &parent, refresh);
    QObject::connect(m_cutter, &QComboBox::currentIndexChanged, &parent, refresh);
    QObject::connect(m_machiningMode, &QComboBox::currentIndexChanged, &parent, refresh);
    refresh();
}

QString EngravingParameterSection::validationError() const
{
    const bool single = m_mode->currentData().toInt() == static_cast<int>(EngravingToolMode::SingleTool);
    const double diameter = single ? m_diameter->value() : m_maxDiameter->value();
    if (m_stepOver->value() > diameter)
        return QStringLiteral("雕刻行距不能大于单刀直径或自动选刀最大直径。");
    if (single && m_cutter->currentData().toString() == "bull_nose" && m_corner->value() >= diameter * 0.5)
        return QStringLiteral("圆鼻刀圆角半径必须小于刀具半径。");
    if (m_machiningMode->currentData().toInt() == static_cast<int>(toolpath::EngravingMachiningMode::StockRoughingAndFinishing)) {
        if (m_roughingAllowance->value() > m_maxStepDown->value())
            return QStringLiteral("开粗保留余量不能大于分层最大切深。");
        if (single && m_maxStepDown->value() > m_length->value())
            return QStringLiteral("分层最大切深不能大于单刀刃长。");
    }
    return {};
}

EngravingSettings EngravingParameterSection::value() const
{
    auto result = m_initial;
    result.scope = static_cast<EngravingSetupScope>(m_scope->currentData().toInt());
    if (result.scope != EngravingSetupScope::PreserveProjectFaces) result.projectFaces.clear();
    result.toolMode = static_cast<EngravingToolMode>(m_mode->currentData().toInt());
    result.libraryId = m_library->currentData().toString().toStdString();
    result.parameters.diameter = m_maxDiameter->value();
    result.parameters.stepOver = m_stepOver->value();
    result.parameters.sampleSpacing = m_sample->value();
    result.parameters.meshDeflection = m_mesh->value();
    result.parameters.allowance = m_allowance->value();
    result.parameters.safeClearance = m_clearance->value();
    result.parameters.mode = static_cast<toolpath::EngravingMachiningMode>(m_machiningMode->currentData().toInt());
    result.parameters.maxStepDown = m_maxStepDown->value();
    result.parameters.roughingAllowance = m_roughingAllowance->value();
    if (result.toolMode != EngravingToolMode::SingleTool) return result;
    auto& tool = result.manualTool;
    tool.valid = true;
    tool.toolId = "engraving-single-tool";
    tool.toolName = "Engraving single tool";
    tool.toolType = m_cutter->currentData().toString().toStdString();
    tool.toolNumber = m_number->value();
    tool.diameter = m_diameter->value();
    tool.cuttingEdgeLength = m_length->value();
    tool.cornerRadius = tool.toolType == "ball_end_mill" ? tool.diameter * 0.5
        : tool.toolType == "bull_nose" ? m_corner->value() : 0;
    tool.overallLength = m_overall->value();
    tool.shankDiameter = m_shank->value();
    tool.holderDiameter = m_holderDiameter->value();
    tool.holderLength = m_holderLength->value();
    tool.spindleSpeed = m_spindle->value();
    tool.horizontalFeed = m_feed->value();
    tool.verticalFeed = m_plunge->value();
    return result;
}
} // namespace oap::app

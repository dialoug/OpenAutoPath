// StockSimulationDialogs 实现 — 参见 StockSimulationDialogs.h。
// 所有对话框只消费值快照并通过回调驱动外部(场景/仿真)状态。
#include "app/dialogs/StockSimulationDialogs.h"
#include <QCoreApplication>

#include <algorithm>
#include <utility>

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QTimer>

namespace oap::app {
namespace {

// 创建带 mm 后缀、0~10000 范围的双精度余量输入框。
QDoubleSpinBox* makeMarginSpin(
    QDialog& dialog,
    double value)
{
    auto* spin = new QDoubleSpinBox(&dialog);
    spin->setRange(0.0, 10000.0);
    spin->setDecimals(3);
    spin->setSingleStep(0.5);
    spin->setSuffix(QStringLiteral(" mm"));
    spin->setValue(value);
    return spin;
}

// 按毛坯来源联动启用/禁用输入:导入真实毛坯时启用文件行、禁用余量,
// 否则反之(源模式 data == 2 表示导入)。
void updateStockSourceInputs(
    const QComboBox& source,
    QWidget& stockFileRow,
    QDoubleSpinBox& marginX,
    QDoubleSpinBox& marginY,
    QDoubleSpinBox& marginZ)
{
    const bool imported = source.currentData().toInt() == 2;
    stockFileRow.setEnabled(imported);
    marginX.setEnabled(!imported);
    marginY.setEnabled(!imported);
    marginZ.setEnabled(!imported);
}

// 打开 STEP 文件选择对话框,返回用户选择的路径(取消返回空串)。
QString chooseStockFile(
    QWidget* parent,
    const QString& current,
    const QString& fallbackDirectory)
{
    QString start = current;
    if (start.isEmpty())
        start = fallbackDirectory;
    return QFileDialog::getOpenFileName(
        parent, QCoreApplication::translate("OpenAutoPath", "选择加工前毛坯 STEP"), start,
        QCoreApplication::translate("OpenAutoPath", "STEP 文件 (*.stp *.step);;所有文件 (*.*)"));
}

} // namespace

// 腔毛坯仿真设置编辑:布局毛坯来源/装夹面/余量/质量/三角化等控件,
// 确认后把控件值回写到状态副本并返回;取消返回 nullopt。
std::optional<PocketStockSimulationDialogState>
editPocketStockSimulationSettings(
    QWidget* parent,
    const PocketStockSimulationDialogState& initial,
    const QString& operationName,
    bool offerOpenGlPlayback)
{
    QDialog dialog(parent);
    dialog.setWindowTitle(QCoreApplication::translate("OpenAutoPath", "%1 IPW 仿真%2")
        .arg(operationName,
             offerOpenGlPlayback
                 ? QCoreApplication::translate("OpenAutoPath", " — FreeCAD CAM 仿真器")
                 : QString()));
    dialog.setMinimumWidth(570);
    auto* layout = new QFormLayout(&dialog);

    const QString playbackText = offerOpenGlPlayback
        ? QCoreApplication::translate("OpenAutoPath", "随后打开刀具动画，显示同一批刀路的运动及剩余毛坯变化。")
        : QCoreApplication::translate("OpenAutoPath", "结果通过 OCCT 主视口的 Multi-Dexel 派生网格和逐工序时间线呈现。");
    auto* description = new QLabel(
        QCoreApplication::translate("OpenAutoPath", "先用加工 Job 中的稳定%1工序执行统一 Multi-Dexel IPW，生成可复用的"
            "逐工序检查点并计算余量、过切；%2%3")
            .arg(operationName, playbackText, !initial.scopeDescription.isEmpty() ? initial.scopeDescription : initial.selectedSetup < 0
                ? QCoreApplication::translate("OpenAutoPath", "按项目全局工序顺序执行全部启用工序，翻面沿用同一材料状态。")
                : QCoreApplication::translate("OpenAutoPath", "存在多个装夹面时需选择一个 VIEW 分别仿真。")),
        &dialog);
    description->setWordWrap(true);
    layout->addRow(description);

    auto* source = new QComboBox(&dialog);
    source->addItem(QCoreApplication::translate("OpenAutoPath", "每个模型单元的独立包围盒"), 0);
    source->addItem(QCoreApplication::translate("OpenAutoPath", "整个模型的单一包围盒"), 1);
    source->addItem(QCoreApplication::translate("OpenAutoPath", "导入真实毛坯 STEP"), 2);
    source->setCurrentIndex(
        std::clamp(initial.source.sourceMode, 0, 2));
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "毛坯来源"), source);

    auto* setup = new QComboBox(&dialog);
    for (const PocketSimulationSetupChoice& choice : initial.setups) {
        setup->addItem(choice.label, choice.setupIndex);
    }
    const int selectedPosition = setup->findData(initial.selectedSetup);
    if (selectedPosition >= 0)
        setup->setCurrentIndex(selectedPosition);
    layout->addRow(initial.selectedSetup < 0 ? QCoreApplication::translate("OpenAutoPath", "仿真范围")
                                           : QCoreApplication::translate("OpenAutoPath", "仿真装夹面"), setup);

    auto* stockFileRow = new QWidget(&dialog);
    auto* stockFileLayout = new QHBoxLayout(stockFileRow);
    stockFileLayout->setContentsMargins(0, 0, 0, 0);
    auto* stockFile = new QLineEdit(
        initial.source.stepPath, stockFileRow);
    stockFile->setPlaceholderText(
        QCoreApplication::translate("OpenAutoPath", "请选择加工前毛坯 .stp/.step 文件"));
    auto* browseStock =
        new QPushButton(QCoreApplication::translate("OpenAutoPath", "浏览…"), stockFileRow);
    stockFileLayout->addWidget(stockFile, 1);
    stockFileLayout->addWidget(browseStock);
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "毛坯 STEP"), stockFileRow);

    auto* marginX = makeMarginSpin(dialog, initial.source.marginX);
    auto* marginY = makeMarginSpin(dialog, initial.source.marginY);
    auto* marginZ = makeMarginSpin(dialog, initial.source.marginZ);
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "X 方向单边余量"), marginX);
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "Y 方向单边余量"), marginY);
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "Z 方向单边余量"), marginZ);

    auto* quality = new QSpinBox(&dialog);
    quality->setRange(1, 10);
    quality->setValue(std::clamp(initial.displayQuality, 1, 10));
    quality->setToolTip(QCoreApplication::translate("OpenAutoPath", "同时控制 Multi-Dexel 材料 XY 分辨率、余量/过切派生网格"
        "与 FreeCAD 动画质量。1–3 为交互档，4–7 为平衡档，"
        "8–10 为详细档；越高，初始化时间和内存会按 XY 单元数增长。"));
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "IPW 仿真/显示质量"), quality);

    auto* triangulation = new QCheckBox(
        QCoreApplication::translate("OpenAutoPath", "为 Multi-Dexel 表面形式生成三角网格"),
        &dialog);
    triangulation->setChecked(initial.triangulationEnabled);
    triangulation->setToolTip(QCoreApplication::translate("OpenAutoPath", "关闭后仍执行 Multi-Dexel 减料、体积、余量和过切计算；"
        "Dexel 柱、点云、体素柱、剖面和 BRep 仍可用于 OCCT 时间线。"));
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "三角化"), triangulation);

    auto* displayCacheMiB = new QSpinBox(&dialog);
    displayCacheMiB->setRange(64, 2048);
    displayCacheMiB->setSingleStep(64);
    displayCacheMiB->setSuffix(QStringLiteral(" MiB"));
    displayCacheMiB->setValue(
        std::clamp(initial.displayCacheMiB, 64, 2048));
    displayCacheMiB->setToolTip(QCoreApplication::translate("OpenAutoPath", "限制可重建 IPW/余量显示网格的进程内缓存；达到上限后"
        "按最近最少使用顺序淘汰，不会删除材料检查点。"));
    displayCacheMiB->setEnabled(triangulation->isChecked());
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "显示缓存上限"), displayCacheMiB);

    const auto updateInputs = [=] {
        updateStockSourceInputs(
            *source, *stockFileRow, *marginX, *marginY, *marginZ);
    };
    QObject::connect(
        source, &QComboBox::currentIndexChanged,
        &dialog, [updateInputs](int) { updateInputs(); });
    QObject::connect(
        triangulation, &QCheckBox::toggled,
        displayCacheMiB, &QWidget::setEnabled);
    QObject::connect(
        browseStock, &QPushButton::clicked,
        &dialog, [&, stockFile] {
            const QString path = chooseStockFile(
                &dialog, stockFile->text(), initial.browseDirectory);
            if (!path.isEmpty())
                stockFile->setText(QDir::toNativeSeparators(path));
        });
    updateInputs();

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    buttons->button(QDialogButtonBox::Ok)->setText(
        QCoreApplication::translate("OpenAutoPath", "启动仿真"));
    QObject::connect(
        buttons, &QDialogButtonBox::accepted,
        &dialog, &QDialog::accept);
    QObject::connect(
        buttons, &QDialogButtonBox::rejected,
        &dialog, &QDialog::reject);
    layout->addRow(buttons);
    if (dialog.exec() != QDialog::Accepted)
        return std::nullopt;

    // 用户确认后把控件值回写到状态副本(其余字段保持 initial 不变)。
    PocketStockSimulationDialogState result = initial;
    result.source.sourceMode = source->currentData().toInt();
    result.source.marginX = marginX->value();
    result.source.marginY = marginY->value();
    result.source.marginZ = marginZ->value();
    result.source.stepPath = stockFile->text().trimmed();
    result.selectedSetup = setup->currentData().toInt();
    result.displayQuality = quality->value();
    result.triangulationEnabled = triangulation->isChecked();
    result.displayCacheMiB = displayCacheMiB->value();
    return result;
}

// 构造函数:以 initial 值快照初始化各控件,连接联动信号,
// 最后刷新一次模型(默认优先显示过切归因视图)。
HoleStockSimulationDialog::HoleStockSimulationDialog(
    QWidget* parent,
    StockSourceSettings initial,
    QString browseDirectory,
    HoleStockDialogCallbacks callbacks)
    : QDialog(parent),
      m_callbacks(std::move(callbacks)),
      m_browseDirectory(std::move(browseDirectory))
{
    setWindowTitle(QCoreApplication::translate("OpenAutoPath", "孔毛胚仿真"));
    setMinimumWidth(560);
    auto* layout = new QFormLayout(this);
    auto* description = new QLabel(
        QCoreApplication::translate("OpenAutoPath", "按当前孔操作顺序执行布尔减料。可用模型包围盒近似毛坯，"
            "也可导入与成品模型使用相同坐标系的实际毛坯 STEP；"
            "耗时操作在后台运行并可取消。完成后可按装夹面启动 "
            "GPU Multi-Dexel OpenGL 刀具动画。"),
        this);
    description->setWordWrap(true);
    layout->addRow(description);

    m_source = new QComboBox(this);
    m_source->addItem(QCoreApplication::translate("OpenAutoPath", "每个模型单元的独立包围盒"), 0);
    m_source->addItem(QCoreApplication::translate("OpenAutoPath", "整个模型的单一包围盒"), 1);
    m_source->addItem(QCoreApplication::translate("OpenAutoPath", "导入真实毛坯 STEP"), 2);
    m_source->setCurrentIndex(std::clamp(initial.sourceMode, 0, 2));
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "毛坯来源"), m_source);

    m_stockFileRow = new QWidget(this);
    auto* stockFileLayout = new QHBoxLayout(m_stockFileRow);
    stockFileLayout->setContentsMargins(0, 0, 0, 0);
    m_stockFile = new QLineEdit(initial.stepPath, m_stockFileRow);
    m_stockFile->setPlaceholderText(
        QCoreApplication::translate("OpenAutoPath", "请选择加工前毛坯 .stp/.step 文件"));
    auto* browseStock =
        new QPushButton(QCoreApplication::translate("OpenAutoPath", "浏览…"), m_stockFileRow);
    stockFileLayout->addWidget(m_stockFile, 1);
    stockFileLayout->addWidget(browseStock);
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "毛坯 STEP"), m_stockFileRow);

    m_marginX = makeMarginSpin(*this, initial.marginX);
    m_marginY = makeMarginSpin(*this, initial.marginY);
    m_marginZ = makeMarginSpin(*this, initial.marginZ);
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "X 方向单边余量"), m_marginX);
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "Y 方向单边余量"), m_marginY);
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "Z 方向单边余量"), m_marginZ);

    m_stepSlider = new QSlider(Qt::Horizontal, this);
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "工序快照"), m_stepSlider);
    m_resultView = new QComboBox(this);
    m_resultView->addItem(QCoreApplication::translate("OpenAutoPath", "毛坯工序快照"), 0);
    m_resultView->addItem(QCoreApplication::translate("OpenAutoPath", "初始缺料 / 坐标错位（红）"), 1);
    m_resultView->addItem(QCoreApplication::translate("OpenAutoPath", "剩余加工余量（橙）"), 2);
    m_resultView->addItem(QCoreApplication::translate("OpenAutoPath", "目标实体过切（紫红）"), 3);
    m_resultView->addItem(QCoreApplication::translate("OpenAutoPath", "逐工序过切归因（亮红）"), 4);
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "三维结果"), m_resultView);
    m_overcutOperation = new QComboBox(this);
    m_overcutOperation->setPlaceholderText(
        QCoreApplication::translate("OpenAutoPath", "没有检测到工序过切"));
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "过切工序"), m_overcutOperation);
    m_summary = new QLabel(this);
    m_summary->setWordWrap(true);
    m_summary->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addRow(m_summary);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Close, this);
    m_runButton = buttons->addButton(
        QCoreApplication::translate("OpenAutoPath", "执行仿真"), QDialogButtonBox::ActionRole);
    m_openGlButton = buttons->addButton(
        QCoreApplication::translate("OpenAutoPath", "OpenGL 刀具动画"),
        QDialogButtonBox::ActionRole);
    auto* clearButton = buttons->addButton(
        QCoreApplication::translate("OpenAutoPath", "清除显示"), QDialogButtonBox::ResetRole);
    layout->addRow(buttons);

    const auto updateInputs = [this] {
        updateStockSourceInputs(
            *m_source, *m_stockFileRow,
            *m_marginX, *m_marginY, *m_marginZ);
    };
    connect(m_source, &QComboBox::currentIndexChanged,
            this, [updateInputs](int) { updateInputs(); });
    connect(browseStock, &QPushButton::clicked, this, [this] {
        const QString path = chooseStockFile(
            this, m_stockFile->text(), m_browseDirectory);
        if (!path.isEmpty())
            m_stockFile->setText(QDir::toNativeSeparators(path));
    });
    connect(m_stepSlider, &QSlider::valueChanged,
            this, [this](int) { refreshSelection(); });
    connect(m_resultView, &QComboBox::currentIndexChanged,
            this, [this](int) { refreshSelection(); });
    connect(m_overcutOperation, &QComboBox::currentIndexChanged,
            this, [this](int) { refreshSelection(); });
    connect(m_runButton, &QPushButton::clicked,
            this, [this] { runSimulation(); });
    connect(m_openGlButton, &QPushButton::clicked,
            this, [this] {
                if (m_callbacks.openGl)
                    m_callbacks.openGl();
            });
    connect(clearButton, &QPushButton::clicked,
            this, [this] { clearSimulation(); });
    connect(buttons, &QDialogButtonBox::rejected,
            this, &QDialog::reject);
    updateInputs();
    refreshModel(true);
}

// 从当前控件状态收集毛坯来源设置。
StockSourceSettings HoleStockSimulationDialog::sourceSettings() const
{
    StockSourceSettings result;
    result.sourceMode = m_source->currentData().toInt();
    result.marginX = m_marginX->value();
    result.marginY = m_marginY->value();
    result.marginZ = m_marginZ->value();
    result.stepPath = m_stockFile->text().trimmed();
    return result;
}

// 从当前控件状态收集显示选择(工序快照索引/视图模式/过切归因)。
HoleStockDisplaySelection
HoleStockSimulationDialog::displaySelection() const
{
    HoleStockDisplaySelection result;
    result.stepIndex = m_stepSlider->value();
    result.viewMode = m_resultView->currentData().toInt();
    result.overcutAttributionIndex =
        m_overcutOperation->currentData().toInt();
    return result;
}

// 从 model() 回调重读仿真模型,重建滑块范围/过切选项并联动刷新;
// 屏蔽信号避免重建过程触发 selection 回调。
void HoleStockSimulationDialog::refreshModel(bool preferOvercut)
{
    const HoleStockDialogModel model = m_callbacks.model
        ? m_callbacks.model() : HoleStockDialogModel{};
    const QSignalBlocker stepBlocker(m_stepSlider);
    const QSignalBlocker viewBlocker(m_resultView);
    const QSignalBlocker overcutBlocker(m_overcutOperation);
    m_stepSlider->setRange(0, model.stepCount);
    m_stepSlider->setValue(model.stepCount);
    m_overcutOperation->clear();
    for (const HoleStockOvercutChoice& choice : model.overcutChoices) {
        m_overcutOperation->addItem(choice.label, choice.attributionIndex);
    }
    if (preferOvercut && !model.overcutChoices.empty())
        m_resultView->setCurrentIndex(m_resultView->findData(4));
    else if (!model.hasSimulation)
        m_resultView->setCurrentIndex(m_resultView->findData(0));
    m_stepSlider->setEnabled(
        model.hasSimulation && m_resultView->currentData().toInt() == 0);
    m_openGlButton->setEnabled(
        model.hasSimulation && static_cast<bool>(m_callbacks.openGl));
    m_overcutOperation->setEnabled(
        model.hasSimulation &&
        m_resultView->currentData().toInt() == 4 &&
        !model.overcutChoices.empty());
    refreshSelection();
}

// 按当前控件选择刷新摘要文本,并回调 present 更新 3D 显示。
void HoleStockSimulationDialog::refreshSelection()
{
    const HoleStockDialogModel model = m_callbacks.model
        ? m_callbacks.model() : HoleStockDialogModel{};
    m_stepSlider->setEnabled(
        model.hasSimulation && m_resultView->currentData().toInt() == 0);
    m_overcutOperation->setEnabled(
        model.hasSimulation &&
        m_resultView->currentData().toInt() == 4 &&
        m_overcutOperation->count() > 0);
    if (!model.hasSimulation) {
        m_summary->setText(QCoreApplication::translate("OpenAutoPath", "尚未执行仿真。"));
        return;
    }
    const HoleStockDisplaySelection selection = displaySelection();
    if (m_callbacks.describe)
        m_summary->setText(m_callbacks.describe(selection));
    if (m_callbacks.present)
        m_callbacks.present(selection);
}

// 调用 run 回调执行仿真(后台);失败弹窗提示,成功则刷新模型视图。
void HoleStockSimulationDialog::runSimulation()
{
    if (!m_callbacks.run)
        return;
    m_runButton->setEnabled(false);
    m_summary->setText(QCoreApplication::translate("OpenAutoPath", "正在后台执行孔加工减料、毛坯差异检查和过切归因……"));
    const HoleStockRunOutcome outcome =
        m_callbacks.run(sourceSettings());
    m_runButton->setEnabled(true);
    if (outcome.closeRequested) {
        reject();
        return;
    }
    if (!outcome.ok) {
        m_summary->setText(outcome.error);
        if (!outcome.error.isEmpty()) {
            QMessageBox::warning(
                this, QCoreApplication::translate("OpenAutoPath", "孔毛胚仿真"), outcome.error);
        }
        return;
    }
    refreshModel(true);
    if (m_callbacks.publishStatus && !outcome.status.isEmpty())
        m_callbacks.publishStatus(outcome.status);
}

// 调用 clear 回调清除仿真显示,并复位界面到未仿真状态。
void HoleStockSimulationDialog::clearSimulation()
{
    if (m_callbacks.clear)
        m_callbacks.clear();
    refreshModel(false);
    m_summary->setText(
        QCoreApplication::translate("OpenAutoPath", "仿真结果和三维毛坯显示已清除。"));
    if (m_callbacks.publishStatus) {
        m_callbacks.publishStatus(
            QCoreApplication::translate("OpenAutoPath", "孔毛胚仿真显示已清除"));
    }
}

// 时间线浏览:工序端点(首/末移动)直接回看不可变检查点,中间移动经
// 80ms 单发计时器节流重放;对话框结束时停止计时器并返回是否继续 OpenGL。
bool showIpwTimelineDialog(
    QWidget* parent,
    const std::vector<IpwTimelineOperation>& operations,
    const std::function<void(int, int)>& displayFrame,
    const std::function<void(const QString&)>& operationSelected,
    bool allowOpenGl)
{
    if (operations.empty())
        return false;
    QDialog dialog(parent);
    dialog.setWindowTitle(QCoreApplication::translate("OpenAutoPath", "统一 IPW 工序与移动时间线"));
    dialog.setMinimumWidth(660);
    auto* layout = new QFormLayout(&dialog);
    auto* description = new QLabel(
        QCoreApplication::translate("OpenAutoPath", "工序两端直接回看不可变 IPW 检查点；拖动到工序内部时，"
            "从父检查点用同一 Multi-Dexel 减料内核重放移动前缀。"
            "中间帧仅用于实时显示，不写入检查点仓库或显示缓存。") +
            (allowOpenGl
                ? QCoreApplication::translate("OpenAutoPath", "需要查看刀具运动时，可手动打开 OpenGL 刀具动画。")
                : QCoreApplication::translate("OpenAutoPath", "关闭时间线后保留统一 IPW 最终检查点。")),
        &dialog);
    description->setWordWrap(true);
    layout->addRow(description);
    auto* operation = new QComboBox(&dialog);
    for (const IpwTimelineOperation& item : operations) {
        operation->addItem(item.label, item.operationIndex);
        operation->setItemData(
            operation->count() - 1, item.moveCount, Qt::UserRole + 1);
        operation->setItemData(
            operation->count() - 1,
            item.formalOperationId, Qt::UserRole + 2);
        operation->setItemData(
            operation->count() - 1,
            item.replayAvailable, Qt::UserRole + 3);
        operation->setItemData(
            operation->count() - 1,
            item.status, Qt::UserRole + 4);
    }
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "当前工序"), operation);
    auto* moveSlider = new QSlider(Qt::Horizontal, &dialog);
    moveSlider->setTracking(true);
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "工序内移动"), moveSlider);
    auto* frameSummary = new QLabel(&dialog);
    frameSummary->setWordWrap(true);
    layout->addRow(QCoreApplication::translate("OpenAutoPath", "当前材料帧"), frameSummary);

    QTimer frameTimer(&dialog);
    frameTimer.setSingleShot(true);
    frameTimer.setInterval(80);
    const auto updateFrameSummary = [=] {
        const int operationIndex = operation->currentData().toInt();
        const int move = moveSlider->value();
        const int total = moveSlider->maximum();
        if (!operation->currentData(Qt::UserRole + 3).toBool()) {
            frameSummary->setText(QCoreApplication::translate("OpenAutoPath", "正式工序 %1｜%2；当前没有可回放的已验证 IPW 检查点。")
                .arg(operation->currentData(Qt::UserRole + 2).toString(),
                     operation->currentData(Qt::UserRole + 4).toString()));
            return;
        }
        QString frameKind = QCoreApplication::translate("OpenAutoPath", "由统一材料内核生成的临时帧");
        if (move == 0)
            frameKind = QCoreApplication::translate("OpenAutoPath", "父工序不可变检查点");
        else if (move == total)
            frameKind = QCoreApplication::translate("OpenAutoPath", "本工序不可变检查点");
        frameSummary->setText(QCoreApplication::translate("OpenAutoPath", "工序 %1，移动 %2/%3；%4。"
            "快速移动和退刀保留时间线位置，但不去除材料。")
            .arg(operationIndex + 1)
            .arg(move)
            .arg(total)
            .arg(frameKind));
    };
    QObject::connect(
        &frameTimer, &QTimer::timeout, &dialog,
        [=] {
            if (operation->currentData(Qt::UserRole + 3).toBool()) {
                displayFrame(
                    operation->currentData().toInt(), moveSlider->value());
            }
        });
    QObject::connect(
        moveSlider, &QSlider::valueChanged, &dialog,
        [&](int value) {
            updateFrameSummary();
            if (!operation->currentData(Qt::UserRole + 3).toBool())
                return;
            if (value == 0 || value == moveSlider->maximum()) {
                frameTimer.stop();
                displayFrame(operation->currentData().toInt(), value);
            } else {
                frameTimer.start();
            }
        });
    const auto selectOperation = [&](int) {
        frameTimer.stop();
        const int moveCount =
            operation->currentData(Qt::UserRole + 1).toInt();
        const bool replayAvailable =
            operation->currentData(Qt::UserRole + 3).toBool();
        const QSignalBlocker blocker(moveSlider);
        moveSlider->setRange(0, moveCount);
        moveSlider->setPageStep(std::max(1, moveCount / 100));
        moveSlider->setValue(moveCount);
        moveSlider->setEnabled(replayAvailable);
        updateFrameSummary();
        if (operationSelected) {
            operationSelected(
                operation->currentData(Qt::UserRole + 2).toString());
        }
        if (replayAvailable)
            displayFrame(operation->currentData().toInt(), moveCount);
    };
    QObject::connect(
        operation, &QComboBox::currentIndexChanged,
        &dialog, selectOperation);
    auto* buttons = new QDialogButtonBox(
        allowOpenGl
            ? QDialogButtonBox::StandardButtons(QDialogButtonBox::Ok | QDialogButtonBox::Cancel)
            : QDialogButtonBox::StandardButtons(QDialogButtonBox::Close), &dialog);
    if (allowOpenGl) {
        buttons->button(QDialogButtonBox::Ok)->setText(
            QCoreApplication::translate("OpenAutoPath", "打开 OpenGL 刀具动画"));
        buttons->button(QDialogButtonBox::Ok)->setAutoDefault(false);
        buttons->button(QDialogButtonBox::Ok)->setDefault(false);
        buttons->button(QDialogButtonBox::Cancel)->setText(
            QCoreApplication::translate("OpenAutoPath", "仅保留统一 IPW"));
        buttons->button(QDialogButtonBox::Cancel)->setDefault(true);
    } else {
        buttons->button(QDialogButtonBox::Close)->setText(
            QCoreApplication::translate("OpenAutoPath", "关闭并保留统一 IPW"));
    }
    QObject::connect(
        buttons, &QDialogButtonBox::accepted,
        &dialog, &QDialog::accept);
    QObject::connect(
        buttons, &QDialogButtonBox::rejected,
        &dialog, &QDialog::reject);
    layout->addRow(buttons);

    {
        const QSignalBlocker blocker(operation);
        operation->setCurrentIndex(operation->count() - 1);
    }
    selectOperation(operation->currentIndex());
    const bool continueToOpenGl = dialog.exec() == QDialog::Accepted;
    frameTimer.stop();
    return allowOpenGl && continueToOpenGl;
}

} // namespace oap::app

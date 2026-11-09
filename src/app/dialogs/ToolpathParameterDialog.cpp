// ToolpathParameterDialog 实现 — 参见 ToolpathParameterDialog.h。
// 单一函数按“孔 / 槽腔 / 外轮廓 / 雕刻 / 装夹 / 输出”六个标签页构建控件,确认前做一致性
// 校验,再把控件值回写到状态副本(initial 的工作拷贝)后返回。
#include "app/dialogs/ToolpathParameterDialog.h"
#include "app/dialogs/EngravingParameterSection.h"
#include "app/dialogs/DirectToolParameterSection.h"

#include <algorithm>
#include <array>
#include <cstddef>

#include <QAbstractScrollArea>
#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QLabel>
#include <QMessageBox>
#include <QObject>
#include <QScreen>
#include <QScrollArea>
#include <QSize>
#include <QSpinBox>
#include <QString>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

namespace oap::app {

// 全程编辑 initial 的工作副本,失败/取消时不影响调用方原状态。
std::optional<ToolpathParameterEditorState> editToolpathParameters(
    QWidget* parent,
    const ToolpathParameterEditorState& initial,
    ToolpathParameterEditorPage initialPage,
    ToolpathParameterEditorScope scope)
{
    ToolpathParameterEditorState edited = initial;
    auto& m_holeToolpathParams = edited.holeParams;
    auto& m_pocketToolpathParams = edited.pocketParams;
    auto& m_profileToolpathParams = edited.profileParams;
    auto& m_moldSetupWorkFrames = edited.moldSetupWorkFrames;
    auto& m_gcodeProgramOptions = edited.gcodeProgramOptions;

    QDialog dialog(parent);
    dialog.setWindowTitle(QStringLiteral("刀路参数"));
    dialog.setSizeGripEnabled(true);

    // 六个独立标签页，每页是一个带滚动条的 QFormLayout。
    auto *dialogLayout = new QVBoxLayout(&dialog);
    auto *settingsTabs = new QTabWidget(&dialog);
    auto makeSettingsPage = [&](const QString& title) {
        auto *scrollArea = new QScrollArea(settingsTabs);
        scrollArea->setWidgetResizable(true);
        scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        scrollArea->setFrameShape(QFrame::NoFrame);
        scrollArea->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
        auto *contents = new QWidget(scrollArea);
        auto *pageForm = new QFormLayout(contents);
        pageForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        scrollArea->setWidget(contents);
        settingsTabs->addTab(scrollArea, title);
        return pageForm;
    };
    QFormLayout *holeForm = makeSettingsPage(QStringLiteral("孔参数"));
    QFormLayout *pocketForm = makeSettingsPage(QStringLiteral("槽腔参数"));
    QFormLayout *profileForm = makeSettingsPage(QStringLiteral("外轮廓参数"));
    QFormLayout *engravingForm = makeSettingsPage(QStringLiteral("雕刻参数"));
    EngravingParameterSection engraving(*engravingForm, dialog, initial);
    QFormLayout *setupForm = makeSettingsPage(QStringLiteral("装夹设置"));
    QFormLayout *outputForm = makeSettingsPage(QStringLiteral("输出设置"));
    settingsTabs->setCurrentIndex(static_cast<int>(initialPage));
    if (scope == ToolpathParameterEditorScope::InitialPageOnly) {
        dialog.setWindowTitle(
            QStringLiteral("刀路工序参数"));
        for (int index = 0; index < settingsTabs->count(); ++index) {
            if (index != static_cast<int>(initialPage))
                settingsTabs->setTabVisible(index, false);
        }
    }
    QFormLayout *form = holeForm;
    dialogLayout->addWidget(settingsTabs, 1);

    QSize initialSize(720, 760);
    if (const QScreen *screen = dialog.screen()) {
        const QSize available = screen->availableGeometry().size();
        initialSize.setWidth(std::min(initialSize.width(),
                                      std::max(480, available.width() - 80)));
        initialSize.setHeight(std::min(initialSize.height(),
                                       std::max(420, available.height() - 80)));
    }
    dialog.resize(initialSize);

    DirectToolParameterSection holeTool(*holeForm, dialog, "hole", true,
        initial.holeToolSelectionMode, initial.holeSingleTool);
    DirectToolParameterSection pocketTool(*pocketForm, dialog, "pocket", false,
        initial.pocketToolSelectionMode, initial.pocketSingleTool);
    DirectToolParameterSection profileTool(*profileForm, dialog, "profile", false,
        initial.profileToolSelectionMode, initial.profileSingleTool);

    // 腔清腔参数:步距 / 清腔额外偏置 / 精修留量。
    auto *stepOver = new QDoubleSpinBox(&dialog);
    stepOver->setRange(1.0, 100.0);
    stepOver->setDecimals(1);
    stepOver->setSingleStep(5.0);
    stepOver->setSuffix(" %");
    stepOver->setValue(
        m_pocketToolpathParams.clearing.stepOverPercent);
    pocketForm->addRow(QStringLiteral("步距"), stepOver);

    auto *extraOffset = new QDoubleSpinBox(&dialog);
    extraOffset->setRange(0.0, 1000.0);
    extraOffset->setDecimals(3);
    extraOffset->setSingleStep(0.1);
    extraOffset->setSuffix(" mm");
    extraOffset->setValue(m_pocketToolpathParams.clearing.extraOffset);
    pocketForm->addRow(QStringLiteral("清腔额外偏置"), extraOffset);

    auto *finishAllowance = new QDoubleSpinBox(&dialog);
    finishAllowance->setRange(0.0, 1000.0);
    finishAllowance->setDecimals(3);
    finishAllowance->setSingleStep(0.1);
    finishAllowance->setSuffix(" mm");
    finishAllowance->setValue(m_pocketToolpathParams.depth.finishDepth);
    finishAllowance->setToolTip(
        QStringLiteral("清腔深度方向精修留量；0 表示直接加工到最终深度"));
    pocketForm->addRow(QStringLiteral("清腔精修留量"), finishAllowance);

    auto *holeAllowance = new QDoubleSpinBox(&dialog);
    holeAllowance->setRange(0.0, 1000.0);
    holeAllowance->setDecimals(3);
    holeAllowance->setSingleStep(0.1);
    holeAllowance->setSuffix(" mm");
    holeAllowance->setValue(m_holeToolpathParams.hole.allowance);
    holeAllowance->setToolTip(
        QStringLiteral("孔侧壁保留量；0 表示加工到识别孔径，增大后刀心半径向内收缩"));
    form->addRow(QStringLiteral("孔侧向余量"), holeAllowance);

    auto *smallHoleDiameterTolerance = new QDoubleSpinBox(&dialog);
    smallHoleDiameterTolerance->setRange(0.0, 10.0);
    smallHoleDiameterTolerance->setDecimals(3);
    smallHoleDiameterTolerance->setSingleStep(0.01);
    smallHoleDiameterTolerance->setSuffix(" mm");
    smallHoleDiameterTolerance->setValue(
        m_holeToolpathParams.hole.smallHoleDiameterTolerance);
    smallHoleDiameterTolerance->setToolTip(
        QStringLiteral("孔径与钻头直径的允许差值；在此范围内仍按中心钻削处理"));
    form->addRow(QStringLiteral("小孔直径容差"), smallHoleDiameterTolerance);

    auto *centerSpotDepth = new QDoubleSpinBox(&dialog);
    centerSpotDepth->setRange(0.01, 100.0);
    centerSpotDepth->setDecimals(3);
    centerSpotDepth->setSingleStep(0.1);
    centerSpotDepth->setSuffix(" mm");
    centerSpotDepth->setValue(m_holeToolpathParams.hole.centerSpotDepth);
    centerSpotDepth->setToolTip(
        QStringLiteral("自动刀库模式中心钻的定位深度；会自动限制为小于孔深"));
    form->addRow(QStringLiteral("中心定位深度"), centerSpotDepth);

    auto *throughHoleBreakthrough = new QDoubleSpinBox(&dialog);
    throughHoleBreakthrough->setRange(0.0, 100.0);
    throughHoleBreakthrough->setDecimals(3);
    throughHoleBreakthrough->setSingleStep(0.1);
    throughHoleBreakthrough->setSuffix(QStringLiteral(" mm"));
    throughHoleBreakthrough->setValue(
        m_holeToolpathParams.hole.throughHoleBreakthrough);
    throughHoleBreakthrough->setToolTip(
        QStringLiteral("完整刀径越过通孔出口面的附加距离；实际钻深还会加上钻尖长度"));
    form->addRow(QStringLiteral("通孔突破量"), throughHoleBreakthrough);

    auto *drillCycleMode = new QComboBox(&dialog);
    drillCycleMode->addItem(
        QStringLiteral("自动"),
        static_cast<int>(oap::toolpath::HoleDrillCycleMode::Auto));
    drillCycleMode->addItem(
        QStringLiteral("直接钻 G81/G82"),
        static_cast<int>(oap::toolpath::HoleDrillCycleMode::Direct));
    drillCycleMode->addItem(
        QStringLiteral("深孔啄钻 G83"),
        static_cast<int>(oap::toolpath::HoleDrillCycleMode::Peck));
    drillCycleMode->addItem(
        QStringLiteral("断屑钻 G73"),
        static_cast<int>(oap::toolpath::HoleDrillCycleMode::ChipBreak));
    const int drillCycleIndex = drillCycleMode->findData(
        static_cast<int>(m_holeToolpathParams.hole.drillCycleMode));
    if (drillCycleIndex >= 0)
        drillCycleMode->setCurrentIndex(drillCycleIndex);
    form->addRow(QStringLiteral("钻削循环"), drillCycleMode);

    auto *drillRetractPolicy = new QLabel(
        QStringLiteral("自动：仅在相邻同循环、R 平面共面且连接避障验证通过时使用 G99；其余使用 G98。"),
        &dialog);
    drillRetractPolicy->setWordWrap(true);
    form->addRow(QStringLiteral("循环返回高度"), drillRetractPolicy);

    auto *peckDepth = new QDoubleSpinBox(&dialog);
    peckDepth->setRange(0.0, 1000.0);
    peckDepth->setDecimals(3);
    peckDepth->setSingleStep(0.5);
    peckDepth->setSuffix(" mm");
    peckDepth->setSpecialValueText(QStringLiteral("使用循环默认"));
    peckDepth->setValue(m_holeToolpathParams.hole.peckDepth);
    peckDepth->setToolTip(
        QStringLiteral("循环 Q 默认值；0 时分别使用 G73/G83 默认 Q"));
    form->addRow(QStringLiteral("刀具 Q 覆盖"), peckDepth);

    auto *drillDwell = new QDoubleSpinBox(&dialog);
    drillDwell->setRange(0.0, 3600.0);
    drillDwell->setDecimals(3);
    drillDwell->setSingleStep(0.1);
    drillDwell->setSuffix(" s");
    drillDwell->setSpecialValueText(QStringLiteral("不停留"));
    drillDwell->setValue(m_holeToolpathParams.hole.dwellSeconds);
    form->addRow(QStringLiteral("孔底停留 P"), drillDwell);

    auto *drillTipAngle = new QDoubleSpinBox(&dialog);
    drillTipAngle->setRange(0.0, 179.0);
    drillTipAngle->setDecimals(1);
    drillTipAngle->setSingleStep(1.0);
    drillTipAngle->setSuffix(QStringLiteral("°"));
    drillTipAngle->setSpecialValueText(QStringLiteral("不补偿"));
    drillTipAngle->setValue(m_holeToolpathParams.hole.drillTipAngleDeg);
    drillTipAngle->setToolTip(
        QStringLiteral("直接补偿 h 未填写时的钻尖长度计算角；平面底、锥底与通孔中心钻削均使用"));
    form->addRow(QStringLiteral("钻尖角"), drillTipAngle);

    auto makeCycleRatio = [&](double value) {
        auto *input = new QDoubleSpinBox(&dialog);
        input->setRange(0.0, 100.0);
        input->setDecimals(3);
        input->setSingleStep(0.25);
        input->setValue(value);
        return input;
    };
    auto *g81MaxRatio = makeCycleRatio(
        m_holeToolpathParams.hole.g81MaxDepthDiameterRatio);
    auto *g73MaxRatio = makeCycleRatio(
        m_holeToolpathParams.hole.g73MaxDepthDiameterRatio);
    g81MaxRatio->setToolTip(
        QStringLiteral("孔的名义深度/刀径不超过该值时使用 G81；设为 0 可禁用 G81"));
    g73MaxRatio->setToolTip(
        QStringLiteral("介于 G81 阈值和该值之间时使用 G73；更深使用 G83"));
    form->addRow(QStringLiteral("G81 最大深径比"), g81MaxRatio);
    form->addRow(QStringLiteral("G73 最大深径比"), g73MaxRatio);

    auto makeCycleLength = [&](double value, double minimum) {
        auto *input = new QDoubleSpinBox(&dialog);
        input->setRange(minimum, 1000.0);
        input->setDecimals(3);
        input->setSingleStep(0.5);
        input->setSuffix(QStringLiteral(" mm"));
        input->setValue(value);
        return input;
    };
    auto *g73PeckDepth = makeCycleLength(
        m_holeToolpathParams.hole.g73PeckDepth, 0.001);
    auto *g83PeckDepth = makeCycleLength(
        m_holeToolpathParams.hole.g83PeckDepth, 0.001);
    auto *g73Retract = makeCycleLength(
        m_holeToolpathParams.hole.g73ChipBreakRetract, 0.001);
    auto *g83Reentry = makeCycleLength(
        m_holeToolpathParams.hole.g83ReentryClearance, 0.0);
    g73PeckDepth->setToolTip(QStringLiteral("G73 默认每次钻深 Q"));
    g83PeckDepth->setToolTip(QStringLiteral("G83 默认每次钻深 Q"));
    g73Retract->setToolTip(QStringLiteral("G73 展开预览时的断屑小退刀量"));
    g83Reentry->setToolTip(QStringLiteral("G83 返回孔内时，在上次深度前保留的重入间隙"));
    form->addRow(QStringLiteral("G73 钻深 Q"), g73PeckDepth);
    form->addRow(QStringLiteral("G83 钻深 Q"), g83PeckDepth);
    form->addRow(QStringLiteral("G73 断屑退刀"), g73Retract);
    form->addRow(QStringLiteral("G83 重入间隙"), g83Reentry);

    auto *collisionEnvelopeMode = new QComboBox(&dialog);
    collisionEnvelopeMode->addItem(
        QStringLiteral("仅刀具半径（兼容模式）"),
        static_cast<int>(oap::toolpath::HoleCollisionEnvelopeMode::CutterOnly));
    collisionEnvelopeMode->addItem(
        QStringLiteral("切削部 + 刀杆 + 刀柄/夹头"),
        static_cast<int>(oap::toolpath::HoleCollisionEnvelopeMode::ToolAssembly));
    collisionEnvelopeMode->addItem(
        QStringLiteral("固定三轴实体连续扫掠（推荐）"),
        static_cast<int>(
            oap::toolpath::HoleCollisionEnvelopeMode::ToolAssemblySolidSweep));
    const int envelopeModeIndex = collisionEnvelopeMode->findData(
        static_cast<int>(m_holeToolpathParams.hole.collisionEnvelopeMode));
    if (envelopeModeIndex >= 0)
        collisionEnvelopeMode->setCurrentIndex(envelopeModeIndex);
    collisionEnvelopeMode->setToolTip(
        QStringLiteral("实体扫掠会检查孔间连接、内部直线移动以及 G2/G3 圆弧/螺旋切削；未知装配段会警告并从包络中省略"));
    form->addRow(QStringLiteral("孔刀路碰撞包络"), collisionEnvelopeMode);

    auto *collisionSampleStep = new QDoubleSpinBox(&dialog);
    collisionSampleStep->setRange(0.1, 50.0);
    collisionSampleStep->setDecimals(2);
    collisionSampleStep->setSingleStep(0.5);
    collisionSampleStep->setSuffix(" mm");
    collisionSampleStep->setValue(
        m_holeToolpathParams.hole.collisionEnvelopeSampleStep);
    collisionSampleStep->setToolTip(
        QStringLiteral("越小越精细但碰撞计算越慢；离散包络会保守扩径以避免漏检"));
    form->addRow(QStringLiteral("刀轴包络采样"), collisionSampleStep);

    auto *mediumHoleRatio = new QDoubleSpinBox(&dialog);
    mediumHoleRatio->setRange(0.01, 1.0);
    mediumHoleRatio->setDecimals(3);
    mediumHoleRatio->setSingleStep(0.05);
    mediumHoleRatio->setValue(
        m_holeToolpathParams.hole.mediumHoleRadiusToolDiameterRatio);
    mediumHoleRatio->setToolTip(
        QStringLiteral("单刀和自动刀库共用：孔半径不大于该系数×刀具直径时采用中孔策略；最大为 1"));
    form->addRow(QStringLiteral("中孔半径/刀径比"), mediumHoleRatio);

    auto *holeStartCorner = new QComboBox(&dialog);
    const double moldLength = edited.moldLength;
    const double moldWidth = edited.moldWidth;
    auto cornerLabel = [](const QString& name, double x, double y) {
        return QStringLiteral("%1 (X=%2, Y=%3 mm)")
            .arg(name).arg(x, 0, 'f', 1).arg(y, 0, 'f', 1);
    };
    holeStartCorner->addItem(
        cornerLabel(QStringLiteral("左下"), 0.0, 0.0),
        static_cast<int>(oap::toolpath::HoleStartCorner::LowerLeft));
    holeStartCorner->addItem(
        cornerLabel(QStringLiteral("右下"), moldLength, 0.0),
        static_cast<int>(oap::toolpath::HoleStartCorner::LowerRight));
    holeStartCorner->addItem(
        cornerLabel(QStringLiteral("左上"), 0.0, moldWidth),
        static_cast<int>(oap::toolpath::HoleStartCorner::UpperLeft));
    holeStartCorner->addItem(
        cornerLabel(QStringLiteral("右上"), moldLength, moldWidth),
        static_cast<int>(oap::toolpath::HoleStartCorner::UpperRight));
    const int cornerIndex = holeStartCorner->findData(
        static_cast<int>(m_holeToolpathParams.hole.startCorner));
    if (cornerIndex >= 0)
        holeStartCorner->setCurrentIndex(cornerIndex);
    holeStartCorner->setToolTip(
        QStringLiteral("选择模具局部 XY 平面中孔加工访问顺序的起始角"));
    holeForm->addRow(QStringLiteral("孔加工起始角"), holeStartCorner);

    auto *clearingPattern = new QComboBox(&dialog);
    clearingPattern->addItem(QStringLiteral("回字形"),
                             static_cast<int>(oap::toolpath::ClearingPattern::Offset));
    clearingPattern->addItem(QStringLiteral("Z字形"),
                             static_cast<int>(oap::toolpath::ClearingPattern::ZigZag));
    clearingPattern->addItem(QStringLiteral("螺旋形"),
                             static_cast<int>(oap::toolpath::ClearingPattern::Spiral));
    clearingPattern->addItem(
        QStringLiteral("自适应清腔（FreePath）"),
        static_cast<int>(oap::toolpath::ClearingPattern::FreePath));
    const int patternIndex =
        clearingPattern->findData(static_cast<int>(
            m_pocketToolpathParams.clearing.pattern));
    if (patternIndex >= 0)
        clearingPattern->setCurrentIndex(patternIndex);
    pocketForm->addRow(QStringLiteral("清腔方式"), clearingPattern);

    auto *expansionDirection = new QComboBox(&dialog);
    expansionDirection->addItem(
        QStringLiteral("外圈向中心"),
        static_cast<int>(
            oap::toolpath::PocketClearingDirection::OutsideToInside));
    expansionDirection->addItem(
        QStringLiteral("中心向外圈"),
        static_cast<int>(
            oap::toolpath::PocketClearingDirection::InsideToOutside));
    const int expansionDirectionIndex =
        expansionDirection->findData(static_cast<int>(
            m_pocketToolpathParams.clearing.expansionDirection));
    if (expansionDirectionIndex >= 0)
        expansionDirection->setCurrentIndex(expansionDirectionIndex);
    pocketForm->addRow(QStringLiteral("清腔扩展方向"),
                       expansionDirection);

    const auto updateExpansionDirectionControl = [=](int) {
        const auto pattern =
            static_cast<oap::toolpath::ClearingPattern>(
                clearingPattern->currentData().toInt());
        const bool adaptiveFreePath =
            pattern == oap::toolpath::ClearingPattern::FreePath;
        if (adaptiveFreePath) {
            const int centerOutIndex = expansionDirection->findData(
                static_cast<int>(
                    oap::toolpath::PocketClearingDirection::InsideToOutside));
            if (centerOutIndex >= 0)
                expansionDirection->setCurrentIndex(centerOutIndex);
        }
        expansionDirection->setEnabled(
            pattern != oap::toolpath::ClearingPattern::ZigZag &&
            !adaptiveFreePath);
        expansionDirection->setToolTip(
            adaptiveFreePath
                ? QStringLiteral(
                      "自适应清腔严格保持 pyocc-exp 的中心向外规划顺序")
                : QStringLiteral(
                      "独立于顺铣/逆铣；适用于回字形和螺旋形，Z字形不使用此参数"));
    };
    QObject::connect(
        clearingPattern,
        qOverload<int>(&QComboBox::currentIndexChanged),
        &dialog,
        updateExpansionDirectionControl);
    updateExpansionDirectionControl(clearingPattern->currentIndex());

    auto *cutMode = new QComboBox(&dialog);
    cutMode->addItem(QStringLiteral("顺铣"), static_cast<int>(oap::toolpath::CutMode::Climb));
    cutMode->addItem(QStringLiteral("逆铣"), static_cast<int>(oap::toolpath::CutMode::Conventional));
    const int cutModeIndex =
        cutMode->findData(static_cast<int>(
            m_pocketToolpathParams.clearing.cutMode));
    if (cutModeIndex >= 0)
        cutMode->setCurrentIndex(cutModeIndex);
    pocketForm->addRow(QStringLiteral("切削方式"), cutMode);

    auto *entryMode = new QComboBox(&dialog);
    entryMode->addItem(
        QStringLiteral("垂直下刀"),
        static_cast<int>(
            oap::toolpath::PocketEntryMode::VerticalPlunge));
    entryMode->addItem(
        QStringLiteral("沿轮廓斜坡（兼容）"),
        static_cast<int>(
            oap::toolpath::PocketEntryMode::RampOnShape));
    entryMode->addItem(
        QStringLiteral("螺旋斜坡"),
        static_cast<int>(
            oap::toolpath::PocketEntryMode::HelicalRamp));
    entryMode->addItem(
        QStringLiteral("直线斜坡（NX Linear）"),
        static_cast<int>(
            oap::toolpath::PocketEntryMode::LinearRamp));
    entryMode->addItem(
        QStringLiteral("往复斜坡（NX Zigzag）"),
        static_cast<int>(
            oap::toolpath::PocketEntryMode::ZigZagRamp));
    const int entryModeIndex = entryMode->findData(
        static_cast<int>(m_pocketToolpathParams.clearing.entryMode));
    if (entryModeIndex >= 0)
        entryMode->setCurrentIndex(entryModeIndex);
    entryMode->setToolTip(
        QStringLiteral("斜坡路径无法满足边界、岛或长度约束时会安全回退"));
    pocketForm->addRow(QStringLiteral("槽腔下刀方式"), entryMode);

    auto *rampStartReference = new QComboBox(&dialog);
    rampStartReference->addItem(
        QStringLiteral("上一切削层"),
        static_cast<int>(
            oap::toolpath::PocketRampStartReference::PreviousLevel));
    rampStartReference->addItem(
        QStringLiteral("槽腔开口面"),
        static_cast<int>(
            oap::toolpath::PocketRampStartReference::OpeningPlane));
    rampStartReference->addItem(
        QStringLiteral("安全面"),
        static_cast<int>(
            oap::toolpath::PocketRampStartReference::SafePlane));
    const int rampReferenceIndex = rampStartReference->findData(
        static_cast<int>(
            m_pocketToolpathParams.clearing.rampStartReference));
    if (rampReferenceIndex >= 0)
        rampStartReference->setCurrentIndex(rampReferenceIndex);
    rampStartReference->setToolTip(
        QStringLiteral("定义斜坡段开始下沉的高度基准；上一切削层可缩短深腔逐层进刀"));
    pocketForm->addRow(QStringLiteral("斜坡高度基准"), rampStartReference);

    auto *rampAngle = new QDoubleSpinBox(&dialog);
    rampAngle->setRange(0.1, 45.0);
    rampAngle->setDecimals(2);
    rampAngle->setSingleStep(0.5);
    rampAngle->setSuffix(QStringLiteral("°"));
    rampAngle->setValue(m_pocketToolpathParams.clearing.rampAngleDeg);
    rampAngle->setToolTip(
        QStringLiteral("相对局部水平面的最大下刀角度；越小所需路径越长"));
    pocketForm->addRow(QStringLiteral("最大斜坡角"), rampAngle);

    auto *maximumRampLength = new QDoubleSpinBox(&dialog);
    maximumRampLength->setRange(0.0, 100000.0);
    maximumRampLength->setDecimals(3);
    maximumRampLength->setSingleStep(5.0);
    maximumRampLength->setSuffix(QStringLiteral(" mm"));
    maximumRampLength->setSpecialValueText(QStringLiteral("不限"));
    maximumRampLength->setValue(
        m_pocketToolpathParams.clearing.maximumRampLength);
    maximumRampLength->setToolTip(
        QStringLiteral("单次入刀允许的最大刀心路径长度；0 表示不限"));
    pocketForm->addRow(QStringLiteral("最大斜坡长度"),
                       maximumRampLength);


    auto *helixDiameter = new QDoubleSpinBox(&dialog);
    helixDiameter->setRange(0.0, 10000.0);
    helixDiameter->setDecimals(3);
    helixDiameter->setSingleStep(0.5);
    helixDiameter->setSuffix(QStringLiteral(" mm"));
    helixDiameter->setSpecialValueText(
        QStringLiteral("自动（刀具直径）"));
    helixDiameter->setValue(
        m_pocketToolpathParams.clearing.helixDiameter);
    helixDiameter->setToolTip(
        QStringLiteral("刀心螺旋轨迹直径，不含刀具本身直径"));
    pocketForm->addRow(QStringLiteral("刀心螺旋直径"), helixDiameter);

    auto *zigzagRampWidth = new QDoubleSpinBox(&dialog);
    zigzagRampWidth->setRange(0.0, 10000.0);
    zigzagRampWidth->setDecimals(3);
    zigzagRampWidth->setSingleStep(1.0);
    zigzagRampWidth->setSuffix(QStringLiteral(" mm"));
    zigzagRampWidth->setSpecialValueText(QStringLiteral("自动（3×刀径）"));
    zigzagRampWidth->setValue(
        m_pocketToolpathParams.clearing.zigzagRampWidth);
    zigzagRampWidth->setToolTip(
        QStringLiteral("往复斜坡的单程宽度；空间不足时将搜索其他方向并安全回退"));
    pocketForm->addRow(QStringLiteral("往复斜坡宽度"), zigzagRampWidth);

    auto *rampFeed = new QDoubleSpinBox(&dialog);
    rampFeed->setRange(0.0, 1000000.0);
    rampFeed->setDecimals(1);
    rampFeed->setSingleStep(50.0);
    rampFeed->setSuffix(QStringLiteral(" mm/min"));
    rampFeed->setSpecialValueText(QStringLiteral("自动（垂直进给）"));
    rampFeed->setValue(m_pocketToolpathParams.clearing.rampFeed);
    pocketForm->addRow(QStringLiteral("斜坡进给"), rampFeed);

    const auto updateEntryControls = [=](int) {
        const auto mode = static_cast<oap::toolpath::PocketEntryMode>(
            entryMode->currentData().toInt());
        const bool ramp =
            mode != oap::toolpath::PocketEntryMode::VerticalPlunge;
        rampAngle->setEnabled(ramp);
        maximumRampLength->setEnabled(ramp);
        rampStartReference->setEnabled(ramp);
        rampFeed->setEnabled(ramp);
        helixDiameter->setEnabled(
            mode == oap::toolpath::PocketEntryMode::HelicalRamp);
        zigzagRampWidth->setEnabled(
            mode == oap::toolpath::PocketEntryMode::ZigZagRamp);
    };
    QObject::connect(
        entryMode, qOverload<int>(&QComboBox::currentIndexChanged),
        &dialog, updateEntryControls);
    updateEntryControls(entryMode->currentIndex());

    auto *precision = new QSpinBox(&dialog);
    precision->setRange(0, 8);
    precision->setValue(m_pocketToolpathParams.clearing.precision);
    pocketForm->addRow(QStringLiteral("Clipper 精度"), precision);

    auto *arcTolerance = new QDoubleSpinBox(&dialog);
    arcTolerance->setRange(0.0, 100.0);
    arcTolerance->setDecimals(4);
    arcTolerance->setSingleStep(0.01);
    arcTolerance->setSuffix(" mm");
    arcTolerance->setSpecialValueText(QStringLiteral("自动"));
    arcTolerance->setValue(m_pocketToolpathParams.clearing.arcTolerance);
    pocketForm->addRow(QStringLiteral("圆弧公差"), arcTolerance);

    const auto& adaptiveValues =
        m_pocketToolpathParams.clearing.adaptive;
    auto *adaptiveTarget = new QDoubleSpinBox(&dialog);
    adaptiveTarget->setRange(0.001, 1.0);
    adaptiveTarget->setDecimals(3);
    adaptiveTarget->setSingleStep(0.01);
    adaptiveTarget->setValue(adaptiveValues.targetEngagement);
    pocketForm->addRow(QStringLiteral("自适应目标啮合比例"), adaptiveTarget);

    auto *adaptiveTargetAngle = new QDoubleSpinBox(&dialog);
    adaptiveTargetAngle->setRange(1.0, 360.0);
    adaptiveTargetAngle->setSuffix(QStringLiteral("°"));
    adaptiveTargetAngle->setValue(adaptiveValues.targetEngagementAngleDeg);
    pocketForm->addRow(QStringLiteral("自适应目标包角"), adaptiveTargetAngle);

    auto *adaptiveMaximumAngle = new QDoubleSpinBox(&dialog);
    adaptiveMaximumAngle->setRange(1.0, 360.0);
    adaptiveMaximumAngle->setSuffix(QStringLiteral("°"));
    adaptiveMaximumAngle->setValue(adaptiveValues.maximumEngagementAngleDeg);
    pocketForm->addRow(QStringLiteral("自适应最大包角"), adaptiveMaximumAngle);

    auto *adaptiveAngleSamples = new QSpinBox(&dialog);
    adaptiveAngleSamples->setRange(8, 720);
    adaptiveAngleSamples->setValue(adaptiveValues.engagementAngleSamples);
    pocketForm->addRow(QStringLiteral("包角采样数"), adaptiveAngleSamples);

    auto *adaptiveTurnRadius = new QDoubleSpinBox(&dialog);
    adaptiveTurnRadius->setRange(0.05, 10.0);
    adaptiveTurnRadius->setDecimals(2);
    adaptiveTurnRadius->setSingleStep(0.05);
    adaptiveTurnRadius->setValue(adaptiveValues.minimumTurnRadiusFactor);
    adaptiveTurnRadius->setToolTip(QStringLiteral("最小转弯半径相对刀具直径的倍数"));
    pocketForm->addRow(QStringLiteral("最小转弯半径系数"), adaptiveTurnRadius);

    auto *adaptiveRemainingTolerance = new QDoubleSpinBox(&dialog);
    adaptiveRemainingTolerance->setRange(0.0, 0.9999);
    adaptiveRemainingTolerance->setDecimals(4);
    adaptiveRemainingTolerance->setSingleStep(0.005);
    adaptiveRemainingTolerance->setValue(adaptiveValues.remainingTolerance);
    pocketForm->addRow(QStringLiteral("允许残料比例"), adaptiveRemainingTolerance);

    auto *adaptiveCellSize = new QDoubleSpinBox(&dialog);
    adaptiveCellSize->setRange(0.0, 100.0);
    adaptiveCellSize->setDecimals(3);
    adaptiveCellSize->setSuffix(QStringLiteral(" mm"));
    adaptiveCellSize->setSpecialValueText(QStringLiteral("自动"));
    adaptiveCellSize->setValue(adaptiveValues.materialCellSize);
    pocketForm->addRow(QStringLiteral("残料栅格尺寸"), adaptiveCellSize);

    auto *adaptiveMaximumPasses = new QSpinBox(&dialog);
    adaptiveMaximumPasses->setRange(1, 500);
    adaptiveMaximumPasses->setValue(adaptiveValues.maximumAdaptivePasses);
    pocketForm->addRow(QStringLiteral("最大自适应 Pass"), adaptiveMaximumPasses);

    auto *adaptiveLookahead = new QSpinBox(&dialog);
    adaptiveLookahead->setRange(1, 4);
    adaptiveLookahead->setValue(adaptiveValues.lookaheadDepth);
    adaptiveLookahead->setToolTip(QStringLiteral("1 表示关闭多步前瞻"));
    pocketForm->addRow(QStringLiteral("前瞻深度"), adaptiveLookahead);

    auto *adaptiveBeamWidth = new QSpinBox(&dialog);
    adaptiveBeamWidth->setRange(1, 16);
    adaptiveBeamWidth->setValue(adaptiveValues.beamWidth);
    pocketForm->addRow(QStringLiteral("Beam 宽度"), adaptiveBeamWidth);

    const auto updateAdaptiveControls = [=](int) {
        const bool enabled =
            static_cast<oap::toolpath::ClearingPattern>(
                clearingPattern->currentData().toInt()) ==
            oap::toolpath::ClearingPattern::FreePath;
        adaptiveTarget->setEnabled(enabled);
        adaptiveTargetAngle->setEnabled(enabled);
        adaptiveMaximumAngle->setEnabled(enabled);
        adaptiveAngleSamples->setEnabled(enabled);
        adaptiveTurnRadius->setEnabled(enabled);
        adaptiveRemainingTolerance->setEnabled(enabled);
        adaptiveCellSize->setEnabled(enabled);
        adaptiveMaximumPasses->setEnabled(enabled);
        adaptiveLookahead->setEnabled(enabled);
        adaptiveBeamWidth->setEnabled(enabled);
    };
    QObject::connect(
        clearingPattern,
        qOverload<int>(&QComboBox::currentIndexChanged),
        &dialog,
        updateAdaptiveControls);
    updateAdaptiveControls(clearingPattern->currentIndex());

    auto *profileMachiningMode = new QComboBox(&dialog);
    profileMachiningMode->addItem(
        QStringLiteral("外部区域清料 + 轮廓精加工"),
        static_cast<int>(oap::toolpath::ProfileMachiningMode::ExternalClearing));
    profileMachiningMode->addItem(
        QStringLiteral("仅轮廓（兼容模式）"),
        static_cast<int>(oap::toolpath::ProfileMachiningMode::ContourOnly));
    profileMachiningMode->setCurrentIndex(
        profileMachiningMode->findData(static_cast<int>(
            m_profileToolpathParams.profile.machiningMode)));
    profileForm->addRow(QStringLiteral("加工策略"), profileMachiningMode);

    auto *profileClearingPattern = new QComboBox(&dialog);
    profileClearingPattern->addItem(
        QStringLiteral("回字形"),
        static_cast<int>(oap::toolpath::ClearingPattern::Offset));
    profileClearingPattern->addItem(
        QStringLiteral("Z字形"),
        static_cast<int>(oap::toolpath::ClearingPattern::ZigZag));
    profileClearingPattern->addItem(
        QStringLiteral("螺旋形（不可用时回退回字形）"),
        static_cast<int>(oap::toolpath::ClearingPattern::Spiral));
    profileClearingPattern->addItem(
        QStringLiteral("自适应清腔（FreePath）"),
        static_cast<int>(oap::toolpath::ClearingPattern::FreePath));
    profileClearingPattern->setCurrentIndex(
        profileClearingPattern->findData(static_cast<int>(
            m_profileToolpathParams.clearing.pattern)));
    profileForm->addRow(QStringLiteral("清料方式"), profileClearingPattern);

    auto *profileZigzagAngle = new QDoubleSpinBox(&dialog);
    profileZigzagAngle->setRange(-360.0, 360.0);
    profileZigzagAngle->setDecimals(1);
    profileZigzagAngle->setSingleStep(5.0);
    profileZigzagAngle->setSuffix(QStringLiteral("°"));
    profileZigzagAngle->setValue(
        m_profileToolpathParams.clearing.zigzagAngle);
    profileForm->addRow(QStringLiteral("Z字扫描角度"), profileZigzagAngle);

    auto *profileCutMode = new QComboBox(&dialog);
    profileCutMode->addItem(
        QStringLiteral("顺铣"),
        static_cast<int>(oap::toolpath::CutMode::Climb));
    profileCutMode->addItem(
        QStringLiteral("逆铣"),
        static_cast<int>(oap::toolpath::CutMode::Conventional));
    const int profileCutModeIndex = profileCutMode->findData(
        static_cast<int>(m_profileToolpathParams.profile.cutMode));
    if (profileCutModeIndex >= 0)
        profileCutMode->setCurrentIndex(profileCutModeIndex);
    profileForm->addRow(QStringLiteral("切削方式"), profileCutMode);

    auto *profileSideAllowance = new QDoubleSpinBox(&dialog);
    profileSideAllowance->setRange(0.0, 1000.0);
    profileSideAllowance->setDecimals(3);
    profileSideAllowance->setSingleStep(0.1);
    profileSideAllowance->setSuffix(QStringLiteral(" mm"));
    profileSideAllowance->setValue(
        m_profileToolpathParams.profile.sideAllowance);
    profileSideAllowance->setToolTip(
        QStringLiteral("最终侧壁保留材料；刀具中心始终位于零件外侧"));
    profileForm->addRow(QStringLiteral("侧壁余量"), profileSideAllowance);

    auto *profileRoughingAllowance = new QDoubleSpinBox(&dialog);
    profileRoughingAllowance->setRange(0.0, 1000.0);
    profileRoughingAllowance->setDecimals(3);
    profileRoughingAllowance->setSingleStep(0.1);
    profileRoughingAllowance->setSuffix(QStringLiteral(" mm"));
    profileRoughingAllowance->setValue(
        m_profileToolpathParams.profile.roughingAllowance);
    profileForm->addRow(QStringLiteral("粗加工侧壁余量"),
                        profileRoughingAllowance);

    auto *profileFinishPass = new QCheckBox(
        QStringLiteral("清料后走最终外轮廓"), &dialog);
    profileFinishPass->setChecked(
        m_profileToolpathParams.profile.finishPass);
    profileForm->addRow(QStringLiteral("轮廓精加工"), profileFinishPass);

    auto *profileRadialPassCount = new QSpinBox(&dialog);
    profileRadialPassCount->setRange(1, 1000);
    profileRadialPassCount->setValue(
        m_profileToolpathParams.profile.radialPassCount);
    profileRadialPassCount->setToolTip(
        QStringLiteral("大于 1 时按 FreeCAD Profile 语义从远处逐刀收敛到最终轮廓"));
    profileForm->addRow(QStringLiteral("径向走刀次数"),
                       profileRadialPassCount);

    auto *profileRadialStepOver = new QDoubleSpinBox(&dialog);
    profileRadialStepOver->setRange(0.0, 10000.0);
    profileRadialStepOver->setDecimals(3);
    profileRadialStepOver->setSingleStep(0.5);
    profileRadialStepOver->setSuffix(QStringLiteral(" mm"));
    profileRadialStepOver->setSpecialValueText(
        QStringLiteral("自动（按刀径步距）"));
    profileRadialStepOver->setValue(
        m_profileToolpathParams.profile.radialStepOver);
    profileForm->addRow(QStringLiteral("径向走刀间距"),
                        profileRadialStepOver);

    auto *profileAutomaticStepOver = new QDoubleSpinBox(&dialog);
    profileAutomaticStepOver->setRange(1.0, 100.0);
    profileAutomaticStepOver->setDecimals(1);
    profileAutomaticStepOver->setSingleStep(5.0);
    profileAutomaticStepOver->setSuffix(QStringLiteral(" %"));
    profileAutomaticStepOver->setValue(
        m_profileToolpathParams.clearing.stepOverPercent);
    profileAutomaticStepOver->setToolTip(
        QStringLiteral("径向走刀间距设为自动时，按刀具直径的此百分比计算"));
    profileForm->addRow(QStringLiteral("自动径向步距"),
                        profileAutomaticStepOver);

    auto makeProfileLeadLength = [&](double value) {
        auto *control = new QDoubleSpinBox(&dialog);
        control->setRange(0.0, 10000.0);
        control->setDecimals(3);
        control->setSingleStep(0.5);
        control->setSuffix(QStringLiteral(" mm"));
        control->setSpecialValueText(QStringLiteral("关闭"));
        control->setValue(value);
        return control;
    };
    auto *profileLeadIn = makeProfileLeadLength(
        m_profileToolpathParams.profile.leadInLength);
    auto *profileLeadOut = makeProfileLeadLength(
        m_profileToolpathParams.profile.leadOutLength);
    profileForm->addRow(QStringLiteral("切向引入长度"), profileLeadIn);
    profileForm->addRow(QStringLiteral("切向引出长度"), profileLeadOut);

    auto *profilePrecision = new QSpinBox(&dialog);
    profilePrecision->setRange(0, 8);
    profilePrecision->setValue(m_profileToolpathParams.profile.precision);
    profileForm->addRow(QStringLiteral("Clipper 精度"),
                       profilePrecision);

    auto *profileArcTolerance = new QDoubleSpinBox(&dialog);
    profileArcTolerance->setRange(0.0, 100.0);
    profileArcTolerance->setDecimals(4);
    profileArcTolerance->setSingleStep(0.01);
    profileArcTolerance->setSuffix(QStringLiteral(" mm"));
    profileArcTolerance->setSpecialValueText(QStringLiteral("自动"));
    profileArcTolerance->setValue(
        m_profileToolpathParams.profile.arcTolerance);
    profileForm->addRow(QStringLiteral("圆弧公差"),
                       profileArcTolerance);

    // 孔深度模式:使用识别深度或手动起始 / 最终偏移(勾选时禁用手动输入)。
    auto *useFeatureDepth =
        new QCheckBox(QStringLiteral("使用识别的孔深度"), &dialog);
    useFeatureDepth->setChecked(
        m_holeToolpathParams.depth.useFeatureDepth);
    form->addRow(QStringLiteral("深度模式"), useFeatureDepth);

    auto *startDepth = new QDoubleSpinBox(&dialog);
    startDepth->setRange(-1000.0, 1000.0);
    startDepth->setDecimals(3);
    startDepth->setSingleStep(0.5);
    startDepth->setSuffix(" mm");
    startDepth->setValue(m_holeToolpathParams.depth.startDepth);
    form->addRow(QStringLiteral("手动起始偏移"), startDepth);

    auto *finalDepth = new QDoubleSpinBox(&dialog);
    finalDepth->setRange(-1000.0, 1000.0);
    finalDepth->setDecimals(3);
    finalDepth->setSingleStep(0.5);
    finalDepth->setSuffix(" mm");
    finalDepth->setValue(m_holeToolpathParams.depth.finalDepth);
    form->addRow(QStringLiteral("手动最终偏移"), finalDepth);

    auto *stepDown = new QDoubleSpinBox(&dialog);
    stepDown->setRange(0.001, 1000.0);
    stepDown->setDecimals(3);
    stepDown->setSingleStep(0.5);
    stepDown->setSuffix(" mm");
    stepDown->setValue(m_holeToolpathParams.depth.stepDown);
    form->addRow(QStringLiteral("每层切深"), stepDown);

    auto *safeHeight = new QDoubleSpinBox(&dialog);
    safeHeight->setRange(0.0, 1000.0);
    safeHeight->setDecimals(3);
    safeHeight->setSingleStep(0.5);
    safeHeight->setSuffix(" mm");
    safeHeight->setValue(m_holeToolpathParams.depth.safeHeight);
    form->addRow(QStringLiteral("安全偏移（相对下刀面）"), safeHeight);

    auto *clearanceHeight = new QDoubleSpinBox(&dialog);
    clearanceHeight->setRange(0.0, 1000.0);
    clearanceHeight->setDecimals(3);
    clearanceHeight->setSingleStep(0.5);
    clearanceHeight->setSuffix(" mm");
    clearanceHeight->setValue(m_holeToolpathParams.depth.clearanceHeight);
    form->addRow(QStringLiteral("抬刀间隙（相对下刀面）"), clearanceHeight);

    auto *pocketUseFeatureDepth =
        new QCheckBox(
            QStringLiteral("使用识别的槽 / 腔深度"), &dialog);
    pocketUseFeatureDepth->setChecked(
        m_pocketToolpathParams.depth.useFeatureDepth);
    pocketForm->addRow(QStringLiteral("深度模式"),
                       pocketUseFeatureDepth);

    auto *pocketStartDepth = new QDoubleSpinBox(&dialog);
    pocketStartDepth->setRange(-1000.0, 1000.0);
    pocketStartDepth->setDecimals(3);
    pocketStartDepth->setSingleStep(0.5);
    pocketStartDepth->setSuffix(" mm");
    pocketStartDepth->setValue(
        m_pocketToolpathParams.depth.startDepth);
    pocketForm->addRow(QStringLiteral("手动起始偏移"),
                       pocketStartDepth);

    auto *pocketFinalDepth = new QDoubleSpinBox(&dialog);
    pocketFinalDepth->setRange(-1000.0, 1000.0);
    pocketFinalDepth->setDecimals(3);
    pocketFinalDepth->setSingleStep(0.5);
    pocketFinalDepth->setSuffix(" mm");
    pocketFinalDepth->setValue(
        m_pocketToolpathParams.depth.finalDepth);
    pocketForm->addRow(QStringLiteral("手动最终偏移"),
                       pocketFinalDepth);

    auto *pocketStepDown = new QDoubleSpinBox(&dialog);
    pocketStepDown->setRange(0.001, 1000.0);
    pocketStepDown->setDecimals(3);
    pocketStepDown->setSingleStep(0.5);
    pocketStepDown->setSuffix(" mm");
    pocketStepDown->setValue(m_pocketToolpathParams.depth.stepDown);
    pocketForm->addRow(QStringLiteral("每层切深"), pocketStepDown);

    auto *pocketSafeHeight = new QDoubleSpinBox(&dialog);
    pocketSafeHeight->setRange(0.0, 1000.0);
    pocketSafeHeight->setDecimals(3);
    pocketSafeHeight->setSingleStep(0.5);
    pocketSafeHeight->setSuffix(" mm");
    pocketSafeHeight->setValue(
        m_pocketToolpathParams.depth.safeHeight);
    pocketForm->addRow(QStringLiteral("安全偏移（相对下刀面）"),
                       pocketSafeHeight);

    auto *pocketClearanceHeight = new QDoubleSpinBox(&dialog);
    pocketClearanceHeight->setRange(0.0, 1000.0);
    pocketClearanceHeight->setDecimals(3);
    pocketClearanceHeight->setSingleStep(0.5);
    pocketClearanceHeight->setSuffix(" mm");
    pocketClearanceHeight->setValue(
        m_pocketToolpathParams.depth.clearanceHeight);
    pocketForm->addRow(QStringLiteral("抬刀间隙（相对下刀面）"),
                        pocketClearanceHeight);

    auto *profileUseFeatureDepth = new QCheckBox(
        QStringLiteral("使用识别的外侧壁深度"), &dialog);
    profileUseFeatureDepth->setChecked(
        m_profileToolpathParams.depth.useFeatureDepth);
    profileForm->addRow(QStringLiteral("深度模式"),
                        profileUseFeatureDepth);

    auto *profileStartDepth = new QDoubleSpinBox(&dialog);
    profileStartDepth->setRange(-1000.0, 1000.0);
    profileStartDepth->setDecimals(3);
    profileStartDepth->setSingleStep(0.5);
    profileStartDepth->setSuffix(QStringLiteral(" mm"));
    profileStartDepth->setValue(m_profileToolpathParams.depth.startDepth);
    profileForm->addRow(QStringLiteral("手动起始偏移"), profileStartDepth);

    auto *profileFinalDepth = new QDoubleSpinBox(&dialog);
    profileFinalDepth->setRange(-1000.0, 1000.0);
    profileFinalDepth->setDecimals(3);
    profileFinalDepth->setSingleStep(0.5);
    profileFinalDepth->setSuffix(QStringLiteral(" mm"));
    profileFinalDepth->setValue(m_profileToolpathParams.depth.finalDepth);
    profileForm->addRow(QStringLiteral("手动最终偏移"), profileFinalDepth);

    auto *profileStepDown = new QDoubleSpinBox(&dialog);
    profileStepDown->setRange(0.001, 1000.0);
    profileStepDown->setDecimals(3);
    profileStepDown->setSingleStep(0.5);
    profileStepDown->setSuffix(QStringLiteral(" mm"));
    profileStepDown->setValue(m_profileToolpathParams.depth.stepDown);
    profileForm->addRow(QStringLiteral("每层切深"), profileStepDown);

    auto *profileSafeHeight = new QDoubleSpinBox(&dialog);
    profileSafeHeight->setRange(0.0, 1000.0);
    profileSafeHeight->setDecimals(3);
    profileSafeHeight->setSingleStep(0.5);
    profileSafeHeight->setSuffix(QStringLiteral(" mm"));
    profileSafeHeight->setValue(m_profileToolpathParams.depth.safeHeight);
    profileForm->addRow(QStringLiteral("安全偏移（相对下刀面）"),
                        profileSafeHeight);

    auto *profileClearanceHeight = new QDoubleSpinBox(&dialog);
    profileClearanceHeight->setRange(0.0, 1000.0);
    profileClearanceHeight->setDecimals(3);
    profileClearanceHeight->setSingleStep(0.5);
    profileClearanceHeight->setSuffix(QStringLiteral(" mm"));
    profileClearanceHeight->setValue(
        m_profileToolpathParams.depth.clearanceHeight);
    profileForm->addRow(QStringLiteral("抬刀间隙（相对下刀面）"),
                        profileClearanceHeight);

    // 正式工序使用六面装夹；不显示未被规划/导出消费的旧单装夹字段。
    auto *setupNotice = new QLabel(
        QStringLiteral("各工序采用下列六面工作坐标；方向相对于项目工件坐标。悬停可查看原点与轴向。勾选表示已核对对应装夹，参考 NC 会保留未确认提示。"),
        &dialog);
    setupNotice->setWordWrap(true);
    setupForm->addRow(setupNotice);
    std::array<QCheckBox*, 6> setupConfirmations{};
    for (int index = 0; index < 6; ++index) {
        const oap::MoldSetupFace face =
            static_cast<oap::MoldSetupFace>(index);
        const oap::toolpath::MoldFaceSetupWorkFrame* setup =
            m_moldSetupWorkFrames.forFace(face);
        auto *confirmation = new QCheckBox(QStringLiteral("已确认"), &dialog);
        setupConfirmations[static_cast<std::size_t>(index)] = confirmation;
        confirmation->setEnabled(setup && setup->valid);
        confirmation->setChecked(setup && setup->confirmed);
        const QString offset = setup
            ? QString::fromLatin1(
                  oap::toolpath::workCoordinateCode(setup->workOffset))
            : QStringLiteral("未建立");
        const QString label = QStringLiteral("%1 / %2")
            .arg(QString::fromLatin1(oap::moldSetupFaceName(face)))
            .arg(offset);
        if (setup && setup->valid) {
            const gp_Pnt origin = setup->setupWorkFrame.Location();
            confirmation->setToolTip(
                QStringLiteral("原始 STEP 世界原点映射：(X=%1, Y=%2, Z=%3 mm)\n轴向：%4\n%5")
                    .arg(origin.X(), 0, 'f', 4)
                    .arg(origin.Y(), 0, 'f', 4)
                    .arg(origin.Z(), 0, 'f', 4)
                    .arg(QString::fromStdString(setup->axisMapping))
                    .arg(QString::fromStdString(setup->operatorInstruction)));
        }
        setupForm->addRow(label, confirmation);
    }

    auto *collisionScope = new QLabel(
        QStringLiteral(
            "采用六面装夹假设：加工一个面时默认其余五面被夹持。"
            "当前仅检查刀具装配与工件碰撞，不加载夹具、压板或机床结构模型；"
            "该范围说明不阻断导出。"),
        &dialog);
    collisionScope->setWordWrap(true);
    collisionScope->setStyleSheet(
        QStringLiteral("QLabel { color: #a15c00; padding: 4px; }"));
    setupForm->addRow(QStringLiteral("碰撞审计范围"), collisionScope);
    // 输出设置页:NC 输出范围说明与冷却指令选项。
    auto *referenceNotice = new QLabel(
        QStringLiteral(
            "当前 path-NC 分支只生成参考 NC：按装夹、刀具和加工阶段拆分，"
            "使用 G28、M07/M09，并保留规划器的 G98/G99 安全决策，"
            "且不输出 T/M6/G43。"),
        &dialog);
    referenceNotice->setWordWrap(true);
    outputForm->addRow(QStringLiteral("NC 输出"), referenceNotice);
    auto *coolant = new QCheckBox(
        QStringLiteral("输出参考模板冷却指令 M07/M09"), &dialog);
    coolant->setChecked(m_gcodeProgramOptions.coolant);
    outputForm->addRow(QStringLiteral("冷却液"), coolant);
    auto setManualDepthEnabled = [=](bool useRecognized) {
        startDepth->setEnabled(!useRecognized);
        finalDepth->setEnabled(!useRecognized);
    };
    setManualDepthEnabled(useFeatureDepth->isChecked());
    QObject::connect(useFeatureDepth, &QCheckBox::toggled, &dialog,
                     setManualDepthEnabled);
    auto setPocketManualDepthEnabled = [=](bool useRecognized) {
        pocketStartDepth->setEnabled(!useRecognized);
        pocketFinalDepth->setEnabled(!useRecognized);
    };
    setPocketManualDepthEnabled(pocketUseFeatureDepth->isChecked());
    QObject::connect(pocketUseFeatureDepth, &QCheckBox::toggled, &dialog,
                      setPocketManualDepthEnabled);

    auto setProfileManualDepthEnabled = [=](bool useRecognized) {
        profileStartDepth->setEnabled(!useRecognized);
        profileFinalDepth->setEnabled(!useRecognized);
    };
    setProfileManualDepthEnabled(profileUseFeatureDepth->isChecked());
    QObject::connect(profileUseFeatureDepth, &QCheckBox::toggled, &dialog,
                     setProfileManualDepthEnabled);

    auto *buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    dialogLayout->addWidget(buttons);
    // 确认前一致性校验:G73 最大深径比不得小于 G81。
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, [&] {
        for (const auto* section : {&holeTool, &pocketTool, &profileTool}) {
            const QString error = section->validationError();
            if (!error.isEmpty()) {
                QMessageBox::warning(&dialog, QStringLiteral("单刀参数"), error);
                return;
            }
        }
        const QString engravingError = engraving.validationError();
        if (!engravingError.isEmpty()) {
            QMessageBox::warning(&dialog, QStringLiteral("雕刻参数"), engravingError);
            return;
        }
        if (g73MaxRatio->value() < g81MaxRatio->value()) {
            QMessageBox::warning(
                &dialog, QStringLiteral("钻削循环参数"),
                QStringLiteral("G73 最大深径比不能小于 G81 最大深径比。"));
            return;
        }
        dialog.accept();
    });
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog,
                     &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted)
        return std::nullopt;

    // 用户确认后,把各控件值逐项回写到工作副本并返回。
    edited.holeToolSelectionMode = holeTool.mode();
    edited.pocketToolSelectionMode = pocketTool.mode();
    edited.profileToolSelectionMode = profileTool.mode();
    if (holeTool.mode() == ToolSelectionMode::SingleTool) edited.holeSingleTool = holeTool.tool();
    if (pocketTool.mode() == ToolSelectionMode::SingleTool) edited.pocketSingleTool = pocketTool.tool();
    if (profileTool.mode() == ToolSelectionMode::SingleTool) edited.profileSingleTool = profileTool.tool();
    edited.engraving = engraving.value();
    m_pocketToolpathParams.clearing.stepOverPercent =
        stepOver->value();
    m_pocketToolpathParams.clearing.extraOffset =
        extraOffset->value();
    m_pocketToolpathParams.depth.finishDepth =
        finishAllowance->value();

    m_holeToolpathParams.hole.allowance = holeAllowance->value();
    m_holeToolpathParams.hole.smallHoleDiameterTolerance =
        smallHoleDiameterTolerance->value();
    m_holeToolpathParams.hole.centerSpotDepth =
        centerSpotDepth->value();
    m_holeToolpathParams.hole.throughHoleBreakthrough =
        throughHoleBreakthrough->value();
    m_holeToolpathParams.hole.drillCycleMode =
        static_cast<oap::toolpath::HoleDrillCycleMode>(
            drillCycleMode->currentData().toInt());
    m_holeToolpathParams.hole.peckDepth = peckDepth->value();
    m_holeToolpathParams.hole.dwellSeconds = drillDwell->value();
    m_holeToolpathParams.hole.drillTipAngleDeg =
        drillTipAngle->value();
    m_holeToolpathParams.hole.g81MaxDepthDiameterRatio =
        g81MaxRatio->value();
    m_holeToolpathParams.hole.g73MaxDepthDiameterRatio =
        g73MaxRatio->value();
    m_holeToolpathParams.hole.g73PeckDepth =
        g73PeckDepth->value();
    m_holeToolpathParams.hole.g83PeckDepth =
        g83PeckDepth->value();
    m_holeToolpathParams.hole.g73ChipBreakRetract =
        g73Retract->value();
    m_holeToolpathParams.hole.g83ReentryClearance =
        g83Reentry->value();
    m_holeToolpathParams.hole.collisionEnvelopeMode =
        static_cast<oap::toolpath::HoleCollisionEnvelopeMode>(
            collisionEnvelopeMode->currentData().toInt());
    m_holeToolpathParams.hole.collisionEnvelopeSampleStep =
        collisionSampleStep->value();
    m_holeToolpathParams.hole.mediumHoleRadiusToolDiameterRatio =
        mediumHoleRatio->value();
    m_holeToolpathParams.hole.startCorner =
        static_cast<oap::toolpath::HoleStartCorner>(
            holeStartCorner->currentData().toInt());
    m_pocketToolpathParams.clearing.pattern =
        static_cast<oap::toolpath::ClearingPattern>(clearingPattern->currentData().toInt());
    m_pocketToolpathParams.clearing.expansionDirection =
        oap::toolpath::effectivePocketClearingDirection(
            m_pocketToolpathParams.clearing.pattern,
            static_cast<oap::toolpath::PocketClearingDirection>(
                expansionDirection->currentData().toInt()));
    m_pocketToolpathParams.clearing.cutMode =
        static_cast<oap::toolpath::CutMode>(cutMode->currentData().toInt());
    m_pocketToolpathParams.clearing.entryMode =
        static_cast<oap::toolpath::PocketEntryMode>(
            entryMode->currentData().toInt());
    m_pocketToolpathParams.clearing.rampStartReference =
        static_cast<oap::toolpath::PocketRampStartReference>(
            rampStartReference->currentData().toInt());
    m_pocketToolpathParams.clearing.rampAngleDeg =
        rampAngle->value();
    m_pocketToolpathParams.clearing.maximumRampLength =
        maximumRampLength->value();
    m_pocketToolpathParams.clearing.helixDiameter =
        helixDiameter->value();
    m_pocketToolpathParams.clearing.zigzagRampWidth =
        zigzagRampWidth->value();
    m_pocketToolpathParams.clearing.rampFeed = rampFeed->value();
    m_pocketToolpathParams.clearing.precision = precision->value();
    m_pocketToolpathParams.clearing.arcTolerance =
        arcTolerance->value();
    m_pocketToolpathParams.clearing.adaptive.targetEngagement =
        adaptiveTarget->value();
    m_pocketToolpathParams.clearing.adaptive.targetEngagementAngleDeg =
        adaptiveTargetAngle->value();
    m_pocketToolpathParams.clearing.adaptive.maximumEngagementAngleDeg =
        adaptiveMaximumAngle->value();
    m_pocketToolpathParams.clearing.adaptive.engagementAngleSamples =
        adaptiveAngleSamples->value();
    m_pocketToolpathParams.clearing.adaptive.minimumTurnRadiusFactor =
        adaptiveTurnRadius->value();
    m_pocketToolpathParams.clearing.adaptive.remainingTolerance =
        adaptiveRemainingTolerance->value();
    m_pocketToolpathParams.clearing.adaptive.materialCellSize =
        adaptiveCellSize->value();
    m_pocketToolpathParams.clearing.adaptive.maximumAdaptivePasses =
        adaptiveMaximumPasses->value();
    m_pocketToolpathParams.clearing.adaptive.lookaheadDepth =
        adaptiveLookahead->value();
    m_pocketToolpathParams.clearing.adaptive.beamWidth =
        adaptiveBeamWidth->value();
    m_profileToolpathParams.profile.machiningMode =
        static_cast<oap::toolpath::ProfileMachiningMode>(
            profileMachiningMode->currentData().toInt());
    m_profileToolpathParams.clearing.pattern =
        static_cast<oap::toolpath::ClearingPattern>(
            profileClearingPattern->currentData().toInt());
    m_profileToolpathParams.clearing.expansionDirection =
        oap::toolpath::effectivePocketClearingDirection(
            m_profileToolpathParams.clearing.pattern,
            m_profileToolpathParams.clearing.expansionDirection);
    m_profileToolpathParams.clearing.zigzagAngle =
        profileZigzagAngle->value();
    m_profileToolpathParams.profile.cutMode =
        static_cast<oap::toolpath::CutMode>(
            profileCutMode->currentData().toInt());
    m_profileToolpathParams.profile.sideAllowance =
        profileSideAllowance->value();
    m_profileToolpathParams.profile.roughingAllowance =
        profileRoughingAllowance->value();
    m_profileToolpathParams.profile.finishPass =
        profileFinishPass->isChecked();
    m_profileToolpathParams.profile.radialPassCount =
        profileRadialPassCount->value();
    m_profileToolpathParams.profile.radialStepOver =
        profileRadialStepOver->value();
    m_profileToolpathParams.clearing.stepOverPercent =
        profileAutomaticStepOver->value();
    m_profileToolpathParams.profile.leadInLength =
        profileLeadIn->value();
    m_profileToolpathParams.profile.leadOutLength =
        profileLeadOut->value();
    m_profileToolpathParams.profile.precision =
        profilePrecision->value();
    m_profileToolpathParams.profile.arcTolerance =
        profileArcTolerance->value();
    m_holeToolpathParams.depth.useFeatureDepth =
        useFeatureDepth->isChecked();
    m_holeToolpathParams.depth.startDepth = startDepth->value();
    m_holeToolpathParams.depth.finalDepth = finalDepth->value();
    m_holeToolpathParams.depth.stepDown = stepDown->value();
    m_holeToolpathParams.depth.safeHeight = safeHeight->value();
    m_holeToolpathParams.depth.clearanceHeight =
        clearanceHeight->value();
    m_pocketToolpathParams.depth.useFeatureDepth =
        pocketUseFeatureDepth->isChecked();
    m_pocketToolpathParams.depth.startDepth =
        pocketStartDepth->value();
    m_pocketToolpathParams.depth.finalDepth =
        pocketFinalDepth->value();
    m_pocketToolpathParams.depth.stepDown = pocketStepDown->value();
    m_pocketToolpathParams.depth.safeHeight =
        pocketSafeHeight->value();
    m_pocketToolpathParams.depth.clearanceHeight =
        pocketClearanceHeight->value();
    m_profileToolpathParams.depth.useFeatureDepth =
        profileUseFeatureDepth->isChecked();
    m_profileToolpathParams.depth.startDepth = profileStartDepth->value();
    m_profileToolpathParams.depth.finalDepth = profileFinalDepth->value();
    m_profileToolpathParams.depth.stepDown = profileStepDown->value();
    m_profileToolpathParams.depth.safeHeight = profileSafeHeight->value();
    m_profileToolpathParams.depth.clearanceHeight =
        profileClearanceHeight->value();

    for (int index = 0; index < 6; ++index) {
        m_moldSetupWorkFrames.setConfirmed(
            static_cast<oap::MoldSetupFace>(index),
            setupConfirmations[static_cast<std::size_t>(index)]->isChecked());
    }
    m_gcodeProgramOptions.coolant = coolant->isChecked();

    return edited;
}

} // namespace oap::app

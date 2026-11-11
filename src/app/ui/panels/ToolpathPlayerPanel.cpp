// ToolpathPlayerPanel 实现 —— 参见 ToolpathPlayerPanel.h。
#include "app/ui/panels/ToolpathPlayerPanel.h"
#include <QCoreApplication>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QSlider>
#include <QLabel>
#include <QSizePolicy>
#include <QTimer>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QCheckBox>

#include <algorithm>
#include <cmath>

namespace {
// 固定 tick;速度倍率控制每 tick 前进的当前移动段比例。
constexpr int kBaseTickMs = 120;
constexpr double kFractionPerTickAt1x = 0.20;
} // namespace

ToolpathPlayerPanel::ToolpathPlayerPanel(QWidget* parent) : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->setSpacing(4);

    // 控制按钮行:播放/暂停 · 单步 · 按刀具批次跳转 · 速度。
    auto* row = new QHBoxLayout();
    row->setContentsMargins(0, 0, 0, 0);

    m_playButton = new QPushButton(QCoreApplication::translate("OpenAutoPath", "播放"), this);
    m_playButton->setCheckable(true);
    connect(m_playButton, &QPushButton::toggled, this, &ToolpathPlayerPanel::onPlayPauseToggled);
    row->addWidget(m_playButton);

    m_previousStepButton = new QPushButton(QCoreApplication::translate("OpenAutoPath", "上一步"), this);
    connect(m_previousStepButton, &QPushButton::clicked, this, &ToolpathPlayerPanel::onStepPrev);
    row->addWidget(m_previousStepButton);

    m_nextStepButton = new QPushButton(QCoreApplication::translate("OpenAutoPath", "下一步"), this);
    connect(m_nextStepButton, &QPushButton::clicked, this, &ToolpathPlayerPanel::onStepNext);
    row->addWidget(m_nextStepButton);

    m_previousToolButton = new QPushButton(QCoreApplication::translate("OpenAutoPath", "上一把刀"), this);
    connect(m_previousToolButton, &QPushButton::clicked,
            this, &ToolpathPlayerPanel::previousToolRequested);
    row->addWidget(m_previousToolButton);

    m_nextToolButton = new QPushButton(QCoreApplication::translate("OpenAutoPath", "下一把刀"), this);
    connect(m_nextToolButton, &QPushButton::clicked,
            this, &ToolpathPlayerPanel::nextToolRequested);
    row->addWidget(m_nextToolButton);

    // 与“播放”按钮一致使用原生 QPushButton，保留完整外围边框；勾选时
    // 由当前 Qt/Windows 主题绘制按下高亮，不硬编码颜色。
    m_currentToolPathButton = new QPushButton(QCoreApplication::translate("OpenAutoPath", "当前刀路径"), this);
    m_currentToolPathButton->setCheckable(true);
    m_currentToolPathButton->setToolTip(
        QCoreApplication::translate("OpenAutoPath", "高亮时只显示当前刀具在当前装夹面的完整路径"));
    connect(m_currentToolPathButton, &QPushButton::toggled,
            this, &ToolpathPlayerPanel::currentToolPathOnlyChanged);
    row->addWidget(m_currentToolPathButton);

    row->addStretch(1);
    row->addWidget(new QLabel(QCoreApplication::translate("OpenAutoPath", "速度"), this));
    m_speedComboBox = new QComboBox(this);
    m_speedComboBox->addItem(QStringLiteral("0.5x"), 0.5);
    m_speedComboBox->addItem(QStringLiteral("1x"), 1.0);
    m_speedComboBox->addItem(QStringLiteral("2x"), 2.0);
    m_speedComboBox->addItem(QStringLiteral("4x"), 4.0);
    m_speedComboBox->addItem(QStringLiteral("8x"), 8.0);
    m_speedComboBox->setCurrentIndex(1); // 默认 1x
    connect(m_speedComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ToolpathPlayerPanel::onSpeedChanged);
    row->addWidget(m_speedComboBox);

    layout->addLayout(row);

    // 普通刀路显示窗口：对应 FreeCAD Path 视图中的 StartIndex、ShowCount
    // 和 ShowNodes。这里只控制主 OCCT 视口中的静态刀路，不接触 OpenGL
    // 模拟器或材料去除状态。
    auto* staticRow = new QHBoxLayout();
    staticRow->setContentsMargins(0, 0, 0, 0);
    m_staticToolpathCountLabel =
        new QLabel(QCoreApplication::translate("OpenAutoPath", "普通刀路 0 段"), this);
    staticRow->addWidget(m_staticToolpathCountLabel);
    m_operationVisibilityLabel =
        new QLabel(QCoreApplication::translate("OpenAutoPath", "工序 —"), this);
    m_operationVisibilityLabel->setObjectName(
        QStringLiteral("operationVisibilityStatus"));
    staticRow->addWidget(m_operationVisibilityLabel);
    staticRow->addWidget(new QLabel(QCoreApplication::translate("OpenAutoPath", "起始"), this));
    m_staticStartSpinBox = new QSpinBox(this);
    m_staticStartSpinBox->setRange(0, 0);
    m_staticStartSpinBox->setToolTip(
        QCoreApplication::translate("OpenAutoPath", "第一条可见刀路段（从 1 开始）"));
    staticRow->addWidget(m_staticStartSpinBox);
    staticRow->addWidget(new QLabel(QCoreApplication::translate("OpenAutoPath", "数量"), this));
    m_staticCountSpinBox = new QSpinBox(this);
    m_staticCountSpinBox->setRange(0, 0);
    m_staticCountSpinBox->setToolTip(
        QCoreApplication::translate("OpenAutoPath", "从起始段开始显示的刀路段数量"));
    staticRow->addWidget(m_staticCountSpinBox);
    m_staticNodesCheckBox = new QCheckBox(QCoreApplication::translate("OpenAutoPath", "节点"), this);
    m_staticNodesCheckBox->setToolTip(
        QCoreApplication::translate("OpenAutoPath", "显示普通刀路各线段的端点节点"));
    staticRow->addWidget(m_staticNodesCheckBox);
    m_staticResetButton =
        new QPushButton(QCoreApplication::translate("OpenAutoPath", "显示全部刀路段"), this);
    m_staticResetButton->setToolTip(
        QCoreApplication::translate("OpenAutoPath", "恢复起始段/数量范围；不会改变左侧工序树的勾选状态"));
    staticRow->addWidget(m_staticResetButton);
    staticRow->addStretch(1);
    layout->addLayout(staticRow);

    connect(m_staticStartSpinBox,
            QOverload<int>::of(&QSpinBox::valueChanged),
            this, &ToolpathPlayerPanel::onStaticToolpathDisplayChanged);
    connect(m_staticCountSpinBox,
            QOverload<int>::of(&QSpinBox::valueChanged),
            this, &ToolpathPlayerPanel::onStaticToolpathDisplayChanged);
    connect(m_staticNodesCheckBox, &QCheckBox::toggled,
            this, &ToolpathPlayerPanel::onStaticToolpathDisplayChanged);
    connect(m_staticResetButton, &QPushButton::clicked, this, [this] {
        if (m_staticToolpathSegmentCount <= 0)
            return;
        const QSignalBlocker startBlock(m_staticStartSpinBox);
        const QSignalBlocker countBlock(m_staticCountSpinBox);
        m_staticStartSpinBox->setValue(1);
        m_staticCountSpinBox->setMaximum(m_staticToolpathSegmentCount);
        m_staticCountSpinBox->setValue(m_staticToolpathSegmentCount);
        emit staticToolpathDisplayChanged(
            0, m_staticToolpathSegmentCount,
            m_staticNodesCheckBox->isChecked());
    });

    // 进度滑块:范围 [0,count],值 = 当前 index,可拖动 seek。
    m_progressSlider = new QSlider(Qt::Horizontal, this);
    m_progressSlider->setMinimum(0);
    m_progressSlider->setMaximum(0);
    connect(m_progressSlider, &QSlider::valueChanged, this, &ToolpathPlayerPanel::onProgressSliderMoved);
    layout->addWidget(m_progressSlider);

    m_phaseInfoLabel = new QLabel(QCoreApplication::translate("OpenAutoPath", "当前阶段：—"), this);
    m_phaseInfoLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(m_phaseInfoLabel);

    m_toolInfoLabel = new QLabel(QCoreApplication::translate("OpenAutoPath", "当前刀具：—"), this);
    m_toolInfoLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(m_toolInfoLabel);

    m_infoLabel = new QLabel(QCoreApplication::translate("OpenAutoPath", "尚未生成刀路"), this);
    m_infoLabel->setWordWrap(true);
    m_infoLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    m_infoLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    m_infoLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_infoLabel->setToolTip(m_infoLabel->text());
    layout->addWidget(m_infoLabel);

    m_timer = new QTimer(this);
    m_timer->setInterval(kBaseTickMs);
    connect(m_timer, &QTimer::timeout, this, &ToolpathPlayerPanel::onTimerTick);

    setPlaybackControlsEnabled(false);
    setStaticToolpathSegmentCount(0);
    setOperationVisibilitySummary(0, 0, false);
}

void ToolpathPlayerPanel::setMoveCount(int moveCount)
{
    stop();
    m_moveCount = std::max(0, moveCount);
    m_completedMoveCount = 0;
    m_moveFraction = 1.0;
    m_hasPreviousTool = false;
    m_hasNextTool = false;

    {
        QSignalBlocker block(m_progressSlider);
        m_progressSlider->setMaximum(m_moveCount);
        m_progressSlider->setValue(0);
    }

    if (m_moveCount <= 0) {
        setPlaybackControlsEnabled(false);
        setMachiningContext(QStringLiteral("—"), QStringLiteral("—"));
        setInfoText(QCoreApplication::translate("OpenAutoPath", "尚未生成刀路"));
        emit playbackReset();
        return;
    }

    setPlaybackControlsEnabled(true);
    setMachiningContext(QCoreApplication::translate("OpenAutoPath", "等待开始"), QStringLiteral("—"));
    setInfoText(QCoreApplication::translate("OpenAutoPath", "第 0/%1 步").arg(m_moveCount));
    emit frameChanged(0, 1.0); // 归零:清掉旧的已走过叠加
}

void ToolpathPlayerPanel::stop()
{
    if (m_timer)
        m_timer->stop();
    if (m_playButton) {
        QSignalBlocker block(m_playButton);
        m_playButton->setChecked(false);
    }
    updatePlayButtonText();
}

void ToolpathPlayerPanel::setInfoText(const QString& text)
{
    m_infoLabel->setText(text);
    m_infoLabel->setToolTip(text);
}

void ToolpathPlayerPanel::setMachiningContext(const QString& phase,
                                               const QString& tool)
{
    const QString phaseText = QCoreApplication::translate("OpenAutoPath", "当前阶段：%1").arg(phase);
    const QString toolText = QCoreApplication::translate("OpenAutoPath", "当前刀具：%1").arg(tool);
    m_phaseInfoLabel->setText(phaseText);
    m_phaseInfoLabel->setToolTip(phaseText);
    m_toolInfoLabel->setText(toolText);
    m_toolInfoLabel->setToolTip(toolText);
}

void ToolpathPlayerPanel::seekToMoveStart(int moveIndex)
{
    stop();
    const int clamped = std::clamp(moveIndex, 0, m_moveCount);
    m_completedMoveCount = clamped;
    m_moveFraction = clamped < m_moveCount ? 0.0 : 1.0;
    {
        QSignalBlocker block(m_progressSlider);
        m_progressSlider->setValue(m_completedMoveCount);
    }
    emit frameChanged(m_completedMoveCount, m_moveFraction);
}

void ToolpathPlayerPanel::setToolNavigationState(bool hasPrevious,
                                                  bool hasNext)
{
    m_hasPreviousTool = hasPrevious;
    m_hasNextTool = hasNext;
    const bool hasMoves = m_moveCount > 0;
    m_previousToolButton->setEnabled(hasMoves && m_hasPreviousTool);
    m_nextToolButton->setEnabled(hasMoves && m_hasNextTool);
}

bool ToolpathPlayerPanel::isPlaying() const
{
    return m_timer && m_timer->isActive();
}

bool ToolpathPlayerPanel::currentToolPathOnly() const
{
    return m_currentToolPathButton && m_currentToolPathButton->isChecked();
}

void ToolpathPlayerPanel::setStaticToolpathSegmentCount(int segmentCount)
{
    m_staticToolpathSegmentCount = std::max(0, segmentCount);
    const bool enabled = m_staticToolpathSegmentCount > 0;
    {
        const QSignalBlocker startBlock(m_staticStartSpinBox);
        const QSignalBlocker countBlock(m_staticCountSpinBox);
        if (enabled) {
            m_staticStartSpinBox->setRange(
                1, m_staticToolpathSegmentCount);
            m_staticStartSpinBox->setValue(1);
            m_staticCountSpinBox->setRange(
                1, m_staticToolpathSegmentCount);
            m_staticCountSpinBox->setValue(
                m_staticToolpathSegmentCount);
        } else {
            m_staticStartSpinBox->setRange(0, 0);
            m_staticStartSpinBox->setValue(0);
            m_staticCountSpinBox->setRange(0, 0);
            m_staticCountSpinBox->setValue(0);
        }
    }
    m_staticToolpathCountLabel->setText(
        QCoreApplication::translate("OpenAutoPath", "普通刀路 %1 段")
            .arg(m_staticToolpathSegmentCount));
    m_staticStartSpinBox->setEnabled(enabled);
    m_staticCountSpinBox->setEnabled(enabled);
    m_staticNodesCheckBox->setEnabled(enabled);
    m_staticResetButton->setEnabled(enabled);
    emit staticToolpathDisplayChanged(
        0, enabled ? m_staticToolpathSegmentCount : -1,
        m_staticNodesCheckBox->isChecked());
}

void ToolpathPlayerPanel::setOperationVisibilitySummary(
    int visibleCount, int totalCount, bool soloMode)
{
    if (!m_operationVisibilityLabel)
        return;

    const int total = std::max(0, totalCount);
    const int visible = std::clamp(visibleCount, 0, total);
    const bool allHidden = total > 0 && visible == 0;
    if (total == 0) {
        m_operationVisibilityLabel->setText(QCoreApplication::translate("OpenAutoPath", "工序 —"));
    } else if (soloMode) {
        m_operationVisibilityLabel->setText(
            QCoreApplication::translate("OpenAutoPath", "单独显示 1/%1 工序").arg(total));
    } else if (allHidden) {
        m_operationVisibilityLabel->setText(
            QCoreApplication::translate("OpenAutoPath", "工序 0/%1（全部隐藏）").arg(total));
    } else {
        m_operationVisibilityLabel->setText(
            QCoreApplication::translate("OpenAutoPath", "工序 %1/%2").arg(visible).arg(total));
    }
    m_operationVisibilityLabel->setToolTip(
        allHidden
            ? QCoreApplication::translate("OpenAutoPath", "全部刀路已由 CAM 工序树隐藏；点击“显示全部工序”恢复")
            : QCoreApplication::translate("OpenAutoPath", "左侧工序树控制工序可见性；起始段/数量只控制显示范围"));
    m_operationVisibilityLabel->setStyleSheet(
        allHidden ? QStringLiteral("color: #ffb74d;") : QString{});
}

void ToolpathPlayerPanel::setPlaybackControlsEnabled(bool enabled)
{
    m_playButton->setEnabled(enabled);
    m_previousStepButton->setEnabled(enabled);
    m_nextStepButton->setEnabled(enabled);
    m_previousToolButton->setEnabled(enabled && m_hasPreviousTool);
    m_nextToolButton->setEnabled(enabled && m_hasNextTool);
    m_currentToolPathButton->setEnabled(enabled);
    m_speedComboBox->setEnabled(enabled);
    m_progressSlider->setEnabled(enabled);
}

void ToolpathPlayerPanel::updatePlayButtonText()
{
    m_playButton->setText(m_playButton->isChecked() ? QCoreApplication::translate("OpenAutoPath", "暂停")
                                              : QCoreApplication::translate("OpenAutoPath", "播放"));
}

void ToolpathPlayerPanel::setCompletedMoveCount(int completedMoveCount)
{
    const int clamped =
        std::clamp(completedMoveCount, 0, m_moveCount);
    if (clamped == m_completedMoveCount) {
        // 即使完成段数未变也刷新一次滑块(拖动被夹取时)。如果正在段内播放，
        // 同一位置的 seek/上一步应回到整段边界并刷新视图。
        QSignalBlocker block(m_progressSlider);
        m_progressSlider->setValue(m_completedMoveCount);
        if (m_moveFraction != 1.0) {
            m_moveFraction = 1.0;
            emit frameChanged(m_completedMoveCount, m_moveFraction);
        }
        return;
    }
    m_completedMoveCount = clamped;
    m_moveFraction = 1.0;
    {
        QSignalBlocker block(m_progressSlider);
        m_progressSlider->setValue(m_completedMoveCount);
    }
    emit frameChanged(m_completedMoveCount, 1.0);
}

void ToolpathPlayerPanel::onTimerTick()
{
    if (m_moveCount <= 0 || m_completedMoveCount >= m_moveCount) {
        stop();
        return;
    }

    if (m_moveFraction >= 1.0)
        m_moveFraction = 0.0;

    m_moveFraction += kFractionPerTickAt1x * m_speedMultiplier;
    while (m_moveFraction >= 1.0 && m_completedMoveCount < m_moveCount) {
        m_moveFraction -= 1.0;
        ++m_completedMoveCount;
        {
            QSignalBlocker block(m_progressSlider);
            m_progressSlider->setValue(m_completedMoveCount);
        }
    }

    if (m_completedMoveCount >= m_moveCount) {
        m_moveFraction = 1.0;
        emit frameChanged(m_moveCount, 1.0);
        stop(); // 走到末尾自动停
        return;
    }

    emit frameChanged(m_completedMoveCount, std::clamp(m_moveFraction, 0.0, 1.0));
}

void ToolpathPlayerPanel::onPlayPauseToggled(bool playing)
{
    updatePlayButtonText();
    if (playing) {
        if (m_moveCount <= 0) {
            stop();
            return;
        }
        if (m_completedMoveCount >= m_moveCount) // 已在末尾则从头重播
            setCompletedMoveCount(0);
        if (m_completedMoveCount < m_moveCount) {
            m_moveFraction = 0.0;
            emit frameChanged(m_completedMoveCount, m_moveFraction);
        }
        m_timer->start();
    } else {
        m_timer->stop();
        if (m_moveCount > 0)
            emit frameChanged(m_completedMoveCount, std::clamp(m_moveFraction, 0.0, 1.0));
    }
}

void ToolpathPlayerPanel::onStepPrev()
{
    stop();
    if (m_moveFraction > 0.0 && m_moveFraction < 1.0)
        setCompletedMoveCount(m_completedMoveCount);
    else
        setCompletedMoveCount(m_completedMoveCount - 1);
}

void ToolpathPlayerPanel::onStepNext()
{
    stop();
    setCompletedMoveCount(m_completedMoveCount + 1);
}

void ToolpathPlayerPanel::onSpeedChanged(int comboIndex)
{
    const QVariant speedData = m_speedComboBox->itemData(comboIndex);
    if (speedData.isValid())
        m_speedMultiplier = speedData.toDouble();
}

void ToolpathPlayerPanel::onProgressSliderMoved(int value)
{
    // 用户拖动 seek：暂停并跳到对应的已完成段数。
    stop();
    setCompletedMoveCount(value);
}

void ToolpathPlayerPanel::onStaticToolpathDisplayChanged()
{
    if (m_staticToolpathSegmentCount <= 0)
        return;
    const int firstIndex = std::clamp(
        m_staticStartSpinBox->value() - 1,
        0, m_staticToolpathSegmentCount - 1);
    const int maximumCount = m_staticToolpathSegmentCount - firstIndex;
    {
        const QSignalBlocker block(m_staticCountSpinBox);
        m_staticCountSpinBox->setMaximum(maximumCount);
        if (m_staticCountSpinBox->value() > maximumCount)
            m_staticCountSpinBox->setValue(maximumCount);
    }
    emit staticToolpathDisplayChanged(
        firstIndex, m_staticCountSpinBox->value(),
        m_staticNodesCheckBox->isChecked());
}

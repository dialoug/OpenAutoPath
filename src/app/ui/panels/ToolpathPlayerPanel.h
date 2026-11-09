// ToolpathPlayerPanel — 刀路走刀过程的播放控制面板(停靠窗口内容)。
// 纯 UI/计时组件,不含任何 OCCT 类型:持有 QTimer 与播放状态(当前步、速度、
// 是否在播放),对外只发信号。播放工作区接收 frameChanged 后发布刀尖和
// 刀尖之后的剩余路径，已走部分实时隐藏；面板不访问 OCCT context。
#pragma once

#include <QWidget>

class QTimer;
class QPushButton;
class QComboBox;
class QSlider;
class QLabel;
class QSpinBox;
class QCheckBox;

// 刀路播放面板：播放控制、进度与刀具过滤。
class ToolpathPlayerPanel : public QWidget
{
    Q_OBJECT
public:
    explicit ToolpathPlayerPanel(QWidget* parent = nullptr);

    // 装载/复位播放序列长度。moveCount<=0 表示无刀路:整体禁用并显示提示。
    // 装载后停在第 0 步(尚未走任何一段),并发一次 frameChanged(0,...) 让视图归零。
    void setMoveCount(int moveCount);

    // 停止播放(不改变已完成段数)。定时器停、播放按钮复位。
    void stop();

    int completedMoveCount() const { return m_completedMoveCount; }
    double currentMoveFraction() const { return m_moveFraction; }
    double speedMultiplier() const { return m_speedMultiplier; }

    // 是否正在播放(供播放适配层判断是否让视图跟随刀尖;步进/拖动 seek 时为 false)。
    bool isPlaying() const;

    // 由 MainWindow 写入的当前步描述(第 i/N 步 · 类型 · 深度层...)。
    void setInfoText(const QString& text);
    // 当前 Operation 的工艺上下文。阶段和刀具独立显示，避免长步骤文本中
    // 看不清换刀/换阶段边界。
    void setMachiningContext(const QString& phase, const QString& tool);
    // 跳到指定移动段的起点。用于按刀具批次跳转；会先暂停播放，并让界面
    // 显示该段所属的新刀具，而不是前一段已经完成的刀具。
    void seekToMoveStart(int moveIndex);
    // 播放适配层根据实际加工程序中的刀具批次更新边界状态。首/末批次的
    // 对应按钮保持禁用，既提供视觉反馈，也从 UI 层阻止越界点击。
    void setToolNavigationState(bool hasPrevious, bool hasNext);
    // 是否仅显示当前刀具批次。按钮保持按下高亮时，播放工作区负责隐藏
    // 其他刀具和其他装夹面的静态预览。
    bool currentToolPathOnly() const;

    // 普通（非 OpenGL/IPW）刀路的显示窗口。装载新预览时默认显示全部，
    // 用户可限制起始段/数量并显示每段节点。
    void setStaticToolpathSegmentCount(int segmentCount);
    // 工序树的有效可见性摘要；与“起始段/数量”范围相互独立。
    void setOperationVisibilitySummary(
        int visibleCount, int totalCount, bool soloMode);

signals:
    // 当前走刀位置变化。completedMoveCount = 已走完的移动段数
    // [0,moveCount]。moveFraction < 1 表示正在执行下一段，刀尖位于该段
    // from->to 的 moveFraction 处；moveFraction == 1 表示停在整段边界。
    void frameChanged(int completedMoveCount, double moveFraction);
    // 序列被清空(moveCount<=0)。播放适配层据此清掉播放叠加。
    void playbackReset();
    void previousToolRequested();
    void nextToolRequested();
    void currentToolPathOnlyChanged(bool enabled);
    void staticToolpathDisplayChanged(
        int firstIndex, int count, bool showNodes);

private slots:
    void onTimerTick();
    void onPlayPauseToggled(bool playing);
    void onStepPrev();
    void onStepNext();
    void onSpeedChanged(int comboIndex);
    void onProgressSliderMoved(int value);
    void onStaticToolpathDisplayChanged();

private:
    // 夹取到 [0,m_moveCount]、刷新滑块/标签、发 frameChanged。
    void setCompletedMoveCount(int completedMoveCount);
    void setPlaybackControlsEnabled(bool enabled);
    void updatePlayButtonText();

    QTimer*      m_timer = nullptr;
    QPushButton* m_playButton = nullptr;  // checkable:播放 / 暂停
    QPushButton* m_previousStepButton = nullptr;
    QPushButton* m_nextStepButton = nullptr;
    QPushButton* m_previousToolButton = nullptr;
    QPushButton* m_nextToolButton = nullptr;
    QPushButton* m_currentToolPathButton = nullptr;
    QComboBox*   m_speedComboBox = nullptr;
    QSlider*     m_progressSlider = nullptr;
    QLabel*      m_infoLabel = nullptr;
    QLabel*      m_phaseInfoLabel = nullptr;
    QLabel*      m_toolInfoLabel = nullptr;
    QLabel*      m_staticToolpathCountLabel = nullptr;
    QLabel*      m_operationVisibilityLabel = nullptr;
    QSpinBox*    m_staticStartSpinBox = nullptr;
    QSpinBox*    m_staticCountSpinBox = nullptr;
    QCheckBox*   m_staticNodesCheckBox = nullptr;
    QPushButton* m_staticResetButton = nullptr;

    int m_completedMoveCount = 0;  // 已走完的移动段数，范围 [0,m_moveCount]
    int m_moveCount = 0;           // 移动段总数
    double m_moveFraction = 1.0;   // 当前段内进度；1.0 表示停在整段边界
    double m_speedMultiplier = 1.0; // 速度倍率(0.5/1/2/4/8)
    bool m_hasPreviousTool = false;
    bool m_hasNextTool = false;
    int m_staticToolpathSegmentCount = 0;
};

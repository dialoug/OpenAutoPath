// 只交换值类型与回调的毛坯/IPW 对话框。
// 对话框不直接触碰场景或模型:一切交互通过值快照 + 回调集
// (model/present/run/clear)完成,调用方在确认后自行提交结果。
#pragma once

#include <optional>
#include <QCoreApplication>
#include <vector>

#include <QDialog>
#include <QString>

#include "app/workflows/simulation/StockSimulationViewModels.h"

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class QSpinBox;
class QWidget;

namespace oap::app {

// 模态编辑腔毛坯仿真设置;确认返回新状态,取消返回 nullopt。
// 仅操作传入值快照的副本,不触碰任何场景状态。
std::optional<PocketStockSimulationDialogState>
editPocketStockSimulationSettings(
    QWidget* parent,
    const PocketStockSimulationDialogState& initial,
    const QString& operationName = QCoreApplication::translate("OpenAutoPath", "槽腔"),
    bool offerOpenGlPlayback = true);

// 孔毛坯仿真对话框:通过回调集查询模型、执行仿真、刷新显示,
// 自身不拥有仿真状态;控件每次从 model() 回调重读以保持一致。
class HoleStockSimulationDialog final : public QDialog {
public:
    // 构造函数:以 initial 值快照初始化控件,并调用 refreshModel(true)。
    HoleStockSimulationDialog(
        QWidget* parent,
        StockSourceSettings initial,
        QString browseDirectory,
        HoleStockDialogCallbacks callbacks);

private:
    // 从控件收集毛坯来源设置(模式/余量/文件路径)。
    StockSourceSettings sourceSettings() const;
    // 从控件收集当前显示选择(工序快照/视图/过切归因)。
    HoleStockDisplaySelection displaySelection() const;
    // 从 model() 回调重建控件范围与选项;preferOvercut 时优先显示过切归因。
    void refreshModel(bool preferOvercut);
    // 按当前选择刷新摘要文本并回调 present 更新 3D 显示。
    void refreshSelection();
    // 调用 run 回调执行后台仿真,失败/关闭时给出反馈。
    void runSimulation();
    // 调用 clear 回调清除仿真显示并复位界面。
    void clearSimulation();

    HoleStockDialogCallbacks m_callbacks;
    QString m_browseDirectory;
    QComboBox* m_source = nullptr;
    QWidget* m_stockFileRow = nullptr;
    QLineEdit* m_stockFile = nullptr;
    QDoubleSpinBox* m_marginX = nullptr;
    QDoubleSpinBox* m_marginY = nullptr;
    QDoubleSpinBox* m_marginZ = nullptr;
    QSlider* m_stepSlider = nullptr;
    QComboBox* m_resultView = nullptr;
    QComboBox* m_overcutOperation = nullptr;
    QLabel* m_summary = nullptr;
    QPushButton* m_runButton = nullptr;
    QPushButton* m_openGlButton = nullptr;
};

// 模态查看统一 IPW 工序/移动时间线;返回是否继续 OpenGL 刀具动画。
// 工序端点直接回看不可变检查点,工序内部移动由计时器节流重放。
bool showIpwTimelineDialog(
    QWidget* parent,
    const std::vector<IpwTimelineOperation>& operations,
    const std::function<void(int operationIndex,
                             int completedMoveCount)>& displayFrame,
    const std::function<void(
        const QString& formalOperationId)>& operationSelected = {},
    bool allowOpenGl = true);

} // namespace oap::app

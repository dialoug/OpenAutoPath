// 模块归属：app/workflows/simulation（oap_app_workflow_simulation）毛坯/IPW 仿真工作流。
// 与毛坯仿真对话框交换的与 UI 无关的值与回调。
#pragma once

#include <functional>
#include <vector>

#include <QString>

#include "app/workflows/simulation/StockSimulationWorkflowController.h"

namespace oap::app {

// 装夹面下拉选项：setupIndex 为装夹面序号，label 为显示文本。
struct PocketSimulationSetupChoice {
    int setupIndex = -1;
    QString label;
};

// 腔仿真对话框状态：来源、装夹面选项、显示质量/缓存与浏览目录。
struct PocketStockSimulationDialogState {
    QString scopeDescription;
    StockSourceSettings source;
    std::vector<PocketSimulationSetupChoice> setups;
    int selectedSetup = -1;
    int displayQuality = 10;
    bool triangulationEnabled = true;
    int displayCacheMiB = 384;
    QString browseDirectory;
};

// 孔仿真对话框模型：是否有仿真、快照步数与过切选项。
struct HoleStockDialogModel {
    bool hasSimulation = false;
    int stepCount = 0;
    std::vector<HoleStockOvercutChoice> overcutChoices;
};

// 孔仿真执行结果：是否成功/是否关闭请求、错误与状态文本。
struct HoleStockRunOutcome {
    bool ok = false;
    bool closeRequested = false;
    QString error;
    QString status;
};

// 孔仿真对话框与用例之间的回调集。
struct HoleStockDialogCallbacks {
    std::function<HoleStockDialogModel()> model;
    std::function<QString(const HoleStockDisplaySelection&)> describe;
    std::function<void(const HoleStockDisplaySelection&)> present;
    std::function<HoleStockRunOutcome(const StockSourceSettings&)> run;
    std::function<void()> openGl;
    std::function<void()> clear;
    std::function<void(const QString&)> publishStatus;
};

// IPW 时间线工序项（IPW 时间线对话框用例）。
struct IpwTimelineOperation {
    int operationIndex = -1;
    int moveCount = 0;
    QString formalOperationId;
    QString label;
    QString status;
    bool replayAvailable = false;
};

// 逐特征余量视图行：特征/工序标识、目标去除与剩余余量体积、完成比。
struct FeatureAllowanceViewItem {
    int resultIndex = -1;
    bool unifiedIpw = false;
    QString featureId;
    QString operationId;
    QString featureKind;
    QString displayName;
    double targetRemovalVolume = 0.0;
    double remainingAllowanceVolume = 0.0;
    double completionRatio = 0.0;
    bool approximateBoundary = false;
};

} // namespace oap::app

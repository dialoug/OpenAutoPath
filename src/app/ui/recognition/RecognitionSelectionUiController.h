// 模型/识别/选择 dock 族与面板回调边界（oap_app_recognition_ui）。
// 拥有模型/特征/分组面板，把选择与分析命令经回调端口转交
// RecognitionWorkspaceCoordinator，不持有识别会话状态。
#pragma once

#include <functional>

#include <QObject>
#include <QPoint>
#include <QString>

class QDockWidget;
class QMainWindow;
class ResultPanel;
class FeaturePanel;
class GroupPanel;

namespace oap::app {

// 识别选择 UI 回调端口：把选择与分析命令转交应用层。
struct RecognitionSelectionUiCallbacks {
    std::function<void(const QString&)> faceSelected;
    std::function<void()> featureSelectionChanged;
    std::function<void()> groupSelectionChanged;
    std::function<void(const QPoint&)> contextMenuRequested;
};

// Owns the model/recognition dock family and translates panel signals into
// shell-level commands. Recognition algorithms and scene state stay outside.
class RecognitionSelectionUiController final : public QObject {
public:
    RecognitionSelectionUiController(
        QMainWindow& window, RecognitionSelectionUiCallbacks callbacks);

    ResultPanel& resultPanel() const noexcept;
    FeaturePanel& featurePanel() const noexcept;
    GroupPanel& groupPanel() const noexcept;

    QDockWidget& analysisDock() const noexcept;
    QDockWidget& featureDock() const noexcept;
    QDockWidget& groupDock() const noexcept;

private:
    ResultPanel* m_resultPanel = nullptr;
    FeaturePanel* m_featurePanel = nullptr;
    GroupPanel* m_groupPanel = nullptr;
    QDockWidget* m_analysisDock = nullptr;
    QDockWidget* m_featureDock = nullptr;
    QDockWidget* m_groupDock = nullptr;
};

} // namespace oap::app

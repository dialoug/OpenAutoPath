// CAM 工作流 dock 与命令回调边界（oap_app_cam_ui）。
// 拥有 CAM 工作流面板、操作树、覆盖审计与特征余量视图的创建与信号路由，
// 通过 CamUiCallbacks 端口把用户命令转交给应用层，不持有 CAM 状态。
#pragma once

#include <functional>

#include <QObject>
#include <QString>

class HoleCoverageAuditPanel;
class HoleToolTreePanel;
class FeatureAllowancePanel;
class QDockWidget;
class QMainWindow;

namespace oap::app {
class CamWorkflowPanel;

// CAM UI 回调端口：把用户命令转交应用层。
struct CamUiCallbacks {
    std::function<void()> openPart;
    std::function<void()> openProject;
    std::function<void()> saveProject;
    std::function<void()> recognizeFeatures;
    std::function<void()> configureSetup;
    std::function<void()> selectTools;
    std::function<void()> manageTools;
    std::function<void()> generateHole;
    std::function<void()> generatePocket;
    std::function<void()> generateProfile;
    std::function<void()> generateFinishing;
    std::function<void()> simulateProject;
    std::function<void()> simulateHole;
    std::function<void()> simulatePocket;
    std::function<void()> simulateProfile;
    std::function<void()> simulateEngraving;
    std::function<void()> showAllowance;
    std::function<void()> exportProgram;
    std::function<void(int)> toolBatchActivated;
    std::function<void(int)> operationActivated;
    std::function<void(const QString&, bool)> operationVisibilityChanged;
    std::function<void(const QString&)> operationSoloRequested;
    std::function<void(int, int, bool)> operationVisibilitySummaryChanged;
    std::function<void()> editHoleParameters;
    std::function<void()> editPocketOperation;
    std::function<void()> regeneratePocket;
    std::function<void()> editProfileOperation;
    std::function<void()> regenerateProfile;
    std::function<void(const QString&)> editFinishingOperation;
    std::function<void(const QString&)> regenerateFinishing;
    std::function<void(int)> coverageSourceSelected;
    std::function<void(const QString&)> formalOperationSelected;
    std::function<void(const QString&)> previewOperation;
};

// Owns the CAM workbench dock and routes widget intent through a callback port.
class CamUiController final : public QObject {
public:
    CamUiController(QMainWindow& window, QDockWidget& inputDock,
                    CamUiCallbacks callbacks);

    CamWorkflowPanel& panel() const noexcept;
    HoleToolTreePanel& operationTree() const noexcept;
    HoleCoverageAuditPanel& coverageAudit() const noexcept;
    FeatureAllowancePanel& featureAllowance() const noexcept;
    QDockWidget& dock() const noexcept;

private:
    CamWorkflowPanel* m_panel = nullptr;
    HoleToolTreePanel* m_operationTree = nullptr;
    HoleCoverageAuditPanel* m_coverageAudit = nullptr;
    FeatureAllowancePanel* m_featureAllowance = nullptr;
    QDockWidget* m_dock = nullptr;
};

} // namespace oap::app

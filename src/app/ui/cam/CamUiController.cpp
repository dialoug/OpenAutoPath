// CamUiController 实现（oap_app_cam_ui）。
// CAM 工作流 dock 与命令回调接线，见 CamUiController.h。
#include "app/ui/cam/CamUiController.h"

#include "app/ui/panels/CamWorkflowPanel.h"
#include "app/ui/panels/FeatureAllowancePanel.h"
#include "app/ui/panels/HoleCoverageAuditPanel.h"
#include "app/ui/panels/HoleToolTreePanel.h"

#include <QDockWidget>
#include <QMainWindow>
#include <utility>

namespace oap::app {
namespace {
template <class Sender, class Signal, class Callback>
void connectCommand(Sender* sender, Signal signal, QObject* context,
                    Callback callback)
{
    QObject::connect(sender, signal, context,
                     [callback = std::move(callback)] {
                         if (callback) callback();
                     });
}
}

CamUiController::CamUiController(
    QMainWindow& window, QDockWidget& inputDock, CamUiCallbacks callbacks)
    : QObject(&window)
{
    m_panel = new CamWorkflowPanel(&window);
    m_dock = new QDockWidget(QStringLiteral("CAM 工作流"), &window);
    m_dock->setObjectName(QStringLiteral("camWorkflowDock"));
    m_dock->setWidget(m_panel);
    m_dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    m_dock->setMinimumWidth(300);
    window.addDockWidget(Qt::LeftDockWidgetArea, m_dock);
    window.tabifyDockWidget(&inputDock, m_dock);
    window.resizeDocks({&inputDock}, {330}, Qt::Horizontal);
    m_dock->raise();

    m_operationTree = m_panel->operationTree();
    m_coverageAudit = m_panel->coverageAudit();
    m_featureAllowance = m_panel->featureAllowance();

    connectCommand(m_panel, &CamWorkflowPanel::openPartRequested, this, callbacks.openPart);
    connectCommand(m_panel, &CamWorkflowPanel::openProjectRequested, this, callbacks.openProject);
    connectCommand(m_panel, &CamWorkflowPanel::saveProjectRequested, this, callbacks.saveProject);
    connectCommand(m_panel, &CamWorkflowPanel::recognizeFeaturesRequested, this, callbacks.recognizeFeatures);
    connectCommand(m_panel, &CamWorkflowPanel::configureSetupRequested, this, callbacks.configureSetup);
    connectCommand(m_panel, &CamWorkflowPanel::selectToolsRequested, this, callbacks.selectTools);
    connectCommand(m_panel, &CamWorkflowPanel::manageToolsRequested, this, callbacks.manageTools);
    connectCommand(m_panel, &CamWorkflowPanel::generateHoleRequested, this, callbacks.generateHole);
    connectCommand(m_panel, &CamWorkflowPanel::generatePocketRequested, this, callbacks.generatePocket);
    connectCommand(m_panel, &CamWorkflowPanel::generateProfileRequested, this, callbacks.generateProfile);
    connectCommand(m_panel, &CamWorkflowPanel::generateFinishingRequested, this, callbacks.generateFinishing);
    connectCommand(m_panel, &CamWorkflowPanel::simulateProjectRequested, this, callbacks.simulateProject);
    connectCommand(m_panel, &CamWorkflowPanel::simulateHoleRequested, this, callbacks.simulateHole);
    connectCommand(m_panel, &CamWorkflowPanel::simulatePocketRequested, this, callbacks.simulatePocket);
    connectCommand(m_panel, &CamWorkflowPanel::simulateProfileRequested, this, callbacks.simulateProfile);
    connectCommand(m_panel, &CamWorkflowPanel::simulateEngravingRequested, this, callbacks.simulateEngraving);
    connectCommand(m_panel, &CamWorkflowPanel::showAllowanceRequested, this, callbacks.showAllowance);
    connectCommand(m_panel, &CamWorkflowPanel::exportProgramRequested, this, callbacks.exportProgram);

    connect(m_operationTree, &HoleToolTreePanel::toolBatchActivated, this,
            [callback = std::move(callbacks.toolBatchActivated)](int index) {
                if (callback) callback(index);
            });
    connect(m_operationTree, &HoleToolTreePanel::operationActivated, this,
            [callback = std::move(callbacks.operationActivated)](
                const QString&, int startMoveIndex) {
                if (callback) callback(startMoveIndex);
            });
    connect(m_operationTree, &HoleToolTreePanel::operationVisibilityChanged,
            this, [callback = std::move(callbacks.operationVisibilityChanged)](
                      const QString& id, bool visible) {
                if (callback) callback(id, visible);
            });
    connect(m_operationTree, &HoleToolTreePanel::operationSoloRequested, this,
            [callback = std::move(callbacks.operationSoloRequested)](
                const QString& id) { if (callback) callback(id); });
    connect(m_operationTree,
            &HoleToolTreePanel::operationVisibilitySummaryChanged, this,
            [callback = std::move(callbacks.operationVisibilitySummaryChanged)](
                int visible, int total, bool solo) {
                if (callback) callback(visible, total, solo);
            });
    connectCommand(m_operationTree, &HoleToolTreePanel::editHoleParametersRequested,
                   this, callbacks.editHoleParameters);
    connectCommand(m_operationTree,
                   &HoleToolTreePanel::editPocketOperationRequested, this,
                   callbacks.editPocketOperation);
    connectCommand(m_operationTree,
                   &HoleToolTreePanel::regeneratePocketRequested, this,
                   callbacks.regeneratePocket);
    connectCommand(m_operationTree,
                   &HoleToolTreePanel::editProfileOperationRequested, this,
                   callbacks.editProfileOperation);
    connectCommand(m_operationTree,
                   &HoleToolTreePanel::regenerateProfileRequested, this,
                   callbacks.regenerateProfile);
    connect(m_operationTree, &HoleToolTreePanel::editFinishingOperationRequested, this,
            [callback = std::move(callbacks.editFinishingOperation)](const QString& id) {
                if (callback) callback(id);
            });
    connect(m_operationTree, &HoleToolTreePanel::regenerateFinishingRequested, this,
            [callback = std::move(callbacks.regenerateFinishing)](const QString& id) {
                if (callback) callback(id);
            });
    connectCommand(m_operationTree, &HoleToolTreePanel::regenerateHoleRequested,
                   this, callbacks.generateHole);
    connect(m_operationTree, &HoleToolTreePanel::previewOperationRequested, this,
            [callback = std::move(callbacks.previewOperation)](const QString& id) {
                if (callback) callback(id);
            });
    connect(m_coverageAudit, &HoleCoverageAuditPanel::sourceHoleSelected, this,
            [callback = std::move(callbacks.coverageSourceSelected)](int index) {
                if (callback) callback(index);
            });
    const auto publishFormalSelection =
        [callback = std::move(callbacks.formalOperationSelected)](
            const QString& operationId) {
            if (callback) callback(operationId);
        };
    connect(m_operationTree,
            &HoleToolTreePanel::formalOperationSelected,
            this, publishFormalSelection);
    connect(m_coverageAudit,
            &HoleCoverageAuditPanel::formalOperationSelected,
            this, publishFormalSelection);
}

CamWorkflowPanel& CamUiController::panel() const noexcept { return *m_panel; }
HoleToolTreePanel& CamUiController::operationTree() const noexcept { return *m_operationTree; }
HoleCoverageAuditPanel& CamUiController::coverageAudit() const noexcept { return *m_coverageAudit; }
FeatureAllowancePanel& CamUiController::featureAllowance() const noexcept { return *m_featureAllowance; }
QDockWidget& CamUiController::dock() const noexcept { return *m_dock; }

} // namespace oap::app

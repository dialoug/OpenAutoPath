// 仿真结果选择到呈现的绑定（oap_app_simulation_ui）。
// 把仿真结果选择绑定到呈现，不让 CAM 面板感知仿真状态或场景变换。
#pragma once

#include <functional>

#include <QObject>
#include <gp_Trsf.hxx>

class FeatureAllowancePanel;

namespace oap::app {
class StockSimulationWorkspaceCoordinator;

// Binds simulation result selection to presentation without making the CAM
// panel know about simulation state or the scene transform.
class SimulationUiController final : public QObject {
public:
    SimulationUiController(
        FeatureAllowancePanel& panel,
        StockSimulationWorkspaceCoordinator& workspace,
        std::function<gp_Trsf()> modelTransform,
        QObject* parent);
};

} // namespace oap::app

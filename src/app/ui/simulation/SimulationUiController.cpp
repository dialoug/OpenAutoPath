// SimulationUiController 实现（oap_app_simulation_ui）。
// 仿真结果选择到呈现的绑定，见 SimulationUiController.h。
#include "app/ui/simulation/SimulationUiController.h"

#include "app/simulation/StockSimulationWorkspaceCoordinator.h"
#include "app/ui/panels/FeatureAllowancePanel.h"

#include <cstddef>
#include <utility>

namespace oap::app {

SimulationUiController::SimulationUiController(
    FeatureAllowancePanel& panel,
    StockSimulationWorkspaceCoordinator& workspace,
    std::function<gp_Trsf()> modelTransform,
    QObject* parent)
    : QObject(parent)
{
    connect(&panel, &FeatureAllowancePanel::allowanceSelected, this,
            [&workspace, transform = std::move(modelTransform)](int row) {
                const auto items = workspace.featureAllowanceItems();
                if (row < 0 || row >= static_cast<int>(items.size()))
                    return;
                workspace.displayFeatureAllowance(
                    items[static_cast<std::size_t>(row)], transform());
            });
}

} // namespace oap::app

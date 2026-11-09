// MainWindowUiBuilder 实现（OpenAutoPathApp）。
// MainWindow 界面骨架的集中构造流程，见 MainWindowUiBuilder.h。
#include "app/ui/shell/MainWindowUiBuilder.h"

#include "app/ui/shell/MainWindow.h"

namespace oap::app {

void MainWindowUiBuilder::build(MainWindow& window)
{
    window.setupMenus();
    window.setupToolbars();
    window.setupMouseBindings();
    window.setupDocks();
    window.connectSignals();
}

} // namespace oap::app

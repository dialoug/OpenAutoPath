// MainWindow 界面骨架的集中构造流程。
#pragma once

class MainWindow;

namespace oap::app {

// 主窗口 UI 构建器：集中构造界面骨架与 chrome 顺序。
class MainWindowUiBuilder {
public:
    // 按固定顺序调用 MainWindow 的私有 setup* 方法:菜单→工具栏→鼠标绑定→
    // 停靠面板→信号连接,把“建界面”与“接业务”分成两个阶段。
    static void build(MainWindow& window);
};

} // namespace oap::app

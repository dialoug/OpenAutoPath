// 面向与 UI 无关的工作流交互端口的 Qt Widgets 适配器。
// makeQtWorkflowInteraction 把 WorkflowInteraction 的进度/忙碌回调实现为
// 模态 QProgressDialog + 等待光标,供后台任务在 UI 线程展示进度。
#pragma once

#include "app/workflows/shared/WorkflowInteraction.h"

class QWidget;

namespace oap::app {

// 构造一个以 parent 为父窗口的 Qt 实现;父窗口负责保证调用发生在 GUI 线程。
// showProgress 自行持有忙碌状态;可与外层 setBusy 嵌套,只在最外层进入/退出时通知。
// externalCancellation 用于把窗口关闭等生命周期取消合并到每个进度请求。
WorkflowInteraction makeQtWorkflowInteraction(
    QWidget* parent,
    WorkflowBusyStateHandler onBusy = {},
    std::function<bool()> externalCancellation = {});

} // namespace oap::app

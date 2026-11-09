// 桌面工作流运行期间使用的与 UI 无关的交互端口。
#pragma once

#include <cstddef>
#include <functional>

#include <QString>

namespace oap::app {

// 一次进度上报:已完成数/总数与当前标签文本。
struct WorkflowProgressSnapshot {
    std::size_t completed = 0;
    std::size_t total = 0;
    QString label;
};

// 进度任务描述:标题/标签文本、总量,以及若干回调——
// snapshot() 取当前进度,ready() 判断任务是否结束,
// cancel() 请求取消,cancellationRequested() 查询取消状态。
struct WorkflowProgressRequest {
    QString title;
    QString label;
    QString cancellingLabel;
    std::size_t initialTotal = 0;
    bool indeterminate = false;
    std::function<WorkflowProgressSnapshot()> snapshot;
    std::function<bool()> ready;
    std::function<void()> cancel;
    std::function<bool()> cancellationRequested;
    // Waiting for the shared heavy-job slot; no geometry work has started.
    std::function<bool()> queued;
};

// 进度呈现器(显示进度对话框)与忙状态处理器的类型别名,
// 由界面层注入,工作流层只通过这两个端口与用户交互。
using WorkflowProgressPresenter =
    std::function<void(const WorkflowProgressRequest&)>;
using WorkflowBusyStateHandler = std::function<void(bool busy)>;

// 工作流与界面之间的交互端口:可整体被替换为无界面桩。
struct WorkflowInteraction {
    WorkflowProgressPresenter showProgress;
    WorkflowBusyStateHandler setBusy;
};

// 呈现器应阻塞直到 ready() 返回 true。无界面回退实现不驱动 GUI 事件循环，
// 从而保证工作流在测试和非 Widgets 客户端中依然可用。
void waitForWorkflowTask(
    const WorkflowProgressPresenter& presenter,
    const WorkflowProgressRequest& request);

} // namespace oap::app

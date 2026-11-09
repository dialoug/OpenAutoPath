// WorkflowInteraction 实现（oap_app_workflow_base）。
// 桌面工作流运行期间使用的与 UI 无关的交互端口，见 WorkflowInteraction.h。
#include "app/workflows/shared/WorkflowInteraction.h"

#include <chrono>
#include <thread>

namespace oap::app {

// 无界面等待循环:交给呈现器(阻塞式)驱动后轮询 ready(),
// 每 10ms 检查一次;ready 回调缺失时直接返回,避免死等。
void waitForWorkflowTask(
    const WorkflowProgressPresenter& presenter,
    const WorkflowProgressRequest& request)
{
    if (!request.ready)
        return;
    if (presenter)
        presenter(request);
    while (!request.ready())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

} // namespace oap::app

// 统一 IPW 桌面运行器。
// 模块归属：app/workflows/simulation（oap_app_workflow_simulation）毛坯/IPW 仿真工作流。
// 无 UI 的可测试入口与交互式进度入口；由 StockSimulationUseCases 调用。
// 执行期间只修改 Job 的运行期 IPW 会话和检查点缓存；
// `.oapjob` 定义及刀路输入保持不变（运行期与定义分离，结果可复用/续跑）。
//
// Job 定义和运行期刀路是输入；Multi-Dexel 检查点是材料真值。用于显示的
// BRep 快照、余量/过切和逐工序归因均从同一检查点链派生。
#pragma once

#include <atomic>
#include <array>
#include <memory>
#include <optional>
#include <string>

#include <TopoDS_Shape.hxx>

#include "app/workflows/shared/WorkflowInteraction.h"
#include "manufacturing/ipw/IpwExecutor.h"
#include "manufacturing/job/MachiningJob.h"
#include "toolpath/core/simulation/StockComparison.h"
#include "toolpath/core/simulation/StockSimulationTypes.h"

namespace oap::app {

// 一次统一 IPW 仿真请求：初始毛坯/目标实体与执行、比较选项。
struct IpwSimulationRequest {
    TopoDS_Shape initialStock;
    TopoDS_Shape targetShape;
    std::string initialStockFingerprint;
    manufacturing::ipw::IpwExecutionOptions options;
    toolpath::StockComparisonOptions comparisonOptions;
    std::optional<std::array<double, 6>> exactTargetBounds;
};

// 运行结果：执行摘要、从检查点派生的时间线/比较/过切归因与错误信息。
struct IpwSimulationRunResult {
    manufacturing::ipw::IpwExecutionResult execution;
    // 兼容现有工序滑块和 AIS 显示的数据视图。该对象只汇总检查点，
    // 不再次执行一条独立的减料仿真。
    toolpath::StockSimulationResult simulation;
    toolpath::StockComparisonResult comparison;
    toolpath::StockOvercutAttributionResult overcutAttribution;
    std::string errorMessage;
    bool cancelled = false;
    bool failed = false;
};

// 无 UI 的可测试入口：同步执行完整 IPW 并派生审计视图。
// control 提供取消回调；request.targetShape 为空时跳过比较与归因。
IpwSimulationRunResult runIpwSimulation(
    manufacturing::MachiningJob& job,
    IpwSimulationRequest request,
    const toolpath::ToolpathGenerationControl* control = nullptr);

// 交互式进度入口：后台线程执行，进度经 progress 端口展示并支持取消。
// 执行期间只修改 Job 的运行期 IPW 会话和检查点缓存；
// `.oapjob` 定义及刀路输入保持不变。
// Exclusively borrows job for the entire synchronous call, including queueing
// and exceptional cleanup. The caller must not mutate or destroy job meanwhile.
IpwSimulationRunResult simulateIpwWithProgress(
    const WorkflowProgressPresenter& progress,
    manufacturing::MachiningJob& job,
    IpwSimulationRequest request,
    std::shared_ptr<std::atomic_bool> cancellation = {});

} // namespace oap::app

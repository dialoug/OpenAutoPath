// 孔在制毛坯运行器：后台加载可选 STEP 毛坯并执行顺序减料，通过交互端口
// 轮询原子进度和处理取消，不在工作线程访问任何 UI/AIS 对象。
#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>

#include "app/workflows/pocket/PocketCamSimulatorTypes.h"
#include "app/workflows/shared/WorkflowInteraction.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/core/simulation/StockComparison.h"
#include "toolpath/hole/HoleMachining.h"
#include "toolpath/hole/HoleStockSimulator.h"

namespace oap::app {

// 孔在制毛坯仿真请求：把初始毛坯/目标体与孔工序打包，
// 交给后台线程执行顺序减料。
struct HoleStockSimulationRequest {
    // stockStepPath 非空时优先后台加载；否则使用调用方构造的 initialStock。
    TopoDS_Shape initialStock;
    TopoDS_Shape targetShape;
    std::string stockStepPath;
    std::vector<toolpath::HoleMachiningOperation> operations;
    toolpath::ToolpathParams fallbackParams;
    toolpath::HoleStockSimulationOptions options;
};

// 仿真运行结果：含顺序减料结果、毛坯比较（缺料/过切）与过切归因，
// cancelled 表示被用户取消。
struct HoleStockSimulationRunResult {
    toolpath::HoleStockSimulationResult simulation;
    toolpath::StockComparisonResult comparison;
    toolpath::StockOvercutAttributionResult overcutAttribution;
    std::string errorMessage;
    bool cancelled = false;
    bool importedStep = false;
};

// 带进度的孔毛坯仿真入口：可选后台加载 STEP 毛坯并执行顺序减料，
// 通过交互端口轮询进度/处理取消，全程不访问 UI/AIS 对象。
HoleStockSimulationRunResult simulateHoleStockWithProgress(
    const WorkflowProgressPresenter& progress,
    HoleStockSimulationRequest request,
    std::shared_ptr<std::atomic_bool> cancellation = {});

// 单个固定轴 Setup 的孔 OpenGL 场景准备请求。initialStock/finalStock 是
// 该 Setup 加工前后的权威孔 IPW BRep 快照；后台把它们栅格化到同一网格，
// OpenGL 只负责重放显示，不重新解释 NC 或验证结果。
struct HoleOpenGlSimulationRequest {
    TopoDS_Shape initialStock;
    TopoDS_Shape finalStock;
    TopoDS_Shape targetShape;
    std::vector<toolpath::HoleMachiningOperation> operations;
    toolpath::ToolpathParams fallbackParams;
    int displayQuality = 10;
};

// 孔 OpenGL 仿真结果：毛坯减刀具扫掠后的显示状态。
struct HoleOpenGlSimulationResult {
    PocketCamSimulatorInput input;
    std::string error;
    std::string warning;
    double gridResolution = 0.0;
    bool cancelled = false;

    bool ok() const noexcept
    {
        return !cancelled && error.empty() &&
               static_cast<bool>(input.initialIpwStock) &&
               !input.operations.empty();
    }
};

// 在后台适配孔工序并构建 GPU Multi-Dexel 初始/最终材料状态。
HoleOpenGlSimulationResult prepareHoleOpenGlWithProgress(
    const WorkflowProgressPresenter& progress,
    HoleOpenGlSimulationRequest request);

} // namespace oap::app

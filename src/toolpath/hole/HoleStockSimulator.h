// 固定刀轴孔操作的在制毛坯仿真。输入必须是加工前毛坯，而不是已经包含
// 目标孔的成品模型；输出按工序保存布尔减料后的状态快照和体积记录。
// 属于 oap_toolpath_hole 模块；复用 oap_toolpath_core 的 InProcessStock 仿真。
#pragma once

#include <vector>

#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/hole/HoleMachining.h"
#include "toolpath/core/simulation/StockSimulationTypes.h"

namespace oap::toolpath {

using HoleStockSimulationOptions = StockSimulationOptions;
using HoleStockSimulationResult = StockSimulationResult;

// 孔操作毛坯仿真器：把孔工序依次减料，返回逐工序状态快照与统计。
// 无状态值对象，可安全复用；fallbackParams 仅在工序未绑定刀具时提供参数。
class HoleStockSimulator {
public:
    // 对 initialStock 依序执行 operations 的去料仿真；
    // 返回结果含每个工序的毛坯快照、布尔耗时与成败诊断。
    HoleStockSimulationResult simulate(
        const TopoDS_Shape& initialStock,
        const std::vector<HoleMachiningOperation>& operations,
        const ToolpathParams& fallbackParams,
        const HoleStockSimulationOptions& options = {},
        const ToolpathGenerationControl* control = nullptr) const;
};

} // namespace oap::toolpath

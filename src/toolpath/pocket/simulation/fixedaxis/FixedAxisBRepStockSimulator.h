// 工序级固定轴 BRep 毛坯仿真。
//
// 位于 oap_toolpath_pocket 的 fixedaxis/ 模块私有实现目录（内部细节）：
// 按工序把刀路增量构建为去除 BRep，再以独立实体布尔减应用到 InProcessStock；
// 汇报每工序进度与统计指标。与此配套的 preflight / layered builder /
// analytic 解析快路径只在该目录内相互协作。
#pragma once

#include <cstddef>
#include <vector>

#include <TopoDS_Shape.hxx>

#include "toolpath/core/ToolpathExecution.h"
#include "toolpath/core/ToolpathParams.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/core/simulation/StockSimulationTypes.h"

namespace oap::toolpath {

// 工序级固定轴 BRep 毛坯仿真器：按工序布尔减并汇报进度。
class FixedAxisBRepStockSimulator {
public:
    static constexpr std::size_t PreparationProgressStepCount = 1;
    static constexpr std::size_t ProgressStepsPerOperation = 4;

    // 总进度步数 = 1 步预处理 + 每工序 4 步（构造减料体、校验、
    // 应用布尔、校验结果）。
    static constexpr std::size_t progressStepCount(
        std::size_t operationCount) noexcept
    {
        return PreparationProgressStepCount +
               ProgressStepsPerOperation * operationCount;
    }

    // 对 initialStock 依次应用全部工序的去除体积，返回含每工序
    // StockRemovalRecord、统计与诊断的仿真结果。要求所有工序刀轴一致；
    // options.stopOnFailure 控制遇到不可修复错误时是否提前中止。
    StockSimulationResult simulate(
        const TopoDS_Shape& initialStock,
        const std::vector<PocketToolpathOperation>& operations,
        const ToolpathParams& params,
        const StockSimulationOptions& options = {},
        const ToolpathGenerationControl* control = nullptr) const;
};

} // namespace oap::toolpath

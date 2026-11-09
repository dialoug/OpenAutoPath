// 孔 ToolpathProgram/Operation 的原生 Multi-Dexel 减料适配。
//
// 本文件属于 oap_manufacturing::ipw 模块(CMake target: oap_manufacturing),
// 是 IPW 执行服务的一部分。职责:把一组孔运行期工序按顺序直接应用到同一
// Multi-Dexel 毛坯上(不经过 BRep 过渡),产出不可变的更新后毛坯与逐工序
// 仿真明细。协作:IpwExecutor 在孔节点上调用本服务;底层复用
// toolpath::MultiDexelStock 的分块多区间引擎。UI 无关。
#pragma once

#include "toolpath/hole/HoleMachining.h"
#include "toolpath/pocket/simulation/MultiDexelStock.h"

#include <memory>
#include <string>
#include <vector>

namespace oap::manufacturing::ipw {
toolpath::DexelToolProfileDefinition holeDexelToolProfile(
    const toolpath::HoleMachiningOperation& operation,
    const toolpath::ToolpathParams& params);

// 一次孔减料更新的结果。stock 为更新后的不可变材料真值(取消或失败时
// 为空,不发布局部中间状态);simulation 为逐工序明细。
struct HoleMultiDexelUpdateResult {
    // 更新后的材料真值;以不可变共享指针共享,可与父检查点共用前缀。
    std::shared_ptr<const toolpath::MultiDexelStock> stock;
    // 逐工序仿真结果明细,含每步布尔运算/网格更新耗时与计数。
    std::shared_ptr<toolpath::StockSimulationResult> simulation;
    // 取消标记与错误消息;二者任一成立即视为本次更新失败。
    bool cancelled = false;
    std::string error;

    bool complete() const noexcept
    {
        return !cancelled && error.empty() && stock &&
            simulation && simulation->complete;
    }
};

// 把 operations 按给定顺序依次应用到 initial 毛坯(前一步输出作为下一步
// 输入),生成更新后的毛坯与仿真结果。前置条件:initial 有效且 operations
// 非空;control 用于逐工序取消检查。参数均为只读共享引用,不转移所有权;
// 调用方可保留 initial 供检查点链复用。
HoleMultiDexelUpdateResult applyHoleOperationsToMultiDexel(
    std::shared_ptr<const toolpath::MultiDexelStock> initial,
    const std::vector<toolpath::HoleMachiningOperation>& operations,
    const toolpath::ToolpathParams& params,
    const toolpath::StockSimulationOptions& options,
    const toolpath::ToolpathGenerationControl* control = nullptr);

} // namespace oap::manufacturing::ipw

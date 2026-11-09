// 把自动选刀批次逐批交给现有 HoleMachining。不同刀具/装夹面分别调用生成器，
// 因此不会产生跨换刀或跨装夹 Rapid。
// 模块归属:oap_toolpath_adapter(tooling/adapter)的自动孔刀路门面;
// 上承刀具库 ToolLibrary + 识别孔段 HoleMachiningInput,下接
// toolpath/workflow 的 HoleToolpathService/HoleBatchExecutor 生成刀路批次,
// 供 MachiningJob/工作流层安装为可执行的孔加工程序。
#pragma once

#include "tooling/adapter/hole/HoleToolSelectionPlanner.h"
#include "toolpath/workflow/hole/HoleToolpathService.h"

namespace oap::tooling {

// 自动生成结果:batch 为已完成的刀路批次(含操作、验证、覆盖审计、诊断),
// selection 为生成该批次所使用的选刀计划。
struct HoleAutomaticToolpathResult {
    oap::toolpath::HoleToolpathBatch batch;
    HoleToolSelectionPlan selection;
};

// 自动孔加工入口:先做自动选刀规划(HoleToolSelectionPlanner),再把每个
// 批次逐批交给孔刀路服务执行。
class HoleAutomaticToolpath {
public:
    // 完整自动流程:归一化参数 -> 自动选刀 -> generateFromPlan。
    // 前置条件:inputs 中 ready 的孔段由识别/映射层填好入口与装夹面;
    HoleAutomaticToolpathResult generate(
        const ToolLibrary& library,
        const std::vector<oap::toolpath::HoleMachiningInput>& inputs,
        const oap::toolpath::ToolpathParams& params,
        const oap::toolpath::HoleRapidPlanningContext* rapidContext = nullptr,
        const oap::toolpath::ToolpathGenerationControl* control = nullptr) const;

    // 执行已完成自动选刀的计划，统一完成 Operation ID 消歧、验证和覆盖审计。
    HoleAutomaticToolpathResult generateFromPlan(
        HoleToolSelectionPlan selection,
        const std::vector<oap::toolpath::HoleMachiningInput>& inputs,
        const oap::toolpath::ToolpathParams& params,
        const oap::toolpath::HoleRapidPlanningContext* rapidContext = nullptr,
        const oap::toolpath::ToolpathGenerationControl* control = nullptr) const;

};

} // namespace oap::tooling

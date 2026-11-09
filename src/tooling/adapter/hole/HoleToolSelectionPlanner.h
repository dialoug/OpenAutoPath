// 六面孔加工自动选刀规划。该模块只做“孔段 -> 工序/刀具批次”的确定性
// 调度，不生成几何移动；实际刀路仍由 HoleMachining 完成。
// 模块归属:oap_toolpath_adapter 的自动选刀核心;产物 HoleToolSelectionPlan
// 是自动批次编排的交换格式,由 HoleAutomaticToolpath 翻译为孔刀路执行请求。
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "tooling/library/ToolLibrary.h"
#include "toolpath/hole/HoleMachining.h"
#include "toolpath/hole/HoleMachiningInput.h"
#include "toolpath/core/ToolpathDiagnostics.h"

namespace oap::tooling {

// 单个孔段的选刀任务：加工输入、来源/物理孔索引与覆盖范围。
struct HoleToolTask {
    oap::toolpath::HoleMachiningInput hole;
    std::size_t sourceHoleIndex = 0;
    // 此任务完成后可被覆盖审计认定为已加工的全部识别孔段。
    // sourceHoleIndex 仍是操作定位/显示使用的主来源。
    std::vector<std::size_t> coveredSourceHoleIndices;
    // 所属物理孔(同一实体孔归并)的稳定编号,跨重规划保持。
    std::size_t physicalHoleIndex = 0;
    oap::toolpath::HoleMachiningStrategy strategy =
        oap::toolpath::HoleMachiningStrategy::Auto;
};

// 同装夹、同阶段、同刀具的一组孔任务批次。
struct HoleToolBatch {
    oap::MoldSetupFace setupFace = oap::MoldSetupFace::Unresolved;
    int setupIndex = 0;
    oap::toolpath::HoleMachiningPhase phase =
        oap::toolpath::HoleMachiningPhase::Standard;
    oap::toolpath::ToolpathToolContext tool;
    std::vector<HoleToolTask> tasks;
    oap::toolpath::HoleVisitOrderMode visitOrderMode =
        oap::toolpath::HoleVisitOrderMode::Optimize;
    oap::toolpath::HoleRapidLinkMode rapidLinkMode =
        oap::toolpath::HoleRapidLinkMode::DirectWhenSafe;
};

// 一次完整选刀的产出:按装夹面组织的有序批次列表 + 诊断 + 跳过统计。
// 由 HoleToolSelectionPlanner 创建，或从持久化自动工序重建。
struct HoleToolSelectionPlan {
    std::vector<HoleToolBatch> batches;
    std::vector<oap::toolpath::ToolpathDiagnostic> diagnostics;
    int physicalHoleCount = 0;
    int skippedPhysicalHoleCount = 0;
    std::vector<std::size_t> skippedPhysicalHoleIndices;
    int setupCount = 0;

    bool empty() const noexcept { return batches.empty(); }
};

// 自动选刀规划器(确定性、无副作用):按固定六面顺序逐面规划,
// 同一装夹面内依次生成中心定位/钻削/平底铣三个阶段批次。
class HoleToolSelectionPlanner {
public:
    // 输入活动刀库、全部识别孔段和加工参数,输出批次计划与诊断。
    // 未解析入口/装夹的孔段不进入批次,仅以诊断标注原因。
    HoleToolSelectionPlan plan(
        const ToolLibrary& library,
        const std::vector<oap::toolpath::HoleMachiningInput>& inputs,
        const oap::toolpath::ToolpathParams& params) const;

};

} // namespace oap::tooling

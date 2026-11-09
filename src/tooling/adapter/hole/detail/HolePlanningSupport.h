// 自动孔选刀各阶段共享的策略判定和诊断工具。
// 属于 hole 模块 detail 私有实现;供 CenterSpotPlanner/DrillBatchPlanner/
// FlatEndMillPlanner 等阶段规划器复用,不对外暴露。
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "tooling/adapter/hole/HoleToolSelectionPlanner.h"

namespace oap::tooling::detail {

// 按孔径与刀径比自动推导加工策略:等径->点钻,否则按中/大径比选螺旋或分层。
oap::toolpath::HoleMachiningStrategy automaticHoleStrategy(
    const oap::toolpath::HoleMachiningInput& hole,
    double toolDiameter,
    const oap::toolpath::HoleMachiningParams& params,
    bool forceBlindCenterline);


// 向计划追加一条选刀诊断(按孔段定位或全局)。
void appendHoleToolDiagnostic(
    HoleToolSelectionPlan& result,
    oap::toolpath::ToolpathDiagnosticCode code,
    oap::toolpath::ToolpathDiagnosticSeverity severity,
    std::string message,
    int sourceIndex = -1);

// 追加一条 Info 级“刀具分配”诊断,描述装夹/阶段/刀号/任务数。
void appendHoleAssignmentDiagnostic(
    HoleToolSelectionPlan& result,
    const HoleToolBatch& batch);

// 把物理孔标记为跳过(去重),并同步 skippedPhysicalHoleCount。
void markPhysicalHoleSkipped(
    HoleToolSelectionPlan& result,
    std::size_t physicalHoleIndex);

} // namespace oap::tooling::detail

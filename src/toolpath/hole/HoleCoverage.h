// 孔加工覆盖审计：验证每个识别孔段都进入了实际去料工序，并汇总自动选刀、
// 装夹解析和生成阶段的遗漏，供 Reference-NC 导出前复核。
// 属于 oap_toolpath_hole 模块；消费孔输入、生成结果与诊断，不持有资源。
#pragma once

#include <string>
#include <vector>

#include "toolpath/core/ToolpathDiagnostics.h"
#include "toolpath/hole/HoleMachining.h"
#include "toolpath/hole/HoleMachiningInput.h"

namespace oap::toolpath {

// 孔覆盖问题：诊断码、来源孔段索引与说明消息。
struct HoleCoverageIssue {
    ToolpathDiagnosticCode code = ToolpathDiagnosticCode::None;
    int sourceHoleIndex = -1;   // 关联的源孔段下标；-1 表示工序级问题。
    std::string message;
};

// 覆盖审计报告：孔段计数、遗漏/排除列表以及阻断/警告问题分类。
struct HoleCoverageReport {
    bool audited = false;
    int inputHoleSegmentCount = 0;
    int sixFaceHoleSegmentCount = 0;
    int physicalHoleCount = 0;
    int coveredHoleSegmentCount = 0;
    int skippedOperationCount = 0;
    // 当前产品范围只加工六个模具装夹面。六面外孔保留在识别总数和警告中，
    // 但不进入覆盖分母；导出参考 NC 时作为复核警告展示。
    std::vector<int> outsideSixFaceSourceHoleIndices;
    // 人工从 CAM 计划中明确删除的孔段；保留识别记录，但不视为漏加工。
    std::vector<int> excludedSourceHoleIndices;
    std::vector<int> uncoveredSourceHoleIndices;   // 六面内未覆盖孔段。
    std::vector<int> unresolvedSourceHoleIndices;  // 入口未解析的孔段。
    std::vector<HoleCoverageIssue> blockingIssues; // 阻断导出的问题。
    std::vector<HoleCoverageIssue> warningIssues;  // 仅供复核的警告。

    // 是否完全覆盖：已审计、六面内孔全部覆盖且无阻断问题。
    bool complete() const noexcept;
    // 生成一行可读摘要（覆盖数/总数与各类遗漏计数）。
    std::string summary() const;
};

// 只有包含实际切削移动的非点钻工序才代表孔段材料已经去除。
bool holeOperationRemovesMaterial(
    const HoleMachiningOperation& operation) noexcept;

// 执行覆盖审计：比对孔段与去料工序，按六面范围、人工排除和诊断严重度
// 汇总报告；排除列表可选项用于跳过人工从 CAM 计划中删除的孔段。
HoleCoverageReport auditHoleCoverage(
    const std::vector<HoleMachiningInput>& holes,
    const std::vector<HoleMachiningOperation>& operations,
    int physicalHoleCount,
    int skippedOperationCount,
    const std::vector<ToolpathDiagnostic>& planningDiagnostics,
    const std::vector<ToolpathDiagnostic>& machiningDiagnostics,
    const std::vector<int>& excludedSourceHoleIndices = {});

} // namespace oap::toolpath

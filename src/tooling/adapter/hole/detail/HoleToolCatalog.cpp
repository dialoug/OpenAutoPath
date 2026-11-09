// 从刀库建立按类型筛选的孔加工刀具目录，并统一可用性与尺寸匹配规则。
// 属于 hole 模块 detail 私有实现:对外仅经 HoleToolCatalog.h 的公共 API,
// 内部把 AssignedTool 翻译为刀路刀具上下文,并提供“直径升序 + Drill 优先”
// 的稳定排序,使上层选刀结果不依赖刀库遍历顺序。
#include "tooling/adapter/hole/HoleToolCatalog.h"
#include "tooling/adapter/AssignedToolContext.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <utility>

namespace oap::tooling {
namespace {

// 按类型筛选后统一排序:直径升序 -> 同直径 Drill 优先 -> T 号升序。
// 该顺序是后续“最大不超径”查找与批次稳定排序的前提。
std::vector<HoleToolCandidate> collectTools(
    const QVector<AssignedTool>& assigned,
    std::initializer_list<ToolType> types)
{
    std::vector<HoleToolCandidate> result;
    for (const AssignedTool& item : assigned) {
        if (std::find(types.begin(), types.end(), item.tool.type) ==
            types.end()) {
            continue;
        }
        HoleToolCandidate candidate;
        candidate.type = item.tool.type;
        candidate.context = makeToolpathToolContext(item);
        if (candidate.context.valid)
            result.push_back(std::move(candidate));
    }

    std::stable_sort(
        result.begin(), result.end(),
        [](const HoleToolCandidate& first,
           const HoleToolCandidate& second) {
            if (std::abs(first.context.diameter -
                         second.context.diameter) >
                kHoleToolMatchTolerance) {
                return first.context.diameter <
                       second.context.diameter;
            }
            if (first.type != second.type) {
                if (first.type == ToolType::Drill)
                    return true;
                if (second.type == ToolType::Drill)
                    return false;
            }
            return first.context.toolNumber <
                   second.context.toolNumber;
        });
    return result;
}

} // namespace

HoleToolCatalog buildHoleToolCatalog(const ToolLibrary& library)
{
    const QVector<AssignedTool> assigned = library.assignedTools();
    HoleToolCatalog catalog;
    catalog.centerDrills =
        collectTools(assigned, {ToolType::CenterDrill});
    catalog.drills = collectTools(
        assigned, {ToolType::Drill, ToolType::IndexableDrill});
    catalog.flatEndMills =
        collectTools(assigned, {ToolType::EndMill});
    return catalog;
}

bool assignedHoleToolContext(const ToolLibrary& library,
                             const std::string& toolId,
                             HoleToolCandidate& result)
{
    const QString id = QString::fromUtf8(toolId.c_str());
    for (const AssignedTool& assigned : library.assignedTools()) {
        if (assigned.tool.id != id)
            continue;
        result.type = assigned.tool.type;
        result.context = makeToolpathToolContext(assigned);
        return result.context.valid;
    }
    return false;
}

// 在升序列表中找不超过目标直径的最大刀:一旦超过立即 break,
// 保证结果是最接近目标直径的可用刀具。
const HoleToolCandidate* findLargestNotExceeding(
    const std::vector<HoleToolCandidate>& tools,
    double diameter,
    double tolerance)
{
    const HoleToolCandidate* selected = nullptr;
    for (const HoleToolCandidate& tool : tools) {
        if (tool.context.diameter > diameter + tolerance)
            break;
        if (!selected ||
            tool.context.diameter >
                selected->context.diameter +
                    kHoleToolMatchTolerance) {
            selected = &tool;
        }
    }
    return selected;
}

bool toolHasRequiredCuttingEdge(
    const HoleToolCandidate& tool,
    double requiredLength) noexcept
{
    if (!std::isfinite(requiredLength) ||
        requiredLength <= kHoleToolReachTolerance) {
        return true;
    }
    const double available = tool.context.cuttingEdgeLength;
    // 未知刃长保持现有的非阻断语义，由统一验证器给出 warning；
    // 只有刀库中已知且明确不足的刀具才在选刀阶段淘汰。
    return !std::isfinite(available) || available <= 0.0 ||
           available + kHoleToolReachTolerance >= requiredLength;
}

bool toolHasRequiredReach(const HoleToolCandidate& tool,
                          double requiredLength) noexcept
{
    if (!std::isfinite(requiredLength) ||
        requiredLength <= kHoleToolReachTolerance) {
        return true;
    }
    const double available = tool.context.overallLength;
    return !std::isfinite(available) || available <= 0.0 ||
           available + kHoleToolReachTolerance >= requiredLength;
}

// 同 findLargestNotExceeding,但额外要求刃长与伸长达标;
// 未达标刀具直接跳过,不阻断整个查找。
const HoleToolCandidate* findLargestUsableNotExceeding(
    const std::vector<HoleToolCandidate>& tools,
    double diameter,
    double requiredCuttingEdgeLength,
    double requiredReach,
    double tolerance)
{
    const HoleToolCandidate* selected = nullptr;
    for (const HoleToolCandidate& tool : tools) {
        if (tool.context.diameter > diameter + tolerance)
            break;
        if (!toolHasRequiredCuttingEdge(
                tool, requiredCuttingEdgeLength) ||
            !toolHasRequiredReach(tool, requiredReach)) {
            continue;
        }
        if (!selected ||
            tool.context.diameter >
                selected->context.diameter +
                    kHoleToolMatchTolerance) {
            selected = &tool;
        }
    }
    return selected;
}

} // namespace oap::tooling

// 孔自动选刀编排器：构建物理孔计划，并按中心钻、钻削和铣削阶段生成批次。
// 规划严格按固定六面顺序进行,保证跨重规划结果可复现;
// 具体阶段批次由 detail 子模块(HoleSetupPlanner/HolePhasePlanners)实现。
#include "tooling/adapter/hole/HoleToolSelectionPlanner.h"

#include <array>

#include "tooling/adapter/hole/detail/HolePlanningSupport.h"
#include "tooling/adapter/hole/detail/HoleSetupPlanner.h"
#include "tooling/adapter/hole/HoleToolCatalog.h"

namespace oap::tooling {
namespace {

// 固定装夹面顺序:正 Z -> 正 X -> 正 Y -> 负 X -> 负 Y -> 负 Z。
// 顺序即 setupIndex 编号顺序,也决定批次在刀路程序中的先后。
constexpr std::array<oap::MoldSetupFace, 6> kSetupOrder = {
    oap::MoldSetupFace::PositiveZ,
    oap::MoldSetupFace::PositiveX,
    oap::MoldSetupFace::PositiveY,
    oap::MoldSetupFace::NegativeX,
    oap::MoldSetupFace::NegativeY,
    oap::MoldSetupFace::NegativeZ};

HoleToolSelectionPlan planReadyInputs(
    const ToolLibrary& library,
    const std::vector<oap::toolpath::HoleMachiningInput>& holes,
    const oap::toolpath::ToolpathParams& params)
{
    HoleToolSelectionPlan result;
    // 先把刀库投影为按类型筛选的刀具目录;目录为空说明没有可用中心钻/
    // 钻头/平底铣,直接整体失败并给出诊断,不进入逐面规划。
    const HoleToolCatalog catalog = buildHoleToolCatalog(library);
    if (catalog.empty()) {
        detail::appendHoleToolDiagnostic(
            result,
            oap::toolpath::ToolpathDiagnosticCode::AutoToolLibraryEmpty,
            oap::toolpath::ToolpathDiagnosticSeverity::Error,
            "active library has no assigned center drill, drill, U-drill or flat end mill");
        return result;
    }

    // 按固定装夹面顺序逐面规划,nextPhysicalIndex 跨面连续计数,
    // 保证物理孔稳定编号在整个识别集上唯一。
    std::size_t nextPhysicalIndex = 0;
    for (const oap::MoldSetupFace face : kSetupOrder) {
        detail::planHoleSetup(
            catalog, holes, face, params,
            nextPhysicalIndex, result);
    }
    return result;
}

} // namespace

HoleToolSelectionPlan HoleToolSelectionPlanner::plan(
    const ToolLibrary& library,
    const std::vector<oap::toolpath::HoleMachiningInput>& inputs,
    const oap::toolpath::ToolpathParams& params) const
{
    // 保持与识别结果相同的数组长度，使覆盖审计和 Operation 的
    // sourceHoleIndex 始终指向同一个孔段。未解析项只作为空槽位，
    // 不会进入规划器。
    std::vector<oap::toolpath::HoleMachiningInput> readyHoles(inputs.size());
    std::vector<std::size_t> unresolvedEntryIndices;
    std::vector<std::size_t> unresolvedSetupIndices;
    for (std::size_t index = 0; index < inputs.size(); ++index) {
        const oap::toolpath::HoleMachiningInput& input = inputs[index];
        if (input.ready()) {
            readyHoles[index] = input;
            continue;
        }
        if (input.mappingState ==
            oap::toolpath::HoleMachiningMappingState::UnresolvedEntry) {
            unresolvedEntryIndices.push_back(index);
        } else {
            unresolvedSetupIndices.push_back(index);
        }
    }

    HoleToolSelectionPlan result = planReadyInputs(
        library, readyHoles, params);
    for (const std::size_t index : unresolvedEntryIndices) {
        detail::appendHoleToolDiagnostic(
            result,
            oap::toolpath::ToolpathDiagnosticCode::HoleOpeningUnresolved,
            oap::toolpath::ToolpathDiagnosticSeverity::Error,
            "hole entry is unresolved and was excluded from automatic planning",
            static_cast<int>(index));
    }
    for (const std::size_t index : unresolvedSetupIndices) {
        detail::appendHoleToolDiagnostic(
            result,
            oap::toolpath::ToolpathDiagnosticCode::HoleOutsideSixFace,
            oap::toolpath::ToolpathDiagnosticSeverity::Warning,
            "hole setup is unresolved and was excluded from automatic planning",
            static_cast<int>(index));
    }
    return result;
}

} // namespace oap::tooling

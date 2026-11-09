// 面向应用的刀路用例编排：把识别实例转换为区域并调用对应生成服务。
//
// 模块归属:oap_toolpath_workflow(无 UI 的刀路用例编排)。
// 编排策略:槽腔用例分两步(区域构建 → 批量生成),并在两步之间支持
// 协作取消;孔与外轮廓用例则直接透传给对应服务。本文件不含算法细节,
// 只做输入传递与结果组装。
#include "toolpath/workflow/ToolpathWorkflow.h"

#include <utility>

namespace oap::toolpath {

ToolpathWorkflow::ToolpathWorkflow() = default;

// 显式构造:把注册表与各自选中的算法 id 分发到三个生成服务。
ToolpathWorkflow::ToolpathWorkflow(
    std::shared_ptr<const ToolpathAlgorithmRegistry> algorithms,
    ToolpathAlgorithmSelection selection)
    : m_pocketService(algorithms, std::move(selection.pocketAlgorithmId)),
      m_holeService(algorithms, std::move(selection.holeAlgorithmId)),
      m_profileService(algorithms, std::move(selection.profileAlgorithmId))
{
}

PocketRegionWorkflowResult ToolpathWorkflow::buildPocketRegions(
    const std::vector<FeatureInstance>& instances,
    const StepIndexer& indexer,
    const PocketRegionBuildOptions& buildOptions) const
{
    PocketRegionWorkflowResult result;
    result.regions = PocketRegionBuilder().build(
        instances, indexer, buildOptions, &result.skippedRegions);
    return result;
}

PocketToolpathWorkflowResult ToolpathWorkflow::generatePockets(
    const std::vector<FeatureInstance>& instances,
    const StepIndexer& indexer,
    const ToolpathParams& params,
    const PocketRegionBuildOptions& buildOptions,
    const std::string& operationPrefix,
    const ToolpathGenerationControl* control) const
{
    // 区域构建前先响应取消:已取消时直接返回空批并附取消诊断。
    PocketToolpathWorkflowResult result;
    if (control && control->cancelled()) {
        result.batch.cancelled = true;
        result.batch.diagnostics.push_back(
            {ToolpathDiagnosticCode::GenerationCancelled,
             ToolpathDiagnosticSeverity::Info,
             "pocket workflow was cancelled before region building"});
        return result;
    }

    PocketRegionWorkflowResult regionResult =
        buildPocketRegions(instances, indexer, buildOptions);
    // 区域构建完成后再次检查取消,避免把大段生成浪费在已取消的任务上。
    result.regions = std::move(regionResult.regions);
    result.skippedRegions = std::move(regionResult.skippedRegions);
    if (control && control->cancelled()) {
        result.batch.cancelled = true;
        result.batch.diagnostics.push_back(
            {ToolpathDiagnosticCode::GenerationCancelled,
             ToolpathDiagnosticSeverity::Info,
             "pocket workflow was cancelled after region building"});
        return result;
    }

    result.batch = m_pocketService.generate(
        result.regions, params, operationPrefix, control);
    return result;
}

HoleToolpathBatch ToolpathWorkflow::generateHoles(
    const std::vector<HoleMachiningInput>& inputs,
    const ToolpathParams& params,
    const HoleRapidPlanningContext* rapidContext,
    const std::vector<HoleMachiningStrategy>* strategyOverrides,
    const ToolpathGenerationControl* control) const
{
    return m_holeService.generate(
        inputs, params, rapidContext, strategyOverrides, control);
}

ProfileToolpathBatch ToolpathWorkflow::generateProfiles(
    const std::vector<ProfileRegion>& regions,
    const ToolpathParams& params,
    const std::string& operationPrefix,
    const ToolpathGenerationControl* control) const
{
    return m_profileService.generate(
        regions, params, operationPrefix, control);
}

} // namespace oap::toolpath

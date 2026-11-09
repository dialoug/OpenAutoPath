// UI 无关的刀路用例层：集中协调“识别实例 -> 区域 -> 生成 -> 安全验证”。
// MainWindow 只负责收集输入与展示结果，不再自行拼装算法对象。
//
// 模块归属:oap_toolpath_workflow(无 UI 的刀路用例编排)。
// 职责:把 UI 层的输入(识别实例/孔输入/工艺参数)编排为完整刀路结果,
// 内部委托 PocketToolpathService、HoleToolpathService 与
// ProfileToolpathService;算法对象经
// 注册表注入,应用可整体替换算法实现。实例不共享可变状态,线程安全。
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "toolpath/workflow/hole/HoleToolpathService.h"
#include "toolpath/pocket/geometry/PocketRegionBuilder.h"
#include "toolpath/workflow/pocket/PocketToolpathService.h"
#include "toolpath/workflow/profile/ProfileToolpathService.h"

namespace oap::toolpath {

// 槽腔完整用例结果:构建出的区域、被跳过的区域(如过小/不可加工)
// 以及最终生成的刀路批。
struct PocketToolpathWorkflowResult {
    std::vector<PocketRegion> regions;
    std::vector<SkippedRegion> skippedRegions;
    PocketToolpathBatch batch;
};

// 仅区域构建阶段的结果,供只关心区域/跳过信息的调用方使用。
struct PocketRegionWorkflowResult {
    std::vector<PocketRegion> regions;
    std::vector<SkippedRegion> skippedRegions;
};

// 刀路用例编排器:对 UI/脚本提供一组高层次入口,屏蔽内部服务细节。
// 组合方式:持有孔、槽腔、外轮廓服务各一;三者通过构造注入的注册表
// 解析所选算法。
class ToolpathWorkflow {
public:
    // 默认构造:使用默认算法注册表与内置算法选择。
    ToolpathWorkflow();
    // 显式构造:注入自定义算法注册表与算法选择(用于可替换实现)。
    ToolpathWorkflow(
        std::shared_ptr<const ToolpathAlgorithmRegistry> algorithms,
        ToolpathAlgorithmSelection selection = {});

    // 仅构建槽腔区域:把识别实例转换为可加工的 PocketRegion 集合。
    // 参数:instances 识别出的特征实例;indexer 步进索引器(区域定位);
    //       buildOptions 构建选项。
    // 返回:区域列表与跳过原因列表。
    PocketRegionWorkflowResult buildPocketRegions(
        const std::vector<FeatureInstance>& instances,
        const StepIndexer& indexer,
        const PocketRegionBuildOptions& buildOptions = {}) const;

    // 完整槽腔用例:区域构建 + 批量生成刀路。
    // 参数:params 工艺参数;operationPrefix 生成工序 ID 的前缀;
    //       control 可选取消控制(在区域构建前后各检查一次)。
    // 返回:含区域、跳过信息与刀路批的结果。
    PocketToolpathWorkflowResult generatePockets(
        const std::vector<FeatureInstance>& instances,
        const StepIndexer& indexer,
        const ToolpathParams& params,
        const PocketRegionBuildOptions& buildOptions = {},
        const std::string& operationPrefix = "POCKET",
        const ToolpathGenerationControl* control = nullptr) const;

    // 孔加工用例:对一组孔输入生成完整批结果。
    // 参数:inputs 孔加工输入;params 工艺参数;rapidContext 可选孔间
    //       快速移动规划上下文;strategyOverrides 可选逐孔策略覆盖;
    //       control 可选取消控制。
    // 返回:含阶段化移动与诊断信息的刀路批。
    HoleToolpathBatch generateHoles(
        const std::vector<HoleMachiningInput>& inputs,
        const ToolpathParams& params,
        const HoleRapidPlanningContext* rapidContext = nullptr,
        const std::vector<HoleMachiningStrategy>* strategyOverrides = nullptr,
        const ToolpathGenerationControl* control = nullptr) const;

    // 外轮廓加工用例:对已构建并排好装夹顺序的区域生成完整批结果。
    ProfileToolpathBatch generateProfiles(
        const std::vector<ProfileRegion>& regions,
        const ToolpathParams& params,
        const std::string& operationPrefix = "PROFILE",
        const ToolpathGenerationControl* control = nullptr) const;

private:
    // 按值持有的三个生成服务;其内部算法解析自注入的注册表。
    PocketToolpathService m_pocketService;
    HoleToolpathService m_holeService;
    ProfileToolpathService m_profileService;
};

} // namespace oap::toolpath

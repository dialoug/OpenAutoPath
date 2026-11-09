// 刀路算法扩展边界。服务层只依赖这些接口和注册表，不直接构造具体算法。
// 新算法实现相应接口、注册唯一 id 后即可由专用服务或工作流选择使用。
//
// 模块归属:oap_toolpath_algorithms(可替换算法实现与注册表)。
// 设计:服务层(oap_toolpath_workflow)只透过 IPocket/IHole 接口 + 注册表
// 拿算法,实现可整体替换;内置算法以稳定字符串 id 注册。注册表对象不可
// 变地共享给后台任务,是异步执行的线程安全基础。
#pragma once

#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "toolpath/hole/HoleMachining.h"
#include "toolpath/pocket/planning/PocketClearing.h"
#include "toolpath/profile/planning/ProfileContour.h"

namespace oap::toolpath {

// 内置算法的稳定注册 id:外部(UI、配置文件、测试)用这些字符串选择算法,
// 注册表查询也依赖它们,因此一旦对外公布不得随意改动。
inline constexpr char kBuiltinPocketAlgorithmId[] =
    "builtin.pocket_clearing";
inline constexpr char kBuiltinHoleAlgorithmId[] =
    "builtin.hole_machining";
inline constexpr char kBuiltinProfileAlgorithmId[] =
    "builtin.profile.freecad";
// 槽腔刀路算法接口:输入封闭槽腔区域与工艺参数,输出清料刀路结果。
// 不变量:实现必须无状态、可并发调用;返回结果的坐标基于区域输入坐标系。
class IPocketToolpathAlgorithm {
public:
    virtual ~IPocketToolpathAlgorithm() = default;

    // 返回算法的稳定唯一标识(注册表键),不得为空。
    virtual std::string_view id() const noexcept = 0;
    // 生成槽腔清料刀路。
    // 参数:region 待加工的槽腔区域;params 刀具与工艺参数。
    // 返回:含移动序列与覆盖/诊断信息的完整结果。
    virtual PocketClearingResult generate(
        const PocketRegion& region,
        const ToolpathParams& params) const = 0;
    // 支持协作取消的扩展入口。现有第三方算法无需立即修改；默认实现保持
    // 两参数接口语义，内置算法会把 control 下传到逐层生成器。
    // 带取消控制的扩展入口。默认实现忽略 control 并退化为两参数接口,
    // 便于第三方算法先接入再逐步支持协作取消。
    virtual PocketClearingResult generate(
        const PocketRegion& region,
        const ToolpathParams& params,
        const ToolpathGenerationControl* control) const
    {
        (void)control;
        return generate(region, params);
    }
};

// 轮廓刀路算法接口：由注册表按 id 选择并生成轮廓刀路。
class IProfileToolpathAlgorithm {
public:
    virtual ~IProfileToolpathAlgorithm() = default;
    virtual std::string_view id() const noexcept = 0;
    virtual ProfileContourResult generate(
        const ProfileRegion& region,
        const ToolpathParams& params,
        const ToolpathGenerationControl* control = nullptr) const = 0;
};

// 孔加工刀路算法接口:对一组孔输入生成含钻孔/锪孔/攻丝阶段的刀路。
// 不变量:实现必须无状态、可并发调用,并遵守 control 的取消语义。
class IHoleToolpathAlgorithm {
public:
    virtual ~IHoleToolpathAlgorithm() = default;

    // 返回算法的稳定唯一标识(注册表键),不得为空。
    virtual std::string_view id() const noexcept = 0;
    // 生成一组孔的加工刀路。
    // 参数:holes 待加工的孔集合;params 工艺参数;rapidContext 可选
    //       快速移动规划上下文(孔间连接);strategyOverrides 可选的
    //       逐孔策略覆盖;control 取消控制(非空时须检查)。
    // 返回:含阶段化移动序列与诊断信息的完整结果。
    virtual HoleMachiningResult generate(
        const std::vector<HoleMachiningInput>& holes,
        const ToolpathParams& params,
        const HoleRapidPlanningContext* rapidContext,
        const std::vector<HoleMachiningStrategy>* strategyOverrides,
        const ToolpathGenerationControl* control) const = 0;
};

// 注册表本身不使用全局可变状态。应用可复制默认注册表、追加自定义算法，再把
// const shared_ptr 注入工作流/服务/异步执行器；后台任务可安全共享算法对象。
class ToolpathAlgorithmRegistry {
public:
    bool registerPocketAlgorithm(
        std::shared_ptr<const IPocketToolpathAlgorithm> algorithm);
    bool registerHoleAlgorithm(
        std::shared_ptr<const IHoleToolpathAlgorithm> algorithm);
    bool registerProfileAlgorithm(
        std::shared_ptr<const IProfileToolpathAlgorithm> algorithm);

    const IPocketToolpathAlgorithm* pocketAlgorithm(
        std::string_view id) const noexcept;
    const IHoleToolpathAlgorithm* holeAlgorithm(
        std::string_view id) const noexcept;
    const IProfileToolpathAlgorithm* profileAlgorithm(
        std::string_view id) const noexcept;

    std::vector<std::string> pocketAlgorithmIds() const;
    std::vector<std::string> holeAlgorithmIds() const;
    std::vector<std::string> profileAlgorithmIds() const;

private:
    std::map<std::string,
             std::shared_ptr<const IPocketToolpathAlgorithm>,
             std::less<>> m_pocketAlgorithms;
    std::map<std::string,
             std::shared_ptr<const IHoleToolpathAlgorithm>,
             std::less<>> m_holeAlgorithms;
    std::map<std::string,
             std::shared_ptr<const IProfileToolpathAlgorithm>,
             std::less<>> m_profileAlgorithms;
};

std::shared_ptr<const ToolpathAlgorithmRegistry>
defaultToolpathAlgorithmRegistry();

// 一次刀路生成任务所选择的算法 id 集合(默认取内置算法),
// 供服务层/异步执行器从注册表解析实际算法对象。
struct ToolpathAlgorithmSelection {
    std::string pocketAlgorithmId = kBuiltinPocketAlgorithmId;
    std::string holeAlgorithmId = kBuiltinHoleAlgorithmId;
    std::string profileAlgorithmId = kBuiltinProfileAlgorithmId;
};

} // namespace oap::toolpath

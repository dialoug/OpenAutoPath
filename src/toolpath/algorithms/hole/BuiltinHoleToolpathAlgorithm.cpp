// 将内置 HoleMachining 包装为可注册的孔刀路算法实现。
//
// 模块归属:oap_toolpath_algorithms。本文件是「内置实现 → 算法接口」的
// 适配层:HoleMachining 的调用约定与 IHoleToolpathAlgorithm 一致,因此
// 这里仅做透传转发,不含业务逻辑;算法本身无状态,可并发调用。
#include "toolpath/algorithms/hole/BuiltinHoleToolpathAlgorithm.h"

#include "toolpath/algorithms/ToolpathAlgorithm.h"
#include "toolpath/hole/HoleMachining.h"

namespace oap::toolpath {
namespace {

// 内置孔加工算法:固定使用 kBuiltinHoleAlgorithmId 作为注册 id。
class BuiltinHoleToolpathAlgorithm final : public IHoleToolpathAlgorithm {
public:
    std::string_view id() const noexcept override
    {
        return kBuiltinHoleAlgorithmId;
    }

    // 透传调用 HoleMachining::generate(参数语义与接口一致),
    // 取消控制 control 被直接下传到生成器内部。
    HoleMachiningResult generate(
        const std::vector<HoleMachiningInput>& holes,
        const ToolpathParams& params,
        const HoleRapidPlanningContext* rapidContext,
        const std::vector<HoleMachiningStrategy>* strategyOverrides,
        const ToolpathGenerationControl* control) const override
    {
        return HoleMachining().generate(
            holes, params, rapidContext, strategyOverrides, control);
    }
};

} // namespace

// 工厂实现:创建一次性的匿名实现对象并作为 const 共享指针返回。
std::shared_ptr<const IHoleToolpathAlgorithm>
makeBuiltinHoleToolpathAlgorithm()
{
    return std::make_shared<BuiltinHoleToolpathAlgorithm>();
}

} // namespace oap::toolpath

// 将内置 PocketClearing 包装为可注册的槽腔刀路算法实现。
//
// 模块归属:oap_toolpath_algorithms。本文件是「内置实现 → 算法接口」的
// 适配层,仅透传转发 PocketClearing 的调用,不含业务逻辑;算法无状态,
// 可并发调用,并支持带取消控制的生成入口。
#include "toolpath/algorithms/pocket/BuiltinPocketToolpathAlgorithm.h"

#include "toolpath/algorithms/ToolpathAlgorithm.h"
#include "toolpath/pocket/planning/PocketClearing.h"

namespace oap::toolpath {
namespace {

// 内置槽腔清料算法:固定使用 kBuiltinPocketAlgorithmId 作为注册 id。
class BuiltinPocketToolpathAlgorithm final
    : public IPocketToolpathAlgorithm {
public:
    std::string_view id() const noexcept override
    {
        return kBuiltinPocketAlgorithmId;
    }

    // 两参数版本:透传调用 PocketClearing::generate。
    PocketClearingResult generate(
        const PocketRegion& region,
        const ToolpathParams& params) const override
    {
        return PocketClearing().generate(region, params);
    }

    // 带取消控制版本:把 control 直接下传到 PocketClearing 的逐层生成器。
    PocketClearingResult generate(
        const PocketRegion& region,
        const ToolpathParams& params,
        const ToolpathGenerationControl* control) const override
    {
        return PocketClearing().generate(region, params, control);
    }
};

} // namespace

// 工厂实现:创建一次性的匿名实现对象并作为 const 共享指针返回。
std::shared_ptr<const IPocketToolpathAlgorithm>
makeBuiltinPocketToolpathAlgorithm()
{
    return std::make_shared<BuiltinPocketToolpathAlgorithm>();
}

} // namespace oap::toolpath

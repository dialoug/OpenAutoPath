// BuiltinProfileToolpathAlgorithm 实现（oap_toolpath_algorithms）。
// 内置轮廓算法仅把刀路生成委托给 ProfileContour，并上报固定算法标识
// kBuiltinProfileAlgorithmId；工厂声明见 BuiltinProfileToolpathAlgorithm.h。
#include "toolpath/algorithms/profile/BuiltinProfileToolpathAlgorithm.h"

#include "toolpath/algorithms/ToolpathAlgorithm.h"

namespace oap::toolpath {
namespace {

class BuiltinProfileToolpathAlgorithm final
    : public IProfileToolpathAlgorithm {
public:
    std::string_view id() const noexcept override
    {
        return kBuiltinProfileAlgorithmId;
    }

    ProfileContourResult generate(
        const ProfileRegion& region,
        const ToolpathParams& params,
        const ToolpathGenerationControl* control) const override
    {
        return ProfileContour().generate(region, params, control);
    }
};

} // namespace

std::shared_ptr<const IProfileToolpathAlgorithm>
makeBuiltinProfileToolpathAlgorithm()
{
    return std::make_shared<BuiltinProfileToolpathAlgorithm>();
}

} // namespace oap::toolpath

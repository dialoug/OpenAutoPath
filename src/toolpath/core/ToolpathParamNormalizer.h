// 刀路参数统一归一化入口。生成器只接收 effective 参数，避免各算法对同一
// 无效输入采用不同回退值。
#pragma once

#include <vector>

#include "ToolpathDiagnostics.h"
#include "ToolpathParams.h"

namespace oap::toolpath {

// 参数归一化结果：生效参数与诊断列表。
struct ToolpathParamNormalization {
    ToolpathParams effective;
    std::vector<ToolpathDiagnostic> diagnostics;

    // 是否无任何 Error 级诊断；Warning 级（自动回退）不阻止生成。
    bool ok() const noexcept;
};

// 参数归一化器：把请求参数规整为生效参数并产出诊断。
class ToolpathParamNormalizer {
public:
    // 对请求参数做一次性归一化：修回无效值、修正依赖关系并记录每个
    // 自动修正。返回结果包含可安全使用的 effective 副本。线程安全：
    // 无状态，可被多个线程共享调用。
    ToolpathParamNormalization normalize(
        const ToolpathParams& requested) const;
};

} // namespace oap::toolpath

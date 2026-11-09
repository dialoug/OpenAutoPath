// 兼容头；槽/腔安全检查已经归入 pocket 模块。
//
// 保留给旧调用方的最小转发壳：把 PocketSafetyValidator 包成旧式接口，
// 便于逐步迁移；新代码应直接使用 PocketSafetyValidator。
#pragma once

#include "toolpath/pocket/validation/PocketSafetyValidator.h"

namespace oap::toolpath {

// 刀路安全校验器：对槽腔工序执行安全校验。
class ToolpathSafetyValidator {
public:
    PocketSafetyValidationResult validatePocketOperation(
        const PocketRegion& region,
        const PocketToolpathOperation& operation,
        const ToolpathParams& params) const
    {
        return PocketSafetyValidator().validate(
            region, operation, params);
    }
};

} // namespace oap::toolpath

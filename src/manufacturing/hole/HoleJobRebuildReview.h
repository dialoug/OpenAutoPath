// Job 重建验证错误的局部隔离结果，供服务层保留仍有效的孔工序。
// 重建失败仅淘汰出错工序而非整批丢弃，避免一次错误使整个 Job 无法编辑。
#pragma once

#include <string>
#include <unordered_set>

#include "toolpath/workflow/hole/HoleToolpathService.h"

namespace oap::manufacturing {

// 重建审查结论：标识应被排除的工序（或全部）并收集降级后的警告。
struct HoleJobRebuildReview {
    bool omitAllOperations = false;
    std::unordered_set<std::string> invalidOperationIds;
    toolpath::ToolpathValidationReport warnings;

    bool omits(const std::string& operationId) const noexcept;
    bool needsReview() const noexcept;
};

// 检查重建生成的批次，把失败工序的记录转换为警告并返回审查结论；
// savedOperationId 是保存的 Job 工序 ID，用于映射重建展开后的操作 ID。
HoleJobRebuildReview reviewHoleJobRebuild(
    const toolpath::HoleToolpathBatch& generated,
    const std::string& savedOperationId);

} // namespace oap::manufacturing

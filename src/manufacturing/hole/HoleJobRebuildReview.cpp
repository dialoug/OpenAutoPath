// HoleJobRebuildReview 的实现（oap_manufacturing）：重建审查与警告转换。
#include "manufacturing/hole/HoleJobRebuildReview.h"

#include <unordered_map>

namespace oap::manufacturing {

bool HoleJobRebuildReview::omits(
    const std::string& operationId) const noexcept
{
    return omitAllOperations || invalidOperationIds.contains(operationId);
}

bool HoleJobRebuildReview::needsReview() const noexcept
{
    return omitAllOperations || !invalidOperationIds.empty();
}

HoleJobRebuildReview reviewHoleJobRebuild(
    const toolpath::HoleToolpathBatch& generated,
    const std::string& savedOperationId)
{
    // 建立“重建操作 ID → 保存工序的展开操作 ID”映射，用于警告定位：
    // 一个保存工序可被展开为多个重建操作（savedOperationId-<序号>）。
    HoleJobRebuildReview result;
    std::unordered_map<std::string, std::string> restoredOperationIds;
    for (std::size_t index = 0;
         index < generated.machining.operations.size(); ++index) {
        restoredOperationIds.emplace(
            generated.machining.operations[index].operationId,
            savedOperationId + "-" + std::to_string(index + 1));
    }
    for (const toolpath::ToolpathIssue& issue : generated.validation.issues) {
        if (issue.severity != toolpath::ToolpathIssueSeverity::Error)
            continue;
        // 重建校验中的错误若指向整个批次（operationId 为空）则全部省略；
        // 否则只淘汰对应工序。失败原因降级为警告，避免用户丢失诊断上下文。
        if (issue.operationId.empty())
            result.omitAllOperations = true;
        else
            result.invalidOperationIds.insert(issue.operationId);
        const auto restored = restoredOperationIds.find(issue.operationId);
        toolpath::addToolpathIssue(
            result.warnings,
            toolpath::ToolpathIssueSeverity::Warning,
            issue.code,
            "saved operation was omitted during Job rebuild: " +
                issue.message,
            restored != restoredOperationIds.end()
                ? restored->second
                : savedOperationId,
            issue.moveIndex);
    }
    return result;
}

} // namespace oap::manufacturing

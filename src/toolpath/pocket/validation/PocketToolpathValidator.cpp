// 检查槽腔工序的区域、深度层、移动结构和刀具上下文是否相互一致。
//
// 先校验批次参数与工序非空，再对每个工序检查：操作 id 唯一性、
// clearing.ok/failureCode、环数与深度层数、clearance/safe 高度关系、
// 工序刀具几何与生成半径一致、rest 加工必须引用更大前刀、逐层
// section 计数与深度层匹配；最后委托通用 moves/primitives 校验。
#include "toolpath/pocket/validation/PocketToolpathValidator.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "toolpath/core/validation/ToolpathMoveValidator.h"
#include "toolpath/core/validation/ToolpathParameterValidator.h"
#include "toolpath/core/validation/ToolpathPrimitiveValidator.h"

namespace oap::toolpath {

ToolpathValidationReport PocketToolpathValidator::validate(
    const std::vector<PocketToolpathOperation>& operations,
    const ToolpathParams& params) const
{
    ToolpathValidationReport report;
    validateToolpathParameters(params, report);

    if (operations.empty()) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Error,
            "empty_operation_set",
            "no pocket clearing operations were generated");
        return report;
    }

    std::unordered_set<std::string> operationIds;
    for (const PocketToolpathOperation& operation : operations) {
        if (operation.operationId.empty()) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Warning,
                "missing_operation_id",
                "pocket operation has no operation id");
        } else if (!operationIds.insert(
                       operation.operationId).second) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Warning,
                "duplicate_operation_id",
                "duplicate pocket operation id",
                operation.operationId);
        }

        if (!operation.clearing.ok) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Error,
                "clearing_not_ok",
                "pocket clearing result is not ok: " +
                    operation.clearing.error,
                operation.operationId);
        } else if (
            operation.clearing.failureCode !=
            PocketClearingFailureCode::None) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Error,
                "successful_clearing_has_failure",
                "successful pocket clearing result still carries a failure code",
                operation.operationId);
        }
        if (operation.clearing.loops.empty()) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Error,
                "missing_loops",
                "pocket operation has no clearing loops",
                operation.operationId);
        }
        if (operation.clearing.depthLayerCount <= 0) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Error,
                "missing_depth_layers",
                "pocket operation has no depth layers",
                operation.operationId);
        }
        if (operation.clearing.clearanceHeightOffset + 1.0e-9 <
            operation.clearing.safeHeightOffset) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Error,
                "clearance_below_safe_result",
                "result clearance height is below safe height",
                operation.operationId);
        }
        if (operation.tool.valid) {
            if (operation.tool.toolNumber <= 0 ||
                !std::isfinite(operation.tool.diameter) ||
                operation.tool.diameter <= 0.0) {
                addToolpathIssue(
                    report, ToolpathIssueSeverity::Error,
                    "invalid_operation_tool",
                    "operation tool must have a positive T number and diameter",
                    operation.operationId);
            } else if (std::abs(
                           0.5 * operation.tool.diameter -
                           operation.clearing.toolRadius) > 1.0e-6) {
                addToolpathIssue(
                    report, ToolpathIssueSeverity::Error,
                    "operation_tool_geometry_mismatch",
                    "operation tool diameter does not match the generated pocket path",
                    operation.operationId);
            }
        }
        if (!std::isfinite(
                operation.restMachiningPreviousToolDiameter) ||
            operation.restMachiningPreviousToolDiameter < 0.0 ||
            (operation.restMachiningPreviousToolDiameter > 0.0 &&
             operation.restMachiningPreviousToolDiameter <=
                 (operation.tool.valid
                      ? operation.tool.diameter
                      : 2.0 * operation.clearing.toolRadius) +
                     1.0e-7)) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Error,
                "invalid_rest_machining_tool_chain",
                "rest machining must reference a larger previous cutter",
                operation.operationId);
        }

        if (operation.clearing.usedPerLayerSections) {
            if (operation.clearing.sectionLayerCount !=
                operation.clearing.depthLayerCount) {
                addToolpathIssue(
                    report, ToolpathIssueSeverity::Error,
                    "section_layer_count_mismatch",
                    "per-layer section count does not match depth layer count",
                    operation.operationId);
            }

            std::vector<bool> hasLoop(
                std::max(0, operation.clearing.depthLayerCount),
                false);
            for (const ClearingLoop& loop :
                 operation.clearing.loops) {
                if (loop.depthIndex >= 0 &&
                    loop.depthIndex <
                        static_cast<int>(hasLoop.size())) {
                    hasLoop[loop.depthIndex] = true;
                }
            }
            for (const bool present : hasLoop) {
                if (present)
                    continue;
                if (operation.clearing.restMachining)
                    continue;
                addToolpathIssue(
                    report, ToolpathIssueSeverity::Error,
                    "missing_section_depth_layer",
                    "per-layer pocket result has a missing depth layer",
                    operation.operationId);
                break;
            }
        }

        validateToolpathMoves(
            operation.clearing.moves,
            operation.operationId, report);
        validateToolpathPrimitives(
            operation.clearing.primitives,
            operation.operationId, report);
        validateToolpathPrimitiveExpansion(
            operation.clearing.primitives,
            operation.clearing.moves,
            operation.operationId, report, true);
    }
    return report;
}

} // namespace oap::toolpath

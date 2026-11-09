// 验证中立程序的工序 ID、装夹一致性、刀具上下文和每段移动结构。
//
// 模块归属:oap_toolpath_program 的 validation 层。本文件把子验证器
// (参数、移动、原语、刀具)聚合到程序级,并用自定义规则补充跨工序的
// 一致性检查(工序 ID 唯一、装夹面不冲突)。策略为「收集而非中断」:
// 一个操作的问题不会阻止其余操作被继续检查,保证报告信息完整。
#include "toolpath/program/validation/ToolpathProgramValidator.h"

#include <cmath>
#include <map>
#include <unordered_set>

#include "toolpath/core/validation/ToolpathMoveValidator.h"
#include "toolpath/core/validation/ToolpathParameterValidator.h"
#include "toolpath/core/validation/ToolpathPrimitiveValidator.h"
#include "toolpath/core/validation/ToolpathToolValidator.h"

namespace oap::toolpath {

ToolpathValidationReport ToolpathProgramValidator::validate(
    const ToolpathProgram& program,
    const ToolpathParams& params) const
{
    ToolpathValidationReport report;
    // 先校验全局工艺参数,再进入操作级检查。
    validateToolpathParameters(params, report);

    if (program.operations.empty()) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Error,
            "empty_operation_set",
            "toolpath program has no operations");
        return report;
    }

    if (operationToolContextMode(program) ==
        OperationToolContextMode::Mixed) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Error,
            "mixed_operation_tool_context",
            "toolpath program mixes operations with and without bound tool contexts");
    }

    // 跨工序一致性状态:已见操作 ID 集合 + 每个装夹索引已解析的
    // 模具面(用于检测同一装夹指到多个不同型腔面的冲突)。
    std::unordered_set<std::string> operationIds;
    std::map<int, MoldSetupFace> setupFaces;
    for (const ToolpathOperation& operation : program.operations) {
        if (operation.operationId.empty()) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Warning,
                "missing_operation_id",
                "toolpath operation has no operation id");
        } else if (!operationIds.insert(
                       operation.operationId).second) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Error,
                "duplicate_operation_id",
                "duplicate toolpath operation id",
                operation.operationId);
        }

        // 装夹索引必须非负;同一 setupIndex 下若出现相互矛盾的已解析
        // 模具面则报 Error,Unresolved 视为占位可被后续补全。
        if (operation.setupIndex < 0) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Error,
                "invalid_setup_index",
                "toolpath operation has a negative setup index",
                operation.operationId);
        }
        auto [setupFace, inserted] = setupFaces.emplace(
            operation.setupIndex, operation.setupFace);
        if (!inserted) {
            if (setupFace->second == MoldSetupFace::Unresolved) {
                setupFace->second = operation.setupFace;
            } else if (
                operation.setupFace != MoldSetupFace::Unresolved &&
                operation.setupFace != setupFace->second) {
                addToolpathIssue(
                    report, ToolpathIssueSeverity::Error,
                    "conflicting_setup_face",
                    "one setup index refers to multiple resolved mold faces",
                    operation.operationId);
            }
        }
        if (operation.tool.valid &&
            (operation.tool.toolNumber <= 0 ||
             !std::isfinite(operation.tool.diameter) ||
             operation.tool.diameter <= 0.0)) {
            addToolpathIssue(
                report, ToolpathIssueSeverity::Error,
                "invalid_operation_tool",
                "operation tool must have a positive T number and diameter",
                operation.operationId);
        }
        // 刀具上下文:有效时校验刀号/直径为正且有限,并核对切削刃长度
        // 需求是否满足。
        validateCuttingEdgeRequirement(
            operation.tool.valid ? &operation.tool : nullptr,
            params.tool.cuttingEdgeLength,
            operation.minimumCuttingEdgeLength,
            operation.operationId, report);

        // 委托移动/原语/原语展开子验证器做逐项结构检查;展开校验对
        // 槽腔清料操作额外要求原语与移动能够相互印证。
        validateToolpathMoves(
            operation.moves, operation.operationId, report);
        validateToolpathPrimitives(
            operation.primitives, operation.operationId, report);
        validateToolpathPrimitiveExpansion(
            operation.primitives, operation.moves,
            operation.operationId, report,
            operation.type == ToolpathOperationType::PocketClearing ||
                operation.type == ToolpathOperationType::ProfileContour ||
                operation.type == ToolpathOperationType::SurfaceFinishing);
    }
    return report;
}

} // namespace oap::toolpath

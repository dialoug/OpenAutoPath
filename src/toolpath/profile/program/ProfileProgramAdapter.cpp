// ProfileProgramAdapter 实现（oap_toolpath_profile）。
// 轮廓工序到中立 ToolpathOperation / ToolpathProgram 的翻译，见
// ProfileProgramAdapter.h。
#include "toolpath/profile/program/ProfileProgramAdapter.h"

namespace oap::toolpath {

ToolpathOperation toToolpathOperation(
    const ProfileToolpathOperation& operation)
{
    ToolpathOperation result;
    result.operationId = operation.operationId;
    result.type = ToolpathOperationType::ProfileContour;
    // FeatureKind 尚未把“工序目标”混入识别特征枚举；Profile 的准确语义由
    // operation type 和 metadata 表达。
    result.featureKind = FeatureKind::Pocket;
    result.sourceIndex = operation.sourceRegionIndex;
    result.setupIndex = operation.setupIndex;
    result.setupFace = operation.setupFace;
    result.tool = operation.tool;
    result.sourceFeatureIds = operation.sourceFaceIds;
    result.metadata.emplace("algorithm_id", operation.algorithmId);
    result.metadata.emplace("profile_source", "root_outer_wire");
    result.metadata.emplace("root_face_id", operation.rootFaceId);
    result.metadata.emplace("root_wire_id", operation.rootWireId);
    result.metadata.emplace(
        "rest_previous_tool_diameter",
        std::to_string(operation.restMachiningPreviousToolDiameter));
    result.metadata.emplace(
        "profile_finish_pass",
        operation.finishPass ? "true" : "false");
    result.moves = operation.contour.moves;
    result.primitives = operation.contour.primitives;
    return result;
}

ToolpathProgram makeToolpathProgram(
    std::span<const ProfileToolpathOperation> operations)
{
    ToolpathProgram program;
    appendToolpathOperations(program, operations);
    return program;
}

void appendToolpathOperations(
    ToolpathProgram& program,
    std::span<const ProfileToolpathOperation> operations)
{
    program.operations.reserve(program.operations.size() + operations.size());
    for (const ProfileToolpathOperation& operation : operations)
        program.operations.push_back(toToolpathOperation(operation));
}

} // namespace oap::toolpath

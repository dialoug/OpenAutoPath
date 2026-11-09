// 把槽腔专用 Operation 投影为中立 ToolpathOperation，供验证和后处理复用。
#include "toolpath/pocket/program/PocketProgramAdapter.h"

namespace oap::toolpath {

ToolpathOperation toToolpathOperation(
    const PocketToolpathOperation& operation)
{
    // 字段映射：槽腔专属字段（kind/sourceRegionIndex/sourceFaceIds/
    // algorithmId/floorFaceId）落入中立工序的对应槽位，移动与图元
    // 原样搬入（仅共享引用语义的 vector 复制，不重新生成几何）。
    ToolpathOperation result;
    result.operationId = operation.operationId;
    result.type = ToolpathOperationType::PocketClearing;
    result.featureKind = operation.kind;
    result.setupIndex = operation.setupIndex;
    result.setupFace = operation.setupFace;
    result.tool = operation.tool;
    result.sourceIndex = operation.sourceRegionIndex;
    result.sourceFeatureIds = operation.sourceFaceIds;
    if (!operation.algorithmId.empty()) {
        result.metadata.emplace(
            "algorithm_id", operation.algorithmId);
    }
    result.metadata.emplace(
        "floor_face_id", operation.floorFaceId);
    result.moves = operation.clearing.moves;
    result.primitives = operation.clearing.primitives;
    return result;
}

ToolpathProgram makeToolpathProgram(
    const std::vector<PocketToolpathOperation>& operations)
{
    ToolpathProgram program;
    appendToolpathOperations(program, operations);
    return program;
}

void appendToolpathOperations(
    ToolpathProgram& program,
    const std::vector<PocketToolpathOperation>& operations)
{
    program.operations.reserve(
        program.operations.size() + operations.size());
    for (const PocketToolpathOperation& operation : operations) {
        program.operations.push_back(
            toToolpathOperation(operation));
    }
}

} // namespace oap::toolpath

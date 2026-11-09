// Compatibility-only bridge for legacy BRep simulation and display consumers.
// 把 ProfileToolpathOperation 只读转成 PocketToolpathOperation 模拟载体，
// 保留真实 moves/primitives、刀具快照与装夹刀轴，见 LegacyProfileSimulationAdapter.h。
#include "toolpath/workflow/compatibility/LegacyProfileSimulationAdapter.h"

namespace oap::toolpath {

PocketToolpathOperation legacyProfileSimulationOperation(
    const ProfileToolpathOperation& source)
{
    PocketToolpathOperation result;
    result.operationId = source.operationId;
    result.algorithmId = source.algorithmId;
    result.kind = FeatureKind::Slot;
    result.setupIndex = source.setupIndex;
    result.setupFace = source.setupFace;
    result.tool = source.tool;
    result.sourceRegionIndex = source.sourceRegionIndex;
    result.outwardToolAxis = source.outwardToolAxis;
    result.sourceFaceIds = source.sourceFaceIds;
    result.floorFaceId = source.rootFaceId;
    result.restMachiningPreviousToolDiameter =
        source.restMachiningPreviousToolDiameter;
    result.clearing.moves = source.contour.moves;
    result.clearing.primitives = source.contour.primitives;
    result.clearing.ok = source.contour.ok;
    result.clearing.cancelled = source.contour.cancelled;
    result.clearing.restMachining = source.contour.restMachining;
    result.clearing.noRestMaterial = source.contour.noRestMaterial;
    result.clearing.warning = source.contour.warning;
    result.clearing.error = source.contour.error;
    result.clearing.depthLayerCount =
        source.contour.depthLayerCount;
    result.clearing.ringCount =
        source.contour.radialPassCount;
    result.clearing.toolRadius = source.contour.toolRadius;
    result.clearing.stepOver = source.contour.radialStepOver;
    result.clearing.firstOffset = source.contour.finalOffset;
    result.clearing.safeHeightOffset =
        source.contour.safeHeightOffset;
    result.clearing.clearanceHeightOffset =
        source.contour.clearanceHeightOffset;
    // 不复制 ProfileLoop 到 ClearingLoop。槽腔解析快路径会把最外闭环内部
    // 解释为应整面清除；外轮廓只允许按真实 Cut/Plunge 图元扫掠刀具体积。
    return result;
}

std::vector<PocketToolpathOperation> legacyProfileSimulationOperations(
    const std::vector<ProfileToolpathOperation>& operations)
{
    std::vector<PocketToolpathOperation> result;
    result.reserve(operations.size());
    for (const ProfileToolpathOperation& operation : operations)
        result.push_back(legacyProfileSimulationOperation(operation));
    return result;
}

} // namespace oap::toolpath

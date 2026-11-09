// holeSimulationOperation(s) 实现（oap_toolpath_workflow）。
// 把孔加工工序只读转成 PocketToolpathOperation 模拟载体，保留真实
// moves/primitives、刀具快照与装夹刀轴，见 HoleSimulationAdapter.h。
#include "toolpath/workflow/hole/HoleSimulationAdapter.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <Precision.hxx>

namespace oap::toolpath {
namespace {

// 计算孔外向刀轴：优先 bottom→entry，退化时回退到 Plunge move 的方向。
gp_Vec holeOutwardAxis(const HoleMachiningOperation& operation)
{
    gp_Vec axis(operation.bottom, operation.entry);
    if (std::isfinite(axis.X()) && std::isfinite(axis.Y()) &&
        std::isfinite(axis.Z()) &&
        axis.Magnitude() > Precision::Confusion()) {
        axis.Normalize();
        return axis;
    }
    for (const ToolpathMove& move : operation.moves) {
        if (move.type != ToolpathMoveType::Plunge)
            continue;
        axis = gp_Vec(move.to, move.from);
        if (std::isfinite(axis.X()) && std::isfinite(axis.Y()) &&
            std::isfinite(axis.Z()) &&
            axis.Magnitude() > Precision::Confusion()) {
            axis.Normalize();
            return axis;
        }
    }
    return {};
}

} // namespace

PocketToolpathOperation holeSimulationOperation(
    const HoleMachiningOperation& source)
{
    PocketToolpathOperation result;
    result.operationId = source.operationId;
    result.algorithmId = source.algorithmId;
    result.kind = source.kind;
    result.setupIndex = source.setupIndex;
    result.setupFace = source.setupFace;
    result.tool = source.tool;
    if ((!std::isfinite(result.tool.tipAngle) ||
         result.tool.tipAngle <= 0.0) &&
        std::isfinite(source.drillTipLength) &&
        source.drillTipLength > Precision::Confusion() &&
        std::isfinite(source.radius) && source.radius > 0.0) {
        result.tool.tipAngle =
            2.0 * std::atan(source.radius / source.drillTipLength) *
            180.0 / std::numbers::pi;
    }
    result.sourceRegionIndex = source.sourceHoleIndex;
    result.outwardToolAxis = holeOutwardAxis(source);
    result.clearing.moves = source.moves;
    result.clearing.primitives = source.primitives;
    result.clearing.ok = !source.moves.empty();
    result.clearing.depthLayerCount = 1;
    result.clearing.ringCount = 1;
    result.clearing.toolRadius =
        std::isfinite(source.radius) && source.radius > 0.0
        ? source.radius
        : source.tool.valid && std::isfinite(source.tool.diameter)
            ? 0.5 * std::max(0.0, source.tool.diameter)
            : 0.0;
    return result;
}

std::vector<PocketToolpathOperation> holeSimulationOperations(
    const std::vector<HoleMachiningOperation>& operations)
{
    std::vector<PocketToolpathOperation> result;
    result.reserve(operations.size());
    for (const HoleMachiningOperation& operation : operations)
        result.push_back(holeSimulationOperation(operation));
    return result;
}

} // namespace oap::toolpath

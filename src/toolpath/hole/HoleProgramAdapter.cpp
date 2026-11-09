// 把孔专用 Operation 无损投影为中立 ToolpathOperation，并保留策略元数据。
// 属于 oap_toolpath_hole 模块内部实现：输出供播放、后处理与导出链路消费。
#include "toolpath/hole/HoleProgramAdapter.h"

#include <string>

namespace oap::toolpath {
namespace {

const char* holeStrategyName(
    HoleMachiningStrategy strategy) noexcept
{
    switch (strategy) {
        case HoleMachiningStrategy::Auto:
            return "auto";
        case HoleMachiningStrategy::SmallDrill:
            return "small_drill";
        case HoleMachiningStrategy::MediumHelical:
            return "medium_helical";
        case HoleMachiningStrategy::LargeLayeredSpiral:
            return "large_layered_spiral";
    }
    return "unknown";
}

const ToolpathDrillCycle* drillCycleOf(
    const HoleMachiningOperation& operation) noexcept
{
    for (const ToolpathPrimitive& primitive : operation.primitives) {
        if (primitive.type == ToolpathPrimitiveType::DrillCycle)
            return &primitive.drillCycle;
    }
    return nullptr;
}

const char* drillCycleName(ToolpathDrillCycleType type) noexcept
{
    switch (type) {
        case ToolpathDrillCycleType::Drilling: return "g81";
        case ToolpathDrillCycleType::Dwell: return "g82";
        case ToolpathDrillCycleType::Peck: return "g83";
        case ToolpathDrillCycleType::ChipBreak: return "g73";
        case ToolpathDrillCycleType::Tapping: return "g84";
    }
    return "unknown";
}

const char* drillCycleSourceName(ToolpathDrillCycleSource source) noexcept
{
    switch (source) {
        case ToolpathDrillCycleSource::AutomaticDepthDiameterRatio:
            return "automatic_depth_diameter_ratio";
        case ToolpathDrillCycleSource::ExplicitSetting:
            return "explicit_setting";
        case ToolpathDrillCycleSource::None:
            return "none";
    }
    return "none";
}

} // namespace

ToolpathOperation toToolpathOperation(
    const HoleMachiningOperation& operation)
{
    ToolpathOperation result;
    result.operationId = operation.operationId;
    result.type = ToolpathOperationType::HoleMachining;
    result.featureKind = operation.kind;
    result.sourceIndex = operation.sourceHoleIndex;
    result.setupIndex = operation.setupIndex;
    result.setupFace = operation.setupFace;
    result.holePhase = operation.phase;
    result.tool = operation.tool;
    result.minimumCuttingEdgeLength =
        operation.minimumCuttingEdgeLength;
    if (!operation.algorithmId.empty()) {
        result.metadata.emplace(
            "algorithm_id", operation.algorithmId);
    }
    result.metadata.emplace(
        "strategy", holeStrategyName(operation.strategy));
    result.metadata.emplace(
        "phase", holeMachiningPhaseName(operation.phase));
    result.metadata.emplace(
        "source_hole_index",
        std::to_string(operation.sourceHoleIndex));
    result.metadata.emplace(
        "hole_batch_origin",
        "automatic");
    if (operation.hasCommandedBottom) {
        result.metadata.emplace(
            "drill_tip_length", std::to_string(operation.drillTipLength));
        result.metadata.emplace(
            "breakthrough_depth", std::to_string(operation.breakthroughDepth));
    }
    if (const ToolpathDrillCycle* cycle = drillCycleOf(operation)) {
        result.metadata.emplace("drill_cycle", drillCycleName(cycle->type));
        result.metadata.emplace(
            "drill_cycle_source", drillCycleSourceName(cycle->source));
        result.metadata.emplace(
            "nominal_depth", std::to_string(cycle->nominalDepth));
        result.metadata.emplace(
            "depth_diameter_ratio",
            std::to_string(cycle->depthDiameterRatio));
    }
    result.moves = operation.moves;
    result.primitives = operation.primitives;
    return result;
}

ToolpathProgram makeToolpathProgram(
    const std::vector<HoleMachiningOperation>& operations)
{
    ToolpathProgram program;
    appendToolpathOperations(program, operations);
    return program;
}

void appendToolpathOperations(
    ToolpathProgram& program,
    const std::vector<HoleMachiningOperation>& operations)
{
    program.operations.reserve(
        program.operations.size() + operations.size());
    for (const HoleMachiningOperation& operation : operations) {
        program.operations.push_back(
            toToolpathOperation(operation));
    }
}

} // namespace oap::toolpath

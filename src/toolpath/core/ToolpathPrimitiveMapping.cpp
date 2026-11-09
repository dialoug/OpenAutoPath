// ToolpathPrimitiveMapping 实现（oap_toolpath_core）。
// 把与控制器无关的 move 流映射回语义原语，并生成 G0/G1 与 G81~G84 等
// 展示/诊断用文本摘要；仅作元数据，不改变校验、仿真或后处理使用的可执行
// move 顺序。参见 ToolpathPrimitiveMapping.h。
#include "toolpath/core/ToolpathPrimitiveMapping.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace oap::toolpath {
namespace {

const char* moveCommandName(ToolpathMoveType type) noexcept
{
    return type == ToolpathMoveType::Rapid ? "G0" : "G1";
}

const char* drillCommandName(ToolpathDrillCycleType type) noexcept
{
    switch (type) {
        case ToolpathDrillCycleType::Drilling: return "G81";
        case ToolpathDrillCycleType::Dwell: return "G82";
        case ToolpathDrillCycleType::Peck: return "G83";
        case ToolpathDrillCycleType::ChipBreak: return "G73";
        case ToolpathDrillCycleType::Tapping: return "G84";
    }
    return "G81";
}

void appendPoint(std::ostringstream& output, const gp_Pnt& point)
{
    output << " X" << point.X()
           << " Y" << point.Y()
           << " Z" << point.Z();
}

std::string moveSummary(const ToolpathMove& move)
{
    std::ostringstream output;
    output << std::fixed << std::setprecision(3)
           << moveCommandName(move.type);
    appendPoint(output, move.to);
    if (move.type != ToolpathMoveType::Rapid && move.feed > 0.0)
        output << " F" << move.feed;
    return output.str();
}

std::string primitiveSummary(const ToolpathPrimitive& primitive)
{
    std::ostringstream output;
    output << std::fixed << std::setprecision(3)
           << toolpathPrimitiveCommandName(primitive);
    if (primitive.type == ToolpathPrimitiveType::Linear) {
        appendPoint(output, primitive.linear.to);
        if (primitive.linear.type != ToolpathMoveType::Rapid &&
            primitive.linear.feed > 0.0) {
            output << " F" << primitive.linear.feed;
        }
        return output.str();
    }
    if (primitive.type == ToolpathPrimitiveType::CircularArc) {
        const ToolpathCircularArc& arc = primitive.arc;
        appendPoint(output, arc.to);
        output << " I" << arc.center.X() - arc.from.X()
               << " J" << arc.center.Y() - arc.from.Y()
               << " K" << arc.center.Z() - arc.from.Z();
        if (arc.feed > 0.0)
            output << " F" << arc.feed;
        return output.str();
    }

    const ToolpathDrillCycle& cycle = primitive.drillCycle;
    output << " X" << cycle.entry.X()
           << " Y" << cycle.entry.Y()
           << " Z" << cycle.bottom.Z()
           << " R" << cycle.retract.Z();
    if ((cycle.type == ToolpathDrillCycleType::Peck ||
         cycle.type == ToolpathDrillCycleType::ChipBreak) &&
        cycle.peckDepth > 0.0) {
        output << " Q" << cycle.peckDepth;
    }
    if (cycle.dwellSeconds > 0.0)
        output << " P" << cycle.dwellSeconds;
    if (cycle.feed > 0.0)
        output << " F" << cycle.feed;
    output << (cycle.retractMode == ToolpathDrillRetractMode::InitialPlane
                   ? " G98" : " G99");
    return output.str();
}

std::size_t explicitExpansionCount(const ToolpathPrimitive& primitive)
{
    if (!primitive.fallbackMoves.empty())
        return primitive.fallbackMoves.size();
    return primitive.type == ToolpathPrimitiveType::Linear ? 1u : 0u;
}

std::size_t inferArcExpansionCount(
    const std::vector<ToolpathMove>& moves,
    std::size_t cursor,
    const ToolpathCircularArc& arc,
    double tolerance)
{
    if (cursor >= moves.size())
        return 0u;

    if (moves[cursor].from.Distance(arc.from) > tolerance)
        return 0u;

    for (std::size_t end = cursor; end < moves.size(); ++end) {
        if (moves[end].to.Distance(arc.to) <= tolerance)
            return end - cursor + 1u;
    }
    return 0u;
}

void assignPrimitive(
    std::vector<ToolpathMoveSemantic>& mapping,
    std::size_t begin,
    std::size_t count,
    int primitiveIndex,
    const ToolpathPrimitive& primitive)
{
    if (count == 0u || begin >= mapping.size())
        return;
    count = std::min(count, mapping.size() - begin);
    const std::string command = toolpathPrimitiveCommandName(primitive);
    const std::string summary = primitiveSummary(primitive);
    for (std::size_t offset = 0; offset < count; ++offset) {
        ToolpathMoveSemantic& semantic = mapping[begin + offset];
        semantic.primitiveIndex = primitiveIndex;
        semantic.primitiveMoveIndex = static_cast<int>(offset);
        semantic.primitiveMoveCount = static_cast<int>(count);
        semantic.primitiveType = primitive.type;
        semantic.commandName = command;
        semantic.commandSummary = summary;
    }
}

} // namespace

const char* toolpathPrimitiveCommandName(
    const ToolpathPrimitive& primitive) noexcept
{
    switch (primitive.type) {
        case ToolpathPrimitiveType::Linear:
            return moveCommandName(primitive.linear.type);
        case ToolpathPrimitiveType::CircularArc:
            return primitive.arc.direction == ToolpathArcDirection::Clockwise
                ? "G2" : "G3";
        case ToolpathPrimitiveType::DrillCycle:
            return drillCommandName(primitive.drillCycle.type);
    }
    return "G1";
}

std::vector<ToolpathMoveSemantic> mapToolpathMovesToPrimitives(
    const std::vector<ToolpathMove>& moves,
    const std::vector<ToolpathPrimitive>& primitives,
    double pointTolerance)
{
    std::vector<ToolpathMoveSemantic> mapping(moves.size());
    for (std::size_t index = 0; index < moves.size(); ++index) {
        mapping[index].commandName = moveCommandName(moves[index].type);
        mapping[index].commandSummary = moveSummary(moves[index]);
    }
    if (primitives.empty())
        return mapping;

    const double tolerance = std::isfinite(pointTolerance)
        ? std::max(1.0e-12, pointTolerance) : 1.0e-7;
    std::size_t cursor = 0u;
    for (std::size_t primitiveIndex = 0;
         primitiveIndex < primitives.size() && cursor < moves.size();
         ++primitiveIndex) {
        const ToolpathPrimitive& primitive = primitives[primitiveIndex];
        std::size_t count = explicitExpansionCount(primitive);
        if (count == 0u &&
            primitive.type == ToolpathPrimitiveType::CircularArc) {
            count = inferArcExpansionCount(
                moves, cursor, primitive.arc, tolerance);
        }
        if (count == 0u)
            break;

        assignPrimitive(
            mapping, cursor, count,
            static_cast<int>(primitiveIndex), primitive);
        cursor = std::min(moves.size(), cursor + count);
    }
    return mapping;
}

} // namespace oap::toolpath

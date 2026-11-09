// Maps the controller-independent move stream back to semantic primitives.
// The mapping is diagnostic/presentation metadata only and never changes the
// executable move order used by validation, simulation, or post processing.
#pragma once

#include <string>
#include <vector>

#include "toolpath/core/ToolpathMoves.h"
#include "toolpath/core/ToolpathPrimitives.h"

namespace oap::toolpath {

// move 到语义原语的映射元数据：原语索引/类型与命令文本摘要。
struct ToolpathMoveSemantic {
    int primitiveIndex = -1;
    int primitiveMoveIndex = -1;
    int primitiveMoveCount = 0;
    ToolpathPrimitiveType primitiveType = ToolpathPrimitiveType::Linear;
    std::string commandName;
    std::string commandSummary;
};

// Returns exactly moves.size() entries. Unmapped moves retain a safe G0/G1
// fallback. Non-linear primitives with fallbackMoves map directly; analytic
// arcs without fallbacks are recovered from their ordered endpoints.
std::vector<ToolpathMoveSemantic> mapToolpathMovesToPrimitives(
    const std::vector<ToolpathMove>& moves,
    const std::vector<ToolpathPrimitive>& primitives,
    double pointTolerance = 1.0e-7);

const char* toolpathPrimitiveCommandName(
    const ToolpathPrimitive& primitive) noexcept;

} // namespace oap::toolpath

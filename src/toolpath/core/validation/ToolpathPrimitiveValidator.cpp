// 验证原语在转换为具体控制器命令前具有有限、非退化的几何和工艺参数。
#include "toolpath/core/validation/ToolpathPrimitiveValidator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <gp_Vec.hxx>
#include "toolpath/core/ToolpathParallel.h"

#include "toolpath/core/validation/ToolpathMoveValidator.h"

namespace oap::toolpath {
namespace {

bool finiteVector(const gp_Vec& vector)
{
    return std::isfinite(vector.X()) && std::isfinite(vector.Y()) &&
           std::isfinite(vector.Z());
}

void addPrimitiveError(ToolpathValidationReport& report,
                       const char* code,
                       const char* message,
                       const std::string& operationId,
                       int primitiveIndex)
{
    addToolpathIssue(report, ToolpathIssueSeverity::Error, code, message,
                     operationId, primitiveIndex);
}

bool sameScalar(double first, double second)
{
    const double scale = std::max(
        {1.0, std::abs(first), std::abs(second)});
    return std::abs(first - second) <= 1.0e-9 * scale;
}

bool sameMove(const ToolpathMove& first,
              const ToolpathMove& second)
{
    return first.type == second.type &&
           first.from.Distance(second.from) <= 1.0e-9 &&
           first.to.Distance(second.to) <= 1.0e-9 &&
           first.loopSequence == second.loopSequence &&
           first.depthIndex == second.depthIndex &&
           first.ring == second.ring &&
           sameScalar(first.feed, second.feed) &&
           first.intent == second.intent;
}

bool sameContext(const ToolpathPrimitiveContext& context,
                 const ToolpathMove& move)
{
    return context.loopSequence == move.loopSequence &&
           context.depthIndex == move.depthIndex &&
           context.ring == move.ring &&
           context.intent == move.intent;
}

constexpr std::size_t kParallelPrimitiveThreshold = 8192;
constexpr std::size_t kMinimumPrimitivesPerChunk = 2048;

std::size_t primitiveChunkCount(std::size_t itemCount)
{
    if (itemCount < kParallelPrimitiveThreshold)
        return 1u;
    const std::size_t bySize =
        (itemCount + kMinimumPrimitivesPerChunk - 1u) /
        kMinimumPrimitivesPerChunk;
    const std::size_t processors = toolpathHardwareParallelism();
    return std::max<std::size_t>(
        1u, std::min(bySize, processors));
}

void validatePrimitiveRange(
    const std::vector<ToolpathPrimitive>& primitives,
    const std::string& operationId,
    std::size_t begin,
    std::size_t end,
    ToolpathValidationReport& report)
{
    for (std::size_t index = begin; index < end; ++index) {
        const ToolpathPrimitive& primitive = primitives[index];
        const int primitiveIndex = static_cast<int>(index);
        if (primitive.type == ToolpathPrimitiveType::Linear) {
            if (!finiteToolpathPoint(primitive.linear.from) ||
                !finiteToolpathPoint(primitive.linear.to)) {
                addPrimitiveError(report, "non_finite_linear_primitive",
                                  "linear primitive contains a non-finite point",
                                  operationId, primitiveIndex);
            }
            if (!sameContext(primitive.context, primitive.linear)) {
                addPrimitiveError(
                    report, "linear_primitive_context_mismatch",
                    "linear primitive context differs from its move metadata",
                    operationId, primitiveIndex);
            }
            continue;
        }

        if (primitive.type == ToolpathPrimitiveType::CircularArc) {
            const ToolpathCircularArc& arc = primitive.arc;
            if (!finiteToolpathPoint(arc.from) ||
                !finiteToolpathPoint(arc.to) ||
                !finiteToolpathPoint(arc.center) ||
                !finiteVector(arc.axis)) {
                addPrimitiveError(report, "non_finite_arc_primitive",
                                  "circular primitive contains non-finite geometry",
                                  operationId, primitiveIndex);
                continue;
            }
            if (arc.axis.SquareMagnitude() <= 1.0e-16 ||
                arc.center.Distance(arc.from) <=
                    kMinimumToolpathMoveLength ||
                !std::isfinite(arc.sweepRadians) ||
                arc.sweepRadians <= 0.0) {
                addPrimitiveError(report, "degenerate_arc_primitive",
                                  "circular primitive has a degenerate axis, radius, or sweep",
                                  operationId, primitiveIndex);
            }
            if (!std::isfinite(arc.feed) || arc.feed < 0.0) {
                addPrimitiveError(report, "invalid_arc_feed",
                                  "circular primitive feed must be finite and non-negative",
                                  operationId, primitiveIndex);
            }
            continue;
        }

        const ToolpathDrillCycle& cycle = primitive.drillCycle;
        if (!finiteToolpathPoint(cycle.clearance) ||
            !finiteToolpathPoint(cycle.retract) ||
            !finiteToolpathPoint(cycle.entry) ||
            !finiteToolpathPoint(cycle.bottom) ||
            !finiteVector(cycle.axis)) {
            addPrimitiveError(report, "non_finite_drill_cycle",
                              "drill cycle contains non-finite geometry",
                              operationId, primitiveIndex);
            continue;
        }
        if (cycle.axis.SquareMagnitude() <= 1.0e-16 ||
            cycle.entry.Distance(cycle.bottom) <=
                kMinimumToolpathMoveLength) {
            addPrimitiveError(report, "degenerate_drill_cycle",
                              "drill cycle has a degenerate axis or depth",
                              operationId, primitiveIndex);
        }
        if (!std::isfinite(cycle.feed) || cycle.feed < 0.0 ||
            !std::isfinite(cycle.peckDepth) || cycle.peckDepth < 0.0 ||
            !std::isfinite(cycle.chipBreakRetract) ||
            cycle.chipBreakRetract < 0.0 ||
            !std::isfinite(cycle.reentryClearance) ||
            cycle.reentryClearance < 0.0 ||
            !std::isfinite(cycle.dwellSeconds) || cycle.dwellSeconds < 0.0) {
            addPrimitiveError(report, "invalid_drill_cycle_parameter",
                              "drill-cycle policy and expansion values are invalid",
                              operationId, primitiveIndex);
        }
        if (cycle.source != ToolpathDrillCycleSource::None &&
            (!std::isfinite(cycle.nominalDepth) ||
             cycle.nominalDepth <= 0.0 ||
             !std::isfinite(cycle.depthDiameterRatio) ||
             cycle.depthDiameterRatio <= 0.0)) {
            addPrimitiveError(report, "invalid_drill_cycle_ratio",
                              "planned drill cycle requires a positive nominal depth and depth/diameter ratio",
                              operationId, primitiveIndex);
        }
        if ((cycle.type == ToolpathDrillCycleType::Peck ||
             cycle.type == ToolpathDrillCycleType::ChipBreak) &&
            cycle.peckDepth <= 0.0) {
            addPrimitiveError(report, "missing_drill_peck_depth",
                              "peck and chip-break cycles require a positive peck depth",
                              operationId, primitiveIndex);
        }
    }
}

} // namespace

// 逐个检查原语的有限性、退化性和工艺参数，并把问题写入报告；
// 先并行检查各分块，再按原顺序合并，保证问题顺序稳定可测试。
void validateToolpathPrimitives(
    const std::vector<ToolpathPrimitive>& primitives,
    const std::string& operationId,
    ToolpathValidationReport& report)
{
    const std::size_t chunkCount =
        primitiveChunkCount(primitives.size());
    std::vector<ToolpathValidationReport> chunks(chunkCount);
    const auto validateChunk = [&](std::size_t chunkIndex) {
        const std::size_t begin =
            primitives.size() * chunkIndex / chunkCount;
        const std::size_t end =
            primitives.size() * (chunkIndex + 1u) / chunkCount;
        validatePrimitiveRange(
            primitives, operationId, begin, end,
            chunks[chunkIndex]);
    };
    toolpathParallelForBudgeted(chunkCount, validateChunk);
    for (ToolpathValidationReport& chunk : chunks) {
        report.append(std::move(chunk));
    }
}

// 验证原语展开后的移动流与 moves 一一对应：先按类型统计每个原语
// 的展开长度（线性无 fallback 记为 1），再并行逐条比较几何/元数据。
// requireCompleteExpansion=true 时非线原语缺少 fallback 直接报错。
void validateToolpathPrimitiveExpansion(
    const std::vector<ToolpathPrimitive>& primitives,
    const std::vector<ToolpathMove>& moves,
    const std::string& operationId,
    ToolpathValidationReport& report,
    bool requireCompleteExpansion)
{
    if (primitives.empty())
        return;

    // Record where each primitive expands into the original move stream.
    // The former implementation copied every fallback move into a second
    // vector before comparing it, which is costly for very large pockets.
    std::vector<std::size_t> moveOffsets(
        primitives.size() + 1u, 0u);
    bool complete = true;
    for (std::size_t index = 0; index < primitives.size(); ++index) {
        const ToolpathPrimitive& primitive = primitives[index];
        std::size_t expansionCount = 0u;
        if (primitive.type == ToolpathPrimitiveType::Linear) {
            expansionCount = primitive.fallbackMoves.empty()
                ? 1u
                : primitive.fallbackMoves.size();
        } else if (primitive.fallbackMoves.empty()) {
            complete = false;
            if (requireCompleteExpansion) {
                addPrimitiveError(
                    report, "missing_primitive_fallback",
                    "non-linear primitive has no controller-independent fallback moves",
                    operationId, static_cast<int>(index));
            }
        } else {
            expansionCount = primitive.fallbackMoves.size();
        }
        moveOffsets[index + 1u] =
            moveOffsets[index] + expansionCount;
    }
    if (!complete)
        return;

    if (moveOffsets.back() != moves.size()) {
        addPrimitiveError(
            report, "primitive_expansion_size_mismatch",
            "expanded primitive count differs from the move stream",
            operationId, static_cast<int>(moveOffsets.back()));
        return;
    }

    const std::size_t chunkCount =
        primitiveChunkCount(primitives.size());
    std::vector<std::size_t> firstMismatch(
        chunkCount, std::numeric_limits<std::size_t>::max());
    const auto compareChunk = [&](std::size_t chunkIndex) {
        const std::size_t begin =
            primitives.size() * chunkIndex / chunkCount;
        const std::size_t end =
            primitives.size() * (chunkIndex + 1u) / chunkCount;
        for (std::size_t primitiveIndex = begin;
             primitiveIndex < end; ++primitiveIndex) {
            const ToolpathPrimitive& primitive =
                primitives[primitiveIndex];
            const std::size_t moveOffset =
                moveOffsets[primitiveIndex];
            if (primitive.type == ToolpathPrimitiveType::Linear &&
                primitive.fallbackMoves.empty()) {
                if (!sameMove(
                        primitive.linear,
                        moves[moveOffset])) {
                    firstMismatch[chunkIndex] = moveOffset;
                    return;
                }
                continue;
            }
            for (std::size_t fallbackIndex = 0;
                 fallbackIndex < primitive.fallbackMoves.size();
                 ++fallbackIndex) {
                const std::size_t moveIndex =
                    moveOffset + fallbackIndex;
                if (!sameMove(
                        primitive.fallbackMoves[fallbackIndex],
                        moves[moveIndex])) {
                    firstMismatch[chunkIndex] = moveIndex;
                    return;
                }
            }
        }
    };
    toolpathParallelForBudgeted(chunkCount, compareChunk);

    const std::size_t mismatch = *std::min_element(
        firstMismatch.begin(), firstMismatch.end());
    if (mismatch != std::numeric_limits<std::size_t>::max()) {
        addPrimitiveError(
            report, "primitive_expansion_move_mismatch",
            "expanded primitive differs from the corresponding move",
            operationId, static_cast<int>(mismatch));
    }
}

} // namespace oap::toolpath

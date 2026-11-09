// 对中立移动序列检查有限坐标、连续性、非退化长度和基本进退刀结构。
#include "toolpath/core/validation/ToolpathMoveValidator.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "toolpath/core/ToolpathParallel.h"

namespace oap::toolpath {

namespace {

constexpr std::size_t kParallelMoveThreshold = 8192;
constexpr std::size_t kMinimumMovesPerChunk = 2048;

struct MoveValidationChunk {
    ToolpathMoveStats stats;
    ToolpathValidationReport report;
};

void validateMoveRange(
    const std::vector<ToolpathMove>& moves,
    const std::string& operationId,
    std::size_t begin,
    std::size_t end,
    MoveValidationChunk& result)
{
    for (std::size_t index = begin; index < end; ++index) {
        const ToolpathMove& move = moves[index];
        switch (move.type) {
            case ToolpathMoveType::Rapid: ++result.stats.rapid; break;
            case ToolpathMoveType::Plunge: ++result.stats.plunge; break;
            case ToolpathMoveType::Cut: ++result.stats.cut; break;
            case ToolpathMoveType::Retract: ++result.stats.retract; break;
        }

        if (!finiteToolpathPoint(move.from) ||
            !finiteToolpathPoint(move.to)) {
            addToolpathIssue(
                result.report, ToolpathIssueSeverity::Error,
                "non_finite_move",
                "move contains non-finite coordinates",
                operationId, static_cast<int>(index));
            continue;
        }

        if (move.from.Distance(move.to) <=
            kMinimumToolpathMoveLength) {
            addToolpathIssue(
                result.report, ToolpathIssueSeverity::Error,
                "degenerate_move",
                "move length is zero or near zero",
                operationId, static_cast<int>(index));
        }

        if ((move.type == ToolpathMoveType::Cut ||
             move.type == ToolpathMoveType::Plunge) &&
            move.feed <= 0.0) {
            addToolpathIssue(
                result.report, ToolpathIssueSeverity::Warning,
                "missing_feed",
                "cutting/plunge move has no positive feed rate",
                operationId, static_cast<int>(index));
        }

        // Each chunk reads the item immediately before its range.  This keeps
        // the cross-chunk continuity check identical to the serial scan.
        if (index > 0 &&
            moves[index - 1].to.Distance(move.from) > 1.0e-6) {
            addToolpathIssue(
                result.report, ToolpathIssueSeverity::Warning,
                "move_gap",
                "move starts away from previous move end; exporter will insert a rapid positioning move",
                operationId, static_cast<int>(index));
        }
    }
}

std::size_t validationChunkCount(std::size_t itemCount)
{
    if (itemCount < kParallelMoveThreshold)
        return 1u;
    const std::size_t bySize =
        (itemCount + kMinimumMovesPerChunk - 1u) /
        kMinimumMovesPerChunk;
    const std::size_t processors = toolpathHardwareParallelism();
    return std::max<std::size_t>(
        1u, std::min(bySize, processors));
}

} // namespace

bool finiteToolpathPoint(const gp_Pnt& point)
{
    return std::isfinite(point.X()) &&
           std::isfinite(point.Y()) &&
           std::isfinite(point.Z());
}

// 并行分块策略：小序列串行；大序列按最少 2048 条/块切分，块数不超过
// 逻辑核数。每个块独立检查，末段统一合并统计与问题报告，保证与串行
// 扫描完全一致（跨块连续性通过各块读取前一条移动实现）。
ToolpathMoveStats validateToolpathMoves(
    const std::vector<ToolpathMove>& moves,
    const std::string& operationId,
    ToolpathValidationReport& report)
{
    ToolpathMoveStats stats;
    if (moves.empty()) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Error, "empty_moves",
            "operation has no toolpath moves", operationId);
        return stats;
    }

    const std::size_t chunkCount = validationChunkCount(moves.size());
    std::vector<MoveValidationChunk> chunks(chunkCount);
    const auto validateChunk = [&](std::size_t chunkIndex) {
        const std::size_t begin =
            moves.size() * chunkIndex / chunkCount;
        const std::size_t end =
            moves.size() * (chunkIndex + 1u) / chunkCount;
        validateMoveRange(
            moves, operationId, begin, end,
            chunks[chunkIndex]);
    };
    toolpathParallelForBudgeted(chunkCount, validateChunk);

    for (MoveValidationChunk& chunk : chunks) {
        stats.rapid += chunk.stats.rapid;
        stats.plunge += chunk.stats.plunge;
        stats.cut += chunk.stats.cut;
        stats.retract += chunk.stats.retract;
        report.append(std::move(chunk.report));
    }

    if (stats.cut == 0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Error, "missing_cut",
            "operation has no cutting moves", operationId);
    }
    if (stats.plunge == 0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "missing_plunge",
            "operation has no plunge moves", operationId);
    }
    if (stats.retract == 0) {
        addToolpathIssue(
            report, ToolpathIssueSeverity::Warning,
            "missing_retract",
            "operation has no retract moves", operationId);
    }
    return stats;
}

} // namespace oap::toolpath

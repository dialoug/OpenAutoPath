// Concentric offset clearing. Each ring is evaluated from the original area
// with an absolute delta, so independent rings can use a bounded parallel batch.
#include "toolpath/pocket/planning/detail/PocketPatterns.h"

#include <algorithm>
#include <exception>
#include <iterator>
#include <utility>

#include "toolpath/core/ToolpathParallel.h"

namespace oap::toolpath::detail {

namespace {

struct OffsetRingResult {
    std::vector<ClearingLoop> loops;
    std::exception_ptr failure;
};

OffsetRingResult generateOffsetRing(
    const PathsD& region2d,
    double delta,
    int ring,
    bool reverse,
    int precision,
    double arcTolerance)
{
    OffsetRingResult result;
    try {
        PathsD offset = Clipper2Lib::InflatePaths(
            region2d, -delta, Clipper2Lib::JoinType::Round,
            Clipper2Lib::EndType::Polygon, 2.0, precision, arcTolerance);
        if (offset.empty())
            return result;

        offset = cleanPaths(offset, precision);
        if (offset.empty())
            return result;
        offset = cleanPaths(
            Clipper2Lib::Union(
                offset, Clipper2Lib::FillRule::NonZero, precision),
            precision);
        if (offset.empty())
            return result;

        const std::vector<PathInfo> infos = sortedPathInfos(offset);
        result.loops.reserve(infos.size());
        for (const PathInfo& info : infos) {
            ClearingLoop loop;
            loop.ring = ring;
            loop.offset = delta;
            loop.closed = true;
            loop.area = info.area;
            loop.isHole = info.area < 0.0;

            PathD outPath = info.path;
            if (reverse)
                std::reverse(outPath.begin(), outPath.end());
            loop.points.reserve(outPath.size());
            for (const PointD& point : outPath)
                loop.points.emplace_back(point.x, point.y);
            result.loops.push_back(std::move(loop));
        }
    } catch (...) {
        result.failure = std::current_exception();
    }
    return result;
}

} // namespace

std::vector<ClearingLoop> generateOffsetLoops(
    const PathsD& region2d,
    double firstDelta,
    double step,
    int maxRings,
    bool reverse,
    PocketClearingDirection expansionDirection,
    int precision,
    double arcTolerance,
    int& ringCountOut,
    const ToolpathGenerationControl* control,
    PocketPatternParallelAudit* parallelAudit)
{
    std::vector<ClearingLoop> planarLoops;
    ringCountOut = 0;
    if (maxRings <= 0)
        return planarLoops;

    // Publish in ring order after each wave. The first empty ring terminates
    // the sequence; the shared CPU-token budget bounds speculative work.
    const std::size_t maximumParallelism = toolpathParallelismFor(
        static_cast<std::size_t>(maxRings));
    ToolpathParallelLease lease(
        control,
        maximumParallelism - 1u,
        toolpathHardwareParallelism() - 1u);
    const std::size_t batchCapacity = std::min(
        maximumParallelism, lease.parallelism());

    bool exhausted = false;
    for (int batchBegin = 0;
         batchBegin < maxRings && !exhausted;) {
        if (control && control->cancelled())
            break;
        const std::size_t batchSize = std::min<std::size_t>(
            batchCapacity,
            static_cast<std::size_t>(maxRings - batchBegin));
        std::vector<OffsetRingResult> generated(batchSize);
        const auto generate = [&](std::size_t batchIndex) {
            const int ring = batchBegin +
                static_cast<int>(batchIndex);
            generated[batchIndex] = generateOffsetRing(
                region2d, firstDelta + ring * step, ring,
                reverse, precision, arcTolerance);
        };
        if (batchSize > 1u) {
            toolpathParallelFor(batchSize, generate);
            if (parallelAudit) {
                ++parallelAudit->batchCount;
                parallelAudit->taskCount += batchSize;
                parallelAudit->peakParallelism = std::max(
                    parallelAudit->peakParallelism, batchSize);
            }
        } else {
            generate(0u);
        }

        for (std::size_t batchIndex = 0u;
             batchIndex < batchSize; ++batchIndex) {
            OffsetRingResult& ringResult = generated[batchIndex];
            if (ringResult.failure)
                std::rethrow_exception(ringResult.failure);
            if (ringResult.loops.empty()) {
                exhausted = true;
                break;
            }
            planarLoops.insert(
                planarLoops.end(),
                std::make_move_iterator(ringResult.loops.begin()),
                std::make_move_iterator(ringResult.loops.end()));
            ringCountOut = batchBegin +
                static_cast<int>(batchIndex) + 1;
        }
        batchBegin += static_cast<int>(batchSize);
    }

    if (expansionDirection ==
        PocketClearingDirection::InsideToOutside) {
        std::reverse(planarLoops.begin(), planarLoops.end());
    }
    return planarLoops;
}

} // namespace oap::toolpath::detail

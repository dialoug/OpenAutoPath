// 把二维清料折线提升为世界坐标移动，并在折线之间插入安全进刀和退刀段。
#include "toolpath/pocket/planning/detail/PocketMoveAssembler.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <utility>

#include "toolpath/core/ToolpathParallel.h"
#include "PocketCurveSemanticBuilder.h"
#include "PocketEntryPlanner.h"
#include "PocketGeometryUtils.h"
#include "PocketSafetyGeometry.h"

namespace oap::toolpath::detail {

namespace {

constexpr double kMoveTolerance = 1.0e-9;
constexpr std::size_t kMaximumLinkSamples = 200000;
constexpr std::size_t kNearbySafetyCandidateLimit = 8;
constexpr char kUnsafeLinkWarning[] =
    "A same-depth pocket loop could not be linked inside the safe "
    "cutting area; a retract and re-entry were inserted";
using Clock = std::chrono::steady_clock;

double elapsedMilliseconds(const Clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(
               Clock::now() - start)
        .count();
}

template <typename Function>
std::size_t parallelFor(
    std::size_t count,
    const ToolpathGenerationControl* control,
    Function&& function)
{
    if (count == 0u)
        return 0u;
    const std::size_t maximumParallelism =
        toolpathParallelismFor(count);
    ToolpathParallelLease lease(
        control, maximumParallelism - 1u,
        toolpathHardwareParallelism() - 1u);
    const std::size_t workerCount = std::min(
        maximumParallelism, lease.parallelism());
    toolpathParallelForWorkers(
        count,
        workerCount,
        std::forward<Function>(function));
    return workerCount;
}

bool canLinkAtCutDepth(
    const gp_Pnt& from,
    const gp_Pnt& to,
    const PocketRegion& region,
    const ToolpathParams& params,
    const PocketSafetyArea& safetyArea,
    int depthIndex)
{
    // 判断能否在同一切削深度上从 from 直接走刀到 to：沿线段按刀心余量
    // 间距采样，所有采样点都必须位于安全区域内（根边界内且避开该层的
    // 材料禁入区）。用于决定层间连接是否需要抬刀重进。
    const double length = from.Distance(to);
    if (!std::isfinite(length))
        return false;
    if (length <= kMoveTolerance)
        return true;

    const double requiredMargin =
        params.tool.radius() +
        std::max(0.0, params.clearing.extraOffset);
    const double sampleSpacing = std::clamp(
        requiredMargin > 0.0 ? requiredMargin / 3.0 : 0.25,
        0.05, 0.5);
    const double requestedSamples =
        std::ceil(length / sampleSpacing);
    if (!std::isfinite(requestedSamples) ||
        requestedSamples >
            static_cast<double>(kMaximumLinkSamples)) {
        return false;
    }
    const std::size_t sampleCount = std::max<std::size_t>(
        1, static_cast<std::size_t>(requestedSamples));
    for (std::size_t sample = 0; sample <= sampleCount; ++sample) {
        const double parameter =
            static_cast<double>(sample) /
            static_cast<double>(sampleCount);
        const gp_Pnt point(
            from.X() + (to.X() - from.X()) * parameter,
            from.Y() + (to.Y() - from.Y()) * parameter,
            from.Z() + (to.Z() - from.Z()) * parameter);
        const gp_Pnt2d projected =
            projectToPocketPlane(point, region.frame);
        if (!safetyArea.clear(projected, depthIndex)) {
            return false;
        }
    }
    return true;
}

struct LoopTravelChoice {
    std::size_t relativeIndex = 0;
    std::size_t startIndex = 0;
    double distance = std::numeric_limits<double>::infinity();
};

const gp_Pnt& loopStart(const ClearingLoop& loop,
                        std::size_t startIndex)
{
    if (loop.closed)
        return loop.world[startIndex];
    return loop.world.front();
}

std::vector<LoopTravelChoice> nearbyTravelChoices(
    const ClearingLoop& loop,
    std::size_t relativeIndex,
    const gp_Pnt& current)
{
    std::vector<LoopTravelChoice> choices;
    if (loop.world.empty())
        return choices;

    if (loop.closed) {
        choices.reserve(std::min(
            kNearbySafetyCandidateLimit,
            loop.world.size()));
        for (std::size_t index = 0; index < loop.world.size(); ++index) {
            LoopTravelChoice choice;
            choice.relativeIndex = relativeIndex;
            choice.startIndex = index;
            choice.distance = current.Distance(loop.world[index]);
            const auto insertion = std::lower_bound(
                choices.begin(), choices.end(), choice,
                [](const LoopTravelChoice& left,
                   const LoopTravelChoice& right) {
                    return left.distance < right.distance;
                });
            if (insertion != choices.end() ||
                choices.size() < kNearbySafetyCandidateLimit) {
                choices.insert(insertion, choice);
                if (choices.size() > kNearbySafetyCandidateLimit)
                    choices.pop_back();
            }
        }
        return choices;
    }

    // 开放 ZigZag/Spiral 的方向由 CutMode 与径向清料方向决定，不能为
    // 缩短跳刀而反转；否则会改变用户选择的工艺语义。
    LoopTravelChoice choice;
    choice.relativeIndex = relativeIndex;
    choice.distance = current.Distance(loop.world.front());
    choices.push_back(choice);
    return choices;
}

LoopTravelChoice nearestTravelChoice(
    const ClearingLoop& loop,
    std::size_t relativeIndex,
    const gp_Pnt& current)
{
    const std::vector<LoopTravelChoice> choices =
        nearbyTravelChoices(loop, relativeIndex, current);
    if (!choices.empty())
        return choices.front();
    LoopTravelChoice empty;
    empty.relativeIndex = relativeIndex;
    return empty;
}

void applyTravelChoice(ClearingLoop& loop,
                       const LoopTravelChoice& choice,
                       PocketClearingResult& result)
{
    if (loop.closed && choice.startIndex > 0 &&
        choice.startIndex < loop.points.size() &&
        loop.world.size() == loop.points.size()) {
        std::rotate(
            loop.points.begin(),
            loop.points.begin() +
                static_cast<std::ptrdiff_t>(choice.startIndex),
            loop.points.end());
        std::rotate(
            loop.world.begin(),
            loop.world.begin() +
                static_cast<std::ptrdiff_t>(choice.startIndex),
            loop.world.end());
        ++result.travelAdjustedStartCount;
    }
}

void optimizeRestMachiningTravel(
    const PocketRegion& region,
    const ToolpathParams& params,
    const PocketSafetyArea& safetyArea,
    PocketClearingResult& result,
    const ToolpathGenerationControl* control)
{
    if (!result.restMachining || result.loops.size() < 2)
        return;

    std::vector<ClearingLoop> optimized;
    optimized.reserve(result.loops.size());
    bool hasCurrent = false;
    gp_Pnt current;
    int currentDepthIndex = -1;

    // 深度层和 ring 顺序承载 step-down 与外向内/内向外工艺语义，不能
    // 跨组交换；只对同一组里的离散余料分量做空间访问优化。
    for (std::size_t groupBegin = 0;
         groupBegin < result.loops.size();) {
        const int depthIndex = result.loops[groupBegin].depthIndex;
        const int ring = result.loops[groupBegin].ring;
        std::size_t groupEnd = groupBegin + 1;
        while (groupEnd < result.loops.size() &&
               result.loops[groupEnd].depthIndex == depthIndex &&
               result.loops[groupEnd].ring == ring) {
            ++groupEnd;
        }

        std::vector<ClearingLoop> pending;
        pending.reserve(groupEnd - groupBegin);
        for (std::size_t index = groupBegin; index < groupEnd; ++index)
            pending.push_back(std::move(result.loops[index]));

        std::vector<unsigned char> active(pending.size(), 1u);
        std::size_t remaining = pending.size();
        while (remaining > 0u) {
            LoopTravelChoice selected;
            if (!hasCurrent) {
                selected.relativeIndex = static_cast<std::size_t>(
                    std::find(active.begin(), active.end(), 1u) -
                    active.begin());
                selected.distance = 0.0;
            } else {
                std::vector<std::size_t> activeIndices;
                activeIndices.reserve(remaining);
                for (std::size_t index = 0u;
                     index < active.size(); ++index) {
                    if (active[index] != 0u)
                        activeIndices.push_back(index);
                }
                std::vector<LoopTravelChoice> choices(
                    activeIndices.size());
                const auto evaluateChoice = [&](std::size_t index) {
                    const std::size_t pendingIndex =
                        activeIndices[index];
                    choices[index] = nearestTravelChoice(
                        pending[pendingIndex], pendingIndex,
                        current);
                };
                constexpr std::size_t kParallelTravelChoiceThreshold = 16u;
                if (activeIndices.size() >=
                    kParallelTravelChoiceThreshold) {
                    const std::size_t workerCount = parallelFor(
                        activeIndices.size(), control,
                        evaluateChoice);
                    if (workerCount > 1u) {
                        ++result.travelChoiceParallelBatchCount;
                        result.travelChoiceParallelTaskCount +=
                            activeIndices.size();
                        result.travelChoicePeakParallelism = std::max(
                            result.travelChoicePeakParallelism,
                            workerCount);
                    }
                } else {
                    for (std::size_t index = 0u;
                         index < activeIndices.size(); ++index) {
                        evaluateChoice(index);
                    }
                }
                const auto choiceLess =
                    [](const LoopTravelChoice& left,
                       const LoopTravelChoice& right) {
                        if (left.distance != right.distance)
                            return left.distance < right.distance;
                        if (left.relativeIndex != right.relativeIndex) {
                            return left.relativeIndex <
                                   right.relativeIndex;
                        }
                        return left.startIndex < right.startIndex;
                    };
                const std::size_t orderedChoiceCount = std::min(
                    kNearbySafetyCandidateLimit,
                    choices.size());
                std::partial_sort(
                    choices.begin(),
                    choices.begin() + static_cast<std::ptrdiff_t>(
                        orderedChoiceCount),
                    choices.end(), choiceLess);
                choices.resize(orderedChoiceCount);
                selected = choices.front();

                if (currentDepthIndex == depthIndex) {
                    const std::size_t safetyCandidateCount =
                        std::min(
                            kNearbySafetyCandidateLimit,
                            choices.size());
                    std::vector<std::optional<LoopTravelChoice>>
                        safeChoices(safetyCandidateCount);
                    const auto evaluateSafety =
                        [&](std::size_t choiceIndex) {
                        const LoopTravelChoice& nearestCandidate =
                            choices[choiceIndex];
                        const ClearingLoop& loop =
                            pending[nearestCandidate.relativeIndex];
                        if (loop.world.empty())
                            return;
                        const std::vector<LoopTravelChoice>
                            startChoices = nearbyTravelChoices(
                                loop,
                                nearestCandidate.relativeIndex,
                                current);
                        for (const LoopTravelChoice& candidate :
                             startChoices) {
                            if (!canLinkAtCutDepth(
                                    current,
                                    loopStart(
                                        loop,
                                        candidate.startIndex),
                                    region, params, safetyArea,
                                    depthIndex)) {
                                continue;
                            }
                            safeChoices[choiceIndex] = candidate;
                            return;
                        }
                    };
                    const std::size_t safetyWorkers =
                        safetyCandidateCount > 1u
                            ? parallelFor(
                                  safetyCandidateCount,
                                  control, evaluateSafety)
                            : 1u;
                    if (safetyCandidateCount == 1u)
                        evaluateSafety(0u);
                    if (safetyWorkers > 1u) {
                        ++result.travelChoiceParallelBatchCount;
                        result.travelChoiceParallelTaskCount +=
                            safetyCandidateCount;
                        result.travelChoicePeakParallelism = std::max(
                            result.travelChoicePeakParallelism,
                            safetyWorkers);
                    }
                    bool foundSafeChoice = false;
                    for (const auto& safeChoice : safeChoices) {
                        if (!safeChoice)
                            continue;
                        if (!foundSafeChoice ||
                            safeChoice->distance + kMoveTolerance <
                                selected.distance) {
                            selected = *safeChoice;
                            foundSafeChoice = true;
                        }
                    }
                }
            }

            ClearingLoop next = std::move(
                pending[selected.relativeIndex]);
            active[selected.relativeIndex] = 0u;
            --remaining;
            if (next.sequence != static_cast<int>(optimized.size()))
                ++result.travelReorderedLoopCount;
            applyTravelChoice(next, selected, result);
            if (!next.world.empty()) {
                current = next.closed
                    ? next.world.front()
                    : next.world.back();
                currentDepthIndex = next.depthIndex;
                hasCurrent = true;
            }
            optimized.push_back(std::move(next));
        }
        groupBegin = groupEnd;
    }

    result.loops = std::move(optimized);
    for (std::size_t index = 0; index < result.loops.size(); ++index)
        result.loops[index].sequence = static_cast<int>(index);
}

bool segmentClearsProtectedMaterial(
    const gp_Pnt2d& from,
    const gp_Pnt2d& to,
    int depthIndex,
    double sampleSpacing,
    const PocketSafetyArea& safetyArea)
{
    const double length = from.Distance(to);
    if (!std::isfinite(length))
        return false;
    const double requestedSamples = std::ceil(
        length / std::max(1.0e-5, sampleSpacing));
    if (!std::isfinite(requestedSamples) ||
        requestedSamples > static_cast<double>(kMaximumLinkSamples)) {
        return false;
    }
    const std::size_t sampleCount = std::max<std::size_t>(
        1u, static_cast<std::size_t>(requestedSamples));
    for (std::size_t sample = 0u; sample <= sampleCount; ++sample) {
        const double t = static_cast<double>(sample) /
                         static_cast<double>(sampleCount);
        if (!safetyArea.protectedMaterialClear(
                gp_Pnt2d(
                    from.X() + t * (to.X() - from.X()),
                    from.Y() + t * (to.Y() - from.Y())),
                depthIndex)) {
            return false;
        }
    }
    return true;
}

std::size_t splitLoopsAtProtectedMaterial(
    PocketClearingResult& result,
    const ToolpathParams& params,
    const PocketSafetyArea& safetyArea)
{
    if (!result.usedProtectedMaterialSections || result.loops.empty())
        return 0u;

    const double requiredMargin =
        params.tool.radius() +
        std::max(0.0, params.clearing.extraOffset);
    const double sampleSpacing = std::clamp(
        requiredMargin > 0.0 ? requiredMargin / 3.0 : 0.25,
        0.05, 0.5);
    std::vector<ClearingLoop> safeLoops;
    safeLoops.reserve(result.loops.size());
    std::size_t removedSegments = 0u;

    for (ClearingLoop& loop : result.loops) {
        if (loop.points.size() < 2u ||
            loop.world.size() != loop.points.size()) {
            safeLoops.push_back(std::move(loop));
            continue;
        }

        const std::size_t segmentCount = loop.closed
            ? loop.points.size()
            : loop.points.size() - 1u;
        std::vector<unsigned char> segmentSafe(segmentCount, 0u);
        bool allSafe = true;
        for (std::size_t segment = 0u;
             segment < segmentCount; ++segment) {
            const bool safe = segmentClearsProtectedMaterial(
                loop.points[segment],
                loop.points[(segment + 1u) % loop.points.size()],
                loop.depthIndex, sampleSpacing, safetyArea);
            segmentSafe[segment] = safe ? 1u : 0u;
            if (!safe) {
                allSafe = false;
                ++removedSegments;
            }
        }
        if (allSafe) {
            safeLoops.push_back(std::move(loop));
            continue;
        }

        std::vector<gp_Pnt2d> fragmentPoints;
        std::vector<gp_Pnt> fragmentWorld;
        const auto flushFragment = [&]() {
            if (fragmentPoints.size() >= 2u) {
                ClearingLoop fragment = loop;
                fragment.points = std::move(fragmentPoints);
                fragment.world = std::move(fragmentWorld);
                fragment.closed = false;
                fragment.area = 0.0;
                fragment.preferredLinkFromPrevious.clear();
                fragment.preferredWorldLinkFromPrevious.clear();
                safeLoops.push_back(std::move(fragment));
            }
            fragmentPoints.clear();
            fragmentWorld.clear();
        };
        const auto consumeSegment = [&](std::size_t segment) {
            if (segmentSafe[segment] == 0u) {
                flushFragment();
                return;
            }
            const std::size_t next =
                (segment + 1u) % loop.points.size();
            if (fragmentPoints.empty()) {
                fragmentPoints.push_back(loop.points[segment]);
                fragmentWorld.push_back(loop.world[segment]);
            }
            fragmentPoints.push_back(loop.points[next]);
            fragmentWorld.push_back(loop.world[next]);
        };

        if (!loop.closed) {
            for (std::size_t segment = 0u;
                 segment < segmentCount; ++segment) {
                consumeSegment(segment);
            }
        } else {
            const auto firstUnsafe = std::find(
                segmentSafe.begin(), segmentSafe.end(), 0u);
            const std::size_t start =
                (static_cast<std::size_t>(std::distance(
                     segmentSafe.begin(), firstUnsafe)) + 1u) %
                segmentCount;
            for (std::size_t offset = 0u;
                 offset < segmentCount; ++offset) {
                consumeSegment((start + offset) % segmentCount);
            }
        }
        flushFragment();
    }

    result.loops = std::move(safeLoops);
    for (std::size_t index = 0u; index < result.loops.size(); ++index)
        result.loops[index].sequence = static_cast<int>(index);
    return removedSegments;
}

bool appendMove(PocketClearingResult& result,
                ToolpathMoveType type,
                const gp_Pnt& from,
                const gp_Pnt& to,
                const ClearingLoop& loop,
                double feed,
                ToolpathMoveIntent intent =
                    ToolpathMoveIntent::General,
                bool appendPrimitive = true)
{
    if (from.Distance(to) <= kMoveTolerance)
        return false;

    ToolpathMove move;
    move.type = type;
    move.from = from;
    move.to = to;
    move.loopSequence = loop.sequence;
    move.depthIndex = loop.depthIndex;
    move.ring = loop.ring;
    move.feed = feed;
    move.intent = intent;
    result.moves.push_back(move);
    if (appendPrimitive)
        result.primitives.push_back(makeLinearPrimitive(move));
    return true;
}

bool mergeCollinearCutMove(std::vector<ToolpathMove>& moves,
                           const gp_Pnt& from,
                           const gp_Pnt& to)
{
    if (moves.empty())
        return false;
    ToolpathMove& previous = moves.back();
    if (previous.to.Distance(from) > kMoveTolerance)
        return false;

    const gp_Vec first(previous.from, previous.to);
    const gp_Vec second(from, to);
    const gp_Vec combined(previous.from, to);
    const double firstLength = first.Magnitude();
    const double secondLength = second.Magnitude();
    const double combinedLength = combined.Magnitude();
    if (firstLength <= kMoveTolerance ||
        secondLength <= kMoveTolerance ||
        combinedLength <= kMoveTolerance ||
        first.Dot(second) <= 0.0) {
        return false;
    }

    // 只合并连接点到合并后直线距离不超过移动零长度容差的同向段，
    // 因而不会引入可观测的几何近似或越过安全边界。
    const double junctionDeviation =
        first.Crossed(combined).Magnitude() / combinedLength;
    if (!std::isfinite(junctionDeviation) ||
        junctionDeviation > kMoveTolerance) {
        return false;
    }
    previous.to = to;
    return true;
}

void replaceHelicalEntryPrimitives(
    PocketClearingResult& result,
    std::size_t primitiveBegin,
    const std::vector<std::size_t>& moveByFeedSegment,
    const PocketEntryPath& entry)
{
    if (entry.arcs.empty())
        return;

    constexpr std::size_t kMissingMove =
        std::numeric_limits<std::size_t>::max();
    std::vector<ToolpathPrimitive> replacement;
    replacement.reserve(moveByFeedSegment.size());
    std::size_t cursor = 0;
    const auto appendLinearRange = [&](std::size_t end) {
        for (; cursor < end; ++cursor) {
            const std::size_t moveIndex = moveByFeedSegment[cursor];
            if (moveIndex != kMissingMove)
                replacement.push_back(
                    makeLinearPrimitive(result.moves[moveIndex]));
        }
    };

    for (const PocketEntryArc& semantic : entry.arcs) {
        if (semantic.firstFeedSegment < cursor ||
            semantic.firstFeedSegment + semantic.feedSegmentCount >
                moveByFeedSegment.size()) {
            return;
        }
        appendLinearRange(semantic.firstFeedSegment);

        std::vector<ToolpathMove> fallback;
        fallback.reserve(semantic.feedSegmentCount);
        for (std::size_t offset = 0;
             offset < semantic.feedSegmentCount; ++offset) {
            const std::size_t moveIndex =
                moveByFeedSegment[semantic.firstFeedSegment + offset];
            if (moveIndex == kMissingMove)
                return;
            fallback.push_back(result.moves[moveIndex]);
        }
        if (fallback.empty() ||
            semantic.arc.from.Distance(fallback.front().from) >
                kMoveTolerance ||
            semantic.arc.to.Distance(fallback.back().to) >
                kMoveTolerance) {
            return;
        }
        const ToolpathPrimitiveContext context{
            fallback.front().loopSequence,
            fallback.front().depthIndex,
            fallback.front().ring,
            fallback.front().intent};
        replacement.push_back(makeArcPrimitive(
            semantic.arc, std::move(fallback), context));
        cursor = semantic.firstFeedSegment +
            semantic.feedSegmentCount;
    }
    appendLinearRange(moveByFeedSegment.size());

    result.primitives.resize(primitiveBegin);
    result.primitives.insert(
        result.primitives.end(),
        std::make_move_iterator(replacement.begin()),
        std::make_move_iterator(replacement.end()));
}

} // namespace

void buildPocketMoves(const PocketRegion& region,
                      const ToolpathParams& params,
                      PocketClearingResult& result,
                      const ToolpathGenerationControl* control)
{
    if (result.loops.empty())
        return;

    double highestCutOffset = result.finalDepthOffset;
    for (const ClearingLoop& loop : result.loops)
        highestCutOffset = std::max(highestCutOffset, loop.depthOffset);
    highestCutOffset = std::max(highestCutOffset, result.startDepthOffset);

    // PocketRegion 的局部深度坐标以腔底为 0，region.depth 对应刀具进入
    // 毛坯的开口面。用户配置的安全高度和清空高度则以该下刀面为 0；
    // 先转换成底面局部坐标，避免不同深度的特征产生不同的绝对抬刀高度。
    const double openingOffset =
        std::isfinite(region.depth)
            ? std::max(0.0, region.depth)
            : highestCutOffset;
    const double safeDistance =
        std::isfinite(params.depth.safeHeight)
            ? std::max(0.0, params.depth.safeHeight)
            : 0.0;
    const double clearanceDistance =
        std::isfinite(params.depth.clearanceHeight)
            ? std::max(safeDistance, params.depth.clearanceHeight)
            : safeDistance;

    const double safeOffset = std::max(
        openingOffset + safeDistance, highestCutOffset);
    const double clearanceOffset = std::max({
        openingOffset + clearanceDistance,
        safeOffset,
        highestCutOffset});
    result.safeHeightOffset = safeOffset;
    result.clearanceHeightOffset = clearanceOffset;

    // 斜坡高度基准可选上一已清除层、槽腔开口面或安全面；默认上一层，
    // 避免深腔最终层从开口面重复生成超长斜坡。
    std::map<int, double> layerOffsets;
    for (const ClearingLoop& loop : result.loops)
        layerOffsets.try_emplace(loop.depthIndex, loop.depthOffset);
    std::map<int, double> rampStartOffsets;
    double previousLayerOffset = result.startDepthOffset;
    for (const auto& [depthIndex, depthOffset] : layerOffsets) {
        double rampStartOffset = previousLayerOffset;
        switch (params.clearing.rampStartReference) {
            case PocketRampStartReference::PreviousLevel:
                break;
            case PocketRampStartReference::OpeningPlane:
                rampStartOffset = openingOffset;
                break;
            case PocketRampStartReference::SafePlane:
                rampStartOffset = safeOffset;
                break;
        }
        rampStartOffsets.emplace(depthIndex, rampStartOffset);
        previousLayerOffset = depthOffset;
    }

    bool hasCurrentClearance = false;
    gp_Pnt currentClearance;
    bool toolAtCutDepth = false;
    gp_Pnt currentCutPoint;
    gp_Pnt2d currentCutPoint2d;
    int currentDepthIndex = -1;
    const ClearingLoop* currentLoop = nullptr;

    const Clock::time_point preparationStart = Clock::now();
    const PocketSafetyArea safetyArea(
        region, params, result.protectedMaterialLayers);
    const std::size_t removedUnsafeSegments =
        splitLoopsAtProtectedMaterial(result, params, safetyArea);
    if (removedUnsafeSegments > 0u) {
        appendWarning(
            result.warning,
            "Pocket pattern segments inside the protected-material "
            "clearance envelope were omitted");
    }
    if (result.loops.empty())
        return;
    optimizeRestMachiningTravel(
        region, params, safetyArea, result, control);

    struct PreparedLoopToolpath {
        std::vector<ToolpathMove> moves;
        std::vector<ToolpathPrimitive> primitives;
        PocketEntryPath entry;
        std::size_t collinearMergedMoveCount = 0;
    };
    std::vector<PreparedLoopToolpath> preparedLoops(
        result.loops.size());
    parallelFor(
        result.loops.size(),
        control,
        [&](std::size_t loopIndex) {
            const ClearingLoop& loop = result.loops[loopIndex];
            PreparedLoopToolpath& prepared =
                preparedLoops[loopIndex];
            if (loop.world.empty())
                return;
            const std::size_t segmentCount = loop.closed
                ? loop.world.size()
                : loop.world.size() - 1;
            prepared.moves.reserve(segmentCount);
            for (std::size_t segment = 0;
                 segment < segmentCount; ++segment) {
                const gp_Pnt& from = loop.world[segment];
                const gp_Pnt& to = loop.world[
                    (segment + 1) % loop.world.size()];
                if (from.Distance(to) <= kMoveTolerance)
                    continue;
                if (mergeCollinearCutMove(
                        prepared.moves, from, to)) {
                    ++prepared.collinearMergedMoveCount;
                    continue;
                }
                ToolpathMove move;
                move.type = ToolpathMoveType::Cut;
                move.from = from;
                move.to = to;
                move.loopSequence = loop.sequence;
                move.depthIndex = loop.depthIndex;
                move.ring = loop.ring;
                move.feed = params.tool.horizFeed;
                prepared.moves.push_back(std::move(move));
            }
            prepared.primitives = buildPocketCurvePrimitives(
                region, params, safetyArea, loop,
                prepared.moves, 0, prepared.moves.size());
            if (prepared.primitives.empty()) {
                appendLinearPrimitives(
                    prepared.primitives, prepared.moves,
                    0, prepared.moves.size());
            }
            const auto rampStart =
                rampStartOffsets.find(loop.depthIndex);
            const double rampStartOffset =
                rampStart != rampStartOffsets.end()
                    ? rampStart->second
                    : safeOffset;
            prepared.entry = planPocketEntry(
                region, params, loop, safeOffset,
                rampStartOffset, &safetyArea);
        });

    // 访问顺序和闭环起点已经固定后，每对相邻环的切深直连判断互不
    // 依赖。提前并行计算，主组装循环只按稳定索引消费结果。
    std::vector<unsigned char> directCutLinks(
        result.loops.size(), 0u);
    std::vector<unsigned char> preferredCutLinks(
        result.loops.size(), 0u);
    if (result.loops.size() > 1u) {
        const std::size_t workerCount = parallelFor(
            result.loops.size() - 1u, control,
            [&](std::size_t index) {
                const std::size_t loopIndex = index + 1u;
                const ClearingLoop& previous =
                    result.loops[loopIndex - 1u];
                const ClearingLoop& loop =
                    result.loops[loopIndex];
                if (previous.world.empty() || loop.world.empty() ||
                    previous.depthIndex != loop.depthIndex ||
                    std::abs(previous.depthOffset -
                             loop.depthOffset) > kMoveTolerance) {
                    return;
                }
                const gp_Pnt& previousEnd = previous.closed
                    ? previous.world.front()
                    : previous.world.back();
                const auto& preferred =
                    loop.preferredWorldLinkFromPrevious;
                if (preferred.size() >= 2u &&
                    preferred.front().Distance(previousEnd) <=
                        kMoveTolerance &&
                    preferred.back().Distance(loop.world.front()) <=
                        kMoveTolerance) {
                    bool preferredSafe = true;
                    for (std::size_t pointIndex = 1u;
                         pointIndex < preferred.size(); ++pointIndex) {
                        if (!canLinkAtCutDepth(
                                preferred[pointIndex - 1u],
                                preferred[pointIndex],
                                region, params, safetyArea,
                                loop.depthIndex)) {
                            preferredSafe = false;
                            break;
                        }
                    }
                    if (preferredSafe) {
                        preferredCutLinks[loopIndex] = 1u;
                        return;
                    }
                }
                directCutLinks[loopIndex] =
                    canLinkAtCutDepth(
                        previousEnd, loop.world.front(),
                        region, params, safetyArea,
                        loop.depthIndex)
                        ? 1u
                        : 0u;
            });
        if (workerCount > 1u) {
            ++result.travelChoiceParallelBatchCount;
            result.travelChoiceParallelTaskCount +=
                result.loops.size() - 1u;
            result.travelChoicePeakParallelism = std::max(
                result.travelChoicePeakParallelism,
                workerCount);
        }
    }

    std::size_t estimatedMoveCount = 0;
    std::size_t preparedPrimitiveCount = 0;
    for (std::size_t loopIndex = 0;
         loopIndex < result.loops.size(); ++loopIndex) {
        const PreparedLoopToolpath& prepared =
            preparedLoops[loopIndex];
        estimatedMoveCount += prepared.moves.size();
        preparedPrimitiveCount +=
            prepared.primitives.size();
        result.collinearMergedMoveCount +=
            prepared.collinearMergedMoveCount;
    }
    result.moves.reserve(result.moves.size() + estimatedMoveCount +
                         result.loops.size() * 4u);
    result.primitives.reserve(
        result.primitives.size() + preparedPrimitiveCount +
        result.loops.size() * 4u);
    result.movePreparationMilliseconds =
        elapsedMilliseconds(preparationStart);
    const Clock::time_point linkingStart = Clock::now();

    const auto retractCurrentCut = [&] {
        if (!toolAtCutDepth || currentLoop == nullptr)
            return;
        const gp_Pnt safePoint = pointFrom2dAtDepth(
            currentCutPoint2d, region.frame, safeOffset);
        const gp_Pnt clearancePoint = pointFrom2dAtDepth(
            currentCutPoint2d, region.frame, clearanceOffset);
        appendMove(result, ToolpathMoveType::Retract,
                   currentCutPoint, safePoint, *currentLoop,
                   params.tool.vertFeed);
        appendMove(result, ToolpathMoveType::Retract,
                   safePoint, clearancePoint, *currentLoop,
                   params.tool.vertRapid);
        currentClearance = clearancePoint;
        hasCurrentClearance = true;
        toolAtCutDepth = false;
        currentLoop = nullptr;
    };

    // 主组装循环：逐环决策“同深度安全直连 还是 抬刀重进”。
    // 直连时在切深处补一段 Cut 连接；否则按深度层规划进刀（垂直/斜坡），
    // 并把预先并行生成的该环切削移动段与语义原语插入结果流。
    for (std::size_t loopIndex = 0;
         loopIndex < result.loops.size(); ++loopIndex) {
        const ClearingLoop& loop = result.loops[loopIndex];
        if (loop.points.empty() || loop.world.empty())
            continue;

        const gp_Pnt cutStart = loop.world.front();
        const gp_Pnt cutEnd = loop.closed ? cutStart : loop.world.back();
        const gp_Pnt2d& end2d =
            loop.closed ? loop.points.front() : loop.points.back();
        const bool sameDepth =
            toolAtCutDepth && currentDepthIndex == loop.depthIndex &&
            currentLoop != nullptr &&
            std::abs(currentLoop->depthOffset - loop.depthOffset) <=
                kMoveTolerance;
        const bool previousIsAdjacent =
            loopIndex > 0u && currentLoop ==
                &result.loops[loopIndex - 1u];
        const bool usePreferredLink =
            sameDepth && previousIsAdjacent &&
            preferredCutLinks[loopIndex] != 0u;
        const bool linkAtCutDepth = sameDepth &&
            (usePreferredLink ||
             (previousIsAdjacent
                 ? directCutLinks[loopIndex] != 0u
                 : canLinkAtCutDepth(
                       currentCutPoint, cutStart, region,
                       params, safetyArea,
                       loop.depthIndex)));

        if (linkAtCutDepth) {
            if (usePreferredLink) {
                const auto& preferred =
                    loop.preferredWorldLinkFromPrevious;
                for (std::size_t pointIndex = 1u;
                     pointIndex < preferred.size(); ++pointIndex) {
                    appendMove(
                        result, ToolpathMoveType::Cut,
                        preferred[pointIndex - 1u],
                        preferred[pointIndex], loop,
                        params.tool.horizFeed);
                }
            } else {
                appendMove(result, ToolpathMoveType::Cut,
                           currentCutPoint, cutStart, loop,
                           params.tool.horizFeed);
            }
        } else {
            if (sameDepth &&
                result.warning.find(kUnsafeLinkWarning) ==
                    std::string::npos) {
                appendWarning(result.warning, kUnsafeLinkWarning);
            }

            const PocketEntryPath& entry =
                preparedLoops[loopIndex].entry;
            if (entry.feedPoints.size() < 2)
                continue;

            retractCurrentCut();
            switch (entry.usedMode) {
                case PocketEntryMode::VerticalPlunge:
                    ++result.verticalEntryCount;
                    break;
                case PocketEntryMode::RampOnShape:
                    ++result.rampOnShapeEntryCount;
                    break;
                case PocketEntryMode::LinearRamp:
                    ++result.linearRampEntryCount;
                    break;
                case PocketEntryMode::ZigZagRamp:
                    ++result.zigzagRampEntryCount;
                    break;
                case PocketEntryMode::HelicalRamp:
                    ++result.helicalRampEntryCount;
                    break;
            }
            if (entry.fallback)
                ++result.entryFallbackCount;
            if (!entry.warning.empty() &&
                result.warning.find(entry.warning) ==
                    std::string::npos) {
                appendWarning(result.warning, entry.warning);
            }

            const gp_Pnt clearancePoint =
                pointFrom2dAtDepth(
                    entry.startPoint, region.frame,
                    clearanceOffset);
            const gp_Pnt& safePoint = entry.feedPoints.front();

            if (hasCurrentClearance) {
                appendMove(result, ToolpathMoveType::Rapid,
                           currentClearance, clearancePoint, loop,
                           params.tool.horizRapid);
            }

            appendMove(result, ToolpathMoveType::Rapid,
                       clearancePoint, safePoint, loop,
                       params.tool.vertRapid);
            ToolpathMoveIntent entryIntent =
                ToolpathMoveIntent::General;
            if (entry.usedMode == PocketEntryMode::RampOnShape ||
                entry.usedMode == PocketEntryMode::LinearRamp ||
                entry.usedMode == PocketEntryMode::ZigZagRamp) {
                entryIntent =
                    ToolpathMoveIntent::
                        PocketLinearRampEntry;
            } else if (
                entry.usedMode ==
                PocketEntryMode::HelicalRamp) {
                entryIntent =
                    ToolpathMoveIntent::
                        PocketHelicalRampEntry;
            }
            const std::size_t entryPrimitiveBegin =
                result.primitives.size();
            constexpr std::size_t kMissingMove =
                std::numeric_limits<std::size_t>::max();
            std::vector<std::size_t> entryMoveBySegment(
                entry.feedPoints.size() - 1, kMissingMove);
            for (std::size_t index = 1;
                 index < entry.feedPoints.size(); ++index) {
                const std::size_t moveIndex = result.moves.size();
                const gp_Pnt2d from2d = projectToPocketPlane(
                    entry.feedPoints[index - 1], region.frame);
                const gp_Pnt2d to2d = projectToPocketPlane(
                    entry.feedPoints[index], region.frame);
                const bool inclined =
                    from2d.Distance(to2d) > kMoveTolerance;
                const double entryFeed = inclined &&
                        params.clearing.rampFeed > 0.0
                    ? params.clearing.rampFeed
                    : params.tool.vertFeed;
                if (appendMove(
                        result, ToolpathMoveType::Plunge,
                        entry.feedPoints[index - 1],
                        entry.feedPoints[index], loop,
                        entryFeed,
                        entryIntent)) {
                    entryMoveBySegment[index - 1] = moveIndex;
                }
            }
            replaceHelicalEntryPrimitives(
                result, entryPrimitiveBegin,
                entryMoveBySegment, entry);
            hasCurrentClearance = false;
        }

        PreparedLoopToolpath& prepared = preparedLoops[loopIndex];
        result.moves.insert(
            result.moves.end(),
            std::make_move_iterator(prepared.moves.begin()),
            std::make_move_iterator(prepared.moves.end()));
        std::vector<ToolpathMove>().swap(prepared.moves);
        result.primitives.insert(
            result.primitives.end(),
            std::make_move_iterator(prepared.primitives.begin()),
            std::make_move_iterator(prepared.primitives.end()));
        std::vector<ToolpathPrimitive>().swap(prepared.primitives);

        currentCutPoint = cutEnd;
        currentCutPoint2d = end2d;
        currentDepthIndex = loop.depthIndex;
        currentLoop = &loop;
        toolAtCutDepth = true;
    }

    retractCurrentCut();
    result.moveLinkingMilliseconds =
        elapsedMilliseconds(linkingStart);
}

} // namespace oap::toolpath::detail

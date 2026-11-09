// Profile motion planning owns travel ordering, safe linking and move assembly.
#include "toolpath/profile/planning/detail/ProfileMotionPlanning.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <gp_Vec.hxx>

#include "toolpath/core/ToolpathPrimitives.h"
#include "toolpath/pocket/geometry/OcctAreaAdapter.h"
#include "toolpath/profile/planning/ProfileContour.h"

namespace oap::toolpath::profile_motion_detail {
namespace {

constexpr double kTolerance = 1.0e-9;
constexpr std::size_t kNearbyLinkCandidateLimit = 8u;
constexpr char kUnsafeLinkWarning[] =
    "a same-depth profile loop could not be linked outside the protected "
    "material envelope; a retract and re-entry were inserted";

gp_Pnt worldPoint(
    const gp_Pnt2d& point, const ProfileRegion& region, double depthOffset)
{
    gp_Pnt world = OcctAreaAdapter::pointFrom2d(point, region.frame);
    world.Translate(gp_Vec(region.openDir) * depthOffset);
    return world;
}

gp_Pnt shiftedAlongAxis(
    const gp_Pnt& point, const gp_Dir& axis, double distance)
{
    gp_Pnt shifted = point;
    shifted.Translate(gp_Vec(axis) * distance);
    return shifted;
}

void appendMove(
    ProfileContourResult& result,
    ToolpathMoveType type,
    const gp_Pnt& from,
    const gp_Pnt& to,
    const ToolpathParams& params,
    const ProfileLoop& loop,
    double feedOverride = -1.0)
{
    if (from.Distance(to) <= kTolerance)
        return;
    ToolpathMove move;
    move.type = type;
    move.from = from;
    move.to = to;
    move.loopSequence = loop.sequence;
    move.depthIndex = loop.depthIndex;
    move.ring = loop.radialPassIndex;
    move.feed = std::isfinite(feedOverride) && feedOverride >= 0.0
        ? feedOverride
        : type == ToolpathMoveType::Cut
            ? params.tool.horizFeed
            : type == ToolpathMoveType::Plunge
                ? params.tool.vertFeed : 0.0;
    result.moves.push_back(move);
    result.primitives.push_back(makeLinearPrimitive(move));
}

gp_Pnt2d tangentialLeadPoint(
    const gp_Pnt2d& anchor,
    const gp_Pnt2d& neighbor,
    double length,
    bool reverse)
{
    const double dx = neighbor.X() - anchor.X();
    const double dy = neighbor.Y() - anchor.Y();
    const double norm = std::hypot(dx, dy);
    if (norm <= kTolerance || length <= kTolerance)
        return anchor;
    const double sign = reverse ? -1.0 : 1.0;
    return gp_Pnt2d(
        anchor.X() + sign * length * dx / norm,
        anchor.Y() + sign * length * dy / norm);
}

double cross2d(
    const gp_Pnt2d& a,
    const gp_Pnt2d& b,
    const gp_Pnt2d& point)
{
    return (b.X() - a.X()) * (point.Y() - a.Y()) -
        (b.Y() - a.Y()) * (point.X() - a.X());
}

bool onSegment(
    const gp_Pnt2d& point,
    const gp_Pnt2d& a,
    const gp_Pnt2d& b,
    double tolerance)
{
    return std::abs(cross2d(a, b, point)) <= tolerance &&
        point.X() >= std::min(a.X(), b.X()) - tolerance &&
        point.X() <= std::max(a.X(), b.X()) + tolerance &&
        point.Y() >= std::min(a.Y(), b.Y()) - tolerance &&
        point.Y() <= std::max(a.Y(), b.Y()) + tolerance;
}

bool segmentsIntersect(
    const gp_Pnt2d& a,
    const gp_Pnt2d& b,
    const gp_Pnt2d& c,
    const gp_Pnt2d& d,
    double tolerance)
{
    const double abC = cross2d(a, b, c);
    const double abD = cross2d(a, b, d);
    const double cdA = cross2d(c, d, a);
    const double cdB = cross2d(c, d, b);
    if (((abC > tolerance && abD < -tolerance) ||
         (abC < -tolerance && abD > tolerance)) &&
        ((cdA > tolerance && cdB < -tolerance) ||
         (cdA < -tolerance && cdB > tolerance))) {
        return true;
    }
    return (std::abs(abC) <= tolerance && onSegment(c, a, b, tolerance)) ||
        (std::abs(abD) <= tolerance && onSegment(d, a, b, tolerance)) ||
        (std::abs(cdA) <= tolerance && onSegment(a, c, d, tolerance)) ||
        (std::abs(cdB) <= tolerance && onSegment(b, c, d, tolerance));
}

double pointSegmentDistance(
    const gp_Pnt2d& point,
    const gp_Pnt2d& from,
    const gp_Pnt2d& to)
{
    const double dx = to.X() - from.X();
    const double dy = to.Y() - from.Y();
    const double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= kTolerance * kTolerance)
        return point.Distance(from);
    const double parameter = std::clamp(
        ((point.X() - from.X()) * dx +
         (point.Y() - from.Y()) * dy) / lengthSquared,
        0.0, 1.0);
    return point.Distance(gp_Pnt2d(
        from.X() + parameter * dx,
        from.Y() + parameter * dy));
}

double segmentDistance(
    const gp_Pnt2d& a,
    const gp_Pnt2d& b,
    const gp_Pnt2d& c,
    const gp_Pnt2d& d,
    double tolerance)
{
    if (segmentsIntersect(a, b, c, d, tolerance))
        return 0.0;
    return std::min({
        pointSegmentDistance(a, c, d),
        pointSegmentDistance(b, c, d),
        pointSegmentDistance(c, a, b),
        pointSegmentDistance(d, a, b)});
}

bool strictlyOutsideBoundary(
    const gp_Pnt2d& point,
    const std::vector<gp_Pnt2d>& boundary,
    double tolerance)
{
    bool inside = false;
    for (std::size_t index = 0u; index < boundary.size(); ++index) {
        const gp_Pnt2d& from = boundary[index];
        const gp_Pnt2d& to = boundary[(index + 1u) % boundary.size()];
        if (pointSegmentDistance(point, from, to) <= tolerance)
            return false;
        const bool crossesRay =
            (from.Y() > point.Y()) != (to.Y() > point.Y());
        if (!crossesRay)
            continue;
        const double intersectionX = from.X() +
            (point.Y() - from.Y()) * (to.X() - from.X()) /
                (to.Y() - from.Y());
        if (point.X() < intersectionX)
            inside = !inside;
    }
    return !inside;
}

const ProfileBoundaries* linkSafetyBoundaries(const ProfileLoop& loop)
{
    if (loop.linkSafetyBoundaries && !loop.linkSafetyBoundaries->empty())
        return loop.linkSafetyBoundaries.get();
    if (loop.materialBoundaries && !loop.materialBoundaries->empty())
        return loop.materialBoundaries.get();
    return nullptr;
}

bool canLinkAtCutDepth(
    const gp_Pnt& from,
    const gp_Pnt& to,
    const ProfileLoop& destination,
    const ProfileRegion& region,
    const ToolpathParams& params)
{
    if (!std::isfinite(from.X()) || !std::isfinite(from.Y()) ||
        !std::isfinite(from.Z()) || !std::isfinite(to.X()) ||
        !std::isfinite(to.Y()) || !std::isfinite(to.Z())) {
        return false;
    }
    const ProfileBoundaries* boundaries = linkSafetyBoundaries(destination);

    const gp_Pnt2d planarFrom =
        OcctAreaAdapter::projectPoint(from, region.frame);
    const gp_Pnt2d planarTo =
        OcctAreaAdapter::projectPoint(to, region.frame);
    const double allowance = destination.passKind == ProfilePassKind::Clearing
        ? std::max(
              std::max(0.0, params.profile.sideAllowance),
              std::max(0.0, params.profile.roughingAllowance))
        : std::max(0.0, params.profile.sideAllowance);
    const double quantization = std::pow(
        10.0, -std::clamp(params.profile.precision, 0, 8));
    const double tolerance = std::max(0.02, 4.0 * quantization);
    const double unsafeDistance = params.tool.radius() + allowance - tolerance;
    const auto clearsBoundary = [&](const ProfileBoundary& boundary) {
        if (boundary.size() < 3 ||
            !strictlyOutsideBoundary(planarFrom, boundary, tolerance) ||
            !strictlyOutsideBoundary(planarTo, boundary, tolerance))
            return false;
        for (std::size_t index = 0; index < boundary.size(); ++index) {
            if (segmentDistance(planarFrom, planarTo, boundary[index],
                    boundary[(index + 1) % boundary.size()], tolerance) <
                unsafeDistance)
                return false;
        }
        return true;
    };
    return boundaries
        ? std::all_of(boundaries->begin(), boundaries->end(), clearsBoundary)
        : clearsBoundary(region.boundary.points);
}

gp_Pnt2d loopEntryPoint2d(
    const ProfileLoop& loop,
    std::size_t startIndex,
    const ToolpathParams& params)
{
    if (loop.points.empty())
        return {};
    if (!loop.closed || startIndex >= loop.points.size())
        startIndex = 0u;
    const std::size_t nextIndex = loop.closed
        ? (startIndex + 1u) % loop.points.size()
        : std::min<std::size_t>(1u, loop.points.size() - 1u);
    return tangentialLeadPoint(
        loop.points[startIndex], loop.points[nextIndex],
        loop.passKind == ProfilePassKind::Finish
            ? params.profile.leadInLength : 0.0,
        true);
}

gp_Pnt loopEntryPoint(
    const ProfileLoop& loop,
    std::size_t startIndex,
    const ProfileRegion& region,
    const ToolpathParams& params)
{
    return worldPoint(
        loopEntryPoint2d(loop, startIndex, params),
        region, loop.depthOffset);
}

gp_Pnt2d loopExitPoint2d(
    const ProfileLoop& loop,
    const ToolpathParams& params)
{
    if (loop.points.empty())
        return {};
    gp_Pnt2d exit = loop.closed
        ? loop.points.front() : loop.points.back();
    if (loop.passKind == ProfilePassKind::Finish &&
        params.profile.leadOutLength > kTolerance &&
        loop.points.size() >= 2u) {
        exit = tangentialLeadPoint(
            loop.points.front(), loop.points.back(),
            params.profile.leadOutLength, true);
    }
    return exit;
}

gp_Pnt loopExitPoint(
    const ProfileLoop& loop,
    const ProfileRegion& region,
    const ToolpathParams& params)
{
    return worldPoint(
        loopExitPoint2d(loop, params),
        region, loop.depthOffset);
}

struct ProfileTravelChoice {
    std::size_t relativeIndex = 0u;
    std::size_t startIndex = 0u;
    double distance = std::numeric_limits<double>::infinity();
};

std::vector<ProfileTravelChoice> nearbyTravelChoices(
    const ProfileLoop& loop,
    std::size_t relativeIndex,
    const gp_Pnt& current,
    const ProfileRegion& region,
    const ToolpathParams& params)
{
    std::vector<ProfileTravelChoice> choices;
    if (loop.world.empty())
        return choices;
    const std::size_t candidateCount = loop.closed
        ? loop.world.size() : 1u;
    choices.reserve(std::min(
        candidateCount, kNearbyLinkCandidateLimit));
    for (std::size_t startIndex = 0u;
         startIndex < candidateCount; ++startIndex) {
        ProfileTravelChoice choice;
        choice.relativeIndex = relativeIndex;
        choice.startIndex = startIndex;
        choice.distance = current.Distance(loopEntryPoint(
            loop, startIndex, region, params));
        const auto insertion = std::lower_bound(
            choices.begin(), choices.end(), choice,
            [](const ProfileTravelChoice& left,
               const ProfileTravelChoice& right) {
                return left.distance < right.distance;
            });
        if (insertion != choices.end() ||
            choices.size() < kNearbyLinkCandidateLimit) {
            choices.insert(insertion, choice);
            if (choices.size() > kNearbyLinkCandidateLimit)
                choices.pop_back();
        }
    }
    return choices;
}

void applyTravelChoice(
    ProfileLoop& loop,
    const ProfileTravelChoice& choice,
    ProfileContourResult& result)
{
    if (!loop.closed || choice.startIndex == 0u ||
        choice.startIndex >= loop.points.size() ||
        loop.world.size() != loop.points.size()) {
        return;
    }
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

} // namespace

void optimizeRestMachiningTravel(
    const ProfileRegion& region,
    const ToolpathParams& params,
    ProfileContourResult& result)
{
    if (!result.restMachining || result.loops.size() < 2u)
        return;

    std::vector<ProfileLoop> optimized;
    optimized.reserve(result.loops.size());
    bool hasCurrent = false;
    gp_Pnt current;
    int currentDepthIndex = -1;

    // 深度层、径向圈次和粗/精工艺顺序不能交换；只在同组离散余料
    // 分量之间优先选择安全直连，否则选择最近的跳刀入口。
    for (std::size_t groupBegin = 0u;
         groupBegin < result.loops.size();) {
        const int depthIndex = result.loops[groupBegin].depthIndex;
        const int radialPassIndex =
            result.loops[groupBegin].radialPassIndex;
        const ProfilePassKind passKind =
            result.loops[groupBegin].passKind;
        std::size_t groupEnd = groupBegin + 1u;
        while (groupEnd < result.loops.size() &&
               result.loops[groupEnd].depthIndex == depthIndex &&
               result.loops[groupEnd].radialPassIndex == radialPassIndex &&
               result.loops[groupEnd].passKind == passKind) {
            ++groupEnd;
        }

        std::vector<ProfileLoop> pending;
        pending.reserve(groupEnd - groupBegin);
        for (std::size_t index = groupBegin; index < groupEnd; ++index)
            pending.push_back(std::move(result.loops[index]));
        std::vector<unsigned char> active(pending.size(), 1u);
        std::size_t remaining = pending.size();
        while (remaining > 0u) {
            ProfileTravelChoice selected;
            if (!hasCurrent) {
                selected.relativeIndex = static_cast<std::size_t>(
                    std::find(active.begin(), active.end(), 1u) -
                    active.begin());
                selected.distance = 0.0;
            } else {
                std::vector<ProfileTravelChoice> nearest;
                nearest.reserve(remaining);
                for (std::size_t index = 0u;
                     index < pending.size(); ++index) {
                    if (active[index] == 0u)
                        continue;
                    const auto choices = nearbyTravelChoices(
                        pending[index], index, current,
                        region, params);
                    if (!choices.empty())
                        nearest.push_back(choices.front());
                }
                const auto choiceLess =
                    [](const ProfileTravelChoice& left,
                       const ProfileTravelChoice& right) {
                        if (left.distance != right.distance)
                            return left.distance < right.distance;
                        if (left.relativeIndex != right.relativeIndex) {
                            return left.relativeIndex <
                                right.relativeIndex;
                        }
                        return left.startIndex < right.startIndex;
                    };
                const std::size_t orderedChoiceCount = std::min(
                    kNearbyLinkCandidateLimit, nearest.size());
                std::partial_sort(
                    nearest.begin(),
                    nearest.begin() + static_cast<std::ptrdiff_t>(
                        orderedChoiceCount),
                    nearest.end(), choiceLess);
                nearest.resize(orderedChoiceCount);
                if (nearest.empty()) {
                    selected.relativeIndex = static_cast<std::size_t>(
                        std::find(active.begin(), active.end(), 1u) -
                        active.begin());
                    selected.distance = 0.0;
                } else {
                    selected = nearest.front();
                }

                if (!nearest.empty() &&
                    currentDepthIndex == depthIndex) {
                    bool foundSafeChoice = false;
                    ProfileTravelChoice safeChoice;
                    const std::size_t loopCandidateCount = std::min(
                        kNearbyLinkCandidateLimit, nearest.size());
                    for (std::size_t candidateIndex = 0u;
                         candidateIndex < loopCandidateCount;
                         ++candidateIndex) {
                        const ProfileLoop& candidateLoop = pending[
                            nearest[candidateIndex].relativeIndex];
                        const auto startChoices = nearbyTravelChoices(
                            candidateLoop,
                            nearest[candidateIndex].relativeIndex,
                            current, region, params);
                        for (const ProfileTravelChoice& candidate :
                             startChoices) {
                            const gp_Pnt entry = loopEntryPoint(
                                candidateLoop, candidate.startIndex,
                                region, params);
                            if (!canLinkAtCutDepth(
                                    current, entry, candidateLoop,
                                    region, params)) {
                                continue;
                            }
                            if (!foundSafeChoice ||
                                candidate.distance < safeChoice.distance) {
                                safeChoice = candidate;
                                foundSafeChoice = true;
                            }
                            break;
                        }
                    }
                    if (foundSafeChoice)
                        selected = safeChoice;
                }
            }

            ProfileLoop next = std::move(
                pending[selected.relativeIndex]);
            active[selected.relativeIndex] = 0u;
            --remaining;
            if (next.sequence != static_cast<int>(optimized.size()))
                ++result.travelReorderedLoopCount;
            applyTravelChoice(next, selected, result);
            current = loopExitPoint(next, region, params);
            currentDepthIndex = next.depthIndex;
            hasCurrent = true;
            optimized.push_back(std::move(next));
        }
        groupBegin = groupEnd;
    }

    result.loops = std::move(optimized);
    for (std::size_t index = 0u; index < result.loops.size(); ++index)
        result.loops[index].sequence = static_cast<int>(index);
}

double estimateCuttingDistance(
    ProfileContourResult& result,
    const ProfileRegion& region,
    const ToolpathParams& params)
{
    bool hasCurrentCut = false;
    gp_Pnt currentCutPoint;
    int currentDepthIndex = -1;
    const ProfileLoop* currentLoop = nullptr;
    double cuttingDistance = 0.0;

    for (const ProfileLoop& loop : result.loops) {
        const std::size_t minimumPoints = loop.closed ? 3u : 2u;
        if (loop.points.size() < minimumPoints ||
            loop.world.size() != loop.points.size()) {
            continue;
        }
        const gp_Pnt2d entry2d = loopEntryPoint2d(loop, 0u, params);
        const gp_Pnt entry = worldPoint(
            entry2d, region, loop.depthOffset);
        const bool sameDepth = hasCurrentCut &&
            currentDepthIndex == loop.depthIndex &&
            currentLoop != nullptr &&
            std::abs(currentLoop->depthOffset - loop.depthOffset) <=
                kTolerance;
        if (sameDepth && canLinkAtCutDepth(
                currentCutPoint, entry, loop, region, params)) {
            cuttingDistance += currentCutPoint.Distance(entry);
            ++result.directCutLinkCount;
        } else {
            ++result.retractEntryCount;
        }

        cuttingDistance += entry.Distance(loop.world.front());
        for (std::size_t index = 1u;
             index < loop.world.size(); ++index) {
            cuttingDistance += loop.world[index - 1u].Distance(
                loop.world[index]);
        }
        if (loop.closed) {
            cuttingDistance += loop.world.back().Distance(
                loop.world.front());
        }

        gp_Pnt exit = loop.closed
            ? loop.world.front() : loop.world.back();
        if (loop.passKind == ProfilePassKind::Finish &&
            params.profile.leadOutLength > kTolerance) {
            const gp_Pnt2d exit2d = tangentialLeadPoint(
                loop.points.front(), loop.points.back(),
                params.profile.leadOutLength, true);
            const gp_Pnt leadExit = worldPoint(
                exit2d, region, loop.depthOffset);
            cuttingDistance += loop.world.front().Distance(leadExit);
            exit = leadExit;
        }
        currentCutPoint = exit;
        currentDepthIndex = loop.depthIndex;
        currentLoop = &loop;
        hasCurrentCut = true;
    }
    return cuttingDistance;
}

void assembleMoves(
    ProfileContourResult& result,
    const ProfileRegion& region,
    const ToolpathParams& params)
{
    if (result.loops.empty())
        return;

    std::size_t estimatedMoveCount = result.loops.size() * 6u;
    for (const ProfileLoop& loop : result.loops) {
        estimatedMoveCount += loop.world.size();
        if (loop.closed)
            ++estimatedMoveCount;
    }
    result.moves.reserve(estimatedMoveCount);
    result.primitives.reserve(estimatedMoveCount);

    bool hasCurrentClearance = false;
    gp_Pnt currentClearance;
    bool toolAtCutDepth = false;
    gp_Pnt currentCutPoint;
    gp_Pnt2d currentCutPoint2d;
    int currentDepthIndex = -1;
    const ProfileLoop* currentLoop = nullptr;

    const auto retractCurrentCut = [&] {
        if (!toolAtCutDepth || currentLoop == nullptr)
            return;
        const gp_Pnt safePoint = worldPoint(
            currentCutPoint2d, region, result.safeHeightOffset);
        const gp_Pnt clearancePoint = worldPoint(
            currentCutPoint2d, region, result.clearanceHeightOffset);
        appendMove(
            result, ToolpathMoveType::Retract,
            currentCutPoint, safePoint, params, *currentLoop,
            params.tool.vertFeed);
        appendMove(
            result, ToolpathMoveType::Retract,
            safePoint, clearancePoint, params, *currentLoop,
            params.tool.vertRapid);
        currentClearance = clearancePoint;
        hasCurrentClearance = true;
        toolAtCutDepth = false;
        currentLoop = nullptr;
    };

    for (const ProfileLoop& loop : result.loops) {
        const std::size_t minimumPoints = loop.closed ? 3u : 2u;
        if (loop.points.size() < minimumPoints ||
            loop.world.size() != loop.points.size())
            continue;
        const bool useLead = loop.passKind == ProfilePassKind::Finish;
        const gp_Pnt2d entry2d = loopEntryPoint2d(loop, 0u, params);
        const gp_Pnt entry = worldPoint(entry2d, region, loop.depthOffset);
        const gp_Pnt clearance = shiftedAlongAxis(
            entry, region.openDir,
            result.clearanceHeightOffset - loop.depthOffset);
        const gp_Pnt safe = shiftedAlongAxis(
            entry, region.openDir,
            result.safeHeightOffset - loop.depthOffset);
        const bool sameDepth = toolAtCutDepth &&
            currentDepthIndex == loop.depthIndex &&
            currentLoop != nullptr &&
            std::abs(currentLoop->depthOffset - loop.depthOffset) <=
                kTolerance;
        const bool linkAtCutDepth = sameDepth &&
            canLinkAtCutDepth(
                currentCutPoint, entry, loop, region, params);

        if (linkAtCutDepth) {
            appendMove(
                result, ToolpathMoveType::Cut,
                currentCutPoint, entry, params, loop,
                params.tool.horizFeed);
            ++result.directCutLinkCount;
        } else {
            if (sameDepth &&
                result.warning.find(kUnsafeLinkWarning) ==
                    std::string::npos) {
                if (!result.warning.empty())
                    result.warning += "; ";
                result.warning += kUnsafeLinkWarning;
            }
            retractCurrentCut();
            if (hasCurrentClearance) {
                appendMove(
                    result, ToolpathMoveType::Rapid,
                    currentClearance, clearance, params, loop,
                    params.tool.horizRapid);
            }
            appendMove(
                result, ToolpathMoveType::Rapid,
                clearance, safe, params, loop,
                params.tool.vertRapid);
            appendMove(
                result, ToolpathMoveType::Plunge,
                safe, entry, params, loop,
                params.tool.vertFeed);
            hasCurrentClearance = false;
            ++result.retractEntryCount;
        }

        appendMove(result, ToolpathMoveType::Cut,
                   entry, loop.world.front(), params, loop);
        for (std::size_t index = 1; index < loop.world.size(); ++index) {
            appendMove(result, ToolpathMoveType::Cut,
                       loop.world[index - 1], loop.world[index], params, loop);
        }
        if (loop.closed) {
            appendMove(result, ToolpathMoveType::Cut,
                       loop.world.back(), loop.world.front(), params, loop);
        }

        gp_Pnt exit = loop.closed ? loop.world.front() : loop.world.back();
        gp_Pnt2d exit2d = loop.closed
            ? loop.points.front() : loop.points.back();
        if (useLead && params.profile.leadOutLength > kTolerance) {
            exit2d = tangentialLeadPoint(
                loop.points.front(), loop.points.back(),
                params.profile.leadOutLength, true);
            exit = worldPoint(exit2d, region, loop.depthOffset);
            appendMove(result, ToolpathMoveType::Cut,
                       loop.world.front(), exit, params, loop);
        }
        currentCutPoint = exit;
        currentCutPoint2d = exit2d;
        currentDepthIndex = loop.depthIndex;
        currentLoop = &loop;
        toolAtCutDepth = true;
    }
    retractCurrentCut();
}

} // namespace oap::toolpath::profile_motion_detail

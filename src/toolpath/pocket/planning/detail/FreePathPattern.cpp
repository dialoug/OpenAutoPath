// Adaptive FreePath planner. The adjacent ADAPTIVE_FREEPATH_DESIGN.md is the
// maintenance contract for this implementation and its pyocc-exp provenance.
#include "PocketPatterns.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <optional>
#include <unordered_set>
#include <utility>

#include "AdaptiveFreePathRaster.h"
#include "AdaptiveFreePathLinking.h"

namespace oap::toolpath::detail {

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kTolerance = 1.0e-9;
constexpr std::size_t kMaximumLocalSegments = 20000u;

struct PlannerOptions {
    double toolRadius = 0.0;
    double stepOver = 0.0;
    double targetEngagement = 0.10;
    double targetAngle = 70.0 * kPi / 180.0;
    double maximumAngle = 150.0 * kPi / 180.0;
    int angleSamples = 72;
    double minimumTurnRadius = 0.0;
    double remainingTolerance = 0.015;
    double cellSize = 0.0;
    int maximumPasses = 60;
    int lookaheadDepth = 3;
    int beamWidth = 4;
    bool clockwise = false;
};

struct MotionState {
    gp_Pnt2d point;
    double heading = 0.0;
    double curvature = 0.0;
};

struct MotionCandidate {
    std::vector<gp_Pnt2d> points;
    double curvature = 0.0;
    double endHeading = 0.0;
    double engagementAngle = 0.0;
    double meanEngagementAngle = 0.0;
    std::vector<std::size_t> removedCells;
    double score = std::numeric_limits<double>::infinity();
};

struct PlannerState {
    AdaptiveFreePathRaster& raster;
    const PlannerOptions& options;
    const ToolpathGenerationControl* control = nullptr;
    std::vector<ClearingLoop> runs;
    int adaptivePasses = 0;
    int overloadSegments = 0;
    double maximumEngagementAngle = 0.0;
    int wavefrontLayers = 0;
    int radialComponents = 0;
    int radialPasses = 0;
    int centralSpiralRuns = 0;
};

double wrappedAngle(double angle)
{
    while (angle <= -kPi)
        angle += kTwoPi;
    while (angle > kPi)
        angle -= kTwoPi;
    return angle;
}

double polylineLength(const std::vector<gp_Pnt2d>& points)
{
    double result = 0.0;
    for (std::size_t index = 1u; index < points.size(); ++index)
        result += points[index - 1u].Distance(points[index]);
    return result;
}

void appendUnique(std::vector<gp_Pnt2d>& points,
                  const gp_Pnt2d& point)
{
    if (points.empty() || points.back().Distance(point) > kTolerance)
        points.push_back(point);
}

bool cancelled(const ToolpathGenerationControl* control)
{
    return control && control->cancelled();
}

PlannerOptions normalizedOptions(
    double toolDiameter,
    double step,
    const AdaptiveClearingParams& source,
    bool reverse)
{
    PlannerOptions result;
    result.toolRadius = 0.5 * toolDiameter;
    result.stepOver = step;
    result.targetEngagement = std::clamp(
        source.targetEngagement, 1.0e-3, 1.0);
    const double targetDegrees = std::clamp(
        source.targetEngagementAngleDeg, 1.0, 360.0);
    const double maximumDegrees = std::clamp(
        source.maximumEngagementAngleDeg, targetDegrees, 360.0);
    result.targetAngle = targetDegrees * kPi / 180.0;
    result.maximumAngle = maximumDegrees * kPi / 180.0;
    result.angleSamples = std::clamp(
        source.engagementAngleSamples, 8, 720);
    result.minimumTurnRadius = std::max(
        0.05 * toolDiameter,
        source.minimumTurnRadiusFactor * toolDiameter);
    result.remainingTolerance = std::clamp(
        source.remainingTolerance, 0.0, 0.999999);
    result.cellSize = source.materialCellSize > 0.0
        ? source.materialCellSize
        : std::min(0.5 * step, 0.1 * toolDiameter);
    result.maximumPasses = std::clamp(
        source.maximumAdaptivePasses, 1, 500);
    result.lookaheadDepth = std::clamp(source.lookaheadDepth, 1, 4);
    result.beamWidth = std::clamp(source.beamWidth, 1, 16);
    result.clockwise = reverse;
    return result;
}

bool pointInArea(const PathsD& area, const gp_Pnt2d& point)
{
    const PointD query(point.X(), point.Y());
    int winding = 0;
    for (const PathD& path : area) {
        const auto relation = Clipper2Lib::PointInPolygon(query, path);
        if (relation == Clipper2Lib::PointInPolygonResult::IsOn)
            return true;
        if (relation == Clipper2Lib::PointInPolygonResult::IsInside)
            winding += signedArea(path) >= 0.0 ? 1 : -1;
    }
    return winding != 0;
}

gp_Pnt2d areaCenter(const PathsD& area)
{
    const Clipper2Lib::RectD bounds = Clipper2Lib::GetBounds(area);
    gp_Pnt2d center(
        0.5 * (bounds.left + bounds.right),
        0.5 * (bounds.top + bounds.bottom));
    if (pointInArea(area, center))
        return center;

    const PathD* largest = nullptr;
    double largestArea = 0.0;
    for (const PathD& path : area) {
        const double areaValue = signedArea(path);
        if (areaValue > largestArea) {
            largestArea = areaValue;
            largest = &path;
        }
    }
    if (!largest || largest->empty())
        return center;
    double x = 0.0;
    double y = 0.0;
    for (const PointD& point : *largest) {
        x += point.x;
        y += point.y;
    }
    const gp_Pnt2d average(
        x / static_cast<double>(largest->size()),
        y / static_cast<double>(largest->size()));
    return pointInArea(area, average)
        ? average
        : gp_Pnt2d(largest->front().x, largest->front().y);
}

void rotateClosedRun(ClearingLoop& loop, const gp_Pnt2d& current)
{
    if (!loop.closed || loop.points.size() < 2u)
        return;
    const auto closest = std::min_element(
        loop.points.begin(), loop.points.end(),
        [&](const gp_Pnt2d& left, const gp_Pnt2d& right) {
            return left.SquareDistance(current) <
                   right.SquareDistance(current);
        });
    std::rotate(loop.points.begin(), closest, loop.points.end());
}

ClearingLoop makeRun(std::vector<gp_Pnt2d> points,
                     bool closed,
                     bool hole = false)
{
    if (closed && points.size() > 1u &&
        points.front().Distance(points.back()) <= kTolerance) {
        points.pop_back();
    }
    ClearingLoop result;
    result.points = std::move(points);
    result.closed = closed;
    result.isHole = hole;
    if (result.closed && result.points.size() >= 3u) {
        PathD path;
        path.reserve(result.points.size());
        for (const gp_Pnt2d& point : result.points)
            path.emplace_back(point.X(), point.Y());
        result.area = signedArea(path);
    }
    return result;
}

std::vector<gp_Pnt2d> pathPoints(const PathD& path)
{
    std::vector<gp_Pnt2d> result;
    result.reserve(path.size());
    for (const PointD& point : path)
        result.emplace_back(point.x, point.y);
    return result;
}

bool isAxisAlignedRectangle(const PathsD& area)
{
    if (area.size() != 1u || area.front().size() != 4u ||
        signedArea(area.front()) <= 0.0) {
        return false;
    }
    const PathD& path = area.front();
    for (std::size_t index = 0u; index < path.size(); ++index) {
        const PointD& a = path[index];
        const PointD& b = path[(index + 1u) % path.size()];
        if (std::abs(a.x - b.x) > kTolerance &&
            std::abs(a.y - b.y) > kTolerance) {
            return false;
        }
    }
    return true;
}

double roundedRectanglePerimeter(double halfWidth,
                                 double halfHeight,
                                 double cornerRadius)
{
    const double radius = std::clamp(
        cornerRadius, 0.0, std::min(halfWidth, halfHeight));
    return 4.0 * (halfWidth + halfHeight - 2.0 * radius) +
           kTwoPi * radius;
}

gp_Pnt2d roundedRectanglePoint(double halfWidth,
                              double halfHeight,
                              double cornerRadius,
                              double phase)
{
    const double radius = std::clamp(
        cornerRadius, 0.0, std::min(halfWidth, halfHeight));
    const double sideHalf = halfHeight - radius;
    const double horizontal = 2.0 * (halfWidth - radius);
    const double vertical = 2.0 * sideHalf;
    const double arc = 0.5 * kPi * radius;
    const double perimeter = roundedRectanglePerimeter(
        halfWidth, halfHeight, radius);
    double distance = std::clamp(phase, 0.0, 1.0) * perimeter;
    if (distance <= sideHalf)
        return {halfWidth, distance};
    distance -= sideHalf;
    if (arc > kTolerance && distance <= arc) {
        const double angle = distance / radius;
        return {halfWidth - radius + radius * std::cos(angle),
                halfHeight - radius + radius * std::sin(angle)};
    }
    distance -= arc;
    if (distance <= horizontal)
        return {halfWidth - radius - distance, halfHeight};
    distance -= horizontal;
    if (arc > kTolerance && distance <= arc) {
        const double angle = 0.5 * kPi + distance / radius;
        return {-halfWidth + radius + radius * std::cos(angle),
                halfHeight - radius + radius * std::sin(angle)};
    }
    distance -= arc;
    if (distance <= vertical)
        return {-halfWidth, halfHeight - radius - distance};
    distance -= vertical;
    if (arc > kTolerance && distance <= arc) {
        const double angle = kPi + distance / radius;
        return {-halfWidth + radius + radius * std::cos(angle),
                -halfHeight + radius + radius * std::sin(angle)};
    }
    distance -= arc;
    if (distance <= horizontal)
        return {-halfWidth + radius + distance, -halfHeight};
    distance -= horizontal;
    if (arc > kTolerance && distance <= arc) {
        const double angle = 1.5 * kPi + distance / radius;
        return {halfWidth - radius + radius * std::cos(angle),
                -halfHeight + radius + radius * std::sin(angle)};
    }
    distance -= arc;
    return {halfWidth, -halfHeight + radius + std::min(distance, sideHalf)};
}

std::vector<gp_Pnt2d> rectangleWavefrontSpiral(
    const PathsD& area,
    const PlannerOptions& options,
    double sampleSpacing)
{
    const Clipper2Lib::RectD bounds = Clipper2Lib::GetBounds(area);
    const gp_Pnt2d center(
        0.5 * (bounds.left + bounds.right),
        0.5 * (bounds.top + bounds.bottom));
    const double halfWidth = 0.5 * (bounds.right - bounds.left);
    const double halfHeight = 0.5 * (bounds.bottom - bounds.top);
    const bool majorIsX = halfWidth >= halfHeight;
    const double major = std::max(halfWidth, halfHeight);
    const double minor = std::min(halfWidth, halfHeight);
    const double coreHalfLength = major - minor;
    const double startGrowth = std::min(
        minor,
        std::max(options.minimumTurnRadius,
                 0.75 * options.toolRadius));
    const double targetCornerRadius = std::min(
        minor,
        std::max(options.minimumTurnRadius,
                 2.0 * options.toolRadius));
    const auto world = [&](const gp_Pnt2d& local) {
        const double x = majorIsX ? local.X() : local.Y();
        const double y = majorIsX ? local.Y() : local.X();
        return gp_Pnt2d(center.X() + x, center.Y() + y);
    };

    const double innerHalfMajor = coreHalfLength + startGrowth;
    std::vector<gp_Pnt2d> points{
        world(gp_Pnt2d(-innerHalfMajor, 0.0)),
        world(gp_Pnt2d(innerHalfMajor, 0.0))};
    const double growthRange = minor - startGrowth;
    const int turns = growthRange > kTolerance
        ? static_cast<int>(std::ceil(growthRange / options.stepOver))
        : 0;
    if (turns <= 0)
        return points;
    const double pitch = growthRange / static_cast<double>(turns);
    double turn = 0.0;
    while (turn < static_cast<double>(turns) - 1.0e-10) {
        const double growth = startGrowth + pitch * turn;
        const double halfMajor = coreHalfLength + growth;
        const double cornerRadius = std::min(growth, targetCornerRadius);
        appendUnique(points, world(roundedRectanglePoint(
            halfMajor, growth, cornerRadius,
            turn - std::floor(turn))));
        const double perimeter = roundedRectanglePerimeter(
            halfMajor, growth, cornerRadius);
        turn = std::min(
            static_cast<double>(turns),
            turn + std::min(
                0.05, sampleSpacing / std::max(perimeter, sampleSpacing)));
    }
    appendUnique(points, world(gp_Pnt2d(major, 0.0)));
    return points;
}

double rayReach(const AdaptiveFreePathRaster& raster,
                const gp_Pnt2d& center,
                double angle,
                double maximum)
{
    double low = 0.0;
    double high = maximum;
    for (int iteration = 0; iteration < 42; ++iteration) {
        const double middle = 0.5 * (low + high);
        const gp_Pnt2d candidate(
            center.X() + middle * std::cos(angle),
            center.Y() + middle * std::sin(angle));
        if (raster.safe(candidate))
            low = middle;
        else
            high = middle;
    }
    return low;
}

bool supportsRadialStrategy(const PathsD& area,
                            const AdaptiveFreePathRaster& raster,
                            const gp_Pnt2d& center,
                            const PlannerOptions& options)
{
    if (!raster.safe(center))
        return false;
    if (std::any_of(area.begin(), area.end(), [](const PathD& path) {
            return signedArea(path) < 0.0;
        })) {
        return false;
    }
    const double sampleSpacing = std::max(
        options.stepOver, 0.7 * options.toolRadius);
    double maximumPointDistance = 0.0;
    for (const PathD& path : area) {
        for (const PointD& point : path) {
            maximumPointDistance = std::max(
                maximumPointDistance,
                center.Distance(gp_Pnt2d(point.x, point.y)));
        }
    }
    const double rayLimit = maximumPointDistance +
        2.0 * std::max(sampleSpacing, 1.0);
    const int samples = std::clamp(
        static_cast<int>(std::ceil(
            kTwoPi * rayLimit / std::max(sampleSpacing, 0.50))),
        96, 360);
    std::vector<double> reaches;
    reaches.reserve(static_cast<std::size_t>(samples));
    for (int sample = 0; sample < samples; ++sample) {
        const double angle = kTwoPi * static_cast<double>(sample) /
                             static_cast<double>(samples);
        const double reach = rayReach(raster, center, angle, rayLimit);
        if (reach <= raster.cellSize())
            return false;
        const gp_Pnt2d end(
            center.X() + 0.98 * reach * std::cos(angle),
            center.Y() + 0.98 * reach * std::sin(angle));
        if (!raster.segmentSafe(center, end, 0.5 * raster.cellSize()))
            return false;
        reaches.push_back(reach);
    }
    // pyocc-exp rejects shapes such as a cross whose polar boundary jumps
    // between neighbouring rays.  A continuous star-shaped boundary is a
    // precondition for fixed-direction radial layers.
    const double angularSpacing = rayLimit * kTwoPi /
                                  static_cast<double>(samples);
    const double maximumJump = 2.0 * std::max(
        sampleSpacing, angularSpacing);
    for (std::size_t index = 0u; index < reaches.size(); ++index) {
        if (std::abs(reaches[index] -
                     reaches[(index + 1u) % reaches.size()]) > maximumJump) {
            return false;
        }
    }
    return true;
}

std::vector<gp_Pnt2d> constrainedCoreSpiral(
    const PathsD& area,
    const AdaptiveFreePathRaster& raster,
    const gp_Pnt2d& center,
    const PlannerOptions& options,
    double sampleSpacing)
{
    const Clipper2Lib::RectD bounds = Clipper2Lib::GetBounds(area);
    const double rayLimit = std::hypot(
        bounds.right - bounds.left, bounds.bottom - bounds.top);
    constexpr int kRaySamples = 360;
    std::vector<double> reaches(kRaySamples);
    for (int sample = 0; sample < kRaySamples; ++sample) {
        const double angle = kTwoPi * static_cast<double>(sample) /
                             static_cast<double>(kRaySamples);
        reaches[static_cast<std::size_t>(sample)] = std::max(
            0.0, rayReach(raster, center, angle, rayLimit) -
                     0.25 * raster.cellSize());
    }
    const auto reachAt = [&](double angle) {
        double normalized = std::fmod(angle, kTwoPi);
        if (normalized < 0.0)
            normalized += kTwoPi;
        const double position = normalized * kRaySamples / kTwoPi;
        const int first = static_cast<int>(std::floor(position)) % kRaySamples;
        const int second = (first + 1) % kRaySamples;
        const double blend = position - std::floor(position);
        return (1.0 - blend) * reaches[static_cast<std::size_t>(first)] +
               blend * reaches[static_cast<std::size_t>(second)];
    };
    const double minimumReach = *std::min_element(
        reaches.begin(), reaches.end());
    const double horizontalRadius = std::min(
        reachAt(0.0), reachAt(kPi));
    const double verticalRadius = std::min(
        reachAt(0.5 * kPi), reachAt(1.5 * kPi));
    const double contactRadius = std::max({
        minimumReach, horizontalRadius, verticalRadius});
    const int turns = std::max(
        1, static_cast<int>(std::ceil(contactRadius / options.stepOver)));
    const double terminalRadius =
        static_cast<double>(turns) * options.stepOver;
    const double thetaMaximum = kTwoPi * terminalRadius / options.stepOver;
    const double direction = options.clockwise ? -1.0 : 1.0;
    std::vector<gp_Pnt2d> result;
    const double safeSpacing = std::max(
        0.25 * raster.cellSize(), 1.0e-4);
    const auto appendSafe = [&](const gp_Pnt2d& candidate) {
        if (result.empty()) {
            result.push_back(candidate);
            return;
        }
        const gp_Pnt2d start = result.back();
        if (raster.segmentSafe(start, candidate, safeSpacing)) {
            appendUnique(result, candidate);
            return;
        }
        const double startRadius = start.Distance(center);
        const double endRadius = candidate.Distance(center);
        const double startAngle = std::atan2(
            start.Y() - center.Y(), start.X() - center.X());
        const double endAngle = std::atan2(
            candidate.Y() - center.Y(), candidate.X() - center.X());
        double low = 0.0;
        double high = 1.0;
        for (int iteration = 0; iteration < 36; ++iteration) {
            const double scale = 0.5 * (low + high);
            const gp_Pnt2d insetStart(
                center.X() + scale * startRadius * std::cos(startAngle),
                center.Y() + scale * startRadius * std::sin(startAngle));
            const gp_Pnt2d insetEnd(
                center.X() + scale * endRadius * std::cos(endAngle),
                center.Y() + scale * endRadius * std::sin(endAngle));
            if (raster.segmentSafe(insetStart, insetEnd, safeSpacing))
                low = scale;
            else
                high = scale;
        }
        const double scale = std::max(0.0, low -
            0.02 * safeSpacing /
                std::max({startRadius, endRadius, safeSpacing}));
        const gp_Pnt2d insetStart(
            center.X() + scale * startRadius * std::cos(startAngle),
            center.Y() + scale * startRadius * std::sin(startAngle));
        const gp_Pnt2d insetEnd(
            center.X() + scale * endRadius * std::cos(endAngle),
            center.Y() + scale * endRadius * std::sin(endAngle));
        appendUnique(result, insetStart);
        appendUnique(result, insetEnd);
        appendUnique(result, candidate);
    };
    double theta = 0.0;
    while (theta < thetaMaximum - kTolerance && result.size() < 200000u) {
        const double guideRadius = options.stepOver * theta / kTwoPi;
        const double angle = direction * theta;
        const double boundary = reachAt(angle);
        const double transition = std::max(0.5 * options.stepOver,
                                           raster.cellSize());
        const double difference = boundary - guideRadius;
        double radius = std::min(guideRadius, boundary);
        if (std::abs(difference) < transition) {
            const double t = std::clamp(
                0.5 + 0.5 * difference / transition, 0.0, 1.0);
            const double smooth = t * t * (3.0 - 2.0 * t);
            radius = boundary - transition * t * (1.0 - smooth);
            radius = std::clamp(radius, 0.0, boundary);
        }
        const gp_Pnt2d candidate(
            center.X() + radius * std::cos(angle),
            center.Y() + radius * std::sin(angle));
        if (raster.safe(candidate))
            appendSafe(candidate);
        const double localRadius = std::max(radius, options.stepOver);
        theta += std::clamp(
            sampleSpacing / localRadius, 0.035, 0.35);
    }
    const auto constrainedPoint = [&](double thetaValue) {
        const double angle = direction * thetaValue;
        const double boundary = reachAt(angle);
        const double transition = std::max(
            0.5 * options.stepOver, raster.cellSize());
        const double difference = boundary - terminalRadius;
        double radius = std::min(terminalRadius, boundary);
        if (std::abs(difference) < transition) {
            const double t = std::clamp(
                0.5 + 0.5 * difference / transition, 0.0, 1.0);
            const double smooth = t * t * (3.0 - 2.0 * t);
            radius = std::clamp(
                boundary - transition * t * (1.0 - smooth),
                0.0, boundary);
        }
        return gp_Pnt2d(
            center.X() + radius * std::cos(angle),
            center.Y() + radius * std::sin(angle));
    };
    appendSafe(constrainedPoint(thetaMaximum));
    const int loopSamples = std::max(
        24, static_cast<int>(std::ceil(
            kTwoPi * terminalRadius / std::max(sampleSpacing, 1.0e-6))));
    for (int index = 1; index <= loopSamples; ++index) {
        appendSafe(constrainedPoint(
            thetaMaximum + kTwoPi * static_cast<double>(index) /
                static_cast<double>(loopSamples)));
    }
    return result;
}

std::vector<gp_Pnt2d> sampleCurvatureMotion(
    const MotionState& state,
    double curvature,
    double length,
    double spacing)
{
    const std::size_t segments = std::clamp<std::size_t>(
        static_cast<std::size_t>(std::ceil(length / std::max(spacing, 1.0e-6))),
        1u, 7u);
    std::vector<gp_Pnt2d> result;
    result.reserve(segments + 1u);
    result.push_back(state.point);
    for (std::size_t index = 1u; index <= segments; ++index) {
        const double distance = length * static_cast<double>(index) /
                                static_cast<double>(segments);
        if (std::abs(curvature) <= 1.0e-10) {
            result.emplace_back(
                state.point.X() + distance * std::cos(state.heading),
                state.point.Y() + distance * std::sin(state.heading));
            continue;
        }
        const double heading = state.heading + curvature * distance;
        result.emplace_back(
            state.point.X() +
                (std::sin(heading) - std::sin(state.heading)) / curvature,
            state.point.Y() -
                (std::cos(heading) - std::cos(state.heading)) / curvature);
    }
    return result;
}

double spiralHeading(const gp_Pnt2d& center,
                     const gp_Pnt2d& point,
                     double radialGrowth,
                     double direction,
                     double fallback)
{
    const double radius = point.Distance(center);
    if (radius <= kTolerance)
        return fallback;
    const double polar = std::atan2(
        point.Y() - center.Y(), point.X() - center.X());
    return polar + direction * std::atan2(radius, radialGrowth);
}

std::vector<MotionCandidate> expandCandidates(
    const MotionState& state,
    const gp_Pnt2d& center,
    double radialGrowth,
    double direction,
    double motionLength,
    AdaptiveFreePathRaster& raster,
    const PlannerOptions& options,
    double safeSpacing,
    const std::unordered_set<std::size_t>* ignored)
{
    const double desiredHeading = spiralHeading(
        center, state.point, radialGrowth, direction, state.heading);
    const double maximumCurvature = 1.0 / options.minimumTurnRadius;
    const double steeringCurvature = std::clamp(
        wrappedAngle(desiredHeading - state.heading) /
            std::max(motionLength, kTolerance),
        -maximumCurvature, maximumCurvature);
    const double radius = state.point.Distance(center);
    const double denominator = std::pow(
        radius * radius + radialGrowth * radialGrowth, 1.5);
    const double idealCurvature = direction * std::min(
        maximumCurvature,
        denominator > kTolerance
            ? (radius * radius + 2.0 * radialGrowth * radialGrowth) /
                  denominator
            : maximumCurvature);
    const double delta = 0.35 * maximumCurvature;
    const double raw[] = {
        state.curvature, steeringCurvature, idealCurvature,
        steeringCurvature - delta, steeringCurvature + delta, 0.0};
    std::vector<double> curvatures;
    for (const double value : raw) {
        const double bounded = std::clamp(
            value, -maximumCurvature, maximumCurvature);
        if (std::none_of(curvatures.begin(), curvatures.end(),
                         [&](double existing) {
                             return std::abs(existing - bounded) <= 1.0e-8;
                         })) {
            curvatures.push_back(bounded);
        }
    }

    std::vector<MotionCandidate> result;
    for (const double curvature : curvatures) {
        MotionCandidate candidate;
        candidate.points = sampleCurvatureMotion(
            state, curvature, motionLength, safeSpacing);
        bool safe = true;
        for (std::size_t index = 1u; index < candidate.points.size(); ++index) {
            if (!raster.segmentSafe(
                    candidate.points[index - 1u], candidate.points[index],
                    safeSpacing)) {
                safe = false;
                break;
            }
        }
        if (!safe)
            continue;
        candidate.curvature = curvature;
        candidate.endHeading = state.heading + curvature * motionLength;
        std::unordered_set<std::size_t> predicted = ignored
            ? *ignored
            : std::unordered_set<std::size_t>{};
        std::vector<double> angles;
        for (std::size_t index = 1u; index < candidate.points.size(); ++index) {
            const gp_Pnt2d midpoint(
                0.5 * (candidate.points[index - 1u].X() +
                       candidate.points[index].X()),
                0.5 * (candidate.points[index - 1u].Y() +
                       candidate.points[index].Y()));
            angles.push_back(raster.angularEngagement(
                midpoint, options.toolRadius, options.angleSamples,
                &predicted).maximumArcAngle);
            const std::vector<std::size_t> cells =
                raster.segmentRemainingIndices(
                    candidate.points[index - 1u], candidate.points[index],
                    options.toolRadius, &predicted);
            candidate.removedCells.insert(
                candidate.removedCells.end(), cells.begin(), cells.end());
            predicted.insert(cells.begin(), cells.end());
        }
        std::sort(candidate.removedCells.begin(), candidate.removedCells.end());
        candidate.removedCells.erase(
            std::unique(candidate.removedCells.begin(),
                        candidate.removedCells.end()),
            candidate.removedCells.end());
        candidate.engagementAngle = angles.empty()
            ? 0.0
            : *std::max_element(angles.begin(), angles.end());
        candidate.meanEngagementAngle = angles.empty()
            ? 0.0
            : std::accumulate(angles.begin(), angles.end(), 0.0) /
                  static_cast<double>(angles.size());
        const double engagementError = std::abs(
            candidate.engagementAngle - options.targetAngle) /
            std::max(options.targetAngle, kTolerance);
        const double overload = std::max(
            0.0, candidate.engagementAngle - options.maximumAngle) /
            std::max(options.targetAngle, kTolerance);
        const double curvaturePenalty =
            0.05 * std::abs(curvature) * options.minimumTurnRadius;
        const double curvatureChangePenalty =
            0.12 * std::abs(curvature - state.curvature) *
            options.minimumTurnRadius;
        const gp_Pnt2d& end = candidate.points.back();
        const double desiredEndHeading = spiralHeading(
            center, end, radialGrowth, direction, candidate.endHeading);
        const double headingError = std::abs(wrappedAngle(
            candidate.endHeading - desiredEndHeading)) / kPi;
        const double startRadius = state.point.Distance(center);
        const double endRadius = end.Distance(center);
        if (startRadius > std::max(motionLength, raster.cellSize()) &&
            endRadius < startRadius - 0.12 * motionLength) {
            continue;
        }
        const double expectedAdvance = motionLength * radialGrowth /
            std::max(std::hypot(startRadius, radialGrowth), kTolerance);
        const double radialError = std::abs(
            (endRadius - startRadius) - expectedAdvance) /
            std::max({expectedAdvance, 0.2 * motionLength,
                      raster.cellSize()});
        candidate.score = engagementError + 4.0 * overload * overload +
            curvaturePenalty + curvatureChangePenalty +
            0.30 * headingError + 0.18 * radialError;
        result.push_back(std::move(candidate));
    }
    return result;
}

double candidateCost(const MotionCandidate& candidate,
                     std::size_t mostMaterial)
{
    return candidate.score +
        (candidate.removedCells.empty() ? 0.55 : 0.0) -
        0.30 * static_cast<double>(candidate.removedCells.size()) /
            static_cast<double>(std::max<std::size_t>(1u, mostMaterial));
}

std::optional<MotionCandidate> chooseCandidate(
    const MotionState& state,
    const gp_Pnt2d& center,
    double radialGrowth,
    double direction,
    double motionLength,
    AdaptiveFreePathRaster& raster,
    const PlannerOptions& options,
    double safeSpacing,
    bool requireMaterial)
{
    std::vector<MotionCandidate> eligible = expandCandidates(
        state, center, radialGrowth, direction, motionLength,
        raster, options, safeSpacing, nullptr);
    eligible.erase(
        std::remove_if(
            eligible.begin(), eligible.end(),
            [&](const MotionCandidate& candidate) {
                return requireMaterial && candidate.removedCells.empty();
            }),
        eligible.end());
    if (eligible.empty())
        return std::nullopt;
    const bool hasCompliant = std::any_of(
        eligible.begin(), eligible.end(), [&](const MotionCandidate& candidate) {
            return candidate.engagementAngle <= options.maximumAngle + kTolerance;
        });
    if (hasCompliant) {
        eligible.erase(
            std::remove_if(
                eligible.begin(), eligible.end(),
                [&](const MotionCandidate& candidate) {
                    return candidate.engagementAngle >
                           options.maximumAngle + kTolerance;
                }),
            eligible.end());
    }
    const auto mostRemoved = std::max_element(
        eligible.begin(), eligible.end(),
        [](const MotionCandidate& left, const MotionCandidate& right) {
            return left.removedCells.size() < right.removedCells.size();
        });
    const std::size_t mostMaterial = std::max<std::size_t>(
        1u, mostRemoved->removedCells.size());
    std::sort(
        eligible.begin(), eligible.end(),
        [&](const MotionCandidate& left, const MotionCandidate& right) {
            return candidateCost(left, mostMaterial) <
                   candidateCost(right, mostMaterial);
        });
    if (options.lookaheadDepth <= 1 || eligible.size() == 1u)
        return eligible.front();

    struct BeamNode {
        MotionState state;
        std::unordered_set<std::size_t> ignored;
        std::size_t root = 0u;
        double score = 0.0;
    };
    std::vector<BeamNode> beam;
    const std::size_t rootCount = std::min<std::size_t>(
        eligible.size(), static_cast<std::size_t>(options.beamWidth));
    for (std::size_t index = 0u; index < rootCount; ++index) {
        BeamNode node;
        node.state = {eligible[index].points.back(),
                      eligible[index].endHeading,
                      eligible[index].curvature};
        node.ignored.insert(
            eligible[index].removedCells.begin(),
            eligible[index].removedCells.end());
        node.root = index;
        node.score = candidateCost(eligible[index], mostMaterial);
        beam.push_back(std::move(node));
    }
    for (int depth = 1; depth < options.lookaheadDepth; ++depth) {
        std::vector<BeamNode> expanded;
        for (const BeamNode& node : beam) {
            std::vector<MotionCandidate> futures = expandCandidates(
                node.state, center, radialGrowth, direction, motionLength,
                raster, options, safeSpacing, &node.ignored);
            if (futures.empty()) {
                BeamNode deadEnd = node;
                deadEnd.score += std::pow(0.72, depth) * 1.5;
                expanded.push_back(std::move(deadEnd));
                continue;
            }
            const auto futureMostIt = std::max_element(
                futures.begin(), futures.end(),
                [](const MotionCandidate& left,
                   const MotionCandidate& right) {
                    return left.removedCells.size() < right.removedCells.size();
                });
            const std::size_t futureMost = std::max<std::size_t>(
                1u, futureMostIt->removedCells.size());
            std::sort(
                futures.begin(), futures.end(),
                [&](const MotionCandidate& left,
                    const MotionCandidate& right) {
                    return candidateCost(left, futureMost) <
                           candidateCost(right, futureMost);
                });
            const std::size_t count = std::min<std::size_t>(
                futures.size(), static_cast<std::size_t>(options.beamWidth));
            for (std::size_t index = 0u; index < count; ++index) {
                BeamNode next = node;
                next.state = {futures[index].points.back(),
                              futures[index].endHeading,
                              futures[index].curvature};
                next.ignored.insert(
                    futures[index].removedCells.begin(),
                    futures[index].removedCells.end());
                next.score += std::pow(0.72, depth) *
                              candidateCost(futures[index], futureMost);
                expanded.push_back(std::move(next));
            }
        }
        std::sort(expanded.begin(), expanded.end(),
                  [](const BeamNode& left, const BeamNode& right) {
                      return left.score < right.score;
                  });
        if (expanded.size() > static_cast<std::size_t>(options.beamWidth))
            expanded.resize(static_cast<std::size_t>(options.beamWidth));
        beam = std::move(expanded);
        if (beam.empty())
            break;
    }
    if (beam.empty())
        return eligible.front();
    const std::size_t root = std::min(
        std::min_element(
            beam.begin(), beam.end(),
            [](const BeamNode& left, const BeamNode& right) {
                return left.score < right.score;
            })->root,
        eligible.size() - 1u);
    return eligible[root];
}

std::optional<gp_Pnt2d> componentSeed(
    const AdaptiveRasterComponent& component,
    AdaptiveFreePathRaster& raster,
    const PlannerOptions& options,
    const gp_Pnt2d& current)
{
    std::optional<gp_Pnt2d> best;
    double bestScore = -std::numeric_limits<double>::infinity();
    const std::size_t stride = std::max<std::size_t>(
        1u, component.cells.size() / 256u);
    const int searchRadius = std::max(
        1, static_cast<int>(std::ceil(
               options.toolRadius / raster.cellSize())) + 1);
    for (std::size_t item = 0u; item < component.cells.size(); item += stride) {
        const std::size_t cell = component.cells[item];
        const long long cx = static_cast<long long>(cell % raster.nx());
        const long long cy = static_cast<long long>(cell / raster.nx());
        for (int oy = -searchRadius; oy <= searchRadius; ++oy) {
            for (int ox = -searchRadius; ox <= searchRadius; ++ox) {
                const long long x = cx + ox;
                const long long y = cy + oy;
                if (x < 0 || y < 0 ||
                    x >= static_cast<long long>(raster.nx()) ||
                    y >= static_cast<long long>(raster.ny())) {
                    continue;
                }
                const gp_Pnt2d candidate = raster.point(
                    static_cast<std::size_t>(x),
                    static_cast<std::size_t>(y));
                if (!raster.safe(candidate))
                    continue;
                const double engagement = raster.engagementRatio(
                    candidate, options.toolRadius);
                if (engagement <= 0.0)
                    continue;
                const double overload = std::max(
                    0.0, engagement - 1.35 * options.targetEngagement);
                const double score =
                    2.0 * engagement - 4.0 * overload * overload -
                    0.55 * std::abs(engagement - options.targetEngagement) -
                    0.01 * candidate.Distance(current) /
                        std::max(options.toolRadius, raster.cellSize());
                if (score > bestScore) {
                    bestScore = score;
                    best = candidate;
                }
            }
        }
    }
    return best;
}

std::size_t adaptiveResidualPass(
    PlannerState& state,
    const AdaptiveRasterComponent& component,
    const gp_Pnt2d& seed,
    double initialHeading)
{
    AdaptiveFreePathRaster& raster = state.raster;
    const PlannerOptions& options = state.options;
    const double componentWidth =
        static_cast<double>(component.maxX - component.minX + 1u) * raster.dx();
    const double componentHeight =
        static_cast<double>(component.maxY - component.minY + 1u) * raster.dy();
    const double toolDiameter = 2.0 * options.toolRadius;
    const double maximumRadius = std::max(
        1.60 * toolDiameter,
        std::min(2.75 * toolDiameter,
                 0.32 * std::hypot(componentWidth, componentHeight) +
                     options.toolRadius));
    const double direction = options.clockwise ? -1.0 : 1.0;
    const double motionLength = std::max(
        1.25 * raster.cellSize(),
        std::min(options.stepOver, 0.75 * options.toolRadius));
    const double safeSpacing = std::min(
        motionLength,
        std::max(raster.cellSize(), 0.25 * options.toolRadius));
    MotionState motion{seed, initialHeading, 0.0};
    std::vector<gp_Pnt2d> currentRun;
    std::vector<gp_Pnt2d> pendingLink;
    std::size_t removedTotal = 0u;
    std::size_t usefulSegments = 0u;
    double lastAngle = 0.0;
    double noProgress = 0.0;

    const auto finishRun = [&]() {
        if (currentRun.size() < 2u) {
            currentRun.clear();
            return;
        }
        ClearingLoop loop = makeRun(std::move(currentRun), false);
        if (pendingLink.size() >= 2u)
            loop.preferredLinkFromPrevious = std::move(pendingLink);
        loop.ring = static_cast<int>(state.runs.size());
        state.runs.push_back(std::move(loop));
        currentRun.clear();
        pendingLink.clear();
    };

    for (std::size_t segment = 0u;
         segment < kMaximumLocalSegments &&
         motion.point.Distance(seed) <= maximumRadius;
         ++segment) {
        if ((segment & 0xffu) == 0u && cancelled(state.control))
            return 0u;
        const double engagement = raster.engagementRatio(
            motion.point, options.toolRadius);
        double factor = 1.0;
        if (lastAngle > kTolerance)
            factor = std::clamp(options.targetAngle / lastAngle, 0.38, 1.40);
        else if (segment > 0u && engagement <= kTolerance)
            factor = 1.40;
        else if (engagement > kTolerance)
            factor = std::clamp(
                options.targetEngagement / engagement, 0.38, 1.40);
        const double radialGrowth = options.stepOver * factor / kTwoPi;
        std::optional<MotionCandidate> selected = chooseCandidate(
            motion, seed, radialGrowth, direction, motionLength,
            raster, options, safeSpacing, true);
        if (selected) {
            if (currentRun.empty())
                currentRun.push_back(selected->points.front());
            currentRun.insert(
                currentRun.end(),
                std::next(selected->points.begin()), selected->points.end());
            removedTotal += raster.removePolyline(
                selected->points, options.toolRadius);
            ++usefulSegments;
            lastAngle = selected->engagementAngle;
            state.maximumEngagementAngle = std::max(
                state.maximumEngagementAngle, selected->engagementAngle);
            if (usefulSegments > 1u &&
                selected->engagementAngle > options.maximumAngle + kTolerance) {
                ++state.overloadSegments;
            }
            motion = {selected->points.back(),
                      selected->endHeading,
                      selected->curvature};
            noProgress = 0.0;
            continue;
        }

        std::optional<MotionCandidate> guide = chooseCandidate(
            motion, seed, radialGrowth, direction, motionLength,
            raster, options, safeSpacing, false);
        if (!guide) {
            finishRun();
            motion.heading += direction * 0.35;
            motion.curvature = 0.0;
            noProgress += 0.35;
        } else {
            if (!currentRun.empty()) {
                const gp_Pnt2d last = currentRun.back();
                finishRun();
                pendingLink.push_back(last);
            }
            if (pendingLink.empty())
                pendingLink.push_back(guide->points.front());
            pendingLink.insert(
                pendingLink.end(),
                std::next(guide->points.begin()), guide->points.end());
            motion = {guide->points.back(),
                      guide->endHeading,
                      guide->curvature};
            noProgress += motionLength /
                std::max(options.stepOver, kTolerance);
        }
        if (noProgress >= kTwoPi)
            break;
    }
    finishRun();
    if (removedTotal > 0u)
        ++state.adaptivePasses;
    return removedTotal;
}

std::vector<gp_Pnt2d> adaptiveTransferLink(
    const ClearingLoop& previous,
    const ClearingLoop& next,
    AdaptiveFreePathRaster& raster,
    const PlannerOptions& options,
    bool requireClearedMaterial)
{
    if (previous.points.size() < 2u || next.points.size() < 2u)
        return {};
    const gp_Pnt2d& start = previous.closed
        ? previous.points.front()
        : previous.points.back();
    const gp_Pnt2d& end = next.points.front();
    const gp_Pnt2d& previousTangentPoint = previous.closed
        ? previous.points.back()
        : previous.points[previous.points.size() - 2u];
    const gp_Pnt2d& nextTangentPoint = next.points[1u];
    const double chord = start.Distance(end);
    if (chord <= kTolerance ||
        start.Distance(previousTangentPoint) <= kTolerance ||
        end.Distance(nextTangentPoint) <= kTolerance) {
        return {};
    }
    const double spacing = std::max(
        0.25 * raster.cellSize(), 1.0e-4);
    const double startHeading = std::atan2(
        start.Y() - previousTangentPoint.Y(),
        start.X() - previousTangentPoint.X());
    const double endHeading = std::atan2(
        nextTangentPoint.Y() - end.Y(),
        nextTangentPoint.X() - end.X());
    const double minimumLeadRadius = std::max(
        4.0 * spacing, kTolerance);
    const auto targetRadius = [&](double cuttingLength) {
        return std::max(
            minimumLeadRadius,
            std::min({options.minimumTurnRadius,
                      0.24 * cuttingLength,
                      0.14 * chord}));
    };
    const auto valid = [&](const std::vector<gp_Pnt2d>& points,
                           double exitRadius,
                           double entryRadius) {
        if (polylineLength(points) >
            chord + kPi * (exitRadius + entryRadius)) {
            return false;
        }
        for (std::size_t index = 1u; index < points.size(); ++index) {
            if (!raster.segmentSafe(
                    points[index - 1u], points[index], spacing)) {
                return false;
            }
        }
        return !requireClearedMaterial ||
               raster.polylineRemainingCount(
                   points, options.toolRadius) == 0u;
    };
    auto csc = bestAdaptiveCscLink(
        start, end, startHeading, endHeading,
        targetRadius(polylineLength(previous.points)),
        targetRadius(polylineLength(next.points)),
        minimumLeadRadius, spacing, valid);
    if (csc)
        return std::move(*csc);

    // Match pyocc-exp's component-transfer policy: preserve tangent continuity
    // whenever a valid reduced-radius CSC exists, and use the straight chord
    // only as the final stay-down fallback.
    std::vector<gp_Pnt2d> direct{start, end};
    if (raster.segmentSafe(start, end, spacing) &&
        (!requireClearedMaterial ||
         raster.polylineRemainingCount(
             direct, options.toolRadius) == 0u)) {
        return direct;
    }

    return {};
}

std::vector<gp_Pnt2d> preferredAdaptiveLink(
    const ClearingLoop& previous,
    const ClearingLoop& next,
    AdaptiveFreePathRaster& raster,
    const PlannerOptions& options)
{
    return adaptiveTransferLink(
        previous, next, raster, options, true);
}

std::size_t removeAndRecord(PlannerState& state,
                            ClearingLoop loop,
                            bool recordEngagement = true,
                            double initialDistanceWithoutDiagnostics = 0.0)
{
    if (loop.points.size() < 2u)
        return 0u;
    if (!state.runs.empty() && loop.preferredLinkFromPrevious.empty()) {
        loop.preferredLinkFromPrevious = preferredAdaptiveLink(
            state.runs.back(), loop, state.raster, state.options);
    }
    std::size_t removed = 0u;
    double travelled = 0.0;
    const std::size_t segmentCount = loop.closed
        ? loop.points.size() : loop.points.size() - 1u;
    for (std::size_t segment = 0u; segment < segmentCount; ++segment) {
        const gp_Pnt2d& from = loop.points[segment];
        const gp_Pnt2d& to = loop.points[
            (segment + 1u) % loop.points.size()];
        const double segmentLength = from.Distance(to);
        double angle = 0.0;
        if (recordEngagement) {
            const gp_Pnt2d midpoint(
                0.5 * (from.X() + to.X()),
                0.5 * (from.Y() + to.Y()));
            angle = state.raster.angularEngagement(
                midpoint, state.options.toolRadius,
                state.options.angleSamples).maximumArcAngle;
        }
        const std::size_t segmentRemoved = state.raster.removeSegment(
            from, to, state.options.toolRadius);
        removed += segmentRemoved;
        travelled += segmentLength;
        if (recordEngagement && segmentRemoved > 0u && segment > 0u &&
            travelled > initialDistanceWithoutDiagnostics) {
            state.maximumEngagementAngle = std::max(
                state.maximumEngagementAngle, angle);
            if (angle > state.options.maximumAngle + kTolerance)
                ++state.overloadSegments;
        }
    }
    loop.ring = static_cast<int>(state.runs.size());
    state.runs.push_back(std::move(loop));
    return removed;
}

void appendBoundaryPasses(PlannerState& state,
                          const PathsD& reachable)
{
    gp_Pnt2d current = state.runs.empty()
        ? areaCenter(reachable)
        : state.runs.back().points.back();
    std::vector<ClearingLoop> boundaries;
    for (const PathD& path : reachable) {
        if (path.size() < 3u)
            continue;
        ClearingLoop loop = makeRun(
            pathPoints(path), true, signedArea(path) < 0.0);
        rotateClosedRun(loop, current);
        boundaries.push_back(std::move(loop));
    }
    std::sort(boundaries.begin(), boundaries.end(),
              [&](const ClearingLoop& left, const ClearingLoop& right) {
                  return left.points.front().SquareDistance(current) <
                         right.points.front().SquareDistance(current);
              });
    for (ClearingLoop& loop : boundaries) {
        current = loop.points.front();
        removeAndRecord(state, std::move(loop), false);
    }
}

void rectanglePrimaryPass(PlannerState& state,
                          const PathsD& reachable)
{
    const double spacing = std::max(
        state.raster.cellSize(),
        std::min(state.options.stepOver,
                 0.5 * state.options.toolRadius));
    std::vector<gp_Pnt2d> points = rectangleWavefrontSpiral(
        reachable, state.options, spacing);
    if (state.options.clockwise)
        std::reverse(points.begin(), points.end());
    const std::size_t removed = removeAndRecord(
        state, makeRun(std::move(points), false), true,
        2.0 * state.options.toolRadius);
    if (removed > 0u)
        ++state.adaptivePasses;
    ++state.wavefrontLayers;
}

struct RadialBranchSector {
    double angle = 0.0;
    double startAngle = 0.0;
    double endAngle = 0.0;
};

std::vector<RadialBranchSector> radialBranchSchedule(
    const std::vector<AdaptiveRasterComponent>& components,
    const gp_Pnt2d& center,
    double startAngle,
    bool clockwise)
{
    std::vector<double> angles;
    angles.reserve(components.size());
    for (const AdaptiveRasterComponent& component : components) {
        double angle = std::fmod(
            std::atan2(component.centroid.Y() - center.Y(),
                       component.centroid.X() - center.X()) + kTwoPi,
            kTwoPi);
        angles.push_back(angle);
    }
    if (angles.empty())
        return {};
    std::sort(angles.begin(), angles.end());
    double normalizedStart = std::fmod(startAngle + kTwoPi, kTwoPi);
    const auto advance = [](double from, double to) {
        double result = std::fmod(to - from, kTwoPi);
        return result < 0.0 ? result + kTwoPi : result;
    };
    std::size_t first = 0u;
    double firstAdvance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0u; index < angles.size(); ++index) {
        const double value = clockwise
            ? advance(angles[index], normalizedStart)
            : advance(normalizedStart, angles[index]);
        if (value < firstAdvance) {
            firstAdvance = value;
            first = index;
        }
    }

    std::vector<RadialBranchSector> schedule;
    schedule.reserve(angles.size());
    for (std::size_t order = 0u; order < angles.size(); ++order) {
        const std::size_t index = clockwise
            ? (first + angles.size() - order) % angles.size()
            : (first + order) % angles.size();
        double branch = angles[index];
        if (!schedule.empty()) {
            if (!clockwise && branch <= schedule.back().angle)
                branch += kTwoPi;
            if (clockwise && branch >= schedule.back().angle)
                branch -= kTwoPi;
        }
        const double previousRaw = angles[
            (index + angles.size() - 1u) % angles.size()];
        const double nextRaw = angles[(index + 1u) % angles.size()];
        const double previousGap = angles.size() == 1u
            ? kTwoPi : advance(previousRaw, angles[index]);
        const double nextGap = angles.size() == 1u
            ? kTwoPi : advance(angles[index], nextRaw);
        schedule.push_back({
            branch,
            branch - 0.5 * previousGap,
            branch + 0.5 * nextGap});
    }
    return schedule;
}

void prependCoreTransition(PlannerState& state,
                           const ClearingLoop& previous,
                           ClearingLoop& following)
{
    std::vector<gp_Pnt2d> transition = adaptiveTransferLink(
        previous, following, state.raster, state.options, false);
    if (transition.size() < 2u)
        return;
    std::optional<std::size_t> firstCutSegment;
    for (std::size_t index = 1u; index < transition.size(); ++index) {
        if (!state.raster.segmentRemainingIndices(
                 transition[index - 1u], transition[index],
                 state.options.toolRadius).empty()) {
            firstCutSegment = index - 1u;
            break;
        }
    }
    if (!firstCutSegment) {
        following.preferredLinkFromPrevious = std::move(transition);
        return;
    }

    const std::size_t split = *firstCutSegment;
    if (split > 0u) {
        following.preferredLinkFromPrevious.assign(
            transition.begin(),
            transition.begin() + static_cast<std::ptrdiff_t>(split + 1u));
    }
    std::vector<gp_Pnt2d> cutting;
    cutting.reserve(
        transition.size() - split + following.points.size());
    cutting.insert(
        cutting.end(),
        transition.begin() + static_cast<std::ptrdiff_t>(split),
        transition.end());
    cutting.insert(
        cutting.end(), std::next(following.points.begin()),
        following.points.end());
    following.points = std::move(cutting);
}

std::size_t radialBranchPass(PlannerState& state,
                             const gp_Pnt2d& center,
                             double rayLimit,
                             const RadialBranchSector& sector,
                             const std::optional<ClearingLoop>& entryRun)
{
    constexpr int kAngularSamples = 160;
    double coreRadius = std::numeric_limits<double>::infinity();
    for (int sample = 0; sample < 180; ++sample) {
        coreRadius = std::min(
            coreRadius,
            rayReach(state.raster, center,
                     kTwoPi * static_cast<double>(sample) / 180.0,
                     rayLimit));
    }
    coreRadius -= 0.25 * state.raster.cellSize();
    const double maximumRadius = rayReach(
        state.raster, center, sector.angle, rayLimit) -
        0.25 * state.raster.cellSize();
    if (!std::isfinite(coreRadius) ||
        maximumRadius <= coreRadius + kTolerance) {
        return 0u;
    }

    const double localSpacing = std::max(
        state.raster.cellSize(),
        std::min(state.options.stepOver,
                 0.5 * state.options.toolRadius));
    const double safeSpacing = std::max(
        0.25 * state.raster.cellSize(), 1.0e-4);
    const double sectorWidth = sector.endAngle - sector.startAngle;
    if (sectorWidth <= kTolerance)
        return 0u;

    const auto safeInterval = [&](double radius)
        -> std::optional<std::pair<double, double>> {
        std::array<unsigned char, kAngularSamples + 1> safe{};
        std::array<double, kAngularSamples + 1> angles{};
        for (int index = 0; index <= kAngularSamples; ++index) {
            const double angle = sector.startAngle + sectorWidth *
                static_cast<double>(index) /
                static_cast<double>(kAngularSamples);
            angles[static_cast<std::size_t>(index)] = angle;
            safe[static_cast<std::size_t>(index)] = state.raster.safe(
                gp_Pnt2d(center.X() + radius * std::cos(angle),
                         center.Y() + radius * std::sin(angle)))
                ? 1u : 0u;
        }
        int middle = std::clamp(
            static_cast<int>(std::lround(
                (sector.angle - sector.startAngle) /
                sectorWidth * kAngularSamples)),
            0, kAngularSamples);
        if (safe[static_cast<std::size_t>(middle)] == 0u)
            return std::nullopt;
        int first = middle;
        int last = middle;
        while (first > 0 && safe[static_cast<std::size_t>(first - 1)] != 0u)
            --first;
        while (last < kAngularSamples &&
               safe[static_cast<std::size_t>(last + 1)] != 0u)
            ++last;
        const double angularStep = sectorWidth /
                                   static_cast<double>(kAngularSamples);
        const int tangentInset = std::max(
            3, static_cast<int>(std::ceil(
                0.20 * state.options.minimumTurnRadius /
                std::max(radius * angularStep, kTolerance))));
        const int endpointInset = std::min(
            tangentInset, std::max(0, (last - first - 2) / 2));
        first += endpointInset;
        last -= endpointInset;
        if (first >= last)
            return std::nullopt;
        return std::make_pair(
            angles[static_cast<std::size_t>(first)],
            angles[static_cast<std::size_t>(last)]);
    };

    std::size_t removedTotal = 0u;
    double previousRadius = coreRadius;
    double radialStep = std::min(
        state.options.stepOver, maximumRadius - coreRadius);
    const double minimumStep = std::max(
        0.30 * state.options.stepOver,
        0.75 * state.raster.cellSize());
    std::optional<ClearingLoop> previousArc;
    for (int layer = 0;
         previousRadius < maximumRadius - kTolerance && layer < 80;
         ++layer) {
        const double candidateRadius = std::min(
            maximumRadius, previousRadius + radialStep);
        const auto interval = safeInterval(candidateRadius);
        if (!interval)
            break;
        const double startAngle = state.options.clockwise
            ? interval->second : interval->first;
        const double endAngle = state.options.clockwise
            ? interval->first : interval->second;
        const double signedAdvance = endAngle - startAngle;
        const int pointCount = std::max(
            2, static_cast<int>(std::ceil(
                candidateRadius * std::abs(signedAdvance) /
                localSpacing)) + 1);
        std::vector<gp_Pnt2d> points;
        points.reserve(static_cast<std::size_t>(pointCount));
        for (int index = 0; index < pointCount; ++index) {
            const double angle = startAngle + signedAdvance *
                static_cast<double>(index) /
                static_cast<double>(pointCount - 1);
            points.emplace_back(
                center.X() + candidateRadius * std::cos(angle),
                center.Y() + candidateRadius * std::sin(angle));
        }
        bool safe = true;
        for (std::size_t index = 1u; index < points.size(); ++index) {
            if (!state.raster.segmentSafe(
                    points[index - 1u], points[index], safeSpacing)) {
                safe = false;
                break;
            }
        }
        if (!safe)
            break;

        std::vector<double> predictedAngles;
        const std::size_t stride = std::max<std::size_t>(
            1u, points.size() / 12u);
        for (std::size_t index = 0u; index < points.size(); index += stride) {
            predictedAngles.push_back(state.raster.angularEngagement(
                points[index], state.options.toolRadius,
                state.options.angleSamples).maximumArcAngle);
        }
        const double predictedMaximum = predictedAngles.empty()
            ? 0.0
            : *std::max_element(
                  predictedAngles.begin(), predictedAngles.end());
        const double increment = candidateRadius - previousRadius;
        if (predictedMaximum >
                state.options.maximumAngle + kTolerance &&
            increment > minimumStep + kTolerance) {
            radialStep = std::max(minimumStep, 0.65 * increment);
            --layer;
            continue;
        }

        ClearingLoop arc = makeRun(std::move(points), false);
        if (!previousArc && entryRun)
            prependCoreTransition(state, *entryRun, arc);
        if (state.raster.polylineRemainingCount(
                arc.points, state.options.toolRadius) > 0u) {
            const std::size_t removed = removeAndRecord(
                state, std::move(arc));
            if (removed > 0u) {
                removedTotal += removed;
                previousArc = state.runs.back();
            }
        }
        previousRadius = candidateRadius;

        double sum = 0.0;
        int count = 0;
        for (const double angle : predictedAngles) {
            if (angle > kTolerance) {
                sum += angle;
                ++count;
            }
        }
        if (count > 0) {
            const double mean = sum / static_cast<double>(count);
            const double feedback = std::sqrt(
                state.options.targetAngle / std::max(mean, kTolerance));
            radialStep = std::max(
                minimumStep,
                std::min(state.options.stepOver,
                         radialStep * std::clamp(feedback, 0.65, 1.20)));
        } else {
            radialStep = std::min(
                state.options.stepOver, 1.20 * radialStep);
        }
    }
    if (removedTotal > 0u) {
        ++state.adaptivePasses;
        ++state.radialPasses;
    }
    return removedTotal;
}

void radialPrimaryPass(PlannerState& state,
                       const PathsD& reachable,
                       const gp_Pnt2d& center)
{
    const double spacing = std::max(
        0.75 * state.raster.cellSize(),
        std::min(state.options.stepOver,
                 0.5 * state.options.toolRadius));
    std::vector<gp_Pnt2d> points = constrainedCoreSpiral(
        reachable, state.raster, center, state.options, spacing);
    const std::size_t coreRemoved = removeAndRecord(
        state, makeRun(std::move(points), false), true,
        2.0 * state.options.toolRadius);
    ++state.wavefrontLayers;
    if (coreRemoved > 0u) {
        ++state.adaptivePasses;
        ++state.centralSpiralRuns;
    }

    const std::size_t minimumCells = std::max<std::size_t>(
        2u, static_cast<std::size_t>(std::ceil(
                0.0005 * state.raster.initialCount())));
    const std::vector<AdaptiveRasterComponent> components =
        state.raster.components(minimumCells);
    state.radialComponents = static_cast<int>(components.size());
    if (components.empty() || state.runs.empty())
        return;

    const Clipper2Lib::RectD bounds = Clipper2Lib::GetBounds(reachable);
    const double rayLimit = std::hypot(
        bounds.right - bounds.left, bounds.bottom - bounds.top);
    const gp_Pnt2d& centralEnd = state.runs.back().points.back();
    const double startAngle = std::atan2(
        centralEnd.Y() - center.Y(), centralEnd.X() - center.X());
    const std::vector<RadialBranchSector> schedule = radialBranchSchedule(
        components, center, startAngle, state.options.clockwise);
    std::optional<ClearingLoop> firstEntry = state.runs.back();
    bool completedBranch = false;
    for (const RadialBranchSector& sector : schedule) {
        const std::size_t before = state.runs.size();
        radialBranchPass(
            state, center, rayLimit, sector,
            completedBranch ? std::nullopt : firstEntry);
        if (state.runs.size() > before)
            completedBranch = true;
    }
}

void genericWavefrontPrimaryPass(PlannerState& state,
                                 const PathsD& reachable,
                                 int precision,
                                 double arcTolerance)
{
    const Clipper2Lib::RectD bounds = Clipper2Lib::GetBounds(reachable);
    const double maximum = 0.5 * std::hypot(
        bounds.right - bounds.left, bounds.bottom - bounds.top);
    std::vector<std::pair<double, PathsD>> levels;
    for (double distance = std::max(
             0.35 * state.options.stepOver,
             0.75 * state.raster.cellSize());
         distance <= maximum + kTolerance;
         distance += state.options.stepOver) {
        PathsD paths = offsetArea(
            reachable, -distance, precision, arcTolerance);
        if (!paths.empty())
            levels.emplace_back(distance, std::move(paths));
    }
    std::reverse(levels.begin(), levels.end());
    gp_Pnt2d current = areaCenter(reachable);
    std::size_t removedTotal = 0u;
    for (auto& level : levels) {
        PathsD& paths = level.second;
        std::vector<ClearingLoop> group;
        for (const PathD& path : paths) {
            if (path.size() < 3u)
                continue;
            ClearingLoop loop = makeRun(
                pathPoints(path), true, signedArea(path) < 0.0);
            rotateClosedRun(loop, current);
            group.push_back(std::move(loop));
        }
        while (!group.empty()) {
            const auto nearest = std::min_element(
                group.begin(), group.end(),
                [&](const ClearingLoop& left, const ClearingLoop& right) {
                    return left.points.front().SquareDistance(current) <
                           right.points.front().SquareDistance(current);
                });
            ClearingLoop loop = std::move(*nearest);
            group.erase(nearest);
            current = loop.points.front();
            removedTotal += removeAndRecord(state, std::move(loop));
        }
        ++state.wavefrontLayers;
    }
    if (removedTotal > 0u)
        ++state.adaptivePasses;
}

void residualCleanup(PlannerState& state)
{
    gp_Pnt2d current = state.runs.empty()
        ? gp_Pnt2d()
        : state.runs.back().points.back();
    const std::size_t minimumCells = std::max<std::size_t>(
        2u, static_cast<std::size_t>(std::ceil(
                0.0005 * state.raster.initialCount())));
    while (state.raster.remainingFraction() >
               state.options.remainingTolerance &&
           state.adaptivePasses < state.options.maximumPasses) {
        if (cancelled(state.control))
            return;
        std::vector<AdaptiveRasterComponent> components =
            state.raster.components(minimumCells);
        if (components.empty())
            return;
        struct Plan {
            double priority = 0.0;
            std::size_t index = 0u;
            gp_Pnt2d seed;
        };
        std::vector<Plan> plans;
        for (std::size_t index = 0u; index < components.size(); ++index) {
            const std::optional<gp_Pnt2d> seed = componentSeed(
                components[index], state.raster, state.options, current);
            if (!seed)
                continue;
            plans.push_back({
                seed->Distance(current) /
                        std::max(state.options.toolRadius,
                                 state.raster.cellSize()) -
                    0.15 * std::log1p(
                        static_cast<double>(components[index].cells.size())),
                index,
                *seed});
        }
        std::sort(plans.begin(), plans.end(),
                  [](const Plan& left, const Plan& right) {
                      return left.priority < right.priority;
                  });
        bool progress = false;
        for (const Plan& plan : plans) {
            AdaptiveRasterComponent& component = components[plan.index];
            double heading = component.principalAngle;
            const double toward = std::atan2(
                component.centroid.Y() - plan.seed.Y(),
                component.centroid.X() - plan.seed.X());
            if (std::abs(wrappedAngle(heading - toward)) > 0.5 * kPi)
                heading += kPi;
            if (adaptiveResidualPass(
                    state, component, plan.seed, heading) > 0u) {
                current = state.runs.back().points.back();
                progress = true;
                break;
            }
        }
        if (!progress)
            return;
    }
}

void finalizeOutput(std::vector<ClearingLoop>& runs, bool reverse)
{
    if (reverse) {
        for (ClearingLoop& run : runs) {
            if (run.closed)
                std::reverse(run.points.begin(), run.points.end());
        }
    }
    for (std::size_t index = 0u; index < runs.size(); ++index)
        runs[index].ring = static_cast<int>(index);
}

} // namespace

std::vector<ClearingLoop> generateFreePathLoops(
    const PathsD& region2d,
    double firstDelta,
    double step,
    double toolDiameter,
    const AdaptiveClearingParams& adaptive,
    bool reverse,
    int precision,
    double arcTolerance,
    int& ringCountOut,
    const ToolpathGenerationControl* control,
    AdaptiveFreePathDiagnostics* diagnostics)
{
    ringCountOut = 0;
    if (diagnostics)
        *diagnostics = {};
    if (cancelled(control) || region2d.empty() ||
        !std::isfinite(step) || step <= kTolerance ||
        !std::isfinite(toolDiameter) || toolDiameter <= kTolerance ||
        step > toolDiameter + kTolerance) {
        return {};
    }

    PathsD reachable = cleanPaths(region2d, precision);
    if (std::isfinite(firstDelta) && firstDelta > kTolerance) {
        reachable = offsetArea(
            reachable, -firstDelta, precision, arcTolerance);
    }
    if (reachable.empty())
        return {};

    const PlannerOptions options = normalizedOptions(
        toolDiameter, step, adaptive, reverse);
    PathsD materialArea = offsetArea(
        reachable, options.toolRadius, precision, arcTolerance);
    if (materialArea.empty())
        materialArea = reachable;
    AdaptiveFreePathRaster raster(
        materialArea, reachable, options.cellSize);
    if (!raster.valid())
        return {};

    PlannerState state{raster, options, control};
    const gp_Pnt2d center = areaCenter(reachable);
    if (isAxisAlignedRectangle(reachable)) {
        rectanglePrimaryPass(state, reachable);
    } else if (supportsRadialStrategy(
                   reachable, raster, center, options)) {
        radialPrimaryPass(state, reachable, center);
    } else {
        genericWavefrontPrimaryPass(
            state, reachable, precision, arcTolerance);
    }
    if (cancelled(control))
        return {};

    appendBoundaryPasses(state, reachable);
    residualCleanup(state);
    if (cancelled(control))
        return {};
    if (state.runs.empty())
        return {};

    finalizeOutput(state.runs, reverse);
    ringCountOut = std::max(1, state.wavefrontLayers + state.adaptivePasses);
    if (diagnostics) {
        diagnostics->adaptivePasses = state.adaptivePasses;
        diagnostics->remainingComponents = static_cast<int>(
            raster.components(std::max<std::size_t>(
                2u, static_cast<std::size_t>(std::ceil(
                        0.0005 * raster.initialCount())))).size());
        diagnostics->overloadSegments = state.overloadSegments;
        diagnostics->wavefrontLayers = state.wavefrontLayers;
        diagnostics->centralSpiralRuns = state.centralSpiralRuns;
        diagnostics->radialComponents = state.radialComponents;
        diagnostics->radialPasses = state.radialPasses;
        diagnostics->remainingMaterialFraction = raster.remainingFraction();
        diagnostics->maximumEngagementAngleDeg =
            state.maximumEngagementAngle * 180.0 / kPi;
    }
    return state.runs;
}

} // namespace oap::toolpath::detail

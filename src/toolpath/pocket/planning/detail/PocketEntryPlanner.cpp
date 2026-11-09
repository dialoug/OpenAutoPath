// NX 风格槽腔入刀规划：区分沿轮廓、直线、往复和螺旋斜坡，候选路径均
// 经过刀心安全区采样；直线/往复会在首段切向两侧搜索可行方向。
#include "toolpath/pocket/planning/detail/PocketEntryPlanner.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

#include "toolpath/pocket/planning/detail/PocketGeometryUtils.h"
#include "toolpath/pocket/planning/detail/PocketSafetyGeometry.h"

namespace oap::toolpath::detail {

namespace {

bool entryPathSafe(const PocketEntryPath& entry,
                   const PocketRegion& region,
                   const ToolpathParams& params,
                   const ClearingLoop& loop,
                   const PocketSafetyArea* safetyArea)
{
    if (entry.feedPoints.size() < 2u)
        return false;

    const double requiredMargin =
        params.tool.radius() +
        std::max(0.0, params.clearing.extraOffset);
    const double sampleSpacing = std::clamp(
        requiredMargin > 0.0 ? requiredMargin / 3.0 : 0.25,
        0.05, 0.5);
    const double quantization = std::pow(
        10.0, -std::clamp(params.clearing.precision, 0, 8));
    const double tolerance = std::max(0.02, 4.0 * quantization);
    for (std::size_t segment = 1u;
         segment < entry.feedPoints.size(); ++segment) {
        const gp_Pnt& from = entry.feedPoints[segment - 1u];
        const gp_Pnt& to = entry.feedPoints[segment];
        const std::size_t samples = std::max<std::size_t>(
            1u, static_cast<std::size_t>(
                    std::ceil(from.Distance(to) / sampleSpacing)));
        for (std::size_t sample = 0u; sample <= samples; ++sample) {
            const double parameter =
                static_cast<double>(sample) /
                static_cast<double>(samples);
            const gp_Pnt point(
                from.X() + (to.X() - from.X()) * parameter,
                from.Y() + (to.Y() - from.Y()) * parameter,
                from.Z() + (to.Z() - from.Z()) * parameter);
            const gp_Pnt2d projected =
                projectToPocketPlane(point, region.frame);
            const bool clear = safetyArea
                ? safetyArea->clear(projected, loop.depthIndex)
                : isPocketCutterCenterSafe(
                      projected, region, requiredMargin, tolerance);
            if (!clear) {
                return false;
            }
        }
    }
    return true;
}

constexpr double kEntryTolerance = 1.0e-9;
constexpr std::size_t kMaximumEntrySegments = 10000;

struct WalkSample {
    gp_Pnt2d point;
    double distance = 0.0;
};

bool firstLoopTangent(const ClearingLoop& loop,
                      double& tangentX,
                      double& tangentY);

PocketEntryPath verticalEntry(const PocketRegion& region,
                              const ClearingLoop& loop,
                              double safeOffset)
{
    PocketEntryPath result;
    result.startPoint = loop.points.front();
    result.feedPoints.push_back(
        pointFrom2dAtDepth(result.startPoint, region.frame, safeOffset));
    result.feedPoints.push_back(loop.world.front());
    return result;
}

double configuredRampLength(double verticalDrop, double angleDegrees)
{
    if (verticalDrop <= kEntryTolerance ||
        !std::isfinite(angleDegrees) ||
        angleDegrees <= 0.0 ||
        angleDegrees >= 90.0) {
        return 0.0;
    }
    const double radians =
        angleDegrees * std::acos(-1.0) / 180.0;
    const double tangent = std::tan(radians);
    return tangent > kEntryTolerance
        ? verticalDrop / tangent : 0.0;
}

bool exceedsLengthLimit(double length, const ClearingParams& params)
{
    return std::isfinite(params.maximumRampLength) &&
           params.maximumRampLength > 0.0 &&
           length > params.maximumRampLength + kEntryTolerance;
}

bool buildRampApproachWalk(const ClearingLoop& loop,
                           double requestedLength,
                           std::vector<WalkSample>& samples)
{
    // 沿清料环收集一段长度为 requestedLength 的入刀接近路径：
    // 闭合环从起点逆向往上游走（之后整体反转），开放环则来回扫描。
    // 采样点连同已走过的弧长距离一起返回，供斜坡深度插值使用。
    samples.clear();
    if (loop.points.size() < 2 ||
        !std::isfinite(requestedLength) ||
        requestedLength <= kEntryTolerance) {
        return false;
    }

    samples.push_back({loop.points.front(), 0.0});
    std::size_t index = 0;
    int direction = 1;
    double travelled = 0.0;
    std::size_t attempts = 0;

    while (travelled + kEntryTolerance < requestedLength &&
           samples.size() <= kMaximumEntrySegments &&
           attempts < 2 * kMaximumEntrySegments) {
        ++attempts;
        std::size_t next = 0;
        if (loop.closed) {
            // 从环起点逆向往上游走。随后反转这些采样点,使斜坡以与平面
            // 切削相同的方向到达起点。完整环在满深度处再次经过此进刀路径,
            // 而不是立即原路返回。
            next = index == 0 ? loop.points.size() - 1 : index - 1;
        } else {
            if (direction > 0 && index + 1 >= loop.points.size())
                direction = -1;
            else if (direction < 0 && index == 0)
                direction = 1;
            next = static_cast<std::size_t>(
                static_cast<int>(index) + direction);
        }

        const gp_Pnt2d& from = loop.points[index];
        const gp_Pnt2d& to = loop.points[next];
        const double segmentLength = from.Distance(to);
        if (segmentLength <= kEntryTolerance) {
            index = next;
            continue;
        }

        const double remaining = requestedLength - travelled;
        if (segmentLength >= remaining - kEntryTolerance) {
            const double fraction =
                std::clamp(remaining / segmentLength, 0.0, 1.0);
            samples.push_back({
                gp_Pnt2d(
                    from.X() + fraction * (to.X() - from.X()),
                    from.Y() + fraction * (to.Y() - from.Y())),
                requestedLength});
            travelled = requestedLength;
            break;
        }

        travelled += segmentLength;
        samples.push_back({to, travelled});
        index = next;
    }

    return travelled + kEntryTolerance >= requestedLength &&
           samples.size() >= 2 &&
           samples.size() <= kMaximumEntrySegments;
}

std::optional<PocketEntryPath> rampOnShapeEntry(
    const PocketRegion& region,
    const ToolpathParams& params,
    const ClearingLoop& loop,
    double safeOffset,
    double rampStartOffset)
{
    const double cutOffset = loop.depthOffset;
    const double verticalDrop =
        std::max(0.0, rampStartOffset - cutOffset);
    const double requiredLength = configuredRampLength(
        verticalDrop, params.clearing.rampAngleDeg);
    if (requiredLength <= kEntryTolerance ||
        exceedsLengthLimit(requiredLength, params.clearing)) {
        return std::nullopt;
    }

    std::vector<WalkSample> approach;
    if (!buildRampApproachWalk(loop, requiredLength, approach))
        return std::nullopt;

    PocketEntryPath result;
    result.usedMode = PocketEntryMode::RampOnShape;
    result.startPoint = approach.back().point;
    result.feedPoints.reserve(approach.size() + 1);
    result.feedPoints.push_back(pointFrom2dAtDepth(
        result.startPoint, region.frame, safeOffset));
    for (auto iterator = approach.rbegin();
         iterator != approach.rend(); ++iterator) {
        const double fraction = std::clamp(
            iterator->distance / requiredLength, 0.0, 1.0);
        const double depth =
            cutOffset + verticalDrop * fraction;
        result.feedPoints.push_back(
            pointFrom2dAtDepth(iterator->point, region.frame, depth));
    }
    result.feedPoints.back() = loop.world.front();
    return result;
}

double rotateX(double x, double y, double angle)
{
    return x * std::cos(angle) - y * std::sin(angle);
}

double rotateY(double x, double y, double angle)
{
    return x * std::sin(angle) + y * std::cos(angle);
}

std::vector<double> rampDirectionOffsets()
{
    std::vector<double> result;
    result.reserve(38u);
    result.push_back(0.0);
    constexpr double kStepDegrees = 5.0;
    for (double degrees = kStepDegrees;
         degrees <= 45.0; degrees += kStepDegrees) {
        const double radians = degrees * std::acos(-1.0) / 180.0;
        result.push_back(radians);
        result.push_back(-radians);
    }
    // If the preferred arrival direction is blocked (common when a loop
    // starts near a corner), search the opposite cutting direction as NX's
    // engage direction alternatives do.
    result.push_back(std::acos(-1.0));
    for (double degrees = kStepDegrees;
         degrees <= 45.0; degrees += kStepDegrees) {
        const double radians = degrees * std::acos(-1.0) / 180.0;
        result.push_back(std::acos(-1.0) + radians);
        result.push_back(std::acos(-1.0) - radians);
    }
    return result;
}

std::optional<PocketEntryPath> straightRampEntry(
    const PocketRegion& region,
    const ToolpathParams& params,
    const ClearingLoop& loop,
    double safeOffset,
    double rampStartOffset,
    const PocketSafetyArea* safetyArea)
{
    const double verticalDrop =
        std::max(0.0, rampStartOffset - loop.depthOffset);
    const double requiredLength = configuredRampLength(
        verticalDrop, params.clearing.rampAngleDeg);
    if (requiredLength <= kEntryTolerance ||
        exceedsLengthLimit(requiredLength, params.clearing)) {
        return std::nullopt;
    }

    double tangentX = 0.0;
    double tangentY = 0.0;
    if (!firstLoopTangent(loop, tangentX, tangentY))
        return std::nullopt;

    const gp_Pnt2d& target = loop.points.front();
    for (const double offset : rampDirectionOffsets()) {
        const double directionX = rotateX(tangentX, tangentY, offset);
        const double directionY = rotateY(tangentX, tangentY, offset);
        const gp_Pnt2d start(
            target.X() - directionX * requiredLength,
            target.Y() - directionY * requiredLength);

        PocketEntryPath candidate;
        candidate.usedMode = PocketEntryMode::LinearRamp;
        candidate.startPoint = start;
        candidate.feedPoints = {
            pointFrom2dAtDepth(start, region.frame, safeOffset),
            pointFrom2dAtDepth(start, region.frame, rampStartOffset),
            loop.world.front()};
        if (entryPathSafe(
                candidate, region, params, loop, safetyArea)) {
            return candidate;
        }
    }
    return std::nullopt;
}

std::optional<PocketEntryPath> zigzagRampEntry(
    const PocketRegion& region,
    const ToolpathParams& params,
    const ClearingLoop& loop,
    double safeOffset,
    double rampStartOffset,
    const PocketSafetyArea* safetyArea)
{
    const double verticalDrop =
        std::max(0.0, rampStartOffset - loop.depthOffset);
    const double requiredLength = configuredRampLength(
        verticalDrop, params.clearing.rampAngleDeg);
    if (requiredLength <= kEntryTolerance ||
        exceedsLengthLimit(requiredLength, params.clearing)) {
        return std::nullopt;
    }

    double tangentX = 0.0;
    double tangentY = 0.0;
    if (!firstLoopTangent(loop, tangentX, tangentY))
        return std::nullopt;

    double width = params.clearing.zigzagRampWidth;
    if (!std::isfinite(width) || width <= kEntryTolerance)
        width = std::max(params.tool.diameter * 3.0, 0.5);
    width = std::min(width, requiredLength);
    if (width <= kEntryTolerance)
        return std::nullopt;
    const std::size_t fullStrokes = static_cast<std::size_t>(
        std::floor(requiredLength / width));
    if (fullStrokes + 3u > kMaximumEntrySegments)
        return std::nullopt;

    const gp_Pnt2d& target = loop.points.front();
    for (const double offset : rampDirectionOffsets()) {
        const double directionX = rotateX(tangentX, tangentY, offset);
        const double directionY = rotateY(tangentX, tangentY, offset);
        const gp_Pnt2d other(
            target.X() - directionX * width,
            target.Y() - directionY * width);

        std::vector<WalkSample> reverseWalk;
        reverseWalk.reserve(fullStrokes + 2u);
        reverseWalk.push_back({target, 0.0});
        double travelled = 0.0;
        bool moveToOther = true;
        while (travelled + kEntryTolerance < requiredLength) {
            const double distance = std::min(
                width, requiredLength - travelled);
            const gp_Pnt2d& destination = moveToOther ? other : target;
            const gp_Pnt2d& from = reverseWalk.back().point;
            const double fraction = distance / width;
            reverseWalk.push_back({
                gp_Pnt2d(
                    from.X() + (destination.X() - from.X()) * fraction,
                    from.Y() + (destination.Y() - from.Y()) * fraction),
                travelled + distance});
            travelled += distance;
            if (distance + kEntryTolerance >= width)
                moveToOther = !moveToOther;
        }

        PocketEntryPath candidate;
        candidate.usedMode = PocketEntryMode::ZigZagRamp;
        candidate.startPoint = reverseWalk.back().point;
        candidate.feedPoints.reserve(reverseWalk.size() + 1u);
        candidate.feedPoints.push_back(pointFrom2dAtDepth(
            candidate.startPoint, region.frame, safeOffset));
        for (auto iterator = reverseWalk.rbegin();
             iterator != reverseWalk.rend(); ++iterator) {
            const double fraction = std::clamp(
                iterator->distance / requiredLength, 0.0, 1.0);
            candidate.feedPoints.push_back(pointFrom2dAtDepth(
                iterator->point, region.frame,
                loop.depthOffset + verticalDrop * fraction));
        }
        candidate.feedPoints.back() = loop.world.front();
        if (entryPathSafe(
                candidate, region, params, loop, safetyArea)) {
            return candidate;
        }
    }
    return std::nullopt;
}

bool firstLoopTangent(const ClearingLoop& loop,
                      double& tangentX,
                      double& tangentY)
{
    const gp_Pnt2d& start = loop.points.front();
    for (std::size_t index = 1; index < loop.points.size(); ++index) {
        const double dx = loop.points[index].X() - start.X();
        const double dy = loop.points[index].Y() - start.Y();
        const double length = std::hypot(dx, dy);
        if (length <= kEntryTolerance)
            continue;
        tangentX = dx / length;
        tangentY = dy / length;
        return true;
    }
    return false;
}

std::optional<PocketEntryPath> helicalRampEntry(
    const PocketRegion& region,
    const ToolpathParams& params,
    const ClearingLoop& loop,
    double safeOffset,
    double rampStartOffset)
{
    // 螺旋斜坡策略：在环起点切向的左右两侧分别布置一个完整刀心圆，
    // 圆上每个采样点都必须满足刀心安全（边界+岛余量），才采用该侧；
    // 圈数由斜坡长度与圆周长之比决定。
    const double cutOffset = loop.depthOffset;
    const double verticalDrop =
        std::max(0.0, rampStartOffset - cutOffset);
    const double requiredLength = configuredRampLength(
        verticalDrop, params.clearing.rampAngleDeg);
    if (requiredLength <= kEntryTolerance)
        return std::nullopt;

    double diameter = params.clearing.helixDiameter;
    if (!std::isfinite(diameter) || diameter <= kEntryTolerance)
        diameter = params.tool.diameter;
    const double radius = 0.5 * diameter;
    if (!std::isfinite(radius) || radius <= kEntryTolerance)
        return std::nullopt;

    const double circumference = 2.0 * std::acos(-1.0) * radius;
    const std::size_t turns = std::max<std::size_t>(
        1, static_cast<std::size_t>(
               std::ceil(requiredLength / circumference)));
    const double actualLength =
        static_cast<double>(turns) * circumference;
    if (exceedsLengthLimit(actualLength, params.clearing))
        return std::nullopt;

    const double requiredMargin =
        params.tool.radius() +
        std::max(0.0, params.clearing.extraOffset);
    const double quantization = std::pow(
        10.0, -std::clamp(params.clearing.precision, 0, 8));
    const double tolerance = std::max(0.02, 4.0 * quantization);
    const double sampleSpacing = std::clamp(
        requiredMargin > 0.0 ? requiredMargin / 3.0 : 0.25,
        0.05, 0.5);
    const std::size_t segmentsPerTurn = std::max<std::size_t>(
        24, static_cast<std::size_t>(
                std::ceil(circumference / sampleSpacing)));
    if (turns > kMaximumEntrySegments / segmentsPerTurn)
        return std::nullopt;
    const std::size_t segmentCount = turns * segmentsPerTurn;

    double tangentX = 0.0;
    double tangentY = 0.0;
    if (!firstLoopTangent(loop, tangentX, tangentY))
        return std::nullopt;

    const gp_Pnt2d& start = loop.points.front();
    const double leftX = -tangentY;
    const double leftY = tangentX;
    for (double side : {1.0, -1.0}) {
        const gp_Pnt2d center(
            start.X() + side * leftX * radius,
            start.Y() + side * leftY * radius);
        const double startAngle = std::atan2(
            start.Y() - center.Y(), start.X() - center.X());
        bool safe = true;
        for (std::size_t sample = 0;
             sample < segmentsPerTurn; ++sample) {
            const double angle =
                startAngle + side * 2.0 * std::acos(-1.0) *
                    static_cast<double>(sample) /
                    static_cast<double>(segmentsPerTurn);
            const gp_Pnt2d point(
                center.X() + radius * std::cos(angle),
                center.Y() + radius * std::sin(angle));
            if (!isPocketCutterCenterSafe(
                    point, region, requiredMargin, tolerance)) {
                safe = false;
                break;
            }
        }
        if (!safe)
            continue;

        PocketEntryPath result;
        result.usedMode = PocketEntryMode::HelicalRamp;
        result.startPoint = start;
        result.feedPoints.reserve(segmentCount + 2);
        result.feedPoints.push_back(
            pointFrom2dAtDepth(start, region.frame, safeOffset));
        for (std::size_t segment = 0;
             segment <= segmentCount; ++segment) {
            const double fraction =
                static_cast<double>(segment) /
                static_cast<double>(segmentCount);
            const double angle =
                startAngle + side * 2.0 * std::acos(-1.0) *
                    static_cast<double>(turns) * fraction;
            const gp_Pnt2d point(
                center.X() + radius * std::cos(angle),
                center.Y() + radius * std::sin(angle));
            result.feedPoints.push_back(
                pointFrom2dAtDepth(
                    point, region.frame,
                    rampStartOffset - verticalDrop * fraction));
        }
        result.feedPoints.back() = loop.world.front();
        result.arcs.reserve(turns);
        for (std::size_t turn = 0; turn < turns; ++turn) {
            const std::size_t firstPoint =
                1 + turn * segmentsPerTurn;
            const std::size_t lastPoint =
                firstPoint + segmentsPerTurn;
            const double startFraction =
                static_cast<double>(turn) /
                static_cast<double>(turns);

            PocketEntryArc semantic;
            semantic.firstFeedSegment = firstPoint;
            semantic.feedSegmentCount = segmentsPerTurn;
            semantic.arc.moveType = ToolpathMoveType::Plunge;
            semantic.arc.direction = side > 0.0
                ? ToolpathArcDirection::Counterclockwise
                : ToolpathArcDirection::Clockwise;
            semantic.arc.from = result.feedPoints[firstPoint];
            semantic.arc.to = result.feedPoints[lastPoint];
            semantic.arc.center = pointFrom2dAtDepth(
                center, region.frame,
                rampStartOffset - verticalDrop * startFraction);
            semantic.arc.axis = gp_Vec(region.frame.Direction());
            semantic.arc.sweepRadians = 2.0 * std::acos(-1.0);
            semantic.arc.feed = params.clearing.rampFeed > 0.0
                ? params.clearing.rampFeed
                : params.tool.vertFeed;
            result.arcs.push_back(std::move(semantic));
        }
        return result;
    }
    return std::nullopt;
}

} // namespace

PocketEntryPath planPocketEntry(const PocketRegion& region,
                                const ToolpathParams& params,
                                const ClearingLoop& loop,
                                double safeOffset,
                                double rampStartOffset,
                                const PocketSafetyArea* safetyArea)
{
    rampStartOffset = std::clamp(
        rampStartOffset, loop.depthOffset, safeOffset);
    PocketEntryPath vertical =
        verticalEntry(region, loop, safeOffset);
    if (params.clearing.entryMode ==
        PocketEntryMode::VerticalPlunge) {
        return vertical;
    }

    if (params.clearing.entryMode ==
        PocketEntryMode::HelicalRamp) {
        if (std::optional<PocketEntryPath> helix =
                helicalRampEntry(
                    region, params, loop, safeOffset,
                    rampStartOffset);
            helix && entryPathSafe(
                *helix, region, params, loop, safetyArea)) {
            return std::move(*helix);
        }
        if (std::optional<PocketEntryPath> shape =
                rampOnShapeEntry(
                    region, params, loop, safeOffset,
                    rampStartOffset);
            shape && entryPathSafe(
                *shape, region, params, loop, safetyArea)) {
            shape->fallback = true;
            shape->warning =
                "helical ramp entry did not fit the cutter-center safe "
                "area; using ramp-on-shape entry";
            return std::move(*shape);
        }
        vertical.fallback = true;
        vertical.warning =
            "helical and ramp-on-shape entry were unavailable; using "
            "vertical plunge";
        return vertical;
    }

    if (params.clearing.entryMode ==
        PocketEntryMode::LinearRamp) {
        if (std::optional<PocketEntryPath> linear =
                straightRampEntry(
                    region, params, loop, safeOffset,
                    rampStartOffset, safetyArea)) {
            return std::move(*linear);
        }
        if (std::optional<PocketEntryPath> shape =
                rampOnShapeEntry(
                    region, params, loop, safeOffset,
                    rampStartOffset);
            shape && entryPathSafe(
                *shape, region, params, loop, safetyArea)) {
            shape->fallback = true;
            shape->warning =
                "linear ramp entry did not fit a safe direction; using "
                "ramp-on-shape entry";
            return std::move(*shape);
        }
        vertical.fallback = true;
        vertical.warning =
            "linear and ramp-on-shape entry were unavailable; using "
            "vertical plunge";
        return vertical;
    }

    if (params.clearing.entryMode ==
        PocketEntryMode::ZigZagRamp) {
        if (std::optional<PocketEntryPath> zigzag =
                zigzagRampEntry(
                    region, params, loop, safeOffset,
                    rampStartOffset, safetyArea)) {
            return std::move(*zigzag);
        }
        if (std::optional<PocketEntryPath> linear =
                straightRampEntry(
                    region, params, loop, safeOffset,
                    rampStartOffset, safetyArea)) {
            linear->fallback = true;
            linear->warning =
                "zigzag ramp entry did not fit a safe direction; using "
                "linear ramp entry";
            return std::move(*linear);
        }
        vertical.fallback = true;
        vertical.warning =
            "zigzag and linear ramp entry were unavailable; using "
            "vertical plunge";
        return vertical;
    }

    if (params.clearing.entryMode ==
        PocketEntryMode::RampOnShape) {
        if (std::optional<PocketEntryPath> shape =
                rampOnShapeEntry(
                    region, params, loop, safeOffset,
                    rampStartOffset);
            shape && entryPathSafe(
                *shape, region, params, loop, safetyArea)) {
            return std::move(*shape);
        }
        vertical.fallback = true;
        vertical.warning =
            "ramp-on-shape entry was unavailable or crossed protected "
            "material; using vertical plunge";
    }
    return vertical;
}

} // namespace oap::toolpath::detail

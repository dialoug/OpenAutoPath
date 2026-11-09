#include "AdaptiveFreePathLinking.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

namespace oap::toolpath::detail {

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kTolerance = 1.0e-9;

double directedAdvance(double startAngle, double endAngle, int direction)
{
    double advance = std::fmod(
        static_cast<double>(direction) * (endAngle - startAngle), kTwoPi);
    if (advance < 0.0)
        advance += kTwoPi;
    if (advance >= kTwoPi - kTolerance)
        return 0.0;
    return advance;
}

std::vector<gp_Pnt2d> sampleOrientedArc(
    const gp_Pnt2d& start,
    const gp_Pnt2d& end,
    const gp_Pnt2d& center,
    double radius,
    int direction,
    double sampleSpacing)
{
    const double startAngle = std::atan2(
        start.Y() - center.Y(), start.X() - center.X());
    const double endAngle = std::atan2(
        end.Y() - center.Y(), end.X() - center.X());
    const double advance = directedAdvance(
        startAngle, endAngle, direction);
    const std::size_t count = std::max<std::size_t>({
        1u,
        static_cast<std::size_t>(std::ceil(
            radius * advance / std::max(sampleSpacing, 1.0e-6))),
        static_cast<std::size_t>(std::ceil(
            advance / (5.0 * kPi / 180.0))) });
    std::vector<gp_Pnt2d> points;
    points.reserve(count + 1u);
    for (std::size_t index = 0u; index <= count; ++index) {
        const double angle = startAngle +
            static_cast<double>(direction) * advance *
                static_cast<double>(index) / static_cast<double>(count);
        points.emplace_back(
            center.X() + radius * std::cos(angle),
            center.Y() + radius * std::sin(angle));
    }
    points.front() = start;
    points.back() = end;
    return points;
}

double polylineLength(const std::vector<gp_Pnt2d>& points)
{
    double length = 0.0;
    for (std::size_t index = 1u; index < points.size(); ++index)
        length += points[index - 1u].Distance(points[index]);
    return length;
}

std::vector<double> reducedRadiusOptions(double target, double minimum)
{
    std::vector<double> result;
    for (const double factor : {1.0, 0.70, 0.40}) {
        const double value = std::max(minimum, factor * target);
        if (std::none_of(result.begin(), result.end(), [&](double existing) {
                return std::abs(existing - value) <= kTolerance;
            })) {
            result.push_back(value);
        }
    }
    return result;
}

} // namespace

std::optional<std::vector<gp_Pnt2d>> adaptiveCscLinkCandidate(
    const gp_Pnt2d& start,
    const gp_Pnt2d& end,
    double startHeading,
    double endHeading,
    double exitRadius,
    double entryRadius,
    int exitDirection,
    int entryDirection,
    double sampleSpacing)
{
    if (exitRadius <= kTolerance || entryRadius <= kTolerance ||
        (exitDirection != -1 && exitDirection != 1) ||
        (entryDirection != -1 && entryDirection != 1)) {
        return std::nullopt;
    }

    const gp_Pnt2d startNormal(
        -std::sin(startHeading), std::cos(startHeading));
    const gp_Pnt2d endNormal(
        -std::sin(endHeading), std::cos(endHeading));
    const gp_Pnt2d exitCenter(
        start.X() + exitDirection * exitRadius * startNormal.X(),
        start.Y() + exitDirection * exitRadius * startNormal.Y());
    const gp_Pnt2d entryCenter(
        end.X() + entryDirection * entryRadius * endNormal.X(),
        end.Y() + entryDirection * entryRadius * endNormal.Y());
    const double centerDx = entryCenter.X() - exitCenter.X();
    const double centerDy = entryCenter.Y() - exitCenter.Y();
    const double centerDistance = std::hypot(centerDx, centerDy);
    const double normalOffset =
        entryDirection * entryRadius - exitDirection * exitRadius;
    if (centerDistance <= kTolerance ||
        std::abs(normalOffset) > centerDistance + kTolerance) {
        return std::nullopt;
    }

    const double centerAngle = std::atan2(centerDy, centerDx);
    const double tangentAngle = centerAngle - std::asin(std::clamp(
        normalOffset / centerDistance, -1.0, 1.0));
    const double tangentX = std::cos(tangentAngle);
    const double tangentY = std::sin(tangentAngle);
    const double tangentNormalX = -tangentY;
    const double tangentNormalY = tangentX;
    const gp_Pnt2d exitTangent(
        exitCenter.X() - exitDirection * exitRadius * tangentNormalX,
        exitCenter.Y() - exitDirection * exitRadius * tangentNormalY);
    const gp_Pnt2d entryTangent(
        entryCenter.X() - entryDirection * entryRadius * tangentNormalX,
        entryCenter.Y() - entryDirection * entryRadius * tangentNormalY);
    const double straightLength =
        (entryTangent.X() - exitTangent.X()) * tangentX +
        (entryTangent.Y() - exitTangent.Y()) * tangentY;
    if (straightLength < -kTolerance)
        return std::nullopt;

    const double exitStartAngle = std::atan2(
        start.Y() - exitCenter.Y(), start.X() - exitCenter.X());
    const double exitTangentAngle = std::atan2(
        exitTangent.Y() - exitCenter.Y(),
        exitTangent.X() - exitCenter.X());
    const double entryTangentAngle = std::atan2(
        entryTangent.Y() - entryCenter.Y(),
        entryTangent.X() - entryCenter.X());
    const double entryEndAngle = std::atan2(
        end.Y() - entryCenter.Y(), end.X() - entryCenter.X());
    const double exitAdvance = directedAdvance(
        exitStartAngle, exitTangentAngle, exitDirection);
    const double entryAdvance = directedAdvance(
        entryTangentAngle, entryEndAngle, entryDirection);
    constexpr double headingTolerance = 1.0 * kPi / 180.0;
    if ((exitAdvance <= kTolerance &&
         std::abs(std::atan2(
             std::sin(tangentAngle - startHeading),
             std::cos(tangentAngle - startHeading))) > headingTolerance) ||
        (entryAdvance <= kTolerance &&
         std::abs(std::atan2(
             std::sin(endHeading - tangentAngle),
             std::cos(endHeading - tangentAngle))) > headingTolerance)) {
        return std::nullopt;
    }

    std::vector<gp_Pnt2d> points = sampleOrientedArc(
        start, exitTangent, exitCenter, exitRadius,
        exitDirection, sampleSpacing);
    if (straightLength > kTolerance) {
        const std::size_t count = std::max<std::size_t>(
            1u, static_cast<std::size_t>(std::ceil(
                straightLength / std::max(sampleSpacing, 1.0e-6))));
        for (std::size_t index = 1u; index <= count; ++index) {
            const double t = static_cast<double>(index) /
                             static_cast<double>(count);
            points.emplace_back(
                exitTangent.X() + t * (entryTangent.X() - exitTangent.X()),
                exitTangent.Y() + t * (entryTangent.Y() - exitTangent.Y()));
        }
    } else if (points.back().Distance(entryTangent) > kTolerance) {
        points.push_back(entryTangent);
    }
    std::vector<gp_Pnt2d> entryArc = sampleOrientedArc(
        entryTangent, end, entryCenter, entryRadius,
        entryDirection, sampleSpacing);
    points.insert(points.end(), std::next(entryArc.begin()), entryArc.end());

    std::vector<gp_Pnt2d> cleaned;
    cleaned.reserve(points.size());
    for (const gp_Pnt2d& point : points) {
        if (cleaned.empty() || cleaned.back().Distance(point) > kTolerance)
            cleaned.push_back(point);
    }
    if (cleaned.size() < 2u)
        return std::nullopt;
    cleaned.front() = start;
    cleaned.back() = end;
    return cleaned;
}

std::optional<std::vector<gp_Pnt2d>> bestAdaptiveCscLink(
    const gp_Pnt2d& start,
    const gp_Pnt2d& end,
    double startHeading,
    double endHeading,
    double targetExitRadius,
    double targetEntryRadius,
    double minimumRadius,
    double sampleSpacing,
    const AdaptiveLinkPredicate& candidateIsValid)
{
    targetExitRadius = std::max(targetExitRadius, minimumRadius);
    targetEntryRadius = std::max(targetEntryRadius, minimumRadius);
    struct RadiusPair {
        double exitRadius = 0.0;
        double entryRadius = 0.0;
        double reduction = 0.0;
    };
    std::vector<RadiusPair> pairs;
    for (const double exitRadius :
         reducedRadiusOptions(targetExitRadius, minimumRadius)) {
        for (const double entryRadius :
             reducedRadiusOptions(targetEntryRadius, minimumRadius)) {
            pairs.push_back({
                exitRadius,
                entryRadius,
                (targetExitRadius - exitRadius) / targetExitRadius +
                    (targetEntryRadius - entryRadius) / targetEntryRadius});
        }
    }
    std::sort(pairs.begin(), pairs.end(), [](const RadiusPair& left,
                                              const RadiusPair& right) {
        if (left.reduction != right.reduction)
            return left.reduction < right.reduction;
        return left.exitRadius + left.entryRadius >
               right.exitRadius + right.entryRadius;
    });

    double currentReduction = std::numeric_limits<double>::quiet_NaN();
    std::optional<std::vector<gp_Pnt2d>> bestAtLevel;
    double bestLength = std::numeric_limits<double>::infinity();
    for (const RadiusPair& pair : pairs) {
        if (std::isfinite(currentReduction) &&
            pair.reduction > currentReduction + kTolerance &&
            bestAtLevel) {
            return bestAtLevel;
        }
        if (!std::isfinite(currentReduction) ||
            pair.reduction > currentReduction + kTolerance) {
            currentReduction = pair.reduction;
            bestAtLevel.reset();
            bestLength = std::numeric_limits<double>::infinity();
        }
        for (const int exitDirection : {-1, 1}) {
            for (const int entryDirection : {-1, 1}) {
                auto candidate = adaptiveCscLinkCandidate(
                    start, end, startHeading, endHeading,
                    pair.exitRadius, pair.entryRadius,
                    exitDirection, entryDirection, sampleSpacing);
                if (!candidate || !candidateIsValid(
                        *candidate, pair.exitRadius, pair.entryRadius)) {
                    continue;
                }
                const double length = polylineLength(*candidate);
                if (length < bestLength) {
                    bestLength = length;
                    bestAtLevel = std::move(candidate);
                }
            }
        }
    }
    return bestAtLevel;
}

} // namespace oap::toolpath::detail

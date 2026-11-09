// 根据孔特征和刀具尺寸构造仅供显示的 OCCT 几何，不参与实际刀路计算。
#include "app/view/HolePreviewGeometryBuilder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>

#include <gp_Vec.hxx>

namespace oap::appdetail {
namespace {

constexpr double kCoincidentTolerance = 1.0e-9;
constexpr double kLineGroupingTolerance = 1.0e-7;

bool isFinite(const gp_Pnt& point)
{
    return std::isfinite(point.X()) && std::isfinite(point.Y()) &&
           std::isfinite(point.Z());
}

int colorPriority(HolePreviewSegmentKind kind)
{
    switch (kind) {
        case HolePreviewSegmentKind::Cut: return 3;
        case HolePreviewSegmentKind::Vertical: return 2;
        case HolePreviewSegmentKind::Rapid: return 1;
    }
    return 0;
}

struct LineKey {
    std::array<std::int64_t, 6> values{};
    bool operator==(const LineKey&) const = default;
};

struct LineKeyHash {
    std::size_t operator()(const LineKey& key) const noexcept
    {
        std::size_t seed = 0;
        for (const std::int64_t value : key.values) {
            seed ^= std::hash<std::int64_t>{}(value) +
                    0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
        }
        return seed;
    }
};

using Interval = std::pair<double, double>;

// 同一直线上的线段被归到同一组:用“规范化方向 + 直线上最近点”两个量化键
// 标识一条直线(方向取符号规范化,使反向线段也归入同组)。组内按优先级
// 分桶保存线段投影区间,供重叠裁剪计算“被更高优先级线段覆盖”的部分。
struct LineGroup {
    std::array<double, 3> direction{};
    std::array<std::vector<Interval>, 4> intervalsByPriority;
    std::array<std::vector<Interval>, 4> coverageAbove;
};

std::int64_t quantizeLineValue(double value)
{
    return static_cast<std::int64_t>(
        std::llround(value / kLineGroupingTolerance));
}

bool makeLineKey(
    const HolePreviewSegment& segment,
    LineKey& key,
    std::array<double, 3>& canonicalDirection)
{
    if (!isFinite(segment.from) || !isFinite(segment.to))
        return false;
    gp_Vec direction(segment.from, segment.to);
    const double length = direction.Magnitude();
    if (!std::isfinite(length) || length < kCoincidentTolerance)
        return false;
    direction /= length;
    const std::array<double, 3> components{
        direction.X(), direction.Y(), direction.Z()};
    for (const double component : components) {
        if (std::abs(component) <= kLineGroupingTolerance)
            continue;
        if (component < 0.0)
            direction.Reverse();
        break;
    }
    const gp_Vec point(segment.from.X(), segment.from.Y(), segment.from.Z());
    const gp_Vec closest = point - direction * point.Dot(direction);
    key.values = {
        quantizeLineValue(direction.X()),
        quantizeLineValue(direction.Y()),
        quantizeLineValue(direction.Z()),
        quantizeLineValue(closest.X()),
        quantizeLineValue(closest.Y()),
        quantizeLineValue(closest.Z())};
    canonicalDirection = {direction.X(), direction.Y(), direction.Z()};
    return true;
}

double projectedCoordinate(
    const gp_Pnt& point,
    const std::array<double, 3>& direction)
{
    return point.X() * direction[0] +
           point.Y() * direction[1] +
           point.Z() * direction[2];
}

std::vector<Interval> mergeIntervals(
    std::vector<Interval> intervals,
    double tolerance)
{
    if (intervals.empty())
        return intervals;
    std::sort(intervals.begin(), intervals.end());
    std::vector<Interval> merged;
    merged.reserve(intervals.size());
    for (const Interval& interval : intervals) {
        if (merged.empty() ||
            interval.first > merged.back().second + tolerance) {
            merged.push_back(interval);
        } else {
            merged.back().second =
                std::max(merged.back().second, interval.second);
        }
    }
    return merged;
}

gp_Pnt interpolateSegment(const HolePreviewSegment& segment, double parameter)
{
    return gp_Pnt(
        segment.from.X() +
            (segment.to.X() - segment.from.X()) * parameter,
        segment.from.Y() +
            (segment.to.Y() - segment.from.Y()) * parameter,
        segment.from.Z() +
            (segment.to.Z() - segment.from.Z()) * parameter);
}

} // namespace

std::vector<std::size_t> holePreviewSampleIndices(
    std::size_t sourceCount,
    std::size_t maximumSegments)
{
    // Bresenham 式均匀采样:在保留首段与末段的前提下,把 sourceCount 个原始
    // 线段等距抽稀到最多 maximumSegments 条,索引序列单调不减。
    const std::size_t sampleCount = std::min(sourceCount, maximumSegments);
    std::vector<std::size_t> indices;
    indices.reserve(sampleCount);
    if (sampleCount == 0)
        return indices;
    if (sampleCount == 1) {
        indices.push_back(0);
        return indices;
    }

    const std::size_t sourceLast = sourceCount - 1;
    const std::size_t targetLast = sampleCount - 1;
    const std::size_t step = sourceLast / targetLast;
    const std::size_t remainder = sourceLast % targetLast;
    std::size_t source = 0;
    std::size_t error = 0;
    for (std::size_t target = 0; target < sampleCount; ++target) {
        indices.push_back(source);
        if (target == targetLast)
            break;
        source += step;
        if (remainder != 0 && error >= targetLast - remainder) {
            ++source;
            error -= targetLast - remainder;
        } else {
            error += remainder;
        }
    }
    return indices;
}

// 只裁剪显示线段，不改动 Operation、播放 Move 或最终 G-code。
std::vector<HolePreviewSegment> resolveHolePreviewColorOverlaps(
    const std::vector<HolePreviewSegment>& segments)
{
    // 思路:把共线线段先按优先级(Cut>Vertical>Rapid)分组,再对每条线段裁掉
    // 与“更高优先级线段”重叠的区间,输出补齐后的碎片。这样快速移动被切削段
    // 遮住时不再争抢像素,避免深度冲突导致的随机变色。
    std::unordered_map<LineKey, LineGroup, LineKeyHash> groups;
    groups.reserve(segments.size());
    std::vector<LineKey> keys(segments.size());
    std::vector<bool> validKeys(segments.size(), false);
    for (std::size_t index = 0; index < segments.size(); ++index) {
        std::array<double, 3> direction{};
        validKeys[index] = makeLineKey(segments[index], keys[index], direction);
        if (!validKeys[index])
            continue;
        auto [group, inserted] = groups.try_emplace(keys[index]);
        if (inserted)
            group->second.direction = direction;
        const double first = projectedCoordinate(
            segments[index].from, group->second.direction);
        const double second = projectedCoordinate(
            segments[index].to, group->second.direction);
        group->second.intervalsByPriority[
            static_cast<std::size_t>(colorPriority(segments[index].kind))]
            .push_back({std::min(first, second), std::max(first, second)});
    }

    for (auto& [key, group] : groups) {
        (void)key;
        std::vector<Interval> accumulated;
        for (int priority = 3; priority >= 0; --priority) {
            group.coverageAbove[static_cast<std::size_t>(priority)] =
                accumulated;
            const auto& current =
                group.intervalsByPriority[static_cast<std::size_t>(priority)];
            accumulated.insert(
                accumulated.end(), current.begin(), current.end());
            accumulated = mergeIntervals(
                std::move(accumulated), kLineGroupingTolerance);
        }
    }

    std::vector<HolePreviewSegment> resolved;
    resolved.reserve(segments.size());
    for (std::size_t index = 0; index < segments.size(); ++index) {
        const HolePreviewSegment& segment = segments[index];
        if (!validKeys[index]) {
            resolved.push_back(segment);
            continue;
        }
        const LineGroup& group = groups.at(keys[index]);
        const std::vector<Interval>& higherCoverage =
            group.coverageAbove[static_cast<std::size_t>(
                colorPriority(segment.kind))];
        if (higherCoverage.empty()) {
            resolved.push_back(segment);
            continue;
        }
        const double start = projectedCoordinate(segment.from, group.direction);
        const double finish = projectedCoordinate(segment.to, group.direction);
        const double delta = finish - start;
        if (std::abs(delta) < kCoincidentTolerance) {
            resolved.push_back(segment);
            continue;
        }
        const double segmentMinimum = std::min(start, finish);
        const double segmentMaximum = std::max(start, finish);
        std::vector<Interval> covered;
        for (const Interval& interval : higherCoverage) {
            const double overlapBegin =
                std::max(segmentMinimum, interval.first);
            const double overlapEnd =
                std::min(segmentMaximum, interval.second);
            if (overlapEnd - overlapBegin <= kLineGroupingTolerance)
                continue;
            double first = (overlapBegin - start) / delta;
            double second = (overlapEnd - start) / delta;
            if (first > second)
                std::swap(first, second);
            covered.push_back(
                {std::clamp(first, 0.0, 1.0),
                 std::clamp(second, 0.0, 1.0)});
        }
        if (covered.empty()) {
            resolved.push_back(segment);
            continue;
        }
        const std::vector<Interval> merged =
            mergeIntervals(std::move(covered), kCoincidentTolerance);
        double cursor = 0.0;
        for (const Interval& interval : merged) {
            if (interval.first - cursor > kCoincidentTolerance) {
                resolved.push_back(
                    {interpolateSegment(segment, cursor),
                     interpolateSegment(segment, interval.first),
                     segment.kind, segment.sourceIndex});
            }
            cursor = std::max(cursor, interval.second);
        }
        if (1.0 - cursor > kCoincidentTolerance) {
            resolved.push_back(
                {interpolateSegment(segment, cursor), segment.to,
                 segment.kind, segment.sourceIndex});
        }
    }
    return resolved;
}

} // namespace oap::appdetail

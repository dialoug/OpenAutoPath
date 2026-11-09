// 从区域外圈向内连续连接偏移环，生成尽量少抬刀的螺旋清料路径。
#include "toolpath/pocket/planning/detail/PocketPatterns.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace oap::toolpath::detail {

namespace {

double pathPerimeter(const PathD& path)
{
    double total = 0.0;
    const int count = static_cast<int>(path.size());
    for (int i = 0; i < count; ++i) {
        const PointD& a = path[i];
        const PointD& b = path[(i + 1) % count];
        total += std::hypot(b.x - a.x, b.y - a.y);
    }
    return total;
}

PointD lerpPoint(const PointD& a, const PointD& b, double t)
{
    return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}

PathD resampleClosed(const PathD& path, int sampleCount, double startDistance)
{
    // 各偏移环顶点数量通常不同，先按弧长重采样才能逐点平滑过渡。
    PathD output;
    const int count = static_cast<int>(path.size());
    if (count < 2 || sampleCount < 2)
        return output;

    std::vector<double> segments(count);
    double total = 0.0;
    for (int i = 0; i < count; ++i) {
        const PointD& a = path[i];
        const PointD& b = path[(i + 1) % count];
        segments[i] = std::hypot(b.x - a.x, b.y - a.y);
        total += segments[i];
    }
    if (total <= 1.0e-12)
        return output;

    output.reserve(sampleCount);
    const double sampleDistance = total / sampleCount;
    for (int sample = 0; sample < sampleCount; ++sample) {
        double distance = std::fmod(startDistance + sample * sampleDistance, total);
        if (distance < 0.0)
            distance += total;
        int segment = 0;
        while (segment < count && distance > segments[segment]) {
            distance -= segments[segment];
            ++segment;
        }
        if (segment >= count)
            segment = count - 1;
        const PointD& a = path[segment];
        const PointD& b = path[(segment + 1) % count];
        const double t = segments[segment] > 1.0e-12
            ? distance / segments[segment]
            : 0.0;
        output.emplace_back(a.x + (b.x - a.x) * t,
                            a.y + (b.y - a.y) * t);
    }
    return output;
}

double bestStartDistance(const PathD& path, int sampleCount, const PointD& reference)
{
    const PathD base = resampleClosed(path, sampleCount, 0.0);
    if (base.empty())
        return 0.0;

    int best = 0;
    double bestDistance = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(base.size()); ++i) {
        const double distance =
            std::hypot(base[i].x - reference.x, base[i].y - reference.y);
        if (distance < bestDistance) {
            bestDistance = distance;
            best = i;
        }
    }
    return (static_cast<double>(best) / sampleCount) * pathPerimeter(path);
}

ClearingLoop morphChainToLoop(const std::vector<PathD>& rings,
                              double offset,
                              bool reverseWinding,
                              double step)
{
    ClearingLoop loop;
    loop.ring = 0;
    loop.offset = offset;
    loop.closed = false;
    loop.isHole = false;
    if (rings.empty())
        return loop;

    double maximumPerimeter = 0.0;
    for (const PathD& ring : rings)
        maximumPerimeter = std::max(maximumPerimeter, pathPerimeter(ring));
    const double chord = std::max(0.25, step * 0.25);
    const int sampleCount = std::clamp(
        static_cast<int>(std::ceil(maximumPerimeter / chord)), 64, 720);

    // 将相邻环的接缝对齐，避免每收缩一圈都产生一次长距离横切。
    std::vector<PathD> resampled;
    resampled.reserve(rings.size());
    PointD seam;
    for (size_t i = 0; i < rings.size(); ++i) {
        PathD source = rings[i];
        if (reverseWinding)
            std::reverse(source.begin(), source.end());
        PathD ring = i == 0
            ? resampleClosed(source, sampleCount, 0.0)
            : resampleClosed(
                  source, sampleCount,
                  bestStartDistance(source, sampleCount, seam));
        if (static_cast<int>(ring.size()) != sampleCount)
            return {};
        seam = ring.front();
        resampled.push_back(std::move(ring));
    }

    // 沿周向逐步增加环间插值比例，把一组闭环展开成一条连续螺旋。
    std::vector<PointD> points;
    for (size_t i = 0; i + 1 < resampled.size(); ++i) {
        for (int j = 0; j < sampleCount; ++j) {
            const double t = static_cast<double>(j) / sampleCount;
            points.push_back(lerpPoint(resampled[i][j], resampled[i + 1][j], t));
        }
    }
    const PathD& last = resampled.back();
    for (int j = 0; j <= sampleCount; ++j)
        points.push_back(last[j % sampleCount]);

    loop.points.reserve(points.size());
    for (const PointD& point : points)
        loop.points.emplace_back(point.x, point.y);
    return loop.points.size() >= 2 ? loop : ClearingLoop{};
}

bool pointInReachable(const PathsD& reachable, const PointD& point)
{
    bool insideOuter = false;
    for (const PathD& path : reachable) {
        const auto result = Clipper2Lib::PointInPolygon(point, path);
        if (result == Clipper2Lib::PointInPolygonResult::IsOn)
            return true;
        if (result == Clipper2Lib::PointInPolygonResult::IsInside) {
            if (signedArea(path) >= 0.0)
                insideOuter = true;
            else
                return false;
        }
    }
    return insideOuter;
}

std::vector<ClearingLoop> generateIslandSpiral(const PathD& outerRing,
                                               const PathD& islandRing,
                                               double step,
                                               bool reverseWinding,
                                               const PathsD& reachable,
                                               int& ringCount)
{
    const double chord = std::max(0.25, step * 0.25);
    const int sampleCount = std::clamp(
        static_cast<int>(std::ceil(pathPerimeter(outerRing) / chord)), 64, 720);
    PathD outerSource = outerRing;
    PathD islandSource = islandRing;
    if (reverseWinding) {
        std::reverse(outerSource.begin(), outerSource.end());
        std::reverse(islandSource.begin(), islandSource.end());
    }
    const PathD outer =
        resampleClosed(outerSource, sampleCount, 0.0);
    if (static_cast<int>(outer.size()) != sampleCount)
        return {};
    const PathD island = resampleClosed(
        islandSource, sampleCount,
        bestStartDistance(islandSource, sampleCount, outer.front()));
    if (static_cast<int>(island.size()) != sampleCount)
        return {};

    // 单岛屿情形在外圈与岛屿圈之间做同参数插值；圈数由最大间隙决定。
    double maximumGap = 0.0;
    for (int i = 0; i < sampleCount; ++i) {
        maximumGap = std::max(
            maximumGap,
            std::hypot(outer[i].x - island[i].x, outer[i].y - island[i].y));
    }
    const int revolutions =
        std::max(1, static_cast<int>(std::ceil(maximumGap / step)));

    std::vector<PointD> points;
    points.reserve(static_cast<size_t>(revolutions + 1) * sampleCount + 1);
    for (int revolution = 0; revolution < revolutions; ++revolution) {
        for (int i = 0; i < sampleCount; ++i) {
            const double t =
                (revolution + static_cast<double>(i) / sampleCount) / revolutions;
            const PointD point = lerpPoint(outer[i], island[i], t);
            if (!pointInReachable(reachable, point))
                return {};
            points.push_back(point);
        }
    }
    for (int i = 0; i <= sampleCount; ++i) {
        const PointD point = island[i % sampleCount];
        if (!pointInReachable(reachable, point))
            return {};
        points.push_back(point);
    }
    ClearingLoop loop;
    loop.closed = false;
    loop.points.reserve(points.size());
    for (const PointD& point : points)
        loop.points.emplace_back(point.x, point.y);
    ringCount = revolutions;
    return {std::move(loop)};
}

void generateSpiralComponent(const PathD& firstRing,
                             double firstOffset,
                             double step,
                             int maximumRings,
                             bool reverseWinding,
                             int precision,
                             double arcTolerance,
                             std::vector<ClearingLoop>& output,
                             int& ringCount)
{
    // 逐层内偏移；若区域分裂，则结束当前螺旋并递归处理每个子区域。
    std::vector<PathD> rings{firstRing};
    const PathsD firstPaths{firstRing};
    for (int ring = 1; ring <= std::max(1, maximumRings); ++ring) {
        PathsD offset = Clipper2Lib::InflatePaths(
            firstPaths, -ring * step, Clipper2Lib::JoinType::Round,
            Clipper2Lib::EndType::Polygon, 2.0, precision, arcTolerance);
        offset = cleanPaths(offset, precision);
        if (offset.empty())
            break;
        offset = cleanPaths(
            Clipper2Lib::Union(offset, Clipper2Lib::FillRule::NonZero, precision),
            precision);
        if (offset.empty())
            break;

        PathsD components;
        for (PathD& path : offset)
            if (signedArea(path) > 0.0)
                components.push_back(std::move(path));
        if (components.empty())
            break;
        if (components.size() == 1) {
            rings.push_back(std::move(components.front()));
            continue;
        }

        ClearingLoop loop = morphChainToLoop(
            rings, firstOffset, reverseWinding, step);
        if (loop.points.size() >= 2) {
            output.push_back(std::move(loop));
            ringCount += static_cast<int>(rings.size());
        }
        for (const PathD& child : components) {
            generateSpiralComponent(
                child, firstOffset + ring * step, step, maximumRings - ring,
                reverseWinding, precision, arcTolerance, output, ringCount);
        }
        return;
    }

    ClearingLoop loop = morphChainToLoop(
        rings, firstOffset, reverseWinding, step);
    if (loop.points.size() >= 2) {
        output.push_back(std::move(loop));
        ringCount += static_cast<int>(rings.size());
    }
}

void applyInsideToOutsideOrder(std::vector<ClearingLoop>& loops)
{
    // 内向外模式：反转螺旋列表的径向顺序，并反转每条螺旋的点序
    // （否则衔接处会反向走刀）。周向方向已由 reverseWinding 决定，
    // 这里只改径向执行顺序。
    std::reverse(loops.begin(), loops.end());
    for (ClearingLoop& loop : loops)
        std::reverse(loop.points.begin(), loop.points.end());
}

} // namespace

std::vector<ClearingLoop> generateSpiralLoops(const PathsD& region,
                                              double firstOffset,
                                              double step,
                                              int maximumRings,
                                              bool reverse,
                                              PocketClearingDirection
                                                  expansionDirection,
                                              int precision,
                                              double arcTolerance,
                                              int& ringCount)
{
    // 螺旋入口：区分“单岛桥接螺旋”与“无岛连续螺旋”两条路径；
    // 输入必须正好一个外环且至多一个岛，否则返回空让调用方回退。
    const bool insideToOutside =
        expansionDirection ==
        PocketClearingDirection::InsideToOutside;
    // 反转整条螺旋会同时改变径向顺序和周向走刀方向。先按异或后的
    // winding 生成外向内路径，再单独反转径向执行顺序，确保 CutMode
    // 与 expansionDirection 互不耦合。
    const bool reverseWinding = reverse != insideToOutside;

    PathsD outers;
    PathsD holes;
    for (const PathD& path : region) {
        if (signedArea(path) > 0.0)
            outers.push_back(path);
        else if (signedArea(path) < 0.0)
            holes.push_back(path);
    }
    if (outers.size() != 1)
        return {};

    // 单外圈加单岛屿可生成桥接螺旋；多岛屿暂时回退给其他清料图案。
    if (holes.size() == 1) {
        PathsD outerRing = Clipper2Lib::InflatePaths(
            PathsD{outers.front()}, -firstOffset, Clipper2Lib::JoinType::Round,
            Clipper2Lib::EndType::Polygon, 2.0, precision, arcTolerance);
        outerRing = cleanPaths(outerRing, precision);

        PathD island = holes.front();
        std::reverse(island.begin(), island.end());
        PathsD islandRing = Clipper2Lib::InflatePaths(
            PathsD{island}, firstOffset, Clipper2Lib::JoinType::Round,
            Clipper2Lib::EndType::Polygon, 2.0, precision, arcTolerance);
        islandRing = cleanPaths(islandRing, precision);

        if (outerRing.size() == 1 && islandRing.size() == 1 &&
            signedArea(outerRing.front()) > 0.0 &&
            signedArea(islandRing.front()) > 0.0) {
            const PathsD reachable = cleanPaths(
                Clipper2Lib::Difference(outerRing, islandRing,
                                        Clipper2Lib::FillRule::NonZero, precision),
                precision);
            if (!reachable.empty()) {
                std::vector<ClearingLoop> output =
                    generateIslandSpiral(
                        outerRing.front(), islandRing.front(), step,
                        reverseWinding, reachable, ringCount);
                if (insideToOutside)
                    applyInsideToOutsideOrder(output);
                return output;
            }
        }
        return {};
    }
    if (!holes.empty())
        return {};

    // 无岛屿区域从首刀偏置后的每个连通分量独立向内收缩。
    PathsD first = Clipper2Lib::InflatePaths(
        region, -firstOffset, Clipper2Lib::JoinType::Round,
        Clipper2Lib::EndType::Polygon, 2.0, precision, arcTolerance);
    first = cleanPaths(first, precision);
    if (!first.empty()) {
        first = cleanPaths(
            Clipper2Lib::Union(first, Clipper2Lib::FillRule::NonZero, precision),
            precision);
    }
    if (first.empty())
        return {};

    std::vector<ClearingLoop> output;
    ringCount = 0;
    for (const PathD& component : first) {
        if (signedArea(component) > 0.0) {
            generateSpiralComponent(
                component, firstOffset, step, maximumRings,
                reverseWinding, precision, arcTolerance, output,
                ringCount);
        }
    }
    if (insideToOutside)
        applyInsideToOutsideOrder(output);
    return output;
}

} // namespace oap::toolpath::detail

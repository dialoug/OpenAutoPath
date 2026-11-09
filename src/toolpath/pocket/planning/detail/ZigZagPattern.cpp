// 用等距扫描线裁剪加工区域，排序并连接为往复式槽腔清料路径。
#include "toolpath/pocket/planning/detail/PocketPatterns.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "toolpath/core/ToolpathParallel.h"

namespace oap::toolpath::detail {

namespace {

bool connectionStaysInside(const PathsD& reachable,
                           const gp_Pnt2d& start,
                           const gp_Pnt2d& end)
{
    // 扫描段之间仅在连接线保持位于可达区域时连续走刀，岛屿会强制断开。
    constexpr int kSamples = 7;
    for (int sample = 1; sample < kSamples; ++sample) {
        const double t = static_cast<double>(sample) / kSamples;
        const PointD point(start.X() + (end.X() - start.X()) * t,
                           start.Y() + (end.Y() - start.Y()) * t);

        int outerInside = 0;
        int islandInside = 0;
        bool onEdge = false;
        for (const PathD& path : reachable) {
            const auto result = Clipper2Lib::PointInPolygon(point, path);
            if (result == Clipper2Lib::PointInPolygonResult::IsOn) {
                onEdge = true;
            } else if (result == Clipper2Lib::PointInPolygonResult::IsInside) {
                if (signedArea(path) >= 0.0)
                    ++outerInside;
                else
                    ++islandInside;
            }
        }
        if (islandInside > 0 || (outerInside == 0 && !onEdge))
            return false;
    }
    return true;
}

} // namespace

std::vector<ClearingLoop> generateZigZagLoops(const PathsD& region,
                                              double radius,
                                              double extraOffset,
                                              double step,
                                              double angleDegrees,
                                              bool perimeterPass,
                                              bool reverse,
                                              int precision,
                                              double arcTolerance,
                                              const ToolpathGenerationControl*
                                                  control,
                                              PocketPatternParallelAudit*
                                                  parallelAudit)
{
    std::vector<ClearingLoop> output;
    // 流程：内缩出刀心可达区 -> 按角度旋转坐标后以 step 间距扫描线裁剪
    // -> 每行内的线段按 u 排序、奇偶行反向连接成往复路径。
    // 先把原始边界按刀具半径内缩，后续所有扫描都在刀具中心可达区域内进行。
    const double inset = radius + std::max(0.0, extraOffset);
    PathsD reachable = Clipper2Lib::InflatePaths(
        region, -inset, Clipper2Lib::JoinType::Round,
        Clipper2Lib::EndType::Polygon, 2.0, precision, arcTolerance);
    reachable = cleanPaths(reachable, precision);
    if (!reachable.empty()) {
        reachable = cleanPaths(
            Clipper2Lib::Union(reachable, Clipper2Lib::FillRule::NonZero, precision),
            precision);
    }
    if (reachable.empty())
        return output;

    if (perimeterPass) {
        for (const PathInfo& info : sortedPathInfos(reachable)) {
            ClearingLoop loop;
            loop.ring = 0;
            loop.offset = inset;
            loop.closed = true;
            loop.area = info.area;
            loop.isHole = info.area < 0.0;
            PathD path = info.path;
            if (reverse)
                std::reverse(path.begin(), path.end());
            loop.points.reserve(path.size());
            for (const PointD& point : path)
                loop.points.emplace_back(point.x, point.y);
            output.push_back(std::move(loop));
        }
    }

    // 在旋转后的 (u, v) 坐标中，u 沿切削方向、v 控制相邻扫描线间距。
    const double theta = angleDegrees * 3.14159265358979323846 / 180.0;
    const double directionX = std::cos(theta);
    const double directionY = std::sin(theta);
    const double normalX = -directionY;
    const double normalY = directionX;

    const Clipper2Lib::RectD bounds = Clipper2Lib::GetBounds(reachable);
    const double corners[4][2] = {
        {bounds.left, bounds.top},
        {bounds.right, bounds.top},
        {bounds.left, bounds.bottom},
        {bounds.right, bounds.bottom},
    };
    double uMin = std::numeric_limits<double>::max();
    double uMax = std::numeric_limits<double>::lowest();
    double vMin = std::numeric_limits<double>::max();
    double vMax = std::numeric_limits<double>::lowest();
    for (const auto& corner : corners) {
        const double u = corner[0] * directionX + corner[1] * directionY;
        const double v = corner[0] * normalX + corner[1] * normalY;
        uMin = std::min(uMin, u);
        uMax = std::max(uMax, u);
        vMin = std::min(vMin, v);
        vMax = std::max(vMax, v);
    }
    uMin -= step;
    uMax += step;
    if (step <= 0.0)
        return output;

    struct Segment {
        gp_Pnt2d start;
        gp_Pnt2d end;
    };
    const int maximumLines =
        std::max(1, static_cast<int>(std::ceil((vMax - vMin) / step)) + 2);
    std::vector<std::vector<Segment>> scanLines(
        static_cast<std::size_t>(maximumLines));
    std::vector<unsigned char> validScanLines(
        static_cast<std::size_t>(maximumLines), 0u);

    // 每条无限长近似扫描线与可达区域求交，岛屿会自然把它切成多个线段。
    const auto generateScanLine = [&](std::size_t scanLineIndex) {
        const int lineIndex = static_cast<int>(scanLineIndex);
        const double v = vMin + lineIndex * step;
        PathD line{
            {uMin * directionX + v * normalX,
             uMin * directionY + v * normalY},
            {uMax * directionX + v * normalX,
             uMax * directionY + v * normalY}};

        Clipper2Lib::ClipperD clipper(precision);
        clipper.AddClip(reachable);
        clipper.AddOpenSubject(PathsD{line});
        PathsD closedOutput;
        PathsD openOutput;
        if (!clipper.Execute(Clipper2Lib::ClipType::Intersection,
                             Clipper2Lib::FillRule::NonZero,
                             closedOutput, openOutput))
            return;

        std::vector<Segment> segments;
        for (const PathD& piece : openOutput) {
            if (piece.size() < 2)
                continue;
            PointD first = piece.front();
            PointD last = piece.back();
            const double firstU = first.x * directionX + first.y * directionY;
            const double lastU = last.x * directionX + last.y * directionY;
            if (lastU < firstU)
                std::swap(first, last);
            segments.push_back({{first.x, first.y}, {last.x, last.y}});
        }
        std::sort(segments.begin(), segments.end(), [&](const Segment& a,
                                                        const Segment& b) {
            return a.start.X() * directionX + a.start.Y() * directionY <
                   b.start.X() * directionX + b.start.Y() * directionY;
        });
        scanLines[scanLineIndex] = std::move(segments);
        validScanLines[scanLineIndex] = 1u;
    };
    const std::size_t lineCount =
        static_cast<std::size_t>(maximumLines);
    const std::size_t maximumParallelism =
        toolpathParallelismFor(lineCount);
    ToolpathParallelLease lease(
        control, maximumParallelism - 1u,
        toolpathHardwareParallelism() - 1u);
    const std::size_t workerCount = std::min(
        maximumParallelism, lease.parallelism());
    toolpathParallelForWorkers(
        lineCount, workerCount, generateScanLine);
    if (workerCount > 1u && parallelAudit) {
        ++parallelAudit->batchCount;
        parallelAudit->taskCount += lineCount;
        parallelAudit->peakParallelism = std::max(
            parallelAudit->peakParallelism, workerCount);
    }
    if (std::find(validScanLines.begin(), validScanLines.end(), 0u) !=
        validScanLines.end()) {
        std::vector<std::vector<Segment>> compact;
        compact.reserve(scanLines.size());
        for (std::size_t index = 0u; index < scanLines.size(); ++index) {
            if (validScanLines[index] != 0u)
                compact.push_back(std::move(scanLines[index]));
        }
        scanLines = std::move(compact);
    }

    ClearingLoop current;
    bool hasCurrent = false;
    gp_Pnt2d lastPoint;
    auto flush = [&]() {
        if (hasCurrent && current.points.size() >= 2)
            output.push_back(std::move(current));
        current = {};
        hasCurrent = false;
    };

    // 奇偶行反向形成往复路径；无法安全横连时结束当前折线并重新进刀。
    for (size_t lineIndex = 0; lineIndex < scanLines.size(); ++lineIndex) {
        const bool leftToRight = lineIndex % 2 == 0;
        std::vector<Segment>& segments = scanLines[lineIndex];
        if (!leftToRight)
            std::reverse(segments.begin(), segments.end());

        for (Segment& segment : segments) {
            const gp_Pnt2d entry = leftToRight ? segment.start : segment.end;
            const gp_Pnt2d exit = leftToRight ? segment.end : segment.start;
            if (hasCurrent && connectionStaysInside(reachable, lastPoint, entry)) {
                current.points.push_back(entry);
            } else if (hasCurrent) {
                flush();
            }
            if (!hasCurrent) {
                current.ring = 1;
                current.offset = inset;
                current.closed = false;
                current.points.push_back(entry);
                hasCurrent = true;
            }
            current.points.push_back(exit);
            lastPoint = exit;
        }
    }
    flush();

    if (reverse) {
        for (ClearingLoop& loop : output)
            if (!loop.closed)
                std::reverse(loop.points.begin(), loop.points.end());
    }
    return output;
}

} // namespace oap::toolpath::detail

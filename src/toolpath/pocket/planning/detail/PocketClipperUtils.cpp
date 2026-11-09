// Clipper2 路径的缩放、布尔运算、方向归一化和面积筛选等公共工具。
#include "toolpath/pocket/planning/detail/PocketClipperUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace oap::toolpath::detail {

namespace {

double gridSize(int precision)
{
    return std::pow(10.0, -precision);
}

bool nearSame(const PointD& a, const PointD& b, double tolerance)
{
    return std::hypot(a.x - b.x, a.y - b.y) <= tolerance;
}

PathD loopToPath(const Loop2d& loop, bool wantPositiveArea)
{
    PathD path;
    if (loop.points.size() < 3)
        return path;

    path.reserve(loop.points.size());
    for (const gp_Pnt2d& p : loop.points)
        path.emplace_back(p.X(), p.Y());

    const double area = Clipper2Lib::Area(path);
    if ((wantPositiveArea && area < 0.0) ||
        (!wantPositiveArea && area > 0.0))
        std::reverse(path.begin(), path.end());
    return path;
}

std::pair<double, double> minPoint(const PathD& path)
{
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    for (const PointD& p : path) {
        minX = std::min(minX, p.x);
        minY = std::min(minY, p.y);
    }
    return {minX, minY};
}

PathsD localProtectedMaterial(
    const PathsD& protectedMaterial,
    const PathsD& reachable,
    double requiredMargin,
    int precision,
    double arcTolerance)
{
    if (protectedMaterial.empty() ||
        reachable.empty()) {
        return {};
    }

    const Clipper2Lib::RectD reachableBounds =
        Clipper2Lib::GetBounds(reachable);
    if (!reachableBounds.IsValid() ||
        reachableBounds.IsEmpty()) {
        return {};
    }

    const double grid =
        std::pow(
            10.0,
            -std::clamp(precision, 0, 8));
    const double defaultArcTolerance =
        std::max(0.0, requiredMargin) *
        0.002;
    const double effectiveArcTolerance =
        std::isfinite(arcTolerance) &&
                arcTolerance > 0.0
            ? arcTolerance
            : defaultArcTolerance;
    // 裁剪产生的人工边界必须在刀具半径偏置后仍位于可达区域之外。
    // 额外 guard 同时覆盖 Clipper 网格和圆弧弦高，避免局部化改变布尔结果。
    const double guard = std::max({
        1.0e-6,
        grid * 4.0,
        effectiveArcTolerance * 2.0});
    const double padding =
        std::max(0.0, requiredMargin) + guard;
    const Clipper2Lib::RectD clipBounds(
        reachableBounds.left - padding,
        reachableBounds.top - padding,
        reachableBounds.right + padding,
        reachableBounds.bottom + padding);

    PathsD local = Clipper2Lib::Intersect(
        protectedMaterial,
        PathsD{clipBounds.AsPath()},
        Clipper2Lib::FillRule::NonZero,
        precision);
    if (local.empty()) {
        // 空结果可能表示局部确实没有材料，也可能是异常嵌套轮廓在矩形裁剪
        // 中退化。回退完整材料保持原有保守语义。
        return protectedMaterial;
    }
    local = cleanPaths(local, precision);
    if (local.empty())
        return protectedMaterial;
    return cleanPaths(
        Clipper2Lib::Union(
            local,
            Clipper2Lib::FillRule::NonZero,
            precision),
        precision);
}

} // namespace

double signedArea(const PathD& path)
{
    return Clipper2Lib::Area(path);
}

PathsD cleanPaths(const PathsD& paths, int precision)
{
    const double epsilon = std::max(1.0e-9, gridSize(precision) * 0.5);
    const double minArea = std::max(1.0e-9, epsilon * epsilon * 4.0);

    PathsD simplified = Clipper2Lib::SimplifyPaths(paths, epsilon, true);
    PathsD cleaned;
    cleaned.reserve(simplified.size());
    for (PathD path : simplified) {
        while (path.size() > 1 && nearSame(path.front(), path.back(), epsilon))
            path.pop_back();
        if (path.size() < 3 || std::abs(signedArea(path)) <= minArea)
            continue;
        cleaned.push_back(std::move(path));
    }
    return cleaned;
}

PathsD offsetArea(const PathsD& paths,
                  double delta,
                  int precision,
                  double arcTolerance)
{
    if (paths.empty() || !std::isfinite(delta))
        return {};
    if (std::abs(delta) <= 1.0e-12) {
        return cleanPaths(
            Clipper2Lib::Union(
                paths, Clipper2Lib::FillRule::NonZero, precision),
            precision);
    }
    PathsD offset = Clipper2Lib::InflatePaths(
        paths, delta,
        Clipper2Lib::JoinType::Round,
        Clipper2Lib::EndType::Polygon,
        2.0, precision, arcTolerance);
    offset = cleanPaths(offset, precision);
    return offset.empty()
        ? offset
        : cleanPaths(
              Clipper2Lib::Union(
                  offset, Clipper2Lib::FillRule::NonZero, precision),
              precision);
}

PathsD normalizedClearArea(const PocketRegion& region, int precision)
{
    // 清腔可加工区域 = 外边界（正面积）减所有岛环（负面积）；
    // 结果统一为正面积的 NonZero 区域，供图案生成器消费。
    PathsD outer{loopToPath(region.boundary, true)};
    outer = cleanPaths(outer, precision);
    if (outer.empty())
        return {};

    PathsD islands;
    islands.reserve(region.islands.size());
    for (const Loop2d& island : region.islands) {
        PathD path = loopToPath(island, true);
        if (!path.empty())
            islands.push_back(std::move(path));
    }
    islands = cleanPaths(islands, precision);

    PathsD area = islands.empty()
        ? Clipper2Lib::Union(outer, Clipper2Lib::FillRule::NonZero, precision)
        : Clipper2Lib::Difference(outer, islands,
                                  Clipper2Lib::FillRule::NonZero, precision);
    area = cleanPaths(area, precision);
    return area.empty()
        ? area
        : cleanPaths(Clipper2Lib::Union(
                         area, Clipper2Lib::FillRule::NonZero, precision),
                     precision);
}

PathsD normalizedClearArea(const std::vector<Loop2d>& contours, int precision)
{
    // 逐层切片版本：按有符号面积把环分为外边界（正）与岛（负），
    // 做“外减内”后统一为正向区域。
    PathsD outer;
    PathsD islands;
    for (const Loop2d& contour : contours) {
        PathD path = loopToPath(contour, true);
        if (path.empty())
            continue;
        if (contour.signedArea >= 0.0)
            outer.push_back(std::move(path));
        else
            islands.push_back(std::move(path));
    }

    outer = cleanPaths(outer, precision);
    islands = cleanPaths(islands, precision);
    if (outer.empty())
        return {};

    PathsD area = islands.empty()
        ? Clipper2Lib::Union(outer, Clipper2Lib::FillRule::NonZero, precision)
        : Clipper2Lib::Difference(outer, islands,
                                  Clipper2Lib::FillRule::NonZero, precision);
    area = cleanPaths(area, precision);
    return area.empty()
        ? area
        : cleanPaths(Clipper2Lib::Union(
                         area, Clipper2Lib::FillRule::NonZero, precision),
                     precision);
}

PathsD normalizedNestedArea(const std::vector<Loop2d>& contours, int precision)
{
    // 与 normalizedClearArea 不同：这里保留所有嵌套层级的正/负轮廓
    // （实体外环、腔孔、腔内凸台），只做绕向归一化与并集清理，
    // 供材料截面、余料阴影等需要区分材料/空隙的语义使用。
    PathsD nested;
    nested.reserve(contours.size());
    for (const Loop2d& contour : contours) {
        PathD path = loopToPath(
            contour, contour.signedArea >= 0.0);
        if (!path.empty())
            nested.push_back(std::move(path));
    }

    nested = cleanPaths(nested, precision);
    if (nested.empty())
        return {};
    return cleanPaths(
        Clipper2Lib::Union(
            nested, Clipper2Lib::FillRule::NonZero, precision),
        precision);
}

std::vector<Loop2d> nestedAreaContours(const PathsD& paths)
{
    std::vector<Loop2d> contours;
    contours.reserve(paths.size());
    for (const PathD& path : paths) {
        if (path.size() < 3)
            continue;
        Loop2d contour;
        contour.points.reserve(path.size());
        for (const PointD& point : path)
            contour.points.emplace_back(point.x, point.y);
        contour.closed = true;
        contour.signedArea = signedArea(path);
        contours.push_back(std::move(contour));
    }
    return contours;
}

PathsD cutterCenterReachableArea(
    const PathsD& clearArea,
    const PathsD& protectedMaterial,
    double requiredMargin,
    int precision,
    double arcTolerance)
{
    // 刀具中心可达区：清腔区整体内缩 requiredMargin，再减去“材料外扩
    // requiredMargin”的禁入区。先做局部化（localProtectedMaterial）减小
    // 布尔运算规模，guard 保证裁剪边界不会影响偏置结果。
    if (clearArea.empty() || protectedMaterial.empty() ||
        !std::isfinite(requiredMargin) ||
        requiredMargin < 0.0) {
        return {};
    }

    const auto normalizedOffset =
        [&](const PathsD& paths, double delta) {
            PathsD offset = Clipper2Lib::InflatePaths(
                paths, delta,
                Clipper2Lib::JoinType::Round,
                Clipper2Lib::EndType::Polygon,
                2.0, precision, arcTolerance);
            offset = cleanPaths(offset, precision);
            return offset.empty()
                ? offset
                : cleanPaths(
                      Clipper2Lib::Union(
                          offset,
                          Clipper2Lib::FillRule::NonZero,
                          precision),
                      precision);
        };

    PathsD reachable = normalizedOffset(
        clearArea, -requiredMargin);
    if (reachable.empty())
        return {};

    const PathsD materialNearReachable =
        localProtectedMaterial(
            protectedMaterial, reachable,
            requiredMargin, precision,
            arcTolerance);
    const PathsD forbidden = normalizedOffset(
        materialNearReachable,
        requiredMargin);
    if (forbidden.empty())
        return {};

    reachable = cleanPaths(
        Clipper2Lib::Difference(
            reachable, forbidden,
            Clipper2Lib::FillRule::NonZero,
            precision),
        precision);
    return reachable.empty()
        ? reachable
        : cleanPaths(
              Clipper2Lib::Union(
                  reachable,
                  Clipper2Lib::FillRule::NonZero,
                  precision),
              precision);
}

PathsD restMachiningCenterArea(
    const PathsD& clearArea,
    const PathsD& protectedMaterial,
    double currentRadius,
    double previousRadius,
    double extraOffset,
    int precision,
    double arcTolerance)
{
    if (clearArea.empty() || !std::isfinite(currentRadius) ||
        !std::isfinite(previousRadius) || currentRadius <= 0.0 ||
        previousRadius <= currentRadius ||
        !std::isfinite(extraOffset) || extraOffset < 0.0) {
        return {};
    }

    const auto normalizedOffset =
        [&](const PathsD& paths, double delta) {
            if (paths.empty())
                return PathsD{};
            PathsD offset = Clipper2Lib::InflatePaths(
                paths, delta,
                Clipper2Lib::JoinType::Round,
                Clipper2Lib::EndType::Polygon,
                2.0, precision, arcTolerance);
            offset = cleanPaths(offset, precision);
            return offset.empty()
                ? offset
                : cleanPaths(
                      Clipper2Lib::Union(
                          offset,
                          Clipper2Lib::FillRule::NonZero,
                          precision),
                      precision);
        };
    const auto centerArea =
        [&](double radius) {
            const double margin = radius + extraOffset;
            return protectedMaterial.empty()
                ? normalizedOffset(clearArea, -margin)
                : cutterCenterReachableArea(
                      clearArea, protectedMaterial,
                      margin, precision, arcTolerance);
        };
    const auto sweptArea =
        [&](const PathsD& centers, double radius) {
            const PathsD expanded = normalizedOffset(centers, radius);
            if (expanded.empty())
                return PathsD{};
            return cleanPaths(
                Clipper2Lib::Intersect(
                    expanded, clearArea,
                    Clipper2Lib::FillRule::NonZero,
                    precision),
                precision);
        };

    const PathsD currentCenters = centerArea(currentRadius);
    if (currentCenters.empty())
        return {};
    const PathsD previousCenters = centerArea(previousRadius);
    if (previousCenters.empty())
        return currentCenters;

    // 余料 = 当前刀扫过的实体区域 减去 上一刀已扫过的实体区域；
    // 再把它按当前刀半径内缩回刀心区域，并与当前刀可达刀心取交，
    // 得到“当前刀相对于上一刀新增需要清除”的刀心路径。
    const PathsD currentSweep =
        sweptArea(currentCenters, currentRadius);
    const PathsD previousSweep =
        sweptArea(previousCenters, previousRadius);
    if (currentSweep.empty())
        return {};
    if (previousSweep.empty())
        return currentCenters;

    const PathsD incrementalMaterial = cleanPaths(
        Clipper2Lib::Difference(
            currentSweep, previousSweep,
            Clipper2Lib::FillRule::NonZero,
            precision),
        precision);
    if (incrementalMaterial.empty())
        return {};

    const PathsD relevantCenters = normalizedOffset(
        incrementalMaterial, currentRadius);
    if (relevantCenters.empty())
        return {};
    PathsD result = cleanPaths(
        Clipper2Lib::Intersect(
            currentCenters, relevantCenters,
            Clipper2Lib::FillRule::NonZero,
            precision),
        precision);
    if (result.empty())
        return result;

    // Rest machining combines several independently rounded offsets and
    // booleans. Leaving its first (delta-zero) ring exactly on that derived
    // boundary can put a reconstructed validation sample just inside the
    // protected-material envelope. Inset only by the same numerical tolerance
    // used by PocketSafetyValidator; sub-tolerance slivers are not reliable
    // machining regions and must not invalidate an otherwise useful large-tool
    // transition.
    const double quantization = std::pow(
        10.0, -std::clamp(precision, 0, 8));
    const double validationInset =
        std::max(0.02, 4.0 * quantization);
    result = normalizedOffset(result, -validationInset);
    if (result.empty())
        return result;
    return cleanPaths(
        Clipper2Lib::Intersect(
            currentCenters, result,
            Clipper2Lib::FillRule::NonZero,
            precision),
        precision);
}

PathsD extendThroughVirtualOpenBoundaries(
    const PathsD& area,
    const std::vector<Loop2d>& contours,
    double extension,
    int precision)
{
    if (area.empty() || !std::isfinite(extension) || extension <= 0.0)
        return area;

    PathsD outletExtensions;
    for (const Loop2d& contour : contours) {
        // 只扩展区域外边界。内岛上的开放线框属于异常几何，不能借此
        // 擅自删除岛屿或扩大可加工区域。
        if (!contour.virtualOpenBoundary ||
            contour.signedArea < 0.0 ||
            contour.points.size() < 3) {
            continue;
        }

        // 开放线框在二维布尔运算中由“末点 -> 首点”隐式封口。
        // 外环已归一化为逆时针，因此该有向封口的右法向指向实体外部。
        const gp_Pnt2d& first = contour.points.front();
        const gp_Pnt2d& last = contour.points.back();
        const double dx = first.X() - last.X();
        const double dy = first.Y() - last.Y();
        const double length = std::hypot(dx, dy);
        if (!std::isfinite(length) ||
            length <= std::max(1.0e-9, gridSize(precision))) {
            continue;
        }

        const double outwardX = dy / length;
        const double outwardY = -dx / length;
        PathD strip{
            PointD(last.X(), last.Y()),
            PointD(first.X(), first.Y()),
            PointD(first.X() + outwardX * extension,
                   first.Y() + outwardY * extension),
            PointD(last.X() + outwardX * extension,
                   last.Y() + outwardY * extension)};
        if (Clipper2Lib::Area(strip) < 0.0)
            std::reverse(strip.begin(), strip.end());
        outletExtensions.push_back(std::move(strip));
    }

    outletExtensions = cleanPaths(outletExtensions, precision);
    if (outletExtensions.empty())
        return area;

    PathsD combined = area;
    combined.insert(combined.end(),
                    outletExtensions.begin(),
                    outletExtensions.end());
    return cleanPaths(
        Clipper2Lib::Union(
            combined, Clipper2Lib::FillRule::NonZero, precision),
        precision);
}

std::vector<PathInfo> sortedPathInfos(const PathsD& paths)
{
    std::vector<PathInfo> infos;
    infos.reserve(paths.size());
    for (const PathD& path : paths) {
        const double area = signedArea(path);
        auto [minX, minY] = minPoint(path);
        infos.push_back({path, area, std::abs(area), 0, minX, minY});
    }

    for (size_t i = 0; i < infos.size(); ++i) {
        if (infos[i].path.empty())
            continue;
        const PointD probe = infos[i].path.front();
        for (size_t j = 0; j < infos.size(); ++j) {
            if (i == j || infos[j].absArea <= infos[i].absArea)
                continue;
            if (Clipper2Lib::PointInPolygon(probe, infos[j].path) ==
                Clipper2Lib::PointInPolygonResult::IsInside)
                ++infos[i].nesting;
        }
    }

    std::sort(infos.begin(), infos.end(), [](const PathInfo& a, const PathInfo& b) {
        if (a.nesting != b.nesting)
            return a.nesting < b.nesting;
        if ((a.area < 0.0) != (b.area < 0.0))
            return a.area > b.area;
        if (std::abs(a.absArea - b.absArea) > 1.0e-9)
            return a.absArea > b.absArea;
        if (std::abs(a.minY - b.minY) > 1.0e-9)
            return a.minY < b.minY;
        return a.minX < b.minX;
    });
    return infos;
}

} // namespace oap::toolpath::detail

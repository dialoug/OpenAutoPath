// 槽腔刀心二维安全判定实现，不依赖具体入刀或验证工作流。
#include "toolpath/pocket/planning/detail/PocketSafetyGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "toolpath/core/ToolpathParallel.h"
#include <gp_Vec.hxx>

namespace oap::toolpath::detail {

gp_Pnt2d projectToPocketPlane(const gp_Pnt& point, const gp_Ax3& frame)
{
    const gp_Vec offset(frame.Location(), point);
    return {offset.Dot(gp_Vec(frame.XDirection())),
            offset.Dot(gp_Vec(frame.YDirection()))};
}

double distanceToPocketLoop(const gp_Pnt2d& point, const Loop2d& loop)
{
    if (loop.points.size() < 2)
        return std::numeric_limits<double>::infinity();

    double minimum = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < loop.points.size(); ++index) {
        const gp_Pnt2d& start = loop.points[index];
        const gp_Pnt2d& end =
            loop.points[(index + 1) % loop.points.size()];
        const double dx = end.X() - start.X();
        const double dy = end.Y() - start.Y();
        const double lengthSquared = dx * dx + dy * dy;
        if (lengthSquared <= 1.0e-24) {
            minimum = std::min(minimum, point.Distance(start));
            continue;
        }
        const double parameter = std::clamp(
            ((point.X() - start.X()) * dx +
             (point.Y() - start.Y()) * dy) / lengthSquared,
            0.0, 1.0);
        minimum = std::min(
            minimum,
            point.Distance(gp_Pnt2d(
                start.X() + parameter * dx,
                start.Y() + parameter * dy)));
    }
    return minimum;
}

bool pointInPocketLoop(const gp_Pnt2d& point,
                       const Loop2d& loop,
                       double tolerance)
{
    if (loop.points.size() < 3)
        return false;
    if (distanceToPocketLoop(point, loop) <= tolerance)
        return true;

    bool inside = false;
    for (std::size_t index = 0, previous = loop.points.size() - 1;
         index < loop.points.size(); previous = index++) {
        const gp_Pnt2d& currentPoint = loop.points[index];
        const gp_Pnt2d& previousPoint = loop.points[previous];
        const bool crosses =
            ((currentPoint.Y() > point.Y()) !=
             (previousPoint.Y() > point.Y())) &&
            (point.X() <
             (previousPoint.X() - currentPoint.X()) *
                     (point.Y() - currentPoint.Y()) /
                     (previousPoint.Y() - currentPoint.Y()) +
                 currentPoint.X());
        if (crosses)
            inside = !inside;
    }
    return inside;
}

bool isPocketCutterCenterSafe(const gp_Pnt2d& point,
                              const PocketRegion& region,
                              double requiredMargin,
                              double tolerance)
{
    if (!pointInPocketLoop(point, region.boundary, tolerance))
        return false;
    if (distanceToPocketLoop(point, region.boundary) + tolerance <
        requiredMargin) {
        return false;
    }
    for (const Loop2d& island : region.islands) {
        if (pointInPocketLoop(point, island, tolerance))
            return false;
        if (distanceToPocketLoop(point, island) + tolerance <
            requiredMargin) {
            return false;
        }
    }
    return true;
}

bool PocketSafetyArea::Bounds2d::contains(
    const gp_Pnt2d& point) const noexcept
{
    return valid &&
           point.X() >= minX && point.X() <= maxX &&
           point.Y() >= minY && point.Y() <= maxY;
}

PocketSafetyArea::PreparedArea PocketSafetyArea::prepare(PathsD paths)
{
    PreparedArea result;
    result.paths = std::move(paths);
    result.bounds.reserve(result.paths.size());
    result.windingDirections.reserve(result.paths.size());
    for (const PathD& path : result.paths) {
        Bounds2d bounds;
        if (!path.empty()) {
            bounds.minX = bounds.maxX = path.front().x;
            bounds.minY = bounds.maxY = path.front().y;
            bounds.valid = true;
        }
        for (const PointD& point : path) {
            bounds.minX = std::min(bounds.minX, point.x);
            bounds.minY = std::min(bounds.minY, point.y);
            bounds.maxX = std::max(bounds.maxX, point.x);
            bounds.maxY = std::max(bounds.maxY, point.y);
        }
        result.bounds.push_back(bounds);
        result.windingDirections.push_back(
            signedArea(path) >= 0.0 ? 1 : -1);
    }
    return result;
}

bool PocketSafetyArea::contains(const PreparedArea& area,
                                const gp_Pnt2d& point,
                                bool includeBoundary)
{
    // 用绕数（winding number）判定点是否位于 NonZero 区域内：按路径
    // 面积符号累计包含次数，非零即“在内”；先做包围盒过滤以加速热路径。
    int winding = 0;
    const PointD clipperPoint(point.X(), point.Y());
    for (std::size_t index = 0; index < area.paths.size(); ++index) {
        if (index >= area.bounds.size() ||
            !area.bounds[index].contains(point)) {
            continue;
        }
        const auto containment = Clipper2Lib::PointInPolygon(
            clipperPoint, area.paths[index]);
        if (containment ==
            Clipper2Lib::PointInPolygonResult::IsOn) {
            return includeBoundary;
        }
        if (containment !=
            Clipper2Lib::PointInPolygonResult::IsInside) {
            continue;
        }
        winding += index < area.windingDirections.size()
            ? area.windingDirections[index]
            : 0;
    }
    return winding != 0;
}

double PocketSafetyArea::distanceToAreaBoundary(
    const PreparedArea& area,
    const gp_Pnt2d& point)
{
    double minimum = std::numeric_limits<double>::infinity();
    for (const PathD& path : area.paths) {
        if (path.size() < 2)
            continue;
        for (std::size_t index = 0; index < path.size(); ++index) {
            const PointD& start = path[index];
            const PointD& end = path[(index + 1) % path.size()];
            const double dx = end.x - start.x;
            const double dy = end.y - start.y;
            const double lengthSquared = dx * dx + dy * dy;
            const double parameter = lengthSquared > 1.0e-24
                ? std::clamp(
                      ((point.X() - start.x) * dx +
                       (point.Y() - start.y) * dy) / lengthSquared,
                      0.0, 1.0)
                : 0.0;
            minimum = std::min(
                minimum,
                std::hypot(
                    point.X() - (start.x + parameter * dx),
                    point.Y() - (start.y + parameter * dy)));
        }
    }
    return minimum;
}

PocketSafetyArea::PocketSafetyArea(
    const PocketRegion& region,
    const ToolpathParams& params,
    const std::vector<ProtectedMaterialLayer>& protectedLayers)
{
    // 构造时一次性预偏置：根边界内缩为刀心安全区，各深度层的材料轮廓
    // 外扩为禁入区。此后 clear() 只做包围盒过滤 + 绕数判定，不再逐点
    // 重复扫描源轮廓，满足最终安全校验的高频采样需求。
    const int precision =
        std::clamp(params.clearing.precision, 0, 8);
    const double quantization = std::pow(10.0, -precision);
    const double tolerance = std::max(0.02, 4.0 * quantization);
    const double requiredMargin =
        params.tool.radius() +
        std::max(0.0, params.clearing.extraOffset);
    const double checkedMargin =
        std::max(0.0, requiredMargin - tolerance);
    m_checkedMargin = checkedMargin;

    const PathsD clearArea = normalizedClearArea(region, precision);
    m_rootSafeArea = prepare(offsetArea(
        clearArea, -checkedMargin, precision,
        params.clearing.arcTolerance));

    std::vector<PreparedProtectedArea> preparedProtected(
        protectedLayers.size());
    std::vector<std::uint8_t> hasPrepared(
        protectedLayers.size(), 0u);
    const auto prepareProtectedLayer = [&](std::size_t index) {
        const ProtectedMaterialLayer& layer =
            protectedLayers[index];
        PathsD material = normalizedNestedArea(
            layer.contours, precision);
        if (material.empty())
            return;
        PreparedArea preparedMaterial = prepare(material);
        PathsD forbidden = offsetArea(
            material, checkedMargin, precision,
            params.clearing.arcTolerance);
        if (forbidden.empty())
            forbidden = material;
        preparedProtected[index].material =
            std::move(preparedMaterial);
        preparedProtected[index].forbidden =
            prepare(std::move(forbidden));
        hasPrepared[index] = 1u;
    };
    toolpathParallelForBudgeted(protectedLayers.size(), prepareProtectedLayer);

    m_forbiddenMaterialByDepth.reserve(protectedLayers.size());
    for (std::size_t index = 0;
         index < protectedLayers.size(); ++index) {
        if (hasPrepared[index] == 0u)
            continue;
        m_forbiddenMaterialByDepth.emplace(
            protectedLayers[index].depthIndex,
            std::move(preparedProtected[index]));
    }
}

bool PocketSafetyArea::rootClear(const gp_Pnt2d& point) const
{
    return contains(m_rootSafeArea, point, true);
}

bool PocketSafetyArea::protectedMaterialClear(
    const gp_Pnt2d& point,
    int depthIndex) const
{
    const auto found = m_forbiddenMaterialByDepth.find(depthIndex);
    if (found == m_forbiddenMaterialByDepth.end())
        return true;

    const PreparedProtectedArea& prepared = found->second;
    if (!contains(prepared.forbidden, point, false))
        return true;

    // Rounded polygon offsets are a conservative broad phase. Around nested
    // concave material contours their discretized topology can locally extend
    // farther than the requested margin. Confirm broad-phase hits against the
    // original nested material and its exact segment distance so valid
    // cutter-centre points are not rejected while real incursions remain
    // errors.
    if (contains(prepared.material, point, true))
        return false;
    return distanceToAreaBoundary(prepared.material, point) >=
        m_checkedMargin;
}

bool PocketSafetyArea::clear(const gp_Pnt2d& point,
                             int depthIndex,
                             bool checkRootBoundary) const
{
    return (!checkRootBoundary || rootClear(point)) &&
           protectedMaterialClear(point, depthIndex);
}

} // namespace oap::toolpath::detail

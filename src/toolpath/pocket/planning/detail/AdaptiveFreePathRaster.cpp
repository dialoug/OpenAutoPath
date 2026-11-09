#include "AdaptiveFreePathRaster.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace oap::toolpath::detail {

namespace {

constexpr double kTolerance = 1.0e-12;

} // namespace

AdaptiveFreePathRaster::AdaptiveFreePathRaster(
    const PathsD& materialArea,
    const PathsD& safeCenterArea,
    double requestedCellSize,
    std::size_t maximumCells)
    : m_safeCenterArea(safeCenterArea)
{
    if (materialArea.empty() || safeCenterArea.empty() ||
        !std::isfinite(requestedCellSize) || requestedCellSize <= 0.0) {
        return;
    }
    const Clipper2Lib::RectD bounds = Clipper2Lib::GetBounds(materialArea);
    m_minX = bounds.left;
    m_minY = bounds.top;
    m_maxX = bounds.right;
    m_maxY = bounds.bottom;
    const double width = m_maxX - m_minX;
    const double height = m_maxY - m_minY;
    if (!(width > 0.0) || !(height > 0.0))
        return;

    double cellSize = requestedCellSize;
    const double requestedX = std::ceil(width / cellSize);
    const double requestedY = std::ceil(height / cellSize);
    const double requestedCount = requestedX * requestedY;
    if (requestedCount > static_cast<double>(maximumCells)) {
        cellSize *= std::sqrt(
            requestedCount / static_cast<double>(maximumCells));
    }
    m_nx = std::max<std::size_t>(
        1u, static_cast<std::size_t>(std::ceil(width / cellSize)));
    m_ny = std::max<std::size_t>(
        1u, static_cast<std::size_t>(std::ceil(height / cellSize)));
    m_dx = width / static_cast<double>(m_nx);
    m_dy = height / static_cast<double>(m_ny);
    m_cellSize = std::max(m_dx, m_dy);
    m_halfDiagonal = 0.5 * std::hypot(m_dx, m_dy);

    const std::size_t total = m_nx * m_ny;
    m_initial.assign(total, 0u);
    m_remaining.assign(total, 0u);
    m_safe.assign(total, 0u);
    for (std::size_t y = 0u; y < m_ny; ++y) {
        for (std::size_t x = 0u; x < m_nx; ++x) {
            const gp_Pnt2d candidate = point(x, y);
            const std::size_t index = y * m_nx + x;
            if (inArea(materialArea, candidate)) {
                m_initial[index] = 1u;
                m_remaining[index] = 1u;
                ++m_initialCount;
            }
            if (inArea(safeCenterArea, candidate)) {
                m_safe[index] = 1u;
                ++m_safeCount;
            }
        }
    }
    m_remainingCount = m_initialCount;
}

double AdaptiveFreePathRaster::remainingFraction() const
{
    return m_initialCount > 0u
        ? static_cast<double>(m_remainingCount) /
              static_cast<double>(m_initialCount)
        : 0.0;
}

gp_Pnt2d AdaptiveFreePathRaster::point(std::size_t x, std::size_t y) const
{
    return {
        m_minX + (static_cast<double>(x) + 0.5) * m_dx,
        m_minY + (static_cast<double>(y) + 0.5) * m_dy};
}

bool AdaptiveFreePathRaster::inArea(
    const PathsD& area,
    const gp_Pnt2d& point) const
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

bool AdaptiveFreePathRaster::safe(const gp_Pnt2d& point) const
{
    return inArea(m_safeCenterArea, point);
}

bool AdaptiveFreePathRaster::segmentSafe(
    const gp_Pnt2d& start,
    const gp_Pnt2d& end,
    double sampleSpacing) const
{
    const double length = start.Distance(end);
    if (!std::isfinite(length))
        return false;
    const double spacing = std::max(1.0e-5, sampleSpacing);
    const std::size_t samples = std::max<std::size_t>(
        1u, static_cast<std::size_t>(std::ceil(length / spacing)));
    for (std::size_t sample = 0u; sample <= samples; ++sample) {
        const double t = static_cast<double>(sample) /
                         static_cast<double>(samples);
        if (!safe(gp_Pnt2d(
                start.X() + t * (end.X() - start.X()),
                start.Y() + t * (end.Y() - start.Y())))) {
            return false;
        }
    }
    return true;
}

void AdaptiveFreePathRaster::cellRange(
    double minX, double minY, double maxX, double maxY,
    std::size_t& x0, std::size_t& y0,
    std::size_t& x1, std::size_t& y1) const
{
    const auto clampIndex = [](long long value, std::size_t count) {
        if (value <= 0)
            return std::size_t{0u};
        return std::min(
            static_cast<std::size_t>(value), count - 1u);
    };
    x0 = clampIndex(
        static_cast<long long>(std::floor((minX - m_minX) / m_dx)), m_nx);
    y0 = clampIndex(
        static_cast<long long>(std::floor((minY - m_minY) / m_dy)), m_ny);
    x1 = clampIndex(
        static_cast<long long>(std::floor((maxX - m_minX) / m_dx)), m_nx);
    y1 = clampIndex(
        static_cast<long long>(std::floor((maxY - m_minY) / m_dy)), m_ny);
}

double AdaptiveFreePathRaster::distanceSquaredToSegment(
    double x, double y,
    const gp_Pnt2d& start,
    const gp_Pnt2d& end) const
{
    const double dx = end.X() - start.X();
    const double dy = end.Y() - start.Y();
    const double lengthSquared = dx * dx + dy * dy;
    const double t = lengthSquared > kTolerance
        ? std::clamp(
              ((x - start.X()) * dx + (y - start.Y()) * dy) /
                  lengthSquared,
              0.0, 1.0)
        : 0.0;
    const double qx = start.X() + t * dx;
    const double qy = start.Y() + t * dy;
    return (x - qx) * (x - qx) + (y - qy) * (y - qy);
}

bool AdaptiveFreePathRaster::remainingAt(
    const gp_Pnt2d& candidate,
    const std::unordered_set<std::size_t>* ignored) const
{
    if (candidate.X() < m_minX || candidate.X() > m_maxX ||
        candidate.Y() < m_minY || candidate.Y() > m_maxY) {
        return false;
    }
    const std::size_t x = std::min(
        m_nx - 1u,
        static_cast<std::size_t>(std::max(
            0.0, std::floor((candidate.X() - m_minX) / m_dx))));
    const std::size_t y = std::min(
        m_ny - 1u,
        static_cast<std::size_t>(std::max(
            0.0, std::floor((candidate.Y() - m_minY) / m_dy))));
    const std::size_t index = y * m_nx + x;
    return m_remaining[index] != 0u &&
           (!ignored || ignored->find(index) == ignored->end());
}

double AdaptiveFreePathRaster::engagementRatio(
    const gp_Pnt2d& center,
    double radius,
    const std::unordered_set<std::size_t>* ignored) const
{
    if (!(radius > 0.0))
        return 0.0;
    std::size_t x0 = 0u, y0 = 0u, x1 = 0u, y1 = 0u;
    cellRange(center.X() - radius, center.Y() - radius,
              center.X() + radius, center.Y() + radius,
              x0, y0, x1, y1);
    const double radiusSquared = radius * radius;
    std::size_t initial = 0u;
    std::size_t remaining = 0u;
    for (std::size_t y = y0; y <= y1; ++y) {
        for (std::size_t x = x0; x <= x1; ++x) {
            const gp_Pnt2d candidate = point(x, y);
            if (candidate.SquareDistance(center) > radiusSquared)
                continue;
            const std::size_t index = y * m_nx + x;
            if (!m_initial[index])
                continue;
            ++initial;
            if (m_remaining[index] &&
                (!ignored || ignored->find(index) == ignored->end())) {
                ++remaining;
            }
        }
    }
    return initial > 0u
        ? static_cast<double>(remaining) / static_cast<double>(initial)
        : 0.0;
}

AdaptiveAngularEngagement AdaptiveFreePathRaster::angularEngagement(
    const gp_Pnt2d& center,
    double radius,
    int samples,
    const std::unordered_set<std::size_t>* ignored) const
{
    AdaptiveAngularEngagement result;
    if (!(radius > 0.0) || samples < 8)
        return result;
    const double pi = std::acos(-1.0);
    const double angleStep = 2.0 * pi / static_cast<double>(samples);
    const double sampleRadius = std::max(0.0, radius - 0.35 * m_cellSize);
    std::vector<std::uint8_t> occupied(static_cast<std::size_t>(samples), 0u);
    int occupiedCount = 0;
    for (int index = 0; index < samples; ++index) {
        const double angle = static_cast<double>(index) * angleStep;
        occupied[static_cast<std::size_t>(index)] = remainingAt(
            gp_Pnt2d(center.X() + sampleRadius * std::cos(angle),
                     center.Y() + sampleRadius * std::sin(angle)),
            ignored) ? 1u : 0u;
        occupiedCount += occupied[static_cast<std::size_t>(index)] ? 1 : 0;
    }
    int longest = 0;
    int current = 0;
    for (int index = 0; index < 2 * samples; ++index) {
        if (occupied[static_cast<std::size_t>(index % samples)]) {
            current = std::min(samples, current + 1);
            longest = std::max(longest, current);
        } else {
            current = 0;
        }
    }
    result.contactFraction = static_cast<double>(occupiedCount) /
                             static_cast<double>(samples);
    result.maximumArcAngle = static_cast<double>(longest) * angleStep;
    return result;
}

std::vector<std::size_t> AdaptiveFreePathRaster::segmentRemainingIndices(
    const gp_Pnt2d& start,
    const gp_Pnt2d& end,
    double radius,
    const std::unordered_set<std::size_t>* ignored) const
{
    std::vector<std::size_t> result;
    const double effectiveRadius = radius + m_halfDiagonal;
    std::size_t x0 = 0u, y0 = 0u, x1 = 0u, y1 = 0u;
    cellRange(
        std::min(start.X(), end.X()) - effectiveRadius,
        std::min(start.Y(), end.Y()) - effectiveRadius,
        std::max(start.X(), end.X()) + effectiveRadius,
        std::max(start.Y(), end.Y()) + effectiveRadius,
        x0, y0, x1, y1);
    const double radiusSquared = effectiveRadius * effectiveRadius;
    for (std::size_t y = y0; y <= y1; ++y) {
        for (std::size_t x = x0; x <= x1; ++x) {
            const std::size_t index = y * m_nx + x;
            if (!m_remaining[index] ||
                (ignored && ignored->find(index) != ignored->end())) {
                continue;
            }
            const gp_Pnt2d candidate = point(x, y);
            if (distanceSquaredToSegment(
                    candidate.X(), candidate.Y(), start, end) <=
                radiusSquared) {
                result.push_back(index);
            }
        }
    }
    return result;
}

std::size_t AdaptiveFreePathRaster::polylineRemainingCount(
    const std::vector<gp_Pnt2d>& points,
    double radius) const
{
    std::unordered_set<std::size_t> found;
    for (std::size_t index = 1u; index < points.size(); ++index) {
        const std::vector<std::size_t> segment = segmentRemainingIndices(
            points[index - 1u], points[index], radius, &found);
        found.insert(segment.begin(), segment.end());
    }
    return found.size();
}

std::size_t AdaptiveFreePathRaster::removeSegment(
    const gp_Pnt2d& start,
    const gp_Pnt2d& end,
    double radius)
{
    const std::vector<std::size_t> indices =
        segmentRemainingIndices(start, end, radius);
    for (const std::size_t index : indices)
        m_remaining[index] = 0u;
    m_remainingCount -= indices.size();
    return indices.size();
}

std::size_t AdaptiveFreePathRaster::removePolyline(
    const std::vector<gp_Pnt2d>& points,
    double radius)
{
    std::size_t removed = 0u;
    for (std::size_t index = 1u; index < points.size(); ++index)
        removed += removeSegment(points[index - 1u], points[index], radius);
    return removed;
}

std::vector<AdaptiveRasterComponent> AdaptiveFreePathRaster::components(
    std::size_t minimumCells) const
{
    std::vector<AdaptiveRasterComponent> result;
    std::vector<std::uint8_t> visited(m_remaining.size(), 0u);
    constexpr int offsets[8][2] = {
        {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
        {1, 0}, {-1, 1}, {0, 1}, {1, 1}};
    for (std::size_t seed = 0u; seed < m_remaining.size(); ++seed) {
        if (!m_remaining[seed] || visited[seed])
            continue;
        std::queue<std::size_t> pending;
        pending.push(seed);
        visited[seed] = 1u;
        AdaptiveRasterComponent component;
        component.minX = component.maxX = seed % m_nx;
        component.minY = component.maxY = seed / m_nx;
        double sumX = 0.0;
        double sumY = 0.0;
        while (!pending.empty()) {
            const std::size_t index = pending.front();
            pending.pop();
            component.cells.push_back(index);
            const std::size_t x = index % m_nx;
            const std::size_t y = index / m_nx;
            component.minX = std::min(component.minX, x);
            component.maxX = std::max(component.maxX, x);
            component.minY = std::min(component.minY, y);
            component.maxY = std::max(component.maxY, y);
            const gp_Pnt2d candidate = point(x, y);
            sumX += candidate.X();
            sumY += candidate.Y();
            for (const auto& offset : offsets) {
                const long long nx = static_cast<long long>(x) + offset[0];
                const long long ny = static_cast<long long>(y) + offset[1];
                if (nx < 0 || ny < 0 ||
                    nx >= static_cast<long long>(m_nx) ||
                    ny >= static_cast<long long>(m_ny)) {
                    continue;
                }
                const std::size_t neighbour =
                    static_cast<std::size_t>(ny) * m_nx +
                    static_cast<std::size_t>(nx);
                if (m_remaining[neighbour] && !visited[neighbour]) {
                    visited[neighbour] = 1u;
                    pending.push(neighbour);
                }
            }
        }
        if (component.cells.size() < minimumCells)
            continue;
        const double count = static_cast<double>(component.cells.size());
        component.centroid = gp_Pnt2d(sumX / count, sumY / count);
        double xx = 0.0;
        double yy = 0.0;
        double xy = 0.0;
        for (const std::size_t index : component.cells) {
            const gp_Pnt2d candidate = point(index % m_nx, index / m_nx);
            const double dx = candidate.X() - component.centroid.X();
            const double dy = candidate.Y() - component.centroid.Y();
            xx += dx * dx;
            yy += dy * dy;
            xy += dx * dy;
        }
        component.principalAngle = 0.5 * std::atan2(2.0 * xy, xx - yy);
        result.push_back(std::move(component));
    }
    return result;
}

} // namespace oap::toolpath::detail

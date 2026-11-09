// Independently compiled private MultiDexelOrientedSweep implementation.
#include "toolpath/pocket/simulation/detail/MultiDexelOrientedSweep.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "toolpath/pocket/simulation/detail/MultiDexelTileUpdates.h"

namespace oap::toolpath::multi_dexel_detail {

OrientedSweepProfile buildOrientedSweepProfile(
    const LocalPoint& axis,
    double maximumRadius,
    double requestedRadialError,
    int maximumFacetCount)
{
    OrientedSweepProfile result;
    const double error = std::clamp(
        requestedRadialError,
        std::max(1.0e-9, maximumRadius * 1.0e-9),
        std::max(1.0e-9, maximumRadius));
    const double halfAngle = std::acos(std::clamp(
        1.0 - error / maximumRadius, -1.0, 1.0));
    const int facetLimit = std::clamp(
        maximumFacetCount, 8, 1024);
    const int facetCount = std::clamp(
        halfAngle > 1.0e-9
            ? static_cast<int>(std::ceil(
                  std::numbers::pi / halfAngle))
            : facetLimit,
        8, facetLimit);
    result.facetCosine = std::cos(
        std::numbers::pi /
        static_cast<double>(facetCount));
    result.maximumFacetError = maximumRadius *
        (1.0 - result.facetCosine);

    const LocalPoint reference = std::abs(axis.w) < 0.9
        ? LocalPoint{0.0, 0.0, 1.0}
        : LocalPoint{1.0, 0.0, 0.0};
    const LocalPoint radialU = normalizedLocal(
        cross(axis, reference));
    const LocalPoint radialV = normalizedLocal(
        cross(axis, radialU));
    result.radialNormals.reserve(
        static_cast<std::size_t>(facetCount));
    for (int facet = 0; facet < facetCount; ++facet) {
        const double angle = 2.0 * std::numbers::pi *
            static_cast<double>(facet) /
            static_cast<double>(facetCount);
        result.radialNormals.push_back({
            radialU.u * std::cos(angle) +
                radialV.u * std::sin(angle),
            radialU.v * std::cos(angle) +
                radialV.v * std::sin(angle),
            radialU.w * std::cos(angle) +
                radialV.w * std::sin(angle)});
    }
    return result;
}

bool clipSweepParameterPolygon(
    std::vector<SweepParameterPoint>& polygon,
    double coefficientW,
    double coefficientT,
    double maximum,
    double numericTolerance)
{
    if (polygon.empty())
        return false;
    std::vector<SweepParameterPoint> clipped;
    clipped.reserve(polygon.size() + 1u);
    auto signedDistance = [&](const SweepParameterPoint& point) {
        return coefficientW * point.w +
               coefficientT * point.t - maximum;
    };
    SweepParameterPoint previous = polygon.back();
    double previousDistance = signedDistance(previous);
    bool previousInside = previousDistance <= numericTolerance;
    for (const SweepParameterPoint& current : polygon) {
        const double currentDistance = signedDistance(current);
        const bool currentInside =
            currentDistance <= numericTolerance;
        if (previousInside != currentInside) {
            const double denominator =
                previousDistance - currentDistance;
            if (std::abs(denominator) > 1.0e-15) {
                const double ratio = std::clamp(
                    previousDistance / denominator,
                    0.0, 1.0);
                clipped.push_back({
                    previous.w + ratio *
                        (current.w - previous.w),
                    previous.t + ratio *
                        (current.t - previous.t)});
            }
        }
        if (currentInside)
            clipped.push_back(current);
        previous = current;
        previousDistance = currentDistance;
        previousInside = currentInside;
    }
    polygon = std::move(clipped);
    return !polygon.empty();
}

std::optional<DexelInterval> orientedSweepIntersection(
    const OrientedRasterMove& move,
    const LocalPoint& axis,
    const OrientedSweepProfile& profile,
    const DexelToolProfileBand& band,
    double cellU,
    double cellV,
    const DexelInterval& material,
    double numericTolerance)
{
    std::vector<SweepParameterPoint> polygon{
        {material.lower, 0.0},
        {material.upper, 0.0},
        {material.upper, 1.0},
        {material.lower, 1.0}};
    const LocalPoint travel = difference(
        move.to, move.from);
    const double axisTravel = dot(axis, travel);
    const double axialFixed =
        axis.u * (cellU - move.from.u) +
        axis.v * (cellV - move.from.v) -
        axis.w * move.from.w;
    if (!clipSweepParameterPolygon(
            polygon, axis.w, -axisTravel,
            band.upper - axialFixed,
            numericTolerance) ||
        !clipSweepParameterPolygon(
            polygon, -axis.w, axisTravel,
            -band.lower + axialFixed,
            numericTolerance)) {
        return std::nullopt;
    }

    const double apothem =
        band.radius * profile.facetCosine;
    for (const LocalPoint& normal :
         profile.radialNormals) {
        const double normalTravel =
            dot(normal, travel);
        const double radialFixed =
            normal.u * (cellU - move.from.u) +
            normal.v * (cellV - move.from.v) -
            normal.w * move.from.w;
        if (!clipSweepParameterPolygon(
                polygon,
                normal.w,
                -normalTravel,
                apothem - radialFixed,
                numericTolerance)) {
            return std::nullopt;
        }
    }

    double lower = std::numeric_limits<double>::infinity();
    double upper = -std::numeric_limits<double>::infinity();
    for (const SweepParameterPoint& point : polygon) {
        lower = std::min(lower, point.w);
        upper = std::max(upper, point.w);
    }
    lower = std::max(lower, material.lower);
    upper = std::min(upper, material.upper);
    if (!std::isfinite(lower) || !std::isfinite(upper) ||
        upper <= lower + numericTolerance) {
        return std::nullopt;
    }
    return DexelInterval{lower, upper};
}

int subtractOrientedMovesFromTile(
    const MultiDexelStock::Impl& grid,
    MultiDexelStock::Impl::Tile& tile,
    const std::vector<OrientedRasterMove>& moves,
    std::span<const int> moveIndices,
    const LocalPoint& operationAxis,
    const OrientedSweepProfile& sweepProfile,
    const DexelToolProfile& toolProfile,
    double numericTolerance,
    const std::function<bool()>& cancellationRequested)
{
    std::vector<CellRemovalAccumulator> removals(
        tile.cells.size());
    std::size_t processedMoveCount = 0u;
    for (const int moveIndex : moveIndices) {
        if ((processedMoveCount++ & 15u) == 0u &&
            cancellationRequested &&
            cancellationRequested()) {
            return -1;
        }
        const OrientedRasterMove& move =
            moves[static_cast<std::size_t>(moveIndex)];
        const int iMin = std::max(move.iMin, tile.iMin);
        const int iMax = std::min(
            move.iMax, tile.iMin + tile.width - 1);
        const int jMin = std::max(move.jMin, tile.jMin);
        const int jMax = std::min(
            move.jMax, tile.jMin + tile.height - 1);
        for (int j = jMin; j <= jMax; ++j) {
            const double v = grid.cellV(j);
            for (int i = iMin; i <= iMax; ++i) {
                MultiDexelStock::Impl::Cell& cell =
                    tile.cell(i, j);
                if (cell.intervals.empty())
                    continue;
                const double u = grid.cellU(i);
                const std::size_t local =
                    static_cast<std::size_t>(j - tile.jMin) *
                        static_cast<std::size_t>(tile.width) +
                    static_cast<std::size_t>(i - tile.iMin);
                for (const DexelInterval& material :
                     cell.intervals) {
                    for (const DexelToolProfileBand& band :
                         toolProfile.bands) {
                        const auto removal =
                            orientedSweepIntersection(
                                move, operationAxis,
                                sweepProfile, band,
                                u, v, material,
                                numericTolerance);
                        if (removal) {
                            removals[local].add(
                                *removal, grid.tolerance);
                        }
                    }
                }
            }
        }
    }

    int changedCellCount = 0;
    for (std::size_t local = 0;
         local < removals.size(); ++local) {
        if (removals[local].apply(
                tile.cells[local].intervals,
                grid.tolerance)) {
            ++changedCellCount;
        }
    }
    return changedCellCount;
}

bool sameInterval(
    const DexelInterval& left,
    const DexelInterval& right,
    double tolerance)
{
    return std::abs(left.lower - right.lower) <= tolerance &&
           std::abs(left.upper - right.upper) <= tolerance;
}

} // namespace oap::toolpath::multi_dexel_detail

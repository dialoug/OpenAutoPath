// Independently compiled private MultiDexelTileUpdates implementation.
#include "toolpath/pocket/simulation/detail/MultiDexelTileUpdates.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <span>
#include <vector>

#include "toolpath/pocket/simulation/DexelIntervalOps.h"

namespace oap::toolpath::multi_dexel_detail {

void CellRemovalAccumulator::add(
        const DexelInterval& removal,
        double tolerance)
{
        if (!std::isfinite(removal.lower) ||
            !std::isfinite(removal.upper) ||
            removal.upper <=
                removal.lower + tolerance) {
            return;
        }
        if (!m_primary) {
            m_primary = removal;
            return;
        }

        DexelInterval merged = removal;
        bool absorbedPrimary = false;
        bool absorbed = false;
        do {
            absorbed = false;
            if (!absorbedPrimary &&
                touching(
                    merged, *m_primary,
                    tolerance)) {
                mergeInto(merged, *m_primary);
                absorbedPrimary = true;
                absorbed = true;
            }
            for (std::size_t index = 0;
                 index < m_additional.size();) {
                if (!touching(
                        merged,
                        m_additional[index],
                        tolerance)) {
                    ++index;
                    continue;
                }
                mergeInto(
                    merged, m_additional[index]);
                m_additional.erase(
                    m_additional.begin() +
                    static_cast<std::ptrdiff_t>(
                        index));
                absorbed = true;
            }
        } while (absorbed);

        if (absorbedPrimary) {
            *m_primary = merged;
        } else {
            m_additional.push_back(merged);
        }
    }

bool CellRemovalAccumulator::apply(
        DexelCellIntervals& material,
        double tolerance)
{
        if (!m_primary || material.empty())
            return false;
        if (m_additional.empty()) {
            return subtractDexelInterval(
                material, *m_primary, tolerance);
        }

        m_additional.push_back(*m_primary);
        std::sort(
            m_additional.begin(),
            m_additional.end(),
            [](const DexelInterval& left,
               const DexelInterval& right) {
                return left.lower < right.lower;
            });
        bool changed = false;
        for (const DexelInterval& removal :
             m_additional) {
            changed =
                subtractDexelInterval(
                    material, removal, tolerance) ||
                changed;
            if (material.empty())
                break;
        }
        return changed;
    }

bool CellRemovalAccumulator::touching(
        const DexelInterval& left,
        const DexelInterval& right,
        double tolerance)
{
        return left.lower <=
                   right.upper + tolerance &&
               right.lower <=
                   left.upper + tolerance;
    }

void CellRemovalAccumulator::mergeInto(
        DexelInterval& target,
        const DexelInterval& source)
{
        target.lower =
            std::min(target.lower, source.lower);
        target.upper =
            std::max(target.upper, source.upper);
    }

int subtractPocketMovesFromTile(
    const MultiDexelStock::Impl& grid,
    MultiDexelStock::Impl::Tile& tile,
    const std::vector<RasterMove>& moves,
    std::span<const int> moveIndices,
    const DexelToolProfile& toolProfile,
    double numericTolerance,
    const std::function<bool()>&
        cancellationRequested)
{
    std::vector<CellRemovalAccumulator> removals(
        tile.cells.size());
    std::size_t processedMoveCount = 0u;
    for (const int moveIndex : moveIndices) {
        if ((processedMoveCount++ & 31u) == 0u &&
            cancellationRequested &&
            cancellationRequested()) {
            return -1;
        }
        const RasterMove& move =
            moves[static_cast<std::size_t>(moveIndex)];
        const int iMin =
            std::max(move.iMin, tile.iMin);
        const int iMax =
            std::min(
                move.iMax,
                tile.iMin + tile.width - 1);
        const int jMin =
            std::max(move.jMin, tile.jMin);
        const int jMax =
            std::min(
                move.jMax,
                tile.jMin + tile.height - 1);
        const double du = move.u1 - move.u0;
        const double dv = move.v1 - move.v0;
        const double dw = move.w1 - move.w0;
        const double lengthSquared =
            du * du + dv * dv;
        for (int j = jMin; j <= jMax; ++j) {
            const double v = grid.cellV(j);
            for (int i = iMin; i <= iMax; ++i) {
                MultiDexelStock::Impl::Cell& cell =
                    tile.cell(i, j);
                if (cell.intervals.empty())
                    continue;

                const double u = grid.cellU(i);
                const double pu = u - move.u0;
                const double pv = v - move.v0;
                for (const DexelToolProfileBand& band :
                     toolProfile.bands) {
                    const double radiusSquared =
                        band.radius * band.radius;
                    double tMin = 0.0;
                    double tMax = 1.0;
                    if (lengthSquared <=
                        numericTolerance) {
                        if (pu * pu + pv * pv >
                            radiusSquared +
                                numericTolerance) {
                            continue;
                        }
                    } else {
                        const double dot =
                            pu * du + pv * dv;
                        const double c =
                            pu * pu + pv * pv -
                            radiusSquared;
                        double discriminant =
                            dot * dot -
                            lengthSquared * c;
                        if (discriminant <
                            -numericTolerance) {
                            continue;
                        }
                        discriminant =
                            std::max(
                                0.0, discriminant);
                        const double root =
                            std::sqrt(discriminant);
                        tMin = std::max(
                            0.0,
                            (dot - root) /
                                lengthSquared);
                        tMax = std::min(
                            1.0,
                            (dot + root) /
                                lengthSquared);
                        if (tMin > tMax)
                            continue;
                    }

                    const double wAtMin =
                        move.w0 + tMin * dw;
                    const double wAtMax =
                        move.w0 + tMax * dw;
                    const DexelInterval removal{
                        std::min(wAtMin, wAtMax) +
                            band.lower,
                        std::max(wAtMin, wAtMax) +
                            band.upper};
                    const std::size_t local =
                        static_cast<std::size_t>(
                            j - tile.jMin) *
                            static_cast<std::size_t>(
                            tile.width) +
                        static_cast<std::size_t>(
                            i - tile.iMin);
                    removals[local].add(
                        removal, grid.tolerance);
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

LocalPoint cross(
    const LocalPoint& left,
    const LocalPoint& right)
{
    return {
        left.v * right.w - left.w * right.v,
        left.w * right.u - left.u * right.w,
        left.u * right.v - left.v * right.u};
}

LocalPoint normalizedLocal(LocalPoint value)
{
    const double length = std::sqrt(dot(value, value));
    if (!std::isfinite(length) || length <= 1.0e-12)
        return {};
    value.u /= length;
    value.v /= length;
    value.w /= length;
    return value;
}

} // namespace oap::toolpath::multi_dexel_detail

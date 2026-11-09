// Private MultiDexelOrientedSweep contract; not part of the module public API.
#pragma once

#include "toolpath/pocket/simulation/detail/MultiDexelTileUpdates.h"

namespace oap::toolpath::multi_dexel_detail {

struct OrientedSweepProfile {
    std::vector<LocalPoint> radialNormals;
    double facetCosine = 1.0;
    double maximumFacetError = 0.0;
};

OrientedSweepProfile buildOrientedSweepProfile(
    const LocalPoint& axis,
    double maximumRadius,
    double requestedRadialError,
    int maximumFacetCount);

bool clipSweepParameterPolygon(
    std::vector<SweepParameterPoint>& polygon,
    double coefficientW,
    double coefficientT,
    double maximum,
    double numericTolerance);

std::optional<DexelInterval> orientedSweepIntersection(
    const OrientedRasterMove& move,
    const LocalPoint& axis,
    const OrientedSweepProfile& profile,
    const DexelToolProfileBand& band,
    double cellU,
    double cellV,
    const DexelInterval& material,
    double numericTolerance);

int subtractOrientedMovesFromTile(
    const MultiDexelStock::Impl& grid,
    MultiDexelStock::Impl::Tile& tile,
    const std::vector<OrientedRasterMove>& moves,
    std::span<const int> moveIndices,
    const LocalPoint& operationAxis,
    const OrientedSweepProfile& sweepProfile,
    const DexelToolProfile& toolProfile,
    double numericTolerance,
    const std::function<bool()>& cancellationRequested);

bool sameInterval(
    const DexelInterval& left,
    const DexelInterval& right,
    double tolerance);

} // namespace oap::toolpath::multi_dexel_detail

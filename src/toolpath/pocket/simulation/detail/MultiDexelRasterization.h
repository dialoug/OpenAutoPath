// Private MultiDexelRasterization contract; not part of the module public API.
#pragma once

#include "toolpath/pocket/simulation/detail/MultiDexelImpl.h"
#include "toolpath/core/simulation/FixedAxisToolpathView.h"

namespace oap::toolpath::multi_dexel_detail {

std::vector<DexelInterval> intervalsFromCrossings(
    std::vector<Crossing>& crossings,
    double tolerance);

// Scratch storage is reusable across grid columns; crossings are consolidated
// in place and no longer needed by callers after interval extraction.
void intervalsFromCrossings(
    std::vector<Crossing>& crossings,
    double tolerance,
    std::vector<DexelInterval>& intervals);

void rasterizeTriangle(
    const MultiDexelStock::Impl& grid,
    const LocalPoint& a,
    const LocalPoint& b,
    const LocalPoint& c,
    int orientationSign,
    std::vector<std::vector<Crossing>>& crossings);

double effectiveCuttingLength(
    const MultiDexelStock::Impl& grid,
    const FixedAxisToolpathView& operation,
    const ToolpathParams& params,
    double cutterDiameter);

bool prepareMoves(
    const MultiDexelStock::Impl& grid,
    const FixedAxisToolpathView& operation,
    std::size_t completedMoveCount,
    double cutterDiameter,
    const StockSimulationOptions& options,
    std::vector<RasterMove>& moves,
    int& planarLayerCount,
    int& nonPlanarMoveCount,
    std::string& error);

bool prepareOrientedMoves(
    const MultiDexelStock::Impl& grid,
    const FixedAxisToolpathView& operation,
    std::size_t completedMoveCount,
    double cutterDiameter,
    double cuttingLength,
    const StockSimulationOptions& options,
    std::vector<OrientedRasterMove>& moves,
    LocalPoint& operationAxis,
    int& planarLayerCount,
    int& nonPlanarMoveCount,
    std::string& error);

} // namespace oap::toolpath::multi_dexel_detail

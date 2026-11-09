// Independently compiled private MultiDexelOperations implementation.
#include "toolpath/pocket/simulation/MultiDexelStock.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "toolpath/pocket/simulation/detail/MultiDexelImpl.h"
#include "toolpath/pocket/simulation/detail/MultiDexelRasterization.h"
#include "toolpath/pocket/simulation/detail/MultiDexelOrientedSweep.h"
#include "toolpath/pocket/simulation/detail/MultiDexelTileUpdates.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/core/simulation/DexelToolProfile.h"

namespace oap::toolpath {
using namespace oap::toolpath::multi_dexel_detail;

namespace {
struct TileWorkPlan {
    std::vector<std::vector<int>> bins;
    std::vector<int> activeTiles;
};

template <typename BoundsForWork>
TileWorkPlan buildTileWorkPlan(
    const MultiDexelStock::Impl& grid,
    int workCount,
    BoundsForWork boundsForWork)
{
    const int tileCount =
        grid.tilesX * grid.tilesY;
    TileWorkPlan plan;
    plan.bins.resize(
        static_cast<std::size_t>(tileCount));
    std::vector<std::uint64_t> estimatedCellVisits(
        static_cast<std::size_t>(tileCount), 0u);
    for (int workIndex = 0;
         workIndex < workCount; ++workIndex) {
        const auto& bounds = boundsForWork(workIndex);
        if (bounds.iMin > bounds.iMax ||
            bounds.jMin > bounds.jMax) {
            continue;
        }
        const int iMin =
            std::clamp(bounds.iMin, 0, grid.nx - 1);
        const int iMax =
            std::clamp(bounds.iMax, 0, grid.nx - 1);
        const int jMin =
            std::clamp(bounds.jMin, 0, grid.ny - 1);
        const int jMax =
            std::clamp(bounds.jMax, 0, grid.ny - 1);
        if (iMin > iMax || jMin > jMax)
            continue;

        const int tx0 = iMin / grid.tileSize;
        const int tx1 = iMax / grid.tileSize;
        const int ty0 = jMin / grid.tileSize;
        const int ty1 = jMax / grid.tileSize;
        for (int ty = ty0; ty <= ty1; ++ty) {
            for (int tx = tx0; tx <= tx1; ++tx) {
                const int tile =
                    ty * grid.tilesX + tx;
                const std::size_t tileOffset =
                    static_cast<std::size_t>(tile);
                plan.bins[tileOffset]
                    .push_back(workIndex);
                const int tileIMin =
                    tx * grid.tileSize;
                const int tileJMin =
                    ty * grid.tileSize;
                const int tileIMax =
                    std::min(
                        grid.nx - 1,
                        tileIMin + grid.tileSize - 1);
                const int tileJMax =
                    std::min(
                        grid.ny - 1,
                        tileJMin + grid.tileSize - 1);
                const std::uint64_t width =
                    static_cast<std::uint64_t>(
                        std::max(
                            0,
                            std::min(iMax, tileIMax) -
                                std::max(iMin, tileIMin) +
                                1));
                const std::uint64_t height =
                    static_cast<std::uint64_t>(
                        std::max(
                            0,
                            std::min(jMax, tileJMax) -
                                std::max(jMin, tileJMin) +
                                1));
                estimatedCellVisits[tileOffset] +=
                    width * height;
            }
        }
    }

    plan.activeTiles.reserve(
        static_cast<std::size_t>(tileCount));
    for (int tile = 0; tile < tileCount; ++tile) {
        if (!plan.bins[static_cast<std::size_t>(
                           tile)]
                 .empty()) {
            plan.activeTiles.push_back(tile);
        }
    }
    // 动态任务队列先投放预计工作量最大的 tile，减少长刀路末尾只剩
    // 少数重 tile 而其他工作线程空闲的拖尾。
    std::stable_sort(
        plan.activeTiles.begin(),
        plan.activeTiles.end(),
        [&](int left, int right) {
            return estimatedCellVisits[
                       static_cast<std::size_t>(left)] >
                   estimatedCellVisits[
                       static_cast<std::size_t>(right)];
        });
    return plan;
}

struct TileUpdateTransaction {
    std::shared_ptr<MultiDexelStock::Impl> stock;
    int changedCellCount = 0;
    double milliseconds = 0.0;
    bool cancelled = false;
};

template <typename UpdateTile>
TileUpdateTransaction runTileUpdateTransaction(
    const std::shared_ptr<const MultiDexelStock::Impl>&
        source,
    std::span<const int> activeTiles,
    bool runParallel,
    const std::function<bool()>& cancellationRequested,
    UpdateTile updateTile)
{
    TileUpdateTransaction result;
    if (activeTiles.empty())
        return result;

    const SteadyClock::time_point updateStart =
        SteadyClock::now();
    auto next =
        std::make_shared<MultiDexelStock::Impl>(*source);
    std::vector<std::shared_ptr<
        MultiDexelStock::Impl::Tile>> mutableTiles(
        source->tiles.size());
    for (const int tile : activeTiles) {
        mutableTiles[static_cast<std::size_t>(tile)] =
            std::make_shared<MultiDexelStock::Impl::Tile>(
                *source->tiles[
                    static_cast<std::size_t>(tile)]);
    }

    std::atomic_int changedCells{0};
    std::atomic_bool wasCancelled{false};
    const auto update = [&](std::size_t work) {
        try {
            if (wasCancelled.load(
                    std::memory_order_relaxed)) {
                return;
            }
            if (cancellationRequested &&
                cancellationRequested()) {
                wasCancelled.store(
                    true, std::memory_order_relaxed);
                return;
            }

            const int tileIndex =
                activeTiles[work];
            const int changed =
                updateTile(
                    tileIndex,
                    *mutableTiles[
                        static_cast<std::size_t>(
                            tileIndex)]);
            if (changed < 0) {
                wasCancelled.store(
                    true, std::memory_order_relaxed);
                return;
            }
            if (changed > 0) {
                changedCells.fetch_add(
                    changed,
                    std::memory_order_relaxed);
            }
        } catch (...) {
            // Stop taking useful work after a callback/allocation failure. The
            // common wrapper joins all workers before rethrowing, so neither
            // temporary tiles nor their captured inputs can escape this scope.
            wasCancelled.store(true, std::memory_order_relaxed);
            throw;
        }
    };

    // The caller supplies one execution slot. Nested/standalone updates use
    // the same inherited/global budget as planning, never a separate group of
    // hardware-sized threads. Explicit serial mode must not borrow any slots.
    if (runParallel) {
        toolpathParallelForBudgeted(activeTiles.size(), update);
    } else {
        for (std::size_t work = 0; work < activeTiles.size(); ++work)
            update(work);
    }
    result.cancelled =
        wasCancelled.load(std::memory_order_relaxed) ||
        (cancellationRequested && cancellationRequested());
    if (result.cancelled) {
        result.milliseconds =
            elapsedMilliseconds(updateStart);
        return result;
    }

    result.changedCellCount =
        changedCells.load(std::memory_order_relaxed);
    if (result.changedCellCount == 0) {
        result.milliseconds =
            elapsedMilliseconds(updateStart);
        return result;
    }

    for (const int tile : activeTiles) {
        next->tiles[static_cast<std::size_t>(tile)] =
            mutableTiles[static_cast<std::size_t>(tile)];
    }
    next->refreshDerivedData(activeTiles);
    next->valid = true;
    result.stock = std::move(next);
    result.milliseconds =
        elapsedMilliseconds(updateStart);
    return result;
}


} // namespace

MultiDexelApplyResult
MultiDexelStock::applyFixedAxis(
    const FixedAxisToolpathView& operation,
    const ToolpathParams& params,
    int sequenceIndex,
    const StockSimulationOptions& options,
    std::function<bool()> cancellationRequested) const
{
    return applyMoveLimit(
        operation, params, sequenceIndex,
        operation.moves.size(),
        false, options,
        std::move(cancellationRequested), nullptr,
        false);
}

MultiDexelApplyResult
MultiDexelStock::applyPocket(
    const PocketToolpathOperation& operation,
    const ToolpathParams& params,
    int sequenceIndex,
    const StockSimulationOptions& options,
    std::function<bool()> cancellationRequested) const
{
    const FixedAxisToolpathView operationView{
        operation.operationId, operation.outwardToolAxis,
        operation.clearing.moves};
    return applyFixedAxis(
        operationView, params, sequenceIndex, options,
        std::move(cancellationRequested));
}

MultiDexelApplyResult
MultiDexelStock::applyAxialToolpath(
    const FixedAxisToolpathView& operation,
    const ToolpathParams& params,
    const DexelToolProfileDefinition& toolProfile,
    int sequenceIndex,
    const StockSimulationOptions& options,
    std::function<bool()> cancellationRequested) const
{
    return applyMoveLimit(
        operation, params, sequenceIndex,
        operation.moves.size(), false,
        options, std::move(cancellationRequested),
        &toolProfile, false);
}

MultiDexelApplyResult
MultiDexelStock::applyAxialToolpath(
    const PocketToolpathOperation& operation,
    const ToolpathParams& params,
    const DexelToolProfileDefinition& toolProfile,
    int sequenceIndex,
    const StockSimulationOptions& options,
    std::function<bool()> cancellationRequested) const
{
    const FixedAxisToolpathView operationView{
        operation.operationId, operation.outwardToolAxis,
        operation.clearing.moves};
    return applyAxialToolpath(
        operationView, params, toolProfile, sequenceIndex, options,
        std::move(cancellationRequested));
}

MultiDexelApplyResult
MultiDexelStock::applyOrientedToolpath(
    const FixedAxisToolpathView& operation,
    const ToolpathParams& params,
    const DexelToolProfileDefinition& toolProfile,
    int sequenceIndex,
    const StockSimulationOptions& options,
    std::function<bool()> cancellationRequested) const
{
    return applyMoveLimit(
        operation, params, sequenceIndex,
        operation.moves.size(), false,
        options, std::move(cancellationRequested),
        &toolProfile, true);
}

MultiDexelApplyResult
MultiDexelStock::applyOrientedToolpath(
    const PocketToolpathOperation& operation,
    const ToolpathParams& params,
    const DexelToolProfileDefinition& toolProfile,
    int sequenceIndex,
    const StockSimulationOptions& options,
    std::function<bool()> cancellationRequested) const
{
    const FixedAxisToolpathView operationView{
        operation.operationId, operation.outwardToolAxis,
        operation.clearing.moves};
    return applyOrientedToolpath(
        operationView, params, toolProfile, sequenceIndex, options,
        std::move(cancellationRequested));
}

MultiDexelApplyResult
MultiDexelStock::applyPocketPrefix(
    const PocketToolpathOperation& operation,
    const ToolpathParams& params,
    int sequenceIndex,
    std::size_t completedMoveCount,
    const StockSimulationOptions& options,
    std::function<bool()> cancellationRequested) const
{
    const FixedAxisToolpathView operationView{
        operation.operationId, operation.outwardToolAxis,
        operation.clearing.moves};
    return applyPocketPrefix(operationView, params, sequenceIndex,
        completedMoveCount, options, std::move(cancellationRequested));
}

MultiDexelApplyResult
MultiDexelStock::applyPocketPrefix(
    const FixedAxisToolpathView& operationView,
    const ToolpathParams& params,
    int sequenceIndex,
    std::size_t completedMoveCount,
    const StockSimulationOptions& options,
    std::function<bool()> cancellationRequested) const
{
    return applyMoveLimit(
        operationView, params, sequenceIndex,
        std::min(completedMoveCount, operationView.moves.size()),
        true, options, std::move(cancellationRequested), nullptr, false);
}

MultiDexelApplyResult
MultiDexelStock::applyToolpathPrefix(
    const FixedAxisToolpathView& operation,
    const ToolpathParams& params,
    const DexelToolProfileDefinition& toolProfile,
    int sequenceIndex,
    std::size_t completedMoveCount,
    const StockSimulationOptions& options,
    std::function<bool()> cancellationRequested) const
{
    return applyMoveLimit(
        operation, params, sequenceIndex,
        std::min(completedMoveCount, operation.moves.size()),
        true, options, std::move(cancellationRequested),
        &toolProfile, !axisAligned(operation.outwardToolAxis));
}

MultiDexelApplyResult
MultiDexelStock::applyToolpathPrefix(
    const PocketToolpathOperation& operation,
    const ToolpathParams& params,
    const DexelToolProfileDefinition& toolProfile,
    int sequenceIndex,
    std::size_t completedMoveCount,
    const StockSimulationOptions& options,
    std::function<bool()> cancellationRequested) const
{
    const FixedAxisToolpathView operationView{
        operation.operationId, operation.outwardToolAxis,
        operation.clearing.moves};
    return applyToolpathPrefix(
        operationView, params, toolProfile, sequenceIndex,
        completedMoveCount, options, std::move(cancellationRequested));
}

MultiDexelApplyResult
MultiDexelStock::applyMoveLimit(
    const FixedAxisToolpathView& operation,
    const ToolpathParams& params,
    int sequenceIndex,
    std::size_t completedMoveCount,
    bool allowEmptyCuttingPrefix,
    const StockSimulationOptions& options,
    std::function<bool()> cancellationRequested,
    const DexelToolProfileDefinition* requestedToolProfile,
    bool orientedSweep) const
{
    MultiDexelApplyResult result;
    result.record.operationId = operation.operationId;
    result.record.sequenceIndex = sequenceIndex;
    result.record.volumeBefore = volume();
    result.record.volumeAfter = volume();
    if (!valid()) {
        result.record.status =
            StockRemovalStatus::InvalidStock;
        result.record.message = error();
        return result;
    }
    if (cancellationRequested &&
        cancellationRequested()) {
        result.record.status =
            StockRemovalStatus::Cancelled;
        result.record.message =
            "multi-dexel update was cancelled";
        return result;
    }

    const double cutterDiameter = requestedToolProfile &&
            std::isfinite(requestedToolProfile->diameter) &&
            requestedToolProfile->diameter > 0.0
        ? requestedToolProfile->diameter
        : params.tool.diameter;
    if (!std::isfinite(cutterDiameter) ||
        cutterDiameter <= m_impl->tolerance) {
        result.record.status =
            StockRemovalStatus::InvalidRemovalVolume;
        result.record.message =
            "multi-dexel pocket update requires a positive cutter diameter";
        return result;
    }
    const double radius = 0.5 * cutterDiameter;
    const double radiusSquared = radius * radius;
    const double numericTolerance =
        std::max(1.0e-12, radiusSquared * 1.0e-12);
    const double cuttingLength = requestedToolProfile &&
            std::isfinite(requestedToolProfile->cuttingLength) &&
            requestedToolProfile->cuttingLength > 0.0
        ? requestedToolProfile->cuttingLength
        : effectiveCuttingLength(
              *m_impl, operation, params,
              cutterDiameter);
    const SteadyClock::time_point validationStart =
        SteadyClock::now();
    std::vector<RasterMove> moves;
    std::vector<OrientedRasterMove> orientedMoves;
    LocalPoint operationAxis;
    std::string updateError;
    const bool prepared = orientedSweep
        ? prepareOrientedMoves(
              *m_impl, operation,
              completedMoveCount,
              cutterDiameter, cuttingLength,
              options, orientedMoves,
              operationAxis,
              result.planarLayerCount,
              result.nonPlanarMoveCount,
              updateError)
        : prepareMoves(
              *m_impl, operation,
              completedMoveCount,
              cutterDiameter,
              options, moves,
              result.planarLayerCount,
              result.nonPlanarMoveCount,
              updateError);
    if (!prepared) {
        result.record.inputValidationMilliseconds =
            elapsedMilliseconds(validationStart);
        const bool noPreparedMoves = orientedSweep
            ? orientedMoves.empty()
            : moves.empty();
        if (allowEmptyCuttingPrefix && noPreparedMoves) {
            result.stock =
                std::shared_ptr<const MultiDexelStock>(
                    new MultiDexelStock(m_impl));
            result.record.status =
                StockRemovalStatus::NoMaterialRemoved;
            result.record.message =
                "pocket movement prefix has not removed material";
            return result;
        }
        result.record.status =
            StockRemovalStatus::InvalidRemovalVolume;
        result.record.message =
            std::move(updateError);
        return result;
    }
    result.record.inputValidationMilliseconds =
        elapsedMilliseconds(validationStart);
    result.cuttingMoveCount =
        orientedSweep
        ? static_cast<int>(orientedMoves.size())
        : static_cast<int>(moves.size());
    result.record.sweepSegmentCount =
        result.cuttingMoveCount;
    result.record.planarSweepLayerCount =
        result.planarLayerCount;
    result.record.fallbackSweepSegmentCount =
        result.nonPlanarMoveCount;
    result.record.removalSolidCount = 1;

    const TileWorkPlan tileWork = orientedSweep
        ? buildTileWorkPlan(
              *m_impl,
              static_cast<int>(orientedMoves.size()),
              [&](int moveIndex)
                  -> const OrientedRasterMove& {
                  return orientedMoves[
                      static_cast<std::size_t>(moveIndex)];
              })
        : buildTileWorkPlan(
              *m_impl,
              static_cast<int>(moves.size()),
              [&](int moveIndex) -> const RasterMove& {
                  return moves[static_cast<std::size_t>(
                      moveIndex)];
              });
    result.touchedTileCount =
        static_cast<int>(
            tileWork.activeTiles.size());
    result.record.booleanBatchCount =
        result.touchedTileCount;

    const double cornerRadiusForTolerance =
        requestedToolProfile
        ? (std::isfinite(requestedToolProfile->cornerRadius)
               ? std::max(0.0, requestedToolProfile->cornerRadius)
               : 0.0)
        : (std::isfinite(params.tool.cornerRadius)
               ? std::max(0.0, params.tool.cornerRadius)
               : 0.0);
    const double profileAxialTolerance =
        std::isfinite(
            options.toolProfileAxialTolerance) &&
                options.toolProfileAxialTolerance >
                    m_impl->tolerance
        ? options.toolProfileAxialTolerance
        : std::max(
              std::clamp(
                  0.05 * cornerRadiusForTolerance,
                  0.05, 2.5),
              std::isfinite(options.arcChordTolerance)
              ? std::max(
                    0.0,
                    options.arcChordTolerance)
               : 0.0);
    const double curvedProfileHeight = std::min(
        cornerRadiusForTolerance, cuttingLength);
    const double defaultRadialTolerance =
        curvedProfileHeight > m_impl->tolerance
        ? std::max(
              profileAxialTolerance,
              std::sqrt(std::max(
                  0.0,
                  2.0 * cornerRadiusForTolerance *
                          std::min(
                              profileAxialTolerance,
                              curvedProfileHeight) -
                      std::pow(
                          std::min(
                              profileAxialTolerance,
                              curvedProfileHeight),
                          2.0))) +
                  m_impl->tolerance)
        : profileAxialTolerance;
    const double profileRadialTolerance =
        std::isfinite(
            options.toolProfileRadialTolerance) &&
                options.toolProfileRadialTolerance >
                    m_impl->tolerance
        ? options.toolProfileRadialTolerance
        : defaultRadialTolerance;
    DexelToolProfile toolProfile;
    std::string toolProfileError;
    DexelToolProfileDefinition profileDefinition =
        requestedToolProfile
        ? *requestedToolProfile
        : DexelToolProfileDefinition{
              DexelToolProfileFamily::RoundedEnd,
              cutterDiameter,
              cuttingLength,
              params.tool.cornerRadius,
              0.0};
    profileDefinition.diameter = cutterDiameter;
    profileDefinition.cuttingLength = cuttingLength;
    if (!buildDexelToolProfile(
            profileDefinition,
            {profileAxialTolerance,
             profileRadialTolerance,
             m_impl->tolerance,
             std::max(
                 1,
                 options.maxToolProfileBands)},
            toolProfile,
            &toolProfileError)) {
        result.record.inputValidationMilliseconds =
            elapsedMilliseconds(validationStart);
        result.record.status =
            StockRemovalStatus::InvalidRemovalVolume;
        result.record.message =
            std::move(toolProfileError);
        return result;
    }
    result.record.inputValidationMilliseconds =
        elapsedMilliseconds(validationStart);
    result.toolProfileBandCount =
        static_cast<int>(toolProfile.bands.size());
    result.maximumToolProfileAxialError =
        toolProfile.maximumAxialError;
    result.maximumToolProfileRadialError =
        toolProfile.maximumRadialError;
    OrientedSweepProfile orientedProfile;
    if (orientedSweep) {
        const double orientedFacetTolerance =
            std::isfinite(options.arcChordTolerance) &&
                    options.arcChordTolerance >
                        m_impl->tolerance
            ? std::min(
                  profileRadialTolerance,
                  options.arcChordTolerance)
            : profileRadialTolerance;
        orientedProfile = buildOrientedSweepProfile(
            operationAxis, toolProfile.maximumRadius,
            orientedFacetTolerance,
            options.maxOrientedToolFacets);
        if (orientedProfile.radialNormals.empty()) {
            result.record.status =
                StockRemovalStatus::InvalidRemovalVolume;
            result.record.message =
                "oriented multi-dexel tool profile has no radial facets";
            return result;
        }
        result.maximumToolProfileRadialError +=
            orientedProfile.maximumFacetError;
    }
    TileUpdateTransaction update;
    if (orientedSweep) {
        update = runTileUpdateTransaction(
            m_impl, tileWork.activeTiles,
            options.runParallel,
            cancellationRequested,
            [&](int tileIndex, Impl::Tile& tile) {
                return subtractOrientedMovesFromTile(
                    *m_impl, tile, orientedMoves,
                    tileWork.bins[
                        static_cast<std::size_t>(
                            tileIndex)],
                    operationAxis,
                    orientedProfile,
                    toolProfile,
                    numericTolerance,
                    cancellationRequested);
            });
    } else {
        update = runTileUpdateTransaction(
            m_impl, tileWork.activeTiles,
            options.runParallel,
            cancellationRequested,
            [&](int tileIndex, Impl::Tile& tile) {
                return subtractPocketMovesFromTile(
                    *m_impl, tile, moves,
                    tileWork.bins[
                        static_cast<std::size_t>(
                            tileIndex)],
                    toolProfile,
                    numericTolerance,
                    cancellationRequested);
            });
    }
    result.updateMilliseconds =
        update.milliseconds;
    result.record.booleanMilliseconds =
        result.updateMilliseconds;
    if (update.cancelled) {
        result.record.status =
            StockRemovalStatus::Cancelled;
        result.record.message =
            "multi-dexel update was cancelled";
        return result;
    }

    result.changedCellCount =
        update.changedCellCount;
    if (result.changedCellCount == 0) {
        result.stock =
            std::shared_ptr<const MultiDexelStock>(
                new MultiDexelStock(m_impl));
        result.record.status =
            StockRemovalStatus::NoMaterialRemoved;
        result.record.message =
            "pocket cutting moves did not remove any multi-dexel material";
        return result;
    }
    std::shared_ptr<Impl> next =
        std::move(update.stock);
    const double removed =
        std::max(0.0, volume() - next->volume);
    const double volumeTolerance =
        std::max(
            1.0e-8,
            1.0e-10 *
                std::max(1.0, volume()));
    if (removed <= volumeTolerance) {
        result.stock =
            std::shared_ptr<const MultiDexelStock>(
                new MultiDexelStock(m_impl));
        result.record.status =
            StockRemovalStatus::NoMaterialRemoved;
        result.record.message =
            "multi-dexel material change was below the volume tolerance";
        return result;
    }

    result.stock =
        std::shared_ptr<const MultiDexelStock>(
            new MultiDexelStock(std::move(next)));
    result.record.status =
        StockRemovalStatus::Applied;
    result.record.volumeAfter =
        result.stock->volume();
    result.record.removedVolume = removed;
    result.record.message = orientedSweep
        ? "material removal applied with oriented tiled multi-interval dexel"
        : "material removal applied with tiled multi-interval dexel";
    return result;
}

} // namespace oap::toolpath

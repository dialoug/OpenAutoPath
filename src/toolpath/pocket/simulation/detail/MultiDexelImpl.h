// Private MultiDexelImpl contract; not part of the module public API.
#pragma once

#include <algorithm>
#include <cmath>
#include <mutex>
#include "toolpath/pocket/simulation/detail/DexelCellIntervals.h"
#include "toolpath/pocket/simulation/detail/MultiDexelPrimitives.h"

namespace oap::toolpath {

class MultiDexelStock::Impl {
public:
    struct Cell {
        multi_dexel_detail::DexelCellIntervals intervals;
    };

    struct Tile {
        int iMin = 0;
        int jMin = 0;
        int width = 0;
        int height = 0;
        std::vector<Cell> cells;

        Cell& cell(int i, int j)
        {
            return cells[
                static_cast<std::size_t>(j - jMin) *
                    static_cast<std::size_t>(width) +
                static_cast<std::size_t>(i - iMin)];
        }

        const Cell& cell(int i, int j) const
        {
            return cells[
                static_cast<std::size_t>(j - jMin) *
                    static_cast<std::size_t>(width) +
                static_cast<std::size_t>(i - iMin)];
        }
    };

    gp_Pnt origin;
    gp_Vec uAxis;
    gp_Vec vAxis;
    gp_Vec wAxis;
    double uMin = 0.0;
    double uMax = 0.0;
    double vMin = 0.0;
    double vMax = 0.0;
    double resolution = 0.0;
    double tolerance = 1.0e-5;
    int nx = 0;
    int ny = 0;
    int tileSize = 32;
    int tilesX = 0;
    int tilesY = 0;
    int activeCellCount = 0;
    int intervalCount = 0;
    double volume = 0.0;
    bool valid = false;
    std::string error;
    struct FingerprintState {
        std::once_flag once;
        std::string value;
    };
    struct TileStatistics {
        int activeCells = 0;
        int intervals = 0;
        double volume = 0.0;
    };
    std::shared_ptr<FingerprintState> fingerprint = std::make_shared<FingerprintState>();
    std::vector<TileStatistics> tileStatistics;
    std::vector<std::shared_ptr<const Tile>> tiles;

    multi_dexel_detail::LocalPoint project(const gp_Pnt& point) const
    {
        const gp_Vec relative(origin, point);
        return {
            relative.Dot(uAxis),
            relative.Dot(vAxis),
            relative.Dot(wAxis)};
    }

    double cellU0(int i) const
    {
        return uMin + static_cast<double>(i) * resolution;
    }

    double cellU1(int i) const
    {
        return std::min(uMax, cellU0(i) + resolution);
    }

    double cellV0(int j) const
    {
        return vMin + static_cast<double>(j) * resolution;
    }

    double cellV1(int j) const
    {
        return std::min(vMax, cellV0(j) + resolution);
    }

    double cellU(int i) const
    {
        return 0.5 * (cellU0(i) + cellU1(i));
    }

    double cellV(int j) const
    {
        return 0.5 * (cellV0(j) + cellV1(j));
    }

    int coordinateToI(double u) const
    {
        return static_cast<int>(
            std::floor((u - uMin) / resolution));
    }

    int coordinateToJ(double v) const
    {
        return static_cast<int>(
            std::floor((v - vMin) / resolution));
    }

    int tileIndexForCell(int i, int j) const
    {
        return (j / tileSize) * tilesX +
               (i / tileSize);
    }

    const Cell& cell(int i, int j) const
    {
        const std::shared_ptr<const Tile>& tile =
            tiles[static_cast<std::size_t>(
                tileIndexForCell(i, j))];
        return tile->cell(i, j);
    }

    double cellArea(int i, int j) const
    {
        return (cellU1(i) - cellU0(i)) *
               (cellV1(j) - cellV0(j));
    }

    void refreshDerivedData();
    void refreshDerivedData(std::span<const int> changedTiles);
    TileStatistics statisticsForTile(std::size_t index) const;
    void sumTileStatistics();
    const std::string& contentFingerprint() const;
};

} // namespace oap::toolpath

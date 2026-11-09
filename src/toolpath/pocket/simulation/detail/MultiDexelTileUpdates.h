// Private MultiDexelTileUpdates contract; not part of the module public API.
#pragma once

#include <optional>
#include "toolpath/pocket/simulation/detail/MultiDexelImpl.h"
#include "toolpath/core/simulation/DexelToolProfile.h"

namespace oap::toolpath::multi_dexel_detail {

class CellRemovalAccumulator {
public:
    void add(
        const DexelInterval& removal,
        double tolerance);

    bool apply(
        DexelCellIntervals& material,
        double tolerance);

private:
    static bool touching(
        const DexelInterval& left,
        const DexelInterval& right,
        double tolerance);

    static void mergeInto(
        DexelInterval& target,
        const DexelInterval& source);

    std::optional<DexelInterval> m_primary;
    std::vector<DexelInterval> m_additional;
};

int subtractPocketMovesFromTile(
    const MultiDexelStock::Impl& grid,
    MultiDexelStock::Impl::Tile& tile,
    const std::vector<RasterMove>& moves,
    std::span<const int> moveIndices,
    const DexelToolProfile& toolProfile,
    double numericTolerance,
    const std::function<bool()>&
        cancellationRequested);

LocalPoint cross(
    const LocalPoint& left,
    const LocalPoint& right);

LocalPoint normalizedLocal(LocalPoint value);

} // namespace oap::toolpath::multi_dexel_detail

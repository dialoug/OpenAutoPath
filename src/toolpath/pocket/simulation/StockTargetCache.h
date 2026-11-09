// Session-owned, bounded cache for an immutable target; no global geometry state.
#pragma once
#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <TopoDS_Shape.hxx>
#include "toolpath/pocket/simulation/DexelDisplayMesh.h"

namespace oap::toolpath {
struct PocketZMapSimulationData;
class MultiDexelStock;
struct StockTargetHeightField {
    std::vector<double> bottoms;
    std::vector<double> tops;
    double volume = 0.0;
};
// Rasterizers fix mesh angle at .35 and derive deflection from resolution/tolerance.
using StockTargetGridKey = std::array<double, 21>;
StockTargetGridKey stockTargetGridKey(const PocketZMapSimulationData&, double tolerance);
StockTargetGridKey stockTargetGridKey(const MultiDexelStock&, double tolerance);

class StockTargetCache {
public:
    // Bounds must belong to this geometry in world coordinates. Geometry edits
    // replace the cache; OCCT triangulation changes do not change exact bounds.
    StockTargetCache(TopoDS_Shape shape, std::string revision,
        std::optional<std::array<double, 6>> exactBounds = {},
        std::size_t maximumRasterBytes = 256u * 1024u * 1024u);
    bool matches(const TopoDS_Shape&, const std::string& revision) const;
    bool matchesShape(const TopoDS_Shape&) const;
    bool bounds(std::array<double, 6>& output) const;
    std::shared_ptr<const StockTargetHeightField> heightField(const StockTargetGridKey&) const;
    std::shared_ptr<const DexelDisplayGrid> multiDexel(const StockTargetGridKey&) const;
    void store(const StockTargetGridKey&, const StockTargetHeightField&);
    void store(const StockTargetGridKey&, const DexelDisplayGrid&);
private:
    TopoDS_Shape m_shape;
    std::string m_revision;
    std::size_t m_maximumRasterBytes;
    mutable std::mutex m_mutex;
    mutable std::optional<std::array<double, 6>> m_bounds;
    StockTargetGridKey m_key{};
    // Only one raster is retained. Material checkpoints are independent.
    std::shared_ptr<const StockTargetHeightField> m_heightField;
    std::shared_ptr<const DexelDisplayGrid> m_multiDexel;
};
} // namespace oap::toolpath

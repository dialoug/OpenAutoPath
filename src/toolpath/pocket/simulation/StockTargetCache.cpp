#include "toolpath/pocket/simulation/StockTargetCache.h"
#include <algorithm>
#include <cmath>
#include <utility>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS_Iterator.hxx>
#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include "toolpath/pocket/simulation/PocketZMapComparison.h"

namespace oap::toolpath {
namespace {
bool sameShape(const TopoDS_Shape& a, const TopoDS_Shape& b)
{
    if (a.IsNull() || b.IsNull()) return false;
    if (a.IsEqual(b)) return true;
    // UI rebuilds wrapper compounds around indexed units. Iterator includes the
    // parent location/orientation; transformed/reversed units must never match.
    if (a.ShapeType() != TopAbs_COMPOUND || b.ShapeType() != TopAbs_COMPOUND) return false;
    TopoDS_Iterator left(a), right(b);
    for (; left.More() && right.More(); left.Next(), right.Next())
        if (!sameShape(left.Value(), right.Value())) return false;
    return !left.More() && !right.More();
}
bool validBounds(const std::array<double, 6>& b)
{
    return std::all_of(b.begin(), b.end(), [](double v) { return std::isfinite(v); }) &&
        b[0] <= b[3] && b[1] <= b[4] && b[2] <= b[5];
}
StockTargetGridKey gridKey(const gp_Pnt& origin, const gp_Vec& u, const gp_Vec& v,
    const gp_Vec& w, double uMin, double uMax, double vMin, double vMax,
    double resolution, int nx, int ny, double tolerance, double gridTolerance)
{
    return {origin.X(), origin.Y(), origin.Z(), u.X(), u.Y(), u.Z(),
        v.X(), v.Y(), v.Z(), w.X(), w.Y(), w.Z(), uMin, uMax, vMin, vMax,
        resolution, static_cast<double>(nx), static_cast<double>(ny), tolerance, gridTolerance};
}
}
StockTargetGridKey stockTargetGridKey(const PocketZMapSimulationData& g, double tolerance)
{
    return gridKey(g.origin, g.uAxis, g.vAxis, g.wAxis, g.uMin, g.uMax,
        g.vMin, g.vMax, g.resolution, g.nx, g.ny, tolerance, 0.0);
}
StockTargetGridKey stockTargetGridKey(const MultiDexelStock& g, double tolerance)
{
    return gridKey(g.origin(), g.uAxis(), g.vAxis(), g.wAxis(), g.uMin(), g.uMax(),
        g.vMin(), g.vMax(), g.resolution(), g.nx(), g.ny(), tolerance, g.geometricTolerance());
}
StockTargetCache::StockTargetCache(TopoDS_Shape shape, std::string revision,
    std::optional<std::array<double, 6>> exactBounds, std::size_t maximumRasterBytes)
    : m_shape(std::move(shape)), m_revision(std::move(revision)), m_maximumRasterBytes(maximumRasterBytes)
{
    if (exactBounds && validBounds(*exactBounds)) m_bounds = exactBounds;
}
bool StockTargetCache::matchesShape(const TopoDS_Shape& shape) const { return sameShape(m_shape, shape); }
bool StockTargetCache::matches(const TopoDS_Shape& shape, const std::string& revision) const
{
    return revision == m_revision && matchesShape(shape);
}
bool StockTargetCache::bounds(std::array<double, 6>& output) const
{
    std::lock_guard lock(m_mutex);
    if (!m_bounds) {
        try {
            Bnd_Box box;
            BRepBndLib::AddOptimal(m_shape, box, Standard_False, Standard_False);
            if (box.IsVoid()) return false;
            std::array<double, 6> b{};
            box.Get(b[0], b[1], b[2], b[3], b[4], b[5]);
            if (!validBounds(b)) return false;
            m_bounds = b;
        } catch (...) { return false; }
    }
    output = *m_bounds;
    return true;
}
std::shared_ptr<const StockTargetHeightField> StockTargetCache::heightField(const StockTargetGridKey& key) const
{
    std::lock_guard lock(m_mutex);
    return key == m_key ? m_heightField : nullptr;
}
std::shared_ptr<const DexelDisplayGrid> StockTargetCache::multiDexel(const StockTargetGridKey& key) const
{
    std::lock_guard lock(m_mutex);
    return key == m_key ? m_multiDexel : nullptr;
}
void StockTargetCache::store(const StockTargetGridKey& key, const StockTargetHeightField& field)
{
    std::lock_guard lock(m_mutex);
    m_heightField.reset(); m_multiDexel.reset();
    if ((field.bottoms.size() + field.tops.size()) * sizeof(double) > m_maximumRasterBytes) return;
    m_heightField = std::make_shared<const StockTargetHeightField>(field);
    m_key = key;
}
void StockTargetCache::store(const StockTargetGridKey& key, const DexelDisplayGrid& field)
{
    std::lock_guard lock(m_mutex);
    m_heightField.reset(); m_multiDexel.reset();
    if (field.offsets.size() * sizeof(field.offsets[0]) + field.intervals.size() * sizeof(DexelInterval) > m_maximumRasterBytes) return;
    m_multiDexel = std::make_shared<const DexelDisplayGrid>(field);
    m_key = key;
}
} // namespace oap::toolpath

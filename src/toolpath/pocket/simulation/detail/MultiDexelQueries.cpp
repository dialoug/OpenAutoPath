// Independently compiled private MultiDexelQueries implementation.
#include "toolpath/pocket/simulation/MultiDexelStock.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

#include <Precision.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include "toolpath/pocket/simulation/detail/MultiDexelImpl.h"
#include "toolpath/pocket/simulation/detail/MultiDexelPrimitives.h"

namespace oap::toolpath {
using namespace oap::toolpath::multi_dexel_detail;

bool MultiDexelStock::valid() const noexcept
{
    return m_impl && m_impl->valid;
}

const std::string& MultiDexelStock::error() const noexcept
{
    static const std::string empty;
    return m_impl ? m_impl->error : empty;
}

const gp_Pnt& MultiDexelStock::origin() const noexcept
{
    return m_impl->origin;
}

const gp_Vec& MultiDexelStock::uAxis() const noexcept
{
    return m_impl->uAxis;
}

const gp_Vec& MultiDexelStock::vAxis() const noexcept
{
    return m_impl->vAxis;
}

const gp_Vec& MultiDexelStock::wAxis() const noexcept
{
    return m_impl->wAxis;
}

double MultiDexelStock::uMin() const noexcept
{
    return m_impl->uMin;
}

double MultiDexelStock::uMax() const noexcept
{
    return m_impl->uMax;
}

double MultiDexelStock::vMin() const noexcept
{
    return m_impl->vMin;
}

double MultiDexelStock::vMax() const noexcept
{
    return m_impl->vMax;
}

double MultiDexelStock::resolution() const noexcept
{
    return m_impl->resolution;
}

int MultiDexelStock::nx() const noexcept
{
    return m_impl->nx;
}

int MultiDexelStock::ny() const noexcept
{
    return m_impl->ny;
}

int MultiDexelStock::tileSize() const noexcept
{
    return m_impl->tileSize;
}

int MultiDexelStock::tileCount() const noexcept
{
    return static_cast<int>(m_impl->tiles.size());
}

int MultiDexelStock::activeCellCount() const noexcept
{
    return m_impl->activeCellCount;
}

int MultiDexelStock::intervalCount() const noexcept
{
    return m_impl->intervalCount;
}

double MultiDexelStock::volume() const noexcept
{
    return m_impl->volume;
}

bool MultiDexelStock::axisAligned(
    const gp_Vec& outwardAxis,
    double angularTolerance) const noexcept
{
    if (!valid())
        return false;
    gp_Vec normalized = outwardAxis;
    if (!std::isfinite(normalized.X()) ||
        !std::isfinite(normalized.Y()) ||
        !std::isfinite(normalized.Z()) ||
        normalized.Magnitude() <=
            Precision::Confusion()) {
        return false;
    }
    normalized.Normalize();
    return normalized.Dot(m_impl->wAxis) >=
        1.0 - std::max(0.0, angularTolerance);
}

std::vector<DexelInterval>
MultiDexelStock::cellIntervals(
    int i,
    int j) const
{
    if (!valid() || i < 0 || j < 0 ||
        i >= m_impl->nx || j >= m_impl->ny) {
        return {};
    }
    const auto& intervals = m_impl->cell(i, j).intervals;
    return {intervals.begin(), intervals.end()};
}

std::span<const DexelInterval>
MultiDexelStock::cellIntervalsView(
    int i,
    int j) const noexcept
{
    if (!valid() || i < 0 || j < 0 ||
        i >= m_impl->nx || j >= m_impl->ny) {
        return {};
    }
    const auto& intervals =
        m_impl->cell(i, j).intervals;
    return {
        intervals.data(),
        intervals.size()};
}

int MultiDexelStock::sharedTileCountWith(
    const MultiDexelStock& other) const noexcept
{
    if (!valid() || !other.valid() ||
        m_impl->nx != other.m_impl->nx ||
        m_impl->ny != other.m_impl->ny ||
        m_impl->tileSize != other.m_impl->tileSize ||
        m_impl->tiles.size() !=
            other.m_impl->tiles.size()) {
        return 0;
    }
    int count = 0;
    for (std::size_t index = 0;
         index < m_impl->tiles.size(); ++index) {
        if (m_impl->tiles[index] ==
            other.m_impl->tiles[index]) {
            ++count;
        }
    }
    return count;
}

const std::string&
MultiDexelStock::contentFingerprint() const
{
    return m_impl->contentFingerprint();
}

double MultiDexelStock::geometricTolerance() const noexcept
{
    return m_impl->tolerance;
}


} // namespace oap::toolpath

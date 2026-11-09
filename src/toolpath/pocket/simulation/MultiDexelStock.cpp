// Immutable stock metadata and deterministic material fingerprint.
#include "toolpath/pocket/simulation/MultiDexelStock.h"

#include <bit>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include "toolpath/pocket/simulation/detail/MultiDexelImpl.h"

namespace oap::toolpath {
namespace {
class Fingerprint128 {
public:
    void add(std::uint64_t value) noexcept
    {
        for (int shift = 0; shift < 64; shift += 8) {
            const std::uint8_t byte =
                static_cast<std::uint8_t>(
                    (value >> shift) & 0xffu);
            m_first ^= byte;
            m_first *= 1099511628211ull;
            m_second += byte + 0x9e3779b97f4a7c15ull;
            m_second ^= m_second >> 29;
            m_second *= 0xbf58476d1ce4e5b9ull;
            m_second ^= m_second >> 32;
        }
    }

    void add(double value) noexcept
    {
        add(std::bit_cast<std::uint64_t>(value));
    }

    std::string finish() const
    {
        std::ostringstream output;
        output << std::hex << std::setfill('0')
               << std::setw(16) << m_first
               << std::setw(16) << m_second;
        return output.str();
    }

private:
    std::uint64_t m_first = 1469598103934665603ull;
    std::uint64_t m_second = 0x6a09e667f3bcc909ull;
};
}
void MultiDexelStock::Impl::refreshDerivedData()
{
    tileStatistics.resize(tiles.size());
    for (std::size_t index = 0; index < tiles.size(); ++index)
        tileStatistics[index] = statisticsForTile(index);
    sumTileStatistics();
}

MultiDexelStock::Impl::TileStatistics
MultiDexelStock::Impl::statisticsForTile(std::size_t index) const
{
    TileStatistics result;
    const auto& tile = *tiles[index];
    for (int j = tile.jMin; j < tile.jMin + tile.height; ++j) {
        for (int i = tile.iMin; i < tile.iMin + tile.width; ++i) {
            const auto& intervals = tile.cell(i, j).intervals;
            result.activeCells += !intervals.empty();
            result.intervals += static_cast<int>(intervals.size());
            for (const auto& interval : intervals)
                result.volume += (interval.upper - interval.lower) * cellArea(i, j);
        }
    }
    return result;
}

void MultiDexelStock::Impl::refreshDerivedData(std::span<const int> changedTiles)
{
    for (const int index : changedTiles)
        tileStatistics[index] = statisticsForTile(index);
    sumTileStatistics();
}

void MultiDexelStock::Impl::sumTileStatistics()
{
    activeCellCount = 0;
    intervalCount = 0;
    volume = 0.0;
    // Stable tile order keeps incremental/full construction statistics equal.
    for (const auto& stats : tileStatistics) {
        activeCellCount += stats.activeCells;
        intervalCount += stats.intervals;
        volume += stats.volume;
    }
    // COW children must never share the parent's lazy digest storage.
    fingerprint = std::make_shared<FingerprintState>();
}

const std::string& MultiDexelStock::Impl::contentFingerprint() const
{
    std::call_once(fingerprint->once, [this] {
        Fingerprint128 hash;
        hash.add(std::uint64_t{0x6d756c7469646578ull});
        hash.add(origin.X());
        hash.add(origin.Y());
        hash.add(origin.Z());
        hash.add(uAxis.X());
        hash.add(uAxis.Y());
        hash.add(uAxis.Z());
        hash.add(vAxis.X());
        hash.add(vAxis.Y());
        hash.add(vAxis.Z());
        hash.add(wAxis.X());
        hash.add(wAxis.Y());
        hash.add(wAxis.Z());
        hash.add(uMin);
        hash.add(uMax);
        hash.add(vMin);
        hash.add(vMax);
        hash.add(resolution);
        hash.add(static_cast<std::uint64_t>(nx));
        hash.add(static_cast<std::uint64_t>(ny));
        for (int j = 0; j < ny; ++j) {
            for (int i = 0; i < nx; ++i) {
                const Cell& current = cell(i, j);
                hash.add(static_cast<std::uint64_t>(
                    current.intervals.size()));
                for (const DexelInterval& interval :
                     current.intervals) {
                    hash.add(interval.lower);
                    hash.add(interval.upper);
                }
            }
        }
        fingerprint->value = hash.finish();
    });
    return fingerprint->value;
}
} // namespace oap::toolpath

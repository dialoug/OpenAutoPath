#include "FaceBoundsCache.h"

#include "analysis/ContentHash.h"
#include "support/Files.h"
#include <BinTools.hxx>
#include <Standard_Version.hxx>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <sstream>

namespace oap::detail {
namespace {
using Coordinates = std::array<double, 6>;
constexpr std::size_t digestLength = 64;
}

std::string faceBoundsKey(const TopTools_IndexedMapOfShape& faces)
{
    // A fresh STEP transfer can enumerate shapes differently. Fingerprint each
    // located face in order, including its trims, before reusing coordinates.
    std::string identity = "face-bounds-v1/" OCC_VERSION_STRING_EXT "/";
    try {
        for (int i = 1; i <= faces.Extent(); ++i) {
            std::ostringstream bytes(std::ios::out | std::ios::binary);
            BinTools::Write(faces(i), bytes, false, false, BinTools_FormatVersion_CURRENT);
            if (!bytes || bytes.tellp() <= 0) return {};
            identity += sha256Hex(bytes.view());
        }
    } catch (const Standard_Failure&) {
        return {};
    }
    return sha256Hex(identity);
}

bool readFaceBounds(const std::string& path, const std::string& key, std::vector<Bbox>& bounds)
{
    const auto size = digestLength + bounds.size() * sizeof(Coordinates);
    std::string bytes;
    if (!readFile(path, bytes, nullptr, size) || bytes.size() != size)
        return false;
    const auto payload = bytes.substr(digestLength);
    if (bytes.compare(0, digestLength, sha256Hex(key + payload)) != 0)
        return false;
    for (std::size_t i = 0; i < bounds.size(); ++i) {
        Coordinates c;
        std::memcpy(c.data(), payload.data() + i * sizeof(c), sizeof(c));
        bounds[i] = {c[0], c[1], c[2], c[3], c[4], c[5]};
    }
    return true;
}

void writeFaceBounds(const std::string& path, const std::string& key, const std::vector<Bbox>& bounds)
{
    std::string payload;
    payload.reserve(bounds.size() * sizeof(Coordinates));
    for (const auto& b : bounds) {
        Coordinates c{b.xmin, b.ymin, b.zmin, b.xmax, b.ymax, b.zmax};
        if (!b.valid() || !std::all_of(c.begin(), c.end(), [](double v) { return std::isfinite(v); }))
            return;
        payload.append(reinterpret_cast<const char*>(c.data()), sizeof(c));
    }
    writeFileAtomically(path, sha256Hex(key + payload) + payload);
}
}

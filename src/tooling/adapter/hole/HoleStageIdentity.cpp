#include "tooling/adapter/hole/HoleStageIdentity.h"
#include <algorithm>
#include <iomanip>
#include <locale>
#include <sstream>
namespace oap::tooling {
std::string holeStageKey(const oap::toolpath::HoleMachiningInput& hole)
{
    std::vector<std::string> ids = hole.stageWallFaceIds;
    if (ids.empty())
        ids = hole.sourceFaceIds;
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    if (!ids.empty()) {
        std::ostringstream value;
        value.imbue(std::locale::classic());
        value << "HOLE_STAGE";
        for (const std::string& id : ids)
            value << '|' << id;
        return value.str();
    }

    std::ostringstream value;
    value.imbue(std::locale::classic());
    value << std::fixed << std::setprecision(5)
          << "HOLE_STAGE_GEOM|" << hole.entry.X() << '|'
          << hole.entry.Y() << '|' << hole.entry.Z() << '|'
          << hole.bottom.X() << '|' << hole.bottom.Y() << '|'
          << hole.bottom.Z() << '|' << hole.radius;
    return value.str();
}

}

// Private PocketZMapDisplayState contract; not part of the module public API.
#pragma once

#include "toolpath/pocket/simulation/detail/PocketZMapCoreSupport.h"

namespace oap::toolpath {

class PocketZMapDisplaySource::Impl {
public:
    std::shared_ptr<const PocketZMapSimulationData> data;
    zmap_detail::TargetField target;
    std::shared_ptr<const MultiDexelComparisonData>
        multiDexelData;
    DexelDisplayGrid multiDexelTarget;
    DexelDisplayGrid multiDexelInitialMissing;
    DexelDisplayGrid multiDexelRemainingAllowance;
    DexelDisplayGrid multiDexelOvercut;
    double tolerance = 1.0e-5;
    std::string fingerprint;
};

} // namespace oap::toolpath

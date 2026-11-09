// Profile execution view for feature-independent fixed-axis material removal.
#pragma once

#include "toolpath/core/simulation/FixedAxisToolpathView.h"
#include "toolpath/profile/program/ProfileToolpathOperation.h"

namespace oap::toolpath {

// Borrows the source moves. The operation must outlive synchronous consumption.
FixedAxisToolpathView profileSimulationView(
    const ProfileToolpathOperation& operation);
FixedAxisToolpathView profileSimulationView(ProfileToolpathOperation&&) = delete;
FixedAxisToolpathView profileSimulationView(const ProfileToolpathOperation&&) = delete;

} // namespace oap::toolpath

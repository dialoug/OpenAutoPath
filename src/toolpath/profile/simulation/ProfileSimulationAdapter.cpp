#include "toolpath/profile/simulation/ProfileSimulationAdapter.h"

namespace oap::toolpath {

FixedAxisToolpathView profileSimulationView(
    const ProfileToolpathOperation& operation)
{
    return {operation.operationId, operation.outwardToolAxis,
            operation.contour.moves};
}

} // namespace oap::toolpath

// Private operationorder stage contract.
#pragma once

#include "manufacturing/hole/detail/HoleJobTypes.h"

namespace oap::manufacturing::hole_job_detail {

void restoreStableHoleIdentityAndGlobalOrder(
    MachiningJobDefinition& definition,
    const std::vector<StableHoleOperationIdentity>& previous);

} // namespace oap::manufacturing::hole_job_detail

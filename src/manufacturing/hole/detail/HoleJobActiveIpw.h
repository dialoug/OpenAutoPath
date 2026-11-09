// Private activeipw stage contract.
#pragma once

#include "manufacturing/hole/detail/HoleJobTypes.h"

namespace oap::manufacturing::hole_job_detail {

ActiveIpwPlan captureActiveIpwPlan(const MachiningJob& job);

bool restoreActiveIpwPlan(MachiningJob& job, const ActiveIpwPlan& saved);

} // namespace oap::manufacturing::hole_job_detail

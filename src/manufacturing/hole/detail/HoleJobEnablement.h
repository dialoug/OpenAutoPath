// Private enablement stage contract.
#pragma once

#include "manufacturing/hole/detail/HoleJobTypes.h"

namespace oap::manufacturing::hole_job_detail {


JobOperationState generatedState(
    const toolpath::HoleToolpathBatch& batch);

void addAutomaticDefinitions(
    MachiningJobDefinition& definition,
    const HoleJobCaptureRequest& request,
    JobOperationState state);


} // namespace oap::manufacturing::hole_job_detail

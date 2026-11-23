#pragma once

#include "support/Json.h"
#include "manufacturing/ipw/IpwExecutor.h"

namespace oap::worker
{
oap::JsonObject replayResult(const manufacturing::MachiningJob &job, bool selectedOnly, int quality);
oap::JsonObject stockResult(const manufacturing::MachiningJob &job,
                        const manufacturing::ipw::IpwExecutionResult &simulation, const oap::JsonObject &request,
                        const TopoDS_Shape &target);
}

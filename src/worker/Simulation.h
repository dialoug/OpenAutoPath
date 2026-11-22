#pragma once

#include "manufacturing/ipw/IpwExecutor.h"
#include <QJsonObject>

namespace oap::worker
{
QJsonObject replayResult(const manufacturing::MachiningJob &job, bool selectedOnly);
QJsonObject stockResult(const manufacturing::MachiningJob &job,
                        const manufacturing::ipw::IpwExecutionResult &simulation, const QJsonObject &request,
                        const TopoDS_Shape &target);
}

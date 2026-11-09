// Legacy BRep/display bridge owned by oap_toolpath_workflow, which may compose
// both feature domains. Native IPW updates use profileSimulationView instead.
// 固定轴材料去除内核消费中立的 moves/primitives，但其历史入口类型为
// PocketToolpathOperation；本适配只翻译运行期表示，不改变 Profile 语义。
#pragma once

#include <vector>

#include "toolpath/pocket/program/PocketToolpathOperation.h"
#include "toolpath/profile/program/ProfileToolpathOperation.h"

namespace oap::toolpath {

// 固定轴材料去除内核消费中立的 moves/primitives，但其历史入口类型仍为
// PocketToolpathOperation。该适配只翻译运行期表示，不改变 Profile 语义。
PocketToolpathOperation legacyProfileSimulationOperation(
    const ProfileToolpathOperation& operation);

std::vector<PocketToolpathOperation> legacyProfileSimulationOperations(
    const std::vector<ProfileToolpathOperation>& operations);

} // namespace oap::toolpath

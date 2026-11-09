// 孔加工运行时工序到固定轴 OpenGL/Multi-Dexel 模拟载体的只读适配。
#pragma once

#include <vector>

#include "toolpath/hole/HoleMachining.h"
#include "toolpath/pocket/program/PocketToolpathOperation.h"

namespace oap::toolpath {

// 现有固定轴模拟内核历史上使用 PocketToolpathOperation 作为中立移动载体。
// 本适配保留孔工序的真实 moves/primitives、刀具快照和装夹刀轴，不改变
// HoleMachiningOperation 的生产语义。
PocketToolpathOperation holeSimulationOperation(
    const HoleMachiningOperation& operation);

std::vector<PocketToolpathOperation> holeSimulationOperations(
    const std::vector<HoleMachiningOperation>& operations);

} // namespace oap::toolpath

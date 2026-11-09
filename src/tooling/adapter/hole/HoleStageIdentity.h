#pragma once
#include <string>
#include "toolpath/hole/HoleMachiningInput.h"
namespace oap::tooling {
// Stable stage identity for automatic plans and persisted target rebinding.
std::string holeStageKey(const oap::toolpath::HoleMachiningInput& hole);
}

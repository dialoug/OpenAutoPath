#pragma once

#include <string>

namespace oap {

std::string randomUuid();
std::string utcTimestamp(bool milliseconds = true);
} // namespace oap

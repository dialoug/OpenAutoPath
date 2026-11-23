#pragma once
#include "support/Json.h"
#include <string>

namespace oap::worker {
oap::JsonObject reviewNc(const std::string &text, const std::string &filename, const std::string &setup,
                     const std::string &tool, const oap::JsonArray &operations, oap::JsonArray warnings);
void writeNcReport(const oap::JsonArray &programs);
int reviewSingleNc(const std::string &setup, const std::string &tool, const oap::JsonArray &operations, oap::JsonArray warnings = {});
}

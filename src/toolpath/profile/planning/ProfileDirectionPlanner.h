// Conservative setup reduction for constant polygonal external envelopes.
#pragma once

#include <functional>
#include <string>
#include <vector>

#include "toolpath/profile/geometry/ProfileRegion.h"

namespace oap::toolpath {

struct ProfileDirectionPlanningOptions {
    // Only full feature depth and world-aligned per-unit box stock are supported.
    bool allowReduction = true;
    gp_Vec stockMargins{0, 0, 0};
    std::function<bool(const ProfileRegion&)> hasCandidateTool;
    std::function<bool()> cancellationRequested;
};

struct ProfileDirectionCoverage {
    std::size_t selectedRegion = 0;
    std::vector<std::size_t> coveredRegions;
};

struct ProfileDirectionPlan {
    std::vector<std::size_t> selectedRegionIndices;
    std::vector<ProfileDirectionCoverage> coverage;
    std::vector<std::string> retainedUnitIds;
    bool complete = false;
    bool cancelled = false;
};

// Input order breaks equal-depth ties (the application supplies +Z first).
// Unproven units retain every input region. This does not replace toolpath
// collision validation, nor authorize a partial program when generation fails.
class ProfileDirectionPlanner {
public:
    ProfileDirectionPlan plan(
        const std::vector<ProfileRegion>& regions,
        const ProfileDirectionPlanningOptions& options = {}) const;
};

} // namespace oap::toolpath

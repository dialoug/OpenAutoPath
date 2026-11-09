// Private profile motion stage: consumes generated contour loops, without
// rebuilding sections, offsets or clearing geometry.
#pragma once

namespace oap::toolpath {

struct ProfileRegion;
struct ToolpathParams;
struct ProfileContourResult;

namespace profile_motion_detail {

// Reorders only eligible rest components and rotates their entry points.
// Updates loop sequences and travel diagnostics; leaves layer/pass order intact.
void optimizeRestMachiningTravel(
    const ProfileRegion& region,
    const ToolpathParams& params,
    ProfileContourResult& result);

// Evaluates the ordered loops without creating executable moves. Accumulates
// direct-link/re-entry counters using the same safety policy as assembly.
double estimateCuttingDistance(
    ProfileContourResult& result,
    const ProfileRegion& region,
    const ToolpathParams& params);

// Appends executable moves/primitives and link diagnostics for ordered loops.
// The caller owns result initialization, counter reset and stage timing.
void assembleMoves(
    ProfileContourResult& result,
    const ProfileRegion& region,
    const ToolpathParams& params);

} // namespace profile_motion_detail
} // namespace oap::toolpath

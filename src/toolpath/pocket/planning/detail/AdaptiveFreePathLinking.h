#pragma once

#include <functional>
#include <optional>
#include <vector>

#include <gp_Pnt2d.hxx>

namespace oap::toolpath::detail {

// Port of pyocc-exp/cam_exp/linking.py.  The predicate owns production safety
// checks (safe cutter centre, cleared material, and any strategy constraints).
using AdaptiveLinkPredicate = std::function<bool(
    const std::vector<gp_Pnt2d>&, double exitRadius, double entryRadius)>;

std::optional<std::vector<gp_Pnt2d>> adaptiveCscLinkCandidate(
    const gp_Pnt2d& start,
    const gp_Pnt2d& end,
    double startHeading,
    double endHeading,
    double exitRadius,
    double entryRadius,
    int exitDirection,
    int entryDirection,
    double sampleSpacing);

std::optional<std::vector<gp_Pnt2d>> bestAdaptiveCscLink(
    const gp_Pnt2d& start,
    const gp_Pnt2d& end,
    double startHeading,
    double endHeading,
    double targetExitRadius,
    double targetEntryRadius,
    double minimumRadius,
    double sampleSpacing,
    const AdaptiveLinkPredicate& candidateIsValid);

} // namespace oap::toolpath::detail

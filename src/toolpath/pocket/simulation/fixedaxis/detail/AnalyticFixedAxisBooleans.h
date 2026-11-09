// Private booleans stage contract.
#pragma once

#include <unordered_set>

#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisTypes.h"

namespace oap::toolpath::analytic_fixed_axis_detail {

bool translatedFace(
    const TopoDS_Face& source,
    const FixedAxisFrame& frame,
    double axial,
    TopoDS_Face& output);

int wireCount(const TopoDS_Face& face);

bool buildClosedOffsetFillRecords(
    const PocketToolpathOperation& operation,
    const ToolpathParams& params,
    const FixedAxisFrame& frame,
    double cuttingLength,
    double stockMinimum,
    double stockMaximum,
    double tolerance,
    double nestingTolerance,
    std::unordered_set<int>& retainedLoopSequences,
    std::vector<AnalyticSweepRecord>& fillRecords,
    double& maximumBoundaryFitError,
    std::string* rejectionReason);

} // namespace oap::toolpath::analytic_fixed_axis_detail

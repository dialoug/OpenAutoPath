// Private sweeps stage contract.
#pragma once

#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisTypes.h"

namespace oap::toolpath::analytic_fixed_axis_detail {

bool linearRampCoveredByPlanarCuts(
    const AnalyticLinearRampGroup& group,
    const std::vector<AnalyticPlanarCut>& cuts,
    const FixedAxisFrame& frame,
    double cuttingLength,
    double stockMinimum,
    double stockMaximum,
    double tolerance);

bool helicalRampCoveredByPlanarCuts(
    const AnalyticHelicalRampGroup& group,
    const std::vector<AnalyticPlanarCoverage>& cuts,
    const FixedAxisFrame& frame,
    double cuttingLength,
    double stockMinimum,
    double stockMaximum,
    double tolerance);

bool appendLinearRampRecords(
    const AnalyticLinearRampGroup& group,
    const FixedAxisFrame& frame,
    const gp_Ax3& referencePlane,
    const AnalyticPlanarSweep& planarSweep,
    double cutterRadius,
    double cuttingLength,
    double stockMinimum,
    double stockMaximum,
    double axialStep,
    double tolerance,
    int maximumSlices,
    const std::function<bool()>& cancellationRequested,
    std::vector<AnalyticSweepRecord>& records,
    AnalyticFixedAxisLayeredBRepBuildResult& result);

bool appendHelicalRampRecords(
    const AnalyticHelicalRampGroup& group,
    const FixedAxisFrame& frame,
    const gp_Ax3& referencePlane,
    const AnalyticPlanarSweep& planarSweep,
    double cutterRadius,
    double cuttingLength,
    double stockMinimum,
    double stockMaximum,
    double axialStep,
    double tolerance,
    int maximumSlices,
    const std::function<bool()>& cancellationRequested,
    std::vector<AnalyticSweepRecord>& records,
    AnalyticFixedAxisLayeredBRepBuildResult& result);

} // namespace oap::toolpath::analytic_fixed_axis_detail

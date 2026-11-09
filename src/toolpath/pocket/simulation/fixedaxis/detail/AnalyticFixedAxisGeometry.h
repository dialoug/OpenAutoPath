// Private geometry stage contract.
#pragma once

#include "toolpath/pocket/simulation/fixedaxis/detail/AnalyticFixedAxisTypes.h"

namespace oap::toolpath::analytic_fixed_axis_detail {

bool cancelled(const std::function<bool()>& requested);

double elapsedMilliseconds(const Clock::time_point& start);

bool makeFrame(
    const gp_Vec& sourceAxis,
    double tolerance,
    FixedAxisFrame& frame);

bool stockAxialBounds(
    const TopoDS_Shape& stock,
    const FixedAxisFrame& frame,
    double& minimum,
    double& maximum);

std::string primitiveGeometryKey(
    const ToolpathPrimitive& primitive,
    const FixedAxisFrame& frame);

std::string projectedCoverageKey(
    const ToolpathPrimitive& primitive,
    const FixedAxisFrame& frame);

gp_Pnt interpolateAtAxial(
    const gp_Pnt& from,
    const gp_Pnt& to,
    double fromW,
    double toW,
    double axial);

AnalyticLinearRampGroup& findOrCreateLinearRampGroup(
    std::vector<AnalyticLinearRampGroup>& groups,
    const ToolpathMove& move);

AnalyticHelicalRampGroup& findOrCreateHelicalRampGroup(
    std::vector<AnalyticHelicalRampGroup>& groups,
    const ToolpathPrimitiveContext& context);

std::pair<double, double> rotated(
    const std::pair<double, double>& vector,
    double angle);

bool makePlanarHelicalSlice(
    const ToolpathPrimitive& primitive,
    const FixedAxisFrame& frame,
    double startFraction,
    double endFraction,
    double tolerance,
    ToolpathPrimitive& slice);

bool projectedSegmentContains(
    const AnalyticPlanarCut& cut,
    const std::pair<double, double>& from,
    const std::pair<double, double>& to,
    double tolerance);

bool intervalCovered(
    std::vector<std::pair<double, double>> intervals,
    double requiredBottom,
    double requiredTop,
    double tolerance);

std::vector<AnalyticSlab> makeSlabs(
    const std::vector<AnalyticSweepRecord>& records,
    double tolerance);

} // namespace oap::toolpath::analytic_fixed_axis_detail

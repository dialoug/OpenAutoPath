// Private stage data shared only within this target.
#pragma once

#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax3.hxx>
#include <gp_Vec.hxx>
#include "toolpath/pocket/simulation/fixedaxis/AnalyticFixedAxisLayeredBRepBuilder.h"
#include "toolpath/pocket/simulation/fixedaxis/AnalyticPlanarSweep.h"

namespace oap::toolpath::analytic_fixed_axis_detail {

using Clock = std::chrono::steady_clock;

struct FixedAxisFrame {
    gp_Pnt origin;
    gp_Dir u;
    gp_Dir v;
    gp_Dir w;

    double axial(const gp_Pnt& point) const
    {
        return gp_Vec(origin, point).Dot(gp_Vec(w));
    }

    double planarDistance(
        const gp_Pnt& first,
        const gp_Pnt& second) const
    {
        const gp_Vec delta(first, second);
        const double x = delta.Dot(gp_Vec(u));
        const double y = delta.Dot(gp_Vec(v));
        return std::hypot(x, y);
    }

    std::pair<double, double> project(
        const gp_Pnt& point) const
    {
        const gp_Vec delta(origin, point);
        return {
            delta.Dot(gp_Vec(u)),
            delta.Dot(gp_Vec(v))};
    }

    gp_Pnt point(
        const std::pair<double, double>& planar,
        double axialValue = 0.0) const
    {
        gp_Pnt result = origin;
        result.Translate(
            gp_Vec(u).Multiplied(planar.first) +
            gp_Vec(v).Multiplied(planar.second) +
            gp_Vec(w).Multiplied(axialValue));
        return result;
    }

    gp_Ax3 referencePlane() const
    {
        return gp_Ax3(origin, w, u);
    }
};

struct AnalyticSweepRecord {
    double bottom = 0.0;
    double top = 0.0;
    TopoDS_Face face;
    std::string geometryKey;
    TopoDS_Wire centerWire;
    double sourceTip = 0.0;
};

struct AnalyticLinearRampGroup {
    int loopSequence = -1;
    int depthIndex = 0;
    int ring = 0;
    double minimumTip = std::numeric_limits<double>::infinity();
    double maximumTip = -std::numeric_limits<double>::infinity();
    std::vector<ToolpathPrimitive> segments;
};

struct AnalyticHelicalRampGroup {
    int loopSequence = -1;
    int depthIndex = 0;
    int ring = 0;
    double minimumTip = std::numeric_limits<double>::infinity();
    double maximumTip = -std::numeric_limits<double>::infinity();
    int moveCount = 0;
    std::vector<ToolpathPrimitive> arcs;
};

struct AnalyticPlanarCut {
    std::pair<double, double> from;
    std::pair<double, double> to;
    double bottom = 0.0;
    double top = 0.0;
};

struct AnalyticPlanarCoverage {
    std::string geometryKey;
    double bottom = 0.0;
    double top = 0.0;
};

struct AnalyticSlab {
    double bottom = 0.0;
    double top = 0.0;
    std::vector<std::size_t> activeRecords;
    std::vector<std::string> geometrySignature;
};

} // namespace oap::toolpath::analytic_fixed_axis_detail

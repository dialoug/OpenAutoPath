// Private MultiDexelPrimitives contract; not part of the module public API.
#pragma once

#include <chrono>
#include <vector>
#include "toolpath/pocket/simulation/MultiDexelStock.h"

namespace oap::toolpath::multi_dexel_detail {

using SteadyClock = std::chrono::steady_clock;

double elapsedMilliseconds(const SteadyClock::time_point& start);

double resolutionForCellBudget(
    double uSpan,
    double vSpan,
    int maximumCellCount);

struct LocalPoint {
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
};

struct Crossing {
    double w = 0.0;
    int sign = 0;
};

struct RasterMove {
    double u0 = 0.0;
    double v0 = 0.0;
    double w0 = 0.0;
    double u1 = 0.0;
    double v1 = 0.0;
    double w1 = 0.0;
    int iMin = 0;
    int iMax = -1;
    int jMin = 0;
    int jMax = -1;
};

struct OrientedRasterMove {
    LocalPoint from;
    LocalPoint to;
    int iMin = 0;
    int iMax = -1;
    int jMin = 0;
    int jMax = -1;
};

struct SweepParameterPoint {
    double w = 0.0;
    double t = 0.0;
};

double dot(const LocalPoint& left, const LocalPoint& right);

LocalPoint difference(
    const LocalPoint& to,
    const LocalPoint& from);

bool finitePoint(const gp_Pnt& point);

bool cuttingMove(const ToolpathMove& move);

gp_Vec normalizedAxis(
    const gp_Vec& source,
    double tolerance);

double exactShapeVolume(const TopoDS_Shape& shape);

gp_Pnt localToWorld(
    const gp_Pnt& origin,
    const gp_Vec& uAxis,
    const gp_Vec& vAxis,
    const gp_Vec& wAxis,
    double u,
    double v,
    double w);

} // namespace oap::toolpath::multi_dexel_detail

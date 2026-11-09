// Independently compiled private MultiDexelRasterization implementation.
#include "toolpath/pocket/simulation/detail/MultiDexelRasterization.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Precision.hxx>
#include <gp_Vec.hxx>

namespace oap::toolpath::multi_dexel_detail {

std::vector<DexelInterval> intervalsFromCrossings(
    std::vector<Crossing>& crossings,
    double tolerance)
{
    std::vector<DexelInterval> intervals;
    intervalsFromCrossings(crossings, tolerance, intervals);
    return intervals;
}

void intervalsFromCrossings(
    std::vector<Crossing>& crossings,
    double tolerance,
    std::vector<DexelInterval>& intervals)
{
    intervals.clear();
    if (crossings.size() < 2)
        return;
    std::sort(
        crossings.begin(), crossings.end(),
        [](const Crossing& left, const Crossing& right) {
            return left.w < right.w;
        });

    std::size_t consolidatedCount = 0;
    for (std::size_t begin = 0;
         begin < crossings.size();) {
        std::size_t end = begin + 1;
        while (end < crossings.size() &&
               std::abs(
                   crossings[end].w -
                   crossings[begin].w) <= tolerance) {
            ++end;
        }
        bool positive = false;
        bool negative = false;
        double average = 0.0;
        for (std::size_t index = begin;
             index < end; ++index) {
            positive = positive || crossings[index].sign > 0;
            negative = negative || crossings[index].sign < 0;
            average += crossings[index].w;
        }
        int sign = 0;
        if (positive != negative)
            sign = positive ? 1 : -1;
        if (sign != 0) {
            crossings[consolidatedCount++] = {
                average / static_cast<double>(end - begin),
                sign};
        }
        begin = end;
    }
    crossings.resize(consolidatedCount);

    int winding = 0;
    std::optional<double> lower;
    for (const Crossing& crossing : crossings) {
        const int before = winding;
        winding += crossing.sign;
        if (before == 0 && winding != 0) {
            lower = crossing.w;
        } else if (before != 0 && winding == 0 &&
                   lower &&
                   crossing.w - *lower > tolerance) {
            intervals.push_back(
                {*lower, crossing.w});
            lower.reset();
        }
    }
}

void rasterizeTriangle(
    const MultiDexelStock::Impl& grid,
    const LocalPoint& a,
    const LocalPoint& b,
    const LocalPoint& c,
    int orientationSign,
    std::vector<std::vector<Crossing>>& crossings)
{
    const double denominator =
        (b.v - c.v) * (a.u - c.u) +
        (c.u - b.u) * (a.v - c.v);
    const double projectedScale = std::max({
        std::abs(a.u), std::abs(a.v),
        std::abs(b.u), std::abs(b.v),
        std::abs(c.u), std::abs(c.v), 1.0});
    if (std::abs(denominator) <=
        std::numeric_limits<double>::epsilon() *
            projectedScale * projectedScale * 32.0) {
        return;
    }
    int iMin = std::clamp(
        grid.coordinateToI(
            std::min({a.u, b.u, c.u})) - 1,
        0, grid.nx - 1);
    int iMax = std::clamp(
        grid.coordinateToI(
            std::max({a.u, b.u, c.u})) + 1,
        0, grid.nx - 1);
    int jMin = std::clamp(
        grid.coordinateToJ(
            std::min({a.v, b.v, c.v})) - 1,
        0, grid.ny - 1);
    int jMax = std::clamp(
        grid.coordinateToJ(
            std::max({a.v, b.v, c.v})) + 1,
        0, grid.ny - 1);
    constexpr double barycentricTolerance = 1.0e-10;
    for (int j = jMin; j <= jMax; ++j) {
        const double v = grid.cellV(j);
        for (int i = iMin; i <= iMax; ++i) {
            const double u = grid.cellU(i);
            const double alpha =
                ((b.v - c.v) * (u - c.u) +
                 (c.u - b.u) * (v - c.v)) /
                denominator;
            const double beta =
                ((c.v - a.v) * (u - c.u) +
                 (a.u - c.u) * (v - c.v)) /
                denominator;
            const double gamma = 1.0 - alpha - beta;
            if (alpha < -barycentricTolerance ||
                beta < -barycentricTolerance ||
                gamma < -barycentricTolerance) {
                continue;
            }
            const double w =
                alpha * a.w + beta * b.w + gamma * c.w;
            crossings[
                static_cast<std::size_t>(j) *
                    static_cast<std::size_t>(grid.nx) +
                static_cast<std::size_t>(i)]
                .push_back({w, orientationSign});
        }
    }
}

double effectiveCuttingLength(
    const MultiDexelStock::Impl& grid,
    const FixedAxisToolpathView& operation,
    const ToolpathParams& params,
    double cutterDiameter)
{
    if (std::isfinite(params.tool.cuttingEdgeLength) &&
        params.tool.cuttingEdgeLength > 0.0) {
        return params.tool.cuttingEdgeLength;
    }
    double minimum =
        std::numeric_limits<double>::infinity();
    double maximum =
        -std::numeric_limits<double>::infinity();
    for (const ToolpathMove& move :
         operation.moves) {
        if (!cuttingMove(move) ||
            !finitePoint(move.from) ||
            !finitePoint(move.to)) {
            continue;
        }
        const LocalPoint from = grid.project(move.from);
        const LocalPoint to = grid.project(move.to);
        minimum = std::min(
            minimum, std::min(from.w, to.w));
        maximum = std::max(
            maximum, std::max(from.w, to.w));
    }
    const double span =
        std::isfinite(minimum) && std::isfinite(maximum)
        ? std::max(0.0, maximum - minimum)
        : 0.0;
    return std::max(cutterDiameter, span + cutterDiameter);
}

bool prepareMoves(
    const MultiDexelStock::Impl& grid,
    const FixedAxisToolpathView& operation,
    std::size_t completedMoveCount,
    double cutterDiameter,
    const StockSimulationOptions& options,
    std::vector<RasterMove>& moves,
    int& planarLayerCount,
    int& nonPlanarMoveCount,
    std::string& error)
{
    const double tolerance = std::max(
        options.geometricTolerance,
        Precision::Confusion());
    gp_Vec operationAxis =
        normalizedAxis(operation.outwardToolAxis, tolerance);
    if (operationAxis.Magnitude() <= tolerance) {
        for (const ToolpathMove& move :
             operation.moves) {
            if (move.type != ToolpathMoveType::Plunge)
                continue;
            operationAxis = normalizedAxis(
                gp_Vec(move.to, move.from), tolerance);
            if (operationAxis.Magnitude() > tolerance)
                break;
        }
    }
    if (operationAxis.Magnitude() <= tolerance ||
        operationAxis.Dot(grid.wAxis) < 1.0 - 1.0e-6) {
        error =
            "multi-dexel operation axis differs from the checkpoint grid";
        return false;
    }

    const int segmentBudget = std::clamp(
        options.maxSweepSegments, 1, 5000000);
    const double radius = 0.5 * cutterDiameter;
    std::vector<std::pair<int, double>> planarLayers;
    moves.reserve(std::min(
        static_cast<std::size_t>(segmentBudget),
        operation.moves.size()));
    const std::size_t moveLimit =
        std::min(
            completedMoveCount,
            operation.moves.size());
    for (std::size_t moveIndex = 0;
         moveIndex < moveLimit; ++moveIndex) {
        const ToolpathMove& move =
            operation.moves[moveIndex];
        if (!cuttingMove(move))
            continue;
        if (static_cast<int>(moves.size()) >=
            segmentBudget) {
            error =
                "multi-dexel cutting move budget exceeded";
            return false;
        }
        if (!finitePoint(move.from) ||
            !finitePoint(move.to)) {
            error =
                "multi-dexel cutting move contains a non-finite point";
            return false;
        }
        const LocalPoint from = grid.project(move.from);
        const LocalPoint to = grid.project(move.to);
        RasterMove raster;
        raster.u0 = from.u;
        raster.v0 = from.v;
        raster.w0 = from.w;
        raster.u1 = to.u;
        raster.v1 = to.v;
        raster.w1 = to.w;
        raster.iMin = std::clamp(
            grid.coordinateToI(
                std::min(from.u, to.u) - radius) - 1,
            0, grid.nx - 1);
        raster.iMax = std::clamp(
            grid.coordinateToI(
                std::max(from.u, to.u) + radius) + 1,
            0, grid.nx - 1);
        raster.jMin = std::clamp(
            grid.coordinateToJ(
                std::min(from.v, to.v) - radius) - 1,
            0, grid.ny - 1);
        raster.jMax = std::clamp(
            grid.coordinateToJ(
                std::max(from.v, to.v) + radius) + 1,
            0, grid.ny - 1);
        if (raster.iMin <= raster.iMax &&
            raster.jMin <= raster.jMax) {
            moves.push_back(raster);
        }

        const double axialTravel =
            std::abs(to.w - from.w);
        const double radialTravel =
            std::hypot(to.u - from.u, to.v - from.v);
        if (axialTravel <= tolerance &&
            radialTravel > tolerance) {
            const double layerW =
                0.5 * (from.w + to.w);
            const bool known = std::any_of(
                planarLayers.begin(),
                planarLayers.end(),
                [&](const auto& layer) {
                    return layer.first == move.depthIndex &&
                           std::abs(layer.second - layerW) <=
                               tolerance;
                });
            if (!known) {
                planarLayers.emplace_back(
                    move.depthIndex, layerW);
            }
        } else {
            ++nonPlanarMoveCount;
        }
    }
    planarLayerCount =
        static_cast<int>(planarLayers.size());
    if (moves.empty()) {
        error =
            "pocket operation contains no cutting moves inside the multi-dexel grid";
        return false;
    }
    return true;
}

bool prepareOrientedMoves(
    const MultiDexelStock::Impl& grid,
    const FixedAxisToolpathView& operation,
    std::size_t completedMoveCount,
    double cutterDiameter,
    double cuttingLength,
    const StockSimulationOptions& options,
    std::vector<OrientedRasterMove>& moves,
    LocalPoint& operationAxis,
    int& planarLayerCount,
    int& nonPlanarMoveCount,
    std::string& error)
{
    const double tolerance = std::max(
        options.geometricTolerance,
        Precision::Confusion());
    gp_Vec axis = normalizedAxis(
        operation.outwardToolAxis, tolerance);
    if (axis.Magnitude() <= tolerance) {
        for (const ToolpathMove& move :
             operation.moves) {
            if (move.type != ToolpathMoveType::Plunge)
                continue;
            axis = normalizedAxis(
                gp_Vec(move.to, move.from), tolerance);
            if (axis.Magnitude() > tolerance)
                break;
        }
    }
    if (axis.Magnitude() <= tolerance) {
        error =
            "oriented multi-dexel operation requires a valid tool axis";
        return false;
    }
    if (!std::isfinite(cuttingLength) ||
        cuttingLength <= tolerance) {
        error =
            "oriented multi-dexel operation requires a positive cutting length";
        return false;
    }
    operationAxis = {
        axis.Dot(grid.uAxis),
        axis.Dot(grid.vAxis),
        axis.Dot(grid.wAxis)};

    const int segmentBudget = std::clamp(
        options.maxSweepSegments, 1, 5000000);
    const double radius = 0.5 * cutterDiameter;
    const double radialU = radius * std::sqrt(
        std::max(0.0, 1.0 -
            operationAxis.u * operationAxis.u));
    const double radialV = radius * std::sqrt(
        std::max(0.0, 1.0 -
            operationAxis.v * operationAxis.v));
    const double axialUMin = std::min(
        0.0, operationAxis.u * cuttingLength);
    const double axialUMax = std::max(
        0.0, operationAxis.u * cuttingLength);
    const double axialVMin = std::min(
        0.0, operationAxis.v * cuttingLength);
    const double axialVMax = std::max(
        0.0, operationAxis.v * cuttingLength);
    std::vector<std::pair<int, double>> planarLayers;
    const std::size_t moveLimit = std::min(
        completedMoveCount,
        operation.moves.size());
    moves.reserve(std::min(
        static_cast<std::size_t>(segmentBudget), moveLimit));
    for (std::size_t moveIndex = 0;
         moveIndex < moveLimit; ++moveIndex) {
        const ToolpathMove& move =
            operation.moves[moveIndex];
        if (!cuttingMove(move))
            continue;
        if (static_cast<int>(moves.size()) >= segmentBudget) {
            error =
                "oriented multi-dexel cutting move budget exceeded";
            return false;
        }
        if (!finitePoint(move.from) ||
            !finitePoint(move.to)) {
            error =
                "oriented multi-dexel cutting move contains a non-finite point";
            return false;
        }
        OrientedRasterMove raster;
        raster.from = grid.project(move.from);
        raster.to = grid.project(move.to);
        const double uMin =
            std::min(raster.from.u, raster.to.u) +
            axialUMin - radialU;
        const double uMax =
            std::max(raster.from.u, raster.to.u) +
            axialUMax + radialU;
        const double vMin =
            std::min(raster.from.v, raster.to.v) +
            axialVMin - radialV;
        const double vMax =
            std::max(raster.from.v, raster.to.v) +
            axialVMax + radialV;
        raster.iMin = std::clamp(
            grid.coordinateToI(uMin) - 1,
            0, grid.nx - 1);
        raster.iMax = std::clamp(
            grid.coordinateToI(uMax) + 1,
            0, grid.nx - 1);
        raster.jMin = std::clamp(
            grid.coordinateToJ(vMin) - 1,
            0, grid.ny - 1);
        raster.jMax = std::clamp(
            grid.coordinateToJ(vMax) + 1,
            0, grid.ny - 1);
        if (raster.iMin <= raster.iMax &&
            raster.jMin <= raster.jMax) {
            moves.push_back(raster);
        }

        const LocalPoint delta = difference(
            raster.to, raster.from);
        const double axialTravel =
            std::abs(dot(delta, operationAxis));
        const double totalSquared = dot(delta, delta);
        const double radialTravel = std::sqrt(
            std::max(
                0.0,
                totalSquared - axialTravel * axialTravel));
        if (axialTravel <= tolerance &&
            radialTravel > tolerance) {
            const double layer = 0.5 * (
                dot(raster.from, operationAxis) +
                dot(raster.to, operationAxis));
            const bool known = std::any_of(
                planarLayers.begin(), planarLayers.end(),
                [&](const auto& candidate) {
                    return candidate.first == move.depthIndex &&
                           std::abs(candidate.second - layer) <=
                               tolerance;
                });
            if (!known) {
                planarLayers.emplace_back(
                    move.depthIndex, layer);
            }
        } else {
            ++nonPlanarMoveCount;
        }
    }
    planarLayerCount = static_cast<int>(
        planarLayers.size());
    if (moves.empty()) {
        error =
            "oriented operation contains no cutting moves inside the multi-dexel grid";
        return false;
    }
    return true;
}

} // namespace oap::toolpath::multi_dexel_detail

// 孔 BRep 参考后端与原生 Multi-Dexel 的程序化对比门禁。
#include "manufacturing/ipw/HoleMultiDexelUpdater.h"
#include "toolpath/hole/HoleStockSimulator.h"
#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/core/simulation/StockComparison.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepCheck_Analyzer.hxx>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

double intervalLength(std::span<const oap::toolpath::DexelInterval> intervals)
{
    double result = 0.0;
    for (const auto& interval : intervals)
        result += interval.upper - interval.lower;
    return result;
}

double overlapLength(
    std::span<const oap::toolpath::DexelInterval> left,
    std::span<const oap::toolpath::DexelInterval> right)
{
    double result = 0.0;
    std::size_t leftIndex = 0;
    std::size_t rightIndex = 0;
    while (leftIndex < left.size() &&
           rightIndex < right.size()) {
        const double lower = std::max(
            left[leftIndex].lower,
            right[rightIndex].lower);
        const double upper = std::min(
            left[leftIndex].upper,
            right[rightIndex].upper);
        result += std::max(0.0, upper - lower);
        if (left[leftIndex].upper < right[rightIndex].upper)
            ++leftIndex;
        else
            ++rightIndex;
    }
    return result;
}

struct SameGridDifference {
    double remainingAllowanceVolume = 0.0;
    double overcutVolume = 0.0;
};

struct MaterialBounds {
    std::array<double, 3> minimum{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()};
    std::array<double, 3> maximum{
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()};
    bool valid = false;
};

void includePoint(MaterialBounds& bounds, const gp_Pnt& point)
{
    const std::array<double, 3> values{
        point.X(), point.Y(), point.Z()};
    for (int axis = 0; axis < 3; ++axis) {
        bounds.minimum[axis] = std::min(
            bounds.minimum[axis], values[axis]);
        bounds.maximum[axis] = std::max(
            bounds.maximum[axis], values[axis]);
    }
    bounds.valid = true;
}

void includeCellInterval(
    MaterialBounds& bounds,
    const oap::toolpath::MultiDexelStock& grid,
    double u0,
    double u1,
    double v0,
    double v1,
    double w0,
    double w1)
{
    for (const double u : {u0, u1}) {
        for (const double v : {v0, v1}) {
            for (const double w : {w0, w1}) {
                gp_Vec offset = grid.uAxis().Multiplied(u);
                offset.Add(grid.vAxis().Multiplied(v));
                offset.Add(grid.wAxis().Multiplied(w));
                includePoint(bounds, grid.origin().Translated(offset));
            }
        }
    }
}

MaterialBounds removedMaterialBounds(
    const oap::toolpath::MultiDexelStock& initial,
    const oap::toolpath::MultiDexelStock& final)
{
    MaterialBounds result;
    constexpr double tolerance = 1.0e-10;
    for (int j = 0; j < initial.ny(); ++j) {
        const double v0 = initial.vMin() +
            static_cast<double>(j) * initial.resolution();
        const double v1 = std::min(
            initial.vMax(), v0 + initial.resolution());
        for (int i = 0; i < initial.nx(); ++i) {
            const double u0 = initial.uMin() +
                static_cast<double>(i) * initial.resolution();
            const double u1 = std::min(
                initial.uMax(), u0 + initial.resolution());
            const auto before = initial.cellIntervalsView(i, j);
            const auto after = final.cellIntervalsView(i, j);
            for (const auto& source : before) {
                double cursor = source.lower;
                for (const auto& retained : after) {
                    if (retained.upper <= cursor + tolerance)
                        continue;
                    if (retained.lower >= source.upper - tolerance)
                        break;
                    const double gapUpper = std::min(
                        source.upper, retained.lower);
                    if (gapUpper > cursor + tolerance) {
                        includeCellInterval(
                            result, initial, u0, u1, v0, v1,
                            cursor, gapUpper);
                    }
                    cursor = std::max(cursor, retained.upper);
                    if (cursor >= source.upper - tolerance)
                        break;
                }
                if (cursor < source.upper - tolerance) {
                    includeCellInterval(
                        result, initial, u0, u1, v0, v1,
                        cursor, source.upper);
                }
            }
        }
    }
    return result;
}

double maximumBoundsDifference(
    const MaterialBounds& left,
    const MaterialBounds& right)
{
    double result = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        result = std::max(result, std::abs(
            left.minimum[axis] - right.minimum[axis]));
        result = std::max(result, std::abs(
            left.maximum[axis] - right.maximum[axis]));
    }
    return result;
}

SameGridDifference compareSameGridMaterial(
    const oap::toolpath::MultiDexelStock& native,
    const oap::toolpath::MultiDexelStock& reference)
{
    SameGridDifference result;
    for (int j = 0; j < native.ny(); ++j) {
        const double v0 = native.vMin() +
            static_cast<double>(j) * native.resolution();
        const double v1 = std::min(
            native.vMax(), v0 + native.resolution());
        for (int i = 0; i < native.nx(); ++i) {
            const double u0 = native.uMin() +
                static_cast<double>(i) * native.resolution();
            const double u1 = std::min(
                native.uMax(), u0 + native.resolution());
            const double area = (u1 - u0) * (v1 - v0);
            const auto nativeIntervals =
                native.cellIntervalsView(i, j);
            const auto referenceIntervals =
                reference.cellIntervalsView(i, j);
            const double common = overlapLength(
                nativeIntervals, referenceIntervals);
            result.remainingAllowanceVolume += area * std::max(
                0.0, intervalLength(nativeIntervals) - common);
            result.overcutVolume += area * std::max(
                0.0, intervalLength(referenceIntervals) - common);
        }
    }
    return result;
}

double referenceVolumeBudget(
    double referenceRemovedVolume,
    double affectedProjectedArea,
    const oap::toolpath::MultiDexelStock& grid,
    const oap::toolpath::StockSimulationResult& simulation)
{
    const double spatialError = std::max({
        1.0e-5,
        std::sqrt(0.5) * grid.resolution(),
        0.02,
        simulation.maximumToolProfileAxialError,
        simulation.maximumToolProfileRadialError});
    return std::max(
        0.12 * referenceRemovedVolume,
        affectedProjectedArea * spatialError);
}

double referenceSpatialBudget(
    const oap::toolpath::MultiDexelStock& grid,
    const oap::toolpath::StockSimulationResult& simulation)
{
    return std::max({
        1.0e-5,
        std::sqrt(0.5) * grid.resolution(),
        0.02,
        simulation.maximumToolProfileAxialError,
        simulation.maximumToolProfileRadialError});
}

oap::toolpath::HoleMachiningOperation drillOperation()
{
    oap::toolpath::HoleMachiningOperation result;
    result.operationId = "reference-drill";
    result.kind = oap::FeatureKind::BlindHole;
    result.strategy = oap::toolpath::HoleMachiningStrategy::SmallDrill;
    result.phase = oap::toolpath::HoleMachiningPhase::Drill;
    result.entry = gp_Pnt(10.0, 10.0, 10.0);
    result.bottom = gp_Pnt(10.0, 10.0, 2.0);
    result.commandedBottom = result.bottom;
    result.hasCommandedBottom = true;
    result.radius = 2.0;
    result.minimumCuttingEdgeLength = 8.0;
    result.minimumToolReach = 8.0;
    result.tool.valid = true;
    result.tool.toolType = "drill";
    result.tool.diameter = 4.0;
    result.tool.cuttingEdgeLength = 12.0;
    result.tool.overallLength = 20.0;
    result.tool.tipAngle = 118.0;

    oap::toolpath::ToolpathMove rapid;
    rapid.type = oap::toolpath::ToolpathMoveType::Rapid;
    rapid.from = gp_Pnt(10.0, 10.0, 12.0);
    rapid.to = result.entry;
    result.moves.push_back(rapid);
    oap::toolpath::ToolpathMove cut;
    cut.type = oap::toolpath::ToolpathMoveType::Cut;
    cut.from = result.entry;
    cut.to = result.bottom;
    result.moves.push_back(cut);
    oap::toolpath::ToolpathMove retract;
    retract.type = oap::toolpath::ToolpathMoveType::Retract;
    retract.from = result.bottom;
    retract.to = gp_Pnt(10.0, 10.0, 12.0);
    result.moves.push_back(retract);
    return result;
}

oap::toolpath::HoleMachiningOperation centerSpotOperation()
{
    auto result = drillOperation();
    result.operationId = "reference-center-spot";
    result.phase = oap::toolpath::HoleMachiningPhase::CenterSpot;
    result.bottom = gp_Pnt(10.0, 10.0, 8.5);
    result.commandedBottom = result.bottom;
    result.minimumCuttingEdgeLength = 1.5;
    result.minimumToolReach = 1.5;
    result.radius = 1.0;
    result.tool.toolType = "center-drill";
    result.tool.diameter = 2.0;
    result.tool.cuttingEdgeLength = 4.0;
    result.moves[1].to = result.bottom;
    result.moves[2].from = result.bottom;
    return result;
}

oap::toolpath::HoleMachiningOperation throughDrillOperation()
{
    auto result = drillOperation();
    result.operationId = "reference-through-drill";
    result.kind = oap::FeatureKind::ThroughHole;
    result.bottom = gp_Pnt(10.0, 10.0, -1.0);
    result.commandedBottom = result.bottom;
    result.breakthroughDepth = 1.0;
    result.minimumCuttingEdgeLength = 11.0;
    result.minimumToolReach = 11.0;
    result.moves[1].to = result.bottom;
    result.moves[2].from = result.bottom;
    return result;
}

oap::toolpath::HoleMachiningOperation drillAlong(
    const gp_Pnt& entry,
    const gp_Pnt& bottom,
    std::string operationId)
{
    auto result = drillOperation();
    result.operationId = std::move(operationId);
    result.entry = entry;
    result.bottom = bottom;
    result.commandedBottom = bottom;
    const gp_Vec outward(bottom, entry);
    gp_Vec clearance = outward;
    clearance.Normalize();
    result.moves[0].from = entry.Translated(
        clearance.Multiplied(2.0));
    result.moves[0].to = entry;
    result.moves[1].from = entry;
    result.moves[1].to = bottom;
    result.moves[2].from = bottom;
    result.moves[2].to = result.moves[0].from;
    return result;
}

oap::toolpath::HoleMachiningOperation millingOperation(
    oap::toolpath::HoleMachiningStrategy strategy,
    double centerX,
    double centerY,
    std::string operationId)
{
    oap::toolpath::HoleMachiningOperation result;
    result.operationId = std::move(operationId);
    result.kind = oap::FeatureKind::BlindHole;
    result.strategy = strategy;
    result.phase = oap::toolpath::HoleMachiningPhase::FlatEndMill;
    result.entry = gp_Pnt(centerX, centerY, 10.0);
    result.bottom = gp_Pnt(centerX, centerY, 4.0);
    result.commandedBottom = result.bottom;
    result.hasCommandedBottom = true;
    result.radius = 3.0;
    result.minimumCuttingEdgeLength = 3.0;
    result.minimumToolReach = 6.0;
    result.tool.valid = true;
    result.tool.toolType = "corner-radius-end-mill";
    result.tool.diameter = 2.0;
    result.tool.cornerRadius = 0.2;
    result.tool.cuttingEdgeLength = 8.0;
    result.tool.overallLength = 16.0;

    constexpr int segmentCount = 24;
    if (strategy == oap::toolpath::HoleMachiningStrategy::MediumHelical) {
        gp_Pnt previous(centerX + 2.0, centerY, 10.0);
        for (int segment = 1; segment <= segmentCount; ++segment) {
            const double angle = 2.0 * std::numbers::pi *
                static_cast<double>(segment) /
                static_cast<double>(segmentCount);
            const double z = 10.0 - 6.0 *
                static_cast<double>(segment) /
                static_cast<double>(segmentCount);
            const gp_Pnt next(
                centerX + 2.0 * std::cos(angle),
                centerY + 2.0 * std::sin(angle), z);
            oap::toolpath::ToolpathMove cut;
            cut.type = oap::toolpath::ToolpathMoveType::Cut;
            cut.from = previous;
            cut.to = next;
            result.moves.push_back(cut);
            previous = next;
        }
        for (int segment = 1; segment <= segmentCount; ++segment) {
            const double angle = 2.0 * std::numbers::pi *
                static_cast<double>(segment) /
                static_cast<double>(segmentCount);
            const gp_Pnt next(
                centerX + 2.0 * std::cos(angle),
                centerY + 2.0 * std::sin(angle), 4.0);
            oap::toolpath::ToolpathMove cut;
            cut.type = oap::toolpath::ToolpathMoveType::Cut;
            cut.from = previous;
            cut.to = next;
            result.moves.push_back(cut);
            previous = next;
        }
    } else {
        gp_Pnt previous(centerX + 0.5, centerY, 10.0);
        for (int layer = 0; layer < 2; ++layer) {
            const double z = layer == 0 ? 7.0 : 4.0;
            oap::toolpath::ToolpathMove plunge;
            plunge.type = oap::toolpath::ToolpathMoveType::Plunge;
            plunge.from = previous;
            plunge.to = gp_Pnt(previous.X(), previous.Y(), z);
            plunge.depthIndex = layer;
            result.moves.push_back(plunge);
            previous = plunge.to;
            for (int ring = 0; ring < 3; ++ring) {
                const double radius = 0.5 + 0.75 * ring;
                for (int segment = 1; segment <= 12; ++segment) {
                    const double angle = 2.0 * std::numbers::pi *
                        static_cast<double>(segment) / 12.0;
                    const gp_Pnt next(
                        centerX + radius * std::cos(angle),
                        centerY + radius * std::sin(angle), z);
                    oap::toolpath::ToolpathMove cut;
                    cut.type = oap::toolpath::ToolpathMoveType::Cut;
                    cut.from = previous;
                    cut.to = next;
                    cut.depthIndex = layer;
                    cut.ring = ring;
                    result.moves.push_back(cut);
                    previous = next;
                }
            }
        }
    }
    return result;
}

gp_Pnt zToX(const gp_Pnt& point)
{
    return {point.Z() + 1.0, point.X(), point.Y()};
}

oap::toolpath::HoleMachiningOperation zToX(
    oap::toolpath::HoleMachiningOperation operation)
{
    operation.entry = zToX(operation.entry);
    operation.bottom = zToX(operation.bottom);
    operation.commandedBottom = zToX(operation.commandedBottom);
    for (auto& move : operation.moves) {
        move.from = zToX(move.from);
        move.to = zToX(move.to);
    }
    return operation;
}

oap::toolpath::StockSimulationOptions dexelOptions(bool parallel)
{
    oap::toolpath::StockSimulationOptions result;
    result.method = oap::toolpath::StockSimulationMethod::HeightField;
    result.runParallel = parallel;
    result.stopOnFailure = true;
    result.zMapResolution = 0.25;
    result.zMapMaxCellCount = 100000;
    result.zMapTileSize = 16;
    result.maxSweepSegments = 10000;
    result.maxToolProfileBands = 512;
    return result;
}

oap::toolpath::ToolpathParams parameters()
{
    oap::toolpath::ToolpathParams result;
    result.tool.diameter = 4.0;
    result.tool.cuttingEdgeLength = 12.0;
    result.hole.drillTipAngleDeg = 118.0;
    return result;
}

oap::toolpath::HoleStockSimulationOptions options(bool parallel)
{
    oap::toolpath::HoleStockSimulationOptions result;
    result.runParallel = parallel;
    result.stopOnFailure = true;
    result.geometricTolerance = 1.0e-5;
    return result;
}

bool referenceRemovalIsMonotonicAndDeterministic()
{
    const TopoDS_Shape stock =
        BRepPrimAPI_MakeBox(20.0, 20.0, 10.0).Shape();
    const std::vector<oap::toolpath::HoleMachiningOperation> operations{
        drillOperation()};
    const auto serial = oap::toolpath::HoleStockSimulator().simulate(
        stock, operations, parameters(), options(false));
    const auto parallel = oap::toolpath::HoleStockSimulator().simulate(
        stock, operations, parameters(), options(true));
    constexpr double tolerance = 1.0e-5;
    if (!expect(serial.complete && parallel.complete &&
                    serial.steps.size() == 1 && parallel.steps.size() == 1 &&
                    serial.steps[0].status ==
                        oap::toolpath::StockRemovalStatus::Applied,
                "the reference drill should produce one committed removal") ||
        !expect(serial.initialVolume > serial.finalVolume &&
                    serial.removedVolume > 80.0 &&
                    serial.removedVolume < 180.0 &&
                    serial.steps[0].volumeAfter <=
                        serial.steps[0].volumeBefore,
                "hole stock volume must decrease within the programmatic reference budget") ||
        !expect(std::abs(serial.initialVolume - 4000.0) <= tolerance &&
                    std::abs(serial.finalVolume - parallel.finalVolume) <=
                        tolerance &&
                    std::abs(serial.removedVolume - parallel.removedVolume) <=
                        tolerance,
                "serial and parallel BRep removal must preserve deterministic volume")) {
        std::cerr << "serial initial=" << serial.initialVolume
                  << " final=" << serial.finalVolume
                  << " removed=" << serial.removedVolume
                  << " parallel final=" << parallel.finalVolume << '\n';
        return false;
    }

    oap::toolpath::ToolpathGenerationControl cancelledControl;
    cancelledControl.isCancellationRequested = [] { return true; };
    const auto cancelled = oap::toolpath::HoleStockSimulator().simulate(
        stock, operations, parameters(), options(false), &cancelledControl);
    return expect(cancelled.cancelled && !cancelled.complete &&
                      cancelled.steps.empty() &&
                      std::abs(cancelled.initialVolume -
                               cancelled.finalVolume) <= tolerance &&
                      cancelled.removedVolume <= tolerance,
                  "pre-cancelled hole simulation must not publish a partial removal");
}

bool helicalCompoundRemovalIsValidAndTransactional()
{
    using namespace oap::toolpath;
    const auto stock = BRepPrimAPI_MakeBox(80.0, 80.0, 10.0).Shape();
    const auto operation = millingOperation(
        HoleMachiningStrategy::MediumHelical, 10.0, 10.0, "compound-helical");
    // D2 follows a radius-2 helix: it must leave the central core. Replacing
    // the fallback with a radius-3 cylinder would silently overcut this fixture.
    double removed = -1.0;
    for (const bool parallel : {false, true}) {
        const auto result = HoleStockSimulator().simulate(
            stock, {operation}, parameters(), options(parallel));
        if (!expect(result.complete && result.appliedOperationCount == 1 &&
                        BRepCheck_Analyzer(result.finalStock).IsValid(),
                    "overlapping helical sweeps must produce a valid committed stock") ||
            !expect(result.removedVolume > 100.0 && result.removedVolume < 160.0,
                    "helical fallback must remove the annulus without filling the central core"))
            return false;
        if (removed < 0.0) removed = result.removedVolume;
        if (!expect(std::abs(removed - result.removedVolume) < 1.e-6,
                    "serial/parallel helical removal volumes differ") ||
            !expect(BRepClass3d_SolidClassifier(result.finalStock, gp_Pnt(10, 10, 5), 1.e-6).State() == TopAbs_IN &&
                        BRepClass3d_SolidClassifier(result.finalStock, gp_Pnt(12, 10, 5), 1.e-6).State() == TopAbs_OUT,
                    "helical fallback lost its core or retained material on the cutter path"))
            return false;
    }
    std::atomic_int polls{0};
    ToolpathGenerationControl control;
    control.isCancellationRequested = [&] { return polls.fetch_add(1) >= 20; };
    const auto cancelled = HoleStockSimulator().simulate(
        stock, {operation}, parameters(), options(false), &control);
    return expect(cancelled.cancelled && !cancelled.complete &&
                      cancelled.appliedOperationCount == 0 &&
                      !cancelled.steps.empty() && cancelled.steps.front().booleanBatchCount > 0 &&
                      cancelled.finalStock.IsSame(stock) &&
                      std::abs(cancelled.finalVolume - cancelled.initialVolume) < 1.e-6 &&
                      BRepCheck_Analyzer(stock).IsValid(),
                  "cancelling a multi-batch helix must retain the parent stock");
}

bool booleanComparisonPreservesMaterialAccounting()
{
    using namespace oap::toolpath;
    const auto initial = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    InProcessStock stock(initial, 1.e-5, false);
    const auto removal = BRepPrimAPI_MakeBox(5.0, 10.0, 10.0).Shape();
    const auto step = stock.applyRemoval(removal, "left-half", 0);
    if (!expect(step.status == StockRemovalStatus::Applied &&
                    std::abs(stock.currentVolume() - 500.0) < 1.e-6,
                "configured single-build cut changed the retained half-stock"))
        return false;
    struct Case { TopoDS_Shape target; double missing, remaining, overcut; };
    const std::array<Case, 4> cases{{
        {initial, 0, 0, 500},
        {stock.currentShape(), 0, 0, 0},
        {BRepPrimAPI_MakeBox(gp_Pnt(5, 0, 0), 10, 10, 10).Shape(), 500, 0, 0},
        {BRepPrimAPI_MakeBox(gp_Pnt(7, 0, 0), 3, 10, 10).Shape(), 0, 200, 0}
    }};
    for (const auto& item : cases) {
        for (const bool prevalidated : {false, true}) {
            StockComparisonOptions opts;
            opts.initialAndFinalShapesPrevalidated = prevalidated;
            const auto result = StockComparison().compare(initial, stock.currentShape(), item.target, opts);
            if (!expect(result.complete &&
                            std::abs(result.initialMissingVolume - item.missing) < 1.e-6 &&
                            std::abs(result.remainingAllowanceVolume - item.remaining) < 1.e-6 &&
                            std::abs(result.overcutVolume - item.overcut) < 1.e-6,
                        "Cut/Common comparison or prevalidated stocks changed material accounting"))
                return false;
        }
    }
    ToolpathGenerationControl cancelled;
    cancelled.isCancellationRequested = [] { return true; };
    const auto rejected = StockComparison().compare(initial, stock.currentShape(), initial, {}, &cancelled);
    return expect(rejected.cancelled && !rejected.complete,
                  "comparison ignored cancellation before its configured Boolean build");
}

bool publicHoleIpwPreservesCallerParallelBudget()
{
    using namespace oap::toolpath;
    const auto initial = MultiDexelStock::fromShape(
        BRepPrimAPI_MakeBox(20.0, 20.0, 10.0).Shape(),
        gp_Vec(0.0, 0.0, 1.0), 4.0, dexelOptions(false));
    if (!expect(initial && initial->valid(), "IPW budget fixture should rasterize"))
        return false;
    const std::vector<HoleMachiningOperation> operations{drillOperation()};
    const auto serial = oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
        initial, operations, parameters(), dexelOptions(false));
    if (!expect(serial.complete() && serial.simulation->zMapTileCount > 1 &&
                    serial.stock->volume() < initial->volume(),
                "IPW budget fixture must reach a real multi-tile removal"))
        return false;
    const auto caller = std::this_thread::get_id();
    const auto* priorScope = currentToolpathParallelControl();
    const std::string initialFingerprint = initial->contentFingerprint();
    for (const std::size_t capacity : {0u, 1u}) {
        ToolpathParallelBudget budget(capacity);
        std::atomic_size_t requests{0u}, acquired{0u}, released{0u}, polls{0u};
        std::atomic_bool offCaller{false};
        ToolpathGenerationControl control;
        control.tryAcquireParallelWorkers = [&](std::size_t requested) {
            requests.fetch_add(1u);
            const auto count = budget.tryAcquire(requested);
            acquired.fetch_add(count);
            return count;
        };
        control.releaseParallelWorkers = [&](std::size_t count) {
            released.fetch_add(count);
            budget.release(count);
        };
        control.isCancellationRequested = [&] {
            polls.fetch_add(1u);
            if (std::this_thread::get_id() != caller)
                offCaller.store(true);
            return false;
        };
        // Do not install a test-side scope: the public manufacturing entry
        // must propagate its explicit control through to the tile engine.
        const auto result = oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
            initial, operations, parameters(), dexelOptions(true), &control);
        const bool hardwareCanBorrow = toolpathHardwareParallelism() > 1u;
        if (!expect(result.complete() && result.stock->contentFingerprint() ==
                        serial.stock->contentFingerprint() && polls.load() > 0u,
                    "public IPW entry must preserve material truth under caller budgets") ||
            !expect((hardwareCanBorrow ? requests.load() > 0u : requests.load() == 0u) &&
                        acquired.load() == (hardwareCanBorrow ? capacity : 0u) &&
                        acquired.load() == released.load() && budget.available() == capacity,
                    "public IPW control must reach tile acquisition and return all granted slots") ||
            !expect((capacity != 0u || !offCaller.load()) &&
                        currentToolpathParallelControl() == priorScope &&
                        initial->contentFingerprint() == initialFingerprint,
                    "zero-budget IPW must stay on its caller and restore scope without changing its parent"))
            return false;
    }
    return true;
}

bool nativeDexelIsTransactionalAndMatchesReference()
{
    const TopoDS_Shape stockShape =
        BRepPrimAPI_MakeBox(20.0, 20.0, 10.0).Shape();
    const auto initial = oap::toolpath::MultiDexelStock::fromShape(
        stockShape, gp_Vec(0.0, 0.0, 1.0), 4.0,
        dexelOptions(false));
    if (!expect(initial && initial->valid(),
                "programmatic stock should build a multi-dexel root")) {
        return false;
    }
    const std::vector<oap::toolpath::HoleMachiningOperation> operations{
        drillOperation()};
    const auto serial =
        oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
            initial, operations, parameters(), dexelOptions(false));
    const auto parallel =
        oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
            initial, operations, parameters(), dexelOptions(true));
    if (!expect(serial.complete() && parallel.complete() &&
                    serial.stock->volume() < initial->volume() &&
                    serial.simulation->steps.size() == 1 &&
                    serial.simulation->steps[0].status ==
                        oap::toolpath::StockRemovalStatus::Applied,
                "native hole update should commit one monotonic tile transaction") ||
        !expect(serial.stock->contentFingerprint() ==
                    parallel.stock->contentFingerprint() &&
                    std::abs(serial.stock->volume() -
                             parallel.stock->volume()) <= 1.0e-9,
                "serial and parallel native updates must have one material fingerprint")) {
        return false;
    }

    const auto repeated =
        oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
            serial.stock, operations, parameters(), dexelOptions(false));
    if (!expect(repeated.complete() &&
                    repeated.stock->contentFingerprint() ==
                        serial.stock->contentFingerprint(),
                "replaying the same hole sweep should be idempotent")) {
        return false;
    }

    const auto reference = oap::toolpath::HoleStockSimulator().simulate(
        stockShape, operations, parameters(), options(false));
    const double dexelRemoved =
        initial->volume() - serial.stock->volume();
    const double allowedDifference = referenceVolumeBudget(
        reference.removedVolume,
        std::numbers::pi * 2.0 * 2.0,
        *initial, *serial.simulation);
    const auto referenceGrid =
        oap::toolpath::MultiDexelStock::fromShapeOnGrid(
            reference.finalStock, *initial,
            dexelOptions(false));
    const SameGridDifference materialDifference =
        referenceGrid
        ? compareSameGridMaterial(*serial.stock, *referenceGrid)
        : SameGridDifference{};
    const MaterialBounds nativeRemovalBounds =
        removedMaterialBounds(*initial, *serial.stock);
    const MaterialBounds referenceRemovalBounds = referenceGrid
        ? removedMaterialBounds(*initial, *referenceGrid)
        : MaterialBounds{};
    const double allowedBoundsDifference = referenceSpatialBudget(
        *initial, *serial.simulation);
    const double boundsDifference =
        nativeRemovalBounds.valid && referenceRemovalBounds.valid
        ? maximumBoundsDifference(
              nativeRemovalBounds, referenceRemovalBounds)
        : std::numeric_limits<double>::infinity();
    if (!expect(reference.complete && referenceGrid &&
                    std::abs(dexelRemoved - reference.removedVolume) <=
                        allowedDifference &&
                    materialDifference.remainingAllowanceVolume <=
                        allowedDifference &&
                    materialDifference.overcutVolume <=
                        allowedDifference &&
                    boundsDifference <= allowedBoundsDifference,
                "native drill volume, allowance, overcut and removal bounds should stay within the reference budget")) {
        std::cerr << "reference removed=" << reference.removedVolume
                  << " dexel removed=" << dexelRemoved
                  << " allowed difference=" << allowedDifference
                  << " allowance="
                  << materialDifference.remainingAllowanceVolume
                  << " overcut=" << materialDifference.overcutVolume
                  << " bounds difference=" << boundsDifference
                  << " allowed bounds difference="
                  << allowedBoundsDifference
                  << '\n';
        return false;
    }

    oap::toolpath::ToolpathGenerationControl cancelledControl;
    cancelledControl.isCancellationRequested = [] { return true; };
    const std::string rootFingerprint = initial->contentFingerprint();
    const auto cancelled =
        oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
            initial, operations, parameters(), dexelOptions(true),
            &cancelledControl);
    return expect(cancelled.cancelled && !cancelled.complete() &&
                      !cancelled.stock &&
                      initial->contentFingerprint() == rootFingerprint,
                  "cancelled native update must discard its child state");
}

bool nativeThroughAndOverlappingToolsAreDeterministic()
{
    const TopoDS_Shape stockShape =
        BRepPrimAPI_MakeBox(20.0, 20.0, 10.0).Shape();
    const auto initial = oap::toolpath::MultiDexelStock::fromShape(
        stockShape, gp_Vec(0.0, 0.0, 1.0), 4.0,
        dexelOptions(false));
    if (!expect(initial && initial->valid(),
                "through-hole stock should build a multi-dexel root")) {
        return false;
    }
    const std::vector<oap::toolpath::HoleMachiningOperation> operations{
        centerSpotOperation(), throughDrillOperation()};
    const auto serial =
        oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
            initial, operations, parameters(), dexelOptions(false));
    const auto parallel =
        oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
            initial, operations, parameters(), dexelOptions(true));
    if (!expect(serial.complete() && parallel.complete() &&
                    serial.simulation->steps.size() == 2 &&
                    serial.simulation->processedOperationCount == 2 &&
                    serial.simulation->appliedOperationCount == 2,
                "center spot and through drill should commit two operations") ||
        !expect(serial.simulation->steps[0].volumeAfter <=
                    serial.simulation->steps[0].volumeBefore &&
                    serial.simulation->steps[1].volumeAfter <=
                    serial.simulation->steps[0].volumeAfter &&
                    serial.stock->volume() < initial->volume(),
                "overlapping tool changes must keep material volume monotonic") ||
        !expect(serial.stock->contentFingerprint() ==
                    parallel.stock->contentFingerprint(),
                "overlapping serial and parallel hole updates must be deterministic")) {
        return false;
    }

    const auto replay =
        oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
            serial.stock, operations, parameters(), dexelOptions(false));
    return expect(replay.complete() &&
                      replay.stock->contentFingerprint() ==
                          serial.stock->contentFingerprint(),
                  "center spot and through drill replay should be idempotent");
}

bool nativeMillingStrategiesUseDeterministicRoundedProfiles()
{
    const TopoDS_Shape stockShape =
        BRepPrimAPI_MakeBox(30.0, 30.0, 10.0).Shape();
    const auto initial = oap::toolpath::MultiDexelStock::fromShape(
        stockShape, gp_Vec(0.0, 0.0, 1.0), 2.0,
        dexelOptions(false));
    if (!expect(initial && initial->valid(),
                "milling stock should build a multi-dexel root")) {
        return false;
    }
    const std::vector<oap::toolpath::HoleMachiningOperation> operations{
        millingOperation(
            oap::toolpath::HoleMachiningStrategy::MediumHelical,
            8.0, 8.0, "reference-helical"),
        millingOperation(
            oap::toolpath::HoleMachiningStrategy::LargeLayeredSpiral,
            22.0, 22.0, "reference-layered-spiral")};
    const auto serial =
        oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
            initial, operations, parameters(), dexelOptions(false));
    const auto parallel =
        oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
            initial, operations, parameters(), dexelOptions(true));
    return expect(serial.complete() && parallel.complete() &&
                      serial.simulation->processedOperationCount == 2 &&
                      serial.simulation->appliedOperationCount == 2,
                  "helical and layered spiral should both commit") &&
           expect(serial.simulation->fallbackSweepSegmentCount > 0 &&
                      serial.simulation->planarSweepLayerCount >= 2 &&
                      serial.simulation->toolProfileBandCount > 0,
                  "milling strategies should exercise non-planar, planar and rounded-profile sweeps") &&
           expect(serial.stock->volume() < initial->volume() &&
                      serial.stock->contentFingerprint() ==
                          parallel.stock->contentFingerprint(),
                  "milling strategy material must be monotonic and deterministic");
}

bool orientedMillingStrategiesMatchReference()
{
    const TopoDS_Shape stockShape =
        BRepPrimAPI_MakeBox(12.0, 30.0, 30.0).Shape();
    const auto initial = oap::toolpath::MultiDexelStock::fromShape(
        stockShape, gp_Vec(0.0, 0.0, 1.0), 2.0,
        dexelOptions(false));
    if (!expect(initial && initial->valid(),
                "oriented milling stock should build a multi-dexel root")) {
        return false;
    }
    std::vector<oap::toolpath::HoleMachiningOperation> operations{
        zToX(millingOperation(
            oap::toolpath::HoleMachiningStrategy::MediumHelical,
            8.0, 8.0, "oriented-helical")),
        zToX(millingOperation(
            oap::toolpath::HoleMachiningStrategy::LargeLayeredSpiral,
            22.0, 22.0, "oriented-layered-spiral"))};
    const auto serial =
        oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
            initial, operations, parameters(), dexelOptions(false));
    const auto parallel =
        oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
            initial, operations, parameters(), dexelOptions(true));
    if (!expect(serial.complete() && parallel.complete() &&
                    serial.stock->volume() < initial->volume() &&
                    serial.stock->contentFingerprint() ==
                        parallel.stock->contentFingerprint(),
                "oriented helical strategies should be native and deterministic")) {
        return false;
    }

    const double nativeRemoved =
        initial->volume() - serial.stock->volume();
    const double analyticRemoved =
        2.0 * std::numbers::pi * 3.0 * 3.0 * 6.0;
    const double difference = std::abs(
        nativeRemoved - analyticRemoved) / analyticRemoved;
    if (!expect(difference <= 0.12,
                "oriented helical removal should stay within the analytic bore budget")) {
        std::cerr << "oriented milling analytic removed="
                  << analyticRemoved
                  << " native removed=" << nativeRemoved
                  << " relative difference=" << difference << '\n';
        return false;
    }
    return true;
}

bool nativeAxisDirectionsUseOrientedSweep()
{
    const TopoDS_Shape stockShape =
        BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape();
    const auto initial = oap::toolpath::MultiDexelStock::fromShape(
        stockShape, gp_Vec(0.0, 0.0, 1.0), 4.0,
        dexelOptions(false));
    if (!expect(initial && initial->valid(),
                "multi-axis stock should build a multi-dexel root")) {
        return false;
    }
    const std::vector<oap::toolpath::HoleMachiningOperation> operations{
        drillAlong({20.0, 10.0, 10.0}, {2.0, 10.0, 10.0}, "axis+x"),
        drillAlong({0.0, 10.0, 10.0}, {18.0, 10.0, 10.0}, "axis-x"),
        drillAlong({10.0, 20.0, 10.0}, {10.0, 2.0, 10.0}, "axis+y"),
        drillAlong({10.0, 0.0, 10.0}, {10.0, 18.0, 10.0}, "axis-y"),
        drillAlong({10.0, 10.0, 20.0}, {10.0, 10.0, 2.0}, "axis+z"),
        drillAlong({10.0, 10.0, 0.0}, {10.0, 10.0, 18.0}, "axis-z"),
        drillAlong({18.0, 10.0, 18.0}, {4.0, 10.0, 4.0}, "axis-oblique")};
    for (const auto& operation : operations) {
        const auto serial =
            oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
                initial, {operation}, parameters(), dexelOptions(false));
        const auto parallel =
            oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
                initial, {operation}, parameters(), dexelOptions(true));
        if (!expect(serial.complete() && parallel.complete() &&
                        serial.stock->volume() < initial->volume(),
                    "every supported setup axis should remove material natively") ||
            !expect(serial.stock->contentFingerprint() ==
                        parallel.stock->contentFingerprint(),
                    "multi-axis serial and parallel updates must be deterministic")) {
            std::cerr << "axis operation=" << operation.operationId << '\n';
            return false;
        }

        const auto reference = oap::toolpath::HoleStockSimulator().simulate(
            stockShape, {operation}, parameters(), options(false));
        const double nativeRemoved =
            initial->volume() - serial.stock->volume();
        const double allowedDifference = referenceVolumeBudget(
            reference.removedVolume, 100.0,
            *initial, *serial.simulation);
        const auto referenceGrid =
            oap::toolpath::MultiDexelStock::fromShapeOnGrid(
                reference.finalStock, *initial,
                dexelOptions(false));
        const SameGridDifference materialDifference =
            referenceGrid
            ? compareSameGridMaterial(*serial.stock, *referenceGrid)
            : SameGridDifference{};
        const MaterialBounds nativeRemovalBounds =
            removedMaterialBounds(*initial, *serial.stock);
        const MaterialBounds referenceRemovalBounds = referenceGrid
            ? removedMaterialBounds(*initial, *referenceGrid)
            : MaterialBounds{};
        const double allowedBoundsDifference = referenceSpatialBudget(
            *initial, *serial.simulation);
        const double boundsDifference =
            nativeRemovalBounds.valid && referenceRemovalBounds.valid
            ? maximumBoundsDifference(
                  nativeRemovalBounds, referenceRemovalBounds)
            : std::numeric_limits<double>::infinity();
        if (!expect(reference.complete && referenceGrid &&
                        std::abs(nativeRemoved - reference.removedVolume) <=
                            allowedDifference &&
                        materialDifference.remainingAllowanceVolume <=
                            allowedDifference &&
                        materialDifference.overcutVolume <=
                            allowedDifference &&
                        boundsDifference <= allowedBoundsDifference,
                    "multi-axis drill volume, allowance, overcut and removal bounds should stay within the reference budget")) {
            std::cerr << "axis operation=" << operation.operationId
                      << " reference removed=" << reference.removedVolume
                      << " native removed=" << nativeRemoved
                      << " allowed difference=" << allowedDifference
                      << " allowance="
                      << materialDifference.remainingAllowanceVolume
                      << " overcut="
                      << materialDifference.overcutVolume
                      << " bounds difference=" << boundsDifference
                      << " allowed bounds difference="
                      << allowedBoundsDifference << '\n';
            return false;
        }
        const auto replay =
            oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
                serial.stock, {operation}, parameters(),
                dexelOptions(false));
        if (!expect(replay.complete() &&
                        replay.stock->contentFingerprint() ==
                            serial.stock->contentFingerprint(),
                    "multi-axis hole replay should be idempotent")) {
            return false;
        }
    }
    return true;
}

bool orientedCancellationDiscardsChildState()
{
    const TopoDS_Shape stockShape =
        BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape();
    const auto initial = oap::toolpath::MultiDexelStock::fromShape(
        stockShape, gp_Vec(0.0, 0.0, 1.0), 4.0,
        dexelOptions(false));
    if (!expect(initial && initial->valid(),
                "oriented cancellation stock should build")) {
        return false;
    }
    const std::string parentFingerprint =
        initial->contentFingerprint();
    std::atomic_int cancellationPolls{0};
    oap::toolpath::ToolpathGenerationControl control;
    control.isCancellationRequested = [&] {
        return cancellationPolls.fetch_add(1) >= 2;
    };
    const auto cancelled =
        oap::manufacturing::ipw::applyHoleOperationsToMultiDexel(
            initial,
            {drillAlong(
                {20.0, 10.0, 10.0},
                {2.0, 10.0, 10.0},
                "oriented-cancel")},
            parameters(), dexelOptions(false), &control);
    return expect(cancelled.cancelled && !cancelled.complete() &&
                      !cancelled.stock &&
                      initial->contentFingerprint() ==
                          parentFingerprint,
                  "cancelling an oriented tile transaction must not publish its child state");
}

} // namespace

int main()
{
    if (!referenceRemovalIsMonotonicAndDeterministic() ||
        !helicalCompoundRemovalIsValidAndTransactional() ||
        !booleanComparisonPreservesMaterialAccounting() ||
        !publicHoleIpwPreservesCallerParallelBudget() ||
        !nativeDexelIsTransactionalAndMatchesReference() ||
        !nativeThroughAndOverlappingToolsAreDeterministic() ||
        !nativeMillingStrategiesUseDeterministicRoundedProfiles() ||
        !orientedMillingStrategiesMatchReference() ||
        !nativeAxisDirectionsUseOrientedSweep() ||
        !orientedCancellationDiscardsChildState())
        return 1;
    std::cout << "hole IPW reference smoke tests passed\n";
    return 0;
}

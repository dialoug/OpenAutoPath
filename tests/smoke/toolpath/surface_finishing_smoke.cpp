// Analytic contact checks exercise the real OpenCAMLib backend, not a mock.
#include "toolpath/finishing/SurfaceFinishing.h"
#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/core/ToolpathOcctParallel.h"
#include "toolpath/program/ToolpathProgramExport.h"
#include "toolpath/program/validation/ToolpathProgramValidator.h"
#include "toolpath/program/validation/GCodeReview.h"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <STEPControl_Writer.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Ax1.hxx>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>

namespace {
using namespace oap::toolpath;

void require(bool condition, const std::string& message)
{
    if (!condition)
        throw std::runtime_error(message);
}

SurfaceFinishingParams parameters()
{
    SurfaceFinishingParams p;
    p.diameter = 4;
    p.cornerRadius = 0.5;
    p.stepOver = 2;
    p.sampleSpacing = 0.75;
    p.meshDeflection = 0.01;
    return p;
}

TopoDS_Shape slope()
{
    BRepBuilderAPI_MakePolygon wire;
    wire.Add(gp_Pnt(-10, -10, 2.5));
    wire.Add(gp_Pnt(10, -10, 7.5));
    wire.Add(gp_Pnt(10, 10, 7.5));
    wire.Add(gp_Pnt(-10, 10, 2.5));
    wire.Close();
    return BRepBuilderAPI_MakeFace(wire.Wire()).Shape();
}

void structure(const SurfaceFinishingResult& result)
{
    require(result.ok, "generation failed: " + result.error);
    require(!result.moves.empty() && !result.primitives.empty(), "missing moves/primitives");
    require(!result.warning.empty(), "missing sampled-contact capability statement");
    const auto program = result.toProgram();
    require(program.operations.size() == 1 &&
        program.operations.front().type == ToolpathOperationType::SurfaceFinishing,
        "finishing must retain its own operation type");
    const auto report = ToolpathProgramValidator().validate(program, result.toolpathParams());
    require(report.ok(), "program validation: " + report.summary());
    for (std::size_t i = 1; i < result.moves.size(); ++i)
        require(result.moves[i - 1].to.Distance(result.moves[i].from) < 1e-7,
            "discontinuous movement chain");
    for (const auto& move : result.moves) {
        if (move.type == ToolpathMoveType::Rapid)
            require(std::abs(move.from.Z() - result.safePlaneOffset) < 1e-7 &&
                std::abs(move.to.Z() - result.safePlaneOffset) < 1e-7,
                "rapid below common clearance plane");
        if (move.type == ToolpathMoveType::Retract || move.type == ToolpathMoveType::Plunge)
            require(std::hypot(move.from.X() - move.to.X(), move.from.Y() - move.to.Y()) < 1e-7,
                "entry/retract is not vertical");
    }
}

void parallelProjection(bool fromStock)
{
    const auto target = slope();
    const auto stock = BRepPrimAPI_MakeBox(gp_Pnt(-10, -10, 0), 20, 20, 10).Shape();
    const auto* planningStock = fromStock ? &stock : nullptr;
    auto p = parameters();
    p.mode = fromStock ? EngravingMachiningMode::StockRoughingAndFinishing : EngravingMachiningMode::FinishOnly;
    p.stepOver = 0.2;
    p.sampleSpacing = 0.2;
    p.maxStepDown = 2;
    p.roughingAllowance = 0.2;
    ToolpathGenerationControl serialControl;
    serialControl.tryAcquireParallelWorkers = [](std::size_t) { return 0u; };
    serialControl.releaseParallelWorkers = [](std::size_t) {};
    ToolpathParallelBudget budget(2);
    std::size_t acquired = 0, released = 0;
    ToolpathGenerationControl parallelControl;
    parallelControl.tryAcquireParallelWorkers = [&](std::size_t requested) {
        const auto count = budget.tryAcquire(requested);
        acquired += count;
        return count;
    };
    parallelControl.releaseParallelWorkers = [&](std::size_t count) {
        released += count;
        budget.release(count);
    };
    for (const auto kind : {FinishingCutterKind::Flat, FinishingCutterKind::Ball, FinishingCutterKind::Bull}) {
        p.cutterKind = kind;
        const auto serial = SurfaceFinishing().generate(target, gp_Ax3(), p, &serialControl, planningStock);
        const auto parallel = SurfaceFinishing().generate(target, gp_Ax3(), p, &parallelControl, planningStock);
        structure(serial);
        structure(parallel);
        require(serial.metrics.projectionWorkerCount == 1 && parallel.metrics.projectionWorkerCount <= 3 &&
            (toolpathHardwareParallelism() == 1 || parallel.metrics.projectionWorkerCount > 1) && acquired == released,
            "projection ignored the shared worker budget or leaked its lease");
        require(serial.moves.size() == parallel.moves.size() && serial.primitives.size() == parallel.primitives.size() &&
            serial.metrics.sampleCount == parallel.metrics.sampleCount &&
            serial.metrics.roughingMoveCount == parallel.metrics.roughingMoveCount &&
            serial.metrics.segmentCount == parallel.metrics.segmentCount,
            "parallel projection changed sample/segment/move counts");
        for (std::size_t i = 0; i < serial.moves.size(); ++i) {
            const auto& a = serial.moves[i];
            const auto& b = parallel.moves[i];
            require(a.type == b.type && a.from.SquareDistance(b.from) == 0 && a.to.SquareDistance(b.to) == 0 &&
                a.feed == b.feed && a.ring == b.ring && a.loopSequence == b.loopSequence,
                "parallel projection changed ordered cutter motion");
        }
        if (kind == FinishingCutterKind::Flat) {
            auto limited = p;
            limited.maxSamples = serial.metrics.sampleCount - 1u;
            const auto failed = SurfaceFinishing().generate(target, gp_Ax3(), limited, &parallelControl, planningStock);
            require(!failed.ok && !failed.cancelled && failed.moves.empty() && failed.primitives.empty() &&
                failed.error.find("sample budget") != std::string::npos && acquired == released,
                "parallel sample exhaustion published paths, hid the failure or leaked workers");
        }
    }
    std::atomic_bool cancel{false};
    std::size_t lastProgress = 0;
    parallelControl.isCancellationRequested = [&] { return cancel.load(); };
    parallelControl.reportProgress = [&](std::size_t completed, std::size_t total) {
        require(completed >= lastProgress && completed <= total, "parallel progress was out of order");
        lastProgress = completed;
        if (completed > 0) cancel.store(true);
    };
    const auto cancelled = SurfaceFinishing().generate(target, gp_Ax3(), p, &parallelControl, planningStock);
    require(cancelled.cancelled && !cancelled.ok && cancelled.moves.empty() && cancelled.primitives.empty() &&
        acquired == released, "parallel cancellation published paths or leaked workers");
    cancel.store(false);
    parallelControl.reportProgress = [](std::size_t completed, std::size_t) {
        if (completed > 0) throw std::runtime_error("projection progress failure");
    };
    const auto rejected = SurfaceFinishing().generate(target, gp_Ax3(), p, &parallelControl, planningStock);
    require(!rejected.ok && !rejected.cancelled && rejected.error == "projection progress failure" &&
        rejected.moves.empty() && rejected.primitives.empty() && acquired == released,
        "parallel callback failure was lost or retained worker resources");
}

void analyticPlanes()
{
    const auto face = slope();
    TopExp_Explorer faces(face, TopAbs_FACE);
    TopLoc_Location location;
    const auto originalMesh = BRep_Tool::Triangulation(TopoDS::Face(faces.Current()), location);
    for (auto kind : {FinishingCutterKind::Ball, FinishingCutterKind::Flat, FinishingCutterKind::Bull}) {
        auto p = parameters();
        p.cutterKind = kind;
        p.allowance = 0.3;
        const auto result = SurfaceFinishing().generate(face, gp_Ax3(), p);
        structure(result);
        const double r = p.diameter * 0.5;
        const double a = 0.25;
        const double curvature = std::sqrt(1 + a * a) - 1;
        const double offset = kind == FinishingCutterKind::Ball ? r * curvature :
            kind == FinishingCutterKind::Flat ? r * a :
            (r - p.cornerRadius) * a + p.cornerRadius * curvature;
        std::size_t checked = 0;
        for (const auto& move : result.moves) {
            if (move.type != ToolpathMoveType::Cut)
                continue;
            const auto& q = move.to;
            if (std::abs(q.X()) < 6 && std::abs(q.Y()) < 6) {
                const double expected = 5 + a * q.X() + offset + p.allowance;
                require(std::abs(q.Z() - expected) < 0.002,
                    "incorrect slope/cutter/vertical allowance contact: got " +
                    std::to_string(q.Z()) + ", expected " + std::to_string(expected));
                ++checked;
            }
        }
        require(checked > 20, "insufficient interior analytical samples");
    }
    require(BRep_Tool::Triangulation(TopoDS::Face(faces.Current()), location) == originalMesh,
        "generation mutated the input's triangulation");
}

void analyticSphere()
{
    const auto shape = BRepPrimAPI_MakeSphere(10).Shape();
    const auto result = SurfaceFinishing().generate(shape, gp_Ax3(), parameters());
    structure(result);
    std::size_t checked = 0;
    for (const auto& move : result.moves) {
        if (move.type != ToolpathMoveType::Cut)
            continue;
        const auto& q = move.to;
        const double rho2 = q.X() * q.X() + q.Y() * q.Y();
        if (rho2 < 49) {
            const double expected = std::sqrt(144 - rho2) - 2;
            require(std::abs(q.Z() - expected) < 0.04, "sphere ball-offset contact exceeds mesh tolerance");
            ++checked;
        }
    }
    require(checked > 20, "insufficient spherical contact samples");
}

void frameAndExport()
{
    const auto base = BRepPrimAPI_MakeBox(20, 16, 5).Shape();
    const auto p = parameters();
    const auto local = SurfaceFinishing().generate(base, gp_Ax3(), p);
    structure(local);
    gp_Trsf transform;
    transform.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 1, 0)), 0.7);
    transform.SetTranslationPart(gp_Vec(30, -20, 40));
    gp_Ax3 frame;
    frame.Transform(transform);
    const auto world = SurfaceFinishing().generate(
        BRepBuilderAPI_Transform(base, transform, true).Shape(), frame, p);
    require(world.ok, "rotated frame generation failed: " + world.error);
    require(world.moves.size() == local.moves.size(), "frame change altered the raster topology: world " +
        std::to_string(world.moves.size()) + " moves/" + std::to_string(world.metrics.scanLineCount) +
        " rows; local " + std::to_string(local.moves.size()) + " moves/" +
        std::to_string(local.metrics.scanLineCount) + " rows");
    for (std::size_t i = 0; i < world.moves.size(); ++i) {
        if (world.moves[i].type == ToolpathMoveType::Cut)
            require(world.moves[i].from.Distance(local.moves[i].from.Transformed(transform)) < 1e-5 &&
                world.moves[i].to.Distance(local.moves[i].to.Transformed(transform)) < 1e-5,
                "world/frame transform changed tool contact");
        // The world-AABB clearance bound may become more conservative on rotation.
        if (world.moves[i].type == ToolpathMoveType::Rapid) {
            const gp_Vec delta(frame.Location(), world.moves[i].to);
            require(std::abs(delta.Dot(gp_Vec(frame.Direction())) - world.safePlaneOffset) < 1e-6,
                "rotated rapid did not use local clearance Z");
        }
    }
    MachiningSetup setup;
    setup.workFrame = frame;
    setup.useMoldFaceWorkFrame = false;
    GCodeProgramOptions options;
    options.programName = "OpenCAMLib finishing regression";
    const auto output = std::filesystem::temp_directory_path() / "oap_surface_finishing_test.nc";
    std::string error;
    require(ToolpathProgramExport().writeGCode(world.toProgram(), world.toolpathParams(),
        setup, options, output.string(), error), "reference NC export: " + error);
    std::ifstream file(output);
    const std::string nc((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();
    std::filesystem::remove(output);
    require(nc.find("G01") != std::string::npos || nc.find("G1") != std::string::npos,
        "NC contains no feed move");
    require(nc.find("nan") == std::string::npos && nc.find("inf") == std::string::npos,
        "NC contains nonfinite numbers");
    require(GCodeReview().review(nc).ok(), "finishing NC did not pass the application's review gate");
}

void rejectAndCancel()
{
    const auto shape = BRepPrimAPI_MakeBox(20, 16, 5).Shape();
    const auto failed = [](const SurfaceFinishingResult& r) {
        require(!r.ok && !r.error.empty() && r.moves.empty() && r.primitives.empty(),
            "failure leaked a partial toolpath or omitted error");
    };
    auto p = parameters();
    p.sampleSpacing = std::numeric_limits<double>::quiet_NaN();
    failed(SurfaceFinishing().generate(shape, gp_Ax3(), p));
    p = parameters();
    p.diameter = 0;
    failed(SurfaceFinishing().generate(shape, gp_Ax3(), p));
    p = parameters();
    p.stepOver = p.diameter * 2;
    failed(SurfaceFinishing().generate(shape, gp_Ax3(), p));
    p = parameters();
    p.diameter = 1e200;
    failed(SurfaceFinishing().generate(shape, gp_Ax3(), p));
    p = parameters();
    p.meshDeflection = 1e-100;
    failed(SurfaceFinishing().generate(shape, gp_Ax3(), p));
    p = parameters();
    p.cutterKind = FinishingCutterKind::Bull;
    p.cornerRadius = p.diameter;
    failed(SurfaceFinishing().generate(shape, gp_Ax3(), p));
    p = parameters();
    p.maxSamples = 3;
    failed(SurfaceFinishing().generate(shape, gp_Ax3(), p));
    p = parameters();
    p.maxTriangles = 1;
    failed(SurfaceFinishing().generate(shape, gp_Ax3(), p));
    failed(SurfaceFinishing().generate({}, gp_Ax3(), parameters()));
    ToolpathGenerationControl control;
    control.isCancellationRequested = [] { return true; };
    auto cancelled = SurfaceFinishing().generate(shape, gp_Ax3(), parameters(), &control);
    require(cancelled.cancelled && !cancelled.ok && cancelled.moves.empty() && cancelled.primitives.empty(),
        "pre-cancellation did not discard output");
    bool stop = false;
    control.isCancellationRequested = [&] { return stop; };
    control.reportProgress = [&](std::size_t completed, std::size_t) { if (completed > 0) stop = true; };
    control.tryAcquireParallelWorkers = [](std::size_t) -> std::size_t { return 0; };
    cancelled = SurfaceFinishing().generate(shape, gp_Ax3(), parameters(), &control);
    require(stop && cancelled.cancelled && !cancelled.ok && cancelled.moves.empty() && cancelled.primitives.empty(),
        "mid-generation cancellation did not discard output");
}

void contactGaps()
{
    TopoDS_Compound islands;
    BRep_Builder builder;
    builder.MakeCompound(islands);
    builder.Add(islands, BRepPrimAPI_MakeBox(gp_Pnt(-18, -10, 0), 10, 20, 6).Shape());
    builder.Add(islands, BRepPrimAPI_MakeBox(gp_Pnt(8, -10, 0), 10, 20, 12).Shape());
    const auto result = SurfaceFinishing().generate(islands, gp_Ax3(), parameters());
    structure(result);
    require(result.metrics.segmentCount >= 2 * result.metrics.scanLineCount,
        "separated islands were connected without clearance retractions");
    for (const auto& move : result.moves)
        if (move.type == ToolpathMoveType::Cut)
            require(move.from.X() * move.to.X() > 0,
                "cutting move crossed a non-contact gap");
}

void parallelCurvesAndGaps()
{
    // Combine curved contact, missing contact, and steep walls. This exercises
    // recursive subdivision and run boundaries, not only flat raster rows.
    BRep_Builder builder;
    TopoDS_Compound target;
    builder.MakeCompound(target);
    builder.Add(target, BRepPrimAPI_MakeSphere(gp_Pnt(-14, 0, 0), 10).Shape());
    builder.Add(target, BRepPrimAPI_MakeBox(gp_Pnt(8, -10, 0), 10, 20, 12).Shape());
    auto p = parameters();
    p.stepOver = 0.3;
    p.sampleSpacing = 0.3;
    p.meshDeflection = 0.08;
    ToolpathGenerationControl serialControl;
    serialControl.tryAcquireParallelWorkers = [](std::size_t) { return 0u; };
    serialControl.releaseParallelWorkers = [](std::size_t) {};
    ToolpathParallelBudget budget(toolpathHardwareParallelism() - 1u);
    ToolpathGenerationControl parallelControl;
    parallelControl.tryAcquireParallelWorkers = [&](std::size_t count) { return budget.tryAcquire(count); };
    parallelControl.releaseParallelWorkers = [&](std::size_t count) { budget.release(count); };
    const auto before = toolpathOcctParallelStatistics();
    const auto serial = SurfaceFinishing().generate(target, gp_Ax3(), p, &serialControl);
    const auto parallel = SurfaceFinishing().generate(target, gp_Ax3(), p, &parallelControl);
    structure(serial);
    structure(parallel);
    require(budget.available() == budget.capacity(), "meshing or projection leaked worker slots");
    const auto after = toolpathOcctParallelStatistics();
    require(after.requested >= before.requested + 2u, "meshing bypassed OCCT parallel admission");
    if (toolpathOcctParallelism() > 1u && toolpathOcctParallelism() <= toolpathHardwareParallelism())
        require(after.admitted > before.admitted, "multi-face meshing did not use available workers");
    require(serial.moves.size() == parallel.moves.size() &&
        serial.metrics.sampleCount == parallel.metrics.sampleCount &&
        serial.metrics.segmentCount == parallel.metrics.segmentCount,
        "parallel curved/gap projection changed its topology or sample count");
    for (std::size_t i = 0; i < serial.moves.size(); ++i) {
        const auto& a = serial.moves[i];
        const auto& b = parallel.moves[i];
        require(a.type == b.type && a.from.SquareDistance(b.from) == 0 && a.to.SquareDistance(b.to) == 0 &&
            a.feed == b.feed && a.ring == b.ring && a.loopSequence == b.loopSequence,
            "parallel meshing/projection changed curved or disconnected cutter motion");
        if (a.type == ToolpathMoveType::Cut)
            require(a.from.X() * a.to.X() > 0, "parallel finish bridged disconnected islands");
    }
    p.maxSamples = serial.metrics.sampleCount - 1u;
    const auto failed = SurfaceFinishing().generate(target, gp_Ax3(), p, &parallelControl);
    require(!failed.ok && !failed.cancelled && failed.moves.empty() && failed.primitives.empty() &&
        failed.error.find("sample budget") != std::string::npos && budget.available() == budget.capacity(),
        "adaptive parallel refinement did not enforce its global sample budget atomically");
}

void stableLayerTemplate()
{
    // A linear slope has no physical discontinuity. Clipping it at successive
    // roughing levels moves a sharp corner along X. Independent per-layer
    // adaptive refinement can reject one corner at its depth limit, then join
    // across that old gap on the next layer and falsely fail entry coverage.
    BRepBuilderAPI_MakePolygon boundary;
    boundary.Add(gp_Pnt(0, 0, 1));
    boundary.Add(gp_Pnt(4, 0, 2));
    boundary.Add(gp_Pnt(4, 2, 2));
    boundary.Add(gp_Pnt(0, 2, 1));
    boundary.Close();
    const auto target = BRepBuilderAPI_MakeFace(boundary.Wire()).Shape();
    const auto stock = BRepPrimAPI_MakeBox(4, 2, 4).Shape();
    auto p = parameters();
    p.mode = EngravingMachiningMode::StockRoughingAndFinishing;
    p.cutterKind = FinishingCutterKind::Flat;
    p.diameter = 0.5;
    p.stepOver = 0.4;
    p.sampleSpacing = 0.75;
    p.meshDeflection = 1.e-7;
    p.maxStepDown = 1;
    p.roughingAllowance = 0.2;
    const auto result = SurfaceFinishing().generate(target, gp_Ax3(), p, nullptr, &stock);
    structure(result);
    require(result.metrics.roughingLayerCount >= 4 &&
        result.metrics.roughingMoveCount < result.moves.size(),
        "stable layer regression lost roughing or finishing");
    using Extent = std::tuple<int, double, double>;
    std::vector<std::vector<Extent>> extents(result.metrics.roughingLayerCount);
    std::size_t layer = 0;
    int precedingRow = -1;
    double entryX = 0;
    for (std::size_t index = 0; index < result.metrics.roughingMoveCount; ++index) {
        const auto& move = result.moves[index];
        if (move.type == ToolpathMoveType::Plunge) {
            if (move.ring < precedingRow) ++layer;
            precedingRow = move.ring;
            entryX = move.to.X();
        } else if (move.type == ToolpathMoveType::Retract) {
            require(layer < extents.size(), "roughing layer identity was lost");
            extents[layer].emplace_back(move.ring, entryX, move.from.X());
        }
    }
    const std::size_t rows = result.metrics.scanLineCount / (result.metrics.roughingLayerCount + 1u);
    require(extents.front().size() > rows, "fixture did not retain its unresolved clipped-corner gaps");
    for (std::size_t index = 1; index < extents.size(); ++index)
        require(extents[index] == extents.front(),
            "a deeper stock layer bridged or omitted an earlier template interval");

    // Coordinate translation, row reversal, and stock margins must not restore
    // the moving-gap failure. The tool frame is deliberately offset from origin.
    gp_Trsf placement;
    placement.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(0, 1, 0)), 3.14159265358979323846);
    placement.SetTranslationPart(gp_Vec(37, -12, 9));
    gp_Ax3 frame;
    frame.Transform(placement);
    const auto transformedTarget = BRepBuilderAPI_Transform(target, placement, true).Shape();
    const auto marginStock = BRepPrimAPI_MakeBox(gp_Pnt(-0.2, -0.2, -0.2), 4.4, 2.4, 4.4).Shape();
    const auto transformedStock = BRepBuilderAPI_Transform(marginStock, placement, true).Shape();
    const auto transformed = SurfaceFinishing().generate(transformedTarget, frame, p, nullptr, &transformedStock);
    require(transformed.ok && transformed.metrics.roughingMoveCount > 0,
        "offset/flipped frame or stock margin broke stable layer coverage: " + transformed.error);

    bool cancel = false;
    ToolpathGenerationControl control;
    control.isCancellationRequested = [&] { return cancel; };
    control.reportProgress = [&](std::size_t completed, std::size_t) { cancel = completed > 0; };
    const auto interrupted = SurfaceFinishing().generate(target, gp_Ax3(), p, &control, &stock);
    require(interrupted.cancelled && interrupted.moves.empty() && interrupted.primitives.empty() &&
        interrupted.metrics.roughingLayerCount == 0,
        "template-stage cancellation published partial engraving output");
    p.maxSamples = 100;
    const auto limited = SurfaceFinishing().generate(target, gp_Ax3(), p, nullptr, &stock);
    require(!limited.ok && limited.moves.empty() && limited.primitives.empty(),
        "shared-template sample budget failure published partial paths");
}

void stockLayeringAndFinishing()
{
    const auto target = BRepPrimAPI_MakeBox(gp_Pnt(3, 2, 0), 6, 6, 3).Shape();
    const auto stock = BRepPrimAPI_MakeBox(12, 10, 8).Shape();
    auto p = parameters();
    p.mode = EngravingMachiningMode::StockRoughingAndFinishing;
    p.diameter = 2;
    p.stepOver = 1;
    p.sampleSpacing = 0.75;
    p.maxStepDown = 1;
    p.roughingAllowance = 0.2;
    const auto result = SurfaceFinishing().generate(target, gp_Ax3(), p, nullptr, &stock);
    structure(result);
    require(result.metrics.roughingLayerCount >= 8 && result.metrics.roughingMoveCount > 0 &&
        result.metrics.roughingMoveCount < result.moves.size(), "missing ordered roughing/finishing passes");
    require(result.safePlaneOffset >= 8 + p.safeClearance,
        "stock extending above target was omitted from the safe plane");
    require(result.toProgram().operations.front().metadata.at("algorithm") == kOpenCamLibStockEngravingAlgorithmId,
        "stock engraving reused the finish-only algorithm identity");
    double previousLayer = 8.0000001;
    std::size_t observedLayers = 0;
    bool clearedBlankStock = false, roughAllowanceSeen = false, finalContactSeen = false;
    for (std::size_t index = 0; index < result.moves.size(); ++index) {
        const auto& move = result.moves[index];
        const bool rough = index < result.metrics.roughingMoveCount;
        if (rough && move.type == ToolpathMoveType::Plunge && move.ring == 0) {
            require(previousLayer - move.to.Z() <= p.maxStepDown + 1.e-6 &&
                move.to.Z() <= previousLayer + 1.e-6,
                "blank stock layer entry exceeded the declared step-down");
            previousLayer = move.to.Z();
            ++observedLayers;
        }
        if (move.type != ToolpathMoveType::Cut) continue;
        const auto& point = move.to;
        if (rough) {
            require(point.Z() >= -1.e-6, "no-contact roughing descended below finite stock bottom");
            if (point.Y() < 0.1 && point.Z() < 0.1) clearedBlankStock = true;
        }
        if (point.X() > 4 && point.X() < 8 && point.Y() > 3 && point.Y() < 7) {
            if (rough) {
                require(point.Z() >= 3 + p.roughingAllowance - 1.e-7,
                    "roughing gouged the protected target allowance");
                roughAllowanceSeen |= std::abs(point.Z() - 3 - p.roughingAllowance) < 1.e-7;
            } else {
                require(std::abs(point.Z() - 3) < 1.e-7, "final engraving did not remove the roughing allowance");
                finalContactSeen = true;
            }
        }
    }
    require(observedLayers == result.metrics.roughingLayerCount && clearedBlankStock &&
        roughAllowanceSeen && finalContactSeen, "layered envelope omitted blank stock or the final contact pass");

    // Each physical setup uses its own local +Z, with the whole world-space
    // stock included. All rapid moves must remain on that setup's safe plane.
    for (const gp_Dir axis : {gp_Dir(1, 0, 0), gp_Dir(-1, 0, 0), gp_Dir(0, 1, 0),
                             gp_Dir(0, -1, 0), gp_Dir(0, 0, -1)}) {
        const gp_Ax3 frame(gp_Pnt(), axis);
        const auto oriented = SurfaceFinishing().generate(target, frame, p, nullptr, &stock);
        require(oriented.ok, "rotated stock engraving failed: " + oriented.error);
        require(oriented.metrics.roughingMoveCount > 0 && oriented.metrics.roughingMoveCount < oriented.moves.size(),
            "rotated stock engraving lost a machining phase");
        for (const auto& move : oriented.moves)
            if (move.type == ToolpathMoveType::Rapid) {
                const double fromZ = gp_Vec(frame.Location(), move.from).Dot(gp_Vec(axis));
                const double toZ = gp_Vec(frame.Location(), move.to).Dot(gp_Vec(axis));
                require(std::abs(fromZ - oriented.safePlaneOffset) < 1.e-7 &&
                    std::abs(toZ - oriented.safePlaneOffset) < 1.e-7, "rotated stock rapid used world Z");
            }
    }

    const auto rejected = [](const SurfaceFinishingResult& value) {
        require(!value.ok && !value.error.empty() && value.moves.empty() && value.primitives.empty(),
            "rejected stock engraving published partial paths");
    };
    rejected(SurfaceFinishing().generate(target, gp_Ax3(), p));
    const auto undersized = BRepPrimAPI_MakeBox(2, 2, 2).Shape();
    const auto shifted = BRepPrimAPI_MakeBox(gp_Pnt(20, 20, 20), 20, 20, 20).Shape();
    rejected(SurfaceFinishing().generate(target, gp_Ax3(), p, nullptr, &undersized));
    rejected(SurfaceFinishing().generate(target, gp_Ax3(), p, nullptr, &shifted));
    auto limited = p;
    limited.maxMoves = result.metrics.roughingMoveCount + 2;
    rejected(SurfaceFinishing().generate(target, gp_Ax3(), limited, nullptr, &stock));
    limited = p;
    limited.maxStepDown = 1.e-7;
    limited.roughingAllowance = 0;
    rejected(SurfaceFinishing().generate(target, gp_Ax3(), limited, nullptr, &stock));
    limited = p;
    limited.roughingAllowance = p.maxStepDown + 0.1;
    rejected(SurfaceFinishing().generate(target, gp_Ax3(), limited, nullptr, &stock));
    bool stop = false;
    ToolpathGenerationControl control;
    control.isCancellationRequested = [&] { return stop; };
    control.reportProgress = [&](std::size_t completed, std::size_t) { if (completed >= 15) stop = true; };
    const auto cancelled = SurfaceFinishing().generate(target, gp_Ax3(), p, &control, &stock);
    require(cancelled.cancelled && cancelled.moves.empty() && cancelled.primitives.empty(),
        "stock engraving cancellation published a partial roughing stage");
    p.mode = EngravingMachiningMode::FinishOnly;
    const auto finish = SurfaceFinishing().generate(target, gp_Ax3(), p, nullptr, &stock);
    structure(finish);
    require(finish.metrics.roughingLayerCount == 0 && finish.metrics.roughingMoveCount == 0 &&
        finish.safePlaneOffset >= 8 + p.safeClearance &&
        finish.toProgram().operations.front().metadata.at("algorithm") == kOpenCamLibSurfaceFinishingAlgorithmId,
        "finish-only compatibility or supplied-stock clearance changed");
}

void writeSamples(const std::filesystem::path& directory)
{
    std::filesystem::create_directories(directory);
    const auto write = [&](const char* name, const TopoDS_Shape& shape) {
        STEPControl_Writer writer;
        require(writer.Transfer(shape, STEPControl_AsIs) == IFSelect_RetDone, "STEP transfer failed");
        require(writer.Write((directory / name).string().c_str()) == IFSelect_RetDone, "STEP write failed");
    };
    write("finishing-slope.step", slope());
    write("finishing-sphere.step", BRepPrimAPI_MakeSphere(10).Shape());
    write("finishing-block.step", BRepPrimAPI_MakeBox(30, 20, 8).Shape());
    TopoDS_Compound islands;
    BRep_Builder builder;
    builder.MakeCompound(islands);
    builder.Add(islands, BRepPrimAPI_MakeBox(gp_Pnt(-18, -10, 0), 10, 20, 6).Shape());
    builder.Add(islands, BRepPrimAPI_MakeBox(gp_Pnt(8, -10, 0), 10, 20, 12).Shape());
    write("finishing-separated-islands.step", islands);
}
} // namespace

int main(int argc, char** argv)
{
    try {
        if (argc == 2 && std::string(argv[1]) == "--stable-layer-template") {
            stableLayerTemplate();
            std::cout << "Stable engraving layer-template regression passed.\n";
            return 0;
        }
        if (argc == 3 && std::string(argv[1]) == "--write-samples") {
            writeSamples(argv[2]);
            return 0;
        }
        analyticPlanes();
        analyticSphere();
        frameAndExport();
        contactGaps();
        stockLayeringAndFinishing();
        stableLayerTemplate();
        std::cout << "Checking parallel stock projection..." << std::endl;
        parallelProjection(true);
        std::cout << "Checking parallel finish projection..." << std::endl;
        parallelProjection(false);
        std::cout << "Checking parallel curved/gap projection and meshing..." << std::endl;
        parallelCurvesAndGaps();
        rejectAndCancel();
        std::cout << "OpenCAMLib finishing analytic, frame, export, budget and cancellation checks passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}

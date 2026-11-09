// Deterministic relief regression and an opt-in STEP acceptance CLI.
// The user's large STEP stays outside the repository and normal CTest runs.
#include "toolpath/finishing/SurfaceFinishing.h"
#include "toolpath/core/ToolpathOcctParallel.h"
#include "toolpath/program/ToolpathProgramExport.h"
#include "toolpath/program/validation/GCodeReview.h"
#include "toolpath/program/validation/ToolpathProgramValidator.h"

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <Geom_BezierSurface.hxx>
#include <STEPControl_Reader.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using namespace oap::toolpath;
using Clock = std::chrono::steady_clock;

void require(bool condition, const std::string& message)
{
    if (!condition)
        throw std::runtime_error(message);
}

TopoDS_Shape smallRelief()
{
    // A smooth raised ornament on a planar backing, with opposing lobes.
    TColgp_Array2OfPnt poles(1, 5, 1, 5);
    for (int x = 1; x <= 5; ++x) {
        for (int y = 1; y <= 5; ++y) {
            const bool rim = x == 1 || x == 5 || y == 1 || y == 5;
            const double z = rim ? 2.0 : (x == 3 && y == 3 ? 7.0 : 3.0 + (x == y ? 3.0 : 0.0));
            poles.SetValue(x, y, gp_Pnt((x - 1) * 5.0, (y - 1) * 5.0, z));
        }
    }
    const Handle(Geom_BezierSurface) relief = new Geom_BezierSurface(poles);
    BRep_Builder builder;
    TopoDS_Compound target;
    builder.MakeCompound(target);
    builder.Add(target, BRepPrimAPI_MakeBox(20.0, 20.0, 2.0).Shape());
    builder.Add(target, BRepBuilderAPI_MakeFace(relief, 1.e-7).Shape());
    return target;
}

ToolpathToolContext acceptanceTool(const SurfaceFinishingParams& params)
{
    ToolpathToolContext tool;
    tool.valid = true;
    tool.toolNumber = 1;
    tool.toolId = "engraving-relief-acceptance-tool";
    tool.toolName = "Engraving relief acceptance cutter";
    tool.toolType = params.cutterKind == FinishingCutterKind::Flat ? "end_mill" : "ball_end_mill";
    tool.diameter = params.diameter;
    tool.cornerRadius = params.cutterKind == FinishingCutterKind::Flat ? 0 : params.diameter * 0.5;
    tool.cuttingEdgeLength = params.cuttingLength;
    tool.horizontalFeed = params.feed;
    tool.verticalFeed = params.plungeFeed;
    tool.spindleSpeed = 8000;
    return tool;
}

void validateResult(const SurfaceFinishingResult& result, bool requireRelief, const gp_Ax3& frame)
{
    if (!result.ok)
        std::cout << "incomplete_triangles=" << result.metrics.triangleCount
            << " incomplete_samples=" << result.metrics.sampleCount
            << " completed_rows=" << result.metrics.scanLineCount
            << " mesh_ms=" << result.metrics.meshMilliseconds
            << " total_ms=" << result.metrics.totalMilliseconds << std::endl;
    require(result.ok && !result.cancelled, "relief generation: " + result.error);
    require(result.metrics.sampleCount <= result.effectiveParams.maxSamples &&
        result.metrics.triangleCount <= result.effectiveParams.maxTriangles,
        "relief generation exceeded its declared budget");
    require(result.metrics.scanLineCount > 2 && result.metrics.segmentCount > 2,
        "relief generation produced insufficient raster coverage");
    const auto program = result.toProgram("ENGRAVING_RELIEF", acceptanceTool(result.effectiveParams));
    const auto report = ToolpathProgramValidator().validate(program, result.toolpathParams());
    require(report.ok(), "relief program validation: " + report.summary());
    double cutMin = std::numeric_limits<double>::infinity();
    double cutMax = -std::numeric_limits<double>::infinity();
    std::size_t cutCount = 0;
    for (std::size_t i = 0; i < result.moves.size(); ++i) {
        const auto& move = result.moves[i];
        require(std::isfinite(move.from.Z()) && std::isfinite(move.to.Z()), "non-finite cutter location");
        if (i > 0)
            require(result.moves[i - 1].to.Distance(move.from) < 1.e-7, "discontinuous relief movement chain");
        if (move.type == ToolpathMoveType::Cut) {
            cutMin = std::min({cutMin, move.from.Z(), move.to.Z()});
            cutMax = std::max({cutMax, move.from.Z(), move.to.Z()});
            ++cutCount;
        } else if (move.type == ToolpathMoveType::Rapid) {
            const auto localZ = [&](const gp_Pnt& point) {
                return gp_Vec(frame.Location(), point).Dot(gp_Vec(frame.Direction()));
            };
            require(std::abs(localZ(move.from) - result.safePlaneOffset) < 1.e-7 &&
                std::abs(localZ(move.to) - result.safePlaneOffset) < 1.e-7,
                "relief rapid moved below the complete-target clearance plane");
        }
    }
    require(cutCount > 20, "relief has no useful cutting sequence");
    if (requireRelief)
        require(cutMax - cutMin > 0.5, "the raised relief was flattened into a constant-Z path");
    std::cout << std::setprecision(10)
        << "triangles=" << result.metrics.triangleCount
        << " samples=" << result.metrics.sampleCount
        << " projection_workers=" << result.metrics.projectionWorkerCount
        << " rows=" << result.metrics.scanLineCount
        << " contact_runs=" << result.metrics.segmentCount
        << " moves=" << result.moves.size()
        << " rough_layers=" << result.metrics.roughingLayerCount
        << " rough_moves=" << result.metrics.roughingMoveCount
        << " cut_z_min=" << cutMin << " cut_z_max=" << cutMax
        << " mesh_ms=" << result.metrics.meshMilliseconds
        << " projection_ms=" << result.metrics.projectionMilliseconds
        << " total_ms=" << result.metrics.totalMilliseconds << '\n';
}

void writeOutputs(const std::filesystem::path& directory, const SurfaceFinishingResult& result, const gp_Ax3& frame)
{
    std::filesystem::create_directories(directory);
    const std::string direction = frame.Direction().Z() < 0 ? "minus-z" : "plus-z";
    std::ofstream points(directory / ("engraving-" + direction + ".csv"));
    require(points.good(), "cannot create cutter-location output");
    points << "type,from_x,from_y,from_z,to_x,to_y,to_z,feed\n" << std::setprecision(17);
    for (const auto& move : result.moves)
        points << static_cast<int>(move.type) << ',' << move.from.X() << ',' << move.from.Y()
            << ',' << move.from.Z() << ',' << move.to.X() << ',' << move.to.Y() << ','
            << move.to.Z() << ',' << move.feed << '\n';
    points.close();
    require(points.good(), "could not finish writing cutter locations");
    MachiningSetup setup;
    setup.workFrame = frame;
    setup.useMoldFaceWorkFrame = false;
    setup.workFrameConfirmed = true;
    GCodeProgramOptions options;
    options.programName = "OpenCAMLib relief engraving " + direction + " acceptance";
    std::string error;
    const auto output = directory / ("engraving-" + direction + ".nc");
    require(ToolpathProgramExport().writeGCode(
        result.toProgram("ENGRAVING_RELIEF", acceptanceTool(result.effectiveParams)),
        result.toolpathParams(), setup, options, output.string(), error), "reference NC export: " + error);
    std::ifstream input(output);
    const std::string nc((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    require(GCodeReview().review(nc).ok(), "relief reference NC failed review");
    std::cout << "outputs=" << directory.string() << '\n';
}
}

int main(int argc, char** argv)
{
    try {
        std::string stepPath;
        std::filesystem::path output;
        double timeoutSeconds = 180.0;
        double stockMargin = 1.0;
        gp_Ax3 frame;
        bool positiveZ = true;
        SurfaceFinishingParams params;
        params.diameter = 2.0;
        params.cuttingLength = 20.0;
        params.stepOver = 0.4;
        params.sampleSpacing = 0.25;
        params.meshDeflection = 0.03;
        for (int i = 1; i < argc; ++i) {
            const std::string argument(argv[i]);
            require(i + 1 < argc, "missing value for " + argument);
            const std::string value(argv[++i]);
            if (argument == "--step") stepPath = value;
            else if (argument == "--output") output = value;
            else if (argument == "--diameter") params.diameter = std::stod(value);
            else if (argument == "--cutting-length") params.cuttingLength = std::stod(value);
            else if (argument == "--cutter") {
                require(value == "flat" || value == "ball", "cutter must be flat or ball");
                params.cutterKind = value == "flat" ? FinishingCutterKind::Flat : FinishingCutterKind::Ball;
            }
            else if (argument == "--axis") {
                require(value == "+Z" || value == "-Z", "axis must be +Z or -Z");
                positiveZ = value == "+Z";
                frame = gp_Ax3(gp_Pnt(), gp_Dir(0, 0, positiveZ ? 1 : -1), gp_Dir(1, 0, 0));
            }
            else if (argument == "--step-over") params.stepOver = std::stod(value);
            else if (argument == "--sample-spacing") params.sampleSpacing = std::stod(value);
            else if (argument == "--deflection") params.meshDeflection = std::stod(value);
            else if (argument == "--timeout-seconds") timeoutSeconds = std::stod(value);
            else if (argument == "--mode") {
                require(value == "stock" || value == "finish", "mode must be stock or finish");
                params.mode = value == "stock" ? EngravingMachiningMode::StockRoughingAndFinishing
                                               : EngravingMachiningMode::FinishOnly;
            }
            else if (argument == "--step-down") params.maxStepDown = std::stod(value);
            else if (argument == "--roughing-allowance") params.roughingAllowance = std::stod(value);
            else if (argument == "--stock-margin") stockMargin = std::stod(value);
            else if (argument == "--max-moves") params.maxMoves = std::stoull(value);
            else throw std::runtime_error("unknown argument: " + argument);
        }
        require(std::isfinite(timeoutSeconds) && timeoutSeconds > 0.0, "invalid timeout");
        const auto start = Clock::now();
        TopoDS_Shape target;
        if (stepPath.empty()) {
            target = smallRelief();
        } else {
            std::cout << "reading_step=" << stepPath << std::endl;
            STEPControl_Reader reader;
            require(reader.ReadFile(stepPath.c_str()) == IFSelect_RetDone, "STEP read failed");
            require(reader.TransferRoots() > 0, "STEP root transfer failed");
            target = reader.OneShape();
            require(!target.IsNull(), "STEP contains no target shape");
            std::cout << "step_transfer_ms="
                << std::chrono::duration<double, std::milli>(Clock::now() - start).count() << std::endl;
        }
        Bnd_Box bounds;
        BRepBndLib::Add(target, bounds, Standard_False);
        require(!bounds.IsVoid() && !bounds.IsOpen(), "target has no finite bounds");
        double xmin, ymin, zmin, xmax, ymax, zmax;
        bounds.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        require(std::isfinite(stockMargin) && stockMargin >= 0 && stockMargin <= 1000,
            "stock margin must be within [0,1000]");
        TopoDS_Shape stock;
        if (params.mode == EngravingMachiningMode::StockRoughingAndFinishing) {
            stock = BRepPrimAPI_MakeBox(gp_Pnt(xmin - stockMargin, ymin - stockMargin, zmin - stockMargin),
                gp_Pnt(xmax + stockMargin, ymax + stockMargin, zmax + stockMargin)).Shape();
        }
        std::size_t faceCount = 0, solidCount = 0;
        for (TopExp_Explorer faces(target, TopAbs_FACE); faces.More(); faces.Next()) ++faceCount;
        for (TopExp_Explorer solids(target, TopAbs_SOLID); solids.More(); solids.Next()) ++solidCount;
        std::cout << std::setprecision(10) << "faces=" << faceCount << " solids=" << solidCount
            << " bounds=[" << xmin << ',' << ymin << ',' << zmin << ";"
            << xmax << ',' << ymax << ',' << zmax << "] load_ms="
            << std::chrono::duration<double, std::milli>(Clock::now() - start).count() << std::endl;
        const auto generationStart = Clock::now();
        ToolpathGenerationControl control;
        control.isCancellationRequested = [&] {
            return std::chrono::duration<double>(Clock::now() - generationStart).count() >= timeoutSeconds;
        };
        control.reportProgress = [](std::size_t completed, std::size_t total) {
            if (completed == 0 || completed == total || completed % 100u == 0)
                std::cout << "rows=" << completed << '/' << total << std::endl;
        };
        const auto result = SurfaceFinishing().generate(target, frame, params, &control,
            stock.IsNull() ? nullptr : &stock);
        const auto meshingParallelism = toolpathOcctParallelStatistics();
        std::cout << "occt_parallel_admitted=" << meshingParallelism.admitted
            << " occt_backend_denied=" << meshingParallelism.backendDenied
            << " occt_budget_denied=" << meshingParallelism.budgetDenied << '\n';
        validateResult(result, positiveZ, frame);
        if (!stock.IsNull()) {
            const double stockTop = positiveZ ? zmax + stockMargin : -zmin + stockMargin;
            require(result.safePlaneOffset >= stockTop + params.safeClearance - 1.e-6,
                "stock engraving safe plane does not clear the stock");
            require(result.metrics.roughingLayerCount > 0 && result.metrics.roughingMoveCount > 0 &&
                result.metrics.roughingMoveCount < result.moves.size(),
                "stock engraving did not include both roughing and finishing");
        }
        if (!output.empty()) writeOutputs(output, result, frame);
        std::cout << "PASS: relief engraving follows the requested axis and respects budgets.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}

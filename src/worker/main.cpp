#include "DefaultToolLibrary.h"
#include "support/Files.h"
#include "support/Utf8Path.h"
#include <algorithm>
#include <array>
#include "support/Json.h"
#include <vector>
#include <string>
#include "analysis/AnalysisPipeline.h"
#include "Project.h"
#include "NcReview.h"
#include "Planning.h"
#include "Progress.h"
#include "recognition/FeatureRecognizer.h"
#include "tooling/adapter/hole/HoleMachiningInputMapper.h"
#include "tooling/adapter/hole/HoleAutomaticToolpath.h"
#include "tooling/library/ToolLibraryStorage.h"
#include "toolpath/core/ToolpathParallel.h"
#include "toolpath/workflow/hole/HoleToolpathService.h"
#include "toolpath/program/ToolpathProgramExport.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFApp_Application.hxx>
#include <iostream>
#include <stdexcept>
#include <set>

void oap::worker::reportStage(const char* name)
{
    std::cout << JsonDocument(JsonObject{{"stage", name}}).toJson(JsonDocument::Compact) << std::endl;
}

namespace {
using namespace oap;

void writeJson(const std::string& path, const oap::JsonDocument& json)
{
    const std::string data = json.toJson(oap::JsonDocument::Compact);
    if (!oap::writeFileAtomically(path, data))
        throw std::runtime_error("Cannot save worker result");
}

oap::JsonArray xyz(const gp_Pnt& p)
{
    return {p.X(), p.Y(), p.Z()};
}

void writeMesh(const StepIndexer& index, const TopoDS_Shape& part)
{
    BRepMesh_IncrementalMesh mesh(part, 0.15);
    oap::JsonArray faces;
    index.forEachFace([&](const std::string& id, const TopoDS_Face& face) {
        TopLoc_Location location;
        const auto triangles = BRep_Tool::Triangulation(face, location);
        if (triangles.IsNull())
            return;
        oap::JsonArray positions, indices;
        for (int n = 1; n <= triangles->NbNodes(); ++n) {
            const auto p = triangles->Node(n).Transformed(location.Transformation());
            positions.append(p.X());
            positions.append(p.Y());
            positions.append(p.Z());
        }
        for (int n = 1; n <= triangles->NbTriangles(); ++n) {
            int a, b, c;
            triangles->Triangle(n).Get(a, b, c);
            if (face.Orientation() == TopAbs_REVERSED)
                std::swap(b, c);
            indices.append(a - 1);
            indices.append(b - 1);
            indices.append(c - 1);
        }
        faces.append(oap::JsonObject{{"id", id},
                                 {"positions", positions}, {"indices", indices}});
    });
    writeJson("mesh.json", oap::JsonDocument(faces));
}

struct Document {
    Handle(TDocStd_Document) value;
    ~Document() { if (!value.IsNull()) XCAFApp_Application::GetApplication()->Close(value); }
};

double positive(const oap::JsonObject& request, const char* key)
{
    const double value = request.value(key).toDouble();
    if (!(value > 0.0 && value <= 100000.0))
        throw std::runtime_error(std::string("Invalid ") + key);
    return value;
}

oap::JsonObject generate(const RecognizedHoleGeometry& hole, const oap::JsonObject& request,
                     const Model& model, toolpath::HoleRapidPlanningContext context,
                     const tooling::ToolLibrary& library)
{
    Bbox bounds;
    for (const auto& unit : model.units)
        bounds.merge(unit.bbox);
    const auto frame = MoldCoordinateFrame::fromBounds(bounds);
    context.moldFrame = &frame;
    const auto input = tooling::HoleMachiningInputMapper().resolve(hole, bounds, frame);
    toolpath::ToolpathParams params;
    const auto toolId = request.value("tool").toString();
    const auto* bit = library.toolById(toolId);
    const auto* slot = library.slotForTool(toolId);
    if (!bit || !slot)
        throw std::runtime_error("Unknown or unassigned tool in the selected library");
    params.tool.diameter = bit->geometry.diameter;
    params.tool.vertFeed = positive(request, "feed");
    params.tool.horizFeed = params.tool.vertFeed;
    params.tool.spindleSpeed = positive(request, "rpm");
    auto selectedSlot = *slot;
    selectedSlot.cuttingDefaults.verticalFeed = params.tool.vertFeed;
    selectedSlot.cuttingDefaults.horizontalFeed = params.tool.horizFeed;
    selectedSlot.cuttingDefaults.spindleSpeed = params.tool.spindleSpeed;
    tooling::ToolLibrary selectedLibrary(library.id(), library.name());
    if (!selectedLibrary.upsertTool(*bit) || !selectedLibrary.assignToolToSlot(selectedSlot))
        throw std::runtime_error("Cannot assign selected cutter");
    const std::vector<toolpath::HoleMachiningInput> inputs{input};
    const auto generated = tooling::HoleAutomaticToolpath().generate(selectedLibrary, inputs, params, &context);
    const auto& batch = generated.batch;
    const auto program = batch.toProgram();
    if (program.empty() || !batch.validation.ok())
        throw std::runtime_error("Hole generation failed: " + batch.validation.summary() +
            (batch.diagnostics.empty() ? "" : " " + batch.diagnostics.back().message));
    std::string error;
    const auto frames = toolpath::MoldSetupWorkFrames::fromMold(bounds, frame);
    const auto* setup = frames.forFace(program.operations.front().setupFace);
    if (!setup || !setup->valid)
        throw std::runtime_error("No valid work frame for this hole");
    toolpath::GCodeProgramOptions options;
    options.programName = "OpenAutoPath hole " + request.value("hole").toString();
    options.toolId = bit->id;
    options.toolName = bit->name;
    worker::reportStage("exporting_nc");
    if (!toolpath::ToolpathProgramExport().writeGCode(program, batch.effectiveParams,
            setup->machiningSetup(), options, "reference.nc", error))
        throw std::runtime_error(error);
    oap::JsonArray segments;
    oap::JsonArray operationIds;
    for (const auto& op : program.operations) {
        operationIds.append(op.operationId);
        for (const auto& move : op.moves)
            segments.append(oap::JsonArray{xyz(move.from), xyz(move.to)});
    }
    writeJson("path.json", oap::JsonDocument(segments));
    worker::captureProject(request, generated, inputs, frames, library);
    worker::reportStage("reviewing_nc");
    oap::JsonArray issues;
    for (const auto& issue : batch.validation.issues) {
        issues.append(oap::JsonObject{{"code", issue.code},
                                  {"message", issue.message}});
    }
    oap::JsonArray warnings;
    if (!batch.coverage.complete())
        warnings.append(oap::JsonObject{{"line", 0}, {"message", std::string(batch.coverage.summary())}});
    if (!batch.productionSafety.complete())
        warnings.append(oap::JsonObject{{"line", 0}, {"message", "刀具装配实体扫掠审计未完成"}});
    const auto ncErrors = worker::reviewSingleNc(toolpath::workCoordinateCode(setup->workOffset), bit->name,
                                                 operationIds, warnings);
    return {{"singleNcReview", true}, {"ncErrors", ncErrors}, {"hole", request.value("hole")}, {"tool", bit->name}, {"toolId", bit->id},
            {"feed", params.tool.vertFeed}, {"rpm", params.tool.spindleSpeed},
            {"validation", std::string(batch.validation.summary())}, {"issues", issues},
            {"coverageComplete", batch.coverage.complete()},
            {"sweepComplete", batch.productionSafety.complete()}, {"moves", segments.size()}};
}

void run(const std::string& fileName)
{
    std::string requestBytes, requestError;
    if (!oap::readFile(fileName, requestBytes, &requestError))
        throw std::runtime_error(requestError);
    const auto request = oap::JsonDocument::fromJson(requestBytes).object();
    const auto action = request.value("action").toString();
    static constexpr std::string_view projectActions[]{"project", "rebuild", "simulate", "replay", "stock",
                                     "setups",  "export",  "holes",    "plan"};
    const bool projectAction = std::ranges::find(projectActions, action) != std::end(projectActions);
    if (request.value("version").toInt() != 1 ||
        (!projectAction && action != "analyze" && action != "generate" && action != "features" &&
         action != "library"))
        throw std::runtime_error("Unsupported worker request");
    tooling::ToolLibrary library;
    std::string libraryError;
    const auto libraryPath = request["libraryFile"].toString();
    const bool loaded = action == "library" && request.contains("document")
        ? tooling::ToolLibraryStorage::deserializeLibrary(
              oap::JsonDocument(request["document"].toObject()).toJson(), library, &libraryError)
        : libraryPath.empty()
            ? tooling::ToolLibraryStorage::deserializeLibrary(
                  std::string(oap::worker::defaultToolLibrary), library, &libraryError)
            : tooling::ToolLibraryStorage::loadLibrary(libraryPath, library, &libraryError);
    if (!loaded) throw std::runtime_error(libraryError);
    if (action == "library" || action == "analyze") {
        if (!tooling::ToolLibraryStorage::saveLibrary("library.oaptl", library, &libraryError))
            throw std::runtime_error(libraryError);
    }
    if (action == "library") {
        writeJson("result.json", oap::JsonDocument(oap::JsonObject{{"version", 1}, {"name", library.name()}}));
        return;
    }
    AnalysisPipelineControl progress;
    if (toolpath::toolpathHardwareParallelism() > 1) {
        progress.index.executeRange = [](std::size_t count, const auto& task) {
            toolpath::toolpathParallelForBudgeted(count, task);
        };
    }
    if (request["cacheFaceBounds"].toBool())
        progress.index.faceBoundsCachePath = request["model"].toString() + ".bounds";
    progress.reportStage = [](AnalysisStage stage) {
        switch (stage) {
        case AnalysisStage::Reading: worker::reportStage("reading_step"); break;
        case AnalysisStage::Transferring: worker::reportStage("transferring_step"); break;
        case AnalysisStage::Indexing: worker::reportStage("indexing"); break;
        case AnalysisStage::PreparingPreview: worker::reportStage("previewing"); break;
        }
    };
    Model model;
    Document document;
    StepIndexer index;
    const auto report = AnalysisPipeline().execute(request.value("model").toString(),
                                                   document.value, index, model, &progress);
    if (!report.ok())
        throw std::runtime_error("STEP analysis failed");
    BRep_Builder builder;
    TopoDS_Compound part;
    builder.MakeCompound(part);
    toolpath::HoleRapidPlanningContext context;
    for (const auto& unit : model.units) {
        if (const auto* shape = index.shapeForUnit(unit.unit_id)) {
            builder.Add(part, *shape);
            context.collisionScene.addBody(unit.unit_id, toolpath::CollisionBodyRole::Workpiece, *shape);
        }
    }
    context.obstacleShape = part;
    if (projectAction)
    {
        worker::reportStage("preparing_project");
        Bbox bounds;
        for (const auto& unit : model.units) bounds.merge(unit.bbox);
        const auto frame = MoldCoordinateFrame::fromBounds(bounds);
        const auto frames = toolpath::MoldSetupWorkFrames::fromMold(bounds, frame);
        context.moldFrame = &frame;
        auto project = worker::processProject(request, index, model, library, frames, context);
        project.insert("version", 1);
        writeJson("result.json", oap::JsonDocument(project));
        return;
    }
    worker::reportStage("recognizing_holes");
    const auto geometries = FeatureRecognizer().recognizeHoleGeometries(model, index).geometries;
    oap::JsonObject result;
    if (action == "analyze" || action == "features") {
        const auto trace = worker::traceRegions(model, index, geometries);
        result.insert("features", worker::planningFeatures(trace));
        oap::JsonArray throughFaces, axes;
        for (const auto &id : std::set<std::string>(trace.throughStructureFaceIds.begin(),
                                                    trace.throughStructureFaceIds.end()))
            throughFaces.append(id);
        for (const auto &axis : trace.axisStats)
            axes.append(oap::JsonObject{{"axis", axis.axisLabel},
                                    {"roots", axis.rootFaceCount},
                                    {"slots", axis.blindSlotPathCount},
                                    {"pockets", axis.blindPocketPathCount},
                                    {"faces", static_cast<std::int64_t>(axis.traceFaceIds.size())}});
        result.insert("recognition", oap::JsonObject{{"throughFaces", throughFaces}, {"axes", axes}});
        if (!trace.ok()) result.insert("featureError", trace.error);
        if (action == "features") {
            result.insert("version", 1);
            writeJson("result.json", oap::JsonDocument(result));
            return;
        }
    }
    oap::JsonArray holes;
    const RecognizedHoleGeometry* selected = nullptr;
    for (const auto& hole : geometries) {
        if (hole.stageWallFaceIds.empty())
            continue;
        const auto id = std::string(hole.stageWallFaceIds.front());
        oap::JsonArray faces;
        for (const auto& face : hole.sourceFaceIds)
            faces.append(face);
        holes.append(oap::JsonObject{{"id", id},
                                 {"diameter", 2 * hole.radius},
                                 {"depth", hole.depth()},
                                 {"faces", faces},
                                 {"kind", hole.kind == FeatureKind::ThroughHole       ? "通孔"
                                          : hole.kind == FeatureKind::CountersinkHole ? "沉孔"
                                                                                      : "盲孔"}});
        if (id == request.value("hole").toString())
            selected = &hole;
    }
    if (action == "analyze") {
        worker::reportStage("meshing");
        writeMesh(index, part);
        result.insert("holes", holes);
        oap::JsonArray units;
        for(const auto& unit:model.units) units.append(oap::JsonObject{{"id",unit.unit_id},
            {"name",unit.decoded_name},{"faces",static_cast<int>(unit.face_ids.size())}});
        result.insert("units",units);
        oap::JsonArray tools;
        for (const auto& item : library.assignedTools()) {
            tools.append(oap::JsonObject{{"id", item.tool.id}, {"name", item.tool.name},
                                     {"diameter", item.tool.geometry.diameter},
                                     {"type", tooling::toolTypeId(item.tool.type)}});
        }
        result.insert("tools", tools);
    } else {
        if (!selected)
            throw std::runtime_error("Unknown hole in this model version");
        worker::reportStage("generating");
        result = generate(*selected, request, model, context, library);
    }
    result.insert("version", 1);
    writeJson("result.json", oap::JsonDocument(result));
}
} // namespace

#ifdef _WIN32
int wmain(int argc, wchar_t** argv)
#else
int main(int argc, char** argv)
#endif
{
    try {
        if (argc != 2)
            throw std::runtime_error("Usage: oap-worker request.json");
        #ifdef _WIN32
        run(oap::pathUtf8(std::filesystem::path(argv[1])));
#else
        run(argv[1]);
#endif
        return 0;
    } catch (const Standard_Failure& failure) {
        std::cerr << failure.GetMessageString() << std::endl;
    } catch (const std::exception& failure) {
        std::cerr << failure.what() << std::endl;
    }
    return 1;
}

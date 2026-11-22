#include "analysis/AnalysisPipeline.h"
#include "Project.h"
#include "Planning.h"
#include "recognition/FeatureRecognizer.h"
#include "tooling/adapter/hole/HoleMachiningInputMapper.h"
#include "tooling/adapter/hole/HoleAutomaticToolpath.h"
#include "tooling/library/ToolLibraryStorage.h"
#include "toolpath/workflow/hole/HoleToolpathService.h"
#include "toolpath/program/ToolpathProgramExport.h"
#include "toolpath/core/setup/MoldSetupWorkFrames.h"

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFApp_Application.hxx>
#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <iostream>
#include <stdexcept>

namespace {
using namespace oap;

void writeJson(const QString& path, const QJsonDocument& json)
{
    QSaveFile file(path);
    const QByteArray data = json.toJson(QJsonDocument::Compact);
    if (!file.open(QIODevice::WriteOnly) || file.write(data) != data.size() || !file.commit())
        throw std::runtime_error("Cannot save worker result");
}

void stage(const char* name)
{
    std::cout << QJsonDocument(QJsonObject{{"stage", name}}).toJson(QJsonDocument::Compact).constData()
              << std::endl;
}

QJsonArray xyz(const gp_Pnt& p)
{
    return {p.X(), p.Y(), p.Z()};
}

void writeMesh(const StepIndexer& index, const TopoDS_Shape& part)
{
    BRepMesh_IncrementalMesh mesh(part, 0.15);
    QJsonArray faces;
    index.forEachFace([&](const std::string& id, const TopoDS_Face& face) {
        TopLoc_Location location;
        const auto triangles = BRep_Tool::Triangulation(face, location);
        if (triangles.IsNull())
            return;
        QJsonArray positions, indices;
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
        faces.append(QJsonObject{{"id", QString::fromStdString(id)},
                                 {"positions", positions}, {"indices", indices}});
    });
    writeJson("mesh.json", QJsonDocument(faces));
}

struct Document {
    Handle(TDocStd_Document) value;
    ~Document() { if (!value.IsNull()) XCAFApp_Application::GetApplication()->Close(value); }
};

double positive(const QJsonObject& request, const char* key)
{
    const double value = request.value(key).toDouble();
    if (!(value > 0.0 && value <= 100000.0))
        throw std::runtime_error(std::string("Invalid ") + key);
    return value;
}

QJsonObject generate(const RecognizedHoleGeometry& hole, const QJsonObject& request,
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
    options.programName = "OpenAutoPath hole " + request.value("hole").toString().toStdString();
    options.toolId = bit->id.toStdString();
    options.toolName = bit->name.toStdString();
    if (!toolpath::ToolpathProgramExport().writeGCode(program, batch.effectiveParams,
            setup->machiningSetup(), options, "reference.nc", error))
        throw std::runtime_error(error);
    QJsonArray segments;
    for (const auto& op : program.operations) {
        for (const auto& move : op.moves)
            segments.append(QJsonArray{xyz(move.from), xyz(move.to)});
    }
    writeJson("path.json", QJsonDocument(segments));
    worker::captureProject(request, generated, inputs, frames, library);
    QJsonArray issues;
    for (const auto& issue : batch.validation.issues) {
        issues.append(QJsonObject{{"code", QString::fromStdString(issue.code)},
                                  {"message", QString::fromStdString(issue.message)}});
    }
    return {{"hole", request.value("hole")}, {"tool", bit->name}, {"toolId", bit->id},
            {"feed", params.tool.vertFeed}, {"rpm", params.tool.spindleSpeed},
            {"validation", QString::fromStdString(batch.validation.summary())}, {"issues", issues},
            {"coverageComplete", batch.coverage.complete()},
            {"sweepComplete", batch.productionSafety.complete()}, {"moves", segments.size()}};
}

void run(const QString& fileName)
{
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error("Cannot open request");
    const auto request = QJsonDocument::fromJson(file.readAll()).object();
    const auto action = request.value("action").toString();
    if (request.value("version").toInt() != 1 || (action != "analyze" && action != "generate" &&
        action != "library" && action != "project" && action != "rebuild" && action != "simulate" && action != "plan"))
        throw std::runtime_error("Unsupported worker request");
    tooling::ToolLibrary library;
    QString libraryError;
    if (action == "library" && request.contains("document")) {
        if (!tooling::ToolLibraryStorage::deserializeLibrary(
                QJsonDocument(request["document"].toObject()).toJson(), library, &libraryError))
            throw std::runtime_error(libraryError.toStdString());
    } else if (!tooling::ToolLibraryStorage::loadLibrary(
                   request["libraryFile"].toString(":/tools/general-simulation.oaptl"), library, &libraryError))
        throw std::runtime_error(libraryError.toStdString());
    if (action == "library" || action == "analyze") {
        if (!tooling::ToolLibraryStorage::saveLibrary("library.oaptl", library, &libraryError))
            throw std::runtime_error(libraryError.toStdString());
    }
    if (action == "library") {
        writeJson("result.json", QJsonDocument(QJsonObject{{"version", 1}, {"name", library.name()}}));
        return;
    }
    stage("analyzing");
    Model model;
    Document document;
    StepIndexer index;
    const auto report = AnalysisPipeline().execute(request.value("model").toString().toStdString(),
                                                   document.value, index, model);
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
    if (action == "project" || action == "rebuild" || action == "simulate" || action == "plan") {
        Bbox bounds;
        for (const auto& unit : model.units) bounds.merge(unit.bbox);
        const auto frame = MoldCoordinateFrame::fromBounds(bounds);
        const auto frames = toolpath::MoldSetupWorkFrames::fromMold(bounds, frame);
        context.moldFrame = &frame;
        auto project = worker::processProject(request, index, model, library, frames, context);
        project.insert("version", 1);
        writeJson("result.json", QJsonDocument(project));
        return;
    }
    const auto geometries = FeatureRecognizer().recognizeHoleGeometries(model, index).geometries;
    QJsonArray holes;
    const RecognizedHoleGeometry* selected = nullptr;
    for (const auto& hole : geometries) {
        if (hole.stageWallFaceIds.empty())
            continue;
        const auto id = QString::fromStdString(hole.stageWallFaceIds.front());
        QJsonArray faces;
        for (const auto& face : hole.sourceFaceIds)
            faces.append(QString::fromStdString(face));
        holes.append(QJsonObject{{"id", id}, {"diameter", 2 * hole.radius},
                                 {"depth", hole.depth()}, {"faces", faces}});
        if (id == request.value("hole").toString())
            selected = &hole;
    }
    QJsonObject result;
    if (action == "analyze") {
        stage("meshing");
        writeMesh(index, part);
        result.insert("holes", holes);
        const auto trace=worker::traceRegions(model,index,geometries);
        result.insert("features",worker::planningFeatures(trace));
        if(!trace.ok()) result.insert("featureError",QString::fromStdString(trace.error));
        QJsonArray units;
        for(const auto& unit:model.units) units.append(QJsonObject{{"id",QString::fromStdString(unit.unit_id)},
            {"name",QString::fromStdString(unit.decoded_name)},{"faces",static_cast<int>(unit.face_ids.size())}});
        result.insert("units",units);
        QJsonArray tools;
        for (const auto& item : library.assignedTools()) {
            tools.append(QJsonObject{{"id", item.tool.id}, {"name", item.tool.name},
                                     {"diameter", item.tool.geometry.diameter},
                                     {"type", tooling::toolTypeId(item.tool.type)}});
        }
        result.insert("tools", tools);
    } else {
        if (!selected)
            throw std::runtime_error("Unknown hole in this model version");
        stage("generating");
        result = generate(*selected, request, model, context, library);
    }
    result.insert("version", 1);
    writeJson("result.json", QJsonDocument(result));
}
} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    try {
        if (app.arguments().size() != 2)
            throw std::runtime_error("Usage: oap-worker request.json");
        run(app.arguments().at(1));
        return 0;
    } catch (const Standard_Failure& failure) {
        std::cerr << failure.GetMessageString() << std::endl;
    } catch (const std::exception& failure) {
        std::cerr << failure.what() << std::endl;
    }
    return 1;
}

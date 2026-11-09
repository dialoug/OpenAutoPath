#include "app/workflows/engraving/EngravingToolpathGenerationUseCase.h"
#include "app/machining/MachiningWorkspaceCoordinator.h"
#include "app/document/ModelDocumentWorkspaceCoordinator.h"
#include "app/workflows/project/EngravingSettingsStorage.h"
#include "app/workflows/project/MachiningProjectInstaller.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "manufacturing/job/MachiningOperationService.h"
#include "manufacturing/ipw/IpwExecutor.h"
#include "toolpath/core/ToolpathPrimitiveMapping.h"
#include "tooling/library/ToolLibraryStorage.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <QCoreApplication>
#include <QTemporaryDir>
#include <QSettings>
#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <limits>
#include <stdexcept>

namespace {
namespace a = oap::app;
namespace m = oap::manufacturing;
namespace t = oap::toolpath;
static_assert(static_cast<int>(a::EngravingSetupScope::AllSixFaces) == 0 &&
              static_cast<int>(a::EngravingSetupScope::PositiveZ) == 1 &&
              static_cast<int>(a::EngravingSetupScope::PreserveProjectFaces) == 2,
              "existing engraving preference values must remain compatible");
void require(bool value, const std::string& message)
{
    if (!value) throw std::runtime_error(message);
}
t::ToolpathToolContext singleTool()
{
    t::ToolpathToolContext value;
    value.valid = true;
    value.toolNumber = 3;
    value.toolType = oap::tooling::toolTypeId(oap::tooling::ToolType::BallEndMill).toStdString();
    value.diameter = 2;
    value.cuttingEdgeLength = 12;
    value.overallLength = 25;
    value.spindleSpeed = 8000;
    value.horizontalFeed = 600;
    value.verticalFeed = 150;
    return value;
}
void addTool(oap::tooling::ToolLibrary& library, int number, double diameter, oap::tooling::ToolType type)
{
    oap::tooling::ToolBit tool;
    tool.id = QStringLiteral("tool-%1").arg(number);
    tool.name = tool.id;
    tool.type = type;
    tool.geometry.diameter = diameter;
    tool.geometry.cuttingEdgeLength = 12;
    tool.geometry.overallLength = 25;
    if (type == oap::tooling::ToolType::BallEndMill) tool.geometry.cornerRadius = diameter / 2;
    QString error;
    require(library.upsertTool(tool, &error), error.toStdString());
    oap::tooling::LibraryToolSlot slot;
    slot.toolId = tool.id;
    slot.toolNumber = number;
    slot.cuttingDefaults.spindleSpeed = 8000;
    require(library.assignToolToSlot(slot, &error), error.toStdString());
}
void installRoughing(m::MachiningJob& job)
{
    auto& def = job.definition();
    def.finishingOperations.clear();
    job.clearFinishingResult();
    def.toolControllers.clear();
    auto tool = singleTool();
    tool.toolNumber = 1;
    tool.toolId = "rough-tool";
    tool.toolType = oap::tooling::toolTypeId(oap::tooling::ToolType::EndMill).toStdString();
    tool.diameter = 6;
    tool.cornerRadius = 0;
    def.toolControllers.push_back({"rough-tool", "rough-library", tool});
    m::PocketOperationDefinition rough;
    rough.id = "rough";
    rough.setupId = def.setups.front().id;
    rough.toolControllerId = "rough-tool";
    rough.algorithmId = "builtin.pocket.freecad";
    rough.target.sourceFaceIds = {"fixture-top"};
    rough.target.floorFaceId = "fixture-top";
    rough.target.geometryFingerprint = "fixture-top-v1";
    rough.state = m::JobOperationState::Generated;
    rough.inputFingerprint = "fixture-rough-v1";
    rough.sequence = 0;
    def.pocketOperations.push_back(rough);
    t::PocketToolpathBatch batch;
    batch.effectiveParams.tool.diameter = 6;
    batch.effectiveParams.tool.cuttingEdgeLength = 12;
    t::PocketToolpathOperation operation;
    operation.operationId = rough.id;
    operation.algorithmId = rough.algorithmId;
    operation.setupFace = oap::MoldSetupFace::PositiveZ;
    operation.outwardToolAxis = gp_Vec(0, 0, 1);
    operation.tool = tool;
    operation.clearing.ok = true;
    t::ToolpathMove move;
    move.type = t::ToolpathMoveType::Cut;
    move.from = gp_Pnt(1, 4, 3.5);
    move.to = gp_Pnt(11, 4, 3.5);
    move.feed = 600;
    operation.clearing.moves.push_back(move);
    operation.clearing.primitives.push_back(t::makeLinearPrimitive(move));
    batch.operations.push_back(std::move(operation));
    job.setPocketResult(std::move(batch), m::MachiningJobStorage::pocketExecutionFingerprint(def).toStdString());
}

// Opt-in desktop-path reproduction; normal CTest invocation remains synthetic.
int diagnoseStep(const QStringList& arguments)
{
    QString stepPath;
    QString libraryPath = QStringLiteral("resources/tool-libraries/general-simulation.oaptl");
    QString requestedScope = QStringLiteral("-Z");
    for (qsizetype index = 1; index < arguments.size(); ++index) {
        const auto key = arguments[index];
        require(index + 1 < arguments.size(), "diagnostic option requires a value: " + key.toStdString());
        const auto value = arguments[++index];
        if (key == "--step") stepPath = value;
        else if (key == "--library") libraryPath = value;
        else if (key == "--scope") requestedScope = value;
        else throw std::runtime_error("unknown diagnostic option: " + key.toStdString());
    }
    require(!stepPath.isEmpty(), "diagnostic usage: --step <path> [--library <oaptl>] [--scope -Z|+Z|-X|+X|-Y|+Y|all]");
    const std::map<QString, std::pair<a::EngravingSetupScope, oap::MoldSetupFace>> scopes{
        {"-Z", {a::EngravingSetupScope::NegativeZ, oap::MoldSetupFace::NegativeZ}},
        {"+Z", {a::EngravingSetupScope::PositiveZ, oap::MoldSetupFace::PositiveZ}},
        {"-X", {a::EngravingSetupScope::NegativeX, oap::MoldSetupFace::NegativeX}},
        {"+X", {a::EngravingSetupScope::PositiveX, oap::MoldSetupFace::PositiveX}},
        {"-Y", {a::EngravingSetupScope::NegativeY, oap::MoldSetupFace::NegativeY}},
        {"+Y", {a::EngravingSetupScope::PositiveY, oap::MoldSetupFace::PositiveY}},
        {"all", {a::EngravingSetupScope::AllSixFaces, oap::MoldSetupFace::Unresolved}}};
    const auto selection = scopes.find(requestedScope);
    require(selection != scopes.end(), "invalid diagnostic scope: " + requestedScope.toStdString());
    std::cout << std::setprecision(17) << "step=" << stepPath.toStdString()
        << "\nlibrary=" << libraryPath.toStdString() << "\nscope=" << requestedScope.toStdString()
        << "\nstage=STEPCAF-read\n" << std::flush;
    const auto start = std::chrono::steady_clock::now();
    Handle(TDocStd_Document) document;
    const auto application = XCAFApp_Application::GetApplication();
    application->NewDocument("BinXCAF", document);
    struct CloseDocument {
        Handle(TDocStd_Document) document;
        ~CloseDocument() { XCAFApp_Application::GetApplication()->Close(document); }
    } close{document};
    STEPCAFControl_Reader reader;
    require(reader.ReadFile(stepPath.toUtf8().constData()) == IFSelect_RetDone && reader.Transfer(document),
        "STEPCAF failed to load diagnostic target");
    std::cout << "stage=StepIndexer-build\n" << std::flush;
    oap::StepIndexer indexer;
    oap::Model model;
    require(indexer.build(document, model) && !model.units.empty(), "StepIndexer failed to analyze diagnostic target");
    oap::Bbox bounds;
    for (const auto& unit : model.units) {
        require(unit.bbox.valid(), "indexed unit has invalid bounds: " + unit.unit_id);
        bounds.merge(unit.bbox);
    }
    const auto mold = oap::MoldCoordinateFrame::fromModel(model);
    require(mold.isValid, "model did not yield a valid desktop mold coordinate frame");
    auto frames = t::MoldSetupWorkFrames::fromMold(bounds, mold);
    require(frames.isValid(), "desktop setup frames are invalid");
    const auto printXyz = [](const char* name, const auto& xyz) {
        std::cout << name << '=' << xyz.X() << ',' << xyz.Y() << ',' << xyz.Z() << '\n';
    };
    const auto spans = bounds.size();
    std::cout << "units=" << model.units.size() << " faces=" << model.faces.size()
        << "\nmodel.bounds=" << bounds.xmin << ',' << bounds.ymin << ',' << bounds.zmin << ','
        << bounds.xmax << ',' << bounds.ymax << ',' << bounds.zmax
        << "\nmodel.spans=" << spans[0] << ',' << spans[1] << ',' << spans[2]
        << "\nmold.dimensions=" << mold.length << ',' << mold.width << ',' << mold.thickness << '\n';
    printXyz("mold.origin", mold.origin);
    printXyz("mold.X", mold.xAxis);
    printXyz("mold.Y", mold.yAxis);
    printXyz("mold.Z", mold.zAxis);
    for (int index = 0; index < 6; ++index) {
        const auto face = static_cast<oap::MoldSetupFace>(index);
        if (selection->second.second != oap::MoldSetupFace::Unresolved && face != selection->second.second) continue;
        const auto* frame = frames.forFace(face);
        require(frame && frame->valid, "selected setup is unavailable");
        std::cout << "setup.face=" << oap::moldSetupFaceName(face) << " mapping=" << frame->axisMapping << '\n';
        printXyz("setup.origin", frame->setupWorkFrame.Location());
        printXyz("setup.X", frame->setupWorkFrame.XDirection());
        printXyz("setup.Y", frame->setupWorkFrame.YDirection());
        printXyz("setup.Z", frame->setupWorkFrame.Direction());
    }
    oap::tooling::ToolLibrary library;
    QString error;
    require(oap::tooling::ToolLibraryStorage::loadLibrary(libraryPath, library, &error), error.toStdString());
    a::EngravingToolpathGenerationRequest request;
    for (const auto& unit : model.units) {
        const auto* shape = indexer.shapeForUnit(unit.unit_id);
        require(shape && !shape->IsNull(), "indexed diagnostic unit is missing: " + unit.unit_id);
        request.targets.push_back(*shape);
        request.unitIds.push_back(unit.unit_id);
    }
    request.definition.sourcePath = stepPath;
    request.definition.activeToolLibraryId = library.id();
    request.setupFrames = &frames;
    request.library = &library;
    request.settings.scope = selection->second.first;
    request.settings.toolMode = a::EngravingToolMode::AutomaticLibrary;
    request.settings.parameters.mode = t::EngravingMachiningMode::StockRoughingAndFinishing;
    request.settings.parameters.maxStepDown = 1;
    request.settings.parameters.roughingAllowance = 0.2;
    request.settings.parameters.stepOver = 0.4;
    request.settings.parameters.sampleSpacing = 0.25;
    request.settings.parameters.meshDeflection = 0.03;
    const auto analyzed = std::chrono::steady_clock::now();
    std::cout << "analysis.seconds=" << std::chrono::duration<double>(analyzed - start).count()
        << "\nparams=stock-roughing-and-finishing ap=1 roughingAllowance=0.2 stepOver=0.4 sampleSpacing=0.25 meshDeflection=0.03"
        << "\nstock=per-unit-boxes margins=0,0,0\nstage=desktop-engraving-usecase\n" << std::flush;
    const auto generated = a::EngravingToolpathGenerationUseCase::execute(request);
    std::cout << "tool.valid=" << generated.tool.valid << " T=" << generated.tool.toolNumber
        << " id=" << generated.tool.toolId << " name=" << generated.tool.toolName
        << " type=" << generated.tool.toolType << " diameter=" << generated.tool.diameter
        << " cuttingLength=" << generated.tool.cuttingEdgeLength << " cornerRadius=" << generated.tool.cornerRadius
        << " feed=" << generated.tool.horizontalFeed << " plunge=" << generated.tool.verticalFeed
        << "\ngeneration.seconds=" << std::chrono::duration<double>(std::chrono::steady_clock::now() - analyzed).count()
        << "\nresult.ok=" << generated.ok << " cancelled=" << generated.cancelled
        << "\nresult.error=" << generated.error.toStdString() << '\n';
    for (const auto& operation : generated.batch.operations)
        std::cout << "result.face=" << oap::moldSetupFaceName(operation.setupFace)
            << " layers=" << operation.result.metrics.roughingLayerCount
            << " moves=" << operation.result.moves.size() << " safeZ=" << operation.result.safePlaneOffset
            << " triangles=" << operation.result.metrics.triangleCount
            << " workers=" << operation.result.metrics.projectionWorkerCount
            << " samples=" << operation.result.metrics.sampleCount
            << " mesh.ms=" << operation.result.metrics.meshMilliseconds
            << " projection.ms=" << operation.result.metrics.projectionMilliseconds
            << " total.ms=" << operation.result.metrics.totalMilliseconds << '\n';
    std::cout << std::flush;
    return generated.ok ? 0 : 1;
}
}

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    using oap::MoldSetupFace;
    try {
        if (argc > 1) return diagnoseStep(QCoreApplication::arguments());
        QTemporaryDir directory;
        require(directory.isValid(), "temporary directory unavailable");
        QSettings::setDefaultFormat(QSettings::IniFormat);
        QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, directory.path());
        const auto target = BRepPrimAPI_MakeBox(12, 8, 3).Shape();
        const auto source = directory.filePath("direct-model.step");
        STEPControl_Writer writer;
        require(writer.Transfer(target, STEPControl_AsIs) == IFSelect_RetDone &&
            writer.Write(source.toUtf8().constData()) == IFSelect_RetDone, "fixture STEP write failed");
        const oap::Bbox bounds{0, 0, 0, 12, 8, 3};
        auto frames = t::MoldSetupWorkFrames::fromMold(bounds, oap::MoldCoordinateFrame::fromBounds(bounds));
        for (int face = 0; face < 6; ++face) frames.setConfirmed(static_cast<MoldSetupFace>(face), true);
        a::EngravingToolpathGenerationRequest request;
        request.targets = {target};
        request.unitIds = {"unit-1"};
        request.setupFrames = &frames;
        request.definition.sourcePath = source;
        request.settings.scope = a::EngravingSetupScope::PositiveZ;
        request.settings.toolMode = a::EngravingToolMode::SingleTool;
        request.settings.manualTool = singleTool();
        request.settings.parameters.stepOver = 1;
        request.settings.parameters.sampleSpacing = 1;
        std::vector<bool> busy;
        request.interaction.setBusy = [&](bool state) { busy.push_back(state); };
        auto top = a::EngravingToolpathGenerationUseCase::execute(request);
        require(top.ok, top.error.toStdString());
        require(busy == std::vector<bool>({true, false}), "worker did not balance busy state");
        require(top.batch.operations.size() == 1 && top.batch.operations.front().setupFace == MoldSetupFace::PositiveZ &&
            top.libraryId.empty() && top.tool.toolNumber == 3 && top.tool.cornerRadius == 1,
            "direct STEP +Z single-tool engraving requires an unrelated feature/library or lost tool binding");

        const std::array<std::pair<a::EngravingSetupScope, MoldSetupFace>, 6> directions{{
            {a::EngravingSetupScope::PositiveZ, MoldSetupFace::PositiveZ},
            {a::EngravingSetupScope::NegativeX, MoldSetupFace::NegativeX},
            {a::EngravingSetupScope::PositiveX, MoldSetupFace::PositiveX},
            {a::EngravingSetupScope::NegativeY, MoldSetupFace::NegativeY},
            {a::EngravingSetupScope::PositiveY, MoldSetupFace::PositiveY},
            {a::EngravingSetupScope::NegativeZ, MoldSetupFace::NegativeZ}}};
        for (const auto& [scope, face] : directions) {
            auto directionRequest = request;
            directionRequest.settings.scope = scope;
            const auto generated = a::EngravingToolpathGenerationUseCase::execute(directionRequest);
            require(generated.ok && generated.batch.operations.size() == 1,
                "single-face engraving failed: " + generated.error.toStdString());
            const auto& operation = generated.batch.operations.front();
            const auto& expectedFrame = frames.forFace(face)->setupWorkFrame;
            require(operation.setupFace == face &&
                operation.workFrame.Location().Distance(expectedFrame.Location()) < 1.e-9 &&
                operation.workFrame.Direction().IsEqual(expectedFrame.Direction(), 1.e-9) &&
                operation.workFrame.XDirection().IsEqual(expectedFrame.XDirection(), 1.e-9),
                "single-face engraving did not bind its corresponding setup work frame");
            std::size_t rapidCount = 0;
            for (const auto& move : operation.result.moves) {
                if (move.type != t::ToolpathMoveType::Rapid) continue;
                ++rapidCount;
                const auto localZ = [&](const gp_Pnt& point) {
                    return gp_Vec(expectedFrame.Location(), point).Dot(gp_Vec(expectedFrame.Direction()));
                };
                require(std::abs(localZ(move.from) - operation.result.safePlaneOffset) < 1.e-7 &&
                    std::abs(localZ(move.to) - operation.result.safePlaneOffset) < 1.e-7,
                    "single-face engraving geometry used a different machining axis");
            }
            require(rapidCount > 0, "single-face fixture did not exercise machining-axis clearances");
            a::MachiningWorkspaceSession workspace;
            a::MachiningConfigurationSession configuration;
            a::ToolLibraryController libraries;
            a::MachiningProjectInstaller::install(a::MachiningProjectController::restoreSnapshot(generated.job),
                workspace, configuration, libraries, frames);
            require(configuration.engravingSettings().scope == scope &&
                configuration.engravingSettings().projectFaces == std::vector<MoldSetupFace>{face},
                "reopened single-face engraving changed its explicit direction scope");
        }
        QSettings directionPreferences(directory.filePath("directions.ini"), QSettings::IniFormat);
        for (int value = 0; value <= 7; ++value) {
            auto saved = request.settings;
            saved.scope = static_cast<a::EngravingSetupScope>(value);
            saved.projectFaces = {MoldSetupFace::PositiveX, MoldSetupFace::NegativeZ};
            QString error;
            a::EngravingSettings reloaded;
            require(a::EngravingSettingsStorage::save(directionPreferences, saved, &error) &&
                a::EngravingSettingsStorage::load(directionPreferences, reloaded, &error) &&
                a::EngravingSettingsStorage::equal(saved, reloaded),
                "engraving direction preference roundtrip failed: " + error.toStdString());
        }
        const auto topId = top.job.definition().finishingOperations.front().id;
        a::MachiningWorkspaceCoordinator coordinator;
        a::ModelDocumentWorkspaceCoordinator document(directory.filePath("cache").toStdString());
        auto edited = coordinator.configurationEditorState(frames, 12, 8);
        edited.engraving = request.settings;
        const auto applied = coordinator.applyConfiguration(edited, document);
        require(applied.engravingParametersChanged && applied.persistenceError.isEmpty(),
            "common parameter configuration did not persist engraving settings");
        a::MachiningWorkspaceCoordinator restored;
        require(restored.loadConfigurationDefaults().isEmpty() &&
            a::EngravingSettingsStorage::equal(restored.engravingSettings(), request.settings),
            "engraving single-tool configuration did not survive local settings reload");
        const auto workspaceResult = restored.generateEngraving(request);
        require(workspaceResult.ok && restored.hasFinishingPreview() && restored.status().finishingOperationCount == 1,
            "workspace did not install configured engraving: " + workspaceResult.error.toStdString());
        auto pendingEdit = restored.configurationEditorState(frames, 12, 8);
        pendingEdit.engraving.parameters.stepOver = 0.5;
        const auto pendingApplied = restored.applyConfiguration(pendingEdit, document);
        require(pendingApplied.engravingParametersChanged && restored.hasPendingRegeneration() &&
            restored.activePreviewNeedsRegeneration(), "service parameter apply did not mark engraving pending");
        require(!restored.hasFinishingPreview() && !restored.prepareToolpathExport().ok && restored.hasProjectContent(),
            "changed engraving parameters left old paths exportable or discarded the project definition");
        QString pendingSaveError;
        a::MachiningProjectSaveRequest pendingSave;
        pendingSave.definition = request.definition;
        pendingSave.setupFrames = &frames;
        require(!restored.synchronizeProjectForSave(pendingSave, &pendingSaveError) &&
            !restored.saveProject(directory.filePath("pending.oapjob"), &pendingSaveError) &&
            restored.regenerateProject({}).failure == a::MachiningProjectLoadFailure::PendingConfiguration,
            "pending engraving edits were silently saved/regenerated using the old formal definition");
        auto cancelledPendingRequest = request;
        cancelledPendingRequest.interaction.showProgress = [](const a::WorkflowProgressRequest& progress) { progress.cancel(); };
        require(restored.generateEngraving(cancelledPendingRequest).cancelled && restored.hasPendingRegeneration(),
            "cancellation cleared pending engraving edits");
        auto invalidPendingRequest = request;
        invalidPendingRequest.targets.clear();
        require(!restored.generateEngraving(invalidPendingRequest).ok && restored.hasPendingRegeneration(),
            "failed generation cleared pending engraving edits");
        require(restored.generateEngraving(request).ok && !restored.hasPendingRegeneration() &&
            restored.synchronizeProjectForSave(pendingSave, &pendingSaveError) &&
            restored.saveProject(directory.filePath("pending.oapjob"), &pendingSaveError),
            "successful regeneration did not make current engraving settings saveable: " + pendingSaveError.toStdString());
        m::MachiningJobDefinition persistedEdit;
        require(m::MachiningJobStorage::load(directory.filePath("pending.oapjob"), persistedEdit, &pendingSaveError) &&
            persistedEdit.finishingOperations.front().parameters.stepOver == 0.5,
            "saving regenerated engraving lost the edited step-over");
        request.currentJob = &top.job;
        request.settings.scope = a::EngravingSetupScope::AllSixFaces;
        auto six = a::EngravingToolpathGenerationUseCase::execute(request);
        require(six.ok, six.error.toStdString());
        require(six.batch.operations.size() == 6, "default engraving did not generate all six faces");
        require(six.batch.operations.front().operationId == topId, "engraving preview ignored preserved global sequence");
        std::map<std::string, int> stable;
        for (const auto& operation : six.job.definition().finishingOperations) stable[operation.id] = operation.sequence;
        require(stable.contains(topId) && stable.at(topId) == 0, "expanding scope replaced +Z identity/order");
        request.currentJob = &six.job;
        auto repeated = a::EngravingToolpathGenerationUseCase::execute(request);
        require(repeated.ok, repeated.error.toStdString());
        require(repeated.job.definition().finishingOperations.size() == 6, "regeneration duplicated engraving operations");
        for (const auto& operation : repeated.job.definition().finishingOperations)
            require(stable.at(operation.id) == operation.sequence, "regeneration changed stable identity/order");

        const auto before = m::MachiningJobStorage::definitionFingerprint(six.job.definition());
        auto rejectedBatchJob = six.job;
        auto badSurface = top.batch.operations.front().result;
        badSurface.ok = false;
        m::SurfaceFinishingJobCaptureRequest validCapture;
        validCapture.result = &top.batch.operations.front().result;
        validCapture.tool = &top.tool;
        validCapture.setupFrames = &frames;
        validCapture.setupFace = MoldSetupFace::PositiveZ;
        validCapture.unitIds = {"unit-1"};
        auto invalidCapture = validCapture;
        invalidCapture.result = &badSurface;
        invalidCapture.setupFace = MoldSetupFace::PositiveX;
        std::string captureError;
        require(!m::SurfaceFinishingMachiningJobService{}.replaceFamily(rejectedBatchJob,
            {validCapture, invalidCapture}, &captureError) &&
            m::MachiningJobStorage::definitionFingerprint(rejectedBatchJob.definition()) == before &&
            rejectedBatchJob.currentFinishingResult()->operations.size() == 6,
            "an invalid later setup partially replaced the valid engraving family");
        request.interaction.showProgress = [](const a::WorkflowProgressRequest& progress) { progress.cancel(); };
        auto cancelled = a::EngravingToolpathGenerationUseCase::execute(request);
        require(cancelled.cancelled && !cancelled.ok &&
            m::MachiningJobStorage::definitionFingerprint(six.job.definition()) == before,
            "cancelled multi-face engraving published partial Job data");
        auto layeredRequest = request;
        layeredRequest.settings.parameters.mode = t::EngravingMachiningMode::StockRoughingAndFinishing;
        layeredRequest.definition.stock.marginZ = 1.0;
        const auto layeredCancelled = a::EngravingToolpathGenerationUseCase::execute(layeredRequest);
        require(layeredCancelled.cancelled && !layeredCancelled.ok &&
            m::MachiningJobStorage::definitionFingerprint(six.job.definition()) == before,
            "cancelled layered multi-face engraving published partial Job data");
        layeredRequest.interaction.showProgress = {};
        layeredRequest.settings.parameters.maxMoves = 4;
        const auto layeredLimited = a::EngravingToolpathGenerationUseCase::execute(layeredRequest);
        require(!layeredLimited.ok &&
            m::MachiningJobStorage::definitionFingerprint(six.job.definition()) == before &&
            six.job.currentFinishingResult()->operations.size() == 6,
            "layered move-budget failure replaced the existing engraving family");
        layeredRequest.settings.parameters.maxMoves = 2000000;
        const auto layeredSix = a::EngravingToolpathGenerationUseCase::execute(layeredRequest);
        require(layeredSix.ok && layeredSix.batch.operations.size() == 6,
            "unrestricted layered six-face fixture failed: " + layeredSix.error.toStdString());
        std::size_t largestFaceMoves = 0;
        std::size_t totalFaceMoves = 0;
        for (const auto& operation : layeredSix.batch.operations) {
            largestFaceMoves = std::max(largestFaceMoves, operation.result.moves.size());
            totalFaceMoves += operation.result.moves.size();
            require(operation.result.effectiveParams.maxMoves == 2000000,
                "shared family budget overwrote the saved operation configuration");
        }
        require(largestFaceMoves < totalFaceMoves, "layered budget fixture has no multi-face accumulation");
        layeredRequest.settings.parameters.maxMoves = largestFaceMoves;
        for (const auto& [scope, face] : directions) {
            auto individual = layeredRequest;
            individual.settings.scope = scope;
            const auto one = a::EngravingToolpathGenerationUseCase::execute(individual);
            require(one.ok && one.batch.operations.front().setupFace == face,
                "an individual layered face did not fit the cumulative test budget: " + one.error.toStdString());
        }
        const auto accumulated = a::EngravingToolpathGenerationUseCase::execute(layeredRequest);
        require(!accumulated.ok &&
            m::MachiningJobStorage::definitionFingerprint(six.job.definition()) == before &&
            six.job.currentFinishingResult()->operations.size() == 6,
            "six individually affordable layered faces bypassed the shared move budget or partially published");
        request.interaction.showProgress = {};
        request.settings.parameters.sampleSpacing = 0;
        auto failed = a::EngravingToolpathGenerationUseCase::execute(request);
        require(!failed.ok && m::MachiningJobStorage::definitionFingerprint(six.job.definition()) == before,
            "failed generation changed existing Job");
        request.settings.parameters.sampleSpacing = 1;
        request.settings.scope = a::EngravingSetupScope::PreserveProjectFaces;
        request.settings.projectFaces = {MoldSetupFace::PositiveX};
        auto side = a::EngravingToolpathGenerationUseCase::execute(request);
        require(side.ok && side.batch.operations.size() == 1 && side.batch.operations.front().setupFace == MoldSetupFace::PositiveX,
            "preserved project face expanded silently: " + side.error.toStdString());
        a::MachiningWorkspaceSession reopenedWorkspace;
        a::MachiningConfigurationSession reopenedConfiguration;
        a::ToolLibraryController reopenedLibraries;
        a::MachiningProjectInstaller::install(a::MachiningProjectController::restoreSnapshot(side.job),
            reopenedWorkspace, reopenedConfiguration, reopenedLibraries, frames);
        require(reopenedConfiguration.engravingSettings().scope == a::EngravingSetupScope::PositiveX &&
            reopenedConfiguration.engravingSettings().projectFaces == std::vector<MoldSetupFace>{MoldSetupFace::PositiveX} &&
            reopenedConfiguration.engravingSettings().toolMode == a::EngravingToolMode::SingleTool &&
            reopenedConfiguration.engravingSettings().manualTool.toolNumber == 3,
            "restoring an old side engraving lost its setup or standalone tool");

        oap::tooling::ToolLibrary library("engraving-library", "Engraving");
        addTool(library, 1, 1, oap::tooling::ToolType::EndMill);
        addTool(library, 2, 4, oap::tooling::ToolType::BallEndMill);
        addTool(library, 3, 2, oap::tooling::ToolType::BallEndMill);
        request.settings.scope = a::EngravingSetupScope::PositiveZ;
        request.settings.toolMode = a::EngravingToolMode::AutomaticLibrary;
        request.library = &library;
        auto selected = a::EngravingToolpathGenerationUseCase::execute(request);
        require(selected.ok && selected.tool.toolNumber == 3 && selected.tool.diameter == 2 &&
            selected.libraryId == "engraving-library" && selected.tool.horizontalFeed > 0,
            "automatic selection did not prefer the smaller suitable ball cutter: " + selected.error.toStdString());
        request.settings.parameters.diameter = 1.5;
        auto limited = a::EngravingToolpathGenerationUseCase::execute(request);
        require(limited.ok && limited.tool.toolNumber == 1, "diameter limit was ignored");
        request.library = nullptr;
        require(!a::EngravingToolpathGenerationUseCase::execute(request).ok, "missing dedicated library accepted");
        request.settings.toolMode = a::EngravingToolMode::SingleTool;
        request.settings.manualTool.toolNumber = 0;
        require(!a::EngravingToolpathGenerationUseCase::execute(request).ok, "invalid manual T number accepted");
        request.settings.manualTool = singleTool();
        request.settings.manualTool.cornerRadius = std::numeric_limits<double>::quiet_NaN();
        require(!a::EngravingToolpathGenerationUseCase::execute(request).ok, "nonfinite manual geometry accepted");
        request.settings.manualTool = singleTool();
        request.settings.manualTool.overallLength = 1;
        require(!a::EngravingToolpathGenerationUseCase::execute(request).ok, "cutting length beyond overall length accepted");
        request.settings.manualTool = singleTool();
        request.settings.manualTool.verticalFeed = 0;
        require(!a::EngravingToolpathGenerationUseCase::execute(request).ok, "nonpositive manual cutting feed accepted");
        request.settings.manualTool = singleTool();

        auto mixed = top.job;
        installRoughing(mixed);
        request.currentJob = &mixed;
        auto appended = a::EngravingToolpathGenerationUseCase::execute(request);
        require(appended.ok, appended.error.toStdString());
        require(appended.job.currentPocketResult() && appended.job.currentFinishingResult() &&
            appended.job.definition().pocketOperations.front().id == "rough" &&
            appended.job.definition().finishingOperations.front().sequence == 1,
            "engraving did not append after preserved roughing");
        m::ipw::IpwExecutionOptions options;
        options.pocket.method = t::StockSimulationMethod::HeightField;
        options.pocket.runParallel = false;
        options.pocket.zMapResolution = 0.5;
        options.pocket.zMapMaxCellCount = 10000;
        const auto execution = m::ipw::IpwExecutor{}.execute(appended.job,
            BRepPrimAPI_MakeBox(12, 8, 4).Shape(), "stock-v1", m::ipw::kHybridIpwEngineId, options);
        require(execution.complete() && execution.executedOperationIds.size() == 2 &&
            execution.executedOperationIds.front() == "rough", "engraving did not continue the native IPW chain: " + execution.message);

        a::MachiningWorkspaceCoordinator libraryWorkspace;
        oap::tooling::ToolLibrary activeLibrary("active-fixture", "Active");
        oap::tooling::ToolLibrary dedicatedLibrary("dedicated-fixture", "Dedicated");
        addTool(activeLibrary, 11, 2, oap::tooling::ToolType::EndMill);
        addTool(dedicatedLibrary, 12, 2, oap::tooling::ToolType::BallEndMill);
        require(libraryWorkspace.applyToolLibraries(QVector<oap::tooling::ToolLibrary>{activeLibrary, dedicatedLibrary}, activeLibrary.id()).ok,
            "cannot install library dependency fixture");
        auto librarySettings = libraryWorkspace.configurationEditorState(frames, 12, 8);
        librarySettings.engraving = request.settings;
        librarySettings.engraving.parameters.diameter = 6;
        librarySettings.engraving.toolMode = a::EngravingToolMode::AutomaticLibrary;
        librarySettings.engraving.libraryId = dedicatedLibrary.id().toStdString();
        libraryWorkspace.applyConfiguration(librarySettings, document);
        require(libraryWorkspace.generateEngraving(request).ok && !libraryWorkspace.hasPendingRegeneration(),
            "dedicated-library fixture did not generate");
        auto unrelatedCatalogEdit = libraryWorkspace.toolCatalog();
        for (auto& item : unrelatedCatalogEdit.editableLibraries())
            if (item.id() == activeLibrary.id()) item.setName(QStringLiteral("Unrelated active library edit"));
        require(libraryWorkspace.applyToolLibraries(std::move(unrelatedCatalogEdit), activeLibrary.id()).ok &&
            !libraryWorkspace.hasPendingRegeneration() && libraryWorkspace.hasFinishingPreview(),
            "editing an unrelated active library invalidated dedicated-library engraving");
        activeLibrary.setName(QStringLiteral("Unrelated active library edit"));
        addTool(dedicatedLibrary, 12, 3, oap::tooling::ToolType::BallEndMill);
        require(libraryWorkspace.applyToolLibraries(QVector<oap::tooling::ToolLibrary>{activeLibrary, dedicatedLibrary}, activeLibrary.id()).ok &&
            libraryWorkspace.hasPendingRegeneration() && !libraryWorkspace.hasFinishingPreview(),
            "editing the non-active engraving library did not invalidate its old program");
        librarySettings = libraryWorkspace.configurationEditorState(frames, 12, 8);
        librarySettings.engraving.toolMode = a::EngravingToolMode::SingleTool;
        librarySettings.engraving.manualTool = singleTool();
        libraryWorkspace.applyConfiguration(librarySettings, document);
        require(libraryWorkspace.generateEngraving(request).ok && !libraryWorkspace.hasPendingRegeneration(),
            "single-tool fixture did not clear its own successfully regenerated pending state");
        dedicatedLibrary.setName(QStringLiteral("Changed library is unused by single tool"));
        require(libraryWorkspace.applyToolLibraries(QVector<oap::tooling::ToolLibrary>{activeLibrary, dedicatedLibrary}, dedicatedLibrary.id()).ok &&
            !libraryWorkspace.hasPendingRegeneration() && libraryWorkspace.hasFinishingPreview(),
            "a catalog/active-library change invalidated standalone engraving");
        auto confirmedOnly = libraryWorkspace.configurationEditorState(frames, 12, 8);
        const auto beforeConfirmation = libraryWorkspace.finishingPreviewProgram();
        const auto confirmationFingerprint = libraryWorkspace.operationDescriptors().front().inputFingerprint;
        confirmedOnly.moldSetupWorkFrames.setConfirmed(MoldSetupFace::PositiveZ, false);
        const auto confirmationApplied = libraryWorkspace.applyConfiguration(confirmedOnly, document);
        require(!confirmationApplied.engravingParametersChanged && !libraryWorkspace.hasPendingRegeneration(),
            "confirmation-only edits unnecessarily invalidated engraving geometry");
        require(libraryWorkspace.operationDescriptors().front().state == m::JobOperationState::NeedsReview &&
            libraryWorkspace.hasFinishingPreview(),
            "unconfirming a setup did not update the formal state or discarded generated moves");
        confirmedOnly.moldSetupWorkFrames.setConfirmed(MoldSetupFace::PositiveZ, true);
        libraryWorkspace.applyConfiguration(confirmedOnly, document);
        const auto afterConfirmation = libraryWorkspace.finishingPreviewProgram();
        require(libraryWorkspace.operationDescriptors().front().state == m::JobOperationState::Warning &&
            !libraryWorkspace.hasPendingRegeneration() && libraryWorkspace.hasFinishingPreview() &&
            afterConfirmation.operations.size() == beforeConfirmation.operations.size() &&
            afterConfirmation.operations.front().moves.size() == beforeConfirmation.operations.front().moves.size() &&
            libraryWorkspace.operationDescriptors().front().inputFingerprint == confirmationFingerprint,
            "confirming a setup required regeneration or failed to restore the formal state with current moves");
        auto shiftedSetup = libraryWorkspace.configurationEditorState(confirmedOnly.moldSetupWorkFrames, 12, 8);
        auto* shiftedFrame = shiftedSetup.moldSetupWorkFrames.forFace(MoldSetupFace::PositiveZ);
        const auto oldOrigin = shiftedFrame->setupWorkFrame.Location();
        shiftedFrame->setupWorkFrame.SetLocation(gp_Pnt(oldOrigin.X(), oldOrigin.Y(), oldOrigin.Z() + 0.1));
        const auto setupApplied = libraryWorkspace.applyConfiguration(shiftedSetup, document);
        require(setupApplied.holeParametersChanged && setupApplied.pocketParametersChanged &&
            setupApplied.profileParametersChanged && setupApplied.engravingParametersChanged &&
            libraryWorkspace.hasPendingRegeneration() && !libraryWorkspace.hasFinishingPreview(),
            "setup origin changes left old machining geometry current");
        libraryWorkspace.applyConfiguration(shiftedSetup, document);
        require(libraryWorkspace.hasPendingRegeneration() && !libraryWorkspace.hasFinishingPreview() &&
            libraryWorkspace.operationDescriptors().front().state == m::JobOperationState::Stale,
            "reapplying confirmed setup settings revived an invalidated engraving path");
        std::cout << "Direct engraving, six-face transactions, manual/library tools, stable replacement, cancellation and roughing IPW passed.\n";
        return 0;
    } catch (const std::exception& failure) {
        std::cerr << "FAIL: " << failure.what() << '\n';
        return 1;
    }
}

#include "app/machining/MachiningWorkspaceCoordinator.h"
#include "app/workflows/playback/PlaybackProgramAssembler.h"
#include "app/workflows/project/MachiningProjectInstaller.h"
#include "app/workflows/engraving/EngravingToolpathGenerationUseCase.h"
#include "app/workflows/simulation/StockSimulationUseCases.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "toolpath/program/validation/GCodeReview.h"
#include "app/workflows/shared/ReferenceNcExportService.h"
#include "tooling/library/ToolLibrary.h"
#include "toolpath/pocket/simulation/PocketCamSimulationScene.h"
#include "toolpath/pocket/simulation/StockTargetCache.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <STEPControl_Writer.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <QCoreApplication>
#include <QTemporaryDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <algorithm>
#include <stdexcept>
#include <iostream>

namespace {
void require(bool ok, const std::string& message)
{
    if (!ok) throw std::runtime_error(message);
}

void verifyProjectAnimation(const oap::app::ProjectStockSimulationUseCaseResult& simulated)
{
    using namespace oap::app;
    const auto& input = simulated.camInput;
    require(simulated.playbackError.isEmpty() && input.authoritativeOperations &&
        input.options.authoritativeIpwPlayback && input.initialIpwStock && input.finalIpwStock,
        "missing project animation: " + simulated.playbackError.toStdString());
    const auto& operations = *input.authoritativeOperations;
    require(input.operationInitialIpwSources.size() == operations.size(), "animation lost operation baselines");
    const auto scene = oap::toolpath::PocketCamSimulationSceneBuilder().build(
        input.stock, operations, input.params, input.options);
    require(scene.ok() && scene.scene.authoritativeIpwPlayback, "cannot build project scene: " + scene.error);
    for (const auto& motion : scene.scene.motions)
        require(motion.outwardToolAxis.IsEqual(operations.at(motion.operationIndex).outwardToolAxis, 1e-8, 1e-8),
            "animation lost the operation tool axis");
    IpwDisplayMeshWorker worker;
    IpwDisplayMeshResult frame;
    QObject::connect(&worker, &IpwDisplayMeshWorker::resultReady,
        [&](const auto& value) { frame = value; });
    const auto replay = [&](std::shared_ptr<const IpwTransientFrameSource> source) {
        IpwDisplayMeshRequest request;
        request.displayMode = IpwOcctDisplayMode::Off;
        request.transientSource = std::move(source);
        worker.build(std::move(request));
        require(frame.ok() && frame.materialStock, "animation frame failed: " + frame.error);
        require(frame.materialContentFingerprint.empty(), "GPU-only animation must not eagerly hash the whole grid");
        return frame.materialStock;
    };
    for (std::size_t i = 0; i < operations.size(); ++i) {
        const auto& baseline = *input.operationInitialIpwSources[i];
        require(baseline.operations == input.authoritativeOperations && baseline.toolProfiles.size() == operations.size(),
            "animation copied moves or lost tool profiles");
        auto full = std::make_shared<IpwTransientFrameSource>(baseline);
        full->completedMoveCount += operations[i].clearing.moves.size();
        const auto expected = replay(full);
        auto first = std::make_shared<IpwTransientFrameSource>(baseline);
        first->completedMoveCount += operations[i].clearing.moves.size() / 2;
        const auto partial = replay(first);
        auto second = std::make_shared<IpwTransientFrameSource>(*full);
        second->parentStock = partial;
        second->firstMoveCount = first->completedMoveCount;
        const auto incremental = replay(second);
        require(incremental->contentFingerprint() == expected->contentFingerprint(),
            "incremental animation differs from original-move replay");
        require(replay(first)->contentFingerprint() == partial->contentFingerprint(),
            "backward seek mutated an immutable checkpoint");
        if (i + 1 < operations.size() &&
            input.operationInitialIpwSources[i + 1]->firstMoveCount == full->completedMoveCount)
            require(expected->contentFingerprint() == input.operationInitialIpwSources[i + 1]->parentStock->contentFingerprint(),
                "animation does not join the next formal operation checkpoint");
        if (i + 1 == operations.size())
            require(expected->contentFingerprint() == input.finalIpwStock->contentFingerprint(),
                "animation final frame differs from authoritative project IPW");
    }
    if (operations.size() > 1) {
        auto legacyOptions = input.options;
        legacyOptions.authoritativeIpwPlayback = false;
        require(!oap::toolpath::PocketCamSimulationSceneBuilder().build(
            input.stock, operations, input.params, legacyOptions).ok(),
            "mixed axes escaped fixed-axis GPU playback guard");
    }
}
}

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    using namespace oap;
    using namespace oap::app;
    try {
        QTemporaryDir directory;
        require(directory.isValid(), "temporary directory unavailable");
        const auto shape = BRepPrimAPI_MakeBox(12, 8, 2).Shape();
        const auto source = directory.filePath("target.step");
        STEPControl_Writer writer;
        require(writer.Transfer(shape, STEPControl_AsIs) == IFSelect_RetDone &&
            writer.Write(source.toUtf8().constData()) == IFSelect_RetDone, "cannot write fixture STEP");
        Handle(TDocStd_Document) document;
        XCAFApp_Application::GetApplication()->NewDocument("BinXCAF", document);
        XCAFDoc_DocumentTool::ShapeTool(document->Main())->AddShape(shape, false);
        StepIndexer indexer;
        Model model;
        require(indexer.build(document, model) && model.units.size() == 1, "cannot index fixture");
        const auto indexedBounds = indexer.exactBoundsForShape(shape, model);
        require(indexedBounds.has_value(), "indexed exact target bounds unavailable");
        std::array<double, 6> computedBounds{};
        require(toolpath::StockTargetCache(shape, "test").bounds(computedBounds), "exact target bounds failed");
        for (std::size_t i = 0; i < computedBounds.size(); ++i)
            require(std::abs(computedBounds[i] - (*indexedBounds)[i]) < 1.e-10,
                "indexed bounds differ from exact geometry bounds");
        require(!indexer.exactBoundsForShape(BRepPrimAPI_MakeBox(12, 8, 2).Shape(), model),
            "unindexed shape must not reuse target bounds");
        auto incompleteModel = model;
        incompleteModel.faces.front().bbox = {};
        require(!indexer.exactBoundsForShape(shape, incompleteModel), "partial face bounds must fall back to geometry");
        const Bbox bounds{0, 0, 0, 12, 8, 2};
        const auto mold = MoldCoordinateFrame::fromBounds(bounds);
        auto frames = toolpath::MoldSetupWorkFrames::fromMold(bounds, mold);
        frames.setConfirmed(MoldSetupFace::PositiveZ, true);
        const auto* frame = frames.forFace(MoldSetupFace::PositiveZ);
        require(frame && frame->valid, "missing frame");
        toolpath::SurfaceFinishingParams parameters;
        parameters.diameter = 4;
        parameters.stepOver = 2;
        parameters.sampleSpacing = 1;
        const auto generated = toolpath::SurfaceFinishing().generate(shape, frame->setupWorkFrame, parameters);
        require(generated.ok, generated.error);
        toolpath::ToolpathToolContext tool;
        tool.valid = true;
        tool.toolNumber = 7;
        tool.toolId = "library-ball";
        tool.toolName = "Finishing ball";
        tool.toolType = tooling::toolTypeId(tooling::ToolType::BallEndMill).toStdString();
        tool.diameter = parameters.diameter;
        tool.cornerRadius = parameters.diameter / 2;
        tool.cuttingEdgeLength = parameters.cuttingLength;
        tool.spindleSpeed = 8000;
        tool.horizontalFeed = parameters.feed;
        tool.verticalFeed = parameters.plungeFeed;
        MachiningProjectDefinitionContext context;
        context.sourcePath = source;
        context.stock.marginZ = 1;
        manufacturing::SurfaceFinishingJobCaptureRequest capture;
        capture.result = &generated;
        capture.tool = &tool;
        capture.setupFrames = &frames;
        capture.setupFace = MoldSetupFace::PositiveZ;
        capture.unitIds = {model.units.front().unit_id};
        capture.libraryId = "finishing-test-library";
        MachiningWorkspaceCoordinator coordinator;
        QString error;
        require(coordinator.installSurfaceFinishing(context, capture, &error), error.toStdString());
        require(coordinator.status().hasFinishingProgram && coordinator.status().finishingOperationCount == 1 &&
            coordinator.hasProjectContent() && coordinator.activePreview() == ActiveToolpathPreview::SurfaceFinishing,
            "formal install did not update active workspace");
        const auto operationId = coordinator.operationDescriptors().front().id;
        require(coordinator.finishingOperation(operationId).has_value(), "formal operation cannot be edited");
        const auto playback = PlaybackProgramAssembler::build(coordinator.finishingPreviewProgram());
        require(!playback.moves.empty() && playback.moves.front().operationType ==
            toolpath::ToolpathOperationType::SurfaceFinishing, "playback lost finishing type");
        const auto tree = coordinator.holeToolTreeModel(playback.moves, playback.toolBatchStarts);
        require(!tree.batches.empty() && tree.batches.front().family == ToolpathBatchFamily::SurfaceFinishing &&
            tree.batches.front().operations.front().formalOperationId.toStdString() == operationId,
            "formal finishing missing from operation tree");
        const auto exportData = coordinator.prepareToolpathExport();
        require(exportData.ok, exportData.error.toStdString());
        const auto nc = ReferenceNcExportService().prepare(exportData.program, exportData.params,
            frames, {}, directory.filePath("finishing.nc"));
        require(nc.ok() && !nc.files.empty(), nc.error);
        require(toolpath::GCodeReview().review(nc.files.front().ncText.toStdString()).ok(), "NC review rejected formal result");
        MachiningProjectSaveRequest save;
        save.definition = context;
        save.setupFrames = &frames;
        require(coordinator.synchronizeProjectForSave(save, &error), error.toStdString());
        const auto path = directory.filePath("finishing.oapjob");
        require(coordinator.saveProject(path, &error), error.toStdString());
        MachiningProjectLoadRequest load;
        load.filePath = path;
        load.currentSourcePath = source;
        load.model = &model;
        load.indexer = &indexer;
        load.moldFrame = &mold;
        load.setupFrames = &frames;
        auto loaded = MachiningProjectController::loadAndRegenerate(load);
        require(loaded.ok() && loaded.hasFinishingDefinitions && loaded.job.hasFinishingResult(),
            "reopening standalone finishing Job failed: " + loaded.error.toStdString());
        auto legacyRoot = QJsonDocument::fromJson(manufacturing::MachiningJobStorage::serializeDefinition(loaded.job.definition())).object();
        auto legacyJob = legacyRoot.value("job").toObject();
        legacyRoot.insert("schema_version", 11);
        legacyJob.insert("schema_version", 11);
        auto legacyStock = legacyJob.value("stock").toObject();
        legacyStock.remove("source_fingerprint");
        legacyJob.insert("stock", legacyStock);
        auto legacyOperations = legacyJob.value("surface_finishing_operations").toArray();
        for (qsizetype i = 0; i < legacyOperations.size(); ++i) {
            auto operation = legacyOperations[i].toObject();
            auto p = operation.value("parameters").toObject();
            for (const auto* key : {"mode", "max_step_down", "roughing_allowance", "max_moves"}) p.remove(key);
            operation.insert("parameters", p);
            legacyOperations[i] = operation;
        }
        legacyJob.insert("surface_finishing_operations", legacyOperations);
        legacyRoot.insert("job", legacyJob);
        manufacturing::MachiningJobDefinition legacy;
        require(manufacturing::MachiningJobStorage::deserializeDefinition(QJsonDocument(legacyRoot).toJson(), legacy, &error) &&
            legacy.finishingOperations.front().parameters.mode == toolpath::EngravingMachiningMode::FinishOnly,
            "v11 project did not retain finish-only semantics: " + error.toStdString());
        MachiningWorkspaceSession workspace;
        MachiningConfigurationSession configuration;
        ToolLibraryController libraries;
        const auto installed = MachiningProjectInstaller::install(
            std::move(loaded), workspace, configuration, libraries, frames);
        require(installed.hasFinishingDefinitions && installed.finishingOperationCount == 1 &&
            workspace.activePreview() == ActiveToolpathPreview::SurfaceFinishing,
            "reopened project installer did not restore finishing preview");
        require(configuration.engravingSettings().scope == EngravingSetupScope::PositiveZ &&
            configuration.engravingSettings().toolMode == EngravingToolMode::AutomaticLibrary &&
            configuration.engravingSettings().libraryId == "finishing-test-library" &&
            configuration.engravingSettings().manualTool.toolNumber == 7,
            "reopened +Z library engraving configuration was not restored");
        StockSimulationSession simulation;
        StockSimulationCoordinator simulationCoordinator;
        ProjectStockSimulationUseCaseRequest request;
        request.session = &simulation;
        request.coordinator = &simulationCoordinator;
        request.job = &workspace.job();
        request.persistSettings = false;
        request.settings.source.sourceMode = 1;
        request.settings.displayQuality = 0;
        request.generatedInitialStock = BRepPrimAPI_MakeBox(12, 8, 3).Shape();
        request.targetShape = shape;
        request.exactTargetBounds = indexedBounds;
        request.runner = [](manufacturing::MachiningJob& job, IpwSimulationRequest input,
                             std::shared_ptr<std::atomic_bool>) {
            return runIpwSimulation(job, std::move(input));
        };
        const auto simulated = ProjectStockSimulationUseCase::execute(request);
        require(simulated.ok() && simulation.hasPocketSimulation(),
            "desktop project IPW failed: " + simulated.error.toStdString());
        verifyProjectAnimation(simulated);
        require(simulated.camInput.operationInitialIpwSources.front()->toolProfiles.front().cornerRadius == parameters.diameter / 2,
            "engraving animation lost ball-end profile");
        require(simulated.runResult.execution.executedOperationIds.size() == 1 &&
            simulated.runResult.execution.executedOperationIds.front() == operationId,
            "desktop IPW omitted formal finishing");
        require(workspace.job().definition().stock.marginZ == context.stock.marginZ &&
            workspace.job().currentFinishingResult(),
            "simulation stock settings changed planning stock or invalidated finishing");
        const auto retainedTargetCache = workspace.job().runtime().stockTargetCache;
        require(retainedTargetCache && simulated.preparation.request.exactTargetBounds == indexedBounds,
            "desktop target bounds were not forwarded to IPW");
        const auto repeated = ProjectStockSimulationUseCase::execute(request);
        require(retainedTargetCache == workspace.job().runtime().stockTargetCache &&
            repeated.runResult.comparison.targetVolume == simulated.runResult.comparison.targetVolume &&
            repeated.runResult.comparison.overcutVolume == simulated.runResult.comparison.overcutVolume,
            "repeated desktop audit lost target cache or changed quantities");
        verifyProjectAnimation(repeated);
        require(repeated.ok() && repeated.runResult.execution.executedNodeCount == 0 &&
            repeated.runResult.execution.reusedCheckpointCount > 0, "desktop IPW did not reuse unchanged finishing");
        using OperationState = manufacturing::JobOperationState;
        const auto checkRejectedState = [&](OperationState state, const QString& expected, bool confirmed = true) {
            auto rejectedJob = workspace.job();
            auto& operation = rejectedJob.definition().finishingOperations.front();
            operation.state = state;
            for (auto& setup : rejectedJob.definition().setups)
                if (setup.id == operation.setupId) setup.confirmation.confirmed = confirmed;
            auto rejectedRequest = request;
            rejectedRequest.job = &rejectedJob;
            bool runnerCalled = false;
            rejectedRequest.runner = [&](manufacturing::MachiningJob&, IpwSimulationRequest,
                                         std::shared_ptr<std::atomic_bool>) {
                runnerCalled = true;
                return IpwSimulationRunResult{};
            };
            const auto rejected = ProjectStockSimulationUseCase::execute(rejectedRequest);
            require(!rejected.ok() && !runnerCalled && rejected.error.contains(expected) &&
                rejected.error.contains(QString::fromStdString(operation.id)),
                "IPW state rejection did not explain its cause: " + rejected.error.toStdString());
        };
        checkRejectedState(OperationState::Planned, QStringLiteral("尚未生成刀路"));
        checkRejectedState(OperationState::Dirty, QStringLiteral("刀路已失效"));
        checkRejectedState(OperationState::Stale, QStringLiteral("刀路已失效"));
        checkRejectedState(OperationState::Error, QStringLiteral("生成或校验失败"));
        checkRejectedState(OperationState::NeedsReview, QStringLiteral("装夹待确认"), false);
        checkRejectedState(OperationState::NeedsReview, QStringLiteral("需要复核"));
        auto invalid = generated;
        invalid.ok = false;
        capture.result = &invalid;
        require(!coordinator.installSurfaceFinishing(context, capture, &error) &&
            coordinator.operationDescriptors().size() == 1 && coordinator.hasFinishingPreview(),
            "failed install replaced the prior valid project");
        auto disabledDefinition = workspace.job().definition();
        disabledDefinition.finishingOperations.front().enabled = false;
        require(manufacturing::MachiningJobStorage::save(path, disabledDefinition, &error), error.toStdString());
        auto disabled = MachiningProjectController::loadAndRegenerate(load);
        require(disabled.ok() && disabled.hasFinishingDefinitions && disabled.finishing.operations.empty(),
            "disabled finishing project could not be reopened: " + disabled.error.toStdString());
        const auto disabledInstalled = MachiningProjectInstaller::install(
            std::move(disabled), workspace, configuration, libraries, frames);
        require(disabledInstalled.hasFinishingDefinitions && disabledInstalled.finishingOperationCount == 0 &&
            workspace.activePreview() == ActiveToolpathPreview::None,
            "disabled finishing unexpectedly activated an empty preview");

        EngravingToolpathGenerationRequest roughRequest;
        roughRequest.targets = {shape};
        roughRequest.unitIds = {model.units.front().unit_id};
        roughRequest.setupFrames = &frames;
        roughRequest.definition = context;
        roughRequest.settings.scope = EngravingSetupScope::PositiveZ;
        roughRequest.settings.toolMode = EngravingToolMode::SingleTool;
        roughRequest.settings.manualTool = tool;
        roughRequest.settings.parameters = parameters;
        roughRequest.settings.parameters.mode = toolpath::EngravingMachiningMode::StockRoughingAndFinishing;
        roughRequest.settings.parameters.maxStepDown = 0.75;
        roughRequest.settings.parameters.roughingAllowance = 0.2;
        const auto rough = EngravingToolpathGenerationUseCase::execute(roughRequest);
        require(rough.ok && rough.batch.operations.front().result.metrics.roughingLayerCount > 0 &&
            rough.job.definition().finishingOperations.front().algorithmId == toolpath::kOpenCamLibStockEngravingAlgorithmId,
            "layered planning from generated stock failed: " + rough.error.toStdString());
        auto sixRequest = roughRequest;
        sixRequest.settings.scope = EngravingSetupScope::AllSixFaces;
        for (int face = 0; face < 6; ++face) frames.setConfirmed(static_cast<MoldSetupFace>(face), true);
        auto six = EngravingToolpathGenerationUseCase::execute(sixRequest);
        require(six.ok && six.batch.operations.size() == 6, "six-face engraving generation failed: " + six.error.toStdString());
        auto sixSimulationRequest = request;
        sixSimulationRequest.job = &six.job;
        const auto sixSimulation = ProjectStockSimulationUseCase::execute(sixSimulationRequest);
        require(sixSimulation.ok(), "six-face IPW failed: " + sixSimulation.error.toStdString());
        verifyProjectAnimation(sixSimulation);
        // The same entry also accepts roughing before engraving, with a tool
        // change and one continuous material chain across setup changes.
        auto mixedJob = six.job;
        auto mixedFinishing = *mixedJob.currentFinishingResult();
        auto& mixedDefinition = mixedJob.definition();
        auto flatTool = tool;
        flatTool.toolNumber = 8;
        flatTool.toolId = "rough-flat";
        flatTool.toolType = tooling::toolTypeId(tooling::ToolType::EndMill).toStdString();
        flatTool.cornerRadius = 0;
        mixedDefinition.toolControllers.push_back({"rough-controller", "fixture", flatTool});
        manufacturing::PocketOperationDefinition pocket;
        pocket.id = "rough-before-engraving";
        const auto topSetup = std::find_if(mixedDefinition.setups.begin(), mixedDefinition.setups.end(),
            [](const auto& setup) { return setup.face == MoldSetupFace::PositiveZ; });
        require(topSetup != mixedDefinition.setups.end(), "missing roughing setup");
        pocket.setupId = topSetup->id;
        pocket.toolControllerId = "rough-controller";
        pocket.algorithmId = "builtin.pocket.freecad";
        pocket.target.sourceFaceIds = {"fixture-top"};
        pocket.target.floorFaceId = "fixture-top";
        pocket.target.geometryFingerprint = "fixture-top-v1";
        pocket.inputFingerprint = "fixture-rough-v1";
        pocket.state = manufacturing::JobOperationState::Generated;
        mixedDefinition.pocketOperations.push_back(pocket);
        for (auto& finish : mixedDefinition.finishingOperations) ++finish.sequence;
        toolpath::PocketToolpathBatch pocketBatch;
        toolpath::PocketToolpathOperation pocketRuntime;
        pocketRuntime.operationId = pocket.id;
        pocketRuntime.algorithmId = pocket.algorithmId;
        pocketRuntime.setupIndex = topSetup->setupIndex;
        pocketRuntime.setupFace = topSetup->face;
        pocketRuntime.outwardToolAxis = gp_Vec(0, 0, 1);
        pocketRuntime.tool = flatTool;
        pocketRuntime.clearing.ok = true;
        toolpath::ToolpathMove roughMove;
        roughMove.type = toolpath::ToolpathMoveType::Cut;
        roughMove.from = gp_Pnt(1, 4, 2.5);
        roughMove.to = gp_Pnt(11, 4, 2.5);
        roughMove.feed = 500;
        pocketRuntime.clearing.moves = {roughMove};
        pocketBatch.operations.push_back(std::move(pocketRuntime));
        mixedJob.setPocketResult(std::move(pocketBatch),
            manufacturing::MachiningJobStorage::pocketExecutionFingerprint(mixedDefinition).toStdString());
        mixedJob.setFinishingResult(std::move(mixedFinishing),
            manufacturing::MachiningJobStorage::finishingExecutionFingerprint(mixedDefinition).toStdString());
        sixSimulationRequest.job = &mixedJob;
        const auto mixedSimulation = ProjectStockSimulationUseCase::execute(sixSimulationRequest);
        require(mixedSimulation.ok(), "roughing/engraving project IPW failed: " + mixedSimulation.error.toStdString());
        verifyProjectAnimation(mixedSimulation);
        const auto mixedScene = toolpath::PocketCamSimulationSceneBuilder().build(
            mixedSimulation.camInput.stock, *mixedSimulation.camInput.authoritativeOperations,
            mixedSimulation.camInput.params, mixedSimulation.camInput.options);
        require(mixedScene.scene.tools.size() == 2 && mixedScene.scene.motions.front().toolIndex !=
            mixedScene.scene.motions.back().toolIndex, "project animation lost the roughing/engraving tool change");
        auto engravingRequest = sixSimulationRequest;
        engravingRequest.engravingOnly = true;
        const auto engravingSimulation = ProjectStockSimulationUseCase::execute(engravingRequest);
        require(engravingSimulation.ok() && engravingSimulation.camInput.authoritativeOperations &&
            engravingSimulation.camInput.authoritativeOperations->size() == 6,
            "dedicated engraving IPW did not select engraving: " + engravingSimulation.error.toStdString());
        verifyProjectAnimation(engravingSimulation);
        const auto& firstEngraving = mixedJob.ipwSession().nodes().at(1);
        const auto precedingRoughing = mixedJob.runtime().ipwCheckpoints.find(firstEngraving.parentStateKey);
        require(engravingSimulation.camInput.initialIpwStock == precedingRoughing->dexelStock &&
            engravingSimulation.camInput.initialIpwStock->volume() < mixedSimulation.camInput.initialIpwStock->volume(),
            "dedicated engraving IPW did not inherit preceding roughing stock");
        const auto firstSetup = six.batch.operations.front().setupIndex;
        engravingRequest.settings.selectedSetup = firstSetup;
        const auto singleFace = ProjectStockSimulationUseCase::execute(engravingRequest);
        require(singleFace.ok() && singleFace.camInput.authoritativeOperations &&
            singleFace.camInput.authoritativeOperations->size() == 1 &&
            singleFace.preparation.includedOperationIds.size() == 2,
            "single-face engraving included later setups or omitted preceding roughing");
        verifyProjectAnimation(singleFace);
        const auto beforeInvalidScope = mixedJob.ipwSession().nodes().back().stateKey;
        engravingRequest.settings.selectedSetup = 999;
        require(!ProjectStockSimulationUseCase::execute(engravingRequest).ok() &&
            mixedJob.ipwSession().nodes().back().stateKey == beforeInvalidScope,
            "empty engraving selection modified the current material chain");
        engravingRequest.settings.selectedSetup = -1;
        auto futureJob = mixedJob;
        manufacturing::ProfileOperationDefinition future;
        future.id = "later-unbuilt-profile";
        future.setupId = topSetup->id;
        future.toolControllerId = "rough-controller";
        future.algorithmId = "builtin.profile.freecad";
        future.target.axisLabel = "Z";
        future.target.sourceFaceIds = {"fixture-top"};
        future.target.rootFaceId = "fixture-top";
        future.target.rootWireId = "fixture-wire";
        future.target.geometryFingerprint = "fixture-profile-v1";
        future.inputFingerprint = "fixture-profile-input-v1";
        future.sequence = 100;
        future.state = manufacturing::JobOperationState::Planned;
        futureJob.definition().profileOperations.push_back(future);
        engravingRequest.job = &futureJob;
        const auto beforeFuture = ProjectStockSimulationUseCase::execute(engravingRequest);
        require(beforeFuture.ok() && beforeFuture.playbackError.isEmpty(),
            "later unbuilt family blocked engraving IPW: " + beforeFuture.error.toStdString() + beforeFuture.playbackError.toStdString());
        auto staleRoughing = mixedJob;
        staleRoughing.definition().pocketOperations.front().state = manufacturing::JobOperationState::Stale;
        engravingRequest.job = &staleRoughing;
        require(!ProjectStockSimulationUseCase::execute(engravingRequest).ok(),
            "engraving IPW accepted stale prerequisite roughing");
        const auto roughPath = directory.filePath("rough.oapjob");
        require(manufacturing::MachiningJobStorage::save(roughPath, rough.job.definition(), &error), error.toStdString());
        load.filePath = roughPath;
        auto roughLoaded = MachiningProjectController::loadAndRegenerate(load);
        require(roughLoaded.ok() && roughLoaded.finishing.operations.front().result.effectiveParams.maxStepDown == 0.75 &&
            roughLoaded.finishing.operations.front().result.metrics.roughingLayerCount ==
                rough.batch.operations.front().result.metrics.roughingLayerCount &&
            roughLoaded.finishing.operations.front().result.moves.size() == rough.batch.operations.front().result.moves.size(),
            "layered project did not reconstruct the same planning stock/path: " + roughLoaded.error.toStdString());
        auto limitedOne = rough.job;
        const auto moveBudget = rough.batch.operations.front().result.moves.size() + 4;
        limitedOne.definition().finishingOperations.front().parameters.maxMoves = moveBudget;
        const auto singleRegenerated = manufacturing::SurfaceFinishingMachiningJobService().regenerate(limitedOne, indexer);
        require(singleRegenerated.validation.ok() && singleRegenerated.operations.size() == 1 &&
            singleRegenerated.operations.front().result.effectiveParams.maxMoves == moveBudget,
            "single layered operation did not fit its measured budget or lost its configured limit");
        auto accumulatedJob = limitedOne;
        auto duplicated = accumulatedJob.definition().finishingOperations.front();
        duplicated.id += "-second";
        duplicated.sequence += 1;
        accumulatedJob.definition().finishingOperations.push_back(duplicated);
        const auto beforeBudgetFailure = manufacturing::MachiningJobStorage::definitionFingerprint(accumulatedJob.definition());
        const auto accumulatedRegeneration = manufacturing::SurfaceFinishingMachiningJobService().regenerate(accumulatedJob, indexer);
        require(accumulatedRegeneration.operations.empty() && !accumulatedRegeneration.validation.ok() &&
            manufacturing::MachiningJobStorage::definitionFingerprint(accumulatedJob.definition()) == beforeBudgetFailure &&
            accumulatedJob.finishingResult()->operations.size() == 1,
            "regeneration bypassed the layered family budget or installed a partial result");
        auto changedStock = rough.job;
        changedStock.definition().stock.marginZ += 1;
        require(!changedStock.currentFinishingResult(), "stock margin change kept the old layered path current");
        auto mismatched = rough.job.definition();
        mismatched.finishingOperations.front().algorithmId = toolpath::kOpenCamLibSurfaceFinishingAlgorithmId;
        manufacturing::MachiningJobDefinition rejected;
        require(!manufacturing::MachiningJobStorage::deserializeDefinition(
            manufacturing::MachiningJobStorage::serializeDefinition(mismatched), rejected, &error),
            "layered parameters were accepted under the finish-only algorithm identity");

        const auto stockPath = directory.filePath("planning-stock.step");
        STEPControl_Writer stockWriter;
        require(stockWriter.Transfer(BRepPrimAPI_MakeBox(12, 8, 4).Shape(), STEPControl_AsIs) == IFSelect_RetDone &&
            stockWriter.Write(stockPath.toUtf8().constData()) == IFSelect_RetDone, "cannot write planning stock");
        roughRequest.definition.stock.sourceKind = manufacturing::StockSourceKind::StepModel;
        roughRequest.definition.stock.sourcePath = stockPath.toStdString();
        auto external = EngravingToolpathGenerationUseCase::execute(roughRequest);
        require(external.ok && !external.job.definition().stock.sourceFingerprint.empty(),
            "external planning stock was not bound: " + external.error.toStdString());
        const auto stockFingerprint = external.job.definition().stock.sourceFingerprint;
        require(MachiningProjectController::ensureDefinition(external.job, roughRequest.definition, &error) &&
            external.job.definition().stock.sourceFingerprint == stockFingerprint,
            "context refresh discarded the planning stock identity");
        MachiningProjectSaveRequest externalSave;
        externalSave.definition = roughRequest.definition;
        externalSave.setupFrames = &frames;
        require(MachiningProjectController::synchronizeForSave(external.job, externalSave, &error) &&
            external.job.definition().stock.sourceFingerprint == stockFingerprint,
            "save synchronization discarded planning stock identity: " + error.toStdString());
        require(manufacturing::MachiningJobStorage::save(roughPath, external.job.definition(), &error), error.toStdString());
        const auto externalReloaded = MachiningProjectController::loadAndRegenerate(load);
        require(externalReloaded.ok() && externalReloaded.job.definition().stock.sourceFingerprint == stockFingerprint,
            "unchanged STEP planning stock did not reopen: " + externalReloaded.error.toStdString());
        auto switched = external.job;
        auto switchedContext = roughRequest.definition;
        switchedContext.stock.sourcePath = directory.filePath("different-stock.step").toStdString();
        require(MachiningProjectController::ensureDefinition(switched, switchedContext, &error) &&
            switched.definition().stock.sourceFingerprint.empty(), "new stock path retained the old file identity");
        QFile modifiedStock(stockPath);
        require(modifiedStock.open(QIODevice::Append) && modifiedStock.write("\n") == 1, "cannot modify stock fixture");
        modifiedStock.close();
        const auto staleStock = MachiningProjectController::loadAndRegenerate(load);
        require(!staleStock.ok() && staleStock.error.contains("stock", Qt::CaseInsensitive),
            "changed external stock was silently accepted on reopen");
        XCAFApp_Application::GetApplication()->Close(document);
        std::cout << "Finishing formal workspace, tree, save/reload, NC and desktop IPW passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}

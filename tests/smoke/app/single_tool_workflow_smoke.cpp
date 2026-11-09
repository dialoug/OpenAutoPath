#include "app/workflows/hole/HoleToolpathGenerationUseCase.h"
#include "app/workflows/pocket/PocketToolpathGenerationUseCase.h"
#include "app/workflows/profile/ProfileToolpathGenerationUseCase.h"
#include "app/workflows/project/MachiningProjectInstaller.h"
#include "app/machining/MachiningWorkspaceCoordinator.h"
#include "analysis/ModelIndex.h"
#include "recognition/FeatureRecognizer.h"
#include "recognition/ThreeAxisWireTraceUseCase.h"
#include "tooling/adapter/hole/HoleMachiningInputMapper.h"
#include "toolpath/profile/geometry/ProfileRegionBuilder.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "manufacturing/ipw/IpwExecutor.h"
#include "manufacturing/job/MachiningOperationService.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <STEPControl_Writer.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <QCoreApplication>
#include <QTemporaryDir>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
namespace a = oap::app;
namespace t = oap::toolpath;
namespace m = oap::manufacturing;
void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}
t::ToolpathToolContext directTool(int number, oap::tooling::ToolType type, double diameter)
{
    t::ToolpathToolContext tool;
    tool.valid = true;
    tool.toolNumber = number;
    tool.toolId = "direct-" + std::to_string(number);
    tool.toolName = tool.toolId;
    tool.toolType = oap::tooling::toolTypeId(type).toStdString();
    tool.diameter = diameter;
    tool.cuttingEdgeLength = 20;
    tool.overallLength = 30;
    tool.spindleSpeed = 6000;
    tool.horizontalFeed = 600;
    tool.verticalFeed = 150;
    if (type == oap::tooling::ToolType::Drill) tool.tipAngle = 118;
    return tool;
}
void checkTools(const m::MachiningJob& job)
{
    require(job.definition().activeToolLibraryId == "existing-user-library", "direct tool changed active library preference");
    for (const auto& operation : m::MachiningOperationService::operations(job.definition())) {
        const auto* controller = job.toolControllerById(operation.toolControllerId);
        require(controller && controller->libraryId.empty() && controller->tool.valid &&
            (controller->tool.toolNumber == 7 || controller->tool.toolNumber == 8),
            "standalone operation lost its frozen tool or retained a library dependency");
    }
}
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    try {
        QTemporaryDir directory;
        require(directory.isValid(), "temporary directory unavailable");
        auto shape = BRepAlgoAPI_Cut(BRepPrimAPI_MakeBox(32, 24, 6).Shape(),
            BRepPrimAPI_MakeBox(gp_Pnt(3, 3, 3), 12, 12, 4).Shape()).Shape();
        shape = BRepAlgoAPI_Cut(shape, BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(24, 12, -1), gp_Dir(0, 0, 1)), 3, 8).Shape()).Shape();
        const auto sourcePath = directory.filePath("single-tools.step");
        STEPControl_Writer writer;
        require(writer.Transfer(shape, STEPControl_AsIs) == IFSelect_RetDone &&
            writer.Write(sourcePath.toUtf8().constData()) == IFSelect_RetDone, "fixture STEP write failed");
        Handle(TDocStd_Document) document;
        XCAFApp_Application::GetApplication()->NewDocument("MDTV-XCAF", document);
        XCAFDoc_DocumentTool::ShapeTool(document->Main())->AddShape(shape, Standard_False);
        oap::Model model;
        oap::StepIndexer indexer;
        require(indexer.build(document, model), "model indexing failed");
        oap::ModelIndex modelIndex;
        modelIndex.build(model);
        const oap::Bbox bounds{0, 0, 0, 32, 24, 6};
        const auto mold = oap::MoldCoordinateFrame::fromBounds(bounds);
        auto frames = t::MoldSetupWorkFrames::fromMold(bounds, mold);
        for (int face = 0; face < 6; ++face) frames.setConfirmed(static_cast<oap::MoldSetupFace>(face), true);
        const auto recognized = oap::FeatureRecognizer{}.recognize(model, indexer);
        const auto holes = oap::tooling::HoleMachiningInputMapper{}.resolve(recognized.holeGeometries, bounds, mold);
        require(!holes.empty(), "fixture hole recognition failed");
        oap::ThreeAxisWireTraceRequest traceRequest;
        traceRequest.model = &model;
        traceRequest.modelIndex = &modelIndex;
        traceRequest.stepIndexer = &indexer;
        for (const auto& [face, kind] : recognized.faceKind) traceRequest.recognizedHoleFaceIds.insert(face);
        const auto traced = oap::ThreeAxisWireTraceUseCase{}.execute(traceRequest);
        require(traced.ok(), traced.error);
        std::vector<t::RootInnerWirePocketSource> pocketSources;
        for (const auto& source : traced.pocketSources) {
            t::RootInnerWirePocketSource mapped;
            mapped.kind = source.kind;
            mapped.sourceFaceIds = source.sourceFaceIds;
            mapped.rootFaceId = source.rootFaceId;
            mapped.rootWireId = source.rootWireId;
            mapped.allowOpenSectionContours = source.allowOpenSectionContours;
            mapped.openBoundaryFaceCount = source.openBoundaryFaceCount;
            pocketSources.push_back(std::move(mapped));
        }
        std::vector<t::SkippedRegion> skipped;
        const auto pockets = t::PocketRegionBuilder{}.buildRootInnerWireRegions(pocketSources, indexer, {}, &skipped);
        std::vector<t::RootOuterWireProfileSource> profileSources;
        std::vector<t::RootOuterWireProfileSource> allProfileSources;
        for (const auto& source : traced.profileSources) {
            t::RootOuterWireProfileSource mapped;
            mapped.unitId = source.unitId;
            mapped.axisLabel = source.axisLabel;
            mapped.normalDirection = source.normalDirection;
            mapped.rootFaceId = source.rootFaceId;
            mapped.rootWireId = source.rootWireId;
            mapped.sourceFaceIds = source.sourceFaceIds;
            mapped.machiningFaceIds = source.machiningFaceIds;
            mapped.terminalFaceIds = source.terminalFaceIds;
            mapped.mainOuterFaceIds = source.mainOuterFaceIds;
            allProfileSources.push_back(mapped);
            if (source.axisLabel != "Z" || source.normalDirection != 1) continue;
            profileSources.push_back(std::move(mapped));
        }
        const auto profiles = t::ProfileRegionBuilder{}.build(profileSources, indexer);
        require(!pockets.empty() && !profiles.empty(), "fixture pocket/profile recognition failed");
        const auto drill = directTool(7, oap::tooling::ToolType::Drill, 6);
        const auto mill = directTool(8, oap::tooling::ToolType::EndMill, 2);
        oap::tooling::ToolLibrary fallback("fallback-library", "Fallback");
        QString error;
        oap::tooling::ToolBit otherTool;
        otherTool.id = "fallback-end-mill";
        otherTool.name = "Fallback end mill";
        otherTool.type = oap::tooling::ToolType::EndMill;
        otherTool.geometry.diameter = 2;
        otherTool.geometry.cuttingEdgeLength = 20;
        otherTool.geometry.overallLength = 30;
        oap::tooling::LibraryToolSlot otherSlot;
        otherSlot.toolId = otherTool.id;
        otherSlot.toolNumber = 99;
        otherSlot.cuttingDefaults.horizontalFeed = 600;
        otherSlot.cuttingDefaults.verticalFeed = 150;
        require(fallback.upsertTool(otherTool, &error) && fallback.assignToolToSlot(otherSlot, &error), error.toStdString());
        t::ToolpathParams params;
        params.depth.stepDown = 2;
        params.profile.machiningMode = t::ProfileMachiningMode::ContourOnly;
        m::MachiningJob job;
        a::MachiningProjectDefinitionContext definition;
        definition.sourcePath = sourcePath;
        definition.activeToolLibraryId = "existing-user-library";
        a::HoleToolpathGenerationRequest holeRequest;
        holeRequest.recognized = &holes;
        holeRequest.model = &model;
        holeRequest.indexer = &indexer;
        holeRequest.moldBounds = bounds;
        holeRequest.moldFrame = &mold;
        holeRequest.params = &params;
        holeRequest.currentJob = &job;
        holeRequest.definition = definition;
        holeRequest.setupFrames = &frames;
        holeRequest.singleTool = &drill;
        auto holeResult = a::HoleToolpathGenerationUseCase::execute(holeRequest);
        require(holeResult.ok(), "hole: " + holeResult.error.toStdString());
        job = std::move(holeResult.committed.job);
        const auto originalHoleFingerprint = job.runtime().holeExecutionFingerprint;
        const auto originalRecognitionProfile = job.definition().part.recognitionProfileVersion;
        require(!originalRecognitionProfile.empty(), "first hole generation did not bind a recognition profile identity");
        auto legacyIdentity = job;
        legacyIdentity.definition().part.recognitionProfileVersion = "legacy-indexer-profile";
        require(a::MachiningProjectController::ensureDefinition(legacyIdentity, definition, &error) &&
            legacyIdentity.definition().part.recognitionProfileVersion == "legacy-indexer-profile",
            "context refresh replaced an existing explicit recognition profile identity");
        checkTools(job);
        require(job.hasHoleResult(), "direct drill did not produce a formal program");
        // Existing automatic policy reserves through holes for drilling and
        // mills planar-bottom blind holes/counterbore shoulders. Direct tools
        // must respect this policy instead of forcing a manual strategy.
        holeRequest.singleTool = &mill;
        auto milledHole = a::HoleToolpathGenerationUseCase::execute(holeRequest);
        require(!milledHole.ok(), "single end mill bypassed the through-hole drilling policy");
        const auto blindShape = BRepAlgoAPI_Cut(BRepPrimAPI_MakeBox(32, 24, 6).Shape(),
            BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(24, 12, 3), gp_Dir(0, 0, 1)), 3, 4).Shape()).Shape();
        const auto blindPath = directory.filePath("blind-mill.step");
        STEPControl_Writer blindWriter;
        require(blindWriter.Transfer(blindShape, STEPControl_AsIs) == IFSelect_RetDone &&
            blindWriter.Write(blindPath.toUtf8().constData()) == IFSelect_RetDone, "blind fixture STEP write failed");
        Handle(TDocStd_Document) blindDocument;
        XCAFApp_Application::GetApplication()->NewDocument("MDTV-XCAF", blindDocument);
        XCAFDoc_DocumentTool::ShapeTool(blindDocument->Main())->AddShape(blindShape, Standard_False);
        oap::Model blindModel;
        oap::StepIndexer blindIndexer;
        require(blindIndexer.build(blindDocument, blindModel), "blind fixture indexing failed");
        const auto blindRecognition = oap::FeatureRecognizer{}.recognize(blindModel, blindIndexer);
        const auto blindInputs = oap::tooling::HoleMachiningInputMapper{}.resolve(blindRecognition.holeGeometries, bounds, mold);
        require(!blindInputs.empty(), "blind fixture recognition failed");
        m::MachiningJob emptyBlindJob;
        auto blindRequest = holeRequest;
        blindRequest.model = &blindModel;
        blindRequest.indexer = &blindIndexer;
        blindRequest.recognized = &blindInputs;
        blindRequest.currentJob = &emptyBlindJob;
        blindRequest.definition.sourcePath = blindPath;
        auto blindResult = a::HoleToolpathGenerationUseCase::execute(blindRequest);
        require(blindResult.ok(), "blind milled hole: " + blindResult.error.toStdString());
        checkTools(blindResult.committed.job);
        XCAFApp_Application::GetApplication()->Close(blindDocument);

        a::PocketToolpathGenerationRequest pocketRequest;
        pocketRequest.candidateCount = static_cast<int>(pocketSources.size());
        pocketRequest.regions = &pockets;
        pocketRequest.buildSkipped = &skipped;
        pocketRequest.params = &params;
        pocketRequest.moldFrame = &mold;
        pocketRequest.currentJob = &job;
        pocketRequest.definition = definition;
        pocketRequest.rootSources = &pocketSources;
        pocketRequest.setupFrames = &frames;
        pocketRequest.singleTool = &mill;
        auto pocketResult = a::PocketToolpathGenerationUseCase::execute(pocketRequest);
        require(pocketResult.ok(), "pocket: " + pocketResult.error.toStdString());
        job = std::move(pocketResult.candidateJob.job);
        checkTools(job);

        a::ProfileToolpathGenerationRequest profileRequest;
        profileRequest.recognizedRegions = &profiles;
        profileRequest.params = &params;
        profileRequest.moldFrame = &mold;
        profileRequest.currentJob = &job;
        profileRequest.definition = definition;
        profileRequest.rootSources = &profileSources;
        profileRequest.setupFrames = &frames;
        profileRequest.singleTool = &mill;
        auto profileResult = a::ProfileToolpathGenerationUseCase::execute(profileRequest);
        const auto allProfiles = t::ProfileRegionBuilder{}.build(allProfileSources, indexer);
        require(allProfiles.size() > profiles.size(), "fixture needs multiple directions");
        auto automaticDirectionRequest = profileRequest;
        automaticDirectionRequest.recognizedRegions = &allProfiles;
        automaticDirectionRequest.rootSources = &allProfileSources;
        automaticDirectionRequest.automaticSetupDirections = true;
        const auto automaticDirectionResult = a::ProfileToolpathGenerationUseCase::execute(automaticDirectionRequest);
        require(automaticDirectionResult.ok() && automaticDirectionResult.regions.size() == 1 &&
                !automaticDirectionResult.directionReport.empty(),
            "automatic profile directions: " + automaticDirectionResult.error.toStdString());
        for (const auto& operation : automaticDirectionResult.batch.operations)
            require(operation.setupFace == oap::MoldSetupFace::PositiveZ && operation.sourceRegionIndex == 0,
                "automatic direction selection lost its setup/source mapping");
        auto conflictingDirectionRequest = automaticDirectionRequest;
        conflictingDirectionRequest.selectedSetupFace = oap::MoldSetupFace::PositiveZ;
        require(!a::ProfileToolpathGenerationUseCase::execute(conflictingDirectionRequest).ok(),
                "automatic and explicit setup selection were accepted together");
        auto allDirectionsRequest = automaticDirectionRequest;
        allDirectionsRequest.automaticSetupDirections = false;
        require(!a::ProfileToolpathGenerationUseCase::execute(allDirectionsRequest).ok(),
                "all-directions mode silently omitted the unreachable side");
        // A selected top setup must ignore an unreachable side profile before
        // generation, and the retained source mapping must survive Job capture.
        auto mixedProfiles = profiles;
        auto deepSide = profiles.front();
        deepSide.depth = 120.0;
        deepSide.openDir = gp_Dir(1, 0, 0);
        mixedProfiles.push_back(deepSide);
        auto scopedRequest = profileRequest;
        scopedRequest.recognizedRegions = &mixedProfiles;
        scopedRequest.selectedSetupFace = oap::MoldSetupFace::PositiveZ;
        const auto scoped = a::ProfileToolpathGenerationUseCase::execute(scopedRequest);
        require(scoped.ok() && scoped.regions.size() == profiles.size(),
            "selected +Z profiles were blocked by a deep side: " + scoped.error.toStdString());
        for (const auto& operation : scoped.batch.operations)
            require(operation.setupFace == oap::MoldSetupFace::PositiveZ && operation.sourceRegionIndex >= 0 &&
                operation.sourceRegionIndex < static_cast<int>(scoped.regions.size()), "selected profile mapping changed");
        scopedRequest.selectedSetupFace = oap::MoldSetupFace::NegativeY;
        require(!a::ProfileToolpathGenerationUseCase::execute(scopedRequest).ok(),
            "empty selected profile direction silently generated other directions");
        require(profileResult.ok(), "profile: " + profileResult.error.toStdString());
        job = std::move(profileResult.committed.job);
        checkTools(job);
        require(job.profileResult()->effectiveParams.profile.machiningMode ==
                    t::ProfileMachiningMode::ContourOnly &&
                job.profileResult()->operations.front().tool.diameter == mill.diameter &&
                job.profileResult()->operations.front().tool.horizontalFeed == mill.horizontalFeed,
            "single contour lost its mode or complete tool snapshot");
        auto clearingParams = params;
        clearingParams.profile.machiningMode = t::ProfileMachiningMode::ExternalClearing;
        auto clearingRequest = profileRequest;
        clearingRequest.params = &clearingParams;
        clearingRequest.definition.stock.marginX = 2.0;
        clearingRequest.definition.stock.marginY = 2.0;
        const auto clearingResult = a::ProfileToolpathGenerationUseCase::execute(clearingRequest);
        require(clearingResult.ok(), "single profile external clearing: " + clearingResult.error.toStdString());
        checkTools(clearingResult.committed.job);
        require(clearingResult.batch.effectiveParams.profile.machiningMode ==
                    t::ProfileMachiningMode::ExternalClearing &&
                !clearingResult.automaticSelection.stages.empty(),
            "single external clearing did not use the staged clearing planner");
        const auto fingerprint = m::MachiningJobStorage::definitionFingerprint(job.definition());
        require(job.definition().part.recognitionProfileVersion == originalRecognitionProfile &&
            job.runtime().holeExecutionFingerprint == originalHoleFingerprint &&
            m::MachiningJobStorage::holeExecutionFingerprint(job.definition()).toStdString() == originalHoleFingerprint,
            "adding pocket/profile operations changed the existing hole recognition identity or execution fingerprint");
        a::MachiningWorkspaceCoordinator previews;
        previews.installProfileProgram(job, params, job.profileResult()->effectiveParams);
        t::ToolpathProgram selectedProgram;
        QString previewError;
        for (const auto& operation : m::MachiningOperationService::operations(job.definition())) {
            const bool activated = previews.activateOperationPreview(operation.id, selectedProgram, &previewError);
            require(activated && selectedProgram.moveCount() > 0,
                "cannot switch " + std::string(m::jobOperationKindId(operation.kind)) + " " + operation.id +
                " in the current mixed Job preview: " + previewError.toStdString());
        }
        const auto previewSave = directory.filePath("preview-switch.oapjob");
        m::MachiningJobDefinition afterPreviewSwitch;
        require(previews.saveProject(previewSave, &previewError) &&
            m::MachiningJobStorage::load(previewSave, afterPreviewSwitch, &previewError) &&
            m::MachiningJobStorage::definitionFingerprint(afterPreviewSwitch) == fingerprint &&
            m::MachiningJobStorage::holeExecutionFingerprint(afterPreviewSwitch).toStdString() == job.runtime().holeExecutionFingerprint &&
            m::MachiningJobStorage::pocketExecutionFingerprint(afterPreviewSwitch).toStdString() == job.runtime().pocketExecutionFingerprint &&
            m::MachiningJobStorage::profileExecutionFingerprint(afterPreviewSwitch).toStdString() == job.runtime().profileExecutionFingerprint &&
            previews.currentPocketResult() && previews.currentProfileResult() &&
            previews.activateOperationPreview(job.definition().holeOperations.front().id, selectedProgram, &previewError),
            "preview switching mutated the Job definition or invalidated its original execution identities: " + previewError.toStdString());
        auto invalid = mill;
        invalid.toolNumber = 0;
        holeRequest.singleTool = pocketRequest.singleTool = profileRequest.singleTool = &invalid;
        holeRequest.automaticLibrary = pocketRequest.automaticLibrary = profileRequest.automaticLibrary = &fallback;
        require(!a::HoleToolpathGenerationUseCase::execute(holeRequest).ok() &&
            !a::PocketToolpathGenerationUseCase::execute(pocketRequest).ok() &&
            !a::ProfileToolpathGenerationUseCase::execute(profileRequest).ok(), "invalid direct T fell back to a library tool");
        invalid.toolNumber = 10000;
        require(!a::HoleToolpathGenerationUseCase::execute(holeRequest).ok() &&
            !a::PocketToolpathGenerationUseCase::execute(pocketRequest).ok() &&
            !a::ProfileToolpathGenerationUseCase::execute(profileRequest).ok(), "out-of-range direct T accepted");
        invalid = drill;
        require(!a::PocketToolpathGenerationUseCase::execute(pocketRequest).ok() &&
            !a::ProfileToolpathGenerationUseCase::execute(profileRequest).ok(), "drill accepted for milling-only operations");
        invalid = mill;
        invalid.diameter = std::numeric_limits<double>::quiet_NaN();
        require(a::HoleToolpathGenerationUseCase::execute(holeRequest).failure == a::HoleToolpathGenerationFailure::InvalidRequest,
            "nonfinite geometry accepted");
        invalid = mill;
        invalid.verticalFeed = 0;
        require(a::PocketToolpathGenerationUseCase::execute(pocketRequest).failure == a::PocketToolpathGenerationFailure::InvalidRequest,
            "zero direct cutting feed accepted");
        const auto cancel = [](const a::WorkflowProgressRequest& progress) { progress.cancel(); };
        holeRequest.singleTool = &drill;
        pocketRequest.singleTool = profileRequest.singleTool = &mill;
        holeRequest.interaction.showProgress = pocketRequest.interaction.showProgress = profileRequest.interaction.showProgress = cancel;
        require(a::HoleToolpathGenerationUseCase::execute(holeRequest).failure == a::HoleToolpathGenerationFailure::Cancelled,
            "cancelled single-drill candidate was published");
        require(a::PocketToolpathGenerationUseCase::execute(pocketRequest).failure == a::PocketToolpathGenerationFailure::Cancelled,
            "cancelled single-pocket candidate was published");
        require(a::ProfileToolpathGenerationUseCase::execute(profileRequest).failure == a::ProfileToolpathGenerationFailure::Cancelled,
            "cancelled single-profile candidate was published");
        clearingRequest.interaction.showProgress = cancel;
        require(a::ProfileToolpathGenerationUseCase::execute(clearingRequest).failure == a::ProfileToolpathGenerationFailure::Cancelled,
            "cancelled single-profile clearing candidate was published");
        automaticDirectionRequest.interaction.showProgress = cancel;
        require(a::ProfileToolpathGenerationUseCase::execute(automaticDirectionRequest).failure == a::ProfileToolpathGenerationFailure::Cancelled,
            "cancelled direction planning published a candidate");
        require(m::MachiningJobStorage::definitionFingerprint(job.definition()) == fingerprint,
            "failed/cancelled request mutated the installed job");
        const auto saved = directory.filePath("single-tools.oapjob");
        require(m::MachiningJobStorage::save(saved, job.definition(), &error), error.toStdString());
        a::MachiningProjectLoadRequest load;
        load.filePath = saved;
        load.currentSourcePath = sourcePath;
        load.model = &model;
        load.indexer = &indexer;
        load.moldFrame = &mold;
        load.setupFrames = &frames;
        auto reopened = a::MachiningProjectController::loadAndRegenerate(load);
        require(reopened.ok(), "reload without any user library: " + reopened.error.toStdString());
        checkTools(reopened.job);
        require(reopened.job.hasHoleResult() && reopened.job.currentPocketResult() && reopened.job.profileResult() &&
            reopened.job.runtime().profileExecutionFingerprint ==
                m::MachiningJobStorage::profileExecutionFingerprint(reopened.job.definition()).toStdString(),
            "reopened direct tools lost a current formal program");
        require(reopened.job.definition().profileParameters.profile.machiningMode ==
                    t::ProfileMachiningMode::ContourOnly &&
                reopened.job.profileResult()->operations.front().tool.horizontalFeed == mill.horizontalFeed,
            "reopened single contour lost its mode or tool cutting defaults");
        const auto autoSaved = directory.filePath("automatic-directions.oapjob");
        require(m::MachiningJobStorage::save(autoSaved, automaticDirectionResult.committed.job.definition(), &error), error.toStdString());
        auto autoLoad = load;autoLoad.filePath = autoSaved;
        const auto autoReopened = a::MachiningProjectController::loadAndRegenerate(autoLoad);
        require(autoReopened.ok() && autoReopened.job.profileResult() &&
                autoReopened.job.runtime().profileExecutionFingerprint == automaticDirectionResult.committed.job.runtime().profileExecutionFingerprint,
            "automatic direction job lost its execution identity on reopen");
        for (const auto& operation : autoReopened.job.profileResult()->operations)
            require(operation.setupFace == oap::MoldSetupFace::PositiveZ,
                "reopen restored omitted directions");
        a::MachiningWorkspaceSession workspace;
        a::MachiningConfigurationSession configuration;
        a::ToolLibraryController libraries;
        a::MachiningProjectInstaller::install(std::move(reopened), workspace, configuration, libraries, frames);
        require(configuration.holeToolSelectionMode() == a::ToolSelectionMode::SingleTool &&
            configuration.pocketToolSelectionMode() == a::ToolSelectionMode::SingleTool &&
            configuration.profileToolSelectionMode() == a::ToolSelectionMode::SingleTool &&
            configuration.holeSingleTool().toolNumber == 7 && configuration.holeSingleTool().diameter == 6 &&
            configuration.pocketSingleTool().toolNumber == 8 && configuration.pocketSingleTool().diameter == 2 &&
            configuration.profileSingleTool().toolNumber == 8 && configuration.profileSingleTool().horizontalFeed == 600,
            "project installation did not restore all direct-tool modes and snapshots");
        require(a::MachiningConfigurationSession::parametersEqual(configuration.holeParams(), workspace.job().definition().holeParameters) &&
            a::MachiningConfigurationSession::parametersEqual(configuration.pocketParams(), workspace.job().definition().pocketParameters) &&
            a::MachiningConfigurationSession::parametersEqual(configuration.profileParams(), workspace.job().definition().profileParameters),
            "project installation left unrelated local defaults in the process parameter editor");

        // Generate both rounded families from standalone tools, then rebuild from
        // the saved Job without a library and run the actual unified IPW executor.
        double flatVolume = 0.0;
        for (const auto type : {oap::tooling::ToolType::EndMill,
                oap::tooling::ToolType::BallEndMill, oap::tooling::ToolType::BullNose}) {
            auto roundedTool = directTool(8, type, 2);
            if (type == oap::tooling::ToolType::BullNose) roundedTool.cornerRadius = 0.4;
            const double expectedRadius = type == oap::tooling::ToolType::BallEndMill ? 1.0 : roundedTool.cornerRadius;
            auto roundedPocketRequest = pocketRequest;
            roundedPocketRequest.singleTool = &roundedTool;
            roundedPocketRequest.interaction = {};
            auto roundedPocket = a::PocketToolpathGenerationUseCase::execute(roundedPocketRequest);
            require(roundedPocket.ok(), "rounded pocket: " + roundedPocket.error.toStdString());
            auto roundedJob = std::move(roundedPocket.candidateJob.job);
            auto roundedProfileRequest = profileRequest;
            roundedProfileRequest.currentJob = &roundedJob;
            roundedProfileRequest.singleTool = &roundedTool;
            roundedProfileRequest.interaction = {};
            auto roundedProfile = a::ProfileToolpathGenerationUseCase::execute(roundedProfileRequest);
            require(roundedProfile.ok(), "rounded profile: " + roundedProfile.error.toStdString());
            roundedJob = std::move(roundedProfile.committed.job);
            if (expectedRadius > 0) require(roundedPocket.summary.contains(QStringLiteral("余料")) &&
                roundedProfile.summary.contains(QStringLiteral("余料")), "rounded residual warning missing from UI");
            const auto path = directory.filePath("rounded-tools.oapjob");
            require(m::MachiningJobStorage::save(path, roundedJob.definition(), &error), error.toStdString());
            auto roundedLoad = load;
            roundedLoad.filePath = path;
            auto restored = a::MachiningProjectController::loadAndRegenerate(roundedLoad);
            require(restored.ok(), "rounded reload: " + restored.error.toStdString());
            for (const auto& op : restored.job.currentPocketResult()->operations)
                require(op.tool.cornerRadius == expectedRadius && op.tool.toolType == roundedTool.toolType,
                    "pocket regeneration flattened the cutter");
            for (const auto& op : restored.job.profileResult()->operations)
                require(op.tool.cornerRadius == expectedRadius && op.tool.toolType == roundedTool.toolType,
                    "profile regeneration flattened the cutter");
            m::ipw::IpwExecutionOptions options;
            options.pocket.zMapResolution = options.hole.zMapResolution = 0.2;
            const auto ipw = m::ipw::IpwExecutor{}.execute(restored.job,
                BRepPrimAPI_MakeBox(gp_Pnt(-2, -2, 0), 36, 28, 6).Shape(),
                "rounded-milling-stock", m::ipw::kHybridIpwEngineId, options);
            require(ipw.complete() && ipw.finalCheckpoint && ipw.finalCheckpoint->dexelStock,
                "rounded unified IPW: " + ipw.message);
            if (expectedRadius == 0) flatVolume = ipw.finalCheckpoint->volume;
            else require(ipw.finalCheckpoint->volume > flatVolume + 0.1,
                "rounded IPW must retain more floor/root material than a flat cutter");
            std::cout << "MILLING_IPW type=" << roundedTool.toolType
                      << " volume=" << ipw.finalCheckpoint->volume << '\n';
        }
        auto badRounded = directTool(8, oap::tooling::ToolType::BullNose, 2);
        auto invalidRoundedRequest = pocketRequest;
        invalidRoundedRequest.interaction = {};
        invalidRoundedRequest.singleTool = &badRounded;
        for (double radius : {0.0, 1.0, 2.0}) {
            badRounded.cornerRadius = radius;
            require(a::PocketToolpathGenerationUseCase::execute(invalidRoundedRequest).failure ==
                a::PocketToolpathGenerationFailure::InvalidRequest, "invalid bull-nose radius accepted");
        }
        badRounded = directTool(8, oap::tooling::ToolType::BallEndMill, 2);
        badRounded.cornerRadius = 0.5;
        require(a::PocketToolpathGenerationUseCase::execute(invalidRoundedRequest).failure ==
            a::PocketToolpathGenerationFailure::InvalidRequest, "inconsistent ball radius accepted");
        badRounded.cornerRadius = 0;
        badRounded.cuttingEdgeLength = 0.5;
        require(a::PocketToolpathGenerationUseCase::execute(invalidRoundedRequest).failure ==
            a::PocketToolpathGenerationFailure::InvalidRequest, "ball shorter than its hemisphere accepted");
        XCAFApp_Application::GetApplication()->Close(document);
        std::cout << "Standalone hole/pocket/profile tools, validation, cancellation and snapshot regeneration passed.\n";
        return 0;
    } catch (const std::exception& failure) {
        std::cerr << "FAIL: " << failure.what() << '\n';
        return 1;
    }
}

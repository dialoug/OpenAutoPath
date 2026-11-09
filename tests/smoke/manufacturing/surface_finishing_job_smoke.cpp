#include "manufacturing/finishing/SurfaceFinishingMachiningJobService.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "manufacturing/job/MachiningJobApplicationService.h"
#include "manufacturing/job/MachiningOperationService.h"
#include "analysis/MoldCoordinateFrame.h"
#include <BRepPrimAPI_MakeBox.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool good, const std::string& message) { if (!good) throw std::runtime_error(message); }
}
int main()
{
    using namespace oap;
    using namespace oap::manufacturing;
    try {
        const auto shape = BRepPrimAPI_MakeBox(12, 8, 3).Shape();
        Handle(TDocStd_Document) document;
        XCAFApp_Application::GetApplication()->NewDocument("MDTV-XCAF", document);
        XCAFDoc_DocumentTool::ShapeTool(document->Main())->AddShape(shape, false);
        StepIndexer indexer;
        Model model;
        require(indexer.build(document, model), "fixture indexing failed");
        const Bbox bounds{0,0,0,12,8,3};
        auto frames = toolpath::MoldSetupWorkFrames::fromMold(bounds, MoldCoordinateFrame::fromBounds(bounds));
        frames.setConfirmed(MoldSetupFace::PositiveZ, true);
        toolpath::SurfaceFinishingParams params;
        params.diameter = 4;
        params.stepOver = 2;
        const auto generated = toolpath::SurfaceFinishing().generate(shape,
            frames.forFace(MoldSetupFace::PositiveZ)->setupWorkFrame, params);
        require(generated.ok, generated.error);
        toolpath::ToolpathToolContext tool;
        tool.valid = true;
        tool.toolNumber = 4;
        tool.toolType = "BallEndMill";
        tool.diameter = params.diameter;
        tool.cornerRadius = params.diameter / 2;
        tool.cuttingEdgeLength = params.cuttingLength;
        tool.horizontalFeed = params.feed;
        tool.verticalFeed = params.plungeFeed;
        MachiningJob job;
        job.definition().id = "finishing-fixture";
        job.definition().part.sourcePath = "generated-box.step";
        job.definition().part.sourceFingerprint = "box-v1";
        SurfaceFinishingJobCaptureRequest request;
        request.result = &generated;
        request.tool = &tool;
        request.setupFrames = &frames;
        request.setupFace = MoldSetupFace::PositiveZ;
        for (const auto& unit : model.units) request.unitIds.push_back(unit.unit_id);
        SurfaceFinishingMachiningJobService service;
        std::string error;
        require(service.capture(job, request, &error), error);
        require(job.currentFinishingResult() && job.validationErrors().empty(), "captured result is not current");
        const auto firstId = job.definition().finishingOperations.front().id;
        require(service.capture(job, request, &error), error);
        require(job.definition().finishingOperations.size() == 2 && job.currentFinishingResult()->operations.size() == 2,
            "append discarded an existing operation");
        require(job.configureIpwPlan("stock", "engine").ok(), "IPW plan configuration failed");
        for (std::size_t i = 0; i < job.ipwSession().nodes().size(); ++i) {
            require(job.ipwSession().beginNode(i), "IPW test node could not begin");
            ipw::IpwArtifactDescriptor artifact;
            artifact.stateKey = job.ipwSession().nodes()[i].stateKey;
            artifact.backendId = "test";
            artifact.contentFingerprint = "material";
            artifact.checkpoint = true;
            require(job.ipwSession().completeNode(i, artifact), "IPW test node could not become ready");
        }
        auto revisedParams = params;
        revisedParams.meshDeflection /= 2;
        const auto revised = toolpath::SurfaceFinishing().generate(shape,
            frames.forFace(MoldSetupFace::PositiveZ)->setupWorkFrame, revisedParams);
        require(revised.ok, revised.error);
        request.result = &revised;
        request.replaceOperationId = firstId;
        require(service.capture(job, request, &error), error);
        require(job.ipwSession().nodes().front().status == ipw::IpwNodeStatus::Dirty &&
            job.ipwSession().nodes().back().status == ipw::IpwNodeStatus::Dirty,
            "editing finishing did not immediately invalidate its IPW suffix");
        require(job.definition().finishingOperations.size() == 2 && job.definition().finishingOperations.front().sequence == 0,
            "edit changed stable identity or order");
        auto corrupted = job;
        corrupted.runtime().finishingResult.operations.pop_back();
        require(!corrupted.currentFinishingResult(), "partial runtime passed publication gate");
        const auto before = MachiningJobStorage::definitionFingerprint(job.definition());
        tool.diameter += 1;
        require(!service.capture(job, request, &error) && MachiningJobStorage::definitionFingerprint(job.definition()) == before,
            "capture accepted a mismatched tool or mutated the job on failure");
        tool.diameter -= 1;

        const QByteArray saved = MachiningJobStorage::serializeDefinition(job.definition());
        require(!saved.contains("\"moves\"") && !saved.contains("\"primitives\""), "runtime geometry leaked into persistence");
        QString loadError;
        MachiningJob restored;
        require(MachiningJobStorage::deserializeDefinition(saved, restored.definition(), &loadError), loadError.toStdString());
        require(restored.definition().schemaVersion == manufacturing::kMachiningJobSchemaVersion && !restored.hasFinishingResult(), "load must rebuild runtime");
        const auto rebuilt = service.regenerate(restored, indexer);
        require(!rebuilt.cancelled && rebuilt.validation.ok() && restored.currentFinishingResult(), rebuilt.validation.summary());
        require(rebuilt.toProgram().moveCount() == job.finishingResult()->toProgram().moveCount(), "save/load regeneration changed path");
        auto& controller = restored.definition().toolControllers.front();
        controller.tool.horizontalFeed += 100;
        require(!restored.currentFinishingResult(), "changed tool did not invalidate runtime");
        const auto updated = service.regenerate(restored, indexer);
        require(updated.validation.ok() && restored.currentFinishingResult(), "bound tool could not regenerate");
        toolpath::ToolpathGenerationControl cancel;
        cancel.isCancellationRequested = [] { return true; };
        const auto stableFingerprint = MachiningJobStorage::definitionFingerprint(restored.definition());
        const auto cancelled = service.regenerate(restored, indexer, &cancel);
        require(cancelled.cancelled && cancelled.operations.empty() && restored.currentFinishingResult() &&
            stableFingerprint == MachiningJobStorage::definitionFingerprint(restored.definition()), "cancel was not transactional");
        auto disabled = restored;
        for (auto& op : disabled.definition().finishingOperations) op.enabled = false;
        const auto disabledBatch = service.regenerate(disabled, indexer);
        require(disabledBatch.validation.ok() && disabledBatch.operations.empty() && !disabled.hasFinishingResult(),
            "disabled finishing produced executable runtime");
        request.replaceOperationId = disabled.definition().finishingOperations.front().id;
        require(service.capture(disabled, request, &error), error);
        require(!disabled.definition().finishingOperations.front().enabled && !disabled.hasFinishingResult(),
            "editing a disabled operation unexpectedly enabled or published it");

        auto root = QJsonDocument::fromJson(saved).object();
        auto jsonJob = root.value("job").toObject();
        auto operations = jsonJob.value("surface_finishing_operations").toArray();
        auto operation = operations[0].toObject();
        auto jsonParameters = operation.value("parameters").toObject();
        jsonParameters.insert("max_samples", -1);
        operation.insert("parameters", jsonParameters);
        operations[0] = operation;
        jsonJob.insert("surface_finishing_operations", operations);
        root.insert("job", jsonJob);
        MachiningJobDefinition rejected;
        require(!MachiningJobStorage::deserializeDefinition(QJsonDocument(root).toJson(), rejected, &loadError),
            "negative persisted budget was accepted");
        root = QJsonDocument::fromJson(saved).object();
        jsonJob = root.value("job").toObject();
        jsonJob.insert("surface_operations", QJsonArray{QJsonObject{{"legacy", true}}});
        root.insert("job", jsonJob);
        require(!MachiningJobStorage::deserializeDefinition(QJsonDocument(root).toJson(), rejected, &loadError),
            "legacy surface was silently migrated");
        XCAFApp_Application::GetApplication()->Close(document);
        std::cout << "Finishing Job capture, persistence, regeneration, invalidation and cancellation passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n'; return 1;
    }
}

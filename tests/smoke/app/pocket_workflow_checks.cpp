#include "app/workflows/pocket/PocketToolpathGenerationUseCase.h"
#include "app/workflows/shared/ToolpathExportController.h"
#include "app/workflows/simulation/StockSimulationWorkflowController.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include <BRepPrimAPI_MakeBox.hxx>
#include <QFile>
#include <QTemporaryDir>
#include <iostream>
#include <string_view>

namespace {
bool expect(bool condition, std::string_view message)
{
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}
} // namespace

bool runPocketSimulationChecks()
{
    using namespace oap;
    manufacturing::MachiningJob job;
    manufacturing::SetupDefinition setup;
    setup.id = "setup";
    setup.setupIndex = 0;
    job.definition().setups.push_back(setup);
    manufacturing::PocketOperationDefinition a;
    a.id = "A";
    a.setupId = setup.id;
    a.sequence = 1;
    auto b = a;
    b.id = "B";
    b.sequence = 2;
    job.definition().pocketOperations = {b, a};
    toolpath::PocketToolpathBatch batch;
    toolpath::PocketToolpathOperation first;
    first.operationId = "A";
    first.setupIndex = 0;
    auto second = first;
    second.operationId = "B";
    batch.operations = {second, first};
    batch.effectiveParams.tool.diameter = 7.0;
    const auto install = [&] {
        job.setPocketResult(batch,
            manufacturing::MachiningJobStorage::pocketExecutionFingerprint(
                job.definition()).toStdString());
    };
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape();
    app::StockSourceSettings source;
    source.sourceMode = 1;
    const auto prepare = [&] {
        return app::StockSimulationWorkflowController::preparePocket(
            job, 0, source, {}, 5, box, box);
    };
    install();
    const auto ordered = prepare();
    if (!expect(ordered.ok && ordered.operations.size() == 2 &&
                    ordered.operations[0].operationId == "A" &&
                    ordered.operations[1].operationId == "B" &&
                    ordered.effectiveParams.tool.diameter == 7.0,
                "simulation must use canonical operations in definition sequence"))
        return false;
    batch.operations[0].operationId = "C";
    install();
    if (!expect(!prepare().ok, "equal counts must not hide a missing runtime identity"))
        return false;
    batch.operations = {first, second, second};
    install();
    if (!expect(!prepare().ok, "duplicate runtime identities must be rejected"))
        return false;
    batch.operations = {second, first};
    install();
    job.definition().pocketParameters.depth.stepDown += 1.0;
    if (!expect(!prepare().ok, "stale Job results must not enter simulation"))
        return false;
    install();
    batch.cancelled = true;
    install();
    if (!expect(!job.currentPocketResult(), "cancelled results must fail the shared gate"))
        return false;
    batch.cancelled = false;
    batch.safetyValidation.errorCount = 1;
    install();
    if (!expect(!job.currentPocketResult(), "safety errors must fail the shared gate"))
        return false;
    batch.safetyValidation.errorCount = 0;
    if (!expect(!batch.readyForPublication(), "unchecked candidates cannot be published"))
        return false;
    batch.safetyValidationPerformed = true;
    if (!expect(batch.readyForPublication(), "checked valid candidates can be published"))
        return false;

    QTemporaryDir sourceDirectory;
    QFile sourceFile(sourceDirectory.filePath(QStringLiteral("part.fixture")));
    if (!sourceFile.open(QIODevice::WriteOnly) || sourceFile.write("fixture") != 7)
        return false;
    sourceFile.close();
    toolpath::MoldSetupWorkFrames frames;
    app::MachiningProjectPocketCaptureRequest capture;
    capture.definition.sourcePath = sourceFile.fileName();
    capture.operations = &batch.operations;
    capture.effectiveParams = &batch.effectiveParams;
    capture.setupFrames = &frames;
    const auto originalDefinition = manufacturing::MachiningJobStorage::serializeDefinition(
        job.definition());
    QString captureError;
    // Source initialization succeeds, then unresolved setup causes capture to fail.
    if (!expect(!app::MachiningProjectController::capturePocketOperations(
                    job, capture, &captureError) && !captureError.isEmpty() &&
                    manufacturing::MachiningJobStorage::serializeDefinition(job.definition()) ==
                        originalDefinition,
                "failed capture after source initialization must preserve the original Job"))
        return false;
    return true;
}

bool runPocketPresentationChecks()
{
    oap::toolpath::PocketToolpathOperation previewOperation;
    previewOperation.kind = oap::FeatureKind::Slot;
    previewOperation.sourceFaceIds = {"F2", "F1"};
    for (int index = 0; index < 3; ++index) {
        oap::toolpath::ToolpathMove move;
        move.type = oap::toolpath::ToolpathMoveType::Cut;
        move.from = gp_Pnt(static_cast<double>(index), 0.0, 0.0);
        move.to = gp_Pnt(static_cast<double>(index + 1), 0.0, 0.0);
        previewOperation.clearing.moves.push_back(move);
    }
    oap::toolpath::ToolpathParams pocketParams;
    pocketParams.clearing.pattern =
        oap::toolpath::ClearingPattern::FreePath;
    oap::app::PocketToolpathGenerationMetrics rejectedMetrics;
    rejectedMetrics.fullCutMoveCount = 3;
    std::vector<oap::app::PocketPreviewEntry> rejectedEntries;
    const bool rejected =
        oap::app::PocketToolpathGenerationUseCase::publishPreview(
            {previewOperation}, pocketParams, 1,
            [](oap::app::PocketPreviewEntry&) { return false; },
            rejectedEntries, rejectedMetrics);
    if (!expect(
            !rejected && rejectedEntries.empty() &&
                rejectedMetrics.skippedDisplaySegments == 3,
            "a rejected pocket presentation must not publish preview state")) {
        return false;
    }

    // A display failure must not turn a complete two-operation program into
    // a successful one-operation candidate, regardless of failure order.
    auto secondPreviewOperation = previewOperation;
    secondPreviewOperation.sourceFaceIds = {"F3"};
    for (int rejectedIndex = 0; rejectedIndex < 2; ++rejectedIndex) {
        oap::app::PocketToolpathGenerationMetrics mixedMetrics;
        mixedMetrics.fullCutMoveCount = 6;
        std::vector<oap::app::PocketPreviewEntry> mixedEntries;
        int callbackIndex = 0;
        const bool mixed =
            oap::app::PocketToolpathGenerationUseCase::publishPreview(
                {previewOperation, secondPreviewOperation}, pocketParams, 6,
                [&](oap::app::PocketPreviewEntry&) {
                    return callbackIndex++ != rejectedIndex;
                },
                mixedEntries, mixedMetrics);
        if (!expect(!mixed && mixedEntries.empty(),
                    "one rejected preview must prevent a partial pocket candidate")) {
            return false;
        }
    }

    oap::app::MachiningWorkspaceSession exportWorkspace;
    exportWorkspace.activePreview() = oap::app::ActiveToolpathPreview::Pocket;
    exportWorkspace.hasPocketPreviewParams() = true;
    oap::toolpath::PocketToolpathBatch exportBatch;
    exportBatch.operations = {previewOperation, secondPreviewOperation};
    exportBatch.operations[0].operationId = "export-A";
    exportBatch.operations[1].operationId = "export-B";
    exportBatch.effectiveParams = pocketParams;
    exportWorkspace.job().definition().pocketParameters = pocketParams;
    const auto exportFingerprint =
        oap::manufacturing::MachiningJobStorage::pocketExecutionFingerprint(
            exportWorkspace.job().definition()).toStdString();
    exportWorkspace.job().setPocketResult(exportBatch, exportFingerprint);
    oap::app::PocketPreviewEntry onlyVisibleEntry;
    onlyVisibleEntry.operation = exportBatch.operations.front();
    exportWorkspace.pocketPreviewEntries().push_back(onlyVisibleEntry);
    const auto completeExport = oap::app::ToolpathExportController::prepare(exportWorkspace);
    if (!expect(completeExport.ok && completeExport.program.operations.size() == 2 &&
                    completeExport.pocketOperations.size() == 2 &&
                    completeExport.program.operations[1].operationId == "export-B",
                "display filtering must not remove operations from Pocket export")) {
        std::cerr << completeExport.error.toStdString() << '\n';
        return false;
    }
    exportWorkspace.pocketPreviewEntries().clear();
    exportWorkspace.hasPocketPreviewParams() = false;
    if (!expect(oap::app::ToolpathExportController::prepare(exportWorkspace).ok,
                "complete Pocket results must remain exportable without display entries"))
        return false;
    exportWorkspace.job().definition().pocketParameters.depth.stepDown += 1.0;
    if (!expect(!oap::app::ToolpathExportController::prepare(exportWorkspace).ok,
                "changed Pocket inputs must invalidate export of the old result"))
        return false;
    exportWorkspace.job().definition().pocketParameters = pocketParams;
    exportWorkspace.job().setPocketResult(exportBatch, {});
    if (!expect(!oap::app::ToolpathExportController::prepare(exportWorkspace).ok,
                "a missing execution fingerprint must prevent Pocket export"))
        return false;
    exportWorkspace.job().clearPocketResult();
    exportWorkspace.pocketPreviewEntries().push_back(onlyVisibleEntry);
    exportWorkspace.hasPocketPreviewParams() = true;
    if (!expect(!oap::app::ToolpathExportController::prepare(exportWorkspace).ok,
                "preview entries must not substitute for a missing Job result"))
        return false;

    oap::app::PocketToolpathGenerationMetrics limitedMetrics;
    limitedMetrics.fullCutMoveCount = 3;
    std::vector<oap::app::PocketPreviewEntry> limitedEntries;
    const bool limited =
        oap::app::PocketToolpathGenerationUseCase::publishPreview(
            {previewOperation}, pocketParams, 1,
            [](oap::app::PocketPreviewEntry&) { return true; },
            limitedEntries, limitedMetrics);
    if (!expect(
            limited && limitedEntries.size() == 1 &&
                limitedEntries.front().key == "F1|F2" &&
                limitedMetrics.previewLimited &&
                limitedMetrics.cutMoveCount == 1 &&
                limitedMetrics.skippedPreviewCutMoves == 2,
            "pocket preview budgeting must be deterministic and window-free")) {
        return false;
    }

    return true;
}

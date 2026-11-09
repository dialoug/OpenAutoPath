// 自动孔批次到 Job 捕获/重建的稳定来源和 Operation 身份特征测试。
#include "manufacturing/hole/HoleMachiningJobService.h"
#include "manufacturing/ipw/IpwExecutor.h"
#include "tooling/adapter/hole/HoleAutomaticToolpath.h"

#include <BRepPrimAPI_MakeBox.hxx>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

oap::toolpath::HoleMachiningInput hole(double x)
{
    oap::toolpath::HoleMachiningInput result;
    result.kind = oap::FeatureKind::BlindHole;
    result.entry = gp_Pnt(x, 10.0, 10.0);
    result.bottom = gp_Pnt(x, 10.0, 4.0);
    result.radius = 2.0;
    result.bottomProfile = oap::HoleBottomProfile::Planar;
    result.entryResolution = oap::HoleEntryResolution::MoldEnvelopeProjected;
    result.setupFace = oap::MoldSetupFace::PositiveZ;
    result.preserveEntrySide = true;
    result.mappingState =
        oap::toolpath::HoleMachiningMappingState::Ready;
    result.sourceFaceIds = {"F" + std::to_string(static_cast<int>(x))};
    result.stageWallFaceIds = result.sourceFaceIds;
    return result;
}

oap::toolpath::ToolpathToolContext drill()
{
    oap::toolpath::ToolpathToolContext result;
    result.valid = true;
    result.toolNumber = 7;
    result.toolId = "drill-4";
    result.toolName = "D4 drill";
    result.toolType = "drill";
    result.diameter = 4.0;
    result.cuttingEdgeLength = 20.0;
    result.overallLength = 30.0;
    result.verticalFeed = 100.0;
    result.horizontalFeed = 200.0;
    result.spindleSpeed = 3000.0;
    result.tipAngle = 118.0;
    return result;
}

oap::toolpath::ToolpathParams parameters()
{
    oap::toolpath::ToolpathParams result;
    result.tool.diameter = 4.0;
    result.tool.cuttingEdgeLength = 20.0;
    result.tool.overallLength = 30.0;
    result.tool.vertFeed = 100.0;
    result.tool.horizFeed = 200.0;
    result.tool.spindleSpeed = 3000.0;
    result.depth.safeHeight = 2.0;
    result.depth.clearanceHeight = 4.0;
    result.hole.useCenterSpot = false;
    result.hole.drillTipAngleDeg = 118.0;
    result.hole.drillCycleMode =
        oap::toolpath::HoleDrillCycleMode::Direct;
    return result;
}

oap::tooling::HoleToolSelectionPlan plan(
    const std::vector<oap::toolpath::HoleMachiningInput>& holes)
{
    oap::tooling::HoleToolSelectionPlan result;
    oap::tooling::HoleToolBatch batch;
    batch.setupFace = oap::MoldSetupFace::PositiveZ;
    batch.setupIndex = 0;
    batch.phase = oap::toolpath::HoleMachiningPhase::Drill;
    batch.tool = drill();
    batch.visitOrderMode = oap::toolpath::HoleVisitOrderMode::PreserveInput;
    for (const std::size_t sourceIndex : {std::size_t{1}, std::size_t{0}}) {
        oap::tooling::HoleToolTask task;
        task.hole = holes[sourceIndex];
        task.sourceHoleIndex = sourceIndex;
        task.coveredSourceHoleIndices = {sourceIndex};
        task.physicalHoleIndex = sourceIndex;
        task.strategy = oap::toolpath::HoleMachiningStrategy::SmallDrill;
        batch.tasks.push_back(std::move(task));
    }
    result.batches.push_back(std::move(batch));
    result.physicalHoleCount = 2;
    result.setupCount = 1;
    return result;
}

bool preservesIdentityAcrossJobRoundTrip()
{
    const std::vector<oap::toolpath::HoleMachiningInput> holes{hole(8.0), hole(24.0)};
    const std::vector<oap::toolpath::HoleMachiningInput> inputs = holes;
    const oap::toolpath::ToolpathParams params = parameters();
    oap::tooling::HoleAutomaticToolpathResult automatic =
        oap::tooling::HoleAutomaticToolpath().generateFromPlan(
            plan(holes), inputs, params);
    if (!expect(!automatic.batch.cancelled &&
                    automatic.batch.machining.operations.size() == 2,
                "the automatic identity fixture should generate two operations")) {
        return false;
    }
    const auto& first = automatic.batch.machining.operations[0];
    const auto& second = automatic.batch.machining.operations[1];
    if (!expect(first.operationId == "HOLE_S1_drill_T7_1" &&
                    second.operationId == "HOLE_S1_drill_T7_2" &&
                    first.sourceHoleIndex == 1 &&
                    second.sourceHoleIndex == 0 &&
                    first.coveredSourceHoleIndices == std::vector<int>({1}) &&
                    second.coveredSourceHoleIndices == std::vector<int>({0}),
                "automatic planning must preserve batch order and source mapping")) {
        return false;
    }

    oap::manufacturing::HoleJobCaptureRequest request;
    request.definition.id = "identity-job";
    request.definition.name = "identity job";
    request.definition.part.sourcePath = "programmatic.step";
    request.definition.part.sourceFingerprint = "programmatic-part-v1";
    request.definition.activeToolLibraryId = "test-library";
    request.inputs = &inputs;
    request.automaticSelection = &automatic.selection;
    request.generated = &automatic.batch;
    oap::manufacturing::MachiningJob job =
        oap::manufacturing::HoleMachiningJobService().capture(request);
    const bool capturedIdentity =
        job.definition().holeOperations.size() == 1 &&
                    job.definition().holeOperations[0].id ==
                        "hole-operation-d3729b11e71487d8" &&
                    job.definition().holeOperations[0].targets.size() == 2 &&
                    job.definition().holeOperations[0].targets[0]
                            .sourceHoleIndex == 1 &&
                    job.definition().holeOperations[0].targets[1]
                            .sourceHoleIndex == 0;
    if (!expect(capturedIdentity,
                "Job capture must preserve stable batch and target identity")) {
        std::cerr << "captured operations="
                  << job.definition().holeOperations.size();
        if (!job.definition().holeOperations.empty()) {
            const auto& operation = job.definition().holeOperations.front();
            std::cerr << " id=" << operation.id
                      << " targets=" << operation.targets.size();
            for (const auto& target : operation.targets)
                std::cerr << " source=" << target.sourceHoleIndex;
        }
        std::cerr << '\n';
        return false;
    }

    const oap::toolpath::HoleToolpathBatch* captured = job.holeResult();
    if (!expect(
            captured && captured->machining.operations.size() == 2 &&
                captured->machining.operations[0].operationId ==
                    "hole-operation-d3729b11e71487d8-1" &&
                captured->machining.operations[1].operationId ==
                    "hole-operation-d3729b11e71487d8-2",
            "Job capture must bind runtime operations to the stable definition ID")) {
        return false;
    }

    oap::manufacturing::ipw::IpwExecutionOptions ipwOptions;
    ipwOptions.hole.zMapResolution = 1.0;
    ipwOptions.hole.zMapMaxCellCount = 100000;
    const auto capturedIpw =
        oap::manufacturing::ipw::IpwExecutor().execute(
            job,
            BRepPrimAPI_MakeBox(36.0, 20.0, 12.0).Shape(),
            "captured-hole-stock-v1",
            oap::manufacturing::ipw::kHybridIpwEngineId,
            ipwOptions);
    if (!expect(
            capturedIpw.complete() &&
                capturedIpw.executedNodeCount == 1 &&
                capturedIpw.executedOperationIds ==
                    std::vector<std::string>({
                        "hole-operation-d3729b11e71487d8"}),
            "a freshly captured hole program must execute in unified IPW without regeneration")) {
        std::cerr << "captured IPW error: " << capturedIpw.message << '\n';
        return false;
    }

    oap::toolpath::ToolpathParams flatDrillParams = parameters();
    flatDrillParams.hole.drillTipAngleDeg = 0.0;
    oap::tooling::HoleToolSelectionPlan flatDrillPlan = plan(holes);
    flatDrillPlan.batches.front().tool.tipAngle = 0.0;
    oap::tooling::HoleAutomaticToolpathResult flatDrill =
        oap::tooling::HoleAutomaticToolpath().generateFromPlan(
            std::move(flatDrillPlan), inputs, flatDrillParams);
    oap::manufacturing::HoleJobCaptureRequest flatDrillRequest = request;
    flatDrillRequest.definition.id = "flat-drill-identity-job";
    flatDrillRequest.automaticSelection = &flatDrill.selection;
    flatDrillRequest.generated = &flatDrill.batch;
    oap::manufacturing::MachiningJob flatDrillJob =
        oap::manufacturing::HoleMachiningJobService().capture(
            flatDrillRequest);
    const auto flatDrillIpw =
        oap::manufacturing::ipw::IpwExecutor().execute(
            flatDrillJob,
            BRepPrimAPI_MakeBox(36.0, 20.0, 12.0).Shape(),
            "flat-drill-stock-v1",
            oap::manufacturing::ipw::kHybridIpwEngineId,
            ipwOptions);
    if (!expect(
            flatDrillIpw.complete() &&
                flatDrillIpw.executedNodeCount == 1,
            "unified IPW must treat an unspecified drill-tip angle as the same flat cutter used by hole simulation")) {
        std::cerr << "flat drill IPW error: "
                  << flatDrillIpw.message << '\n';
        return false;
    }

    const std::vector<std::string> validationErrors = job.validationErrors();
    if (!validationErrors.empty()) {
        std::cerr << "Job validation errors before regenerate:\n";
        for (const std::string& error : validationErrors)
            std::cerr << "  " << error << '\n';
    }
    const oap::toolpath::HoleToolpathBatch rebuilt =
        oap::manufacturing::HoleMachiningJobService().regenerate(job);
    if (!expect(!rebuilt.cancelled &&
                    rebuilt.machining.operations.size() == 2,
                "Job regeneration should rebuild both operations")) {
        std::cerr << "rebuilt cancelled=" << rebuilt.cancelled
                  << " operations=" << rebuilt.machining.operations.size()
                  << " validation errors=" << rebuilt.validation.errorCount
                  << " warnings=" << rebuilt.validation.warningCount
                  << " diagnostics=" << rebuilt.diagnostics.size() << '\n';
        for (const auto& diagnostic : rebuilt.diagnostics) {
            std::cerr << "diagnostic " << diagnostic.operationId
                      << ": " << diagnostic.message << '\n';
        }
        return false;
    }
    if (!expect(
        rebuilt.machining.operations[0].operationId ==
                "hole-operation-d3729b11e71487d8-1" &&
            rebuilt.machining.operations[1].operationId ==
                "hole-operation-d3729b11e71487d8-2" &&
            rebuilt.machining.operations[0].sourceHoleIndex == 1 &&
            rebuilt.machining.operations[1].sourceHoleIndex == 0 &&
            rebuilt.machining.operations[0].coveredSourceHoleIndices ==
                std::vector<int>({1}) &&
            rebuilt.machining.operations[1].coveredSourceHoleIndices ==
                std::vector<int>({0}),
        "Job regeneration must preserve target order, stable IDs and covered sources")) {
        return false;
    }

    job.definition().holeOperations[0].targets[0].hole.entryResolution =
        oap::HoleEntryResolution::Unresolved;
    job.definition().holeOperations[0].targets[0].hole.mappingState =
        oap::toolpath::HoleMachiningMappingState::UnresolvedEntry;
    const oap::toolpath::HoleToolpathBatch rejected =
        oap::manufacturing::HoleMachiningJobService().regenerate(job);
    const bool hasUnresolvedDiagnostic = std::any_of(
        rejected.diagnostics.begin(), rejected.diagnostics.end(),
        [](const oap::toolpath::ToolpathDiagnostic& diagnostic) {
            return diagnostic.code ==
                       oap::toolpath::ToolpathDiagnosticCode::
                           HoleOpeningUnresolved &&
                   diagnostic.sourceIndex == 1;
        });
    return expect(
        rejected.machining.operations.empty() &&
            hasUnresolvedDiagnostic &&
            job.definition().holeOperations[0].state ==
                oap::manufacturing::JobOperationState::Error,
        "Job rebuild must reject unresolved targets through the shared mapping gate");
}

} // namespace

int main()
{
    if (!preservesIdentityAcrossJobRoundTrip())
        return 1;
    std::cout << "hole batch identity smoke tests passed\n";
    return 0;
}

// Real OpenCAMLib paths join an existing roughing IPW chain and formal export.
#include "manufacturing/finishing/SurfaceFinishingMachiningJobService.h"
#include "manufacturing/ipw/IpwExecutor.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "manufacturing/program/MachiningProgramAssembler.h"
#include "manufacturing/program/MachiningProgramPartitioner.h"
#include "toolpath/core/ToolpathPrimitiveMapping.h"
#include "toolpath/core/simulation/DexelToolProfile.h"

#include <BRepPrimAPI_MakeBox.hxx>

#include <iostream>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {
namespace m = oap::manufacturing;
namespace t = oap::toolpath;

void require(bool condition, const std::string& message)
{
    if (!condition)
        throw std::runtime_error(message);
}

t::ToolpathToolContext tool(int number, double diameter, double corner)
{
    t::ToolpathToolContext value;
    value.valid = true;
    value.toolNumber = number;
    value.toolId = "tool-" + std::to_string(number);
    value.toolName = value.toolId;
    value.toolType = corner == diameter / 2 ? "BallEndMill" : corner == 0 ? "EndMill" : "BullNose";
    value.diameter = diameter;
    value.cornerRadius = corner;
    value.cuttingEdgeLength = 12;
    value.overallLength = 25;
    value.spindleSpeed = 6000;
    value.horizontalFeed = 600;
    value.verticalFeed = 150;
    return value;
}

m::SetupDefinition setup(const m::PartDefinition& part, int index, const gp_Ax3& frame)
{
    m::SetupDefinition value;
    value.id = "setup-" + std::to_string(index);
    value.setupIndex = index;
    value.face = index == 0 ? oap::MoldSetupFace::PositiveZ : oap::MoldSetupFace::PositiveX;
    value.workOffset = index == 0 ? t::WorkCoordinateSystem::G54 : t::WorkCoordinateSystem::G55;
    value.origin = m::toJobVector(frame.Location());
    value.xAxis = m::toJobVector(frame.XDirection());
    value.yAxis = m::toJobVector(frame.YDirection());
    value.zAxis = m::toJobVector(frame.Direction());
    value.valid = true;
    value.confirmation.confirmed = true;
    value.confirmation.confirmedInputFingerprint =
        m::MachiningJobStorage::setupInputFingerprint(part, value).toStdString();
    return value;
}

void installFinishing(m::MachiningJob& job, const TopoDS_Shape& target)
{
    t::SurfaceFinishingBatch batch;
    for (const auto& definition : job.definition().finishingOperations) {
        if (!definition.enabled)
            continue;
        const auto* frame = job.setupById(definition.setupId);
        t::SurfaceFinishingOperation operation;
        operation.operationId = definition.id;
        operation.setupIndex = frame->setupIndex;
        operation.setupFace = frame->face;
        operation.workFrame = gp_Ax3(m::toPoint(frame->origin),
            gp_Dir(frame->zAxis.x, frame->zAxis.y, frame->zAxis.z),
            gp_Dir(frame->xAxis.x, frame->xAxis.y, frame->xAxis.z));
        operation.tool = job.toolControllerById(definition.toolControllerId)->tool;
        operation.result = t::SurfaceFinishing{}.generate(target, operation.workFrame, definition.parameters);
        require(operation.result.ok, operation.result.error);
        batch.effectiveParams = operation.result.toolpathParams();
        batch.operations.push_back(std::move(operation));
    }
    job.setFinishingResult(std::move(batch),
        m::MachiningJobStorage::finishingExecutionFingerprint(job.definition()).toStdString());
}

m::MachiningJob fixture(const TopoDS_Shape& target)
{
    m::MachiningJob job;
    auto& def = job.definition();
    def.id = "finishing-ipw-fixture";
    def.name = def.id;
    def.part.sourcePath = "fixture.step";
    def.part.sourceFingerprint = "fixture-part-v1";
    def.setups.push_back(setup(def.part, 0, gp_Ax3()));
    def.setups.push_back(setup(def.part, 1,
        gp_Ax3(gp_Pnt(), gp_Dir(1, 0, 0), gp_Dir(0, 1, 0))));
    def.toolControllers.push_back({"rough-tool", "fixture", tool(1, 6, 0)});
    def.toolControllers.push_back({"ball-tool", "fixture", tool(2, 2, 1)});
    def.toolControllers.push_back({"bull-tool", "fixture", tool(3, 4, 0.5)});
    m::PocketOperationDefinition rough;
    rough.id = "rough";
    rough.setupId = def.setups[0].id;
    rough.toolControllerId = "rough-tool";
    rough.algorithmId = "builtin.pocket.freecad";
    rough.target.sourceFaceIds = {"fixture-top"};
    rough.target.floorFaceId = "fixture-top";
    rough.target.geometryFingerprint = "rough-target-v1";
    rough.state = m::JobOperationState::Generated;
    rough.inputFingerprint = "rough-input-v1";
    def.pocketOperations.push_back(rough);
    for (int i = 0; i < 2; ++i) {
        m::SurfaceFinishingOperationDefinition finish;
        finish.id = i == 0 ? "finish-top" : "finish-side";
        finish.setupId = def.setups[i].id;
        finish.toolControllerId = i == 0 ? "ball-tool" : "bull-tool";
        finish.target.unitIds = {"unit-1"};
        finish.target.geometryFingerprint = "target-v1";
        finish.parameters.cutterKind = i == 0 ? t::FinishingCutterKind::Ball : t::FinishingCutterKind::Bull;
        finish.parameters.diameter = i == 0 ? 2 : 4;
        finish.parameters.cornerRadius = i == 0 ? 1 : 0.5;
        finish.parameters.cuttingLength = 12;
        finish.parameters.stepOver = 1;
        finish.parameters.sampleSpacing = 1;
        finish.sequence = i + 1;
        finish.state = m::JobOperationState::Warning;
        finish.inputFingerprint = m::SurfaceFinishingMachiningJobService::operationFingerprint(def, finish);
        def.finishingOperations.push_back(finish);
    }
    t::PocketToolpathBatch roughBatch;
    roughBatch.effectiveParams.tool.diameter = 6;
    roughBatch.effectiveParams.tool.cuttingEdgeLength = 12;
    t::PocketToolpathOperation roughRuntime;
    roughRuntime.operationId = rough.id;
    roughRuntime.algorithmId = rough.algorithmId;
    roughRuntime.setupFace = oap::MoldSetupFace::PositiveZ;
    roughRuntime.outwardToolAxis = gp_Vec(0, 0, 1);
    roughRuntime.tool = def.toolControllers[0].tool;
    roughRuntime.clearing.ok = true;
    t::ToolpathMove move;
    move.type = t::ToolpathMoveType::Cut;
    move.from = gp_Pnt(1, 5, 4);
    move.to = gp_Pnt(11, 5, 4);
    move.feed = 600;
    roughRuntime.clearing.moves.push_back(move);
    roughRuntime.clearing.primitives.push_back(t::makeLinearPrimitive(move));
    roughBatch.operations.push_back(std::move(roughRuntime));
    job.setPocketResult(std::move(roughBatch),
        m::MachiningJobStorage::pocketExecutionFingerprint(def).toStdString());
    installFinishing(job, target);
    const auto errors = job.validationErrors();
    require(errors.empty(), errors.empty() ? "" : errors.front());
    return job;
}

void verifyChain()
{
    const auto target = BRepPrimAPI_MakeBox(12, 10, 2).Shape();
    const auto stock = BRepPrimAPI_MakeBox(15, 10, 8).Shape();
    auto job = fixture(target);
    m::ipw::IpwExecutionOptions options;
    options.pocket.method = t::StockSimulationMethod::HeightField;
    options.pocket.runParallel = false;
    options.pocket.zMapResolution = 0.5;
    options.pocket.zMapMaxCellCount = 10000;
    const auto run = [&] {
        return m::ipw::IpwExecutor{}.execute(job, stock, "stock-v1",
            m::ipw::kHybridIpwEngineId, options);
    };
    const auto first = run();
    require(first.complete(), first.message);
    require(first.executedOperationIds == std::vector<std::string>({"rough", "finish-top", "finish-side"}),
        "formal global order was not preserved in IPW");
    const auto& nodes = job.ipwSession().nodes();
    require(nodes.size() == 3 && nodes[1].operation.kind == m::ipw::IpwOperationKind::SurfaceFinishing,
        "finishing lost its own IPW kind");
    const auto root = job.runtime().ipwCheckpoints.find(job.ipwSession().initialStateKey());
    const auto rough = job.runtime().ipwCheckpoints.find(nodes[0].stateKey);
    const auto top = job.runtime().ipwCheckpoints.find(nodes[1].stateKey);
    const auto side = job.runtime().ipwCheckpoints.find(nodes[2].stateKey);
    require(root && rough && top && side, "IPW checkpoint chain is incomplete");
    require(rough->volume < root->volume && top->volume < rough->volume && side->volume < top->volume,
        "rough/top-finishing/oriented-finishing did not progressively remove stock");
    require(top->backendId.find("finishing") != std::string::npos &&
        side->backendId == top->backendId && side->dexelStock->axisAligned(gp_Vec(0, 0, 1)),
        "non-Z finishing failed to preserve the parent's native material grid");
    require(top->operationResult->toolProfileBandCount > 1 &&
        side->operationResult->toolProfileBandCount > 1,
        "rounded cutters were flattened into cylinders");
    const auto assembly = m::MachiningProgramAssembler{}.assemble(job);
    require(assembly.ok(), assembly.issues.empty() ? "assembly failed" : assembly.issues.front().message);
    require(assembly.program.operations.size() == 3 &&
        assembly.program.operations[1].type == t::ToolpathOperationType::SurfaceFinishing,
        "formal program lost the finishing operation");
    const auto partition = m::MachiningProgramPartitioner{}.partition(job);
    require(partition.ok(), partition.issues.empty() ? "partition failed" : partition.issues.front().message);
    require(partition.partitions.size() == 3, "tool/setup partitions were merged incorrectly");
    const auto second = run();
    require(second.complete() && second.reusedCheckpointCount == 3 && second.executedNodeCount == 0,
        "identical finishing chain did not reuse its complete checkpoint prefix");

    // A changed input cannot reuse Ready geometry, even before regeneration.
    const auto oldTopKey = nodes[1].stateKey;
    auto& changed = job.definition().finishingOperations.front();
    changed.parameters.allowance = 0.4;
    require(!run().complete(), "stale finishing result bypassed the IPW fingerprint gate");
    require(!m::MachiningProgramAssembler{}.assemble(job).ok(), "stale finishing was exported");
    changed.inputFingerprint = m::SurfaceFinishingMachiningJobService::operationFingerprint(job.definition(), changed);
    installFinishing(job, target);
    const auto third = run();
    require(third.complete() && third.reusedCheckpointCount == 1 && third.executedNodeCount == 2,
        "finishing edit did not preserve only the roughing prefix");
    require(job.ipwSession().nodes()[1].stateKey != oldTopKey,
        "finishing parameter edit reused its old state key");

    job.runtime().ipwSession.clear();
    job.runtime().ipwCheckpoints.clear();
    bool cancelled = false;
    t::ToolpathGenerationControl control;
    control.isCancellationRequested = [&] { return cancelled; };
    control.reportProgress = [&](std::size_t done, std::size_t) { cancelled = done >= 1; };
    const auto interrupted = m::ipw::IpwExecutor{}.execute(job, stock, "stock-v1",
        m::ipw::kHybridIpwEngineId, options, &control);
    require(interrupted.status == m::ipw::IpwExecutionStatus::Cancelled &&
        interrupted.executedNodeCount == 1 && !job.ipwSession().complete(),
        "cancelled mixed chain published finishing results");
}

void verifyStockRoughingAndFinishing()
{
    const auto target = BRepPrimAPI_MakeBox(8, 6, 1).Shape();
    const auto stock = BRepPrimAPI_MakeBox(8, 6, 3).Shape();
    m::MachiningJob job;
    auto& def = job.definition();
    def.id = "layered-engraving-ipw";
    def.name = def.id;
    def.part.sourcePath = "layered-target.step";
    def.part.sourceFingerprint = "layered-target-v1";
    def.setups.push_back(setup(def.part, 0, gp_Ax3()));
    def.toolControllers.push_back({"ball-tool", "fixture", tool(2, 2, 1)});
    m::SurfaceFinishingOperationDefinition definition;
    definition.id = "stock-to-finished-part";
    definition.setupId = def.setups.front().id;
    definition.toolControllerId = "ball-tool";
    definition.algorithmId = t::kOpenCamLibStockEngravingAlgorithmId;
    definition.target.unitIds = {"unit-1"};
    definition.target.geometryFingerprint = "layered-target-v1";
    definition.parameters.mode = t::EngravingMachiningMode::StockRoughingAndFinishing;
    definition.parameters.diameter = 2;
    definition.parameters.cornerRadius = 1;
    definition.parameters.cuttingLength = 12;
    definition.parameters.stepOver = 1;
    definition.parameters.sampleSpacing = 0.5;
    definition.parameters.maxStepDown = 0.75;
    definition.parameters.roughingAllowance = 0.2;
    definition.state = m::JobOperationState::Warning;
    definition.inputFingerprint = m::SurfaceFinishingMachiningJobService::operationFingerprint(def, definition);
    def.finishingOperations.push_back(definition);
    t::SurfaceFinishingOperation operation;
    operation.operationId = definition.id;
    operation.setupFace = oap::MoldSetupFace::PositiveZ;
    operation.setupIndex = 0;
    operation.workFrame = gp_Ax3();
    operation.tool = def.toolControllers.front().tool;
    operation.result = t::SurfaceFinishing{}.generate(target, operation.workFrame,
        definition.parameters, nullptr, &stock);
    require(operation.result.ok && operation.result.metrics.roughingLayerCount > 1 &&
        operation.result.metrics.roughingMoveCount > 0 &&
        operation.result.metrics.roughingMoveCount < operation.result.moves.size(),
        "stock-to-finish generation did not contain both phases: " + operation.result.error);
    for (const auto& move : operation.result.moves) {
        require(move.from.Z() >= 1.0 - 1e-6 && move.to.Z() >= 1.0 - 1e-6,
            "layered path crossed the protected flat target top");
        if (move.type == t::ToolpathMoveType::Rapid)
            require(move.from.Z() >= 3.0 && move.to.Z() >= 3.0,
                "layered path traversed inside the initial stock");
    }
    t::SurfaceFinishingBatch batch;
    batch.effectiveParams = operation.result.toolpathParams();
    batch.operations.push_back(std::move(operation));
    job.setFinishingResult(std::move(batch), m::MachiningJobStorage::finishingExecutionFingerprint(def).toStdString());
    require(job.validationErrors().empty(), "layered formal Job is invalid");
    const auto assembled = m::MachiningProgramAssembler{}.assemble(job);
    require(assembled.ok() && assembled.program.operations.size() == 1 &&
        assembled.program.operations.front().moves.size() == job.currentFinishingResult()->operations.front().result.moves.size(),
        "formal export did not retain the complete layered program");
    m::ipw::IpwExecutionOptions options;
    options.pocket.method = t::StockSimulationMethod::HeightField;
    options.pocket.runParallel = false;
    options.pocket.zMapResolution = 0.25;
    options.pocket.zMapMaxCellCount = 10000;
    const auto run = [&] {
        return m::ipw::IpwExecutor{}.execute(job, stock, "layered-stock-v1",
            m::ipw::kHybridIpwEngineId, options);
    };
    const auto first = run();
    require(first.complete() && first.executedNodeCount == 1 && first.finalCheckpoint &&
        first.finalCheckpoint->operationResult && first.finalCheckpoint->operationResult->complete,
        "layered path did not finish native IPW: " + first.message);
    const auto& removal = *first.finalCheckpoint->operationResult;
    require(removal.removedVolume > 0 && first.finalCheckpoint->volume > 0 &&
        first.finalCheckpoint->volume < 8.0 * 6.0 * 3.0 &&
        first.finalCheckpoint->volume >= 8.0 * 6.0 * 1.0 - 1.0,
        "layered IPW removed no stock or crossed the protected target-volume bound");
    require(std::abs(removal.initialVolume - removal.finalVolume - removal.removedVolume) < 1e-6 &&
        std::abs(removal.finalVolume - first.finalCheckpoint->volume) < 1e-6,
        "layered checkpoint volume disagrees with its removal result");
    const auto repeated = run();
    require(repeated.complete() && repeated.executedNodeCount == 0 && repeated.reusedCheckpointCount == 1 &&
        repeated.finalCheckpoint && repeated.finalCheckpoint->contentFingerprint == first.finalCheckpoint->contentFingerprint,
        "unchanged layered path did not reuse its completed stock checkpoint");
}
} // namespace

int main()
{
    try {
        verifyChain();
        verifyStockRoughingAndFinishing();
        std::cout << "Surface finishing mixed IPW, rounded cutters, oriented setup, export and invalidation passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}

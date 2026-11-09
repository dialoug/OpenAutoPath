// 加工项目快速回归测试：覆盖 IPW 状态链与 Job 的全局工序映射。
// 测试只构造内存对象，不依赖本地 STEP、刀库或图形环境。
#include "manufacturing/ipw/IpwSession.h"
#include "manufacturing/ipw/IpwExecutor.h"
#include "manufacturing/job/MachiningJob.h"
#include "manufacturing/job/MachiningJobApplicationService.h"
#include "manufacturing/job/MachiningOperationService.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "analysis/MoldCoordinateFrame.h"
#include "analysis/Model.h"
#include "analysis/StepIndexer.h"
#include "toolpath/core/ToolpathPrimitives.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include <algorithm>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

oap::manufacturing::ipw::IpwOperationDefinition operation(
    std::string id,
    int sequence,
    std::string fingerprint)
{
    oap::manufacturing::ipw::IpwOperationDefinition result;
    result.id = std::move(id);
    result.setupId = "setup-z-plus";
    result.toolControllerId = "tool-7";
    result.inputFingerprint = std::move(fingerprint);
    result.sequence = sequence;
    result.kind = oap::manufacturing::ipw::IpwOperationKind::Hole;
    return result;
}

oap::manufacturing::ipw::IpwArtifactDescriptor artifact(
    const oap::manufacturing::ipw::IpwNode& node,
    std::string contentFingerprint)
{
    oap::manufacturing::ipw::IpwArtifactDescriptor result;
    result.stateKey = node.stateKey;
    result.backendId = "multi-dexel-v1";
    result.contentFingerprint = std::move(contentFingerprint);
    result.checkpoint = true;
    return result;
}

bool ipwSessionInvalidatesOnlyChangedSuffix()
{
    namespace ipw = oap::manufacturing::ipw;
    ipw::IpwSession session;
    std::vector<ipw::IpwOperationDefinition> operations{
        operation("op-3", 2, "input-3"),
        operation("op-1", 0, "input-1"),
        operation("op-2", 1, "input-2"),
    };

    const ipw::IpwPlanUpdate initial =
        session.configure("stock-v1", "engine-v1", operations);
    if (!expect(initial.ok() && initial.dirtyNodeCount == 3,
                "initial plan should make all nodes dirty") ||
        !expect(session.nodes().size() == 3 &&
                    session.nodes()[0].operation.id == "op-1" &&
                    session.nodes()[1].operation.id == "op-2" &&
                    session.nodes()[2].operation.id == "op-3",
                "IPW nodes should follow the global sequence") ||
        !expect(session.firstRunnableNode() ==
                    std::optional<std::size_t>(0),
                "the first dirty node should be runnable") ||
        !expect(!session.beginNode(1) && session.beginNode(0),
                "a node must wait for its predecessor")) {
        return false;
    }

    ipw::IpwArtifactDescriptor stale =
        artifact(session.nodes()[0], "content-1");
    stale.stateKey = "stale-state-key";
    if (!expect(!session.completeNode(0, std::move(stale)),
                "a stale artifact should be rejected") ||
        !expect(session.completeNode(
                    0, artifact(session.nodes()[0], "content-1")),
                "a matching artifact should complete the first node") ||
        !expect(session.beginNode(1) &&
                    session.completeNode(
                        1, artifact(session.nodes()[1], "content-2")),
                "the second node should run after the first is ready")) {
        return false;
    }

    const std::size_t revisionBeforeReuse = session.revision();
    const ipw::IpwPlanUpdate unchanged =
        session.configure("stock-v1", "engine-v1", operations);
    if (!expect(unchanged.ok() && !unchanged.changed() &&
                    unchanged.reusedNodeCount == 3 &&
                    session.revision() == revisionBeforeReuse,
                "an unchanged plan should reuse every node") ||
        !expect(session.firstRunnableNode() ==
                    std::optional<std::size_t>(2),
                "only the dirty suffix should remain runnable")) {
        return false;
    }

    operations[2].inputFingerprint = "input-2-revised";
    const ipw::IpwPlanUpdate changed =
        session.configure("stock-v1", "engine-v1", operations);
    if (!expect(changed.ok() && changed.reusedNodeCount == 1 &&
                    changed.dirtyNodeCount == 2,
                "a changed operation should invalidate its suffix") ||
        !expect(session.nodes()[0].status == ipw::IpwNodeStatus::Ready &&
                    session.nodes()[1].status == ipw::IpwNodeStatus::Dirty &&
                    !session.nodes()[1].artifact.has_value(),
                "the unchanged prefix should retain its artifact")) {
        return false;
    }

    std::vector<ipw::IpwOperationDefinition> invalid = operations;
    invalid[1].sequence = invalid[0].sequence;
    const std::size_t revisionBeforeError = session.revision();
    const ipw::IpwPlanUpdate rejected =
        session.configure("other-stock", "other-engine", std::move(invalid));
    return expect(!rejected.ok() &&
                      session.initialStockFingerprint() == "stock-v1" &&
                      session.revision() == revisionBeforeError,
                  "an invalid plan should leave the session unchanged");
}

bool machiningJobBuildsSharedGlobalPlan()
{
    namespace manufacturing = oap::manufacturing;
    namespace ipw = oap::manufacturing::ipw;

    manufacturing::MachiningJob job;
    manufacturing::HoleOperationDefinition hole;
    hole.id = "hole-op";
    hole.setupId = "setup-z-plus";
    hole.toolControllerId = "drill-7";
    hole.inputFingerprint = "hole-input-v1";
    hole.sequence = 20;

    manufacturing::PocketOperationDefinition pocket;
    pocket.id = "pocket-op";
    pocket.setupId = "setup-z-plus";
    pocket.toolControllerId = "mill-3";
    pocket.inputFingerprint = "pocket-input-v1";
    pocket.sequence = 10;

    manufacturing::HoleOperationDefinition disabled = hole;
    disabled.id = "disabled-op";
    disabled.sequence = 30;
    disabled.enabled = false;

    manufacturing::ProfileOperationDefinition profile;
    profile.id = "profile-op";
    profile.setupId = "setup-z-plus";
    profile.toolControllerId = "mill-3";
    profile.inputFingerprint = "profile-input-v1";
    profile.sequence = 15;

    job.definition().holeOperations = {hole, disabled};
    job.definition().pocketOperations = {pocket};
    job.definition().profileOperations = {profile};

    const ipw::IpwPlanUpdate all =
        job.configureIpwPlan("stock-v1", "engine-v1");
    const auto& nodes = job.ipwSession().nodes();
    if (!expect(all.ok() && nodes.size() == 3,
                "the Job should map enabled operations into IPW") ||
        !expect(nodes[0].operation.id == "pocket-op" &&
                    nodes[0].operation.kind == ipw::IpwOperationKind::Pocket &&
                    nodes[1].operation.id == "profile-op" &&
                    nodes[1].operation.kind == ipw::IpwOperationKind::Profile &&
                    nodes[2].operation.id == "hole-op" &&
                    nodes[2].operation.kind == ipw::IpwOperationKind::Hole,
                "hole, pocket and profile operations should share one global order")) {
        return false;
    }

    const ipw::IpwPlanUpdate selected = job.configureIpwPlan(
        "stock-v1", "engine-v1", {"hole-op"});
    if (!expect(selected.ok() && job.ipwSession().nodes().size() == 1 &&
                    job.ipwSession().nodes()[0].operation.id == "hole-op",
                "an explicit operation selection should filter the plan")) {
        return false;
    }

    const std::string retainedOperationId =
        job.ipwSession().nodes()[0].operation.id;
    const ipw::IpwPlanUpdate missing = job.configureIpwPlan(
        "stock-v2", "engine-v2", {"missing-op"});
    if (!expect(!missing.ok() &&
                    job.ipwSession().nodes().size() == 1 &&
                    job.ipwSession().nodes()[0].operation.id ==
                        retainedOperationId,
                "a missing selection should not mutate the current plan")) {
        return false;
    }

    job.invalidateRuntime();
    return expect(!job.ipwSession().configured() &&
                      job.ipwSession().nodes().empty() &&
                      job.definition().holeOperations.size() == 2,
                  "runtime invalidation should preserve the Job definition");
}

bool profileExecutorPublishesMaterialCheckpoint()
{
    namespace manufacturing = oap::manufacturing;
    namespace ipw = oap::manufacturing::ipw;

    manufacturing::MachiningJob job;
    manufacturing::ProfileOperationDefinition definition;
    definition.id = "profile-cut";
    definition.setupId = "setup-z";
    definition.toolControllerId = "mill-2";
    definition.sequence = 0;
    definition.inputFingerprint = "profile-cut-v1";
    definition.state = manufacturing::JobOperationState::Generated;
    job.definition().profileOperations.push_back(definition);

    oap::toolpath::ProfileToolpathBatch batch;
    batch.effectiveParams.tool.diameter = 2.0;
    batch.effectiveParams.tool.cuttingEdgeLength = 8.0;
    oap::toolpath::ProfileToolpathOperation operation;
    operation.operationId = definition.id;
    operation.outwardToolAxis = gp_Vec(0.0, 0.0, 1.0);
    operation.contour.ok = true;
    oap::toolpath::ToolpathMove cut;
    cut.type = oap::toolpath::ToolpathMoveType::Cut;
    cut.from = gp_Pnt(2.0, 2.0, 4.0);
    cut.to = gp_Pnt(18.0, 2.0, 4.0);
    operation.contour.moves.push_back(cut);
    operation.contour.primitives.push_back(
        oap::toolpath::makeLinearPrimitive(cut));
    batch.operations.push_back(std::move(operation));
    job.setProfileResult(std::move(batch), "profile-runtime-v1");

    ipw::IpwExecutionOptions options;
    options.pocket.method =
        oap::toolpath::StockSimulationMethod::HeightField;
    options.pocket.runParallel = false;
    options.pocket.zMapResolution = 0.5;
    options.pocket.zMapMaxCellCount = 100000;
    options.safeMixedTopologyFallback = false;
    options.includedOperationIds = {definition.id};
    const TopoDS_Shape stock =
        BRepPrimAPI_MakeBox(20.0, 20.0, 10.0).Shape();
    const auto result = ipw::IpwExecutor().execute(
        job, stock, "profile-stock-v1",
        ipw::kHybridIpwEngineId, options);
    if (!expect(result.complete() && result.executedNodeCount == 1 &&
                    result.finalCheckpoint &&
                    result.finalCheckpoint->dexelStock &&
                    result.finalCheckpoint->dexelStock->volume() < 4000.0,
                "profile IPW should publish a material-removal checkpoint")) {
        std::cerr << "profile IPW error=" << result.message << '\n';
        return false;
    }
    const auto& nodes = job.ipwSession().nodes();
    return expect(nodes.size() == 1 &&
                      nodes.front().operation.kind ==
                          ipw::IpwOperationKind::Profile &&
                      nodes.front().artifact &&
                      nodes.front().artifact->backendId ==
                          "tiled-multi-dexel-profile-v1-semantic-primitives",
                  "profile IPW should retain distinct operation and backend semantics");
}

oap::toolpath::HoleMachiningOperation runtimeDrill(
    std::string id,
    const gp_Pnt& entry,
    const gp_Pnt& bottom)
{
    oap::toolpath::HoleMachiningOperation result;
    result.operationId = std::move(id);
    result.kind = oap::FeatureKind::BlindHole;
    result.strategy =
        oap::toolpath::HoleMachiningStrategy::SmallDrill;
    result.phase = oap::toolpath::HoleMachiningPhase::Drill;
    result.entry = entry;
    result.bottom = bottom;
    result.commandedBottom = bottom;
    result.hasCommandedBottom = true;
    result.radius = 2.0;
    result.minimumCuttingEdgeLength = entry.Distance(bottom);
    result.minimumToolReach = result.minimumCuttingEdgeLength;
    result.tool.valid = true;
    result.tool.toolType = "drill";
    result.tool.diameter = 4.0;
    result.tool.cuttingEdgeLength = 12.0;
    result.tool.overallLength = 20.0;
    result.tool.tipAngle = 118.0;
    oap::toolpath::ToolpathMove move;
    move.type = oap::toolpath::ToolpathMoveType::Cut;
    move.from = entry;
    move.to = bottom;
    result.moves.push_back(move);
    return result;
}

bool multiSetupExecutorKeepsOneNativeCheckpointGrid()
{
    namespace manufacturing = oap::manufacturing;
    namespace ipw = oap::manufacturing::ipw;

    manufacturing::MachiningJob job;
    const auto addDefinition = [&](std::string id,
                                   std::string setupId,
                                   int sequence,
                                   std::string fingerprint) {
        manufacturing::HoleOperationDefinition definition;
        definition.id = std::move(id);
        definition.setupId = std::move(setupId);
        definition.toolControllerId = "drill-4";
        definition.sequence = sequence;
        definition.inputFingerprint = std::move(fingerprint);
        definition.state = manufacturing::JobOperationState::Generated;
        job.definition().holeOperations.push_back(std::move(definition));
    };
    addDefinition("setup-a-hole", "setup-z", 10, "input-a-v1");
    addDefinition("setup-b-hole", "setup-x", 20, "input-b-v1");
    addDefinition("setup-c-hole", "setup-z", 30, "input-c-v1");

    oap::toolpath::HoleToolpathBatch batch;
    batch.effectiveParams.tool.diameter = 4.0;
    batch.effectiveParams.tool.cuttingEdgeLength = 12.0;
    batch.effectiveParams.hole.drillTipAngleDeg = 118.0;
    batch.machining.operations = {
        runtimeDrill(
            "setup-a-hole",
            {5.0, 5.0, 20.0}, {5.0, 5.0, 4.0}),
        runtimeDrill(
            "setup-b-hole",
            {20.0, 5.0, 10.0}, {4.0, 5.0, 10.0}),
        runtimeDrill(
            "setup-c-hole",
            {15.0, 15.0, 20.0}, {15.0, 15.0, 4.0})};
    job.setHoleResult(std::move(batch), "hole-runtime-v1");

    ipw::IpwExecutionOptions executionOptions;
    executionOptions.hole.method =
        oap::toolpath::StockSimulationMethod::HeightField;
    executionOptions.hole.runParallel = true;
    executionOptions.hole.zMapResolution = 0.5;
    executionOptions.hole.zMapMaxCellCount = 100000;
    executionOptions.pocket.zMapResolution = 0.5;
    executionOptions.pocket.zMapMaxCellCount = 100000;

    const TopoDS_Shape stock =
        BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape();
    const auto initial = ipw::IpwExecutor().execute(
        job, stock, "multi-setup-stock-v1",
        ipw::kHybridIpwEngineId,
        executionOptions);
    if (!expect(initial.complete() &&
                    initial.executedNodeCount == 3 &&
                    initial.reusedCheckpointCount == 0 &&
                    initial.nativeHoleNodeCount == 3 &&
                    initial.finalCheckpoint &&
                    initial.finalCheckpoint->dexelStock &&
                    initial.finalCheckpoint->dexelStock->axisAligned(
                        gp_Vec(0.0, 0.0, 1.0)) &&
                    initial.finalCheckpoint->dexelStock->intervalCount() >
                        initial.finalCheckpoint->dexelStock->activeCellCount(),
                "A-B-A setup axes should execute on one native checkpoint grid")) {
        std::cerr << "multi-setup initial error=" << initial.message << '\n';
        return false;
    }
    for (const auto& node : job.ipwSession().nodes()) {
        const auto checkpoint =
            job.runtime().ipwCheckpoints.find(node.stateKey);
        if (!expect(checkpoint &&
                        checkpoint->backendId ==
                            "tiled-multi-dexel-hole-v2-oriented-profile",
                    "every multi-setup hole checkpoint should use the native backend")) {
            return false;
        }
    }
    const std::string firstFinalFingerprint =
        initial.finalCheckpoint->contentFingerprint;
    const std::string firstMaterialFingerprint =
        initial.finalCheckpoint->dexelStock->contentFingerprint();

    const auto unchanged = ipw::IpwExecutor().execute(
        job, stock, "multi-setup-stock-v1",
        ipw::kHybridIpwEngineId,
        executionOptions);
    if (!expect(unchanged.complete() &&
                    unchanged.reusedCheckpointCount == 3 &&
                    unchanged.executedNodeCount == 0 &&
                    unchanged.finalCheckpoint->contentFingerprint ==
                        firstFinalFingerprint,
                "unchanged multi-setup execution should reuse the complete prefix")) {
        return false;
    }

    job.definition().holeOperations[1].inputFingerprint =
        "input-b-v2";
    const auto changedMiddle = ipw::IpwExecutor().execute(
        job, stock, "multi-setup-stock-v1",
        ipw::kHybridIpwEngineId,
        executionOptions);
    if (!expect(changedMiddle.complete() &&
                    changedMiddle.reusedCheckpointCount == 1 &&
                    changedMiddle.executedNodeCount == 2 &&
                    changedMiddle.finalCheckpoint->dexelStock &&
                    changedMiddle.finalCheckpoint->dexelStock
                            ->contentFingerprint() ==
                        firstMaterialFingerprint,
                "changing setup B input should retain only setup A prefix")) {
        std::cerr << "changed-middle complete=" << changedMiddle.complete()
                  << " reused=" << changedMiddle.reusedCheckpointCount
                  << " executed=" << changedMiddle.executedNodeCount
                  << " message=" << changedMiddle.message << '\n';
        if (changedMiddle.finalCheckpoint) {
            std::cerr << "changed-middle material fingerprint="
                      << changedMiddle.finalCheckpoint->dexelStock
                             ->contentFingerprint()
                      << " expected=" << firstMaterialFingerprint << '\n';
        }
        return false;
    }

    job.definition().holeOperations[1].setupId =
        "setup-x-rebound";
    const auto changedSetup = ipw::IpwExecutor().execute(
        job, stock, "multi-setup-stock-v1",
        ipw::kHybridIpwEngineId,
        executionOptions);
    if (!expect(changedSetup.complete() &&
                    changedSetup.reusedCheckpointCount == 1 &&
                    changedSetup.executedNodeCount == 2 &&
                    changedSetup.finalCheckpoint->dexelStock &&
                    changedSetup.finalCheckpoint->dexelStock
                            ->contentFingerprint() ==
                        firstMaterialFingerprint,
                "changing setup B identity should invalidate B and its suffix")) {
        return false;
    }

    executionOptions.hole.method =
        oap::toolpath::StockSimulationMethod::FixedAxisLayeredBRep;
    const auto unsupportedBackend = ipw::IpwExecutor().execute(
        job, stock, "multi-setup-stock-v1",
        ipw::kHybridIpwEngineId,
        executionOptions);
    const bool noHoleStatePublished = std::all_of(
        job.ipwSession().nodes().begin(),
        job.ipwSession().nodes().end(),
        [](const ipw::IpwNode& node) {
            return !node.artifact.has_value();
        });
    return expect(!unsupportedBackend.complete() &&
                      unsupportedBackend.reusedCheckpointCount == 0 &&
                      unsupportedBackend.executedNodeCount == 0 &&
                      unsupportedBackend.nativeHoleNodeCount == 0 &&
                      noHoleStatePublished &&
                      unsupportedBackend.message.find(
                          "cannot publish a native Dexel state") !=
                          std::string::npos,
                  "unsupported hole backends must fail explicitly without publishing a fallback state");
}

bool checkpointStoreRejectsConflictingContent()
{
    namespace ipw = oap::manufacturing::ipw;

    oap::toolpath::StockSimulationOptions options;
    options.zMapResolution = 1.0;
    options.zMapMaxCellCount = 4096;
    options.runParallel = false;
    std::string buildError;
    const std::shared_ptr<const oap::toolpath::MultiDexelStock> stock =
        oap::toolpath::MultiDexelStock::fromShape(
            BRepPrimAPI_MakeBox(4.0, 4.0, 4.0).Shape(),
            gp_Vec(0.0, 0.0, 1.0),
            2.0,
            options,
            nullptr,
            &buildError);
    if (!expect(stock && stock->valid() && stock->volume() > 0.0,
                "the checkpoint fixture stock should be valid")) {
        if (!buildError.empty())
            std::cerr << "multi-dexel fixture: " << buildError << '\n';
        return false;
    }

    ipw::IpwCheckpointStore store;
    std::string error;
    ipw::IpwStockCheckpoint invalid;
    invalid.stateKey = "invalid";
    invalid.backendId = "multi-dexel-v1";
    invalid.contentFingerprint = "missing-stock";
    if (!expect(!store.publish(std::move(invalid), &error) &&
                    !error.empty() && store.size() == 0,
                "a checkpoint without material truth must be rejected")) {
        return false;
    }

    ipw::IpwStockCheckpoint root;
    root.stateKey = "root-state";
    root.backendId = "multi-dexel-v1";
    root.contentFingerprint = stock->contentFingerprint();
    root.dexelStock = stock;
    root.volume = stock->volume();
    if (!expect(store.publish(root, &error) && error.empty(),
                "a valid root checkpoint should publish")) {
        return false;
    }
    const auto publishedRoot = store.find(root.stateKey);
    if (!expect(publishedRoot && publishedRoot->dexelStock == stock &&
                    store.publish(root, &error) &&
                    store.find(root.stateKey) == publishedRoot,
                "publishing identical content should be idempotent")) {
        return false;
    }

    ipw::IpwStockCheckpoint conflict = root;
    conflict.contentFingerprint = "different-content";
    if (!expect(!store.publish(std::move(conflict), &error) &&
                    store.find(root.stateKey) == publishedRoot,
                "a state key must not be rebound to different content")) {
        return false;
    }

    ipw::IpwStockCheckpoint child;
    child.stateKey = "child-state";
    child.parentStateKey = root.stateKey;
    child.operationId = "hole-op";
    child.backendId = root.backendId;
    child.contentFingerprint = stock->contentFingerprint();
    child.dexelStock = stock;
    child.volume = stock->volume();
    if (!expect(store.publish(std::move(child), &error) && store.size() == 2,
                "a child checkpoint should publish")) {
        return false;
    }
    store.retain({"child-state"});
    if (!expect(!store.contains(root.stateKey) &&
                    store.contains("child-state") && store.size() == 1,
                "retain should remove unreachable checkpoints")) {
        return false;
    }
    store.clear();
    return expect(store.size() == 0,
                  "clear should remove every checkpoint");
}

QByteArray frozenV1Job()
{
    return QByteArray(R"JSON({
  "format": "oap-machining-job",
  "schema_version": 1,
  "job": {
    "schema_version": 1,
    "id": "frozen-v1-operation",
    "name": "Frozen V1 operation",
    "created_at": "2025-01-01T00:00:00.000Z",
    "modified_at": "2025-01-01T00:00:00.000Z",
    "part": {
      "source_path": "frozen-v1.step",
      "source_fingerprint": "frozen-v1-part",
      "units": "mm",
      "recognition_profile_version": "v1"
    },
    "stock": {
      "source_kind": 0,
      "source_path": "",
      "margin_x": 0,
      "margin_y": 0,
      "margin_z": 0
    },
    "post": {"id": "reference.nc", "version": "1"},
    "active_tool_library_id": "legacy-library",
    "algorithm_id": "hole.auto_tool_library",
    "hole_parameters": {},
    "setups": [{
      "id": "setup-v1",
      "setup_index": 0,
      "face": 0,
      "work_offset": 0,
      "origin": {"x": 0, "y": 0, "z": 0},
      "x_axis": {"x": 1, "y": 0, "z": 0},
      "y_axis": {"x": 0, "y": 1, "z": 0},
      "z_axis": {"x": 0, "y": 0, "z": 1},
      "valid": false,
      "requires_physical_reclamp": false,
      "axis_mapping": "XYZ",
      "operator_instruction": "",
      "confirmation": {"confirmed": false}
    }],
    "tool_controllers": [{
      "id": "tool-v1",
      "library_id": "legacy-library",
      "manual": false,
      "tool": {
        "valid": true,
        "tool_number": 7,
        "tool_id": "drill-6",
        "tool_name": "D6 drill",
        "tool_type": "drill",
        "diameter": 6,
        "tip_angle": 118,
        "drill_cycle": 0,
        "coolant": 0
      }
    }],
    "hole_operations": [{
      "id": "hole-operation-v1",
      "setup_id": "setup-v1",
      "tool_controller_id": "tool-v1",
      "phase": 2,
      "targets": [{
        "feature_id": "hole:v1",
        "geometry_fingerprint": "geometry-v1",
        "source_hole_index": 0,
        "physical_hole_index": 0,
        "strategy": 1,
        "hole": {
          "kind": 1,
          "entry": {"x": 10, "y": 20, "z": 0},
          "bottom": {"x": 10, "y": 20, "z": -12},
          "radius": 3,
          "bottom_profile": 2,
          "entry_resolution": 0,
          "setup_face": 0,
          "setup_face_entry_distance": 0,
          "setup_face_other_distance": 12,
          "setup_face_decision": 1,
          "preserve_entry_side": true,
          "source_face_ids": ["legacy-wall"],
          "parent_relation": 0,
          "parent_counterbore_face_id": "",
          "parent_counterbore_floor_face_id": "",
          "parent_counterbore_radius": 0
        }
      }],
      "enabled": true,
      "sequence": 0,
      "state": 2,
      "input_fingerprint": "v1-operation-input"
    }]
  }
})JSON");
}

bool helixPrecisionSurvivesStorageAndChangesExecutionIdentity()
{
    using namespace oap::manufacturing;
    MachiningJobDefinition definition;
    QString error;
    if (!expect(MachiningJobStorage::deserializeDefinition(
                    frozenV1Job(), definition, &error),
                "legacy job must load for precision migration test") ||
        !expect(definition.holeParameters.hole.helixChordTolerance == 0.0,
                "missing chord tolerance must preserve legacy sampling"))
        return false;
    const auto original = MachiningJobStorage::holeExecutionFingerprint(definition);
    definition.holeParameters.hole.helixChordTolerance = 0.02;
    MachiningJobDefinition restored;
    return expect(MachiningJobStorage::deserializeDefinition(
                      MachiningJobStorage::serializeDefinition(definition),
                      restored, &error),
                  "precision-configured job must round-trip") &&
           expect(restored.holeParameters.hole.helixChordTolerance == 0.02,
                  "job storage must preserve chord tolerance") &&
           expect(original != MachiningJobStorage::holeExecutionFingerprint(restored),
                  "chord tolerance must participate in hole execution identity");
}

bool jobStorageMigratesAndSavesAtomically()
{
    namespace manufacturing = oap::manufacturing;

    manufacturing::MachiningJobDefinition migrated;
    QString error;
    if (!expect(manufacturing::MachiningJobStorage::deserializeDefinition(
                    frozenV1Job(), migrated, &error),
                error.toStdString().c_str()) ||
        !expect(migrated.schemaVersion ==
                        manufacturing::kMachiningJobSchemaVersion &&
                    migrated.recognizedHoles.size() == 1 &&
                    migrated.recognizedPhysicalHoleCount == 1 &&
                    migrated.holeOperations.size() == 1 &&
                    migrated.holeOperations.front().targets.front()
                            .coveredSourceHoleIndices ==
                        std::vector<int>({0}),
                "V1 operation geometry should migrate to V4 sources")) {
        return false;
    }

    manufacturing::PocketOperationDefinition restPocket;
    restPocket.id = "pocket-rest-v1";
    restPocket.setupId = "setup-v1";
    restPocket.toolControllerId = "tool-v1";
    restPocket.restMachiningPreviousToolDiameter = 12.0;
    restPocket.algorithmId = oap::toolpath::kBuiltinPocketAlgorithmId;
    restPocket.target.kind = oap::FeatureKind::Pocket;
    restPocket.target.sourceFaceIds = {"legacy-pocket-floor"};
    restPocket.target.floorFaceId = "legacy-pocket-floor";
    restPocket.target.geometryFingerprint = "legacy-pocket-geometry";
    restPocket.sequence = 1;
    restPocket.state = manufacturing::JobOperationState::Generated;
    restPocket.inputFingerprint = "legacy-pocket-input";
    migrated.pocketOperations.push_back(std::move(restPocket));
    migrated.pocketParameters.clearing.entryMode =
        oap::toolpath::PocketEntryMode::ZigZagRamp;
    migrated.pocketParameters.clearing.rampStartReference =
        oap::toolpath::PocketRampStartReference::OpeningPlane;
    migrated.pocketParameters.clearing.zigzagRampWidth = 9.5;
    migrated.pocketParameters.clearing.rampFeed = 321.0;
    migrated.pocketParameters.clearing.pattern =
        oap::toolpath::ClearingPattern::FreePath;
    migrated.pocketParameters.clearing.expansionDirection =
        oap::toolpath::PocketClearingDirection::OutsideToInside;
    auto& adaptive = migrated.pocketParameters.clearing.adaptive;
    adaptive.targetEngagement = 0.17;
    adaptive.targetEngagementAngleDeg = 62.0;
    adaptive.maximumEngagementAngleDeg = 128.0;
    adaptive.engagementAngleSamples = 96;
    adaptive.minimumTurnRadiusFactor = 0.75;
    adaptive.remainingTolerance = 0.009;
    adaptive.materialCellSize = 0.42;
    adaptive.maximumAdaptivePasses = 47;
    adaptive.lookaheadDepth = 2;
    adaptive.beamWidth = 7;

    manufacturing::ProfileOperationDefinition profile;
    profile.id = "profile-outer-v1";
    profile.setupId = "setup-v1";
    profile.toolControllerId = "tool-v1";
    profile.algorithmId = oap::toolpath::kBuiltinProfileAlgorithmId;
    profile.target.axisLabel = "Z";
    profile.target.normalDirection = 1;
    profile.target.sourceFaceIds = {"outer-wall-1", "outer-wall-2"};
    profile.target.mainOuterFaceIds = {"outer-top", "outer-bottom"};
    profile.target.rootFaceId = "outer-top";
    profile.target.rootWireId = "outer-wire";
    profile.target.geometryFingerprint = "profile-geometry-v1";
    profile.restMachiningPreviousToolDiameter = 14.0;
    profile.finishPass = false;
    profile.sequence = 2;
    profile.state = manufacturing::JobOperationState::Generated;
    profile.inputFingerprint = "profile-input-v1";
    migrated.profileOperations.push_back(profile);
    migrated.profileParameters.profile.sideAllowance = 0.4;
    migrated.profileParameters.profile.machiningMode =
        oap::toolpath::ProfileMachiningMode::ExternalClearing;
    migrated.profileParameters.profile.roughingAllowance = 0.65;
    migrated.profileParameters.profile.finishPass = false;
    migrated.profileParameters.profile.radialPassCount = 3;

    const QByteArray v4 =
        manufacturing::MachiningJobStorage::serializeDefinition(migrated);
    const QJsonObject currentRoot = QJsonDocument::fromJson(v4).object();
    if (!expect(currentRoot.value(QStringLiteral("schema_version")).toInt() == manufacturing::kMachiningJobSchemaVersion &&
                    !currentRoot.value(QStringLiteral("job")).toObject()
                         .contains(QStringLiteral("surface_operations")),
                "current schema must omit removed legacy surface operations")) {
        return false;
    }
    // Empty retired fields migrate; authored or malformed manual input never disappears silently.
    QJsonObject automaticV10 = currentRoot;
    automaticV10.insert(QStringLiteral("schema_version"), 10);
    QJsonObject automaticJob = automaticV10.value(QStringLiteral("job")).toObject();
    automaticJob.insert(QStringLiteral("schema_version"), 10);
    automaticJob.insert(QStringLiteral("manual_hole_plan"), QJsonObject{
        {QStringLiteral("version"), 1}, {QStringLiteral("stages"), QJsonArray{}},
        {QStringLiteral("last_application_sequence"), 0}});
    QJsonArray automaticOperations = automaticJob.value(QStringLiteral("hole_operations")).toArray();
    QJsonObject automaticOperation = automaticOperations.at(0).toObject();
    automaticOperation.insert(QStringLiteral("operation_overrides"),
        QJsonObject{{QStringLiteral("retract_preference"), 0}});
    automaticOperation.insert(QStringLiteral("local_batch"), false);
    automaticOperations.replace(0, automaticOperation);
    automaticJob.insert(QStringLiteral("hole_operations"), automaticOperations);
    automaticV10.insert(QStringLiteral("job"), automaticJob);
    manufacturing::MachiningJobDefinition restoredAutomatic;
    if (!expect(manufacturing::MachiningJobStorage::deserializeDefinition(
                    QJsonDocument(automaticV10).toJson(), restoredAutomatic, &error) &&
                    restoredAutomatic.schemaVersion == manufacturing::kMachiningJobSchemaVersion &&
                    !QJsonDocument::fromJson(manufacturing::MachiningJobStorage::serializeDefinition(restoredAutomatic))
                        .object().value(QStringLiteral("job")).toObject().contains(QStringLiteral("manual_hole_plan")),
                "automatic V10 projects must migrate without retired manual fields")) return false;
    for (int invalidCase = 0; invalidCase < 11; ++invalidCase) {
        QJsonObject rejected = automaticV10;
        QJsonObject rejectedJob = automaticJob;
        if (invalidCase < 3) {
            rejectedJob.insert(QStringLiteral("manual_hole_plan"), invalidCase == 0
                ? QJsonValue(QJsonObject{{QStringLiteral("stages"), QJsonArray{QJsonObject{}}}})
                : invalidCase == 1 ? QJsonValue(QJsonValue::Null) : QJsonValue(QJsonObject{}));
        } else if (invalidCase < 6) {
            QJsonObject operation = automaticOperation;
            if (invalidCase == 3) operation.insert(QStringLiteral("operation_overrides"), QJsonObject{{QStringLiteral("feed"), 12}});
            if (invalidCase == 4) operation.insert(QStringLiteral("operation_overrides"), QJsonValue(QJsonValue::Null));
            if (invalidCase == 5) operation.insert(QStringLiteral("local_batch"), true);
            QJsonArray operations = automaticOperations;
            operations.replace(0, operation);
            rejectedJob.insert(QStringLiteral("hole_operations"), operations);
        } else if (invalidCase < 8) {
            QJsonArray controllers = rejectedJob.value(QStringLiteral("tool_controllers")).toArray();
            QJsonObject controller = controllers.at(0).toObject();
            controller.insert(QStringLiteral("manual"), invalidCase == 6 ? QJsonValue(true) : QJsonValue(QStringLiteral("invalid")));
            controllers.replace(0, controller);
            rejectedJob.insert(QStringLiteral("tool_controllers"), controllers);
        } else if (invalidCase < 10) {
            QJsonObject operation = automaticOperation;
            operation.insert(QStringLiteral("operation_overrides"), QJsonObject{
                {QStringLiteral("retract_preference"), invalidCase == 8 ? QJsonValue(1) : QJsonValue(QStringLiteral("0"))}});
            QJsonArray operations = automaticOperations;
            operations.replace(0, operation);
            rejectedJob.insert(QStringLiteral("hole_operations"), operations);
        } else {
            QJsonArray controllers = rejectedJob.value(QStringLiteral("tool_controllers")).toArray();
            QJsonObject controller = controllers.at(0).toObject();
            QJsonObject tool = controller.value(QStringLiteral("tool")).toObject();
            tool.insert(QStringLiteral("valid"), false);
            controller.insert(QStringLiteral("tool"), tool);
            controllers.replace(0, controller);
            rejectedJob.insert(QStringLiteral("tool_controllers"), controllers);
        }
        rejected.insert(QStringLiteral("job"), rejectedJob);
        auto unchanged = migrated;
        if (!expect(!manufacturing::MachiningJobStorage::deserializeDefinition(
                        QJsonDocument(rejected).toJson(), unchanged, &error) && !error.isEmpty() &&
                        manufacturing::MachiningJobStorage::serializeDefinition(unchanged) == v4,
                    "removed manual input must fail atomically with an explanatory error")) return false;
    }
    for (const int legacyVersion : {8, 9}) {
      for (const bool withEmptyArray : {false, true}) {
        QJsonObject legacyRoot = currentRoot;
        legacyRoot.insert(QStringLiteral("schema_version"), legacyVersion);
        QJsonObject legacyJob = legacyRoot.value(QStringLiteral("job")).toObject();
        legacyJob.insert(QStringLiteral("schema_version"), legacyVersion);
        legacyJob.remove(QStringLiteral("surface_finishing_operations"));
        if (withEmptyArray)
            legacyJob.insert(QStringLiteral("surface_operations"), QJsonArray{});
        legacyRoot.insert(QStringLiteral("job"), legacyJob);
        manufacturing::MachiningJobDefinition restored;
        if (!expect(manufacturing::MachiningJobStorage::deserializeDefinition(
                        QJsonDocument(legacyRoot).toJson(), restored, &error) &&
                        restored.schemaVersion == manufacturing::kMachiningJobSchemaVersion &&
                        restored.holeOperations.size() == migrated.holeOperations.size() &&
                        restored.pocketOperations.size() == migrated.pocketOperations.size() &&
                        restored.profileOperations.size() == migrated.profileOperations.size(),
                    "schema 8/9 migration must preserve old families without creating finishing")) {
            return false;
        }
      }
    }
    const std::vector<QJsonValue> removedSurfaceValues{
        QJsonArray{QJsonObject{{QStringLiteral("id"), QStringLiteral("legacy-surface")}}},
        QJsonObject{}, QJsonValue(QJsonValue::Null), QStringLiteral("invalid")};
    for (const int version : {8, 9}) {
        for (const QJsonValue& surfaceValue : removedSurfaceValues) {
            QJsonObject rejectedRoot = currentRoot;
            rejectedRoot.insert(QStringLiteral("schema_version"), version);
            QJsonObject rejectedJob = rejectedRoot.value(QStringLiteral("job")).toObject();
            rejectedJob.insert(QStringLiteral("schema_version"), version);
            rejectedJob.insert(QStringLiteral("surface_operations"), surfaceValue);
            rejectedRoot.insert(QStringLiteral("job"), rejectedJob);
            manufacturing::MachiningJobDefinition unchanged = migrated;
            error.clear();
            if (!expect(!manufacturing::MachiningJobStorage::deserializeDefinition(
                            QJsonDocument(rejectedRoot).toJson(), unchanged, &error) &&
                            error.contains(QStringLiteral("曲面加工模块已移除")) &&
                            manufacturing::MachiningJobStorage::serializeDefinition(unchanged) == v4,
                        "removed or malformed surface operations must fail without replacing the active definition")) {
                return false;
            }
        }
    }

    for (const int legacyVersion : {2, 3, 4}) {
        QJsonObject root = QJsonDocument::fromJson(v4).object();
        root.insert(QStringLiteral("schema_version"), legacyVersion);
        QJsonObject job = root.value(QStringLiteral("job")).toObject();
        job.insert(QStringLiteral("schema_version"), legacyVersion);
        root.insert(QStringLiteral("job"), job);
        const QByteArray versioned = QJsonDocument(root).toJson();
        manufacturing::MachiningJobDefinition loaded;
        error.clear();
        if (!expect(
                manufacturing::MachiningJobStorage::deserializeDefinition(
                    versioned, loaded, &error) &&
                    loaded.schemaVersion ==
                        manufacturing::kMachiningJobSchemaVersion &&
                    loaded.pocketOperations.size() == 1 &&
                    loaded.profileOperations.size() == 1 &&
                    loaded.profileOperations.front().target.rootWireId ==
                        "outer-wire" &&
                    loaded.profileOperations.front().target.mainOuterFaceIds ==
                        std::vector<std::string>{"outer-top", "outer-bottom"} &&
                    std::abs(loaded.profileOperations.front()
                                 .restMachiningPreviousToolDiameter -
                             14.0) < 1.0e-9 &&
                    !loaded.profileOperations.front().finishPass &&
                    std::abs(loaded.profileParameters.profile.sideAllowance -
                             0.4) < 1.0e-9 &&
                    loaded.profileParameters.profile.machiningMode ==
                        oap::toolpath::ProfileMachiningMode::ExternalClearing &&
                    std::abs(loaded.profileParameters.profile.roughingAllowance -
                             0.65) < 1.0e-9 &&
                    !loaded.profileParameters.profile.finishPass &&
                    loaded.profileParameters.profile.radialPassCount == 3 &&
                    loaded.pocketParameters.clearing.entryMode ==
                        oap::toolpath::PocketEntryMode::ZigZagRamp &&
                    loaded.pocketParameters.clearing.rampStartReference ==
                        oap::toolpath::PocketRampStartReference::OpeningPlane &&
                    std::abs(loaded.pocketParameters.clearing
                                 .zigzagRampWidth - 9.5) < 1.0e-9 &&
                    std::abs(loaded.pocketParameters.clearing.rampFeed -
                             321.0) < 1.0e-9 &&
                    loaded.pocketParameters.clearing.pattern ==
                        oap::toolpath::ClearingPattern::FreePath &&
                    loaded.pocketParameters.clearing.expansionDirection ==
                        oap::toolpath::PocketClearingDirection::InsideToOutside &&
                    std::abs(loaded.pocketParameters.clearing.adaptive
                                 .targetEngagement - 0.17) < 1.0e-9 &&
                    std::abs(loaded.pocketParameters.clearing.adaptive
                                 .targetEngagementAngleDeg - 62.0) < 1.0e-9 &&
                    std::abs(loaded.pocketParameters.clearing.adaptive
                                 .maximumEngagementAngleDeg - 128.0) < 1.0e-9 &&
                    loaded.pocketParameters.clearing.adaptive
                            .engagementAngleSamples == 96 &&
                    std::abs(loaded.pocketParameters.clearing.adaptive
                                 .minimumTurnRadiusFactor - 0.75) < 1.0e-9 &&
                    std::abs(loaded.pocketParameters.clearing.adaptive
                                 .remainingTolerance - 0.009) < 1.0e-9 &&
                    std::abs(loaded.pocketParameters.clearing.adaptive
                                 .materialCellSize - 0.42) < 1.0e-9 &&
                    loaded.pocketParameters.clearing.adaptive
                            .maximumAdaptivePasses == 47 &&
                    loaded.pocketParameters.clearing.adaptive
                            .lookaheadDepth == 2 &&
                    loaded.pocketParameters.clearing.adaptive
                            .beamWidth == 7 &&
                    std::abs(
                        loaded.pocketOperations.front()
                                .restMachiningPreviousToolDiameter -
                            12.0) < 1.0e-9,
                "V2-V4 documents should normalize to the current schema")) {
            if (!error.isEmpty())
                std::cerr << error.toStdString() << '\n';
            return false;
        }
    }

    // V5 已有 Profile 工序，但没有多刀余料字段；迁移时应恢复为
    // “单刀全量 + 保留精修”的旧行为。
    QJsonObject legacyV5Root = QJsonDocument::fromJson(v4).object();
    legacyV5Root.insert(QStringLiteral("schema_version"), 5);
    QJsonObject legacyV5Job =
        legacyV5Root.value(QStringLiteral("job")).toObject();
    legacyV5Job.insert(QStringLiteral("schema_version"), 5);
    QJsonArray legacyV5Profiles = legacyV5Job.value(
        QStringLiteral("profile_operations")).toArray();
    QJsonObject legacyV5Profile = legacyV5Profiles.at(0).toObject();
    legacyV5Profile.remove(
        QStringLiteral("rest_previous_tool_diameter"));
    legacyV5Profile.remove(QStringLiteral("finish_pass"));
    legacyV5Profiles.replace(0, legacyV5Profile);
    legacyV5Job.insert(
        QStringLiteral("profile_operations"), legacyV5Profiles);
    legacyV5Root.insert(QStringLiteral("job"), legacyV5Job);
    manufacturing::MachiningJobDefinition migratedV5;
    if (!expect(
            manufacturing::MachiningJobStorage::deserializeDefinition(
                QJsonDocument(legacyV5Root).toJson(), migratedV5, &error) &&
                migratedV5.profileOperations.size() == 1 &&
                migratedV5.profileOperations.front()
                        .restMachiningPreviousToolDiameter == 0.0 &&
                migratedV5.profileOperations.front().finishPass,
            "a V5 profile operation should migrate to single-tool defaults")) {
        if (!error.isEmpty())
            std::cerr << error.toStdString() << '\n';
        return false;
    }

    // 真实 V4 文档不含 Profile 字段；迁移到当前版本时必须保留旧槽腔参数并
    // 让新工序族保持空定义，不能把缺字段判为损坏。
    QJsonObject legacyV4Root = QJsonDocument::fromJson(v4).object();
    legacyV4Root.insert(QStringLiteral("format"), QStringLiteral("faceface-machining-job"));
    legacyV4Root.insert(QStringLiteral("schema_version"), 4);
    QJsonObject legacyV4Job =
        legacyV4Root.value(QStringLiteral("job")).toObject();
    legacyV4Job.insert(QStringLiteral("schema_version"), 4);
    legacyV4Job.remove(QStringLiteral("profile_parameters"));
    legacyV4Job.remove(QStringLiteral("profile_operations"));
    legacyV4Root.insert(QStringLiteral("job"), legacyV4Job);
    manufacturing::MachiningJobDefinition migratedV4;
    if (!expect(
            manufacturing::MachiningJobStorage::deserializeDefinition(
                QJsonDocument(legacyV4Root).toJson(), migratedV4, &error) &&
                migratedV4.profileOperations.empty() &&
                QJsonDocument::fromJson(manufacturing::MachiningJobStorage::serializeDefinition(migratedV4))
                    .object().value(QStringLiteral("format")).toString() ==
                    QStringLiteral("oap-machining-job") &&
                migratedV4.profileParameters.clearing.entryMode ==
                    migratedV4.pocketParameters.clearing.entryMode,
            "a genuine V4 document should migrate with an empty Profile family")) {
        if (!error.isEmpty())
            std::cerr << error.toStdString() << '\n';
        return false;
    }

    manufacturing::MachiningJobDefinition stale = migrated;
    stale.setups.front().confirmation.confirmed = true;
    stale.setups.front().confirmation.confirmedInputFingerprint =
        manufacturing::MachiningJobStorage::setupInputFingerprint(
            stale.part, stale.setups.front()).toStdString();
    stale.part.sourceFingerprint = "changed-part-fingerprint";
    stale.holeOperations.front().state =
        manufacturing::JobOperationState::Generated;
    manufacturing::MachiningJobDefinition reviewed;
    if (!expect(manufacturing::MachiningJobStorage::deserializeDefinition(
                    manufacturing::MachiningJobStorage::serializeDefinition(
                        stale),
                    reviewed,
                    &error) &&
                    !reviewed.setups.front().confirmation.confirmed &&
                    reviewed.holeOperations.front().state ==
                        manufacturing::JobOperationState::NeedsReview,
                "a stale setup fingerprint should force operation review")) {
        return false;
    }

    QTemporaryDir directory;
    if (!expect(directory.isValid(),
                "the Job storage test directory should be available")) {
        return false;
    }
    const QString path = directory.filePath(QStringLiteral("job.oapjob"));
    QFile sentinel(path);
    if (!sentinel.open(QIODevice::WriteOnly) ||
        sentinel.write("old-content") != 11) {
        return expect(false, "the pre-existing Job file should be writable");
    }
    sentinel.close();

    if (!expect(manufacturing::MachiningJobStorage::save(
                    path, migrated, &error),
                error.toStdString().c_str())) {
        return false;
    }
    manufacturing::MachiningJobDefinition loaded;
    if (!expect(manufacturing::MachiningJobStorage::load(
                    path, loaded, &error) &&
                    manufacturing::MachiningJobStorage::definitionFingerprint(
                        loaded) ==
                        manufacturing::MachiningJobStorage::definitionFingerprint(
                            migrated),
                "an atomically saved Job should round-trip unchanged")) {
        return false;
    }
    const QStringList files = QDir(directory.path()).entryList(
        QDir::Files | QDir::NoDotAndDotDot);
    return expect(files == QStringList{QStringLiteral("job.oapjob")},
                  "QSaveFile should not leave a temporary sibling behind");
}

bool jobApplicationServiceSynchronizesSetupsAndStates()
{
    namespace manufacturing = oap::manufacturing;

    const oap::Bbox originalBounds{0.0, 0.0, 0.0, 20.0, 10.0, 5.0};
    const oap::MoldCoordinateFrame originalFrame =
        oap::MoldCoordinateFrame::fromBounds(originalBounds);
    oap::toolpath::MoldSetupWorkFrames frames =
        oap::toolpath::MoldSetupWorkFrames::fromMold(
            originalBounds, originalFrame);
    if (!expect(frames.isValid() &&
                    frames.setConfirmed(oap::MoldSetupFace::PositiveZ, true),
                "the setup synchronization fixture should be valid")) {
        return false;
    }

    manufacturing::MachiningJobDefinition definition;
    definition.part.sourceFingerprint = "part-v1";
    manufacturing::SetupDefinition confirmedSetup;
    confirmedSetup.id = "setup-z-plus";
    confirmedSetup.face = oap::MoldSetupFace::PositiveZ;
    manufacturing::SetupDefinition reviewSetup;
    reviewSetup.id = "setup-x-plus";
    reviewSetup.face = oap::MoldSetupFace::PositiveX;
    definition.setups = {confirmedSetup, reviewSetup};

    manufacturing::HoleOperationDefinition generatedHole;
    generatedHole.id = "hole-confirmed";
    generatedHole.setupId = confirmedSetup.id;
    manufacturing::HoleOperationDefinition reviewHole = generatedHole;
    reviewHole.id = "hole-review";
    reviewHole.setupId = reviewSetup.id;
    definition.holeOperations = {generatedHole, reviewHole};

    manufacturing::PocketOperationDefinition generatedPocket;
    generatedPocket.id = "pocket-confirmed";
    generatedPocket.setupId = confirmedSetup.id;
    definition.pocketOperations = {generatedPocket};

    const manufacturing::SetupSynchronizationReport captured =
        manufacturing::MachiningJobApplicationService::captureSetups(
            definition, frames, "2026-08-04T10:00:00Z", "test-user");
    if (!expect(captured.validSetupCount == 2 &&
                    definition.setups[0].confirmation.confirmed &&
                    definition.setups[0].confirmation.confirmedBy ==
                        "test-user" &&
                    definition.setups[0].confirmation.confirmedAt ==
                        "2026-08-04T10:00:00Z" &&
                    !definition.setups[1].confirmation.confirmed,
                "capture should persist only current frame confirmations")) {
        return false;
    }

    const std::string savedFingerprint = definition.setups[0]
        .confirmation.confirmedInputFingerprint;
    const manufacturing::SetupSynchronizationReport recaptured =
        manufacturing::MachiningJobApplicationService::captureSetups(
            definition, frames, "2026-08-04T11:00:00Z", "other-user");
    if (!expect(recaptured.retainedConfirmationCount == 1 &&
                    definition.setups[0].confirmation.confirmedAt ==
                        "2026-08-04T10:00:00Z" &&
                    definition.setups[0].confirmation
                            .confirmedInputFingerprint == savedFingerprint,
                "unchanged setup input should retain confirmation identity")) {
        return false;
    }

    oap::toolpath::HoleToolpathBatch holeResult;
    oap::toolpath::PocketToolpathBatch pocketResult;
    manufacturing::MachiningJobApplicationService::reconcileOperationStates(
        definition, &holeResult, &pocketResult);
    if (!expect(definition.holeOperations[0].state ==
                        manufacturing::JobOperationState::Generated &&
                    definition.holeOperations[1].state ==
                        manufacturing::JobOperationState::NeedsReview &&
                    definition.pocketOperations[0].state ==
                        manufacturing::JobOperationState::Generated,
                "result state should respect setup confirmation")) {
        return false;
    }
    holeResult.validation.warningCount = 1;
    manufacturing::MachiningJobApplicationService::reconcileOperationStates(
        definition, &holeResult, nullptr);
    if (!expect(definition.holeOperations[0].state ==
                        manufacturing::JobOperationState::Warning &&
                    definition.holeOperations[1].state ==
                        manufacturing::JobOperationState::NeedsReview,
                "warnings should not override an unconfirmed setup review")) {
        return false;
    }
    holeResult.validation.errorCount = 1;
    manufacturing::MachiningJobApplicationService::reconcileOperationStates(
        definition, &holeResult, nullptr);
    if (!expect(definition.holeOperations[0].state ==
                        manufacturing::JobOperationState::Error &&
                    definition.holeOperations[1].state ==
                        manufacturing::JobOperationState::Error,
                "invalid derived output should put every related operation in error")) {
        return false;
    }

    const oap::Bbox shiftedBounds{1.0, 0.0, 0.0, 21.0, 10.0, 5.0};
    const oap::MoldCoordinateFrame shiftedFrame =
        oap::MoldCoordinateFrame::fromBounds(shiftedBounds);
    const oap::toolpath::MoldSetupWorkFrames shiftedFrames =
        oap::toolpath::MoldSetupWorkFrames::fromMold(
            shiftedBounds, shiftedFrame);
    const manufacturing::SetupSynchronizationReport rebound =
        manufacturing::MachiningJobApplicationService::rebindLoadedSetups(
            definition, shiftedFrames);
    return expect(rebound.clearedConfirmationCount == 1 &&
                      rebound.operationsMarkedForReview == 3 &&
                      !definition.setups[0].confirmation.confirmed &&
                      definition.holeOperations[0].state ==
                          manufacturing::JobOperationState::NeedsReview &&
                      definition.pocketOperations[0].state ==
                          manufacturing::JobOperationState::NeedsReview,
                  "changed setup geometry should clear confirmation and review all consumers");
}







} // namespace

int main()
{
    const bool ok = helixPrecisionSurvivesStorageAndChangesExecutionIdentity() &&
                    ipwSessionInvalidatesOnlyChangedSuffix() &&
                    machiningJobBuildsSharedGlobalPlan() &&
                    profileExecutorPublishesMaterialCheckpoint() &&
                    multiSetupExecutorKeepsOneNativeCheckpointGrid() &&
                    checkpointStoreRejectsConflictingContent() &&
                    jobStorageMigratesAndSavesAtomically() &&
                    jobApplicationServiceSynchronizesSetupsAndStates();
    if (!ok)
        return 1;
    std::cout << "machining job smoke tests passed\n";
    return 0;
}

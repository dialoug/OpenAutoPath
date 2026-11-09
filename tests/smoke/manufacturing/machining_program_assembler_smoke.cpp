// 项目级正式程序组装回归：只构造内存 Job/runtime，不依赖本地模型文件。
#include "manufacturing/program/MachiningProgramAssembler.h"
#include "manufacturing/program/MachiningProgramPartitioner.h"

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include <BRepPrimAPI_MakeBox.hxx>

#include "manufacturing/job/MachiningJobStorage.h"
#include "toolpath/pocket/simulation/MultiDexelStock.h"
#include "toolpath/core/ToolpathPrimitives.h"

namespace {

using oap::manufacturing::JobOperationState;
using oap::manufacturing::MachiningProgramAssemblyIssueCode;
using oap::manufacturing::MachiningProgramPartitionIssueCode;

bool expect(bool condition, const char* message)
{
    if (condition)
        return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

bool hasIssue(
    const oap::manufacturing::MachiningProgramAssemblyResult& result,
    MachiningProgramAssemblyIssueCode code)
{
    return std::any_of(
        result.issues.begin(), result.issues.end(),
        [code](const auto& issue) { return issue.code == code; });
}

bool hasPartitionIssue(
    const oap::manufacturing::MachiningProgramPartitionResult& result,
    MachiningProgramPartitionIssueCode code)
{
    return std::any_of(
        result.issues.begin(), result.issues.end(),
        [code](const auto& issue) { return issue.code == code; });
}

oap::toolpath::ToolpathToolContext tool(
    int number,
    std::string id,
    std::string type,
    double diameter,
    double cornerRadius = 0.0)
{
    oap::toolpath::ToolpathToolContext result;
    result.valid = true;
    result.toolNumber = number;
    result.toolId = std::move(id);
    result.toolName = result.toolId;
    result.toolType = std::move(type);
    result.diameter = diameter;
    result.cornerRadius = cornerRadius;
    result.spindleSpeed = 6000.0;
    result.horizontalFeed = 300.0;
    result.verticalFeed = 100.0;
    result.horizontalRapid = 1200.0;
    result.verticalRapid = 800.0;
    result.cuttingEdgeLength = 12.0;
    result.overallLength = 30.0;
    result.shankDiameter = diameter;
    result.holderDiameter = diameter + 4.0;
    result.holderLength = 10.0;
    result.spindleEnvelopeDiameter = diameter + 8.0;
    result.spindleEnvelopeLength = 8.0;
    return result;
}

oap::toolpath::ToolpathMove cutMove(double x)
{
    oap::toolpath::ToolpathMove result;
    result.type = oap::toolpath::ToolpathMoveType::Cut;
    result.from = gp_Pnt(x, 0.0, 1.0);
    result.to = gp_Pnt(x + 1.0, 0.0, 1.0);
    result.feed = 300.0;
    return result;
}

oap::manufacturing::SetupDefinition setup(
    const oap::manufacturing::PartDefinition& part,
    std::string id = "setup-z-plus",
    int setupIndex = 0,
    oap::MoldSetupFace face = oap::MoldSetupFace::PositiveZ,
    oap::toolpath::WorkCoordinateSystem workOffset =
        oap::toolpath::WorkCoordinateSystem::G54)
{
    oap::manufacturing::SetupDefinition result;
    result.id = std::move(id);
    result.setupIndex = setupIndex;
    result.face = face;
    result.workOffset = workOffset;
    result.valid = true;
    result.confirmation.confirmed = true;
    result.confirmation.confirmedBy = "test";
    result.confirmation.confirmedAt = "2026-08-24T00:00:00Z";
    result.confirmation.confirmedInputFingerprint =
        oap::manufacturing::MachiningJobStorage::setupInputFingerprint(
            part, result).toStdString();
    return result;
}

oap::manufacturing::MachiningJob validJob()
{
    namespace manufacturing = oap::manufacturing;
    namespace toolpath = oap::toolpath;

    manufacturing::MachiningJob job;
    auto& definition = job.definition();
    definition.id = "project-program-fixture";
    definition.name = "M6 assembly fixture";
    definition.part.sourcePath = "fixture.step";
    definition.part.sourceFingerprint = "fixture-part-v1";
    definition.setups.push_back(setup(definition.part));

    manufacturing::JobToolController drill;
    drill.id = "drill-controller";
    drill.libraryId = "fixture-library";
    drill.tool = tool(1, "drill-4", "drill", 4.0);
    manufacturing::JobToolController pocket;
    pocket.id = "pocket-controller";
    pocket.libraryId = "fixture-library";
    pocket.tool = tool(2, "flat-6", "flat_end_mill", 6.0);
    manufacturing::JobToolController profile;
    profile.id = "profile-controller";
    profile.libraryId = "fixture-library";
    profile.tool = tool(3, "flat-8", "flat_end_mill", 8.0);
    definition.toolControllers = {drill, pocket, profile};

    toolpath::HoleMachiningInput holeInput;
    holeInput.entry = gp_Pnt(0.0, 0.0, 4.0);
    holeInput.bottom = gp_Pnt(0.0, 0.0, 0.0);
    holeInput.radius = 2.0;
    holeInput.kind = oap::FeatureKind::BlindHole;
    holeInput.mappingState = toolpath::HoleMachiningMappingState::Ready;
    holeInput.entryResolution =
        oap::HoleEntryResolution::MoldEnvelopeProjected;
    holeInput.setupFace = oap::MoldSetupFace::PositiveZ;
    definition.recognizedHoles = {holeInput};
    definition.recognizedPhysicalHoleCount = 1;

    manufacturing::HoleTargetReference holeTarget;
    holeTarget.featureId = "hole-feature-0";
    holeTarget.geometryFingerprint = "hole-geometry-v1";
    holeTarget.sourceHoleIndex = 0;
    holeTarget.coveredSourceHoleIndices = {0};
    holeTarget.physicalHoleIndex = 0;
    holeTarget.hole = holeInput;
    holeTarget.strategy = toolpath::HoleMachiningStrategy::SmallDrill;
    manufacturing::HoleOperationDefinition holeDefinition;
    holeDefinition.id = "hole-op";
    holeDefinition.setupId = definition.setups.front().id;
    holeDefinition.toolControllerId = drill.id;
    holeDefinition.phase = toolpath::HoleMachiningPhase::Drill;
    holeDefinition.targets = {holeTarget};
    holeDefinition.sequence = 30;
    holeDefinition.state = JobOperationState::Generated;
    holeDefinition.inputFingerprint = "hole-input-v1";
    definition.holeOperations = {holeDefinition};

    manufacturing::PocketOperationDefinition pocketDefinition;
    pocketDefinition.id = "pocket-op";
    pocketDefinition.setupId = definition.setups.front().id;
    pocketDefinition.toolControllerId = pocket.id;
    pocketDefinition.algorithmId = "builtin.pocket.freecad";
    pocketDefinition.target.kind = oap::FeatureKind::Pocket;
    pocketDefinition.target.sourceFaceIds = {"pocket-floor"};
    pocketDefinition.target.floorFaceId = "pocket-floor";
    pocketDefinition.target.geometryFingerprint = "pocket-geometry-v1";
    pocketDefinition.sequence = 10;
    pocketDefinition.state = JobOperationState::Generated;
    pocketDefinition.inputFingerprint = "pocket-input-v1";
    definition.pocketOperations = {pocketDefinition};

    manufacturing::ProfileOperationDefinition profileDefinition;
    profileDefinition.id = "profile-op";
    profileDefinition.setupId = definition.setups.front().id;
    profileDefinition.toolControllerId = profile.id;
    profileDefinition.algorithmId = "builtin.profile.freecad";
    profileDefinition.target.axisLabel = "Z";
    profileDefinition.target.sourceFaceIds = {"profile-face"};
    profileDefinition.target.rootFaceId = "profile-root-face";
    profileDefinition.target.rootWireId = "profile-root-wire";
    profileDefinition.target.geometryFingerprint = "profile-geometry-v1";
    profileDefinition.sequence = 20;
    profileDefinition.state = JobOperationState::Generated;
    profileDefinition.inputFingerprint = "profile-input-v1";
    definition.profileOperations = {profileDefinition};

    auto finishDefinition = profileDefinition;
    finishDefinition.id = "profile-finish";
    finishDefinition.sequence = 40;
    finishDefinition.inputFingerprint = "profile-finish-input-v1";
    definition.profileOperations.push_back(finishDefinition);

    toolpath::HoleToolpathBatch holeBatch;
    for (int index = 0; index < 2; ++index) {
        toolpath::HoleMachiningOperation operation;
        operation.operationId = "hole-op-" + std::to_string(index);
        operation.algorithmId = definition.algorithmId;
        operation.sourceHoleIndex = 0;
        operation.coveredSourceHoleIndices = {0};
        operation.kind = oap::FeatureKind::BlindHole;
        operation.strategy = toolpath::HoleMachiningStrategy::SmallDrill;
        operation.phase = toolpath::HoleMachiningPhase::Drill;
        operation.setupFace = definition.setups.front().face;
        operation.setupIndex = definition.setups.front().setupIndex;
        operation.tool = drill.tool;
        operation.entry = gp_Pnt(index * 2.0, 0.0, 4.0);
        operation.bottom = gp_Pnt(index * 2.0, 0.0, 0.0);
        operation.commandedBottom = operation.bottom;
        operation.hasCommandedBottom = true;
        operation.radius = 2.0;
        operation.moves.push_back(cutMove(30.0 + index));
        operation.primitives.push_back(
            toolpath::makeLinearPrimitive(operation.moves.front()));
        holeBatch.machining.operations.push_back(std::move(operation));
    }
    holeBatch.algorithmId = definition.algorithmId;
    job.setHoleResult(
        std::move(holeBatch),
        manufacturing::MachiningJobStorage::holeExecutionFingerprint(
            definition).toStdString());

    toolpath::PocketToolpathBatch pocketBatch;
    toolpath::PocketToolpathOperation pocketOperation;
    pocketOperation.operationId = "pocket-op-0";
    pocketOperation.algorithmId = pocketDefinition.algorithmId;
    pocketOperation.setupIndex = definition.setups.front().setupIndex;
    pocketOperation.setupFace = definition.setups.front().face;
    pocketOperation.tool = pocket.tool;
    pocketOperation.clearing.ok = true;
    pocketOperation.clearing.moves.push_back(cutMove(10.0));
    pocketOperation.clearing.primitives.push_back(
        toolpath::makeLinearPrimitive(
            pocketOperation.clearing.moves.front()));
    pocketBatch.operations.push_back(std::move(pocketOperation));
    job.setPocketResult(
        std::move(pocketBatch),
        manufacturing::MachiningJobStorage::pocketExecutionFingerprint(
            definition).toStdString());

    toolpath::ProfileToolpathBatch profileBatch;
    toolpath::ProfileToolpathOperation profileOperation;
    profileOperation.operationId = "profile-op";
    profileOperation.algorithmId = profileDefinition.algorithmId;
    profileOperation.setupIndex = definition.setups.front().setupIndex;
    profileOperation.setupFace = definition.setups.front().face;
    profileOperation.tool = profile.tool;
    profileOperation.contour.ok = true;
    profileOperation.contour.moves.push_back(cutMove(20.0));
    profileOperation.contour.primitives.push_back(
        toolpath::makeLinearPrimitive(
            profileOperation.contour.moves.front()));
    profileBatch.operations.push_back(std::move(profileOperation));
    job.setProfileResult(
        std::move(profileBatch),
        manufacturing::MachiningJobStorage::profileExecutionFingerprint(
            definition).toStdString());

    auto finishRuntime = job.profileResult()->operations.front();
    finishRuntime.operationId = "profile-finish";
    finishRuntime.contour.moves = {cutMove(40.0)};
    finishRuntime.contour.primitives = {
        toolpath::makeLinearPrimitive(finishRuntime.contour.moves.front())};
    job.profileResult()->operations.push_back(std::move(finishRuntime));
    return job;
}

oap::manufacturing::MachiningJob partitioningJob()
{
    namespace manufacturing = oap::manufacturing;
    namespace toolpath = oap::toolpath;

    manufacturing::MachiningJob job = validJob();
    auto& definition = job.definition();
    definition.setups.push_back(setup(
        definition.part, "setup-x-plus", 1,
        oap::MoldSetupFace::PositiveX,
        toolpath::WorkCoordinateSystem::G55));

    // 连续 Pocket/Profile 共用同一不可变控制器，应合并为一个包。
    definition.profileOperations.front().toolControllerId =
        definition.pocketOperations.front().toolControllerId;
    job.profileResult()->operations.front().tool =
        job.pocketResult()->operations.front().tool;

    // 同 Setup/刀具增加 CenterSpot；它与后续 Drill 必须按 Hole 阶段拆开。
    manufacturing::HoleOperationDefinition center =
        definition.holeOperations.front();
    center.id = "center-op";
    center.phase = toolpath::HoleMachiningPhase::CenterSpot;
    center.sequence = 25;
    center.inputFingerprint = "center-input-v1";
    definition.holeOperations.push_back(center);
    toolpath::HoleMachiningOperation centerRuntime =
        job.holeResult()->machining.operations.front();
    centerRuntime.operationId = "center-op-0";
    centerRuntime.phase = toolpath::HoleMachiningPhase::CenterSpot;
    job.holeResult()->machining.operations.insert(
        job.holeResult()->machining.operations.begin(),
        std::move(centerRuntime));

    // 相同 Setup/刀具在 Hole 阶段之后再次出现时必须形成新包，不能回收
    // 到前一个同键包并把全局执行顺序改成 Pocket/Profile/Pocket/Hole。
    manufacturing::PocketOperationDefinition latePocket =
        definition.pocketOperations.front();
    latePocket.id = "pocket-late";
    latePocket.sequence = 35;
    latePocket.inputFingerprint = "pocket-late-input-v1";
    latePocket.target.geometryFingerprint =
        "pocket-late-geometry-v1";
    definition.pocketOperations.push_back(latePocket);
    toolpath::PocketToolpathOperation latePocketRuntime =
        job.pocketResult()->operations.front();
    latePocketRuntime.operationId = "pocket-late-0";
    job.pocketResult()->operations.push_back(
        std::move(latePocketRuntime));

    // 最后一道外轮廓使用独立 Setup/G55，形成物理换装边界。
    definition.profileOperations.back().setupId = definition.setups.back().id;
    job.profileResult()->operations.back().setupIndex = definition.setups.back().setupIndex;
    job.profileResult()->operations.back().setupFace = definition.setups.back().face;

    job.runtime().holeExecutionFingerprint =
        manufacturing::MachiningJobStorage::holeExecutionFingerprint(
            definition).toStdString();
    job.runtime().pocketExecutionFingerprint =
        manufacturing::MachiningJobStorage::pocketExecutionFingerprint(
            definition).toStdString();
    job.runtime().profileExecutionFingerprint =
        manufacturing::MachiningJobStorage::profileExecutionFingerprint(
            definition).toStdString();
    return job;
}

bool completeIpw(oap::manufacturing::MachiningJob& job)
{
    namespace ipw = oap::manufacturing::ipw;
    namespace toolpath = oap::toolpath;
    const ipw::IpwPlanUpdate configured =
        job.configureIpwPlan("fixture-stock-v1", "fixture-ipw-engine-v1");
    if (!configured.ok())
        return false;

    toolpath::StockSimulationOptions options;
    options.runParallel = false;
    options.zMapResolution = 1.0;
    options.zMapMaxCellCount = 10000;
    std::string buildError;
    const auto stock = toolpath::MultiDexelStock::fromShape(
        BRepPrimAPI_MakeBox(4.0, 4.0, 4.0).Shape(),
        gp_Vec(0.0, 0.0, 1.0), 4.0, options,
        nullptr, &buildError);
    if (!stock || !stock->valid())
        return false;

    ipw::IpwStockCheckpoint root;
    root.stateKey = job.ipwSession().initialStateKey();
    root.backendId = "fixture-ipw-root";
    root.contentFingerprint = "ipw-root-content";
    root.dexelStock = stock;
    root.volume = stock->volume();
    if (!job.runtime().ipwCheckpoints.publish(std::move(root)))
        return false;

    for (std::size_t index = 0;
         index < job.ipwSession().nodes().size(); ++index) {
        const auto node = job.ipwSession().nodes()[index];
        if (!job.ipwSession().beginNode(index))
            return false;
        ipw::IpwStockCheckpoint checkpoint;
        checkpoint.stateKey = node.stateKey;
        checkpoint.parentStateKey = node.parentStateKey;
        checkpoint.operationId = node.operation.id;
        checkpoint.backendId = "fixture-ipw-backend";
        checkpoint.contentFingerprint =
            "ipw-content-" + std::to_string(index);
        checkpoint.dexelStock = stock;
        checkpoint.volume = stock->volume();
        if (!job.runtime().ipwCheckpoints.publish(
                std::move(checkpoint))) {
            return false;
        }
        ipw::IpwArtifactDescriptor artifact;
        artifact.stateKey = node.stateKey;
        artifact.backendId = "fixture-ipw-backend";
        artifact.contentFingerprint =
            "ipw-content-" + std::to_string(index);
        artifact.checkpoint = true;
        if (!job.ipwSession().completeNode(index, std::move(artifact)))
            return false;
    }
    return job.ipwSession().complete();
}

bool assemblesAllDomainsInGlobalSequence()
{
    const oap::manufacturing::MachiningJob job = validJob();
    const auto result =
        oap::manufacturing::MachiningProgramAssembler().assemble(job);
    if (!expect(result.ok(),
                "fresh formal operations should assemble") ||
        !expect(result.program.operations.size() == 5 &&
                    result.mappings.size() == 4,
                "one project program should retain all runtime children")) {
        for (const auto& issue : result.issues)
            std::cerr << issue.message << '\n';
        return false;
    }
    const auto& operations = result.program.operations;
    const bool ordered =
        operations[0].operationId == "pocket-op-0" &&
        operations[1].operationId == "profile-op" &&
        operations[2].operationId == "hole-op-0" &&
        operations[3].operationId == "hole-op-1" &&
        operations[4].operationId == "profile-finish";
    const bool mapped =
        result.mappings[0].formalOperationId == "pocket-op" &&
        result.mappings[1].formalOperationId == "profile-op" &&
        result.mappings[2].formalOperationId == "hole-op" &&
        result.mappings[2].firstProgramOperation == 2 &&
        result.mappings[2].programOperationCount == 2 &&
        result.mappings[3].formalOperationId == "profile-finish";
    const bool traceable =
        !result.jobDefinitionFingerprint.empty() &&
        operations[4].metadata.at("job_operation_id") == "profile-finish" &&
        operations[4].metadata.at("job_operation_kind") == "profile" &&
        operations[4].metadata.at("job_sequence") == "40";
    return expect(ordered && mapped && traceable,
                  "project assembly must preserve sequence, child ranges, and trace metadata");
}

bool rejectsMissingStaleAndOrphanRuntime()
{
    oap::manufacturing::MachiningProgramAssembler assembler;

    auto missing = validJob();
    missing.clearProfileResult();
    const auto missingResult = assembler.assemble(missing);

    auto stale = validJob();
    stale.runtime().profileExecutionFingerprint = "stale-profile-runtime";
    const auto staleResult = assembler.assemble(stale);

    auto orphan = validJob();
    orphan.pocketResult()->operations.front().operationId = "orphan-runtime";
    const auto orphanResult = assembler.assemble(orphan);

    auto unconfirmed = validJob();
    unconfirmed.definition().setups.front().confirmation = {};
    const auto unconfirmedResult = assembler.assemble(unconfirmed);

    auto wrongPhase = validJob();
    wrongPhase.holeResult()->machining.operations.front().phase =
        oap::toolpath::HoleMachiningPhase::CenterSpot;
    const auto wrongPhaseResult = assembler.assemble(wrongPhase);

    return expect(
        !missingResult.ok() && missingResult.program.empty() &&
            hasIssue(missingResult,
                     MachiningProgramAssemblyIssueCode::MissingRuntimeResult) &&
            !staleResult.ok() && staleResult.program.empty() &&
            hasIssue(staleResult,
                     MachiningProgramAssemblyIssueCode::StaleRuntimeResult) &&
            !orphanResult.ok() && orphanResult.program.empty() &&
            hasIssue(orphanResult,
                     MachiningProgramAssemblyIssueCode::UnmappedRuntimeOperation) &&
            !unconfirmedResult.ok() && unconfirmedResult.program.empty() &&
            hasIssue(unconfirmedResult,
                     MachiningProgramAssemblyIssueCode::UnconfirmedSetup) &&
            !wrongPhaseResult.ok() && wrongPhaseResult.program.empty() &&
            hasIssue(wrongPhaseResult,
                     MachiningProgramAssemblyIssueCode::HolePhaseMismatch),
        "missing, stale, orphaned, or unconfirmed formal inputs must publish no partial program");
}

bool partitionsContinuousSetupToolAndPhaseRuns()
{
    oap::manufacturing::MachiningJob job = partitioningJob();
    if (!expect(completeIpw(job),
                "partition fixture IPW should complete")) {
        return false;
    }
    const auto result =
        oap::manufacturing::MachiningProgramPartitioner().partition(job);
    if (!expect(result.ok() && result.partitions.size() == 5,
                "fresh project program should form five conservative continuous partitions")) {
        for (const auto& issue : result.issues)
            std::cerr << issue.message << '\n';
        return false;
    }

    const auto& combined = result.partitions[0];
    const auto& center = result.partitions[1];
    const auto& drill = result.partitions[2];
    const auto& latePocket = result.partitions[3];
    const auto& finish = result.partitions[4];
    const bool grouping =
        combined.formalOperationIds ==
            std::vector<std::string>({"pocket-op", "profile-op"}) &&
        combined.program.operations.size() == 2 &&
        !combined.holePhaseApplies &&
        center.formalOperationIds ==
            std::vector<std::string>({"center-op"}) &&
        center.holePhaseApplies &&
        center.holePhase ==
            oap::toolpath::HoleMachiningPhase::CenterSpot &&
        drill.formalOperationIds ==
            std::vector<std::string>({"hole-op"}) &&
        drill.holePhase == oap::toolpath::HoleMachiningPhase::Drill &&
        drill.program.operations.size() == 2 &&
        latePocket.formalOperationIds ==
            std::vector<std::string>({"pocket-late"}) &&
        latePocket.setupId == combined.setupId &&
        latePocket.toolControllerId == combined.toolControllerId &&
        finish.setupId == "setup-x-plus" &&
        finish.setupIndex == 1 &&
        finish.workOffset == oap::toolpath::WorkCoordinateSystem::G55;

    std::vector<std::string> partitionedIds;
    for (const auto& partition : result.partitions) {
        for (const auto& operation : partition.program.operations)
            partitionedIds.push_back(operation.operationId);
    }
    std::vector<std::string> assembledIds;
    for (const auto& operation : result.assembly.program.operations)
        assembledIds.push_back(operation.operationId);
    const bool traceable =
        partitionedIds == assembledIds &&
        result.finalIpwStateKey == job.ipwSession().nodes().back().stateKey &&
        result.finalIpwContentFingerprint == "ipw-content-5" &&
        result.ipwEngineFingerprint == "fixture-ipw-engine-v1";
    return expect(
        grouping && traceable,
        "partitions must preserve continuous order, WCS/tool/phase boundaries, and final IPW identity");
}

bool rejectsMissingDirtyAndStaleIpw()
{
    oap::manufacturing::MachiningProgramPartitioner partitioner;

    auto missing = partitioningJob();
    const auto missingResult = partitioner.partition(missing);

    auto dirty = partitioningJob();
    const bool configured = dirty.configureIpwPlan(
        "fixture-stock-v1", "fixture-ipw-engine-v1").ok();
    const auto dirtyResult = partitioner.partition(dirty);

    auto stale = partitioningJob();
    const bool completed = completeIpw(stale);
    stale.definition().profileOperations.front().inputFingerprint =
        "profile-input-v2";
    stale.runtime().profileExecutionFingerprint =
        oap::manufacturing::MachiningJobStorage::profileExecutionFingerprint(
            stale.definition()).toStdString();
    const auto staleResult = partitioner.partition(stale);

    auto missingCheckpoint = partitioningJob();
    const bool completedWithoutStore = completeIpw(missingCheckpoint);
    missingCheckpoint.runtime().ipwCheckpoints.clear();
    const auto missingCheckpointResult =
        partitioner.partition(missingCheckpoint);

    return expect(
        !missingResult.ok() && missingResult.partitions.empty() &&
            hasPartitionIssue(
                missingResult,
                MachiningProgramPartitionIssueCode::MissingIpwPlan) &&
            configured && !dirtyResult.ok() &&
            dirtyResult.partitions.empty() &&
            hasPartitionIssue(
                dirtyResult,
                MachiningProgramPartitionIssueCode::IpwNodeNotReady) &&
            completed && !staleResult.ok() &&
            staleResult.partitions.empty() && staleResult.assembly.ok() &&
            hasPartitionIssue(
                staleResult,
                MachiningProgramPartitionIssueCode::IpwPlanMismatch) &&
            completedWithoutStore && !missingCheckpointResult.ok() &&
            missingCheckpointResult.partitions.empty() &&
            hasPartitionIssue(
                missingCheckpointResult,
                MachiningProgramPartitionIssueCode::MissingIpwCheckpoint),
        "missing, non-Ready, or stale IPW must publish no project partitions");
}

} // namespace

int main()
{
    const bool ok = assemblesAllDomainsInGlobalSequence() &&
        rejectsMissingStaleAndOrphanRuntime() &&
        partitionsContinuousSetupToolAndPhaseRuns() &&
        rejectsMissingDirtyAndStaleIpw();
    if (!ok)
        return 1;
    std::cout << "machining program assembler smoke tests passed\n";
    return 0;
}

// 正式项目程序的连续 Setup/WCS/刀具/Hole 阶段分包与 IPW 发布门禁。
#include "manufacturing/program/MachiningProgramPartitioner.h"

#include <utility>

#include "toolpath/core/ToolpathToolContext.h"

namespace oap::manufacturing {
namespace {

using IssueCode = MachiningProgramPartitionIssueCode;

void addIssue(MachiningProgramPartitionResult& result,
              IssueCode code,
              std::string operationId,
              std::string message)
{
    result.issues.push_back(
        {code, std::move(operationId), std::move(message)});
}

ipw::IpwOperationKind ipwKind(JobOperationKind kind) noexcept
{
    switch (kind) {
        case JobOperationKind::Hole: return ipw::IpwOperationKind::Hole;
        case JobOperationKind::Pocket: return ipw::IpwOperationKind::Pocket;
        case JobOperationKind::Profile: return ipw::IpwOperationKind::Profile;
        case JobOperationKind::SurfaceFinishing: return ipw::IpwOperationKind::SurfaceFinishing;
    }
    return ipw::IpwOperationKind::Other;
}

struct PartitionKey {
    std::string setupId;
    int setupIndex = 0;
    MoldSetupFace setupFace = MoldSetupFace::Unresolved;
    toolpath::WorkCoordinateSystem workOffset =
        toolpath::WorkCoordinateSystem::None;
    std::string toolControllerId;
    bool holePhaseApplies = false;
    toolpath::HoleMachiningPhase holePhase =
        toolpath::HoleMachiningPhase::Standard;

};

PartitionKey partitionKey(
    const MachiningProgramOperationMapping& mapping)
{
    PartitionKey key;
    key.setupId = mapping.setupId;
    key.setupIndex = mapping.setupIndex;
    key.setupFace = mapping.setupFace;
    key.workOffset = mapping.workOffset;
    key.toolControllerId = mapping.toolControllerId;
    key.holePhaseApplies = mapping.kind == JobOperationKind::Hole;
    key.holePhase = key.holePhaseApplies
        ? mapping.holePhase
        : toolpath::HoleMachiningPhase::Standard;
    return key;
}

bool ipwNodeMatches(
    const ipw::IpwNode& node,
    const MachiningProgramOperationMapping& mapping) noexcept
{
    return node.operation.id == mapping.formalOperationId &&
        node.operation.setupId == mapping.setupId &&
        node.operation.toolControllerId == mapping.toolControllerId &&
        node.operation.inputFingerprint == mapping.inputFingerprint &&
        node.operation.sequence == mapping.sequence &&
        node.operation.kind == ipwKind(mapping.kind);
}

void clearPublishedOutput(MachiningProgramPartitionResult& result)
{
    result.partitions.clear();
    result.initialIpwStateKey.clear();
    result.finalIpwStateKey.clear();
    result.finalIpwContentFingerprint.clear();
    result.ipwEngineFingerprint.clear();
}

} // namespace

const char* machiningProgramPartitionIssueCodeId(
    MachiningProgramPartitionIssueCode code) noexcept
{
    switch (code) {
        case IssueCode::AssemblyRejected: return "assembly_rejected";
        case IssueCode::MissingIpwPlan: return "missing_ipw_plan";
        case IssueCode::IpwPlanMismatch: return "ipw_plan_mismatch";
        case IssueCode::IpwNodeNotReady: return "ipw_node_not_ready";
        case IssueCode::MissingIpwArtifact: return "missing_ipw_artifact";
        case IssueCode::StaleIpwArtifact: return "stale_ipw_artifact";
        case IssueCode::MissingIpwCheckpoint:
            return "missing_ipw_checkpoint";
        case IssueCode::StaleIpwCheckpoint:
            return "stale_ipw_checkpoint";
        case IssueCode::PartitionInvariantViolation:
            return "partition_invariant_violation";
    }
    return "assembly_rejected";
}

MachiningProgramPartitionResult MachiningProgramPartitioner::partition(
    const MachiningJob& job) const
{
    MachiningProgramPartitionResult result;
    result.assembly = MachiningProgramAssembler().assemble(job);
    if (!result.assembly.ok()) {
        const std::string operationId = result.assembly.issues.empty()
            ? std::string{}
            : result.assembly.issues.front().operationId;
        const std::string reason = result.assembly.issues.empty()
            ? "formal project program assembly failed"
            : result.assembly.issues.front().message;
        addIssue(
            result, IssueCode::AssemblyRejected, operationId,
            "project program partitioning rejected assembly: " + reason);
        return result;
    }

    const ipw::IpwSession& session = job.ipwSession();
    const std::vector<ipw::IpwNode>& nodes = session.nodes();
    if (!session.configured() || nodes.empty()) {
        addIssue(
            result, IssueCode::MissingIpwPlan, {},
            "project program partitioning requires a configured IPW plan");
        return result;
    }
    if (nodes.size() != result.assembly.mappings.size()) {
        addIssue(
            result, IssueCode::IpwPlanMismatch, {},
            "IPW plan does not cover exactly all enabled formal operations");
        return result;
    }

    const ipw::IpwCheckpointStore& checkpoints =
        job.runtime().ipwCheckpoints;
    const auto rootCheckpoint = checkpoints.find(
        session.initialStateKey());
    if (!rootCheckpoint) {
        addIssue(
            result, IssueCode::MissingIpwCheckpoint, {},
            "IPW initial-state checkpoint is missing");
    } else if (rootCheckpoint->stateKey != session.initialStateKey() ||
               !rootCheckpoint->parentStateKey.empty() ||
               !rootCheckpoint->operationId.empty() ||
               !rootCheckpoint->dexelStock ||
               !rootCheckpoint->dexelStock->valid() ||
               rootCheckpoint->backendId.empty() ||
               rootCheckpoint->contentFingerprint.empty()) {
        addIssue(
            result, IssueCode::StaleIpwCheckpoint, {},
            "IPW initial-state checkpoint is invalid or stale");
    }

    std::string expectedParentStateKey = session.initialStateKey();
    for (std::size_t index = 0; index < nodes.size(); ++index) {
        const ipw::IpwNode& node = nodes[index];
        const MachiningProgramOperationMapping& mapping =
            result.assembly.mappings[index];
        if (!ipwNodeMatches(node, mapping) ||
            node.parentStateKey != expectedParentStateKey ||
            node.stateKey.empty()) {
            addIssue(
                result, IssueCode::IpwPlanMismatch,
                mapping.formalOperationId,
                "IPW node identity or state-key chain does not match the current formal operation");
            continue;
        }
        expectedParentStateKey = node.stateKey;
        if (node.status != ipw::IpwNodeStatus::Ready) {
            addIssue(
                result, IssueCode::IpwNodeNotReady,
                mapping.formalOperationId,
                "IPW node is not Ready for project program publication");
            continue;
        }
        if (!node.artifact) {
            addIssue(
                result, IssueCode::MissingIpwArtifact,
                mapping.formalOperationId,
                "Ready IPW node has no published artifact descriptor");
            continue;
        }
        if (node.artifact->stateKey != node.stateKey ||
            node.artifact->backendId.empty() ||
            node.artifact->contentFingerprint.empty()) {
            addIssue(
                result, IssueCode::StaleIpwArtifact,
                mapping.formalOperationId,
                "IPW artifact identity is empty or does not match its current node");
            continue;
        }
        const auto checkpoint = checkpoints.find(node.stateKey);
        if (!checkpoint) {
            addIssue(
                result, IssueCode::MissingIpwCheckpoint,
                mapping.formalOperationId,
                "Ready IPW node has no immutable stock checkpoint");
            continue;
        }
        if (checkpoint->stateKey != node.stateKey ||
            checkpoint->parentStateKey != node.parentStateKey ||
            checkpoint->operationId != node.operation.id ||
            checkpoint->backendId != node.artifact->backendId ||
            checkpoint->contentFingerprint !=
                node.artifact->contentFingerprint ||
            !checkpoint->dexelStock ||
            !checkpoint->dexelStock->valid()) {
            addIssue(
                result, IssueCode::StaleIpwCheckpoint,
                mapping.formalOperationId,
                "IPW stock checkpoint does not match its current Ready node");
        }
    }
    if (!session.complete() || !result.issues.empty()) {
        clearPublishedOutput(result);
        return result;
    }

    std::size_t nextAssemblyOperation = 0u;
    for (const MachiningProgramOperationMapping& mapping :
         result.assembly.mappings) {
        if (mapping.firstProgramOperation != nextAssemblyOperation ||
            mapping.programOperationCount == 0u ||
            mapping.runtimeOperationIds.size() !=
                mapping.programOperationCount ||
            mapping.firstProgramOperation + mapping.programOperationCount >
                result.assembly.program.operations.size()) {
            addIssue(
                result, IssueCode::PartitionInvariantViolation,
                mapping.formalOperationId,
                "assembly mapping ranges are not contiguous and complete");
            break;
        }

        const PartitionKey key = partitionKey(mapping);
        // 只比较当前连续包自身保存的键；不跨越中间不同键回收旧包。
        bool appendToCurrent = false;
        if (!result.partitions.empty()) {
            const MachiningProgramPartition& current =
                result.partitions.back();
            appendToCurrent = current.setupId == key.setupId &&
                current.setupIndex == key.setupIndex &&
                current.setupFace == key.setupFace &&
                current.workOffset == key.workOffset &&
                current.toolControllerId == key.toolControllerId &&
                current.holePhaseApplies == key.holePhaseApplies &&
                current.holePhase == key.holePhase;
        }
        if (!appendToCurrent) {
            const SetupDefinition* setup = job.setupById(mapping.setupId);
            const JobToolController* controller =
                job.toolControllerById(mapping.toolControllerId);
            if (!setup || !controller) {
                addIssue(
                    result, IssueCode::PartitionInvariantViolation,
                    mapping.formalOperationId,
                    "assembled operation resources disappeared before partitioning");
                break;
            }
            MachiningProgramPartition partition;
            partition.index = result.partitions.size();
            partition.firstAssemblyOperation =
                mapping.firstProgramOperation;
            partition.setupId = mapping.setupId;
            partition.setupIndex = mapping.setupIndex;
            partition.setupFace = mapping.setupFace;
            partition.workOffset = mapping.workOffset;
            partition.setupInputFingerprint =
                setup->confirmation.confirmedInputFingerprint;
            partition.toolControllerId = mapping.toolControllerId;
            partition.toolContextFingerprint =
                toolpath::toolpathToolContextKey(controller->tool);
            partition.toolNumber = controller->tool.toolNumber;
            partition.toolId = controller->tool.toolId;
            partition.holePhaseApplies = key.holePhaseApplies;
            partition.holePhase = key.holePhase;
            result.partitions.push_back(std::move(partition));
        }

        MachiningProgramPartition& partition = result.partitions.back();
        partition.formalOperationIds.push_back(mapping.formalOperationId);
        partition.runtimeOperationIds.insert(
            partition.runtimeOperationIds.end(),
            mapping.runtimeOperationIds.begin(),
            mapping.runtimeOperationIds.end());
        const auto first = result.assembly.program.operations.begin() +
            static_cast<std::ptrdiff_t>(mapping.firstProgramOperation);
        const auto last = first +
            static_cast<std::ptrdiff_t>(mapping.programOperationCount);
        partition.program.operations.insert(
            partition.program.operations.end(), first, last);
        partition.assemblyOperationCount +=
            mapping.programOperationCount;
        nextAssemblyOperation += mapping.programOperationCount;
    }

    std::size_t partitionOperationCount = 0u;
    std::size_t partitionMoveCount = 0u;
    for (const MachiningProgramPartition& partition : result.partitions) {
        partitionOperationCount += partition.program.operations.size();
        partitionMoveCount += partition.program.moveCount();
    }
    if (nextAssemblyOperation != result.assembly.program.operations.size() ||
        partitionOperationCount != result.assembly.program.operations.size() ||
        partitionMoveCount != result.assembly.program.moveCount()) {
        addIssue(
            result, IssueCode::PartitionInvariantViolation, {},
            "partitioned program does not conserve assembled operations and moves");
    }
    if (!result.issues.empty()) {
        clearPublishedOutput(result);
        return result;
    }

    const ipw::IpwNode& finalNode = nodes.back();
    result.initialIpwStateKey = session.initialStateKey();
    result.finalIpwStateKey = finalNode.stateKey;
    result.finalIpwContentFingerprint =
        checkpoints.find(finalNode.stateKey)->contentFingerprint;
    result.ipwEngineFingerprint = session.engineFingerprint();
    return result;
}

} // namespace oap::manufacturing

// 项目级正式程序组装：验证 Job/runtime 身份并按全局 sequence 合并中立工序。
#include "manufacturing/program/MachiningProgramAssembler.h"

#include <algorithm>
#include <array>
#include <iterator>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "manufacturing/job/MachiningJobStorage.h"
#include "toolpath/core/ToolpathToolContext.h"

namespace oap::manufacturing {
namespace {

using IssueCode = MachiningProgramAssemblyIssueCode;

void addIssue(MachiningProgramAssemblyResult& result,
              IssueCode code,
              std::string operationId,
              std::string message)
{
    result.issues.push_back(
        {code, std::move(operationId), std::move(message)});
}

bool publishableState(JobOperationState state) noexcept
{
    return state == JobOperationState::Generated ||
        state == JobOperationState::Warning;
}

toolpath::ToolpathOperationType expectedOperationType(
    JobOperationKind kind) noexcept
{
    switch (kind) {
        case JobOperationKind::Hole:
            return toolpath::ToolpathOperationType::HoleMachining;
        case JobOperationKind::Pocket:
            return toolpath::ToolpathOperationType::PocketClearing;
        case JobOperationKind::Profile:
            return toolpath::ToolpathOperationType::ProfileContour;
        case JobOperationKind::SurfaceFinishing:
            return toolpath::ToolpathOperationType::SurfaceFinishing;
    }
    return toolpath::ToolpathOperationType::Generic;
}

std::size_t kindIndex(JobOperationKind kind) noexcept
{
    return static_cast<std::size_t>(kind);
}

std::string expectedExecutionFingerprint(
    const MachiningJobDefinition& definition,
    JobOperationKind kind)
{
    switch (kind) {
        case JobOperationKind::Hole:
            return MachiningJobStorage::holeExecutionFingerprint(
                definition).toStdString();
        case JobOperationKind::Pocket:
            return MachiningJobStorage::pocketExecutionFingerprint(
                definition).toStdString();
        case JobOperationKind::Profile:
            return MachiningJobStorage::profileExecutionFingerprint(
                definition).toStdString();
        case JobOperationKind::SurfaceFinishing:
            return MachiningJobStorage::finishingExecutionFingerprint(
                definition).toStdString();
    }
    return {};
}

const std::string& storedExecutionFingerprint(
    const MachiningJobRuntime& runtime,
    JobOperationKind kind) noexcept
{
    switch (kind) {
        case JobOperationKind::Hole:
            return runtime.holeExecutionFingerprint;
        case JobOperationKind::Pocket:
            return runtime.pocketExecutionFingerprint;
        case JobOperationKind::Profile:
            return runtime.profileExecutionFingerprint;
        case JobOperationKind::SurfaceFinishing:
            return runtime.finishingExecutionFingerprint;
    }
    return runtime.holeExecutionFingerprint;
}

bool hasRuntimeResult(const MachiningJob& job,
                      JobOperationKind kind) noexcept
{
    switch (kind) {
        case JobOperationKind::Hole: return job.hasHoleResult();
        case JobOperationKind::Pocket: return job.hasPocketResult();
        case JobOperationKind::Profile: return job.hasProfileResult();
        case JobOperationKind::SurfaceFinishing: return job.hasFinishingResult();
    }
    return false;
}

std::string firstOperationIdOfKind(
    const std::vector<JobOperationDescriptor>& operations,
    JobOperationKind kind)
{
    const auto found = std::find_if(
        operations.begin(), operations.end(),
        [kind](const JobOperationDescriptor& operation) {
            return operation.enabled && operation.kind == kind;
        });
    return found == operations.end() ? std::string{} : found->id;
}

bool sameTool(const toolpath::ToolpathToolContext& left,
              const toolpath::ToolpathToolContext& right)
{
    return left.valid && right.valid &&
        toolpath::toolpathToolContextKey(left) ==
            toolpath::toolpathToolContextKey(right);
}

} // namespace

const char* machiningProgramAssemblyIssueCodeId(
    MachiningProgramAssemblyIssueCode code) noexcept
{
    switch (code) {
        case IssueCode::InvalidJobDefinition:
            return "invalid_job_definition";
        case IssueCode::EmptyOperationSet:
            return "empty_operation_set";
        case IssueCode::InvalidOperationState:
            return "invalid_operation_state";
        case IssueCode::MissingSetup:
            return "missing_setup";
        case IssueCode::UnconfirmedSetup:
            return "unconfirmed_setup";
        case IssueCode::MissingToolController:
            return "missing_tool_controller";
        case IssueCode::MissingRuntimeResult:
            return "missing_runtime_result";
        case IssueCode::StaleRuntimeResult:
            return "stale_runtime_result";
        case IssueCode::RuntimeProgramRejected:
            return "runtime_program_rejected";
        case IssueCode::UnmappedRuntimeOperation:
            return "unmapped_runtime_operation";
        case IssueCode::DisabledRuntimeOperation:
            return "disabled_runtime_operation";
        case IssueCode::DuplicateRuntimeOperationId:
            return "duplicate_runtime_operation_id";
        case IssueCode::OperationKindMismatch:
            return "operation_kind_mismatch";
        case IssueCode::HolePhaseMismatch:
            return "hole_phase_mismatch";
        case IssueCode::SetupMismatch:
            return "setup_mismatch";
        case IssueCode::ToolMismatch:
            return "tool_mismatch";
        case IssueCode::EmptyRuntimeOperation:
            return "empty_runtime_operation";
        case IssueCode::MissingRuntimeOperation:
            return "missing_runtime_operation";
    }
    return "invalid_job_definition";
}

MachiningProgramAssemblyResult MachiningProgramAssembler::assemble(
    const MachiningJob& job,
    const std::vector<std::string>& includedOperationIds) const
{
    MachiningProgramAssemblyResult result;
    const MachiningJobDefinition& definition = job.definition();
    const std::vector<std::string> definitionErrors =
        job.validationErrors();
    for (const std::string& error : definitionErrors) {
        addIssue(result, IssueCode::InvalidJobDefinition, {}, error);
    }
    if (!result.issues.empty())
        return result;

    result.jobDefinitionFingerprint =
        MachiningJobStorage::definitionFingerprint(definition)
            .toStdString();
    const std::vector<JobOperationDescriptor> allOperations =
        MachiningOperationService::operations(definition);
    std::vector<JobOperationDescriptor> operations;
    std::copy_if(
        allOperations.begin(), allOperations.end(),
        std::back_inserter(operations),
        [&](const JobOperationDescriptor& operation) {
            return operation.enabled && (includedOperationIds.empty() ||
                std::find(includedOperationIds.begin(), includedOperationIds.end(), operation.id) != includedOperationIds.end());
        });
    for (const auto& id : includedOperationIds) {
        if (std::none_of(operations.begin(), operations.end(), [&](const auto& item) { return item.id == id; }))
            addIssue(result, IssueCode::MissingRuntimeOperation, id, "selected formal operation is missing or disabled");
    }
    if (!result.issues.empty()) return result;
    if (operations.empty()) {
        addIssue(
            result, IssueCode::EmptyOperationSet, {},
            "machining Job has no enabled formal operations");
        return result;
    }

    for (const JobOperationDescriptor& operation : operations) {
        if (!publishableState(operation.state)) {
            addIssue(
                result, IssueCode::InvalidOperationState, operation.id,
                "formal operation is not generated or warning-ready");
        }
        const SetupDefinition* setup = job.setupById(operation.setupId);
        if (!setup) {
            addIssue(
                result, IssueCode::MissingSetup, operation.id,
                "formal operation references a missing setup");
        } else {
            const std::string expectedConfirmation =
                MachiningJobStorage::setupInputFingerprint(
                    definition.part, *setup).toStdString();
            if (!setup->valid || !setup->confirmation.confirmed ||
                setup->confirmation.confirmedInputFingerprint !=
                    expectedConfirmation) {
                addIssue(
                    result, IssueCode::UnconfirmedSetup, operation.id,
                    "formal operation setup is invalid, unconfirmed, or stale");
            }
        }
        const JobToolController* controller =
            job.toolControllerById(operation.toolControllerId);
        if (!controller || !controller->tool.valid) {
            addIssue(
                result, IssueCode::MissingToolController, operation.id,
                "formal operation has no valid immutable tool controller");
        }
    }

    std::array<bool, 4> enabledKinds{};
    for (const JobOperationDescriptor& operation : operations)
        enabledKinds[kindIndex(operation.kind)] = true;
    for (std::size_t index = 0; index < enabledKinds.size(); ++index) {
        if (!enabledKinds[index])
            continue;
        const JobOperationKind kind = static_cast<JobOperationKind>(index);
        const std::string operationId =
            firstOperationIdOfKind(operations, kind);
        if (!hasRuntimeResult(job, kind)) {
            addIssue(
                result, IssueCode::MissingRuntimeResult, operationId,
                std::string("formal ") + jobOperationKindId(kind) +
                    " operations have no complete runtime result");
            continue;
        }
        const std::string expected =
            expectedExecutionFingerprint(definition, kind);
        const std::string& stored =
            storedExecutionFingerprint(job.runtime(), kind);
        if (stored.empty() || stored != expected) {
            addIssue(
                result, IssueCode::StaleRuntimeResult, operationId,
                std::string("formal ") + jobOperationKindId(kind) +
                    " runtime execution fingerprint is stale");
        }
    }
    if (!result.issues.empty()) {
        result.jobDefinitionFingerprint.clear();
        return result;
    }

    std::array<toolpath::ToolpathProgram, 4> domainPrograms;
    if (enabledKinds[kindIndex(JobOperationKind::Hole)])
        domainPrograms[kindIndex(JobOperationKind::Hole)] =
            job.holeResult()->toProgram();
    if (enabledKinds[kindIndex(JobOperationKind::Pocket)])
        domainPrograms[kindIndex(JobOperationKind::Pocket)] =
            job.pocketResult()->toProgram();
    if (enabledKinds[kindIndex(JobOperationKind::Profile)])
        domainPrograms[kindIndex(JobOperationKind::Profile)] =
            job.profileResult()->toProgram();
    if (enabledKinds[kindIndex(JobOperationKind::SurfaceFinishing)]) {
        std::string error;
        const auto* finishing = job.currentFinishingResult(&error);
        if (!finishing)
            addIssue(result, IssueCode::RuntimeProgramRejected,
                firstOperationIdOfKind(operations, JobOperationKind::SurfaceFinishing),
                std::move(error));
        else
            domainPrograms[kindIndex(JobOperationKind::SurfaceFinishing)] =
                finishing->toProgram();
    }
    if (!result.issues.empty()) {
        result.jobDefinitionFingerprint.clear();
        return result;
    }

    std::unordered_map<std::string, JobOperationDescriptor> enabledById;
    enabledById.reserve(operations.size());
    for (const JobOperationDescriptor& operation : operations)
        enabledById.emplace(operation.id, operation);
    std::unordered_map<std::string, std::vector<toolpath::ToolpathOperation>>
        runtimeByFormalId;
    runtimeByFormalId.reserve(operations.size());
    std::unordered_set<std::string> runtimeIds;

    for (std::size_t kindNumber = 0;
         kindNumber < domainPrograms.size(); ++kindNumber) {
        const JobOperationKind domainKind =
            static_cast<JobOperationKind>(kindNumber);
        for (toolpath::ToolpathOperation& runtimeOperation :
             domainPrograms[kindNumber].operations) {
            if (!includedOperationIds.empty()) {
                const auto selected = MachiningOperationService::operationForRuntimeId(definition, runtimeOperation.operationId);
                if (selected && !enabledById.contains(selected->id)) continue;
            }
            if (runtimeOperation.operationId.empty() ||
                !runtimeIds.insert(runtimeOperation.operationId).second) {
                addIssue(
                    result, IssueCode::DuplicateRuntimeOperationId,
                    runtimeOperation.operationId,
                    "runtime operation IDs must be non-empty and globally unique");
                continue;
            }
            const auto formal =
                MachiningOperationService::operationForRuntimeId(
                    definition, runtimeOperation.operationId);
            if (!formal) {
                addIssue(
                    result, IssueCode::UnmappedRuntimeOperation,
                    runtimeOperation.operationId,
                    "runtime operation does not map to a formal Job operation");
                continue;
            }
            const auto enabled = enabledById.find(formal->id);
            if (enabled == enabledById.end()) {
                addIssue(
                    result, IssueCode::DisabledRuntimeOperation,
                    runtimeOperation.operationId,
                    "runtime operation maps to a disabled formal operation");
                continue;
            }
            if (formal->kind != domainKind ||
                runtimeOperation.type != expectedOperationType(formal->kind)) {
                addIssue(
                    result, IssueCode::OperationKindMismatch,
                    runtimeOperation.operationId,
                    "runtime operation kind does not match its formal operation");
                continue;
            }
            if (formal->kind == JobOperationKind::Hole &&
                runtimeOperation.holePhase != formal->holePhase) {
                addIssue(
                    result, IssueCode::HolePhaseMismatch,
                    runtimeOperation.operationId,
                    "runtime hole phase does not match its formal operation");
                continue;
            }
            const SetupDefinition* setup = job.setupById(formal->setupId);
            if (!setup || runtimeOperation.setupIndex != setup->setupIndex ||
                (runtimeOperation.setupFace != MoldSetupFace::Unresolved &&
                 setup->face != MoldSetupFace::Unresolved &&
                 runtimeOperation.setupFace != setup->face)) {
                addIssue(
                    result, IssueCode::SetupMismatch,
                    runtimeOperation.operationId,
                    "runtime operation setup does not match the formal setup snapshot");
                continue;
            }
            const JobToolController* controller =
                job.toolControllerById(formal->toolControllerId);
            if (!controller ||
                !sameTool(runtimeOperation.tool, controller->tool)) {
                addIssue(
                    result, IssueCode::ToolMismatch,
                    runtimeOperation.operationId,
                    "runtime operation tool does not match the formal controller snapshot");
                continue;
            }
            if (runtimeOperation.moves.empty()) {
                addIssue(
                    result, IssueCode::EmptyRuntimeOperation,
                    runtimeOperation.operationId,
                    "runtime operation has no executable moves");
                continue;
            }
            runtimeByFormalId[formal->id].push_back(
                std::move(runtimeOperation));
        }
    }

    for (const JobOperationDescriptor& formal : operations) {
        const auto found = runtimeByFormalId.find(formal.id);
        if (found == runtimeByFormalId.end() || found->second.empty()) {
            addIssue(
                result, IssueCode::MissingRuntimeOperation, formal.id,
                "formal operation has no mapped runtime operations");
        }
    }
    if (!result.issues.empty()) {
        result.jobDefinitionFingerprint.clear();
        return result;
    }

    std::size_t operationCount = 0u;
    for (const auto& [formalId, runtime] : runtimeByFormalId) {
        (void)formalId;
        operationCount += runtime.size();
    }
    result.program.operations.reserve(operationCount);
    result.mappings.reserve(operations.size());
    for (const JobOperationDescriptor& formal : operations) {
        auto& runtime = runtimeByFormalId.at(formal.id);
        MachiningProgramOperationMapping mapping;
        mapping.formalOperationId = formal.id;
        mapping.kind = formal.kind;
        mapping.sequence = formal.sequence;
        mapping.setupId = formal.setupId;
        mapping.toolControllerId = formal.toolControllerId;
        mapping.inputFingerprint = formal.inputFingerprint;
        const SetupDefinition* setup = job.setupById(formal.setupId);
        mapping.setupIndex = setup->setupIndex;
        mapping.setupFace = setup->face;
        mapping.workOffset = setup->workOffset;
        mapping.holePhase = formal.holePhase;
        mapping.firstProgramOperation = result.program.operations.size();
        mapping.programOperationCount = runtime.size();
        mapping.runtimeOperationIds.reserve(runtime.size());
        for (toolpath::ToolpathOperation& operation : runtime) {
            mapping.runtimeOperationIds.push_back(operation.operationId);
            operation.metadata.insert_or_assign(
                "job_operation_id", formal.id);
            operation.metadata.insert_or_assign(
                "job_operation_kind", jobOperationKindId(formal.kind));
            operation.metadata.insert_or_assign(
                "job_sequence", std::to_string(formal.sequence));
            operation.metadata.insert_or_assign(
                "job_setup_id", formal.setupId);
            operation.metadata.insert_or_assign(
                "job_tool_controller_id", formal.toolControllerId);
            operation.metadata.insert_or_assign(
                "job_input_fingerprint", formal.inputFingerprint);
            if (formal.kind == JobOperationKind::Hole) {
                operation.metadata.insert_or_assign(
                    "job_hole_phase",
                    toolpath::holeMachiningPhaseName(formal.holePhase));
            }
            result.program.operations.push_back(std::move(operation));
        }
        result.mappings.push_back(std::move(mapping));
    }
    return result;
}

} // namespace oap::manufacturing

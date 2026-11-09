// Capture and atomically install hole Job definitions.
#include "manufacturing/hole/HoleMachiningJobService.h"
#include <utility>
#include <QDateTime>
#include "manufacturing/job/MachiningJobStorage.h"

#include "manufacturing/hole/detail/HoleJobTypes.h"
#include "manufacturing/hole/detail/HoleJobFoundation.h"
#include "manufacturing/hole/detail/HoleJobEnablement.h"
#include "manufacturing/hole/detail/HoleJobIdentity.h"
#include "manufacturing/hole/detail/HoleJobOperationOrder.h"
#include "manufacturing/hole/detail/HoleJobActiveIpw.h"

namespace oap::manufacturing {
using namespace hole_job_detail;

// 并原子安装到既有 Job（保留槽腔结果与可复用的 IPW 前缀）。

MachiningJob HoleMachiningJobService::capture(
    const HoleJobCaptureRequest& request) const
{
    // 先捕获旧孔工序的稳定身份，用于替换后恢复排序与跨捕获一致性。
    MachiningJob job;
    job.definition() = request.definition;
    MachiningJobDefinition& definition = job.definition();
    definition.schemaVersion = kMachiningJobSchemaVersion;
    const std::vector<StableHoleOperationIdentity>
        previousHoleIdentities = captureStableHoleIdentities(
            definition, definition.recognizedHoles);
    discardOldHoleResources(definition);
    if (!request.inputs || !request.generated || !request.automaticSelection)
        return job;
    const std::vector<toolpath::HoleMachiningInput>& inputs =
        *request.inputs;

    definition.holeParameters = request.generated->effectiveParams;
    definition.algorithmId = request.generated->algorithmId;
    definition.recognizedHoles = inputs;
    definition.recognizedPhysicalHoleCount =
        request.generated->coverage.physicalHoleCount;
    const std::string now =
        QDateTime::currentDateTimeUtc()
            .toString(Qt::ISODateWithMs).toStdString();
    if (definition.createdAt.empty())
        definition.createdAt = now;
    definition.modifiedAt = now;
    const JobOperationState state = generatedState(*request.generated);
    addAutomaticDefinitions(definition, request, state);
    // 恢复跨捕获稳定的工序身份与全局顺序，避免每次识别导致 ID/排序漂移。
    restoreStableHoleIdentityAndGlobalOrder(
        definition, previousHoleIdentities);
    for (HoleOperationDefinition& operation : definition.holeOperations) {
        const SetupDefinition* setup = job.setupById(operation.setupId);
        if (operation.state != JobOperationState::Error &&
            (!setup || !setup->valid ||
             !setup->confirmation.confirmed)) {
            operation.state = JobOperationState::NeedsReview;
        }
    }

    job.setHoleResult(
        bindCapturedRuntimeIdentities(definition, *request.generated),
        MachiningJobStorage::holeExecutionFingerprint(
            definition).toStdString());
    return job;
}

bool HoleMachiningJobService::capture(
    MachiningJob& job,
    const HoleJobCaptureRequest& request,
    std::string* error) const
{
    // 原子安装：先在副本上捕获并完整校验，全部通过后才替换既有 Job，
    // 失败时原 Job 保持原样，不会出现半更新状态。
    if (error)
        error->clear();
    if (!request.automaticSelection || !request.inputs || !request.generated) {
        if (error) *error = "hole capture requires an automatic tool-library plan and generated result";
        return false;
    }
    MachiningJob candidate = capture(request);
    if (!capturedRuntimeIdentitiesComplete(
            candidate.definition(),
            candidate.runtime().holeResult)) {
        if (error) {
            *error =
                "generated hole toolpath could not bind to stable Job operations";
        }
        return false;
    }
    const std::vector<std::string> errors = candidate.validationErrors();
    if (!errors.empty()) {
        if (error)
            *error = errors.front();
        return false;
    }

    const ActiveIpwPlan activeIpw = captureActiveIpwPlan(job);
    MachiningJob replacement = job;
    replacement.definition() = std::move(candidate.definition());
    replacement.setHoleResult(
        std::move(candidate.runtime().holeResult),
        MachiningJobStorage::holeExecutionFingerprint(
            replacement.definition()).toStdString());
    if (replacement.hasPocketResult()) {
        toolpath::PocketToolpathBatch retainedPocket =
            replacement.runtime().pocketResult;
        replacement.setPocketResult(
            std::move(retainedPocket),
            MachiningJobStorage::pocketExecutionFingerprint(
                replacement.definition()).toStdString());
    }
    // 新孔定义若无法重配置 IPW（如前缀被破坏）则整体放弃本次安装。
    if (!restoreActiveIpwPlan(replacement, activeIpw)) {
        if (error)
            *error = "updated hole definitions could not reconfigure IPW";
        return false;
    }
    job = std::move(replacement);
    return true;
}

} // namespace oap::manufacturing

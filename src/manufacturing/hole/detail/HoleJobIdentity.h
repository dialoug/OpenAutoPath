// Private identity stage contract.
#pragma once

#include <unordered_set>

#include "manufacturing/hole/detail/HoleJobTypes.h"

namespace oap::manufacturing::hole_job_detail {

MoldSetupFace operationSetupFace(
    const MachiningJobDefinition& definition,
    const HoleOperationDefinition& operation);

toolpath::HoleToolpathBatch bindCapturedRuntimeIdentities(
    const MachiningJobDefinition& definition,
    const toolpath::HoleToolpathBatch& generated);

bool capturedRuntimeIdentitiesComplete(
    const MachiningJobDefinition& definition,
    const toolpath::HoleToolpathBatch& runtime);

std::vector<std::string> operationStageKeys(
    const HoleOperationDefinition& operation,
    const std::vector<toolpath::HoleMachiningInput>& holes);

std::vector<StableHoleOperationIdentity> captureStableHoleIdentities(
    const MachiningJobDefinition& definition,
    const std::vector<toolpath::HoleMachiningInput>& holes);

std::size_t identityOverlap(
    const StableHoleOperationIdentity& first,
    const StableHoleOperationIdentity& second);

std::vector<int> matchStableHoleIdentities(
    const std::vector<StableHoleOperationIdentity>& previous,
    const std::vector<StableHoleOperationIdentity>& current);

std::optional<int> placementAnchor(
    const StableHoleOperationIdentity& current,
    const std::vector<StableHoleOperationIdentity>& previous);

std::string stableHoleOperationId(
    const StableHoleOperationIdentity& identity,
    const std::unordered_set<std::string>& used);

} // namespace oap::manufacturing::hole_job_detail

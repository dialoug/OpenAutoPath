// Private foundation stage contract.
#pragma once

#include "manufacturing/hole/detail/HoleJobTypes.h"

namespace oap::manufacturing::hole_job_detail {

std::string sha256(const std::string& value);

std::string featureId(const toolpath::HoleMachiningInput& hole);

std::string geometryFingerprint(const toolpath::HoleMachiningInput& hole);

std::string controllerId(const toolpath::ToolpathToolContext& tool,
                         std::size_t fallbackIndex,
                         const std::string& snapshotFingerprint = {});

std::string controllerKey(
    const toolpath::ToolpathToolContext& tool);


std::string ensureSetup(
    MachiningJobDefinition& definition,
    int preferredIndex,
    MoldSetupFace face,
    const toolpath::MoldSetupWorkFrames* frames);

void discardOldHoleResources(
    MachiningJobDefinition& definition);

HoleTargetReference makeTarget(
    const toolpath::HoleMachiningInput& hole, int sourceIndex, int physicalIndex,
    toolpath::HoleMachiningStrategy strategy,
    const std::vector<std::size_t>& coveredSourceIndices = {});

std::string operationFingerprint(
    const MachiningJobDefinition& definition,
    const HoleOperationDefinition& operation);

} // namespace oap::manufacturing::hole_job_detail

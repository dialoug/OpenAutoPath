// 加工 Job 工序管理事务实现，见 MachiningOperationService.h。
#include "manufacturing/job/MachiningOperationService.h"

#include <algorithm>
#include <functional>
#include <type_traits>
#include <iomanip>
#include <iterator>
#include <locale>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <QCryptographicHash>
#include <QDateTime>

#include "manufacturing/job/MachiningJobStorage.h"

namespace oap::manufacturing {
namespace {

template <class Operation>
void appendDescriptors(
    std::vector<JobOperationDescriptor>& output,
    const std::vector<Operation>& operations,
    JobOperationKind kind,
    const std::function<std::string(const Operation&)>& algorithm)
{
    for (const Operation& operation : operations) {
        JobOperationDescriptor descriptor;
        descriptor.id = operation.id;
        descriptor.kind = kind;
        descriptor.setupId = operation.setupId;
        descriptor.toolControllerId = operation.toolControllerId;
        descriptor.algorithmId = algorithm(operation);
        descriptor.inputFingerprint = operation.inputFingerprint;
        if constexpr (std::is_same_v<Operation, HoleOperationDefinition>)
            descriptor.holePhase = operation.phase;
        descriptor.sequence = operation.sequence;
        descriptor.enabled = operation.enabled;
        descriptor.state = operation.state;
        output.push_back(std::move(descriptor));
    }
}

bool matchesRuntimeId(
    const std::string& formalId,
    const std::string& runtimeId)
{
    if (runtimeId == formalId)
        return true;
    if (!runtimeId.starts_with(formalId) ||
        runtimeId.size() <= formalId.size() + 1u ||
        runtimeId[formalId.size()] != '-') {
        return false;
    }
    std::size_t digit = formalId.size() + 1u;
    if (runtimeId[digit] == 'r')
        ++digit;
    return digit < runtimeId.size() &&
        std::all_of(
            runtimeId.begin() + static_cast<std::ptrdiff_t>(digit),
            runtimeId.end(),
            [](char value) { return value >= '0' && value <= '9'; });
}

} // namespace

const char* jobOperationKindId(JobOperationKind kind) noexcept
{
    switch (kind) {
        case JobOperationKind::Hole: return "hole";
        case JobOperationKind::Pocket: return "pocket";
        case JobOperationKind::Profile: return "profile";
        case JobOperationKind::SurfaceFinishing: return "surface_finishing";
    }
    return "unknown";
}

std::vector<JobOperationDescriptor> MachiningOperationService::operations(
    const MachiningJobDefinition& definition)
{
    std::vector<JobOperationDescriptor> result;
    result.reserve(
        definition.holeOperations.size() +
        definition.pocketOperations.size() +
        definition.profileOperations.size());
    appendDescriptors<HoleOperationDefinition>(
        result, definition.holeOperations, JobOperationKind::Hole,
        [&definition](const HoleOperationDefinition&) {
            return definition.algorithmId;
        });
    appendDescriptors<PocketOperationDefinition>(
        result, definition.pocketOperations, JobOperationKind::Pocket,
        [](const PocketOperationDefinition& operation) {
            return operation.algorithmId;
        });
    appendDescriptors<ProfileOperationDefinition>(
        result, definition.profileOperations, JobOperationKind::Profile,
        [](const ProfileOperationDefinition& operation) {
            return operation.algorithmId;
        });
    appendDescriptors<SurfaceFinishingOperationDefinition>(
        result, definition.finishingOperations, JobOperationKind::SurfaceFinishing,
        [](const SurfaceFinishingOperationDefinition& operation) { return operation.algorithmId; });
    std::stable_sort(
        result.begin(), result.end(),
        [](const JobOperationDescriptor& left,
           const JobOperationDescriptor& right) {
            if (left.sequence != right.sequence)
                return left.sequence < right.sequence;
            return left.id < right.id;
        });
    return result;
}

std::optional<JobOperationDescriptor>
MachiningOperationService::operationForRuntimeId(
    const MachiningJobDefinition& definition,
    const std::string& runtimeOperationId)
{
    std::optional<JobOperationDescriptor> result;
    for (const JobOperationDescriptor& descriptor : operations(definition)) {
        if (matchesRuntimeId(descriptor.id, runtimeOperationId) &&
            (!result || descriptor.id.size() > result->id.size())) {
            result = descriptor;
        }
    }
    return result;
}

bool MachiningOperationService::runtimeOperationIdMatches(
    const std::string& formalOperationId,
    const std::string& runtimeOperationId)
{
    return matchesRuntimeId(formalOperationId, runtimeOperationId);
}

std::string MachiningOperationService::formalOperationIdForRuntime(
    const MachiningJobDefinition& definition,
    const std::string& runtimeOperationId)
{
    const std::optional<JobOperationDescriptor> operation =
        operationForRuntimeId(definition, runtimeOperationId);
    return operation ? operation->id : std::string{};
}

} // namespace oap::manufacturing

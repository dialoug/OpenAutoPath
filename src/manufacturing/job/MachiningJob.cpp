// MachiningJob 聚合根与 MachiningJobDefinition 的默认实现（oap_manufacturing）。
// 包含：OCCT 几何与枚举的转换、孔/槽腔刀路结果的设置与清理、IPW 计划配置，
// 以及可序列化定义的结构校验（validationErrors，不验证刀路/机床安全）。
// 由 MachiningJobStorage / MachiningJobApplicationService 共同消费，无 UI 依赖。
#include "manufacturing/job/MachiningJob.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "manufacturing/job/detail/SurfaceFinishingJson.h"
#include "manufacturing/finishing/SurfaceFinishingMachiningJobService.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_set>

namespace oap::manufacturing {

// 几何/枚举与可持久化表示之间的转换：JobVector3 用于 JSON 与指纹计算。
bool runtimeOperationIdMatches(const std::string& definitionId,
                               const std::string& runtimeId)
{
    if (runtimeId == definitionId)
        return true;
    if (runtimeId.size() <= definitionId.size() + 1u ||
        !runtimeId.starts_with(definitionId) ||
        runtimeId[definitionId.size()] != '-') {
        return false;
    }
    std::size_t firstDigit = definitionId.size() + 1u;
    if (runtimeId.size() > firstDigit + 1u &&
        runtimeId[firstDigit] == 'r') {
        ++firstDigit;
    }
    return firstDigit < runtimeId.size() && std::all_of(
        runtimeId.begin() + static_cast<std::ptrdiff_t>(firstDigit),
        runtimeId.end(),
        [](char value) { return value >= '0' && value <= '9'; });
}

JobVector3 toJobVector(const gp_Pnt& point) noexcept
{
    return {point.X(), point.Y(), point.Z()};
}

JobVector3 toJobVector(const gp_Dir& direction) noexcept
{
    return {direction.X(), direction.Y(), direction.Z()};
}

gp_Pnt toPoint(const JobVector3& value) noexcept
{
    return {value.x, value.y, value.z};
}

const char* stockSourceKindId(StockSourceKind kind) noexcept
{
    switch (kind) {
        case StockSourceKind::PerUnitBoxes: return "per_unit_boxes";
        case StockSourceKind::ModelBox: return "model_box";
        case StockSourceKind::StepModel: return "step_model";
    }
    return "per_unit_boxes";
}

const char* jobOperationStateId(JobOperationState state) noexcept
{
    switch (state) {
        case JobOperationState::Planned: return "planned";
        case JobOperationState::Dirty: return "dirty";
        case JobOperationState::Generated: return "generated";
        case JobOperationState::Warning: return "warning";
        case JobOperationState::Error: return "error";
        case JobOperationState::Stale: return "stale";
        case JobOperationState::NeedsReview: return "needs_review";
    }
    return "planned";
}

bool MachiningJob::hasHoleResult() const noexcept
{
    return m_runtime.hasHoleResult;
}

const toolpath::HoleToolpathBatch* MachiningJob::holeResult() const noexcept
{
    return hasHoleResult() ? &m_runtime.holeResult : nullptr;
}

toolpath::HoleToolpathBatch* MachiningJob::holeResult() noexcept
{
    return hasHoleResult() ? &m_runtime.holeResult : nullptr;
}

void MachiningJob::setHoleResult(
    toolpath::HoleToolpathBatch result,
    std::string executionFingerprint)
{
    m_runtime.holeResult = std::move(result);
    m_runtime.holeExecutionFingerprint =
        std::move(executionFingerprint);
    // 空操作批次只有在覆盖审计完整时才算有效结果。
    m_runtime.hasHoleResult =
        !m_runtime.holeResult.machining.operations.empty() ||
        m_runtime.holeResult.coverage.complete();
}

void MachiningJob::clearHoleResult()
{
    m_runtime.holeResult = {};
    m_runtime.holeExecutionFingerprint.clear();
    m_runtime.hasHoleResult = false;
}

bool MachiningJob::hasPocketResult() const noexcept
{
    return m_runtime.hasPocketResult &&
           !m_runtime.pocketResult.operations.empty();
}

const toolpath::PocketToolpathBatch*
MachiningJob::pocketResult() const noexcept
{
    return hasPocketResult() ? &m_runtime.pocketResult : nullptr;
}

toolpath::PocketToolpathBatch*
MachiningJob::pocketResult() noexcept
{
    return hasPocketResult() ? &m_runtime.pocketResult : nullptr;
}

void MachiningJob::setPocketResult(
    toolpath::PocketToolpathBatch result,
    std::string executionFingerprint)
{
    m_runtime.pocketResult = std::move(result);
    m_runtime.pocketExecutionFingerprint =
        std::move(executionFingerprint);
    m_runtime.hasPocketResult =
        !m_runtime.pocketResult.operations.empty();
}

const toolpath::PocketToolpathBatch* MachiningJob::currentPocketResult(
    std::string* error) const
{
    if (error) error->clear();
    const auto reject = [error](const char* message) {
        if (error) *error = message;
        return static_cast<const toolpath::PocketToolpathBatch*>(nullptr);
    };
    const auto* batch = pocketResult();
    if (!batch)
        return reject("Pocket result is missing; regenerate the toolpath.");
    if (batch->cancelled)
        return reject("Pocket generation was cancelled.");
    if (!batch->validation.ok() || !batch->safetyValidation.ok())
        return reject("Pocket result has validation errors.");
    if (m_runtime.pocketExecutionFingerprint.empty() ||
        m_runtime.pocketExecutionFingerprint !=
            MachiningJobStorage::pocketExecutionFingerprint(m_definition).toStdString())
        return reject("Pocket result is stale; regenerate the toolpath.");
    return batch;
}

void MachiningJob::clearPocketResult()
{
    m_runtime.pocketResult = {};
    m_runtime.pocketExecutionFingerprint.clear();
    m_runtime.hasPocketResult = false;
}

bool MachiningJob::hasProfileResult() const noexcept
{
    return m_runtime.hasProfileResult &&
           !m_runtime.profileResult.operations.empty();
}

const toolpath::ProfileToolpathBatch*
MachiningJob::profileResult() const noexcept
{
    return hasProfileResult() ? &m_runtime.profileResult : nullptr;
}

toolpath::ProfileToolpathBatch* MachiningJob::profileResult() noexcept
{
    return hasProfileResult() ? &m_runtime.profileResult : nullptr;
}

void MachiningJob::setProfileResult(
    toolpath::ProfileToolpathBatch result,
    std::string executionFingerprint)
{
    m_runtime.profileResult = std::move(result);
    m_runtime.profileExecutionFingerprint =
        std::move(executionFingerprint);
    m_runtime.hasProfileResult =
        !m_runtime.profileResult.operations.empty();
}

void MachiningJob::clearProfileResult()
{
    m_runtime.profileResult = {};
    m_runtime.profileExecutionFingerprint.clear();
    m_runtime.hasProfileResult = false;
}

bool MachiningJob::hasFinishingResult() const noexcept
{
    return m_runtime.hasFinishingResult && !m_runtime.finishingResult.operations.empty();
}
const toolpath::SurfaceFinishingBatch* MachiningJob::finishingResult() const noexcept
{
    return hasFinishingResult() ? &m_runtime.finishingResult : nullptr;
}
toolpath::SurfaceFinishingBatch* MachiningJob::finishingResult() noexcept
{
    return hasFinishingResult() ? &m_runtime.finishingResult : nullptr;
}
void MachiningJob::setFinishingResult(toolpath::SurfaceFinishingBatch result, std::string fingerprint)
{
    m_runtime.finishingResult = std::move(result);
    m_runtime.finishingExecutionFingerprint = std::move(fingerprint);
    m_runtime.hasFinishingResult = !m_runtime.finishingResult.cancelled &&
        m_runtime.finishingResult.validation.ok() && !m_runtime.finishingResult.operations.empty();
}
void MachiningJob::clearFinishingResult()
{
    m_runtime.finishingResult = {};
    m_runtime.finishingExecutionFingerprint.clear();
    m_runtime.hasFinishingResult = false;
}
const toolpath::SurfaceFinishingBatch* MachiningJob::currentFinishingResult(std::string* error) const
{
    if (error) error->clear();
    const auto reject = [error](const char* message) -> const toolpath::SurfaceFinishingBatch* {
        if (error) *error = message;
        return nullptr;
    };
    const auto sameDirection = [](const gp_Dir& axis, const JobVector3& value) {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z) &&
            std::abs(axis.X() - value.x) <= 1e-9 && std::abs(axis.Y() - value.y) <= 1e-9 && std::abs(axis.Z() - value.z) <= 1e-9;
    };
    const auto* result = finishingResult();
    if (!result || result->cancelled || !result->validation.ok() ||
        m_runtime.finishingExecutionFingerprint.empty() ||
        m_runtime.finishingExecutionFingerprint != MachiningJobStorage::finishingExecutionFingerprint(m_definition).toStdString()) {
        return reject("Engraving result is missing, invalid or stale; regenerate it.");
    }
    std::unordered_set<std::string> matched;
    for (const auto& operation : result->operations) {
        const auto found = std::find_if(m_definition.finishingOperations.begin(), m_definition.finishingOperations.end(),
            [&](const auto& source) { return source.id == operation.operationId && source.enabled; });
        if (found == m_definition.finishingOperations.end() || !matched.insert(operation.operationId).second ||
            !operation.result.ok || operation.result.cancelled || operation.result.moves.empty() || operation.result.primitives.empty())
            return reject("Engraving result contains a missing, duplicate or failed operation.");
        const auto* setup = setupById(found->setupId);
        const auto* controller = toolControllerById(found->toolControllerId);
        if (!setup || !setup->valid || !controller || !operation.tool.valid ||
            operation.setupFace != setup->face || operation.setupIndex != setup->setupIndex ||
            operation.workFrame.Location().Distance(toPoint(setup->origin)) > 1e-7 ||
            !sameDirection(operation.workFrame.XDirection(), setup->xAxis) ||
            !sameDirection(operation.workFrame.Direction(), setup->zAxis) ||
            toolpath::toolpathToolContextKey(operation.tool) != toolpath::toolpathToolContextKey(controller->tool))
            return reject("Engraving result no longer matches its setup or tool snapshot.");
        auto generatedDefinition = *found;
        generatedDefinition.parameters = operation.result.effectiveParams;
        if (SurfaceFinishingMachiningJobService::operationFingerprint(m_definition, generatedDefinition) !=
            SurfaceFinishingMachiningJobService::operationFingerprint(m_definition, *found))
            return reject("Engraving result no longer matches its operation parameters.");
    }
    for (const auto& operation : m_definition.finishingOperations)
        if (operation.enabled && !matched.contains(operation.id))
            return reject("Engraving result is incomplete; regenerate all enabled engraving operations.");
    return result;
}

ipw::IpwPlanUpdate MachiningJob::configureIpwPlan(
    std::string initialStockFingerprint,
    std::string engineFingerprint,
    const std::vector<std::string>& includedOperationIds)
{
    // 把启用的持久化工序（含指纹与全局 sequence）翻译成 IPW 操作定义；
    // 若调用方显式指定工序集合，则任何缺失/被禁用的工序都使配置整体失败。
    const std::unordered_set<std::string> included(
        includedOperationIds.begin(),
        includedOperationIds.end());
    const auto selected = [&included](
                              const std::string& operationId) {
        return included.empty() ||
               included.contains(operationId);
    };
    std::unordered_set<std::string> matched;
    std::vector<ipw::IpwOperationDefinition> operations;
    operations.reserve(m_definition.holeOperations.size() +
                       m_definition.pocketOperations.size() +
                       m_definition.profileOperations.size());
    for (const HoleOperationDefinition& source :
         m_definition.holeOperations) {
        if (!source.enabled || !selected(source.id))
            continue;
        matched.insert(source.id);
        ipw::IpwOperationDefinition operation;
        operation.id = source.id;
        operation.setupId = source.setupId;
        operation.toolControllerId =
            source.toolControllerId;
        operation.inputFingerprint =
            source.inputFingerprint;
        operation.sequence = source.sequence;
        operation.kind = ipw::IpwOperationKind::Hole;
        operations.push_back(std::move(operation));
    }
    for (const PocketOperationDefinition& source :
         m_definition.pocketOperations) {
        if (!source.enabled || !selected(source.id))
            continue;
        matched.insert(source.id);
        ipw::IpwOperationDefinition operation;
        operation.id = source.id;
        operation.setupId = source.setupId;
        operation.toolControllerId =
            source.toolControllerId;
        operation.inputFingerprint =
            source.inputFingerprint;
        operation.sequence = source.sequence;
        operation.kind = ipw::IpwOperationKind::Pocket;
        operations.push_back(std::move(operation));
    }
    for (const ProfileOperationDefinition& source :
         m_definition.profileOperations) {
        if (!source.enabled || !selected(source.id))
            continue;
        matched.insert(source.id);
        ipw::IpwOperationDefinition operation;
        operation.id = source.id;
        operation.setupId = source.setupId;
        operation.toolControllerId = source.toolControllerId;
        operation.inputFingerprint = source.inputFingerprint;
        operation.sequence = source.sequence;
        operation.kind = ipw::IpwOperationKind::Profile;
        operations.push_back(std::move(operation));
    }
    for (const auto& source : m_definition.finishingOperations) {
        if (!source.enabled || !selected(source.id)) continue;
        matched.insert(source.id);
        ipw::IpwOperationDefinition operation;
        operation.id = source.id;
        operation.setupId = source.setupId;
        operation.toolControllerId = source.toolControllerId;
        operation.inputFingerprint = SurfaceFinishingMachiningJobService::operationFingerprint(m_definition, source);
        operation.sequence = source.sequence;
        operation.kind = ipw::IpwOperationKind::SurfaceFinishing;
        operations.push_back(std::move(operation));
    }
    if (!included.empty() &&
        matched.size() != included.size()) {
        ipw::IpwPlanUpdate rejected;
        const auto missing = std::find_if(
            includedOperationIds.begin(),
            includedOperationIds.end(),
            [&matched](const std::string& operationId) {
                return !matched.contains(operationId);
            });
        rejected.error =
            "selected IPW operation is missing or disabled";
        if (missing != includedOperationIds.end())
            rejected.error += ": " + *missing;
        return rejected;
    }
    ipw::IpwPlanUpdate update = m_runtime.ipwSession.configure(
        std::move(initialStockFingerprint),
        std::move(engineFingerprint),
        std::move(operations));
    if (!update.ok())
        return update;

    // configure 已保留相同连续前缀的节点状态；同步只保留仍可到达的根和
    // Ready 前缀，立即丢弃首个变化节点及其旧后缀检查点。
    std::vector<std::string> retainedStateKeys;
    retainedStateKeys.push_back(m_runtime.ipwSession.initialStateKey());
    for (const ipw::IpwNode& node : m_runtime.ipwSession.nodes()) {
        if (node.status != ipw::IpwNodeStatus::Ready)
            break;
        retainedStateKeys.push_back(node.stateKey);
    }
    m_runtime.ipwCheckpoints.retain(retainedStateKeys);
    return update;
}

void MachiningJob::invalidateRuntime()
{
    m_runtime = {};
}

void MachiningJob::clear()
{
    m_definition = {};
    m_runtime = {};
}

const SetupDefinition* MachiningJob::setupById(
    const std::string& id) const noexcept
{
    const auto found = std::find_if(
        m_definition.setups.begin(), m_definition.setups.end(),
        [&id](const SetupDefinition& setup) { return setup.id == id; });
    return found == m_definition.setups.end() ? nullptr : &*found;
}

const JobToolController* MachiningJob::toolControllerById(
    const std::string& id) const noexcept
{
    const auto found = std::find_if(
        m_definition.toolControllers.begin(),
        m_definition.toolControllers.end(),
        [&id](const JobToolController& controller) {
            return controller.id == id;
        });
    return found == m_definition.toolControllers.end() ? nullptr : &*found;
}

namespace {

// ---- 以下辅助谓词供 validationErrors() 使用，只检查有限性与枚举取值范围。 ----

bool finiteVector(const JobVector3& value) noexcept
{
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

double lengthSquared(const JobVector3& value) noexcept
{
    return value.x * value.x + value.y * value.y + value.z * value.z;
}

bool validSetupFace(MoldSetupFace face) noexcept
{
    const int value = static_cast<int>(face);
    return value >= static_cast<int>(MoldSetupFace::PositiveZ) &&
           value <= static_cast<int>(MoldSetupFace::Unresolved);
}

bool validOperationState(JobOperationState state) noexcept
{
    const int value = static_cast<int>(state);
    return value >= static_cast<int>(JobOperationState::Planned) &&
           value <= static_cast<int>(JobOperationState::NeedsReview);
}

template <typename Enum>
bool enumInRange(Enum value, Enum first, Enum last) noexcept
{
    const int number = static_cast<int>(value);
    return number >= static_cast<int>(first) &&
           number <= static_cast<int>(last);
}

bool validHole(const toolpath::HoleMachiningInput& hole) noexcept
{
    return std::isfinite(hole.entry.X()) &&
           std::isfinite(hole.entry.Y()) &&
           std::isfinite(hole.entry.Z()) &&
           std::isfinite(hole.bottom.X()) &&
           std::isfinite(hole.bottom.Y()) &&
           std::isfinite(hole.bottom.Z()) &&
           std::isfinite(hole.radius) && hole.radius > 0.0 &&
           enumInRange(hole.kind,
                       FeatureKind::ThroughHole,
                       FeatureKind::CountersinkHole) &&
           enumInRange(hole.stageRole,
                       HoleStageRole::Standard,
                       HoleStageRole::CounterboreFloorChild) &&
           hole.entry.Distance(hole.bottom) > 1.0e-9;
}

// 校验刀具上下文：所有数值非负有限、直径>0、枚举值在范围内。
bool validToolContext(const toolpath::ToolpathToolContext& tool) noexcept
{
    const std::array<double, 16> values{
        tool.diameter,
        tool.spindleSpeed,
        tool.horizontalFeed,
        tool.verticalFeed,
        tool.horizontalRapid,
        tool.verticalRapid,
        tool.cuttingEdgeLength,
        tool.overallLength,
        tool.shankDiameter,
        tool.holderDiameter,
        tool.holderLength,
        tool.spindleEnvelopeDiameter,
        tool.spindleEnvelopeLength,
        tool.tipAngle,
        tool.tipCompensation,
        tool.peckDepth,
    };
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value) && value >= 0.0;
    }) && tool.diameter > 0.0 &&
        tool.valid && tool.toolNumber > 0 &&
        enumInRange(tool.drillCycle,
                    toolpath::ToolpathDrillCyclePreference::Auto,
                    toolpath::ToolpathDrillCyclePreference::ChipBreak) &&
        enumInRange(tool.coolant,
                    toolpath::ToolpathCoolantMode::Off,
                    toolpath::ToolpathCoolantMode::ThroughTool);
}

bool hasEmptyId(const std::vector<std::string>& ids) noexcept
{
    return std::any_of(
        ids.begin(), ids.end(),
        [](const std::string& id) { return id.empty(); });
}



} // namespace

std::vector<std::string> MachiningJob::validationErrors() const
{
    // 只做可序列化结构与引用完整性的领域校验，不取代刀路/机床安全验证：
    // 依次检查 schema、零件/毛坯/碰撞、Setup 唯一性、刀具控制器、
    // 各类工序的引用与指纹。
    std::vector<std::string> errors;
    if (m_definition.schemaVersion != kMachiningJobSchemaVersion)
        errors.emplace_back("unsupported machining job schema version");
    if (m_definition.id.empty())
        errors.emplace_back("job id is empty");
    if (m_definition.part.sourcePath.empty())
        errors.emplace_back("part source path is empty");
    if (!std::isfinite(m_definition.stock.marginX) ||
        !std::isfinite(m_definition.stock.marginY) ||
        !std::isfinite(m_definition.stock.marginZ) ||
        m_definition.stock.marginX < 0.0 ||
        m_definition.stock.marginY < 0.0 ||
        m_definition.stock.marginZ < 0.0) {
        errors.emplace_back("stock margins must be finite and non-negative");
    }
    if (!enumInRange(m_definition.stock.sourceKind,
                     StockSourceKind::PerUnitBoxes,
                     StockSourceKind::StepModel)) {
        errors.emplace_back("stock source kind is invalid");
    }
    if (m_definition.stock.sourceKind == StockSourceKind::StepModel &&
        m_definition.stock.sourcePath.empty()) {
        errors.emplace_back("STEP stock source path is empty");
    }
    for (const CollisionModelReference& model :
         m_definition.collisionScene.models) {
        const int role = static_cast<int>(model.role);
        if (model.sourcePath.empty() ||
            !finiteVector(model.translation) ||
            !finiteVector(model.rotationDeg) ||
            role < static_cast<int>(toolpath::CollisionBodyRole::Fixture) ||
            role > static_cast<int>(toolpath::CollisionBodyRole::Machine)) {
            errors.emplace_back("collision model reference is invalid");
        }
    if (m_definition.collisionScene.confirmed &&
            model.sourceFingerprint.empty()) {
            errors.emplace_back(
                "confirmed collision model is missing its source fingerprint");
        }
    }
    if (m_definition.recognizedPhysicalHoleCount < 0 ||
        (!m_definition.recognizedHoles.empty() &&
         (m_definition.recognizedPhysicalHoleCount <= 0 ||
          m_definition.recognizedPhysicalHoleCount >
              static_cast<int>(m_definition.recognizedHoles.size())))) {
        errors.emplace_back("recognized physical-hole count is invalid");
    }
    for (const toolpath::HoleMachiningInput& hole : m_definition.recognizedHoles) {
        if (!validHole(hole)) {
            errors.emplace_back("recognized hole geometry is invalid");
            break;
        }
    }

    std::unordered_set<std::string> setupIds;
    std::unordered_set<int> setupIndices;
    for (const SetupDefinition& setup : m_definition.setups) {
        if (setup.id.empty() || !setupIds.insert(setup.id).second) {
            errors.emplace_back("setup ids must be non-empty and unique");
            continue;
        }
        if (!finiteVector(setup.origin) || !finiteVector(setup.xAxis) ||
            !finiteVector(setup.yAxis) || !finiteVector(setup.zAxis)) {
            errors.emplace_back("setup coordinates must be finite: " + setup.id);
        }
        if (setup.setupIndex < 0 || setup.setupIndex >= 6) {
            errors.emplace_back("setup index is outside the six-face range: " +
                                setup.id);
        } else if (!setupIndices.insert(setup.setupIndex).second) {
            errors.emplace_back("setup indices must be unique: " + setup.id);
        }
        if (!validSetupFace(setup.face) ||
            !enumInRange(setup.workOffset,
                         toolpath::WorkCoordinateSystem::None,
                         toolpath::WorkCoordinateSystem::G59) ||
            lengthSquared(setup.xAxis) < 1.0e-12 ||
            lengthSquared(setup.yAxis) < 1.0e-12 ||
            lengthSquared(setup.zAxis) < 1.0e-12) {
            errors.emplace_back("setup axes or face are invalid: " + setup.id);
        }
    }

    std::unordered_set<std::string> controllerIds;
    for (const JobToolController& controller :
         m_definition.toolControllers) {
        if (controller.id.empty() ||
            !controllerIds.insert(controller.id).second) {
            errors.emplace_back(
                "tool controller ids must be non-empty and unique");
        }
        if (!validToolContext(controller.tool)) {
            errors.emplace_back(
                "tool controller parameters are invalid: " + controller.id);
        }
    }

    const std::size_t recognizedHoleCount =
        m_definition.recognizedHoles.size();
    const std::size_t recognizedPhysicalHoleCount =
        static_cast<std::size_t>(
            std::max(0, m_definition.recognizedPhysicalHoleCount));
    std::unordered_set<std::string> operationIds;
    std::unordered_set<int> operationSequences;
    for (const HoleOperationDefinition& operation :
         m_definition.holeOperations) {
        if (operation.id.empty() ||
            !operationIds.insert(operation.id).second) {
            errors.emplace_back(
                "operation ids must be non-empty and globally unique");
        }
        if (!setupIds.contains(operation.setupId)) {
            errors.emplace_back(
                "hole operation references a missing setup: " + operation.id);
        }
        if (!controllerIds.contains(operation.toolControllerId)) {
            errors.emplace_back(
                "hole operation references a missing tool controller: " +
                operation.id);
        }
        if (operation.targets.empty()) {
            errors.emplace_back("hole operation has no targets: " + operation.id);
        }
        if (operation.sequence < 0 ||
            !operationSequences.insert(operation.sequence).second) {
            errors.emplace_back(
                "operation sequences must be non-negative and globally unique: " +
                operation.id);
        }
        if (!validOperationState(operation.state)) {
            errors.emplace_back("hole operation state is invalid: " + operation.id);
        }
        if (operation.inputFingerprint.empty()) {
            errors.emplace_back(
                "hole operation input fingerprint is empty: " +
                operation.id);
        }
        if (!enumInRange(operation.phase,
                         toolpath::HoleMachiningPhase::Standard,
                         toolpath::HoleMachiningPhase::FlatEndMill)) {
            errors.emplace_back("hole operation phase is invalid: " + operation.id);
        }
        for (const HoleTargetReference& target : operation.targets) {
            const bool invalidCoveredSource = std::any_of(
                target.coveredSourceHoleIndices.begin(),
                target.coveredSourceHoleIndices.end(),
                [recognizedHoleCount](int sourceIndex) {
                    return sourceIndex < 0 ||
                           static_cast<std::size_t>(sourceIndex) >=
                               recognizedHoleCount;
                });
            if (target.featureId.empty() ||
                target.geometryFingerprint.empty() ||
                target.sourceHoleIndex < 0 ||
                static_cast<std::size_t>(target.sourceHoleIndex) >=
                    recognizedHoleCount ||
                target.physicalHoleIndex < 0 ||
                static_cast<std::size_t>(target.physicalHoleIndex) >=
                    recognizedPhysicalHoleCount || invalidCoveredSource ||
                !validHole(target.hole) ||
                !enumInRange(target.strategy,
                             toolpath::HoleMachiningStrategy::Auto,
                             toolpath::HoleMachiningStrategy::LargeLayeredSpiral)) {
                errors.emplace_back(
                    "hole operation has an invalid target: " + operation.id);
            }
        }
    }
    for (const PocketOperationDefinition& operation :
         m_definition.pocketOperations) {
        if (operation.id.empty() ||
            !operationIds.insert(operation.id).second) {
            errors.emplace_back(
                "operation ids must be non-empty and globally unique");
        }
        if (!setupIds.contains(operation.setupId)) {
            errors.emplace_back(
                "pocket operation references a missing setup: " +
                operation.id);
        }
        if (!controllerIds.contains(operation.toolControllerId)) {
            errors.emplace_back(
                "pocket operation references a missing tool controller: " +
                operation.id);
        }
        if (operation.algorithmId.empty()) {
            errors.emplace_back(
                "pocket operation algorithm id is empty: " + operation.id);
        }
        if (!std::isfinite(
                operation.restMachiningPreviousToolDiameter) ||
            operation.restMachiningPreviousToolDiameter < 0.0) {
            errors.emplace_back(
                "pocket operation rest-machining diameter is invalid: " +
                operation.id);
        }
        if (operation.sequence < 0 ||
            !operationSequences.insert(operation.sequence).second) {
            errors.emplace_back(
                "operation sequences must be non-negative and globally unique: " +
                operation.id);
        }
        if (!validOperationState(operation.state)) {
            errors.emplace_back(
                "pocket operation state is invalid: " + operation.id);
        }
        const bool validKind =
            operation.target.kind == FeatureKind::Slot ||
            operation.target.kind == FeatureKind::Pocket;
        const bool validSourceKind =
            operation.target.sourceKind ==
                PocketRegionSourceKind::FeatureFaces ||
            operation.target.sourceKind ==
                PocketRegionSourceKind::RootInnerWire;
        const bool emptyFaceId = std::any_of(
            operation.target.sourceFaceIds.begin(),
            operation.target.sourceFaceIds.end(),
            [](const std::string& id) { return id.empty(); });
        const bool invalidRootInnerWire =
            operation.target.sourceKind ==
                PocketRegionSourceKind::RootInnerWire &&
            (operation.target.rootFaceId.empty() ||
             operation.target.rootWireId.empty() ||
             operation.target.floorFaceId !=
                 operation.target.rootFaceId ||
             operation.target.openBoundaryFaceCount < 0);
        const bool invalidFeatureFloor =
            operation.target.sourceKind ==
                PocketRegionSourceKind::FeatureFaces &&
            std::find(
                operation.target.sourceFaceIds.begin(),
                operation.target.sourceFaceIds.end(),
                operation.target.floorFaceId) ==
                operation.target.sourceFaceIds.end();
        if (!validKind || !validSourceKind ||
            operation.target.sourceFaceIds.empty() ||
            emptyFaceId ||
            operation.target.floorFaceId.empty() ||
            invalidRootInnerWire || invalidFeatureFloor ||
            operation.target.geometryFingerprint.empty()) {
            errors.emplace_back(
                "pocket operation has an invalid target: " + operation.id);
        }
        if (operation.inputFingerprint.empty()) {
            errors.emplace_back(
                "pocket operation input fingerprint is empty: " +
                operation.id);
        }
    }
    for (const ProfileOperationDefinition& operation :
         m_definition.profileOperations) {
        if (operation.id.empty() ||
            !operationIds.insert(operation.id).second) {
            errors.emplace_back(
                "operation ids must be non-empty and globally unique");
        }
        if (!setupIds.contains(operation.setupId)) {
            errors.emplace_back(
                "profile operation references a missing setup: " +
                operation.id);
        }
        if (!controllerIds.contains(operation.toolControllerId)) {
            errors.emplace_back(
                "profile operation references a missing tool controller: " +
                operation.id);
        }
        if (operation.algorithmId.empty()) {
            errors.emplace_back(
                "profile operation algorithm id is empty: " + operation.id);
        }
        if (operation.sequence < 0 ||
            !operationSequences.insert(operation.sequence).second) {
            errors.emplace_back(
                "operation sequences must be non-negative and globally unique: " +
                operation.id);
        }
        if (!validOperationState(operation.state)) {
            errors.emplace_back(
                "profile operation state is invalid: " + operation.id);
        }
        if (!std::isfinite(
                operation.restMachiningPreviousToolDiameter) ||
            operation.restMachiningPreviousToolDiameter < 0.0) {
            errors.emplace_back(
                "profile operation previous tool diameter is invalid: " +
                operation.id);
        }
        const bool validAxis =
            operation.target.axisLabel == "X" ||
            operation.target.axisLabel == "Y" ||
            operation.target.axisLabel == "Z";
        const bool emptyFaceId = std::any_of(
            operation.target.sourceFaceIds.begin(),
            operation.target.sourceFaceIds.end(),
            [](const std::string& id) { return id.empty(); });
        if (!validAxis ||
            (operation.target.normalDirection != -1 &&
             operation.target.normalDirection != 1) ||
            operation.target.sourceFaceIds.empty() || emptyFaceId ||
            operation.target.rootFaceId.empty() ||
            operation.target.rootWireId.empty() ||
            operation.target.geometryFingerprint.empty()) {
            errors.emplace_back(
                "profile operation has an invalid target: " + operation.id);
        }
        if (operation.inputFingerprint.empty()) {
            errors.emplace_back(
                "profile operation input fingerprint is empty: " +
                operation.id);
        }
    }
    for (const auto& operation : m_definition.finishingOperations) {
        if (operation.id.empty() || !operationIds.insert(operation.id).second)
            errors.emplace_back("operation ids must be non-empty and globally unique");
        if (operation.sequence < 0 || !operationSequences.insert(operation.sequence).second)
            errors.emplace_back("operation sequences must be non-negative and globally unique: " + operation.id);
        if (!setupIds.contains(operation.setupId) || !controllerIds.contains(operation.toolControllerId))
            errors.emplace_back("engraving operation references missing setup/tool: " + operation.id);
        const auto* controller = toolControllerById(operation.toolControllerId);
        if (controller && !detail::finishingCutterMatchesTool(operation.parameters.cutterKind, controller->tool))
            errors.emplace_back("engraving cutter kind differs from its tool snapshot: " + operation.id);
        const auto* expectedAlgorithm = operation.parameters.mode == toolpath::EngravingMachiningMode::StockRoughingAndFinishing
            ? toolpath::kOpenCamLibStockEngravingAlgorithmId : toolpath::kOpenCamLibSurfaceFinishingAlgorithmId;
        if (operation.algorithmId != expectedAlgorithm ||
            !validOperationState(operation.state) || !detail::validFinishingParameters(operation.parameters))
            errors.emplace_back("engraving operation parameters or algorithm are invalid: " + operation.id);
        if (operation.parameters.mode == toolpath::EngravingMachiningMode::StockRoughingAndFinishing &&
            m_definition.stock.sourceKind == StockSourceKind::StepModel && m_definition.stock.sourceFingerprint.empty())
            errors.emplace_back("layered engraving requires a bound external planning stock fingerprint: " + operation.id);
        const std::unordered_set<std::string> units(operation.target.unitIds.begin(), operation.target.unitIds.end());
        if (units.empty() || units.contains("") || units.size() != operation.target.unitIds.size() ||
            operation.target.geometryFingerprint.empty() || operation.inputFingerprint.empty())
            errors.emplace_back("engraving operation target/fingerprint is invalid: " + operation.id);
    }
    return errors;
}

} // namespace oap::manufacturing

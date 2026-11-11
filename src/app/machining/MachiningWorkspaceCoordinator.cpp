// MachiningWorkspaceCoordinator 实现（oap_app_machining）。
// 每模型加工状态、参数与刀具库的应用门面，见 MachiningWorkspaceCoordinator.h。
#include "MachiningWorkspaceCoordinator.h"
#include <QCoreApplication>

#include <algorithm>
#include <unordered_set>
#include <QDateTime>

#include "app/document/ModelDocumentWorkspaceCoordinator.h"
#include "app/recognition/RecognitionWorkspaceCoordinator.h"
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"
#include "manufacturing/job/MachiningJobStorage.h"
#include "manufacturing/job/MachiningJobApplicationService.h"
#include "toolpath/pocket/program/PocketProgramAdapter.h"
#include "toolpath/profile/program/ProfileProgramAdapter.h"
#include "tooling/library/ToolLibraryStorage.h"

namespace oap::app {

// 模型切换时的会话级联重置:清空加工任务/工作流/预览等全部模型相关
// 状态;配置与刀具库是全局的,不受模型切换影响。
void MachiningWorkspaceCoordinator::resetForModel()
{
    m_session.resetForModel();
}

// 孔程序的原子安装:一次性写入任务、工作流与孔源,再从任务重建预览
// 条目;顺序固定,保证观察者看到的预览与任务始终一致。
void MachiningWorkspaceCoordinator::installHoleProgram(
    manufacturing::MachiningJob job,
    std::vector<RecognizedHoleSource> sources,
    const toolpath::ToolpathParams& effectiveParams)
{
    m_session.job() = std::move(job);
    m_session.recognizedHoleSources() = std::move(sources);
    m_session.rebuildHolePreviewEntriesFromJob();
    m_session.activePreview() = ActiveToolpathPreview::Hole;
    m_session.holePreviewParams() = effectiveParams;
    m_session.hasHolePreviewParams() = true;
    m_session.holeToolpathNeedsRegeneration() = false;
}

bool MachiningWorkspaceCoordinator::activateInstalledHoleProgram()
{
    const toolpath::HoleToolpathBatch* result = m_session.job().holeResult();
    if (!result || m_session.holeToolpathNeedsRegeneration())
        return false;
    m_session.holePreviewParams() = result->effectiveParams;
    m_session.hasHolePreviewParams() = true;
    m_session.activePreview() = ActiveToolpathPreview::Hole;
    return true;
}

const toolpath::ProfileToolpathBatch* MachiningWorkspaceCoordinator::currentProfileResult(QString* error) const
{
    if (error) error->clear();
    const auto& job = m_session.job();
    const auto* batch = job.profileResult();
    const auto reject = [error](const QString& message) -> const toolpath::ProfileToolpathBatch* {
        if (error) *error = message;
        return nullptr;
    };
    if (m_session.toolpathNeedsRegeneration(ActiveToolpathPreview::Profile))
        return reject(QCoreApplication::translate("OpenAutoPath", "外轮廓参数已变更，请先重新生成刀路。"));
    if (!batch || batch->cancelled || batch->operations.empty())
        return reject(QCoreApplication::translate("OpenAutoPath", "外轮廓没有已完成的刀路，请先生成。"));
    if (!batch->safetyValidation.ok())
        return reject(QCoreApplication::translate("OpenAutoPath", "外轮廓刀路有验证错误，不能显示为当前结果。"));
    for (const auto& operation : job.definition().profileOperations) {
        if (!operation.enabled) continue;
        using State = manufacturing::JobOperationState;
        if (operation.state != State::Generated && operation.state != State::Warning && operation.state != State::NeedsReview)
            return reject(QCoreApplication::translate("OpenAutoPath", "外轮廓工序尚未生成或已失效，请重新生成。"));
    }
    if (job.runtime().profileExecutionFingerprint.empty() ||
        job.runtime().profileExecutionFingerprint != manufacturing::MachiningJobStorage::profileExecutionFingerprint(job.definition()).toStdString())
        return reject(QCoreApplication::translate("OpenAutoPath", "外轮廓刀路与当前项目定义不一致，请重新生成。"));
    return batch;
}

bool MachiningWorkspaceCoordinator::activateOperationPreview(
    const std::string& formalOperationId, toolpath::ToolpathProgram& program, QString* error)
{
    if (error) error->clear();
    const auto reject = [error](const QString& message) {
        if (error) *error = message;
        return false;
    };
    const auto& job = m_session.job();
    const auto definitions = manufacturing::MachiningOperationService::operations(job.definition());
    const auto selected = std::find_if(definitions.begin(), definitions.end(),
        [&](const auto& operation) { return operation.id == formalOperationId; });
    if (selected == definitions.end()) return reject(QCoreApplication::translate("OpenAutoPath", "所选正式工序已不存在。"));
    if (!selected->enabled) return reject(QCoreApplication::translate("OpenAutoPath", "所选工序已禁用，不能显示其刀路。"));
    ActiveToolpathPreview family = ActiveToolpathPreview::None;
    using Kind = manufacturing::JobOperationKind;
    switch (selected->kind) {
    case Kind::Hole: family = ActiveToolpathPreview::Hole; break;
    case Kind::Pocket: family = ActiveToolpathPreview::Pocket; break;
    case Kind::Profile: family = ActiveToolpathPreview::Profile; break;
    case Kind::SurfaceFinishing: family = ActiveToolpathPreview::SurfaceFinishing; break;
    }
    if (m_session.toolpathNeedsRegeneration(family))
        return reject(QCoreApplication::translate("OpenAutoPath", "此类加工参数已变更，请先成功重新生成刀路。"));
    for (const auto& operation : definitions) {
        if (!operation.enabled || operation.kind != selected->kind) continue;
        using State = manufacturing::JobOperationState;
        if (operation.state != State::Generated && operation.state != State::Warning && operation.state != State::NeedsReview)
            return reject(QCoreApplication::translate("OpenAutoPath", "此类加工包含尚未生成或已失效的工序，请先重新生成。"));
    }
    toolpath::ToolpathProgram candidate;
    if (family == ActiveToolpathPreview::Hole) {
        const auto* batch = job.holeResult();
        if (!batch || batch->cancelled || !batch->validation.ok())
            return reject(QCoreApplication::translate("OpenAutoPath", "孔加工没有已完成且有效的刀路，请先生成。"));
        if (job.runtime().holeExecutionFingerprint.empty() || job.runtime().holeExecutionFingerprint !=
            manufacturing::MachiningJobStorage::holeExecutionFingerprint(job.definition()).toStdString())
            return reject(QCoreApplication::translate("OpenAutoPath", "孔刀路与当前项目定义不一致，请重新生成。"));
        candidate = batch->toProgram();
    } else if (family == ActiveToolpathPreview::Pocket) {
        if (const auto* batch = job.currentPocketResult()) candidate = batch->toProgram();
        else return reject(QCoreApplication::translate("OpenAutoPath", "槽腔刀路不存在、未完成或已失效，请重新生成。"));
    } else if (family == ActiveToolpathPreview::Profile) {
        if (const auto* batch = currentProfileResult(error)) candidate = batch->toProgram();
        else return false;
    } else {
        if (const auto* batch = job.currentFinishingResult()) candidate = batch->toProgram();
        else return reject(QCoreApplication::translate("OpenAutoPath", "雕刻刀路不存在、未完成或已失效，请重新生成。"));
    }
    if (candidate.empty() || candidate.moveCount() == 0)
        return reject(QCoreApplication::translate("OpenAutoPath", "此类加工没有可显示的移动。"));
    std::unordered_set<std::string> represented;
    for (const auto& operation : candidate.operations) {
        const auto formal = manufacturing::MachiningOperationService::operationForRuntimeId(job.definition(), operation.operationId);
        if (!formal || !formal->enabled || formal->kind != selected->kind || operation.moves.empty())
            return reject(QCoreApplication::translate("OpenAutoPath", "运行期刀路与正式工序的对应关系不完整，请重新生成。"));
        represented.insert(formal->id);
    }
    for (const auto& operation : definitions)
        if (operation.enabled && operation.kind == selected->kind && !represented.contains(operation.id))
            return reject(QCoreApplication::translate("OpenAutoPath", "此类加工的刀路不完整，请重新生成全部启用工序。"));
    // Update derived preview state only after every freshness/mapping check.
    if (family == ActiveToolpathPreview::Hole) {
        if (!activateInstalledHoleProgram()) return reject(QCoreApplication::translate("OpenAutoPath", "孔刀路已失效，请重新生成。"));
        m_session.rebuildHolePreviewEntriesFromJob();
    } else if (family == ActiveToolpathPreview::Profile) {
        const auto* batch = currentProfileResult();
        m_session.profilePreviewOperations() = batch->operations;
        m_session.profilePreviewEffectiveParams() = batch->effectiveParams;
        m_session.profilePreviewParams() = job.definition().profileParameters;
        m_session.hasProfilePreviewParams() = true;
    } else if (family == ActiveToolpathPreview::Pocket) {
        const auto* batch = job.currentPocketResult();
        m_session.pocketPreviewEffectiveParams() = batch->effectiveParams;
        m_session.pocketPreviewParams() = job.definition().pocketParameters;
        m_session.hasPocketPreviewParams() = true;
        m_session.pocketPreviewActive() = true;
    }
    m_session.activePreview() = family;
    program = std::move(candidate);
    return true;
}

void MachiningWorkspaceCoordinator::markHoleRegenerationRequired()
{
    if (m_session.job().hasHoleResult() || !m_session.job().definition().holeOperations.empty())
        m_session.holeToolpathNeedsRegeneration() = true;
    m_session.job().clearHoleResult();
    m_session.holePreviewEntries().clear();
    m_session.hasHolePreviewParams() = false;
    for (auto& operation : m_session.job().definition().holeOperations)
        operation.state = manufacturing::JobOperationState::Stale;
}

void MachiningWorkspaceCoordinator::invalidateHoleProductionSafety()
{
    if (toolpath::HoleToolpathBatch* result = m_session.job().holeResult())
        result->productionSafety = {};
}

void MachiningWorkspaceCoordinator::markPocketRegenerationRequired()
{
    if (!m_session.job().definition().pocketOperations.empty())
        m_session.setToolpathNeedsRegeneration(ActiveToolpathPreview::Pocket, true);
    m_session.hasPocketPreviewParams() = false;
    m_session.pocketPreviewActive() = false;
    m_session.pocketPreviewEntries().clear();
    m_session.job().clearPocketResult();
    for (auto& operation : m_session.job().definition().pocketOperations)
        operation.state = manufacturing::JobOperationState::Stale;
    // Preserve the selected family so export reports a missing current result
    // instead of silently falling back to an unrelated hole program.
}

void MachiningWorkspaceCoordinator::markProfileRegenerationRequired()
{
    if (!m_session.job().definition().profileOperations.empty())
        m_session.setToolpathNeedsRegeneration(ActiveToolpathPreview::Profile, true);
    m_session.hasProfilePreviewParams() = false;
    m_session.profilePreviewOperations().clear();
    m_session.job().clearProfileResult();
    for (auto& operation : m_session.job().definition().profileOperations)
        operation.state = manufacturing::JobOperationState::Stale;
}

PocketPreviewSnapshot
MachiningWorkspaceCoordinator::capturePocketPreview() const
{
    PocketPreviewSnapshot snapshot;
    snapshot.activePreview = m_session.activePreview();
    snapshot.entries = m_session.pocketPreviewEntries();
    snapshot.params = m_session.pocketPreviewParams();
    snapshot.effectiveParams = m_session.pocketPreviewEffectiveParams();
    snapshot.previewActive = m_session.pocketPreviewActive();
    snapshot.hasParams = m_session.hasPocketPreviewParams();
    snapshot.profileOperations = m_session.profilePreviewOperations();
    snapshot.profileParams = m_session.profilePreviewParams();
    snapshot.profileEffectiveParams =
        m_session.profilePreviewEffectiveParams();
    snapshot.hasProfileParams = m_session.hasProfilePreviewParams();
    return snapshot;
}

void MachiningWorkspaceCoordinator::restorePocketPreview(
    PocketPreviewSnapshot snapshot)
{
    m_session.activePreview() = snapshot.activePreview;
    m_session.pocketPreviewEntries() = std::move(snapshot.entries);
    m_session.pocketPreviewParams() = snapshot.params;
    m_session.pocketPreviewEffectiveParams() = snapshot.effectiveParams;
    m_session.pocketPreviewActive() = snapshot.previewActive;
    m_session.hasPocketPreviewParams() = snapshot.hasParams;
    m_session.profilePreviewOperations() =
        std::move(snapshot.profileOperations);
    m_session.profilePreviewParams() = snapshot.profileParams;
    m_session.profilePreviewEffectiveParams() =
        snapshot.profileEffectiveParams;
    m_session.hasProfilePreviewParams() = snapshot.hasProfileParams;
}

void MachiningWorkspaceCoordinator::installPocketProgram(
    manufacturing::MachiningJob job,
    std::vector<PocketPreviewEntry> entries,
    const toolpath::ToolpathParams& requestedParams,
    const toolpath::ToolpathParams& effectiveParams)
{
    m_session.job() = std::move(job);
    m_session.pocketPreviewEntries() = std::move(entries);
    m_session.pocketPreviewActive() = true;
    m_session.activePreview() = ActiveToolpathPreview::Pocket;
    m_session.pocketPreviewParams() = requestedParams;
    m_session.pocketPreviewEffectiveParams() = effectiveParams;
    m_session.hasPocketPreviewParams() = true;
    m_session.setToolpathNeedsRegeneration(ActiveToolpathPreview::Pocket, false);
}

void MachiningWorkspaceCoordinator::installProfileProgram(
    manufacturing::MachiningJob job,
    const toolpath::ToolpathParams& requestedParams,
    const toolpath::ToolpathParams& effectiveParams)
{
    std::vector<toolpath::ProfileToolpathOperation> operations;
    if (const toolpath::ProfileToolpathBatch* result = job.profileResult())
        operations = result->operations;
    m_session.job() = std::move(job);
    m_session.profilePreviewOperations() = std::move(operations);
    m_session.profilePreviewParams() = requestedParams;
    m_session.profilePreviewEffectiveParams() = effectiveParams;
    m_session.hasProfilePreviewParams() = true;
    m_session.activePreview() = ActiveToolpathPreview::Profile;
    m_session.setToolpathNeedsRegeneration(ActiveToolpathPreview::Profile, false);
}

void MachiningWorkspaceCoordinator::clearPocketPreviewState()
{
    m_session.pocketPreviewEntries().clear();
    m_session.pocketPreviewActive() = false;
    m_session.hasPocketPreviewParams() = false;
    if (m_session.activePreview() == ActiveToolpathPreview::Pocket)
        m_session.activePreview() = ActiveToolpathPreview::None;
}

// 过滤槽预览条目:取出候选后原地筛选,避免过滤期间其他引用读到
// 正在缩容的容器;返回保留数量供 UI 判断是否仍可显示。
int MachiningWorkspaceCoordinator::retainDisplayablePocketPreviewEntries(
    const std::function<bool(PocketPreviewEntry&)>& display)
{
    std::vector<PocketPreviewEntry> candidates =
        std::move(m_session.pocketPreviewEntries());
    m_session.pocketPreviewEntries().clear();
    for (PocketPreviewEntry& entry : candidates) {
        if (display(entry)) {
            m_session.pocketPreviewEntries().push_back(std::move(entry));
        }
    }
    return static_cast<int>(m_session.pocketPreviewEntries().size());
}

HoleToolpathGenerationResult
MachiningWorkspaceCoordinator::generateHoleToolpath(
    HoleToolpathGenerationRequest request) const
{
    request.currentJob = &m_session.job();
    return HoleToolpathGenerationUseCase::execute(request);
}

PocketToolpathGenerationResult
MachiningWorkspaceCoordinator::generatePocketToolpath(
    PocketToolpathGenerationRequest request,
    const PocketPreviewPublisher& previewPublisher) const
{
    request.currentJob = &m_session.job();
    return PocketToolpathGenerationUseCase::execute(
        request, previewPublisher);
}

ProfileToolpathGenerationResult
MachiningWorkspaceCoordinator::generateProfileToolpath(
    ProfileToolpathGenerationRequest request,
    const ProfilePreviewPublisher& previewPublisher) const
{
    request.currentJob = &m_session.job();
    return ProfileToolpathGenerationUseCase::execute(
        request, previewPublisher);
}

bool MachiningWorkspaceCoordinator::ensureProjectDefinition(
    const MachiningProjectDefinitionContext& context,
    QString* error)
{
    return MachiningProjectController::ensureDefinition(
        m_session.job(), context, error);
}

bool MachiningWorkspaceCoordinator::capturePocketProjectOperations(
    const MachiningProjectPocketCaptureRequest& request,
    QString* error)
{
    MachiningProjectPocketCaptureRequest resolved = request;
    std::string resultError;
    const auto* batch = m_session.job().currentPocketResult(&resultError);
    if (!batch) {
        if (error) *error = QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(resultError).toUtf8().constData());
        return false;
    }
    resolved.operations = &batch->operations;
    resolved.effectiveParams = &batch->effectiveParams;
    return MachiningProjectController::capturePocketOperations(
        m_session.job(), resolved, error);
}

bool MachiningWorkspaceCoordinator::captureProfileProjectPreview(
    const MachiningProjectProfileCaptureRequest& request,
    QString* error)
{
    MachiningProjectProfileCaptureRequest resolved = request;
    resolved.operations = &m_session.profilePreviewOperations();
    if (!resolved.effectiveParams && m_session.hasProfilePreviewParams())
        resolved.effectiveParams = &m_session.profilePreviewEffectiveParams();
    return MachiningProjectController::captureProfilePreview(
        m_session.job(), resolved, error);
}

std::vector<manufacturing::JobOperationDescriptor>
MachiningWorkspaceCoordinator::operationDescriptors() const
{
    return manufacturing::MachiningOperationService::operations(
        m_session.job().definition());
}

MachiningProjectLoadResult
MachiningWorkspaceCoordinator::regenerateProject(
    const MachiningProjectLoadRequest& request) const
{
    if (hasPendingRegeneration()) {
        MachiningProjectLoadResult rejected;
        rejected.failure = MachiningProjectLoadFailure::PendingConfiguration;
        rejected.error = QCoreApplication::translate("OpenAutoPath", "存在未应用到正式工序的参数变更，请先通过对应加工入口重新生成。");
        return rejected;
    }
    return MachiningProjectController::regenerateJob(
        m_session.job(), request);
}

void MachiningWorkspaceCoordinator::invalidateOperationPreviews()
{
    m_session.pocketPreviewEntries().clear();
    m_session.pocketPreviewActive() = false;
    m_session.hasPocketPreviewParams() = false;
    m_session.profilePreviewOperations().clear();
    m_session.hasProfilePreviewParams() = false;
    m_session.holePreviewEntries().clear();
    m_session.hasHolePreviewParams() = false;
    m_session.holeToolpathNeedsRegeneration() = true;
    m_session.activePreview() = ActiveToolpathPreview::None;
}

bool MachiningWorkspaceCoordinator::synchronizeProjectForSave(
    const MachiningProjectSaveRequest& request,
    QString* error)
{
    if (hasPendingRegeneration()) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "加工参数已修改但尚未成功重新生成，请先重新生成受影响工序再保存项目。");
        return false;
    }
    return MachiningProjectController::synchronizeForSave(
        m_session.job(), request, error);
}

bool MachiningWorkspaceCoordinator::saveProject(
    const QString& filePath,
    QString* error) const
{
    if (hasPendingRegeneration()) {
        if (error) *error = QCoreApplication::translate("OpenAutoPath", "存在待重新生成的工序；旧工序不能作为当前参数保存，请先重新生成。");
        return false;
    }
    return manufacturing::MachiningJobStorage::save(
        filePath, m_session.job().definition(), error);
}

MachiningWorkspaceStatus
MachiningWorkspaceCoordinator::status() const noexcept
{
    MachiningWorkspaceStatus result;
    const manufacturing::MachiningJob& job = m_session.job();
    result.hasHoleProgram = job.hasHoleResult();
    result.hasPocketProgram = job.hasPocketResult();
    result.hasProfileProgram = job.hasProfileResult();
    result.hasFinishingProgram = job.hasFinishingResult();
    if (const auto* finishing = job.finishingResult())
        result.finishingOperationCount = static_cast<int>(finishing->operations.size());
    result.hasProjectContent = hasProjectContent();
    if (const toolpath::HoleToolpathBatch* hole = job.holeResult()) {
        result.holeOperationCount = static_cast<int>(
            hole->machining.operations.size());
    }
    if (const toolpath::PocketToolpathBatch* pocket = job.pocketResult()) {
        result.pocketOperationCount = static_cast<int>(
            pocket->operations.size());
    }
    result.profileOperationCount = static_cast<int>(
        m_session.profilePreviewOperations().size());
    const toolpath::HoleCoverageReport& coverage =
        job.runtime().holeResult.coverage;
    result.coverageAudited = coverage.audited;
    result.coveragePassed = coverage.complete();
    result.coverageHasWarnings =
        !coverage.outsideSixFaceSourceHoleIndices.empty() ||
        !coverage.warningIssues.empty();
    return result;
}

bool MachiningWorkspaceCoordinator::hasHoleProgram() const noexcept
{
    return m_session.job().hasHoleResult();
}

bool MachiningWorkspaceCoordinator::hasProjectContent() const noexcept
{
    const manufacturing::MachiningJobDefinition& definition =
        m_session.job().definition();
    return !definition.holeOperations.empty() ||
           !definition.pocketOperations.empty() ||
           !definition.profileOperations.empty() ||
           !definition.finishingOperations.empty();
}

const toolpath::HoleToolpathBatch*
MachiningWorkspaceCoordinator::holeProgram() const noexcept
{
    return m_session.job().holeResult();
}

std::optional<toolpath::HoleToolpathBatch>
MachiningWorkspaceCoordinator::holeProgramSnapshot() const
{
    const toolpath::HoleToolpathBatch* program = holeProgram();
    return program
        ? std::optional<toolpath::HoleToolpathBatch>(*program)
        : std::nullopt;
}

const toolpath::HoleCoverageReport&
MachiningWorkspaceCoordinator::holeCoverageReport() const noexcept
{
    return m_session.job().runtime().holeResult.coverage;
}

std::vector<std::string>
MachiningWorkspaceCoordinator::validationErrors() const
{
    return m_session.job().validationErrors();
}

toolpath::ToolpathProgram
MachiningWorkspaceCoordinator::pocketPreviewProgram() const
{
    const auto* batch = m_session.job().currentPocketResult();
    return batch ? batch->toProgram() : toolpath::ToolpathProgram{};
}

toolpath::ToolpathProgram
MachiningWorkspaceCoordinator::profilePreviewProgram() const
{
    return toolpath::makeToolpathProgram(
        m_session.profilePreviewOperations());
}

const RecognizedHoleSource*
MachiningWorkspaceCoordinator::recognizedHoleSource(int index) const noexcept
{
    if (index < 0 ||
        index >= static_cast<int>(m_session.recognizedHoleSources().size())) {
        return nullptr;
    }
    return &m_session.recognizedHoleSources()[
        static_cast<std::size_t>(index)];
}

ToolpathExportPreparation
MachiningWorkspaceCoordinator::prepareToolpathExport() const
{
    return ToolpathExportController::prepare(m_session);
}

// 工程安装的跨工作区同步:安装器先原子替换本协调器三块状态,再依次
// 把毛坯定义、识别孔与装夹工位坐标系推送到 stock/recognition/document,
// 使各工作区围绕同一工程保持一致。
MachiningProjectInstallResult MachiningWorkspaceCoordinator::installProject(
    MachiningProjectLoadResult loaded,
    StockSimulationWorkspaceCoordinator& stockWorkspace,
    RecognitionWorkspaceCoordinator& recognitionWorkspace,
    ModelDocumentWorkspaceCoordinator& documentWorkspace)
{
    MachiningProjectInstallResult result = MachiningProjectInstaller::install(
        std::move(loaded), m_session, m_configuration,
        m_toolLibraries, documentWorkspace.setupWorkFrames());
    stockWorkspace.restoreHoleStockDefinition(result.stockDefinition);
    stockWorkspace.installUnifiedIpwRuntime(m_session.job());
    recognitionWorkspace.synchronizeLoadedProjectHoles(
        m_session.job().definition().recognizedHoles);
    documentWorkspace.installSetupWorkFrames(
        std::move(result.setupWorkFrames));
    return result;
}

HoleStockRunOutcome
MachiningWorkspaceCoordinator::executeHoleStockSimulation(
    StockSimulationWorkspaceCoordinator& stockWorkspace,
    HoleStockSimulationUseCaseRequest request)
{
    return stockWorkspace.executeHoleSimulation(
        std::move(request), m_session.job());
}

PocketStockSimulationUseCaseResult
MachiningWorkspaceCoordinator::executePocketStockSimulation(
    StockSimulationWorkspaceCoordinator& stockWorkspace,
    PocketStockSimulationUseCaseRequest request)
{
    return stockWorkspace.executePocketSimulation(
        std::move(request), m_session.job());
}

ProfileStockSimulationUseCaseResult
MachiningWorkspaceCoordinator::executeProfileStockSimulation(
    StockSimulationWorkspaceCoordinator& stockWorkspace,
    ProfileStockSimulationUseCaseRequest request)
{
    return stockWorkspace.executeProfileSimulation(
        std::move(request), m_session.job(),
        m_session.profilePreviewOperations(),
        m_session.profilePreviewEffectiveParams());
}

void MachiningWorkspaceCoordinator::refreshPocketSimulationDisplay(
    StockSimulationWorkspaceCoordinator& stockWorkspace,
    const gp_Trsf& modelTransform) const
{
    stockWorkspace.refreshPocketDisplay(
        m_session.job(), modelTransform);
}

std::vector<IpwTimelineOperation>
MachiningWorkspaceCoordinator::pocketSimulationTimeline(
    const StockSimulationWorkspaceCoordinator& stockWorkspace) const
{
    return stockWorkspace.timelineOperations(m_session.job());
}

void MachiningWorkspaceCoordinator::displayPocketSimulationTransient(
    StockSimulationWorkspaceCoordinator& stockWorkspace,
    int operationIndex,
    int completedMoveCount,
    const gp_Trsf& modelTransform) const
{
    stockWorkspace.displayPocketTransient(
        m_session.job(), operationIndex, completedMoveCount,
        modelTransform);
}

void MachiningWorkspaceCoordinator::finishPocketSimulationTimeline(
    StockSimulationWorkspaceCoordinator& stockWorkspace,
    const gp_Trsf& modelTransform) const
{
    stockWorkspace.finishTimeline(m_session.job(), modelTransform);
}

QString MachiningWorkspaceCoordinator::loadConfigurationDefaults()
{
    return m_configuration.loadDefaults();
}

ToolpathParameterEditorState
MachiningWorkspaceCoordinator::configurationEditorState(
    const toolpath::MoldSetupWorkFrames& workFrames,
    double moldLength,
    double moldWidth) const
{
    auto state = m_configuration.editorState(workFrames, moldLength, moldWidth);
    state.activeToolLibraryId = m_toolLibraries.activeLibraryId().toStdString();
    for (const auto& library : m_toolLibraries.libraries())
        state.engravingLibraries.push_back({library.id().toStdString(), library.name().toStdString()});
    return state;
}

// 应用配置后把推导出的装夹工位坐标系一并安装到 document 协调器,
// 保证后续加工/仿真使用同一套工位坐标系。
MachiningConfigurationApplyResult
MachiningWorkspaceCoordinator::applyConfiguration(
    ToolpathParameterEditorState edited,
    ModelDocumentWorkspaceCoordinator& documentWorkspace)
{
    const auto sameFrame = [](const gp_Ax3& a, const gp_Ax3& b) {
        return a.Location().IsEqual(b.Location(), 1e-10) &&
            a.Direction().IsEqual(b.Direction(), 1e-10) &&
            a.XDirection().IsEqual(b.XDirection(), 1e-10);
    };
    const auto& previousSetup = m_configuration.machiningSetup();
    bool setupChanged = previousSetup.workOffset != edited.machiningSetup.workOffset ||
        !sameFrame(previousSetup.workFrame, edited.machiningSetup.workFrame);
    for (int index = 0; index < 6; ++index) {
        const auto face = static_cast<MoldSetupFace>(index);
        const auto* previous = documentWorkspace.setupWorkFrames().forFace(face);
        const auto* next = edited.moldSetupWorkFrames.forFace(face);
        setupChanged = setupChanged || (previous == nullptr) != (next == nullptr);
        if (previous && next)
            setupChanged = setupChanged || previous->valid != next->valid ||
                previous->workOffset != next->workOffset ||
                !sameFrame(previous->setupWorkFrame, next->setupWorkFrame);
    }
    MachiningConfigurationApplyResult result =
        m_configuration.apply(std::move(edited));
    result.holeParametersChanged = result.holeParametersChanged || setupChanged;
    result.pocketParametersChanged = result.pocketParametersChanged || setupChanged;
    result.profileParametersChanged = result.profileParametersChanged || setupChanged;
    result.engravingParametersChanged = result.engravingParametersChanged || setupChanged;
    if (result.holeParametersChanged) markHoleRegenerationRequired();
    if (result.pocketParametersChanged) markPocketRegenerationRequired();
    if (result.profileParametersChanged) markProfileRegenerationRequired();
    if (result.engravingParametersChanged) markEngravingRegenerationRequired();
    if (!setupChanged) {
        // Confirmation-only edits must reach the formal Job immediately. Keep
        // generated moves and their fingerprints; never revive stale results.
        auto& job = m_session.job();
        auto& definition = job.definition();
        manufacturing::MachiningJobApplicationService::captureSetups(
            definition, result.setupWorkFrames,
            QDateTime::currentDateTimeUtc().toString(Qt::ISODate).toStdString());
        const auto* hole = job.runtime().holeExecutionFingerprint ==
            manufacturing::MachiningJobStorage::holeExecutionFingerprint(definition).toStdString()
            ? job.holeResult() : nullptr;
        const auto* profile = job.runtime().profileExecutionFingerprint ==
            manufacturing::MachiningJobStorage::profileExecutionFingerprint(definition).toStdString()
            ? job.profileResult() : nullptr;
        manufacturing::MachiningJobApplicationService::reconcileOperationStates(
            definition, hole, job.currentPocketResult(), profile, job.currentFinishingResult());
    }
    documentWorkspace.installSetupWorkFrames(
        result.setupWorkFrames);
    return result;
}

QStringList MachiningWorkspaceCoordinator::loadToolLibraries()
{
    return m_toolLibraries.load();
}

ToolLibraryApplyResult MachiningWorkspaceCoordinator::applyToolLibraries(
    tooling::ToolAssetCatalog catalog,
    QString activeLibraryId)
{
    return applyToolLibraryChange([&] {
        return m_toolLibraries.apply(std::move(catalog), std::move(activeLibraryId));
    });
}

ToolLibraryApplyResult MachiningWorkspaceCoordinator::applyToolLibraries(
    QVector<tooling::ToolLibrary> libraries,
    QString activeLibraryId)
{
    return applyToolLibraryChange([&] {
        return m_toolLibraries.apply(std::move(libraries), std::move(activeLibraryId));
    });
}

ToolLibraryApplyResult MachiningWorkspaceCoordinator::applyToolLibraryChange(
    const std::function<ToolLibraryApplyResult()>& apply)
{
    const auto snapshot = [&](const std::string& requestedId) {
        const auto id = requestedId.empty() ? m_toolLibraries.activeLibraryId() : QString::fromStdString(requestedId);
        for (const auto& library : m_toolLibraries.libraries())
            if (library.id() == id) return tooling::ToolLibraryStorage::serializeLibrary(library);
        return QByteArray{};
    };
    const auto activeBefore = snapshot({});
    const auto engravingBefore = snapshot(m_configuration.engravingSettings().libraryId);
    auto result = apply();
    if (!result.ok) return result;
    if (activeBefore != snapshot({})) {
        if (m_configuration.holeToolSelectionMode() == ToolSelectionMode::AutomaticLibrary) markHoleRegenerationRequired();
        if (m_configuration.pocketToolSelectionMode() == ToolSelectionMode::AutomaticLibrary) markPocketRegenerationRequired();
        if (m_configuration.profileToolSelectionMode() == ToolSelectionMode::AutomaticLibrary) markProfileRegenerationRequired();
    }
    if (m_configuration.engravingSettings().toolMode == EngravingToolMode::AutomaticLibrary &&
        engravingBefore != snapshot(m_configuration.engravingSettings().libraryId))
        markEngravingRegenerationRequired();
    return result;
}

bool MachiningWorkspaceCoordinator::restoreActiveToolLibrary(
    const QString& activeLibraryId)
{
    return m_toolLibraries.restoreActiveLibrary(activeLibraryId);
}

bool MachiningWorkspaceCoordinator::toggleHighlightedHoleStage(
    int sourceIndex)
{
    auto& highlighted = m_session.highlightedHoleStageSources();
    if (highlighted.erase(sourceIndex) > 0)
        return false;
    highlighted.insert(sourceIndex);
    return true;
}

void MachiningWorkspaceCoordinator::removeHighlightedHoleStage(
    int sourceIndex)
{
    m_session.highlightedHoleStageSources().erase(sourceIndex);
}

void MachiningWorkspaceCoordinator::clearHighlightedHoleStages()
{
    m_session.highlightedHoleStageSources().clear();
}

std::vector<toolpath::HoleMachiningInput>
MachiningWorkspaceCoordinator::sourceHoleInputs() const
{
    return HoleToolpathWorkbenchCoordinator::sourceInputs(m_session);
}

int MachiningWorkspaceCoordinator::selectedHoleStageIndex(
    const std::vector<std::string>& selectedFaceIds) const
{
    return HoleToolpathWorkbenchCoordinator::selectedStageIndex(
        m_session, selectedFaceIds);
}

HoleToolTreeModel MachiningWorkspaceCoordinator::holeToolTreeModel(
    const std::vector<PlaybackMove>& moves,
    const std::vector<int>& toolBatchStarts) const
{
    return HoleToolpathWorkbenchCoordinator::buildTreeModel(
        m_session, moves, toolBatchStarts);
}

QVector<HoleCoverageOperationBinding>
MachiningWorkspaceCoordinator::holeCoverageOperationBindings(
    const std::vector<PlaybackMove>& moves) const
{
    return HoleToolpathWorkbenchCoordinator::buildCoverageOperationBindings(
        m_session, moves);
}

QString MachiningWorkspaceCoordinator::formalOperationIdForRuntime(
    const QString& runtimeOperationId) const
{
    return QString::fromStdString(
        manufacturing::MachiningOperationService::
            formalOperationIdForRuntime(
                m_session.job().definition(),
                runtimeOperationId.toStdString()));
}

} // namespace oap::app

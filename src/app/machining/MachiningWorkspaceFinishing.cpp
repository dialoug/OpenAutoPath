// Formal finishing installation is transactional across definition and runtime.
#include "app/machining/MachiningWorkspaceCoordinator.h"
#include <QCoreApplication>
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"

namespace oap::app {

const EngravingSettings& MachiningWorkspaceCoordinator::engravingSettings() const
{
    return m_configuration.engravingSettings();
}

void MachiningWorkspaceCoordinator::markEngravingRegenerationRequired()
{
    if (!m_session.job().definition().finishingOperations.empty())
        m_session.setToolpathNeedsRegeneration(ActiveToolpathPreview::SurfaceFinishing, true);
    m_session.job().clearFinishingResult();
    for (auto& operation : m_session.job().definition().finishingOperations)
        operation.state = manufacturing::JobOperationState::Stale;
}

EngravingToolpathGenerationResult MachiningWorkspaceCoordinator::generateEngraving(
    EngravingToolpathGenerationRequest request)
{
    request.settings = m_configuration.engravingSettings();
    request.currentJob = &m_session.job();
    request.library = nullptr;
    if (request.settings.toolMode == EngravingToolMode::AutomaticLibrary) {
        if (request.settings.libraryId.empty()) {
            request.library = m_toolLibraries.activeLibrary();
        } else {
            for (const auto& library : m_toolLibraries.libraries()) {
                if (library.id().toStdString() == request.settings.libraryId) {
                    request.library = &library;
                    break;
                }
            }
        }
    }
    auto result = EngravingToolpathGenerationUseCase::execute(request);
    if (result.ok) {
        m_session.job() = result.job;
        m_session.setToolpathNeedsRegeneration(ActiveToolpathPreview::SurfaceFinishing, false);
        m_session.activePreview() = m_session.job().currentFinishingResult()
            ? ActiveToolpathPreview::SurfaceFinishing : ActiveToolpathPreview::None;
    }
    return result;
}

ProjectStockSimulationUseCaseResult MachiningWorkspaceCoordinator::executeProjectStockSimulation(
    StockSimulationWorkspaceCoordinator& stockWorkspace,
    ProjectStockSimulationUseCaseRequest request)
{
    if (!request.engravingOnly && m_session.hasPendingRegeneration(true)) {
        ProjectStockSimulationUseCaseResult rejected;
        rejected.error = QCoreApplication::translate("OpenAutoPath", "启用工序有尚未成功重新生成的参数变更，不能执行完整项目 IPW。");
        return rejected;
    }
    return stockWorkspace.executeProjectSimulation(std::move(request), m_session.job());
}

std::optional<SurfaceFinishingEditState>
MachiningWorkspaceCoordinator::finishingOperation(const std::string& id) const
{
    const auto& job = m_session.job();
    for (const auto& operation : job.definition().finishingOperations) {
        if (operation.id != id)
            continue;
        const auto* setup = job.setupById(operation.setupId);
        const auto* controller = job.toolControllerById(operation.toolControllerId);
        if (!setup || !controller)
            return std::nullopt;
        return SurfaceFinishingEditState{operation.parameters, controller->tool,
            setup->face, operation.target.unitIds, controller->libraryId};
    }
    return std::nullopt;
}

bool MachiningWorkspaceCoordinator::installSurfaceFinishing(
    const MachiningProjectDefinitionContext& definition,
    const manufacturing::SurfaceFinishingJobCaptureRequest& request,
    QString* error)
{
    auto candidate = m_session.job();
    if (!MachiningProjectController::ensureDefinition(candidate, definition, error))
        return false;
    std::string detail;
    if (!manufacturing::SurfaceFinishingMachiningJobService().capture(candidate, request, &detail)) {
        if (error)
            *error = QCoreApplication::translate("OpenAutoPathDiagnostic", QString::fromStdString(detail).toUtf8().constData());
        return false;
    }
    m_session.job() = std::move(candidate);
    m_session.activePreview() = m_session.job().currentFinishingResult()
        ? ActiveToolpathPreview::SurfaceFinishing : ActiveToolpathPreview::None;
    if (error)
        error->clear();
    return true;
}

bool MachiningWorkspaceCoordinator::hasFinishingPreview() const
{
    return m_session.job().currentFinishingResult() != nullptr;
}

toolpath::ToolpathProgram MachiningWorkspaceCoordinator::finishingPreviewProgram() const
{
    const auto* batch = m_session.job().currentFinishingResult();
    return batch ? batch->toProgram() : toolpath::ToolpathProgram{};
}

toolpath::ToolpathParams MachiningWorkspaceCoordinator::finishingPreviewParams() const
{
    const auto* batch = m_session.job().currentFinishingResult();
    return batch ? batch->effectiveParams : toolpath::ToolpathParams{};
}

} // namespace oap::app

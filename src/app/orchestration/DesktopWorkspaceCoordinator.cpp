// DesktopWorkspaceCoordinator 实现（oap_app_orchestration）。
// 跨工作区事务与生命周期协调，见 DesktopWorkspaceCoordinator.h。
#include "DesktopWorkspaceCoordinator.h"

#include "app/document/ModelDocumentWorkspaceCoordinator.h"
#include "app/machining/MachiningWorkspaceCoordinator.h"
#include "app/playback/PlaybackWorkspaceCoordinator.h"
#include "app/recognition/RecognitionWorkspaceCoordinator.h"
#include "app/simulation/StockSimulationWorkspaceCoordinator.h"

namespace oap::app {

DesktopWorkspaceCoordinator::DesktopWorkspaceCoordinator(
    MachiningWorkspaceCoordinator& machining,
    StockSimulationWorkspaceCoordinator& stockSimulation,
    PlaybackWorkspaceCoordinator& playback,
    RecognitionWorkspaceCoordinator& recognition,
    ModelDocumentWorkspaceCoordinator& document) noexcept
    : m_machining(machining),
      m_stockSimulation(stockSimulation),
      m_playback(playback),
      m_recognition(recognition),
      m_document(document)
{
}

bool DesktopWorkspaceCoordinator::backgroundBusy() const noexcept
{
    return m_stockSimulation.busy() || m_document.busy();
}

void DesktopWorkspaceCoordinator::requestCancel() noexcept
{
    m_stockSimulation.requestCancel();
    m_recognition.cancel();
    m_document.requestCancelAll();
    m_playback.requestCancel();
}

void DesktopWorkspaceCoordinator::cancelAndWait()
{
    m_stockSimulation.requestCancel();
    m_recognition.cancelAndWait();
    m_stockSimulation.cancelAndWait();
    m_playback.cancelAndWait();
    m_document.cancelAndWait();
}

void DesktopWorkspaceCoordinator::resetForModel()
{
    m_machining.resetForModel();
    m_stockSimulation.resetForModel();
    m_playback.clearPresentation();
    m_recognition.resetForModel();
}

void DesktopWorkspaceCoordinator::clearTransientState(
    bool resetHoleStageSources)
{
    m_playback.clearHoleStageHighlight();
    if (resetHoleStageSources)
        m_machining.clearHighlightedHoleStages();
    m_recognition.clearInspection();
}

void DesktopWorkspaceCoordinator::clearToolpathState()
{
    m_playback.clearStaticToolpathPresentation();
    m_machining.clearHighlightedHoleStages();
    m_machining.clearPocketPreviewState();
    m_machining.setActivePreview(ActiveToolpathPreview::None);
}

void DesktopWorkspaceCoordinator::clearPlaybackProgram()
{
    m_playback.clearProgram();
}

void DesktopWorkspaceCoordinator::clearHoleSimulation()
{
    m_stockSimulation.clearHole();
}

void DesktopWorkspaceCoordinator::clearPocketSimulation()
{
    m_stockSimulation.clearPocket();
}

void DesktopWorkspaceCoordinator::clearSimulations()
{
    clearHoleSimulation();
    clearPocketSimulation();
}

void DesktopWorkspaceCoordinator::publishPocketProgram(
    manufacturing::MachiningJob job,
    std::vector<PocketPreviewEntry> entries,
    const toolpath::ToolpathParams& requestedParams,
    const toolpath::ToolpathParams& effectiveParams)
{
    clearPocketSimulation();
    m_machining.installPocketProgram(
        std::move(job), std::move(entries), requestedParams,
        effectiveParams);
}

void DesktopWorkspaceCoordinator::publishProfileProgram(
    manufacturing::MachiningJob job,
    const toolpath::ToolpathParams& requestedParams,
    const toolpath::ToolpathParams& effectiveParams)
{
    clearPocketSimulation();
    m_machining.installProfileProgram(
        std::move(job), requestedParams, effectiveParams);
}

MachiningProjectInstallResult DesktopWorkspaceCoordinator::installProject(
    MachiningProjectLoadResult loaded)
{
    clearSimulations();
    return m_machining.installProject(
        std::move(loaded), m_stockSimulation, m_recognition, m_document);
}

} // namespace oap::app

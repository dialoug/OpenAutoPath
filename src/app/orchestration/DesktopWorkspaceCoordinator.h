// 跨工作区的桌面事务与生命周期协调（oap_app_orchestration）。
// 聚合 Machining/ModelDocument/Playback/Recognition/StockSimulation 各
// 工作区协调器，负责跨工作区的状态事务（模型重置、项目恢复/安装、失效
// 广播）；MainWindow 只通过本门面触发跨区操作，不直接持有各工作区状态。
#pragma once

#include <utility>
#include <vector>

#include "app/workflows/project/MachiningProjectController.h"
#include "app/workflows/project/MachiningProjectInstaller.h"
#include "app/workflows/project/MachiningWorkspaceSession.h"
#include "manufacturing/job/MachiningJob.h"
#include "toolpath/core/ToolpathParams.h"

namespace oap::app {

class MachiningWorkspaceCoordinator;
class ModelDocumentWorkspaceCoordinator;
class PlaybackWorkspaceCoordinator;
class RecognitionWorkspaceCoordinator;
class StockSimulationWorkspaceCoordinator;

// Application-level transaction boundary for operations spanning multiple
// workspaces. UI code owns presentation widgets; this coordinator owns the
// ordering and consistency rules between domain workspaces.
class DesktopWorkspaceCoordinator final {
public:
    DesktopWorkspaceCoordinator(
        MachiningWorkspaceCoordinator& machining,
        StockSimulationWorkspaceCoordinator& stockSimulation,
        PlaybackWorkspaceCoordinator& playback,
        RecognitionWorkspaceCoordinator& recognition,
        ModelDocumentWorkspaceCoordinator& document) noexcept;

    bool backgroundBusy() const noexcept;
    void requestCancel() noexcept;
    void cancelAndWait();

    void resetForModel();
    void clearTransientState(bool resetHoleStageSources);
    void clearToolpathState();
    void clearPlaybackProgram();
    void clearHoleSimulation();
    void clearPocketSimulation();
    void clearSimulations();

    void publishPocketProgram(
        manufacturing::MachiningJob job,
        std::vector<PocketPreviewEntry> entries,
        const toolpath::ToolpathParams& requestedParams,
        const toolpath::ToolpathParams& effectiveParams);
    void publishProfileProgram(
        manufacturing::MachiningJob job,
        const toolpath::ToolpathParams& requestedParams,
        const toolpath::ToolpathParams& effectiveParams);

    MachiningProjectInstallResult installProject(
        MachiningProjectLoadResult loaded);

private:
    MachiningWorkspaceCoordinator& m_machining;
    StockSimulationWorkspaceCoordinator& m_stockSimulation;
    PlaybackWorkspaceCoordinator& m_playback;
    RecognitionWorkspaceCoordinator& m_recognition;
    ModelDocumentWorkspaceCoordinator& m_document;
};

} // namespace oap::app

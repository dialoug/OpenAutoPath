# Source ownership for app targets.

oap_module_sources(oap_app_view OAP_APP_VIEW_SOURCES
    src/app/view/ToolPreviewGeometryBuilder.cpp
    src/app/view/ToolPreviewGeometryBuilder.h
    src/app/view/HolePreviewGeometryBuilder.cpp
    src/app/view/HolePreviewGeometryBuilder.h
    src/app/view/DisplayMaterials.h
    src/app/view/IpwDisplayGeometry.h)

oap_module_sources(oap_app_presentation OAP_APP_PRESENTATION_SOURCES
    src/app/presentation/StepLoadPreviewPresenter.cpp
    src/app/presentation/StepLoadPreviewPresenter.h
    src/app/presentation/SceneController.cpp
    src/app/presentation/SceneController.h
    src/app/presentation/HoleStageHighlightPresenter.cpp
    src/app/presentation/HoleStageHighlightPresenter.h
    src/app/presentation/InspectionOverlayPresenter.cpp
    src/app/presentation/InspectionOverlayPresenter.h
    src/app/presentation/IpwVisualAcceptanceCapture.cpp
    src/app/presentation/IpwVisualAcceptanceCapture.h
    src/app/presentation/MultiDexelPrimitivesObject.cpp
    src/app/presentation/detail/MultiDexelPrimitivesObject.h
    src/app/presentation/PlaybackRemainingPathObject.cpp
    src/app/presentation/detail/PlaybackRemainingPathObject.h
    src/app/presentation/PlaybackPresenter.cpp
    src/app/presentation/PlaybackPresenter.h
    src/app/presentation/RecognitionPresentationCoordinator.cpp
    src/app/presentation/RecognitionPresentationCoordinator.h
    src/app/presentation/ToolpathPreviewPresenter.cpp
    src/app/presentation/ToolpathPreviewPresenter.h
    src/app/presentation/ToolpathSegmentsObject.cpp
    src/app/presentation/ToolpathSegmentsObject.h
    src/app/presentation/PlaybackOverlayPolicy.h
    src/app/presentation/StockSimulationPresenter.cpp
    src/app/presentation/StockSimulationPresenter.h
    src/app/presentation/OverlayController.cpp
    src/app/presentation/OverlayController.h)

oap_module_sources(oap_app_playback OAP_APP_PLAYBACK_SOURCES
    src/app/playback/PlaybackCameraFollower.cpp
    src/app/playback/PlaybackCameraFollower.h
    src/app/playback/PlaybackPanelViewModel.cpp
    src/app/playback/PlaybackPanelViewModel.h
    src/app/playback/PlaybackWorkspaceCoordinator.cpp
    src/app/playback/PlaybackWorkspaceCoordinator.h)

oap_module_sources(oap_app_playback_ui OAP_APP_PLAYBACK_UI_SOURCES
    src/app/ui/playback/PlaybackUiController.cpp
    src/app/ui/playback/PlaybackUiController.h
    src/app/ui/panels/ToolpathPlayerPanel.cpp
    src/app/ui/panels/ToolpathPlayerPanel.h)

oap_module_sources(oap_app_recognition_ui OAP_APP_RECOGNITION_UI_SOURCES
    src/app/ui/recognition/RecognitionSelectionUiController.cpp
    src/app/ui/recognition/RecognitionSelectionUiController.h)

oap_module_sources(oap_app_document_ui OAP_APP_DOCUMENT_UI_SOURCES
    src/app/ui/document/DocumentUiController.cpp
    src/app/ui/document/DocumentUiController.h
    src/app/ui/panels/FileListPanel.cpp
    src/app/ui/panels/FileListPanel.h)

oap_module_sources(oap_app_cam_ui OAP_APP_CAM_UI_SOURCES
    src/app/ui/cam/CamUiController.cpp
    src/app/ui/cam/CamUiController.h
    src/app/ui/panels/CamWorkflowPanel.cpp
    src/app/ui/panels/CamWorkflowPanel.h
    src/app/ui/panels/HoleToolTreePanel.cpp
    src/app/ui/panels/HoleToolTreePanel.h
    src/app/ui/panels/HoleCoverageAuditPanel.cpp
    src/app/ui/panels/HoleCoverageAuditPanel.h
    src/app/ui/panels/FeatureAllowancePanel.cpp
    src/app/ui/panels/FeatureAllowancePanel.h)

oap_module_sources(oap_app_simulation_ui OAP_APP_SIMULATION_UI_SOURCES
    src/app/ui/simulation/SimulationUiController.cpp
    src/app/ui/simulation/SimulationUiController.h)

oap_module_sources(oap_app_simulation OAP_APP_SIMULATION_SOURCES
    src/app/simulation/StockSimulationWorkspaceCoordinator.cpp
    src/app/simulation/StockSimulationWorkspaceCoordinator.h)

oap_module_sources(oap_app_recognition OAP_APP_RECOGNITION_SOURCES
    src/app/recognition/RecognitionWorkspaceCoordinator.cpp
    src/app/recognition/RecognitionWorkspaceCoordinator.h)

oap_module_sources(oap_app_scene OAP_APP_SCENE_SOURCES
    src/app/scene/SceneInteractionCoordinator.cpp
    src/app/scene/SceneInteractionCoordinator.h)

oap_module_sources(oap_app_document OAP_APP_DOCUMENT_SOURCES
    src/app/document/ModelDocumentWorkspaceCoordinator.cpp
    src/app/document/ModelDocumentWorkspaceCoordinator.h)



oap_module_sources(oap_app_machining OAP_APP_MACHINING_SOURCES
    src/app/machining/MachiningWorkspaceFinishing.cpp
    src/app/machining/MachiningWorkspaceCoordinator.cpp
    src/app/machining/MachiningWorkspaceCoordinator.h)

oap_module_sources(oap_app_orchestration OAP_APP_ORCHESTRATION_SOURCES
    src/app/orchestration/DesktopWorkspaceCoordinator.cpp
    src/app/orchestration/DesktopWorkspaceCoordinator.h)

oap_module_sources(oap_app_analysis OAP_APP_ANALYSIS_SOURCES
    src/app/analysis/AnalysisLoadResult.h
    src/app/analysis/StepAnalysisCache.cpp
    src/app/analysis/StepAnalysisCache.h
    src/app/analysis/AnalysisLoadController.cpp
    src/app/analysis/AnalysisLoadController.h
    src/app/analysis/AnalysisWorker.cpp
    src/app/analysis/AnalysisWorker.h
    src/app/analysis/ThreeAxisRecognitionController.cpp
    src/app/analysis/ThreeAxisRecognitionController.h)

oap_module_sources(oap_app_panels OAP_APP_PANEL_SOURCES
    src/app/panels/ResultPanel.cpp
    src/app/panels/ResultPanel.h
    src/app/panels/FeaturePanel.cpp
    src/app/panels/FeaturePanel.h
    src/app/panels/GroupPanel.cpp
    src/app/panels/GroupPanel.h
    src/app/panels/StickyTreeWidget.cpp
    src/app/panels/StickyTreeWidget.h)

set(OAP_APP_WORKFLOW_ALL_SOURCES
    src/app/workflows/project/DirectToolSource.h
    src/app/workflows/project/DirectToolSource.cpp
    src/app/workflows/project/DirectToolSettingsStorage.h
    src/app/workflows/project/DirectToolSettingsStorage.cpp
    src/app/workflows/project/detail/ToolContextSettingsJson.h
    src/app/workflows/project/detail/ToolContextSettingsJson.cpp
    src/app/workflows/engraving/EngravingSettings.h
    src/app/workflows/engraving/EngravingToolpathGenerationUseCase.h
    src/app/workflows/engraving/EngravingToolpathGenerationUseCase.cpp
    src/app/workflows/project/EngravingSettingsStorage.h
    src/app/workflows/project/EngravingSettingsStorage.cpp
    src/app/workflows/simulation/ProjectStockSimulationUseCase.cpp
    src/app/workflows/shared/ModelDocumentSession.cpp
    src/app/workflows/shared/ModelDocumentSession.h
    src/app/workflows/shared/RecognitionSession.cpp
    src/app/workflows/shared/RecognitionSession.h
    src/app/workflows/shared/SelectionCoordinator.cpp
    src/app/workflows/shared/SelectionCoordinator.h
    src/app/workflows/shared/UnitDisplayGrouping.cpp
    src/app/workflows/shared/UnitDisplayGrouping.h
    src/app/workflows/shared/UndoHistory.cpp
    src/app/workflows/shared/UndoHistory.h
    src/app/workflows/hole/HoleJobSourceSynchronizer.cpp
    src/app/workflows/hole/HoleJobSourceSynchronizer.h
    src/app/workflows/hole/HoleToolpathRunner.cpp
    src/app/workflows/hole/HoleToolpathRunner.h
    src/app/workflows/hole/HoleToolpathGenerationUseCase.cpp
    src/app/workflows/hole/HoleToolpathGenerationUseCase.h
    src/app/workflows/hole/HoleToolpathCommandPolicy.h
    src/app/workflows/hole/HoleToolpathWorkflowController.cpp
    src/app/workflows/hole/HoleToolpathWorkflowController.h
    src/app/workflows/hole/HoleStockSimulationRunner.cpp
    src/app/workflows/hole/HoleStockSimulationRunner.h
    src/app/workflows/hole/HoleStockGeometryBuilder.cpp
    src/app/workflows/hole/HoleStockGeometryBuilder.h
    src/app/workflows/simulation/IpwDisplayArtifactCache.cpp
    src/app/workflows/simulation/detail/IpwDisplayArtifactCache.h
    src/app/workflows/simulation/IpwDisplayMeshController.cpp
    src/app/workflows/simulation/IpwDisplayMeshController.h
    src/app/workflows/simulation/IpwDisplayMeshWorker.cpp
    src/app/workflows/simulation/IpwDisplayMeshWorker.h
    src/app/workflows/simulation/IpwSimulationRunner.cpp
    src/app/workflows/simulation/detail/IpwSimulationRunner.h
    src/app/workflows/project/MachiningConfigurationSession.cpp
    src/app/workflows/project/MachiningConfigurationSession.h
    src/app/workflows/project/ToolpathParameterEditorState.h
    src/app/workflows/project/MachiningProjectController.cpp
    src/app/workflows/project/MachiningProjectController.h
    src/app/workflows/project/MachiningProjectInstaller.cpp
    src/app/workflows/project/MachiningProjectInstaller.h
    src/app/workflows/project/MachiningWorkspaceSession.cpp
    src/app/workflows/project/MachiningWorkspaceSession.h
    src/app/workflows/playback/PlaybackProgramAssembler.cpp
    src/app/workflows/playback/PlaybackProgramAssembler.h
    src/app/workflows/playback/PlaybackSession.cpp
    src/app/workflows/playback/PlaybackSession.h
    src/app/workflows/hole/HoleToolpathWorkbenchCoordinator.cpp
    src/app/workflows/hole/HoleToolpathWorkbenchCoordinator.h
      src/app/workflows/pocket/PocketToolpathRunner.cpp
      src/app/workflows/pocket/detail/PocketToolpathRunner.h
      src/app/workflows/pocket/PocketToolpathGenerationUseCase.cpp
      src/app/workflows/pocket/PocketToolpathGenerationUseCase.h
      src/app/workflows/pocket/PocketToolpathWorkflowController.cpp
      src/app/workflows/pocket/PocketToolpathWorkflowController.h
      src/app/workflows/pocket/PocketCamSimulatorTypes.h
      src/app/workflows/profile/ProfileToolpathRunner.cpp
      src/app/workflows/profile/ProfileToolpathRunner.h
      src/app/workflows/profile/ProfileToolpathGenerationUseCase.cpp
      src/app/workflows/profile/ProfileToolpathGenerationUseCase.h
      src/app/workflows/profile/ProfileToolpathWorkflowController.cpp
      src/app/workflows/profile/ProfileToolpathWorkflowController.h
      src/app/workflows/profile/ProfileSetupContextAssigner.h








      src/app/workflows/playback/ToolpathPlaybackController.cpp
    src/app/workflows/playback/ToolpathPlaybackController.h
    src/app/workflows/playback/ToolpathPlaybackWorker.cpp
    src/app/workflows/playback/ToolpathPlaybackWorker.h
    src/app/workflows/shared/ToolpathExportController.cpp
    src/app/workflows/shared/ToolpathExportController.h
    src/app/workflows/shared/ToolpathGenerationValidation.h
    src/app/workflows/project/ToolpathParameterSettings.cpp
    src/app/workflows/project/ToolpathParameterSettings.h
    src/app/workflows/shared/ToolpathRequestBuilder.cpp
    src/app/workflows/shared/detail/ToolpathRequestBuilder.h
    src/app/workflows/shared/WorkflowInteraction.cpp
    src/app/workflows/shared/WorkflowInteraction.h
    src/app/workflows/shared/ReferenceNcExportService.cpp
    src/app/workflows/shared/ReferenceNcExportService.h
    src/app/workflows/simulation/StockSimulationSession.cpp
    src/app/workflows/simulation/StockSimulationSession.h
    src/app/workflows/simulation/StockSimulationCoordinator.cpp
    src/app/workflows/simulation/StockSimulationCoordinator.h
    src/app/workflows/simulation/StockSimulationViewModels.h
      src/app/workflows/simulation/StockSimulationWorkflowController.cpp
      src/app/workflows/simulation/StockSimulationWorkflowController.h
      src/app/workflows/simulation/StockSimulationUseCases.cpp
      src/app/workflows/simulation/StockSimulationUseCases.h
    src/app/workflows/project/ToolLibraryController.cpp
    src/app/workflows/project/ToolLibraryController.h)

# Keep physical workflow ownership aligned with CheckWorkflowBoundaries.cmake.
# The four targets form a one-way dependency graph:
# base <- playback <- cam <- simulation.
oap_module_sources(oap_app_workflow_base OAP_APP_WORKFLOW_BASE_SOURCES ${OAP_APP_WORKFLOW_ALL_SOURCES})
list(FILTER OAP_APP_WORKFLOW_BASE_SOURCES INCLUDE REGEX
    "^src/app/workflows/shared/")
list(REMOVE_ITEM OAP_APP_WORKFLOW_BASE_SOURCES
    src/app/workflows/shared/UndoHistory.cpp
    src/app/workflows/shared/UndoHistory.h
    src/app/workflows/shared/ToolpathExportController.cpp
    src/app/workflows/shared/ToolpathExportController.h
    src/app/workflows/shared/ToolpathGenerationValidation.h
    src/app/workflows/shared/ToolpathRequestBuilder.cpp
    src/app/workflows/shared/detail/ToolpathRequestBuilder.h)

oap_module_sources(oap_app_workflow_playback OAP_APP_WORKFLOW_PLAYBACK_SOURCES ${OAP_APP_WORKFLOW_ALL_SOURCES})
list(FILTER OAP_APP_WORKFLOW_PLAYBACK_SOURCES INCLUDE REGEX
    "^src/app/workflows/playback/")

oap_module_sources(oap_app_workflow_simulation OAP_APP_WORKFLOW_SIMULATION_SOURCES ${OAP_APP_WORKFLOW_ALL_SOURCES})
list(FILTER OAP_APP_WORKFLOW_SIMULATION_SOURCES INCLUDE REGEX
    "^src/app/workflows/simulation/")
list(APPEND OAP_APP_WORKFLOW_SIMULATION_SOURCES
    src/app/workflows/hole/HoleStockSimulationRunner.cpp
    src/app/workflows/hole/HoleStockSimulationRunner.h)

oap_module_sources(oap_app_workflow_cam OAP_APP_WORKFLOW_CAM_SOURCES ${OAP_APP_WORKFLOW_ALL_SOURCES})
list(REMOVE_ITEM OAP_APP_WORKFLOW_CAM_SOURCES
    ${OAP_APP_WORKFLOW_BASE_SOURCES}
    ${OAP_APP_WORKFLOW_PLAYBACK_SOURCES}
    ${OAP_APP_WORKFLOW_SIMULATION_SOURCES})

oap_module_sources(oap_app_dialogs OAP_APP_DIALOG_SOURCES
    src/app/dialogs/DirectToolParameterSection.h
    src/app/dialogs/DirectToolParameterSection.cpp
    src/app/dialogs/EngravingParameterSection.h
    src/app/dialogs/EngravingParameterSection.cpp
    src/app/dialogs/QtWorkflowInteraction.cpp
    src/app/dialogs/QtWorkflowInteraction.h
    src/app/dialogs/StockSimulationDialogs.cpp
    src/app/dialogs/StockSimulationDialogs.h
    src/app/dialogs/ToolpathParameterDialog.cpp
    src/app/dialogs/ToolpathParameterDialog.h)

oap_module_sources(OpenAutoPathApp OAP_APP_SOURCES
    src/app/ui/shell/MainWindowFinishing.cpp
    src/app/ui/main.cpp
    src/app/ui/shell/MainWindow.cpp
    src/app/ui/shell/MainWindowDocument.cpp
    src/app/ui/shell/MainWindowHoleWorkflow.cpp
    src/app/ui/shell/MainWindowPlayback.cpp
    src/app/ui/shell/MainWindowRecognition.cpp
    src/app/ui/shell/MainWindowRecognitionCommands.cpp
    src/app/ui/shell/MainWindowSelection.cpp
    src/app/ui/shell/MainWindowSimulation.cpp
    src/app/ui/shell/MainWindowToolpath.cpp
    src/app/ui/shell/MainWindowUi.cpp
    src/app/ui/shell/MainWindowUiAssembly.cpp
    src/app/ui/shell/MainWindowView.cpp
    src/app/ui/shell/MainWindow.h
    src/app/ui/shell/MainWindowUiBuilder.cpp
    src/app/ui/shell/MainWindowUiBuilder.h
    src/app/ui/dialogs/PocketCamSimulatorDialog.cpp
    src/app/ui/dialogs/PocketCamSimulatorDialog.h
    src/app/ui/view/OcctView.cpp
    src/app/ui/view/OcctView.h
    src/app/ui/dialogs/ShortcutSettingsDialog.cpp
    src/app/ui/dialogs/ShortcutSettingsDialog.h
    src/app/ui/dialogs/GCodeReviewDialog.cpp
    src/app/ui/dialogs/GCodeReviewDialog.h
    src/app/ui/dialogs/ToolLibraryDialog.cpp
    src/app/ui/dialogs/ToolLibraryDialog.h
    src/app/ui/dialogs/detail/PocketCamView.h
    src/app/ui/dialogs/detail/PocketCamView.cpp
    src/app/ui/dialogs/detail/PocketCamVulkanWidget.h
    src/app/ui/dialogs/detail/PocketCamVulkanDexel.h
    src/app/ui/dialogs/detail/PocketCamVulkanDexel.cpp
    src/app/ui/dialogs/detail/PocketCamVulkanPlayback.cpp
    src/app/ui/dialogs/detail/PocketCamVulkanRendering.cpp
    src/app/ui/dialogs/detail/PocketCamDialog.h
    src/app/ui/dialogs/detail/PocketCamDialog.cpp
    src/app/ui/dialogs/detail/PocketCamOpenGLWidget.h
    src/app/ui/dialogs/detail/PocketCamOpenGLWidget.cpp
    src/app/ui/dialogs/detail/PocketCamOpenGLCamera.cpp
    src/app/ui/dialogs/detail/PocketCamOpenGLRendering.cpp
    src/app/ui/dialogs/detail/PocketCamGpuDexelPrograms.cpp
    src/app/ui/dialogs/detail/PocketCamGpuDexelStock.cpp
    src/app/ui/dialogs/detail/PocketCamGpuDexelRendering.cpp
    src/app/ui/dialogs/detail/PocketCamOpenGLPlayback.cpp
    src/app/ui/dialogs/detail/PocketCamRenderSupport.h
    src/app/ui/dialogs/detail/PocketCamRenderSupport.cpp)

# Simulation presentation is a linkable module shared by the desktop and GPU tests.
oap_module_sources(oap_app_cam_renderer OAP_APP_CAM_RENDERER_SOURCES ${OAP_APP_SOURCES})
list(FILTER OAP_APP_CAM_RENDERER_SOURCES INCLUDE REGEX "/PocketCam[^/]*\\.(cpp|h)$")
list(REMOVE_ITEM OAP_APP_SOURCES ${OAP_APP_CAM_RENDERER_SOURCES})

# Desktop and presentation smoke-test targets.
add_executable(oap_test_cam_renderer ${OAP_TEST_CAM_RENDERER_SOURCES})
target_link_libraries(oap_test_cam_renderer PRIVATE OAP::AppCamRenderer OAP::AppWorkflowSimulation)
oap_configure_occt_viewer_target(oap_test_cam_renderer PRIVATE)
set_target_properties(oap_test_cam_renderer PROPERTIES AUTOMOC ON)
oap_configure_fast_test(oap_test_cam_renderer)

add_executable(oap_test_step_load ${OAP_TEST_STEP_LOAD_SOURCES})
target_link_libraries(oap_test_step_load PRIVATE OAP::AppDocument OAP::ToolpathCore)
oap_configure_fast_test(oap_test_step_load)

add_executable(oap_test_toolpath_parameter_dialog ${OAP_TEST_TOOLPATH_PARAMETER_DIALOG_SOURCES})
target_link_libraries(oap_test_toolpath_parameter_dialog PRIVATE OAP::AppDialogs Qt6::Widgets)
oap_configure_fast_test(oap_test_toolpath_parameter_dialog)

add_executable(oap_test_surface_finishing_project ${OAP_TEST_SURFACE_FINISHING_PROJECT_SOURCES})
target_link_libraries(oap_test_surface_finishing_project PRIVATE
    OAP::AppMachining OAP::AppWorkflowSimulation)
oap_configure_fast_test(oap_test_surface_finishing_project)

add_executable(oap_test_tool_preview_geometry
    ${OAP_TEST_TOOL_PREVIEW_GEOMETRY_SOURCES})
target_link_libraries(oap_test_tool_preview_geometry PRIVATE
    OAP::AppView)
oap_configure_fast_test(oap_test_tool_preview_geometry)

add_executable(oap_test_ipw_display_modes
    ${OAP_TEST_IPW_DISPLAY_MODES_SOURCES})
target_link_libraries(oap_test_ipw_display_modes PRIVATE
    OAP::AppWorkflowSimulation)
oap_configure_fast_test(oap_test_ipw_display_modes)

add_executable(oap_test_stock_geometry_builder
    ${OAP_TEST_STOCK_GEOMETRY_BUILDER_SOURCES})
target_link_libraries(oap_test_stock_geometry_builder PRIVATE
    OAP::AppWorkflowSimulation)
oap_configure_fast_test(oap_test_stock_geometry_builder)

add_executable(oap_test_task_lifecycle
    ${OAP_TEST_TASK_LIFECYCLE_SOURCES})
target_link_libraries(oap_test_task_lifecycle PRIVATE
    Qt6::Core OAP::ToolpathCore OAP::AppAnalysis
    OAP::AppWorkflowPlayback OAP::ToolpathWorkflow)
set_target_properties(oap_test_task_lifecycle PROPERTIES AUTOMOC ON)
oap_configure_fast_test(oap_test_task_lifecycle)

add_executable(oap_test_overlay_controller
    ${OAP_TEST_OVERLAY_CONTROLLER_SOURCES})
target_link_libraries(oap_test_overlay_controller PRIVATE
    OAP::AppPresentation)
oap_configure_fast_test(oap_test_overlay_controller)

add_executable(oap_test_playback_workspace
    ${OAP_TEST_PLAYBACK_WORKSPACE_SOURCES})
target_link_libraries(oap_test_playback_workspace PRIVATE
    OAP::AppPlayback)
oap_configure_fast_test(oap_test_playback_workspace)

add_executable(oap_test_stock_simulation_workspace
    ${OAP_TEST_STOCK_SIMULATION_WORKSPACE_SOURCES})
target_link_libraries(oap_test_stock_simulation_workspace PRIVATE
    OAP::AppSimulation)
oap_configure_fast_test(oap_test_stock_simulation_workspace)

add_executable(oap_test_recognition_workspace
    ${OAP_TEST_RECOGNITION_WORKSPACE_SOURCES})
target_link_libraries(oap_test_recognition_workspace PRIVATE
    OAP::AppRecognition)
oap_configure_fast_test(oap_test_recognition_workspace)

add_executable(oap_test_selection_coordinator
    ${OAP_TEST_SELECTION_COORDINATOR_SOURCES})
target_link_libraries(oap_test_selection_coordinator PRIVATE
    OAP::AppWorkflowBase)
oap_configure_fast_test(oap_test_selection_coordinator)

add_executable(oap_test_scene_interaction
    ${OAP_TEST_SCENE_INTERACTION_SOURCES})
target_link_libraries(oap_test_scene_interaction PRIVATE
    OAP::AppScene)
oap_configure_fast_test(oap_test_scene_interaction)

add_executable(oap_test_document_workspace
    ${OAP_TEST_DOCUMENT_WORKSPACE_SOURCES})
target_link_libraries(oap_test_document_workspace PRIVATE
    OAP::AppDocument)
oap_configure_fast_test(oap_test_document_workspace)

add_executable(oap_test_ui_slice_controllers
    ${OAP_TEST_UI_SLICE_CONTROLLERS_SOURCES})
target_link_libraries(oap_test_ui_slice_controllers PRIVATE
    OAP::AppRecognitionUi
    OAP::AppDocumentUi
    OAP::AppCamUi
    OAP::AppSimulationUi
    OAP::AppPresentation)
oap_configure_fast_test(oap_test_ui_slice_controllers)

add_executable(oap_test_machining_workspace
    ${OAP_TEST_MACHINING_WORKSPACE_SOURCES})
target_link_libraries(oap_test_machining_workspace PRIVATE
    OAP::AppMachining)
oap_configure_fast_test(oap_test_machining_workspace)

add_executable(oap_test_desktop_workspace
    ${OAP_TEST_DESKTOP_WORKSPACE_SOURCES})
target_link_libraries(oap_test_desktop_workspace PRIVATE
    OAP::AppOrchestration)
oap_configure_fast_test(oap_test_desktop_workspace)

add_executable(oap_test_playback_session
    ${OAP_TEST_PLAYBACK_SESSION_SOURCES})
target_link_libraries(oap_test_playback_session PRIVATE
    OAP::AppWorkflowPlayback)
oap_configure_fast_test(oap_test_playback_session)

add_executable(oap_test_app_workflow_controllers
    ${OAP_TEST_APP_WORKFLOW_CONTROLLERS_SOURCES})
target_link_libraries(oap_test_app_workflow_controllers PRIVATE
    OAP::AppWorkflowCam
    OAP::AppWorkflowSimulation)
oap_configure_fast_test(oap_test_app_workflow_controllers)

add_executable(oap_test_machining_project_controller
    ${OAP_TEST_MACHINING_PROJECT_CONTROLLER_SOURCES})
target_link_libraries(oap_test_machining_project_controller PRIVATE
    OAP::AppWorkflowCam
    OAP::AppWorkflowSimulation)
oap_configure_fast_test(oap_test_machining_project_controller)

add_executable(oap_test_engraving_workflow ${OAP_TEST_ENGRAVING_WORKFLOW_SOURCES})
target_link_libraries(oap_test_engraving_workflow PRIVATE OAP::AppMachining OAP::AppWorkflowSimulation)
oap_configure_fast_test(oap_test_engraving_workflow)

add_executable(oap_test_single_tool_workflow ${OAP_TEST_SINGLE_TOOL_WORKFLOW_SOURCES})
target_link_libraries(oap_test_single_tool_workflow PRIVATE OAP::AppMachining OAP::AppWorkflowSimulation)
oap_configure_fast_test(oap_test_single_tool_workflow)

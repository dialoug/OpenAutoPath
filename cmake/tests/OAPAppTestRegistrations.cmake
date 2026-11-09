# Desktop CTest registrations.
oap_add_test_with_runtime(cam_renderer oap_test_cam_renderer)

oap_add_test_with_runtime(cam_renderer_gpu_material oap_test_cam_renderer --gpu-material)

oap_add_test_with_runtime(cam_renderer_switch_first oap_test_cam_renderer --switch-first)

oap_add_test_with_runtime(cam_renderer_switch_after_playback oap_test_cam_renderer --switch-after-playback)

set_tests_properties(cam_renderer cam_renderer_gpu_material cam_renderer_switch_first cam_renderer_switch_after_playback PROPERTIES
    ENVIRONMENT "QT_PLUGIN_PATH=${OAP_QT_PLUGIN_DIR}"
    TIMEOUT 120 SKIP_RETURN_CODE 77 RUN_SERIAL TRUE LABELS "gpu;ui;vulkan")

oap_add_test_with_runtime(step_load oap_test_step_load)

oap_add_test_with_runtime(toolpath_parameter_dialog oap_test_toolpath_parameter_dialog)

set_tests_properties(toolpath_parameter_dialog PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen;QT_PLUGIN_PATH=${OAP_QT_PLUGIN_DIR}")

oap_add_test_with_runtime(surface_finishing_project oap_test_surface_finishing_project)

oap_add_test_with_runtime(
    tool_preview_geometry
    oap_test_tool_preview_geometry)

oap_add_test_with_runtime(
    ipw_display_modes
    oap_test_ipw_display_modes)

oap_add_test_with_runtime(
    stock_geometry_builder
    oap_test_stock_geometry_builder)

oap_add_test_with_runtime(
    task_lifecycle
    oap_test_task_lifecycle)

oap_add_test_with_runtime(
    overlay_controller
    oap_test_overlay_controller)

oap_add_test_with_runtime(
    playback_workspace
    oap_test_playback_workspace)

oap_add_test_with_runtime(
    stock_simulation_workspace
    oap_test_stock_simulation_workspace)

oap_add_test_with_runtime(
    recognition_workspace
    oap_test_recognition_workspace)

oap_add_test_with_runtime(
    selection_coordinator
    oap_test_selection_coordinator)

oap_add_test_with_runtime(
    scene_interaction
    oap_test_scene_interaction)

oap_add_test_with_runtime(
    document_workspace
    oap_test_document_workspace)

oap_add_test_with_runtime(
    ui_slice_controllers
    oap_test_ui_slice_controllers)

oap_add_test_with_runtime(
    machining_workspace
    oap_test_machining_workspace)

oap_add_test_with_runtime(
    desktop_workspace
    oap_test_desktop_workspace)

oap_add_test_with_runtime(
    playback_session
    oap_test_playback_session)

oap_add_test_with_runtime(
    app_workflow_controllers
    oap_test_app_workflow_controllers)

oap_add_test_with_runtime(
    machining_project_controller
    oap_test_machining_project_controller)

oap_add_test_with_runtime(engraving_workflow oap_test_engraving_workflow)

oap_add_test_with_runtime(single_tool_workflow oap_test_single_tool_workflow)

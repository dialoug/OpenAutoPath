# CTest registration for smoke-test executables.
oap_add_test_with_runtime(cam_renderer oap_test_cam_renderer)
oap_add_test_with_runtime(cam_renderer_gpu_material oap_test_cam_renderer --gpu-material)
oap_add_test_with_runtime(cam_renderer_switch_first oap_test_cam_renderer --switch-first)
oap_add_test_with_runtime(cam_renderer_switch_after_playback oap_test_cam_renderer --switch-after-playback)
set_tests_properties(cam_renderer cam_renderer_gpu_material cam_renderer_switch_first cam_renderer_switch_after_playback PROPERTIES
    ENVIRONMENT "QT_PLUGIN_PATH=${OAP_QT_ROOT}/plugins"
    TIMEOUT 120 SKIP_RETURN_CODE 77 RUN_SERIAL TRUE LABELS "gpu;ui;vulkan")

oap_add_test_with_runtime(step_load oap_test_step_load)

oap_add_test_with_runtime(engraving_relief oap_test_engraving_relief)

oap_add_test_with_runtime(toolpath_parameter_dialog oap_test_toolpath_parameter_dialog)
set_tests_properties(toolpath_parameter_dialog PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen;QT_PLUGIN_PATH=${OAP_QT_ROOT}/plugins")

oap_add_test_with_runtime(surface_finishing oap_test_surface_finishing)
oap_add_test_with_runtime(surface_finishing_job oap_test_surface_finishing_job)
oap_add_test_with_runtime(surface_finishing_ipw oap_test_surface_finishing_ipw)
oap_add_test_with_runtime(surface_finishing_project oap_test_surface_finishing_project)

oap_add_test_with_runtime(fixed_axis_execution oap_test_fixed_axis_execution)
oap_add_test_with_runtime(profile_direction_planning oap_test_profile_direction_planning)

oap_add_test_with_runtime(workflow_contract oap_test_workflow_contract)
oap_add_test_with_runtime(profile_invalid_parameters
    oap_test_workflow_contract --profile-invalid-parameters)

oap_add_test_with_runtime(heavy_job_scheduler oap_test_heavy_job_scheduler)
oap_add_test_with_runtime(multi_dexel_single_thread_budget oap_test_multi_dexel_ipw)
set_tests_properties(multi_dexel_single_thread_budget PROPERTIES ENVIRONMENT "OAP_CAM_MAX_THREADS=1")
oap_add_test_with_runtime(multi_dexel_parallel_budget oap_test_multi_dexel_ipw)
set_tests_properties(multi_dexel_parallel_budget PROPERTIES ENVIRONMENT "OAP_CAM_MAX_THREADS=4")
oap_add_test_with_runtime(recognition_single_thread_budget oap_test_three_axis_recognition)
set_tests_properties(recognition_single_thread_budget PROPERTIES ENVIRONMENT "OAP_CAM_MAX_THREADS=1")
oap_add_test_with_runtime(recognition_parallel_budget oap_test_three_axis_recognition)
set_tests_properties(recognition_parallel_budget PROPERTIES ENVIRONMENT "OAP_CAM_MAX_THREADS=4")


oap_add_test_with_runtime(model_index oap_test_model_index)

oap_add_test_with_runtime(
    dexel_interval
    oap_test_dexel_interval)
oap_add_test_with_runtime(
    dexel_tool_profile
    oap_test_dexel_tool_profile)
oap_add_test_with_runtime(
    tool_preview_geometry
    oap_test_tool_preview_geometry)
oap_add_test_with_runtime(
    machining_job
    oap_test_machining_job)
oap_add_test_with_runtime(
    machining_program_assembler
    oap_test_machining_program_assembler)
oap_add_test_with_runtime(
    toolpath_program
    oap_test_toolpath_program)
oap_add_test_with_runtime(
    occt_recognition
    oap_test_occt_recognition)
oap_add_test_with_runtime(
    multi_dexel_ipw
    oap_test_multi_dexel_ipw)
oap_add_test_with_runtime(
    pocket_layer_linking
    oap_test_pocket_layer_linking)
oap_add_test_with_runtime(
    profile_contour
    oap_test_profile_contour)
oap_add_test_with_runtime(
    pocket_automatic_toolpath
    oap_test_pocket_automatic_toolpath)
oap_add_test_with_runtime(
    pocket_parallel_budget
    oap_test_pocket_automatic_toolpath --parallel-budget)
set_tests_properties(pocket_parallel_budget PROPERTIES
    ENVIRONMENT "OAP_CAM_MAX_THREADS=4" TIMEOUT 60)
oap_add_test_with_runtime(
    pocket_single_thread_budget
    oap_test_pocket_automatic_toolpath --parallel-budget)
set_tests_properties(pocket_single_thread_budget PROPERTIES
    ENVIRONMENT "OAP_CAM_MAX_THREADS=1" TIMEOUT 60)
oap_add_test_with_runtime(
    pocket_hardware_budget
    oap_test_pocket_automatic_toolpath --parallel-budget)
set_tests_properties(pocket_hardware_budget PROPERTIES
    ENVIRONMENT "OAP_CAM_MAX_THREADS=999" TIMEOUT 60)
oap_add_test_with_runtime(
    profile_automatic_toolpath
    oap_test_profile_automatic_toolpath)
oap_add_test_with_runtime(
    ipw_display_modes
    oap_test_ipw_display_modes)
oap_add_test_with_runtime(
    stock_geometry_builder
    oap_test_stock_geometry_builder)
oap_add_test_with_runtime(
    hole_batch_execution
    oap_test_hole_batch_execution)
oap_add_test_with_runtime(
    three_axis_recognition
    oap_test_three_axis_recognition)
oap_add_test_with_runtime(
    task_lifecycle
    oap_test_task_lifecycle)
oap_add_test_with_runtime(
    hole_batch_identity
    oap_test_hole_batch_identity)
oap_add_test_with_runtime(
    hole_ipw_reference
    oap_test_hole_ipw_reference)
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
    hole_dto_mapping
    oap_test_hole_dto_mapping)
oap_add_test_with_runtime(
    tool_type_parameter_schema
    oap_test_tool_type_parameter_schema)
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

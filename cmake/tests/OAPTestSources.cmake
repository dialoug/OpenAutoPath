# Source ownership for tests targets.
set(OAP_TEST_PROFILE_DIRECTION_PLANNING_SOURCES tests/smoke/toolpath/profile_direction_planning_smoke.cpp)
set(OAP_TEST_STEP_LOAD_SOURCES tests/smoke/app/step_load_smoke.cpp)

set(OAP_TEST_ENGRAVING_RELIEF_SOURCES
    tests/smoke/toolpath/engraving_relief_smoke.cpp)

set(OAP_TEST_SURFACE_FINISHING_SOURCES
    tests/smoke/toolpath/surface_finishing_smoke.cpp)

set(OAP_TEST_SURFACE_FINISHING_JOB_SOURCES
    tests/smoke/manufacturing/surface_finishing_job_smoke.cpp)
set(OAP_TEST_SURFACE_FINISHING_IPW_SOURCES
    tests/smoke/manufacturing/surface_finishing_ipw_smoke.cpp)
set(OAP_TEST_SURFACE_FINISHING_PROJECT_SOURCES
    tests/smoke/app/surface_finishing_project_smoke.cpp)


set(OAP_TEST_FIXED_AXIS_EXECUTION_SOURCES
    tests/smoke/toolpath/fixed_axis_execution_smoke.cpp)

set(OAP_TEST_WORKFLOW_CONTRACT_SOURCES
    tests/smoke/toolpath/workflow_contract_smoke.cpp)

set(OAP_TEST_MODEL_INDEX_SOURCES
    tests/smoke/analysis/model_index_smoke.cpp)

set(OAP_TEST_DEXEL_INTERVAL_SOURCES
    tests/smoke/toolpath/dexel_interval_smoke.cpp)

# Exact test/header pairs for storage ownership tests. This does not expose the
# header as public API or permit other tests to include module internals.
set(OAP_TEST_PRIVATE_HEADER_ACCESS
    "tests/smoke/app/cam_renderer_smoke.cpp|app/ui/dialogs/detail/PocketCamVulkanWidget.h"
    "tests/smoke/app/cam_renderer_smoke.cpp|app/ui/dialogs/detail/PocketCamOpenGLWidget.h"
    "tests/smoke/app/cam_renderer_smoke.cpp|app/ui/dialogs/detail/PocketCamDialog.h"
    "tests/smoke/toolpath/dexel_interval_smoke.cpp|toolpath/pocket/simulation/detail/DexelCellIntervals.h")

set(OAP_TEST_CAM_RENDERER_SOURCES tests/smoke/app/cam_renderer_smoke.cpp)

set(OAP_TEST_DEXEL_TOOL_PROFILE_SOURCES
    tests/smoke/toolpath/dexel_tool_profile_smoke.cpp)


set(OAP_TEST_TOOL_PREVIEW_GEOMETRY_SOURCES
    tests/smoke/app/tool_preview_geometry_smoke.cpp)

set(OAP_TEST_MACHINING_JOB_SOURCES
    tests/smoke/manufacturing/machining_job_smoke.cpp)

set(OAP_TEST_MACHINING_PROGRAM_ASSEMBLER_SOURCES
    tests/smoke/manufacturing/machining_program_assembler_smoke.cpp)

set(OAP_TEST_TOOLPATH_PROGRAM_SOURCES
    tests/smoke/toolpath/toolpath_program_smoke.cpp)

set(OAP_TEST_OCCT_RECOGNITION_SOURCES
    tests/smoke/recognition/occt_recognition_smoke.cpp)

set(OAP_TEST_MULTI_DEXEL_IPW_SOURCES
    tests/smoke/toolpath/multi_dexel_ipw_smoke.cpp)

set(OAP_TEST_POCKET_LAYER_LINKING_SOURCES
    tests/smoke/toolpath/pocket_layer_linking_smoke.cpp)

set(OAP_TEST_PROFILE_CONTOUR_SOURCES
    tests/smoke/toolpath/profile_contour_smoke.cpp)








set(OAP_TEST_POCKET_AUTOMATIC_TOOLPATH_SOURCES
    tests/smoke/tooling/pocket_automatic_toolpath_smoke.cpp)

set(OAP_TEST_PROFILE_AUTOMATIC_TOOLPATH_SOURCES
    tests/smoke/tooling/profile_automatic_toolpath_smoke.cpp)

set(OAP_TEST_IPW_DISPLAY_MODES_SOURCES
    tests/smoke/app/ipw_display_modes_smoke.cpp)

set(OAP_TEST_STOCK_GEOMETRY_BUILDER_SOURCES
    tests/smoke/app/stock_geometry_builder_smoke.cpp)

set(OAP_TEST_HOLE_BATCH_EXECUTION_SOURCES
    tests/smoke/toolpath/hole_batch_execution_smoke.cpp)

set(OAP_TEST_THREE_AXIS_RECOGNITION_SOURCES
    tests/smoke/recognition/RecognitionSampleBenchmark.cpp
    tests/smoke/recognition/RecognitionSampleBenchmark.h
    tests/smoke/recognition/three_axis_recognition_smoke.cpp)

set(OAP_TEST_TASK_LIFECYCLE_SOURCES
    tests/smoke/app/task_lifecycle_smoke.cpp)

set(OAP_TEST_HOLE_BATCH_IDENTITY_SOURCES
    tests/smoke/manufacturing/hole_batch_identity_smoke.cpp)

set(OAP_TEST_HOLE_IPW_REFERENCE_SOURCES
    tests/smoke/manufacturing/hole_ipw_reference_smoke.cpp)

set(OAP_TEST_OVERLAY_CONTROLLER_SOURCES
    tests/smoke/app/overlay_controller_smoke.cpp)

set(OAP_TEST_PLAYBACK_WORKSPACE_SOURCES
    tests/smoke/app/playback_workspace_smoke.cpp)

set(OAP_TEST_STOCK_SIMULATION_WORKSPACE_SOURCES
    tests/smoke/app/stock_simulation_workspace_smoke.cpp)

set(OAP_TEST_RECOGNITION_WORKSPACE_SOURCES
    tests/smoke/app/recognition_workspace_smoke.cpp)

set(OAP_TEST_HOLE_DTO_MAPPING_SOURCES
    tests/smoke/tooling/hole_dto_mapping_smoke.cpp)

set(OAP_TEST_TOOL_TYPE_PARAMETER_SCHEMA_SOURCES
    tests/smoke/tooling/tool_type_parameter_schema_smoke.cpp)

set(OAP_TEST_SELECTION_COORDINATOR_SOURCES
    tests/smoke/app/selection_coordinator_smoke.cpp)

set(OAP_TEST_SCENE_INTERACTION_SOURCES
    tests/smoke/app/scene_interaction_smoke.cpp)

set(OAP_TEST_DOCUMENT_WORKSPACE_SOURCES
    tests/smoke/app/document_workspace_smoke.cpp)



set(OAP_TEST_UI_SLICE_CONTROLLERS_SOURCES
    tests/smoke/app/ui_slice_controllers_smoke.cpp)

set(OAP_TEST_MACHINING_WORKSPACE_SOURCES
    tests/smoke/app/machining_workspace_smoke.cpp)

set(OAP_TEST_DESKTOP_WORKSPACE_SOURCES
    tests/smoke/app/desktop_workspace_smoke.cpp)

set(OAP_TEST_PLAYBACK_SESSION_SOURCES
    tests/smoke/app/playback_session_smoke.cpp)

set(OAP_TEST_APP_WORKFLOW_CONTROLLERS_SOURCES
    tests/smoke/app/app_workflow_controllers_smoke.cpp
    tests/smoke/app/pocket_workflow_checks.cpp)

set(OAP_TEST_MACHINING_PROJECT_CONTROLLER_SOURCES
    tests/smoke/app/machining_project_controller_smoke.cpp)
set(OAP_TEST_HEAVY_JOB_SCHEDULER_SOURCES
    tests/smoke/core/heavy_job_scheduler_smoke.cpp)
# Automatic process settings remain editable after manual planning removal.
set(OAP_TEST_TOOLPATH_PARAMETER_DIALOG_SOURCES
    tests/smoke/app/toolpath_parameter_dialog_smoke.cpp)

set(OAP_TEST_ENGRAVING_WORKFLOW_SOURCES tests/smoke/app/engraving_workflow_smoke.cpp)

set(OAP_TEST_SINGLE_TOOL_WORKFLOW_SOURCES tests/smoke/app/single_tool_workflow_smoke.cpp)

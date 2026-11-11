# Desktop and presentation smoke tests: each executable and its cases are declared together.

oap_add_smoke_test(cam_renderer
    SOURCES tests/smoke/app/cam_renderer_smoke.cpp
    LIBRARIES OAP::AppCamRenderer OAP::AppWorkflowSimulation
    AUTOMOC
    OCCT_VIEWER
    LABELS gpu ui vulkan
    TIMEOUT 120
    SKIP_RETURN_CODE 77
    RUN_SERIAL
    ENVIRONMENT "QT_PLUGIN_PATH=${OAP_QT_PLUGIN_DIR}")

oap_add_smoke_test_case(cam_renderer_gpu_material oap_test_cam_renderer
    ARGS --gpu-material
    LABELS gpu ui vulkan
    TIMEOUT 120
    SKIP_RETURN_CODE 77
    RUN_SERIAL
    ENVIRONMENT "QT_PLUGIN_PATH=${OAP_QT_PLUGIN_DIR}")

oap_add_smoke_test_case(cam_renderer_switch_first oap_test_cam_renderer
    ARGS --switch-first
    LABELS gpu ui vulkan
    TIMEOUT 120
    SKIP_RETURN_CODE 77
    RUN_SERIAL
    ENVIRONMENT "QT_PLUGIN_PATH=${OAP_QT_PLUGIN_DIR}")

oap_add_smoke_test_case(cam_renderer_switch_after_playback oap_test_cam_renderer
    ARGS --switch-after-playback
    LABELS gpu ui vulkan
    TIMEOUT 120
    SKIP_RETURN_CODE 77
    RUN_SERIAL
    ENVIRONMENT "QT_PLUGIN_PATH=${OAP_QT_PLUGIN_DIR}")

oap_add_smoke_test(step_load
    SOURCES tests/smoke/app/step_load_smoke.cpp
    LIBRARIES OAP::AppDocument OAP::ToolpathCore
    LABELS analysis app-workflow fast high-risk lifecycle
    TIMEOUT 90)

oap_add_smoke_test(toolpath_parameter_dialog
    SOURCES tests/smoke/app/toolpath_parameter_dialog_smoke.cpp
    LIBRARIES OAP::AppDialogs Qt6::Widgets
    LABELS app-workflow fast ui
    TIMEOUT 30
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen" "QT_PLUGIN_PATH=${OAP_QT_PLUGIN_DIR}")

oap_add_smoke_test(surface_finishing_project
    SOURCES tests/smoke/app/surface_finishing_project_smoke.cpp
    LIBRARIES OAP::AppMachining OAP::AppWorkflowSimulation
    LABELS fast finishing high-risk ipw manufacturing
    TIMEOUT 120)

oap_add_smoke_test(tool_preview_geometry
    SOURCES tests/smoke/app/tool_preview_geometry_smoke.cpp
    LIBRARIES OAP::AppView
    LABELS fast geometry
    TIMEOUT 30)

oap_add_smoke_test(ipw_display_modes
    SOURCES tests/smoke/app/ipw_display_modes_smoke.cpp
    LIBRARIES OAP::AppWorkflowSimulation
    LABELS app-workflow fast ipw multi-dexel occt
    TIMEOUT 30)

oap_add_smoke_test(stock_geometry_builder
    SOURCES tests/smoke/app/stock_geometry_builder_smoke.cpp
    LIBRARIES OAP::AppWorkflowSimulation
    LABELS app-workflow fast geometry
    TIMEOUT 30)

oap_add_smoke_test(task_lifecycle
    SOURCES tests/smoke/app/task_lifecycle_smoke.cpp
    LIBRARIES Qt6::Core OAP::ToolpathCore OAP::AppAnalysis OAP::AppWorkflowPlayback OAP::ToolpathWorkflow
    AUTOMOC
    LABELS app-workflow fast high-risk lifecycle
    TIMEOUT 30)

oap_add_smoke_test(overlay_controller
    SOURCES tests/smoke/app/overlay_controller_smoke.cpp
    LIBRARIES OAP::AppPresentation
    LABELS fast high-risk occt presentation
    TIMEOUT 30)

oap_add_smoke_test(playback_workspace
    SOURCES tests/smoke/app/playback_workspace_smoke.cpp
    LIBRARIES OAP::AppPlayback
    LABELS app-workspace fast high-risk occt presentation
    TIMEOUT 30)

oap_add_smoke_test(stock_simulation_workspace
    SOURCES tests/smoke/app/stock_simulation_workspace_smoke.cpp
    LIBRARIES OAP::AppSimulation
    LABELS app-workspace fast high-risk occt presentation
    TIMEOUT 30)

oap_add_smoke_test(recognition_workspace
    SOURCES tests/smoke/app/recognition_workspace_smoke.cpp
    LIBRARIES OAP::AppRecognition
    LABELS app-workspace fast high-risk occt presentation
    TIMEOUT 30)

oap_add_smoke_test(selection_coordinator
    SOURCES tests/smoke/app/selection_coordinator_smoke.cpp
    LIBRARIES OAP::AppWorkflowBase
    LABELS app-workflow fast
    TIMEOUT 30)

oap_add_smoke_test(scene_interaction
    SOURCES tests/smoke/app/scene_interaction_smoke.cpp
    LIBRARIES OAP::AppScene
    LABELS app-workflow fast
    TIMEOUT 30)

oap_add_smoke_test(document_workspace
    SOURCES tests/smoke/app/document_workspace_smoke.cpp
    LIBRARIES OAP::AppDocument
    LABELS app-workflow fast
    TIMEOUT 30)

oap_add_smoke_test(ui_slice_controllers
    SOURCES tests/smoke/app/ui_slice_controllers_smoke.cpp
    LIBRARIES OAP::AppRecognitionUi OAP::AppDocumentUi OAP::AppCamUi OAP::AppSimulationUi OAP::AppPresentation)

oap_add_smoke_test(machining_workspace
    SOURCES tests/smoke/app/machining_workspace_smoke.cpp
    LIBRARIES OAP::AppMachining
    LABELS app-workflow fast
    TIMEOUT 30)

oap_add_smoke_test(desktop_workspace
    SOURCES tests/smoke/app/desktop_workspace_smoke.cpp
    LIBRARIES OAP::AppOrchestration
    LABELS app-workflow fast
    TIMEOUT 30)

oap_add_smoke_test(playback_session
    SOURCES tests/smoke/app/playback_session_smoke.cpp
    LIBRARIES OAP::AppWorkflowPlayback
    LABELS app-workflow fast
    TIMEOUT 30)

oap_add_smoke_test(app_workflow_controllers
    SOURCES tests/smoke/app/app_workflow_controllers_smoke.cpp
        tests/smoke/app/pocket_workflow_checks.cpp
    LIBRARIES OAP::AppWorkflowCam OAP::AppWorkflowSimulation
    LABELS app-workflow fast
    TIMEOUT 30)

oap_add_smoke_test(machining_project_controller
    SOURCES tests/smoke/app/machining_project_controller_smoke.cpp
    LIBRARIES OAP::AppWorkflowCam OAP::AppWorkflowSimulation
    LABELS app-workflow high-risk manufacturing
    TIMEOUT 90)

oap_add_smoke_test(engraving_workflow
    SOURCES tests/smoke/app/engraving_workflow_smoke.cpp
    LIBRARIES OAP::AppMachining OAP::AppWorkflowSimulation
    LABELS app engraving fast high-risk
    TIMEOUT 120)

oap_add_smoke_test(single_tool_workflow
    SOURCES tests/smoke/app/single_tool_workflow_smoke.cpp
    LIBRARIES OAP::AppMachining OAP::AppWorkflowSimulation
    LABELS app-workflow fast high-risk
    TIMEOUT 180)

oap_add_smoke_test(language
    SOURCES tests/smoke/app/language_smoke.cpp
    LIBRARIES OAP::AppDialogs OAP::ToolLibrary Qt6::Widgets
    ARGS "${CMAKE_CURRENT_SOURCE_DIR}"
    LABELS app-workflow fast ui i18n
    TIMEOUT 30
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen" "QT_PLUGIN_PATH=${OAP_QT_PLUGIN_DIR}")
qt_add_resources(oap_test_language "language_test_translations"
    PREFIX "/i18n"
    BASE "resources/i18n"
    FILES resources/i18n/en.json
          resources/i18n/diagnostics_zh_CN.json
          resources/i18n/logs_zh_CN.json
          resources/i18n/labels.json
          resources/i18n/qt/qtbase_zh_CN.ts)

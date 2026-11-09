# Desktop test labels and timeouts.
set_tests_properties(step_load PROPERTIES LABELS "fast;analysis;app-workflow;lifecycle;high-risk" TIMEOUT 90)

set_tests_properties(toolpath_parameter_dialog PROPERTIES
    LABELS "fast;app-workflow;ui" TIMEOUT 30)

set_tests_properties(surface_finishing_project PROPERTIES
    LABELS "fast;finishing;manufacturing;ipw;high-risk" TIMEOUT 120)

set_tests_properties(tool_preview_geometry PROPERTIES
        LABELS "fast;geometry"
        TIMEOUT 30)

set_tests_properties(ipw_display_modes PROPERTIES
        LABELS "fast;app-workflow;ipw;multi-dexel;occt"
        TIMEOUT 30)

set_tests_properties(stock_geometry_builder PROPERTIES
        LABELS "fast;app-workflow;geometry"
        TIMEOUT 30)

set_tests_properties(task_lifecycle PROPERTIES
        LABELS "fast;app-workflow;lifecycle;high-risk"
        TIMEOUT 30)

set_tests_properties(overlay_controller PROPERTIES
        LABELS "fast;presentation;occt;high-risk"
        TIMEOUT 30)

set_tests_properties(playback_workspace stock_simulation_workspace recognition_workspace PROPERTIES
        LABELS "fast;app-workspace;presentation;occt;high-risk"
        TIMEOUT 30)

set_tests_properties(selection_coordinator scene_interaction document_workspace machining_workspace desktop_workspace playback_session app_workflow_controllers PROPERTIES
        LABELS "fast;app-workflow"
        TIMEOUT 30)

set_tests_properties(machining_project_controller PROPERTIES
        LABELS "app-workflow;manufacturing;high-risk"
        TIMEOUT 90)

set_tests_properties(engraving_workflow PROPERTIES LABELS "fast;app;engraving;high-risk" TIMEOUT 120)

set_tests_properties(single_tool_workflow PROPERTIES LABELS "fast;app-workflow;high-risk" TIMEOUT 180)

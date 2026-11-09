# Core test labels and timeouts.
set_tests_properties(profile_direction_planning PROPERTIES LABELS "fast;toolpath;profile;high-risk" TIMEOUT 90)

set_tests_properties(engraving_relief PROPERTIES LABELS "fast;toolpath;engraving;high-risk" TIMEOUT 120)

set_tests_properties(surface_finishing_job surface_finishing_ipw PROPERTIES
    LABELS "fast;finishing;manufacturing;ipw;high-risk" TIMEOUT 120)

set_tests_properties(surface_finishing PROPERTIES
    LABELS "fast;toolpath;finishing;high-risk" TIMEOUT 90)

set_tests_properties(fixed_axis_execution PROPERTIES
    LABELS "fast;toolpath;ipw;high-risk" TIMEOUT 30)

set_tests_properties(workflow_contract profile_invalid_parameters PROPERTIES
    LABELS "fast;toolpath;lifecycle;high-risk" TIMEOUT 30)

set_tests_properties(heavy_job_scheduler PROPERTIES LABELS "fast;lifecycle;high-risk" TIMEOUT 30)

set_tests_properties(multi_dexel_single_thread_budget multi_dexel_parallel_budget PROPERTIES LABELS "fast;ipw;multi-dexel;high-risk" TIMEOUT 60)

set_tests_properties(recognition_single_thread_budget recognition_parallel_budget PROPERTIES LABELS "fast;recognition;high-risk" TIMEOUT 60)

set_tests_properties(pocket_parallel_budget pocket_single_thread_budget pocket_hardware_budget PROPERTIES
    LABELS "fast;toolpath;pocket;high-risk" TIMEOUT 60)

set_tests_properties(model_index PROPERTIES LABELS "fast;analysis" TIMEOUT 30)

set_tests_properties(dexel_interval dexel_tool_profile PROPERTIES
        LABELS "fast;geometry"
        TIMEOUT 30)

set_tests_properties(machining_job PROPERTIES
        LABELS "fast;manufacturing;ipw"
        TIMEOUT 60)

set_tests_properties(machining_program_assembler PROPERTIES
        LABELS "fast;manufacturing;program"
        TIMEOUT 30)

set_tests_properties(toolpath_program PROPERTIES
        LABELS "fast;toolpath;gcode"
        TIMEOUT 30)

set_tests_properties(occt_recognition PROPERTIES
        LABELS "fast;geometry;recognition;occt"
        TIMEOUT 30)

set_tests_properties(multi_dexel_ipw PROPERTIES
        LABELS "fast;e2e;ipw;multi-dexel"
        TIMEOUT 30)

set_tests_properties(pocket_layer_linking PROPERTIES
        LABELS "fast;toolpath;pocket;high-risk"
        TIMEOUT 30)

set_tests_properties(hole_batch_execution PROPERTIES
        LABELS "fast;toolpath;hole"
        TIMEOUT 30)

set_tests_properties(three_axis_recognition PROPERTIES
        LABELS "fast;geometry;recognition;occt;high-risk"
        TIMEOUT 30)

set_tests_properties(hole_batch_identity PROPERTIES
        LABELS "fast;manufacturing;toolpath;hole;high-risk"
        TIMEOUT 30)

set_tests_properties(hole_ipw_reference PROPERTIES
        LABELS "fast;geometry;ipw;hole;occt;high-risk"
        TIMEOUT 60)

set_tests_properties(hole_dto_mapping PROPERTIES
        LABELS "fast;analysis;tooling;toolpath;hole;high-risk"
        TIMEOUT 30)

set_tests_properties(profile_contour profile_automatic_toolpath PROPERTIES
        LABELS "fast;toolpath;profile;high-risk"
        TIMEOUT 120)

set_tests_properties(tool_type_parameter_schema PROPERTIES
        LABELS "fast;tooling"
        TIMEOUT 30)

set_tests_properties(module_boundaries module_boundaries_forbidden_target module_boundaries_private_header module_boundaries_transitive_target module_boundaries_dependency_cycle module_boundaries_workflow_reverse_target workflow_internal_boundaries documentation_links PROPERTIES
        LABELS "fast;architecture"
        TIMEOUT 30)

if(OAP_BUILD_GUI)
    include(${CMAKE_CURRENT_LIST_DIR}/OAPAppTestLabels.cmake)
endif()

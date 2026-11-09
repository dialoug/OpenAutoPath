# Core CTest registrations.
oap_add_test_with_runtime(engraving_relief oap_test_engraving_relief)

oap_add_test_with_runtime(surface_finishing oap_test_surface_finishing)

oap_add_test_with_runtime(surface_finishing_job oap_test_surface_finishing_job)

oap_add_test_with_runtime(surface_finishing_ipw oap_test_surface_finishing_ipw)

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
    hole_batch_execution
    oap_test_hole_batch_execution)

oap_add_test_with_runtime(
    three_axis_recognition
    oap_test_three_axis_recognition)

oap_add_test_with_runtime(
    hole_batch_identity
    oap_test_hole_batch_identity)

oap_add_test_with_runtime(
    hole_ipw_reference
    oap_test_hole_ipw_reference)

oap_add_test_with_runtime(
    hole_dto_mapping
    oap_test_hole_dto_mapping)

oap_add_test_with_runtime(
    tool_type_parameter_schema
    oap_test_tool_type_parameter_schema)

if(OAP_BUILD_GUI)
    include(${CMAKE_CURRENT_LIST_DIR}/OAPAppTestRegistrations.cmake)
endif()

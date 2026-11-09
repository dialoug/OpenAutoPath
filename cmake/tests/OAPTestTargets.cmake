# Core smoke-test executable targets.
add_executable(oap_test_engraving_relief ${OAP_TEST_ENGRAVING_RELIEF_SOURCES})
target_link_libraries(oap_test_engraving_relief PRIVATE OAP::FinishingToolpath OAP::ToolpathProgram)
oap_configure_fast_test(oap_test_engraving_relief)

add_executable(oap_test_surface_finishing ${OAP_TEST_SURFACE_FINISHING_SOURCES})
target_link_libraries(oap_test_surface_finishing PRIVATE
    OAP::FinishingToolpath OAP::ToolpathProgram)
oap_configure_fast_test(oap_test_surface_finishing)

add_executable(oap_test_surface_finishing_job ${OAP_TEST_SURFACE_FINISHING_JOB_SOURCES})
target_link_libraries(oap_test_surface_finishing_job PRIVATE OAP::Manufacturing)
oap_configure_fast_test(oap_test_surface_finishing_job)

add_executable(oap_test_surface_finishing_ipw ${OAP_TEST_SURFACE_FINISHING_IPW_SOURCES})
target_link_libraries(oap_test_surface_finishing_ipw PRIVATE OAP::Manufacturing)
oap_configure_fast_test(oap_test_surface_finishing_ipw)

add_executable(oap_test_fixed_axis_execution ${OAP_TEST_FIXED_AXIS_EXECUTION_SOURCES})
target_link_libraries(oap_test_fixed_axis_execution PRIVATE OAP::ToolpathWorkflow)
oap_configure_fast_test(oap_test_fixed_axis_execution)

add_executable(oap_test_workflow_contract ${OAP_TEST_WORKFLOW_CONTRACT_SOURCES})
target_link_libraries(oap_test_workflow_contract PRIVATE OAP::ToolpathWorkflow)
oap_configure_fast_test(oap_test_workflow_contract)

add_executable(oap_test_heavy_job_scheduler
    ${OAP_TEST_HEAVY_JOB_SCHEDULER_SOURCES})
target_link_libraries(oap_test_heavy_job_scheduler PRIVATE OAP::ToolpathCore)
oap_configure_fast_test(oap_test_heavy_job_scheduler)

add_executable(oap_test_model_index ${OAP_TEST_MODEL_INDEX_SOURCES})
target_link_libraries(oap_test_model_index PRIVATE OAP::Analysis)
oap_configure_fast_test(oap_test_model_index)

add_executable(oap_test_dexel_interval
    ${OAP_TEST_DEXEL_INTERVAL_SOURCES})
target_link_libraries(oap_test_dexel_interval PRIVATE
    OAP::PocketToolpath)
oap_configure_fast_test(oap_test_dexel_interval)

add_executable(oap_test_dexel_tool_profile
    ${OAP_TEST_DEXEL_TOOL_PROFILE_SOURCES})
target_link_libraries(oap_test_dexel_tool_profile PRIVATE
    OAP::ToolpathCore)
oap_configure_fast_test(oap_test_dexel_tool_profile)

add_executable(oap_test_machining_job
    ${OAP_TEST_MACHINING_JOB_SOURCES})
target_link_libraries(oap_test_machining_job PRIVATE
    OAP::Manufacturing)
oap_configure_fast_test(oap_test_machining_job)

add_executable(oap_test_machining_program_assembler
    ${OAP_TEST_MACHINING_PROGRAM_ASSEMBLER_SOURCES})
target_link_libraries(oap_test_machining_program_assembler PRIVATE
    OAP::Manufacturing)
oap_configure_fast_test(oap_test_machining_program_assembler)

add_executable(oap_test_toolpath_program
    ${OAP_TEST_TOOLPATH_PROGRAM_SOURCES})
target_link_libraries(oap_test_toolpath_program PRIVATE
    OAP::ToolpathProgram)
oap_configure_fast_test(oap_test_toolpath_program)

add_executable(oap_test_occt_recognition
    ${OAP_TEST_OCCT_RECOGNITION_SOURCES})
target_link_libraries(oap_test_occt_recognition PRIVATE
    OAP::Recognition)
oap_configure_fast_test(oap_test_occt_recognition)

add_executable(oap_test_multi_dexel_ipw
    ${OAP_TEST_MULTI_DEXEL_IPW_SOURCES})
target_link_libraries(oap_test_multi_dexel_ipw PRIVATE
    OAP::PocketToolpath)
oap_configure_fast_test(oap_test_multi_dexel_ipw)

add_executable(oap_test_pocket_layer_linking
    ${OAP_TEST_POCKET_LAYER_LINKING_SOURCES})
target_link_libraries(oap_test_pocket_layer_linking PRIVATE
    OAP::PocketToolpath)
oap_configure_fast_test(oap_test_pocket_layer_linking)

add_executable(oap_test_profile_contour
    ${OAP_TEST_PROFILE_CONTOUR_SOURCES})
target_link_libraries(oap_test_profile_contour PRIVATE
    OAP::ProfileToolpath
    OAP::ToolpathWorkflow
    OAP::Recognition)
oap_configure_fast_test(oap_test_profile_contour)

add_executable(oap_test_profile_direction_planning ${OAP_TEST_PROFILE_DIRECTION_PLANNING_SOURCES})
target_link_libraries(oap_test_profile_direction_planning PRIVATE OAP::ProfileToolpath OAP::Recognition OAP::ToolpathAdapter)
oap_configure_fast_test(oap_test_profile_direction_planning)

add_executable(oap_test_pocket_automatic_toolpath
    ${OAP_TEST_POCKET_AUTOMATIC_TOOLPATH_SOURCES})
target_link_libraries(oap_test_pocket_automatic_toolpath PRIVATE
    OAP::ToolpathAdapter)
oap_configure_fast_test(oap_test_pocket_automatic_toolpath)

add_executable(oap_test_profile_automatic_toolpath
    ${OAP_TEST_PROFILE_AUTOMATIC_TOOLPATH_SOURCES})
target_link_libraries(oap_test_profile_automatic_toolpath PRIVATE
    OAP::ToolpathAdapter)
oap_configure_fast_test(oap_test_profile_automatic_toolpath)

add_executable(oap_test_hole_batch_execution
    ${OAP_TEST_HOLE_BATCH_EXECUTION_SOURCES})
target_link_libraries(oap_test_hole_batch_execution PRIVATE
    OAP::ToolpathWorkflow)
oap_configure_fast_test(oap_test_hole_batch_execution)

add_executable(oap_test_three_axis_recognition
    ${OAP_TEST_THREE_AXIS_RECOGNITION_SOURCES})
target_link_libraries(oap_test_three_axis_recognition PRIVATE
    OAP::Recognition OAP::ToolpathCore)
oap_configure_fast_test(oap_test_three_axis_recognition)

add_executable(oap_test_hole_batch_identity
    ${OAP_TEST_HOLE_BATCH_IDENTITY_SOURCES})
target_link_libraries(oap_test_hole_batch_identity PRIVATE
    OAP::Manufacturing)
oap_configure_fast_test(oap_test_hole_batch_identity)

add_executable(oap_test_hole_ipw_reference
    ${OAP_TEST_HOLE_IPW_REFERENCE_SOURCES})
target_link_libraries(oap_test_hole_ipw_reference PRIVATE
    OAP::Manufacturing OAP::ToolpathCore)
oap_configure_fast_test(oap_test_hole_ipw_reference)

add_executable(oap_test_hole_dto_mapping
    ${OAP_TEST_HOLE_DTO_MAPPING_SOURCES})
target_link_libraries(oap_test_hole_dto_mapping PRIVATE
    OAP::ToolpathAdapter)
oap_configure_fast_test(oap_test_hole_dto_mapping)

add_executable(oap_test_tool_type_parameter_schema
    ${OAP_TEST_TOOL_TYPE_PARAMETER_SCHEMA_SOURCES})
target_link_libraries(oap_test_tool_type_parameter_schema PRIVATE
    OAP::ToolLibrary)
oap_configure_fast_test(oap_test_tool_type_parameter_schema)

if(OAP_BUILD_GUI)
    include(${CMAKE_CURRENT_LIST_DIR}/OAPAppTestTargets.cmake)
endif()

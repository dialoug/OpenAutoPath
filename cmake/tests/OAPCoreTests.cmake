# Core smoke tests: each executable and its cases are declared together.

oap_add_smoke_test(engraving_relief
    SOURCES tests/smoke/toolpath/engraving_relief_smoke.cpp
    LIBRARIES OAP::FinishingToolpath OAP::ToolpathProgram
    LABELS engraving fast high-risk toolpath
    TIMEOUT 120)

oap_add_smoke_test(surface_finishing
    SOURCES tests/smoke/toolpath/surface_finishing_smoke.cpp
    LIBRARIES OAP::FinishingToolpath OAP::ToolpathProgram
    LABELS fast finishing high-risk toolpath
    TIMEOUT 90)

oap_add_smoke_test(surface_finishing_job
    SOURCES tests/smoke/manufacturing/surface_finishing_job_smoke.cpp
    LIBRARIES OAP::Manufacturing
    LABELS fast finishing high-risk ipw manufacturing
    TIMEOUT 120)

oap_add_smoke_test(surface_finishing_ipw
    SOURCES tests/smoke/manufacturing/surface_finishing_ipw_smoke.cpp
    LIBRARIES OAP::Manufacturing
    LABELS fast finishing high-risk ipw manufacturing
    TIMEOUT 120)

oap_add_smoke_test(fixed_axis_execution
    SOURCES tests/smoke/toolpath/fixed_axis_execution_smoke.cpp
    LIBRARIES OAP::ToolpathWorkflow
    LABELS fast high-risk ipw toolpath
    TIMEOUT 30)

oap_add_smoke_test(workflow_contract
    SOURCES tests/smoke/toolpath/workflow_contract_smoke.cpp
    LIBRARIES OAP::ToolpathWorkflow
    LABELS fast high-risk lifecycle toolpath
    TIMEOUT 30)

oap_add_smoke_test_case(profile_invalid_parameters oap_test_workflow_contract
    ARGS --profile-invalid-parameters
    LABELS fast high-risk lifecycle toolpath
    TIMEOUT 30)

oap_add_smoke_test(heavy_job_scheduler
    SOURCES tests/smoke/core/heavy_job_scheduler_smoke.cpp
    LIBRARIES OAP::ToolpathCore
    LABELS fast high-risk lifecycle
    TIMEOUT 30)

oap_add_smoke_test(model_index
    SOURCES tests/smoke/analysis/model_index_smoke.cpp
    LIBRARIES OAP::Analysis
    LABELS analysis fast
    TIMEOUT 30)

oap_add_smoke_test(dexel_interval
    SOURCES tests/smoke/toolpath/dexel_interval_smoke.cpp
    LIBRARIES OAP::PocketToolpath
    LABELS fast geometry
    TIMEOUT 30)

oap_add_smoke_test(dexel_tool_profile
    SOURCES tests/smoke/toolpath/dexel_tool_profile_smoke.cpp
    LIBRARIES OAP::ToolpathCore
    LABELS fast geometry
    TIMEOUT 30)

oap_add_smoke_test(machining_job
    SOURCES tests/smoke/manufacturing/machining_job_smoke.cpp
    LIBRARIES OAP::Manufacturing
    LABELS fast ipw manufacturing
    TIMEOUT 60)

oap_add_smoke_test(machining_program_assembler
    SOURCES tests/smoke/manufacturing/machining_program_assembler_smoke.cpp
    LIBRARIES OAP::Manufacturing
    LABELS fast manufacturing program
    TIMEOUT 30)

oap_add_smoke_test(toolpath_program
    SOURCES tests/smoke/toolpath/toolpath_program_smoke.cpp
    LIBRARIES OAP::ToolpathProgram
    LABELS fast gcode toolpath
    TIMEOUT 30)

oap_add_smoke_test(occt_recognition
    SOURCES tests/smoke/recognition/occt_recognition_smoke.cpp
    LIBRARIES OAP::Recognition
    LABELS fast geometry occt recognition
    TIMEOUT 30)

oap_add_smoke_test(multi_dexel_ipw
    SOURCES tests/smoke/toolpath/multi_dexel_ipw_smoke.cpp
    LIBRARIES OAP::PocketToolpath
    LABELS e2e fast ipw multi-dexel
    TIMEOUT 30)

oap_add_smoke_test_case(multi_dexel_single_thread_budget oap_test_multi_dexel_ipw
    LABELS fast high-risk ipw multi-dexel
    TIMEOUT 60
    ENVIRONMENT "OAP_CAM_MAX_THREADS=1")

oap_add_smoke_test_case(multi_dexel_parallel_budget oap_test_multi_dexel_ipw
    LABELS fast high-risk ipw multi-dexel
    TIMEOUT 60
    ENVIRONMENT "OAP_CAM_MAX_THREADS=4")

oap_add_smoke_test(pocket_layer_linking
    SOURCES tests/smoke/toolpath/pocket_layer_linking_smoke.cpp
    LIBRARIES OAP::PocketToolpath
    LABELS fast high-risk pocket toolpath
    TIMEOUT 30)

oap_add_smoke_test_case(pocket_layer_linking_budget_1 oap_test_pocket_layer_linking
    LABELS fast high-risk pocket toolpath
    TIMEOUT 60
    ENVIRONMENT "OAP_CAM_MAX_THREADS=1")

oap_add_smoke_test_case(pocket_layer_linking_budget_2 oap_test_pocket_layer_linking
    LABELS fast high-risk pocket toolpath
    TIMEOUT 60
    ENVIRONMENT "OAP_CAM_MAX_THREADS=2")

oap_add_smoke_test(profile_contour
    SOURCES tests/smoke/toolpath/profile_contour_smoke.cpp
    LIBRARIES OAP::ProfileToolpath OAP::ToolpathWorkflow OAP::Recognition
    LABELS fast high-risk profile toolpath
    TIMEOUT 120)

oap_add_smoke_test(profile_direction_planning
    SOURCES tests/smoke/toolpath/profile_direction_planning_smoke.cpp
    LIBRARIES OAP::ProfileToolpath OAP::Recognition OAP::ToolpathAdapter
    LABELS fast high-risk profile toolpath
    TIMEOUT 90)

oap_add_smoke_test(pocket_automatic_toolpath
    SOURCES tests/smoke/tooling/pocket_automatic_toolpath_smoke.cpp
    LIBRARIES OAP::ToolpathAdapter)

oap_add_smoke_test_case(pocket_parallel_budget oap_test_pocket_automatic_toolpath
    ARGS --parallel-budget
    LABELS fast high-risk pocket toolpath
    TIMEOUT 60
    ENVIRONMENT "OAP_CAM_MAX_THREADS=4")

oap_add_smoke_test_case(pocket_single_thread_budget oap_test_pocket_automatic_toolpath
    ARGS --parallel-budget
    LABELS fast high-risk pocket toolpath
    TIMEOUT 60
    ENVIRONMENT "OAP_CAM_MAX_THREADS=1")

oap_add_smoke_test_case(pocket_hardware_budget oap_test_pocket_automatic_toolpath
    ARGS --parallel-budget
    LABELS fast high-risk pocket toolpath
    TIMEOUT 60
    ENVIRONMENT "OAP_CAM_MAX_THREADS=999")

oap_add_smoke_test(profile_automatic_toolpath
    SOURCES tests/smoke/tooling/profile_automatic_toolpath_smoke.cpp
    LIBRARIES OAP::ToolpathAdapter
    LABELS fast high-risk profile toolpath
    TIMEOUT 120)

oap_add_smoke_test(hole_batch_execution
    SOURCES tests/smoke/toolpath/hole_batch_execution_smoke.cpp
    LIBRARIES OAP::ToolpathWorkflow
    LABELS fast hole toolpath
    TIMEOUT 30)

oap_add_smoke_test(three_axis_recognition
    SOURCES tests/smoke/recognition/RecognitionSampleBenchmark.cpp
        tests/smoke/recognition/RecognitionSampleBenchmark.h
        tests/smoke/recognition/three_axis_recognition_smoke.cpp
    LIBRARIES OAP::Recognition OAP::ToolpathCore
    LABELS fast geometry high-risk occt recognition
    TIMEOUT 30)

oap_add_smoke_test_case(recognition_single_thread_budget oap_test_three_axis_recognition
    LABELS fast high-risk recognition
    TIMEOUT 60
    ENVIRONMENT "OAP_CAM_MAX_THREADS=1")

oap_add_smoke_test_case(recognition_parallel_budget oap_test_three_axis_recognition
    LABELS fast high-risk recognition
    TIMEOUT 60
    ENVIRONMENT "OAP_CAM_MAX_THREADS=4")

oap_add_smoke_test(hole_batch_identity
    SOURCES tests/smoke/manufacturing/hole_batch_identity_smoke.cpp
    LIBRARIES OAP::Manufacturing
    LABELS fast high-risk hole manufacturing toolpath
    TIMEOUT 30)

oap_add_smoke_test(hole_ipw_reference
    SOURCES tests/smoke/manufacturing/hole_ipw_reference_smoke.cpp
    LIBRARIES OAP::Manufacturing OAP::ToolpathCore
    LABELS fast geometry high-risk hole ipw occt
    TIMEOUT 60)

oap_add_smoke_test(hole_dto_mapping
    SOURCES tests/smoke/tooling/hole_dto_mapping_smoke.cpp
    LIBRARIES OAP::ToolpathAdapter
    LABELS analysis fast high-risk hole tooling toolpath
    TIMEOUT 30)

oap_add_smoke_test(tool_type_parameter_schema
    SOURCES tests/smoke/tooling/tool_type_parameter_schema_smoke.cpp
    LIBRARIES OAP::ToolLibrary
    LABELS fast tooling
    TIMEOUT 30)

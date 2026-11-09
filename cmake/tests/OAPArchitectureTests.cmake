# Static architecture and documentation checks.

add_test(NAME module_boundaries
    COMMAND "${CMAKE_COMMAND}"
        "-DOAP_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CheckModuleBoundaries.cmake")

add_test(NAME module_boundaries_forbidden_target
    COMMAND "${CMAKE_COMMAND}"
        "-DOAP_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DOAP_BOUNDARY_FIXTURE=forbidden_target"
        "-DOAP_EXPECTED_MESSAGE=forbidden target dependency oap_app_view -> oap_analysis"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/RunExpectedBoundaryFailure.cmake")

add_test(NAME module_boundaries_private_header
    COMMAND "${CMAKE_COMMAND}"
        "-DOAP_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DOAP_BOUNDARY_FIXTURE=private_header"
        "-DOAP_EXPECTED_MESSAGE=includes private header 'app/ui/shell/MainWindow.h'"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/RunExpectedBoundaryFailure.cmake")

add_test(NAME module_boundaries_transitive_target
    COMMAND "${CMAKE_COMMAND}"
        "-DOAP_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DOAP_BOUNDARY_FIXTURE=transitive_target"
        "-DOAP_EXPECTED_MESSAGE=forbidden target dependency oap_toolpath_workflow -> oap_analysis"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/RunExpectedBoundaryFailure.cmake")

add_test(NAME module_boundaries_dependency_cycle
    COMMAND "${CMAKE_COMMAND}"
        "-DOAP_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DOAP_BOUNDARY_FIXTURE=dependency_cycle"
        "-DOAP_EXPECTED_MESSAGE=First-party dependency cycle detected"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/RunExpectedBoundaryFailure.cmake")

add_test(NAME module_boundaries_workflow_reverse_target
    COMMAND "${CMAKE_COMMAND}"
        "-DOAP_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        "-DOAP_BOUNDARY_FIXTURE=workflow_reverse_target"
        "-DOAP_EXPECTED_MESSAGE=forbidden target dependency oap_app_workflow_cam -> oap_app_workflow_simulation"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/RunExpectedBoundaryFailure.cmake")

add_test(NAME workflow_internal_boundaries
    COMMAND "${CMAKE_COMMAND}"
        "-DOAP_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CheckWorkflowBoundaries.cmake")

add_test(NAME documentation_links
    COMMAND "${CMAKE_COMMAND}"
        "-DOAP_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CheckDocumentationLinks.cmake")

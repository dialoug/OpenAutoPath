cmake_minimum_required(VERSION 3.21)

if(NOT DEFINED OAP_SOURCE_DIR OR
   NOT DEFINED OAP_BOUNDARY_FIXTURE OR
   NOT DEFINED OAP_EXPECTED_MESSAGE)
    message(FATAL_ERROR
        "OAP_SOURCE_DIR, OAP_BOUNDARY_FIXTURE and OAP_EXPECTED_MESSAGE are required")
endif()

execute_process(
    COMMAND "${CMAKE_COMMAND}"
        "-DOAP_SOURCE_DIR=${OAP_SOURCE_DIR}"
        "-DOAP_BOUNDARY_FIXTURE=${OAP_BOUNDARY_FIXTURE}"
        -P "${OAP_SOURCE_DIR}/cmake/CheckModuleBoundaries.cmake"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error)

if(result EQUAL 0)
    message(FATAL_ERROR
        "Boundary fixture '${OAP_BOUNDARY_FIXTURE}' unexpectedly passed")
endif()

set(combined "${output}\n${error}")
if(NOT combined MATCHES "${OAP_EXPECTED_MESSAGE}")
    message(FATAL_ERROR
        "Boundary fixture '${OAP_BOUNDARY_FIXTURE}' failed for the wrong reason:\n${combined}")
endif()

message(STATUS
    "Boundary fixture '${OAP_BOUNDARY_FIXTURE}' rejected as expected")

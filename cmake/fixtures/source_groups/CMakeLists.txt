cmake_minimum_required(VERSION 3.22)
project(OAPSourceGroupCheck LANGUAGES NONE)

include("${OAP_SOURCE_DIR}/cmake/OAPTargetHelpers.cmake")
# Mimic FetchContent or an SDK located outside the project's source directory.
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/external.cpp" "// generated fixture\n")
add_custom_target(source_group_fixture SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt"
    "${CMAKE_CURRENT_BINARY_DIR}/external.cpp")
oap_group_sources(source_group_fixture)

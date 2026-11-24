# CAM 核心库与 Web 服务的原生 worker。
cmake_minimum_required(VERSION 3.22)
include(cmake/platforms/OAPDefaults.cmake)
project(OpenAutoPath LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(cmake/platforms/OAPPlatform.cmake)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(OAP_ENABLE_STRICT_WARNINGS
       "Enable strict compiler warnings for first-party targets" ON)

include(cmake/OAPDependencies.cmake)
include(cmake/OAPSources.cmake)
include(cmake/OAPModuleDependencies.cmake)
include(cmake/OAPTargetHelpers.cmake)

set(OAP_OUTPUT_ROOT "${CMAKE_BINARY_DIR}/$<CONFIG>")
set(OAP_RUNTIME_OUTPUT_DIR "${OAP_OUTPUT_ROOT}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OAP_RUNTIME_OUTPUT_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OAP_RUNTIME_OUTPUT_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OAP_OUTPUT_ROOT}/lib")
set(CMAKE_PDB_OUTPUT_DIRECTORY "${OAP_OUTPUT_ROOT}/symbols")
# Keep compiler PDBs in CMake's per-target intermediate directories. Unlike
# linker PDBs, they can share the default name (vc143.pdb) across targets.

# ---------------------------------------------------------------------------
# 可复用的核心模块
# ---------------------------------------------------------------------------
# 将与 UI 无关的代码放入库中，供应用与回归测试复用。所有第一方直接依赖
# 由 OAPModuleDependencies.cmake 统一提供给构建和架构门禁。

add_library(oap_support STATIC ${OAP_SUPPORT_SOURCES})
oap_enable_utf8(oap_support)
oap_enable_core_guideline_warnings(oap_support)
target_include_directories(oap_support SYSTEM INTERFACE
    "$<BUILD_INTERFACE:${OAP_JSON_INCLUDE_DIR}>")

add_library(oap_analysis STATIC ${OAP_ANALYSIS_SOURCES})
add_library(OAP::Analysis ALIAS oap_analysis)
oap_configure_occt_core_target(oap_analysis PUBLIC)
target_link_libraries(oap_analysis PRIVATE oap_support)
oap_enable_utf8(oap_analysis)
oap_enable_core_guideline_warnings(oap_analysis)
oap_group_sources(oap_analysis)

add_library(oap_recognition STATIC ${OAP_RECOGNITION_SOURCES})
add_library(OAP::Recognition ALIAS oap_recognition)
target_link_libraries(oap_recognition
    PUBLIC ${OAP_INTERNAL_DEPS_oap_recognition} Threads::Threads)
target_compile_definitions(oap_recognition PRIVATE _USE_MATH_DEFINES)
oap_enable_utf8(oap_recognition)
oap_enable_core_guideline_warnings(oap_recognition)
oap_group_sources(oap_recognition)

add_library(oap_toolpath_core STATIC ${OAP_TOOLPATH_CORE_SOURCES})
add_library(OAP::ToolpathCore ALIAS oap_toolpath_core)
target_include_directories(oap_toolpath_core
    SYSTEM PRIVATE "${OAP_CLIPPER2_INCLUDE_DIR}")
target_link_libraries(oap_toolpath_core
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_core} Threads::Threads)

add_library(oap_toolpath_program STATIC ${OAP_TOOLPATH_PROGRAM_SOURCES})
add_library(OAP::ToolpathProgram ALIAS oap_toolpath_program)
target_link_libraries(oap_toolpath_program
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_program})

add_library(oap_toolpath_hole STATIC ${OAP_TOOLPATH_HOLE_SOURCES})
add_library(OAP::HoleToolpath ALIAS oap_toolpath_hole)
target_link_libraries(oap_toolpath_hole
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_hole})

add_library(oap_toolpath_pocket STATIC ${OAP_TOOLPATH_POCKET_SOURCES})
add_library(OAP::PocketToolpath ALIAS oap_toolpath_pocket)
target_include_directories(oap_toolpath_pocket
    SYSTEM PUBLIC
        "$<BUILD_INTERFACE:${OAP_CLIPPER2_INCLUDE_DIR}>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/OpenAutoPath/thirdparty>")
target_link_libraries(oap_toolpath_pocket
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_pocket})

add_library(oap_toolpath_profile STATIC ${OAP_TOOLPATH_PROFILE_SOURCES})
add_library(OAP::ProfileToolpath ALIAS oap_toolpath_profile)
target_include_directories(oap_toolpath_profile
    SYSTEM PRIVATE "${OAP_CLIPPER2_INCLUDE_DIR}")
target_link_libraries(oap_toolpath_profile
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_profile})

add_library(oap_toolpath_finishing STATIC ${OAP_TOOLPATH_FINISHING_SOURCES})
add_library(OAP::FinishingToolpath ALIAS oap_toolpath_finishing)
target_link_libraries(oap_toolpath_finishing
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_finishing}
    PRIVATE OAP::OpenCAMLib)
# Use the same TBB runtime as the Windows OCCT SDK to bound its mesher.
set(OAP_FINISHING_TBB FALSE)
if(OAP_OCCT_PROVIDER STREQUAL "WINDOWS_SDK")
    find_package(TBB QUIET CONFIG PATHS "${OAP_THIRDPARTY_ROOT}/tbb-2021.13.0-x64" NO_DEFAULT_PATH)
    if(TARGET TBB::tbb)
        set(OAP_FINISHING_TBB TRUE)
        target_compile_definitions(oap_toolpath_finishing PRIVATE OAP_FINISHING_TBB __TBB_NO_IMPLICIT_LINKAGE)
        target_link_libraries(oap_toolpath_finishing PRIVATE TBB::tbb)
    endif()
endif()

add_library(oap_toolpath_algorithms STATIC ${OAP_TOOLPATH_ALGORITHM_SOURCES})
add_library(OAP::ToolpathAlgorithms ALIAS oap_toolpath_algorithms)
target_link_libraries(oap_toolpath_algorithms
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_algorithms})

add_library(oap_toolpath_workflow STATIC ${OAP_TOOLPATH_WORKFLOW_SOURCES})
add_library(OAP::ToolpathWorkflow ALIAS oap_toolpath_workflow)
target_link_libraries(oap_toolpath_workflow
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_workflow})

foreach(toolpath_target
        oap_toolpath_core
        oap_toolpath_hole
        oap_toolpath_pocket
        oap_toolpath_profile
        oap_toolpath_finishing
        oap_toolpath_algorithms
        oap_toolpath_program
        oap_toolpath_workflow)
    oap_enable_utf8(${toolpath_target})
    oap_enable_core_guideline_warnings(${toolpath_target})
    oap_group_sources(${toolpath_target})
endforeach()

add_library(oap_toollibrary STATIC ${OAP_TOOLING_CORE_SOURCES})
add_library(OAP::ToolLibrary ALIAS oap_toollibrary)
target_link_libraries(oap_toollibrary PRIVATE oap_support)
oap_enable_utf8(oap_toollibrary)
oap_enable_core_guideline_warnings(oap_toollibrary)
oap_group_sources(oap_toollibrary)

# 适配器是 tooling 与 toolpath 的唯一依赖交汇点。核心刀库可独立用于
# 导入、编辑和持久化；仅需要生成 CAM 参数的调用方才链接这个目标。
add_library(oap_toolpath_adapter STATIC ${OAP_TOOLPATH_ADAPTER_SOURCES})
add_library(OAP::ToolpathAdapter ALIAS oap_toolpath_adapter)
target_link_libraries(oap_toolpath_adapter
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_adapter})
oap_enable_utf8(oap_toolpath_adapter)
oap_enable_core_guideline_warnings(oap_toolpath_adapter)
oap_group_sources(oap_toolpath_adapter)

add_library(oap_manufacturing STATIC ${OAP_MANUFACTURING_SOURCES})
add_library(OAP::Manufacturing ALIAS oap_manufacturing)
target_link_libraries(oap_manufacturing
    PUBLIC ${OAP_INTERNAL_DEPS_oap_manufacturing})
oap_enable_utf8(oap_manufacturing)
oap_enable_core_guideline_warnings(oap_manufacturing)
oap_group_sources(oap_manufacturing)

set(worker_library_file "${PROJECT_SOURCE_DIR}/resources/tool-libraries/general-simulation.oaptl")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${worker_library_file}")
file(READ "${worker_library_file}" OAP_DEFAULT_TOOL_LIBRARY)
configure_file(src/worker/DefaultToolLibrary.h.in generated/DefaultToolLibrary.h @ONLY)
add_executable(oap-worker ${OAP_WORKER_SOURCES})
target_include_directories(oap-worker PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(oap-worker PRIVATE ${OAP_INTERNAL_DEPS_oap-worker})
oap_enable_utf8(oap-worker)
oap_enable_core_guideline_warnings(oap-worker)
if(APPLE)
    set_target_properties(oap-worker PROPERTIES INSTALL_RPATH "@loader_path/../${CMAKE_INSTALL_LIBDIR}")
elseif(UNIX)
    set_target_properties(oap-worker PROPERTIES INSTALL_RPATH "\$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
endif()

foreach(first_party_target IN LISTS OAP_FIRST_PARTY_TARGETS)
    if(TARGET ${first_party_target})
        oap_configure_first_party_includes(${first_party_target})
        if(first_party_target IN_LIST OAP_FIRST_PARTY_LIBRARY_TARGETS)
            oap_require_cxx20(${first_party_target} PUBLIC)
        else()
            oap_require_cxx20(${first_party_target} PRIVATE)
        endif()
    endif()
endforeach()

include(cmake/OAPRuntimePaths.cmake)

if(BUILD_TESTING)
    include(cmake/OAPTests.cmake)
endif()

include(cmake/OAPInstall.cmake)
if(BUILD_TESTING AND OAP_INSTALL_DEVELOPMENT)
    include(cmake/tests/OAPInstallTests.cmake)
endif()

# Audited OpenCAMLib C++ integration. Do not execute upstream's top-level CMake:
# it rewrites global source/output settings and builds duplicate object libraries.
# We compile the upstream C++ source list unchanged in an isolated shared target.
include_guard(GLOBAL)
include(FetchContent)
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

set(OAP_OPENCAMLIB_REVISION "1a450b7c3d3b669224a2a5d09aab7f319794574c")
set(oap_ocl_root_default "$ENV{OAP_OPENCAMLIB_ROOT}")
if(NOT oap_ocl_root_default AND
   EXISTS "${PROJECT_SOURCE_DIR}/external/opencamlib/src/ocl.hpp")
    set(oap_ocl_root_default "${PROJECT_SOURCE_DIR}/external/opencamlib")
endif()
set(OAP_OPENCAMLIB_ROOT "${oap_ocl_root_default}" CACHE PATH
    "Optional OpenCAMLib source checkout at the audited revision")
if(OAP_OPENCAMLIB_ROOT)
    set(oap_ocl_source_root "${OAP_OPENCAMLIB_ROOT}")
    if(EXISTS "${oap_ocl_source_root}/.git")
        find_package(Git REQUIRED)
        execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${oap_ocl_source_root}" rev-parse HEAD
            OUTPUT_VARIABLE oap_ocl_checkout_revision OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE oap_ocl_revision_result)
        if(NOT oap_ocl_revision_result EQUAL 0 OR
           NOT oap_ocl_checkout_revision STREQUAL OAP_OPENCAMLIB_REVISION)
            message(FATAL_ERROR
                "OAP_OPENCAMLIB_ROOT must use audited revision ${OAP_OPENCAMLIB_REVISION}")
        endif()
    endif()
else()
    FetchContent_Declare(OAPOpenCamLibSource
        GIT_REPOSITORY https://github.com/aewallin/opencamlib.git
        GIT_TAG "${OAP_OPENCAMLIB_REVISION}"
        GIT_SUBMODULES ""
        SOURCE_SUBDIR _oap_source_only)
    FetchContent_MakeAvailable(OAPOpenCamLibSource)
    set(oap_ocl_source_root "${oapopencamlibsource_SOURCE_DIR}")
endif()

set(oap_ocl_boost_default "$ENV{OAP_OPENCAMLIB_BOOST_ROOT}")
if(NOT oap_ocl_boost_default AND
   EXISTS "${PROJECT_SOURCE_DIR}/external/boost_1_87_0/boost/version.hpp")
    set(oap_ocl_boost_default "${PROJECT_SOURCE_DIR}/external/boost_1_87_0")
endif()
set(OAP_OPENCAMLIB_BOOST_ROOT "${oap_ocl_boost_default}" CACHE PATH
    "Optional Boost source root containing boost/version.hpp and LICENSE_1_0.txt")
if(OAP_OPENCAMLIB_BOOST_ROOT)
    set(oap_ocl_boost_include "${OAP_OPENCAMLIB_BOOST_ROOT}")
else()
    # Only headers are consumed; no Boost libraries, Python or Node bindings.
    FetchContent_Declare(OAPOpenCamLibBoost
        URL https://archives.boost.io/release/1.87.0/source/boost_1_87_0.tar.gz
        URL_HASH SHA256=f55c340aa49763b1925ccf02b2e83f35fdcf634c9d5164a2acb87540173c741d
        SOURCE_SUBDIR _oap_headers_only)
    FetchContent_MakeAvailable(OAPOpenCamLibBoost)
    set(oap_ocl_boost_include "${oapopencamlibboost_SOURCE_DIR}")
endif()
foreach(oap_ocl_required
        "${oap_ocl_source_root}/src/ocl.hpp"
        "${oap_ocl_source_root}/src/dropcutter/pointdropcutter.hpp"
        "${oap_ocl_source_root}/COPYING"
        "${oap_ocl_boost_include}/boost/version.hpp"
        "${oap_ocl_boost_include}/LICENSE_1_0.txt")
    if(NOT EXISTS "${oap_ocl_required}")
        message(FATAL_ERROR "Required OpenCAMLib dependency file is missing: ${oap_ocl_required}")
    endif()
endforeach()

set(oap_ocl_relative_sources
    ocl.cpp
    geo/arc.cpp geo/bbox.cpp geo/ccpoint.cpp geo/clpoint.cpp geo/line.cpp
    geo/path.cpp geo/point.cpp geo/stlreader.cpp geo/stlsurf.cpp geo/triangle.cpp
    cutters/ballcutter.cpp cutters/bullcutter.cpp cutters/compositecutter.cpp
    cutters/conecutter.cpp cutters/millingcutter.cpp cutters/cylcutter.cpp
    cutters/ellipse.cpp cutters/ellipseposition.cpp
    dropcutter/batchdropcutter.cpp dropcutter/pointdropcutter.cpp
    dropcutter/pathdropcutter.cpp dropcutter/adaptivepathdropcutter.cpp
    algo/batchpushcutter.cpp algo/fiberpushcutter.cpp algo/interval.cpp
    algo/fiber.cpp algo/waterline.cpp algo/adaptivewaterline.cpp
    algo/weave.cpp algo/simple_weave.cpp algo/smart_weave.cpp
    common/numeric.cpp common/lineclfilter.cpp)
list(TRANSFORM oap_ocl_relative_sources PREPEND "${oap_ocl_source_root}/src/")
add_library(oap_opencamlib SHARED ${oap_ocl_relative_sources})
add_library(OAP::OpenCAMLib ALIAS oap_opencamlib)
set(oap_ocl_generated "${CMAKE_BINARY_DIR}/generated/opencamlib")
file(MAKE_DIRECTORY "${oap_ocl_generated}")
file(GENERATE OUTPUT "${oap_ocl_generated}/version_string.hpp" CONTENT
    "#pragma once\n#define VERSION_STRING \"${OAP_OPENCAMLIB_REVISION}\"\n")
target_include_directories(oap_opencamlib PRIVATE "${oap_ocl_generated}")
target_include_directories(oap_opencamlib SYSTEM PUBLIC
    "$<BUILD_INTERFACE:${oap_ocl_source_root}/src>"
    "$<BUILD_INTERFACE:${oap_ocl_source_root}/src/geo>"
    "$<BUILD_INTERFACE:${oap_ocl_source_root}/src/cutters>"
    "$<BUILD_INTERFACE:${oap_ocl_source_root}/src/dropcutter>"
    "$<BUILD_INTERFACE:${oap_ocl_source_root}/src/algo>"
    "$<BUILD_INTERFACE:${oap_ocl_source_root}/src/common>"
    "$<BUILD_INTERFACE:${oap_ocl_boost_include}>")
target_compile_definitions(oap_opencamlib PRIVATE BOOST_ALL_NO_LIB _USE_MATH_DEFINES)
if(MSVC)
    target_compile_options(oap_opencamlib PRIVATE /utf-8 /openmp-)
else()
    target_compile_options(oap_opencamlib PRIVATE -fno-openmp)
endif()
# OpenCAMLib's internal parallel loops are intentionally disabled. Callers own
# task admission and cancellation through the application's shared budget.
set_target_properties(oap_opencamlib PROPERTIES
    CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF
    WINDOWS_EXPORT_ALL_SYMBOLS ON INTERPROCEDURAL_OPTIMIZATION FALSE
    POSITION_INDEPENDENT_CODE ON FOLDER "external"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>/lib"
    PDB_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>/symbols")

# Preserve the upstream LGPL text alongside the replaceable shared library.
set(OAP_OPENCAMLIB_LICENSE_FILE "${oap_ocl_source_root}/COPYING")
set(OAP_OPENCAMLIB_NOTICE_FILE "${oap_ocl_generated}/OpenCAMLib-NOTICE.txt")
file(WRITE "${OAP_OPENCAMLIB_NOTICE_FILE}"
    "OpenCAMLib: https://github.com/aewallin/opencamlib\n"
    "Source revision: ${OAP_OPENCAMLIB_REVISION}\n"
    "Copyright: Anders Wallin and OpenCAMLib contributors.\n"
    "License: LGPL-2.1-or-later; see OpenCAMLib-COPYING.txt.\n"
    "Unmodified upstream C++ sources; custom CMake wrapper, OpenMP disabled.\n"
    "This replaceable shared library uses Boost headers under Boost Software License 1.0.\n"
    "See Boost-LICENSE_1_0.txt.\n")
add_custom_command(TARGET oap_opencamlib POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E make_directory "$<TARGET_FILE_DIR:oap_opencamlib>/licenses"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "${OAP_OPENCAMLIB_LICENSE_FILE}"
        "$<TARGET_FILE_DIR:oap_opencamlib>/licenses/OpenCAMLib-COPYING.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "${oap_ocl_boost_include}/LICENSE_1_0.txt"
        "$<TARGET_FILE_DIR:oap_opencamlib>/licenses/Boost-LICENSE_1_0.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "${OAP_OPENCAMLIB_NOTICE_FILE}"
        "$<TARGET_FILE_DIR:oap_opencamlib>/licenses/OpenCAMLib-NOTICE.txt")
message(STATUS "OpenCAMLib ${OAP_OPENCAMLIB_REVISION}: shared C++ library, OpenMP disabled")

unset(oap_ocl_root_default)
unset(oap_ocl_boost_default)

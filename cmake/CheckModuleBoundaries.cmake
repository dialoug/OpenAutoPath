# Static source/target boundary check for first-party includes.

cmake_minimum_required(VERSION 3.21)

if(NOT DEFINED OAP_SOURCE_DIR OR NOT IS_DIRECTORY "${OAP_SOURCE_DIR}/src")
    message(FATAL_ERROR
        "OAP_SOURCE_DIR must point to the OpenAutoPath source root")
endif()

include("${OAP_SOURCE_DIR}/cmake/OAPSources.cmake")
include("${OAP_SOURCE_DIR}/cmake/OAPModuleDependencies.cmake")
include("${OAP_SOURCE_DIR}/cmake/OAPPublicHeaders.cmake")
include("${OAP_SOURCE_DIR}/cmake/tests/OAPTestPrivateHeaders.cmake")

if(OAP_BOUNDARY_FIXTURE STREQUAL "dependency_cycle")
    list(APPEND OAP_INTERNAL_DEPS_oap_analysis oap_recognition)
endif()

# Reject cycles and unknown first-party targets before inspecting source files.
# Repeatedly removing dependency-free nodes provides a topological sort that is
# simple enough to run in CMake script mode.
set(OAP_TOPOLOGY_REMAINING ${OAP_FIRST_PARTY_TARGETS})
while(OAP_TOPOLOGY_REMAINING)
    set(topology_progress FALSE)
    foreach(candidate IN LISTS OAP_TOPOLOGY_REMAINING)
        set(candidate_has_remaining_dependency FALSE)
        foreach(dependency IN LISTS OAP_INTERNAL_DEPS_${candidate})
            list(FIND OAP_FIRST_PARTY_TARGETS "${dependency}" known_index)
            if(known_index EQUAL -1)
                message(FATAL_ERROR
                    "Unknown first-party dependency ${candidate} -> ${dependency}")
            endif()
            list(FIND OAP_TOPOLOGY_REMAINING
                "${dependency}" remaining_index)
            if(NOT remaining_index EQUAL -1)
                set(candidate_has_remaining_dependency TRUE)
            endif()
        endforeach()
        if(NOT candidate_has_remaining_dependency)
            list(REMOVE_ITEM OAP_TOPOLOGY_REMAINING "${candidate}")
            set(topology_progress TRUE)
            break()
        endif()
    endforeach()
    if(NOT topology_progress)
        list(JOIN OAP_TOPOLOGY_REMAINING ", " cycle_targets)
        message(FATAL_ERROR
            "First-party dependency cycle detected among: ${cycle_targets}")
    endif()
endwhile()

file(GLOB_RECURSE OAP_BOUNDARY_SOURCE_FILES
    LIST_DIRECTORIES FALSE
    "${OAP_SOURCE_DIR}/src/*.cpp"
    "${OAP_SOURCE_DIR}/src/*.h"
    "${OAP_SOURCE_DIR}/src/*.inc"
    "${OAP_SOURCE_DIR}/tests/smoke/*.cpp"
    "${OAP_SOURCE_DIR}/tests/smoke/*.h")
if(OAP_BOUNDARY_FIXTURE STREQUAL "forbidden_target")
    list(APPEND OAP_BOUNDARY_SOURCE_FILES
        "${OAP_SOURCE_DIR}/cmake/fixtures/module_boundaries/forbidden_target.cpp")
elseif(OAP_BOUNDARY_FIXTURE STREQUAL "private_header")
    list(APPEND OAP_BOUNDARY_SOURCE_FILES
        "${OAP_SOURCE_DIR}/cmake/fixtures/module_boundaries/private_header.cpp")
elseif(OAP_BOUNDARY_FIXTURE STREQUAL "transitive_target")
    list(APPEND OAP_BOUNDARY_SOURCE_FILES
        "${OAP_SOURCE_DIR}/cmake/fixtures/module_boundaries/transitive_target.cpp")
elseif(OAP_BOUNDARY_FIXTURE STREQUAL "workflow_reverse_target")
    list(APPEND OAP_BOUNDARY_SOURCE_FILES
        "${OAP_SOURCE_DIR}/cmake/fixtures/module_boundaries/workflow_reverse_target.cpp")
endif()

set(OAP_PRODUCTION_TARGETS)
function(oap_register_boundary_target target source_variable)
    list(APPEND OAP_PRODUCTION_TARGETS "${target}")
    set(OAP_PRODUCTION_TARGETS "${OAP_PRODUCTION_TARGETS}" PARENT_SCOPE)
    set("OAP_ALLOWED_DEPS_${target}" "${ARGN}" PARENT_SCOPE)
    foreach(source IN LISTS ${source_variable})
        if(IS_ABSOLUTE "${source}")
            file(RELATIVE_PATH relative "${OAP_SOURCE_DIR}" "${source}")
        else()
            set(relative "${source}")
        endif()
        string(REPLACE "\\" "/" relative "${relative}")
        if(NOT relative MATCHES "^src/")
            continue()
        endif()
        string(MD5 source_key "${relative}")
        if(DEFINED "OAP_SOURCE_OWNER_${source_key}" AND
           NOT "${OAP_SOURCE_OWNER_${source_key}}" STREQUAL "${target}")
            message(FATAL_ERROR
                "Source '${relative}' belongs to both "
                "${OAP_SOURCE_OWNER_${source_key}} and ${target}")
        endif()
        set("OAP_SOURCE_OWNER_${source_key}" "${target}" PARENT_SCOPE)
        # Cross-target callers may include only headers in the explicit API
        # manifest. All other headers are private regardless of their path.
        list(FIND OAP_PUBLIC_TARGET_HEADERS "${relative}" public_index)
        if(public_index GREATER_EQUAL 0)
            set("OAP_PUBLIC_HEADER_${source_key}" TRUE PARENT_SCOPE)
        endif()
    endforeach()
endfunction()

foreach(target IN LISTS OAP_FIRST_PARTY_TARGETS)
    oap_register_boundary_target(${target} ${OAP_SOURCE_VARIABLE_${target}}
        ${OAP_INTERNAL_DEPS_${target}})
endforeach()

foreach(public_header IN LISTS OAP_PUBLIC_TARGET_HEADERS)
    if(public_header MATCHES "(^|/)detail/")
        message(FATAL_ERROR
            "Detail header '${public_header}' cannot be part of the public API")
    endif()
    string(MD5 public_header_key "${public_header}")
    if(NOT DEFINED "OAP_SOURCE_OWNER_${public_header_key}")
        message(FATAL_ERROR
            "Public header '${public_header}' is not assigned to a production target")
    endif()
endforeach()

set(OAP_ANY_INCLUDE_REGEX
    [=[^[ 	]*#[ 	]*include[ 	]*[<"]([^>"]+)[>"]]=])

set(OAP_BOUNDARY_VIOLATIONS)
set(OAP_USED_PUBLIC_HEADERS)
set(OAP_SCANNED_INCLUDE_COUNT 0)
set(OAP_CROSS_TARGET_INCLUDE_COUNT 0)

foreach(source_file IN LISTS OAP_BOUNDARY_SOURCE_FILES)
    file(RELATIVE_PATH relative_file "${OAP_SOURCE_DIR}" "${source_file}")
    string(REPLACE "\\" "/" relative_file "${relative_file}")
    if(relative_file MATCHES "^src/.*\\.inc$")
        list(APPEND OAP_BOUNDARY_VIOLATIONS
            "${relative_file}: implementation fragments must become independently compiled sources")
    endif()
    string(MD5 source_key "${relative_file}")
    if(DEFINED "OAP_SOURCE_OWNER_${source_key}")
        set(source_target "${OAP_SOURCE_OWNER_${source_key}}")
    elseif(relative_file MATCHES "^tests/smoke/")
        set(source_target oap_tests)
    elseif(relative_file MATCHES "^src/launcher/")
        set(source_target oap_launcher)
    elseif(relative_file MATCHES
           "^cmake/fixtures/module_boundaries/forbidden_target\\.cpp$")
        set(source_target oap_app_view)
    elseif(relative_file MATCHES
           "^cmake/fixtures/module_boundaries/private_header\\.cpp$")
        set(source_target oap_tests)
    elseif(relative_file MATCHES
           "^cmake/fixtures/module_boundaries/transitive_target\\.cpp$")
        set(source_target oap_toolpath_workflow)
    elseif(relative_file MATCHES
           "^cmake/fixtures/module_boundaries/workflow_reverse_target\\.cpp$")
        set(source_target oap_app_workflow_cam)
    elseif(relative_file MATCHES
           "^src/(analysis|recognition|toolpath|tooling|manufacturing|app)/")
        list(APPEND OAP_BOUNDARY_VIOLATIONS
            "${relative_file}: source is not assigned by OAPSources.cmake")
        continue()
    else()
        continue()
    endif()

    if(source_target STREQUAL OpenAutoPathApp AND
       relative_file MATCHES "^src/app/ui/shell/MainWindow.*\\.(cpp|h)$")
        file(READ "${source_file}" main_window_source)
        set(main_window_workspace_escape_tokens
            "m_machiningWorkspace->session("
            "m_machiningWorkspace->jobForWorkflow("
            "m_machiningWorkspace->resetForModel("
            "m_machiningWorkspace->installProject("
            "m_machiningWorkspace->holeManualWorkflowForWorkflow("
            "m_stockSimulationWorkspace->session("
            "m_stockSimulationWorkspace->taskCoordinator("
            "m_stockSimulationWorkspace->resetForModel("
            "m_playbackWorkspace->session("
            "m_playbackWorkspace->cancelAndWait("
            "m_recognitionWorkspace->session("
            "m_recognitionWorkspace->resetForModel("
            "recognitionSession("
            "m_documentWorkspace->session("
            "m_documentWorkspace->cancelAndWait("
            "documentSession(")
        foreach(escape_token IN LISTS main_window_workspace_escape_tokens)
            string(FIND "${main_window_source}" "${escape_token}"
                escape_index)
            if(NOT escape_index EQUAL -1)
                list(APPEND OAP_BOUNDARY_VIOLATIONS
                    "${relative_file}: MainWindow bypasses a Workspace facade via '${escape_token}'")
            endif()
        endforeach()
    endif()

    if(relative_file MATCHES
       "^src/app/.+/[A-Za-z0-9_]*WorkspaceCoordinator\\.h$")
        file(READ "${source_file}" workspace_header_source)
        set(workspace_session_escape_tokens
            " session("
            " sessionFor")
        foreach(escape_token IN LISTS workspace_session_escape_tokens)
            string(FIND "${workspace_header_source}" "${escape_token}"
                escape_index)
            if(NOT escape_index EQUAL -1)
                list(APPEND OAP_BOUNDARY_VIOLATIONS
                    "${relative_file}: Workspace facade exposes Session access via '${escape_token}'")
            endif()
        endforeach()
    endif()

    if(relative_file STREQUAL "src/app/ui/shell/MainWindow.cpp")
        file(READ "${source_file}" main_window_root_source)
        set(main_window_moved_definition_tokens
            "MainWindow::setupMenus("
            "MainWindow::setupToolbars("
            "MainWindow::setupDocks("
            "MainWindow::loadFile("
            "MainWindow::onEntityPicked("
            "MainWindow::recognizeInnerOuterStructures(")
        foreach(moved_token IN LISTS main_window_moved_definition_tokens)
            string(FIND "${main_window_root_source}" "${moved_token}"
                moved_index)
            if(NOT moved_index EQUAL -1)
                list(APPEND OAP_BOUNDARY_VIOLATIONS
                    "${relative_file}: composition root regained moved adapter '${moved_token}'")
            endif()
        endforeach()
    endif()

    file(STRINGS "${source_file}" internal_includes
        REGEX "${OAP_ANY_INCLUDE_REGEX}")
    foreach(include_line IN LISTS internal_includes)
        if(NOT include_line MATCHES "${OAP_ANY_INCLUDE_REGEX}")
            continue()
        endif()
        set(raw_include_path "${CMAKE_MATCH_1}")
        if(source_target STREQUAL oap_toolpath_profile AND raw_include_path STREQUAL
           "toolpath/pocket/program/PocketToolpathOperation.h")
            list(APPEND OAP_BOUNDARY_VIOLATIONS
                "${relative_file}: Profile material execution must use a neutral view; legacy bridges belong in workflow")
        endif()
        if(raw_include_path MATCHES "\\.(cpp|inc)$")
            list(APPEND OAP_BOUNDARY_VIOLATIONS
                "${relative_file}: textual implementation include '${raw_include_path}' is forbidden")
        endif()
        if(raw_include_path MATCHES
           "^(analysis|recognition|toolpath|tooling|manufacturing|app)/")
            set(include_path "${raw_include_path}")
            set(include_file "src/${include_path}")
        elseif(include_line MATCHES
               [=[^[ 	]*#[ 	]*include[ 	]*"([^"]+)"]=])
            get_filename_component(source_directory "${source_file}" DIRECTORY)
            get_filename_component(absolute_include "${raw_include_path}"
                ABSOLUTE BASE_DIR "${source_directory}")
            if(NOT EXISTS "${absolute_include}")
                continue()
            endif()
            file(RELATIVE_PATH include_file
                "${OAP_SOURCE_DIR}" "${absolute_include}")
            string(REPLACE "\\" "/" include_file "${include_file}")
            if(NOT include_file MATCHES "^src/")
                continue()
            endif()
            string(REGEX REPLACE "^src/" "" include_path "${include_file}")
        else()
            continue()
        endif()
        string(MD5 include_key "${include_file}")
        math(EXPR OAP_SCANNED_INCLUDE_COUNT
            "${OAP_SCANNED_INCLUDE_COUNT} + 1")

        if(NOT DEFINED "OAP_SOURCE_OWNER_${include_key}")
            list(APPEND OAP_BOUNDARY_VIOLATIONS
                "${relative_file}: ${source_target} includes unregistered header '${include_path}'")
            continue()
        endif()
        set(target_target "${OAP_SOURCE_OWNER_${include_key}}")
        if(source_target STREQUAL target_target)
            continue()
        endif()
        math(EXPR OAP_CROSS_TARGET_INCLUDE_COUNT
            "${OAP_CROSS_TARGET_INCLUDE_COUNT} + 1")

        if(source_target STREQUAL oap_tests OR
           source_target STREQUAL oap_launcher)
            set(dependency_allowed TRUE)
        else()
            list(FIND "OAP_ALLOWED_DEPS_${source_target}"
                "${target_target}" dependency_index)
            if(dependency_index EQUAL -1)
                set(dependency_allowed FALSE)
            else()
                set(dependency_allowed TRUE)
            endif()
        endif()
        if(NOT dependency_allowed)
            list(APPEND OAP_BOUNDARY_VIOLATIONS
                "${relative_file}: forbidden target dependency ${source_target} -> ${target_target} via '${include_path}'")
        endif()
        if(NOT DEFINED "OAP_PUBLIC_HEADER_${include_key}")
            list(FIND OAP_TEST_PRIVATE_HEADER_ACCESS
                "${relative_file}|${include_path}" private_test_index)
            if(NOT source_target STREQUAL oap_tests OR private_test_index LESS 0)
                list(APPEND OAP_BOUNDARY_VIOLATIONS
                    "${relative_file}: ${source_target} -> ${target_target} includes private header '${include_path}'")
            endif()
        else()
            list(APPEND OAP_USED_PUBLIC_HEADERS "${include_file}")
        endif()
    endforeach()
endforeach()

list(REMOVE_DUPLICATES OAP_USED_PUBLIC_HEADERS)
set(OAP_UNUSED_PUBLIC_HEADERS)
foreach(public_header IN LISTS OAP_PUBLIC_TARGET_HEADERS)
    list(FIND OAP_USED_PUBLIC_HEADERS "${public_header}" used_index)
    if(used_index LESS 0)
        list(APPEND OAP_UNUSED_PUBLIC_HEADERS "${public_header}")
    endif()
endforeach()

if(OAP_UNUSED_PUBLIC_HEADERS)
    foreach(unused_public_header IN LISTS OAP_UNUSED_PUBLIC_HEADERS)
        list(APPEND OAP_BOUNDARY_VIOLATIONS
            "public API manifest exposes unused header '${unused_public_header}'")
    endforeach()
endif()

if(OAP_BOUNDARY_VIOLATIONS)
    list(REMOVE_DUPLICATES OAP_BOUNDARY_VIOLATIONS)
    list(JOIN OAP_BOUNDARY_VIOLATIONS "\n  - " violation_text)
    message(FATAL_ERROR
        "OAP target boundary check failed:\n  - ${violation_text}")
endif()

list(LENGTH OAP_BOUNDARY_SOURCE_FILES OAP_SCANNED_SOURCE_COUNT)
list(LENGTH OAP_PRODUCTION_TARGETS OAP_SCANNED_TARGET_COUNT)
list(LENGTH OAP_UNUSED_PUBLIC_HEADERS OAP_UNUSED_PUBLIC_HEADER_COUNT)
if(OAP_UNUSED_PUBLIC_HEADERS)
    list(JOIN OAP_UNUSED_PUBLIC_HEADERS ";" unused_public_text)
    message(STATUS "Unused public headers: ${unused_public_text}")
endif()
message(STATUS
    "OAP target boundary check passed: "
    "${OAP_SCANNED_TARGET_COUNT} targets, "
    "${OAP_SCANNED_SOURCE_COUNT} files, "
    "${OAP_SCANNED_INCLUDE_COUNT} first-party includes, "
    "${OAP_CROSS_TARGET_INCLUDE_COUNT} cross-target public includes, "
    "${OAP_UNUSED_PUBLIC_HEADER_COUNT} unused public headers")

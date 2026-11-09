# Verify the workflow ownership rules independently of the physical target
# declarations, so newly added files cannot bypass the component DAG.

if(NOT DEFINED OAP_SOURCE_DIR)
    get_filename_component(OAP_SOURCE_DIR
        "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

function(oap_workflow_component relative_path output)
    string(REPLACE "\\" "/" normalized "${relative_path}")
    if(normalized MATCHES
       "^src/app/workflows/shared/(UndoHistory|ToolpathExportController|ToolpathGenerationValidation|ToolpathRequestBuilder)\\.(h|cpp)$" OR
       normalized STREQUAL
       "src/app/workflows/shared/detail/ToolpathRequestBuilder.h")
        set(component cam)
    elseif(normalized MATCHES "^src/app/workflows/shared/")
        set(component base)
    elseif(normalized MATCHES "^src/app/workflows/playback/")
        set(component playback)
    elseif(normalized MATCHES
       "^src/app/workflows/hole/HoleStockSimulationRunner\\.(h|cpp)$" OR
       normalized MATCHES "^src/app/workflows/simulation/")
        set(component simulation)
    elseif(normalized MATCHES
       "^src/app/workflows/(hole|pocket|profile|engraving|project)/")
        set(component cam)
    else()
        set(component unknown)
    endif()
    set(${output} "${component}" PARENT_SCOPE)
endfunction()

set(OAP_WORKFLOW_ALLOWED_base)
set(OAP_WORKFLOW_ALLOWED_playback base)
set(OAP_WORKFLOW_ALLOWED_cam base playback)
set(OAP_WORKFLOW_ALLOWED_simulation base cam)

file(GLOB_RECURSE workflow_files
    RELATIVE "${OAP_SOURCE_DIR}"
    "${OAP_SOURCE_DIR}/src/app/workflows/*.h"
    "${OAP_SOURCE_DIR}/src/app/workflows/*.cpp"
    "${OAP_SOURCE_DIR}/src/app/workflows/*.inc")

set(violations)
set(scanned_include_count 0)
set(cross_component_count 0)
foreach(relative_file IN LISTS workflow_files)
    oap_workflow_component("${relative_file}" source_component)
    if(source_component STREQUAL unknown)
        list(APPEND violations
            "${relative_file}: has no internal workflow component owner")
        continue()
    endif()

    file(STRINGS "${OAP_SOURCE_DIR}/${relative_file}" include_lines
        REGEX "^[ \t]*#[ \t]*include[ \t]*\"app/workflows/")
    foreach(include_line IN LISTS include_lines)
        math(EXPR scanned_include_count "${scanned_include_count} + 1")
        string(REGEX REPLACE
            ".*#[ \t]*include[ \t]*\"([^\"]+)\".*" "\\1"
            include_path "${include_line}")
        set(included_relative "src/${include_path}")
        if(NOT EXISTS "${OAP_SOURCE_DIR}/${included_relative}")
            list(APPEND violations
                "${relative_file}: unresolved workflow include '${include_path}'")
            continue()
        endif()
        oap_workflow_component("${included_relative}" target_component)
        if(target_component STREQUAL unknown)
            list(APPEND violations
                "${relative_file}: include '${include_path}' has no component owner")
        elseif(NOT target_component STREQUAL source_component)
            math(EXPR cross_component_count "${cross_component_count} + 1")
            set(allowed ${OAP_WORKFLOW_ALLOWED_${source_component}})
            list(FIND allowed "${target_component}" allowed_index)
            if(allowed_index LESS 0)
                list(APPEND violations
                    "${relative_file}: forbidden internal workflow dependency "
                    "${source_component} -> ${target_component} via '${include_path}'")
            endif()
        endif()
    endforeach()
endforeach()

if(violations)
    list(JOIN violations "\n  " violation_text)
    message(FATAL_ERROR
        "Workflow boundary violations:\n  ${violation_text}")
endif()

list(LENGTH workflow_files workflow_file_count)
message(STATUS
    "Workflow boundaries OK: ${workflow_file_count} files, "
    "${scanned_include_count} internal includes, "
    "${cross_component_count} cross-component includes")

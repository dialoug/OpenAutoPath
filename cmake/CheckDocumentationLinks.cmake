# Validate relative Markdown-to-Markdown links in first-party, non-ignored files.
# git supplies the file set so ignored local docs, SDKs and build trees never
# become architecture inputs or CI prerequisites.

cmake_minimum_required(VERSION 3.21)

if(NOT DEFINED OAP_SOURCE_DIR OR NOT IS_DIRECTORY "${OAP_SOURCE_DIR}/src")
    message(FATAL_ERROR
        "OAP_SOURCE_DIR must point to the OpenAutoPath source root")
endif()

execute_process(
    COMMAND git -C "${OAP_SOURCE_DIR}" ls-files
        --cached --others --exclude-standard -- "*.md"
    RESULT_VARIABLE git_result
    OUTPUT_VARIABLE markdown_output
    ERROR_VARIABLE git_error
    OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT git_result EQUAL 0)
    message(FATAL_ERROR
        "Unable to enumerate first-party Markdown files: ${git_error}")
endif()

string(REPLACE "\r\n" "\n" markdown_output "${markdown_output}")
string(REPLACE "\n" ";" markdown_files "${markdown_output}")
set(link_errors)
set(link_count 0)
foreach(relative_document IN LISTS markdown_files)
    if(relative_document STREQUAL "")
        continue()
    endif()
    set(document "${OAP_SOURCE_DIR}/${relative_document}")
    if(NOT EXISTS "${document}")
        continue()
    endif()
    file(STRINGS "${document}" document_lines)
    get_filename_component(document_directory "${document}" DIRECTORY)
    foreach(document_line IN LISTS document_lines)
        set(link_search "${document_line}")
        while(link_search MATCHES
              [=[\]\(([^#\)]*\.md)(#[^\)]*)?\)]=])
            set(markdown_link "${CMAKE_MATCH_0}")
            set(link_target "${CMAKE_MATCH_1}")
            string(FIND "${link_search}" "${markdown_link}" match_index)
            string(LENGTH "${markdown_link}" match_length)
            string(LENGTH "${link_search}" search_length)
            math(EXPR remainder_index "${match_index} + ${match_length}")
            math(EXPR remainder_length
                "${search_length} - ${remainder_index}")
            string(SUBSTRING "${link_search}" ${remainder_index}
                ${remainder_length} link_search)

            if(link_target MATCHES "^[A-Za-z][A-Za-z0-9+.-]*://")
                continue()
            endif()
            string(REPLACE "%20" " " link_target "${link_target}")
            get_filename_component(
                absolute_target "${link_target}" ABSOLUTE
                BASE_DIR "${document_directory}")
            math(EXPR link_count "${link_count} + 1")
            if(NOT EXISTS "${absolute_target}")
                list(APPEND link_errors
                    "${relative_document}: missing '${link_target}'")
            endif()
        endwhile()
    endforeach()
endforeach()

if(link_errors)
    list(REMOVE_DUPLICATES link_errors)
    list(JOIN link_errors "\n  - " error_text)
    message(FATAL_ERROR
        "OAP documentation link check failed:\n  - ${error_text}")
endif()

list(LENGTH markdown_files markdown_count)
message(STATUS
    "OAP documentation link check passed: "
    "${markdown_count} documents, ${link_count} local Markdown links")

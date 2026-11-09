# The installed core API is the core subset of the cross-target API manifest,
# plus explicitly reviewed support types needed by those headers.
# Keep it closed under first-party includes: installing private directories to
# compensate for an incomplete public manifest hides accidental API leakage.
include("${CMAKE_CURRENT_LIST_DIR}/OAPPublicHeaders.cmake")
set(OAP_SDK_HEADERS ${OAP_PUBLIC_TARGET_HEADERS})
list(FILTER OAP_SDK_HEADERS INCLUDE REGEX
    "^src/(analysis|recognition|toolpath|tooling|manufacturing)/")
# MachiningJob embeds this store by value; IpwExecutor exposes its checkpoint
# type. Installing it does not authorize new cross-module source dependencies.
set(OAP_SDK_SUPPORT_HEADERS src/manufacturing/ipw/IpwCheckpointStore.h)
list(APPEND OAP_SDK_HEADERS ${OAP_SDK_SUPPORT_HEADERS})
list(REMOVE_DUPLICATES OAP_SDK_HEADERS)
list(SORT OAP_SDK_HEADERS)

function(oap_validate_sdk_headers source_root)
    set(referenced_headers)
    foreach(header IN LISTS OAP_SDK_HEADERS)
        if(NOT EXISTS "${source_root}/${header}")
            message(FATAL_ERROR "SDK header does not exist: ${header}")
        endif()
        get_filename_component(header_directory "${source_root}/${header}" DIRECTORY)
        file(STRINGS "${source_root}/${header}" include_lines
            REGEX "^[ \t]*#[ \t]*include[ \t]*[<\"]")
        foreach(line IN LISTS include_lines)
            if(NOT line MATCHES "^[ \t]*#[ \t]*include[ \t]*[<\"]([^>\"]+)[>\"]")
                continue()
            endif()
            set(include_name "${CMAKE_MATCH_1}")
            foreach(candidate "${header_directory}/${include_name}"
                    "${source_root}/src/${include_name}")
                if(EXISTS "${candidate}")
                    get_filename_component(candidate "${candidate}" ABSOLUTE)
                    file(RELATIVE_PATH dependency "${source_root}" "${candidate}")
                    if(NOT dependency IN_LIST OAP_SDK_HEADERS)
                        message(FATAL_ERROR
                            "SDK header ${header} includes non-public header ${dependency}")
                    endif()
                    list(APPEND referenced_headers "${dependency}")
                    break()
                endif()
            endforeach()
        endforeach()
    endforeach()
    foreach(header IN LISTS OAP_SDK_SUPPORT_HEADERS)
        if(NOT header IN_LIST referenced_headers)
            message(FATAL_ERROR "Unused SDK support header: ${header}")
        endif()
    endforeach()
endfunction()

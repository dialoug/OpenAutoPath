# Fast, versioned regression checks. Keep this list independent of ignored STEP
# fixtures so a clean source checkout can execute it deterministically.

set(OAP_TEST_RUNTIME_DIRECTORIES
    "${OAP_QT_BIN_DIR}"
    "${OAP_OCCT_RUNTIME_DIR}"
    "$<TARGET_FILE_DIR:oap_opencamlib>")
foreach(runtime_dll IN LISTS OAP_THIRDPARTY_DLLS)
    get_filename_component(runtime_dir "${runtime_dll}" DIRECTORY)
    list(APPEND OAP_TEST_RUNTIME_DIRECTORIES "${runtime_dir}")
endforeach()
if(OAP_VCOMP_RUNTIME_DLL)
    get_filename_component(vcomp_runtime_dir
        "${OAP_VCOMP_RUNTIME_DLL}" DIRECTORY)
    list(APPEND OAP_TEST_RUNTIME_DIRECTORIES "${vcomp_runtime_dir}")
endif()
list(REMOVE_DUPLICATES OAP_TEST_RUNTIME_DIRECTORIES)
list(JOIN OAP_TEST_RUNTIME_DIRECTORIES ";" OAP_TEST_RUNTIME_PATH)

function(oap_configure_fast_test target)
    oap_configure_first_party_includes(${target})
    oap_place_test(${target})
    oap_enable_utf8(${target})
    oap_enable_core_guideline_warnings(${target})
    oap_group_sources(${target})
    set_target_properties(${target} PROPERTIES FOLDER "tests")
    if(MSVC)
        set_target_properties(${target} PROPERTIES
            VS_DEBUGGER_ENVIRONMENT
                "PATH=${OAP_TEST_RUNTIME_PATH};%PATH%")
    endif()
endfunction()

include(${CMAKE_CURRENT_LIST_DIR}/tests/OAPTestTargets.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/tests/OAPTestRegistrations.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/tests/OAPArchitectureTests.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/tests/OAPTestLabels.cmake)

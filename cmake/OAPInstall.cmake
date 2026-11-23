include(CMakePackageConfigHelpers)
option(OAP_INSTALL_DEVELOPMENT "Install core libraries, headers and CMake package" ON)

if(OAP_INSTALL_DEVELOPMENT)
    include("${CMAKE_CURRENT_LIST_DIR}/OAPSdkHeaders.cmake")
    oap_validate_sdk_headers("${PROJECT_SOURCE_DIR}")
    set(oap_core_exports
        "oap_analysis|Analysis" "oap_recognition|Recognition"
        "oap_toolpath_core|ToolpathCore" "oap_toolpath_program|ToolpathProgram"
        "oap_toolpath_hole|HoleToolpath" "oap_toolpath_pocket|PocketToolpath"
        "oap_toolpath_profile|ProfileToolpath" "oap_toolpath_finishing|FinishingToolpath"
        "oap_toolpath_algorithms|ToolpathAlgorithms" "oap_toolpath_workflow|ToolpathWorkflow"
        "oap_toollibrary|ToolLibrary" "oap_toolpath_adapter|ToolpathAdapter"
        "oap_manufacturing|Manufacturing")
    set(oap_install_targets oap_opencamlib oap_support)
    set_target_properties(oap_opencamlib PROPERTIES EXPORT_NAME OpenCAMLib)
    foreach(entry IN LISTS oap_core_exports)
        string(REPLACE "|" ";" entry "${entry}")
        list(GET entry 0 target)
        list(GET entry 1 export_name)
        set_target_properties(${target} PROPERTIES EXPORT_NAME ${export_name})
        target_include_directories(${target} INTERFACE
            "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/OpenAutoPath>")
        list(APPEND oap_install_targets ${target})
    endforeach()
    install(TARGETS ${oap_install_targets} EXPORT OpenAutoPathTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Development
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Development
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Development)
    foreach(header IN LISTS OAP_SDK_HEADERS)
        string(REGEX REPLACE "^src/" "" relative_header "${header}")
        get_filename_component(header_directory "${relative_header}" DIRECTORY)
        install(FILES "${PROJECT_SOURCE_DIR}/${header}"
            DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/OpenAutoPath/${header_directory}"
            COMPONENT Development)
    endforeach()
    install(DIRECTORY "${OAP_CLIPPER2_INCLUDE_DIR}/clipper2"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/OpenAutoPath/thirdparty
        COMPONENT Development FILES_MATCHING PATTERN "*.h")
    set(oap_package_dir "${CMAKE_INSTALL_LIBDIR}/cmake/OpenAutoPath")
    configure_package_config_file(
        "${CMAKE_CURRENT_LIST_DIR}/OpenAutoPathConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/OpenAutoPathConfig.cmake"
        INSTALL_DESTINATION "${oap_package_dir}")
    install(EXPORT OpenAutoPathTargets NAMESPACE OAP::
        DESTINATION "${oap_package_dir}" COMPONENT Development)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/OpenAutoPathConfig.cmake"
        "${CMAKE_CURRENT_LIST_DIR}/dependencies/OAPOCCT.cmake"
        DESTINATION "${oap_package_dir}" COMPONENT Development)
endif()

install(TARGETS oap-worker RUNTIME_DEPENDENCY_SET oap_worker_runtime
    RUNTIME DESTINATION bin COMPONENT ServerWorker)
install(TARGETS oap_opencamlib
    RUNTIME DESTINATION bin COMPONENT ServerWorker
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT ServerWorker)
if(WIN32)
    set(worker_dll_dirs "${OAP_OCCT_RUNTIME_DIR}")
    foreach(dll IN LISTS OAP_THIRDPARTY_DLLS OAP_VCOMP_RUNTIME_DLL)
        get_filename_component(dll_dir "${dll}" DIRECTORY)
        list(APPEND worker_dll_dirs "${dll_dir}")
    endforeach()
    list(REMOVE_DUPLICATES worker_dll_dirs)
    install(RUNTIME_DEPENDENCY_SET oap_worker_runtime
        DIRECTORIES ${worker_dll_dirs}
        PRE_EXCLUDE_REGEXES "^(api|ext)-ms-"
        POST_EXCLUDE_REGEXES "[/\\\\][Ss]ystem32[/\\\\]"
        RUNTIME DESTINATION bin COMPONENT ServerWorker)
    install(DIRECTORY "${PROJECT_SOURCE_DIR}/licenses/runtime/"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/OpenAutoPath/runtime
        COMPONENT ServerWorker)
endif()

foreach(component Development ServerWorker)
    install(FILES "${PROJECT_SOURCE_DIR}/LICENSE"
        "${PROJECT_SOURCE_DIR}/THIRD_PARTY_NOTICES.md"
        "${PROJECT_SOURCE_DIR}/licenses/nlohmann-json-MIT.txt"
        "${PROJECT_SOURCE_DIR}/licenses/AnalysisSitus-BSD-3-Clause.txt"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/OpenAutoPath
        COMPONENT ${component})
    install(FILES "${OAP_OPENCAMLIB_LICENSE_FILE}"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/OpenAutoPath
        RENAME OpenCAMLib-COPYING.txt COMPONENT ${component})
    install(FILES "${OAP_OPENCAMLIB_NOTICE_FILE}"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/OpenAutoPath COMPONENT ${component})
    install(FILES "${oap_ocl_boost_include}/LICENSE_1_0.txt"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/OpenAutoPath
        RENAME Boost-LICENSE_1_0.txt COMPONENT ${component})
    if(EXISTS "${OAP_CLIPPER2_SOURCE_DIR}/../../../LICENSE")
        install(FILES "${OAP_CLIPPER2_SOURCE_DIR}/../../../LICENSE"
            DESTINATION ${CMAKE_INSTALL_DATADIR}/licenses/OpenAutoPath
            RENAME Clipper2-LICENSE.txt COMPONENT ${component})
    endif()
endforeach()
install(FILES BUILDING.md DESTINATION ${CMAKE_INSTALL_DATADIR}/doc/OpenAutoPath)

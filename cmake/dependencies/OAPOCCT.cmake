# A stable target interface for either the bundled SDK or a native OCCT package.
include_guard(GLOBAL)
set(OAP_OCCT_CORE_LIBRARIES
    TKDESTEP TKXSBase TKDE TKDECascade
    TKXCAF TKLCAF TKCAF TKCDF
    TKBinXCAF TKBin TKBinL TKStd TKStdL TKService
    TKMesh TKShHealing TKTopAlgo TKGeomAlgo
    TKBO TKBool TKFillet TKOffset TKPrim
    TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel)
set(OAP_OCCT_VIEWER_LIBRARIES TKV3d TKOpenGl)

function(oap_load_occt_windows_sdk sdk_root)
    if(NOT WIN32)
        message(FATAL_ERROR "The prebuilt OCCT Windows SDK cannot be used on this platform")
    endif()
    set(include_dir "${sdk_root}/inc")
    set(library_dir "${sdk_root}/win64/vc14/lib")
    set(runtime_dir "${sdk_root}/win64/vc14/bin")
    if(NOT EXISTS "${include_dir}/Standard_Version.hxx")
        message(FATAL_ERROR "OAP_OCCT_ROOT must point to a valid Windows SDK: ${sdk_root}")
    endif()
    set(toolkits ${OAP_OCCT_CORE_LIBRARIES})
    if(OAP_BUILD_GUI)
        list(APPEND toolkits ${OAP_OCCT_VIEWER_LIBRARIES})
    endif()
    foreach(toolkit IN LISTS toolkits)
        if(TARGET OCCT::${toolkit})
            continue()
        endif()
        foreach(required_file "${library_dir}/${toolkit}.lib" "${runtime_dir}/${toolkit}.dll")
            if(NOT EXISTS "${required_file}")
                message(FATAL_ERROR "Required OCCT release library is missing: ${required_file}")
            endif()
        endforeach()
        add_library(OCCT::${toolkit} SHARED IMPORTED)
        set_target_properties(OCCT::${toolkit} PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${include_dir}"
            IMPORTED_CONFIGURATIONS RELEASE
            IMPORTED_IMPLIB_RELEASE "${library_dir}/${toolkit}.lib"
            IMPORTED_LOCATION_RELEASE "${runtime_dir}/${toolkit}.dll"
            MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release
            MAP_IMPORTED_CONFIG_MINSIZEREL Release)
    endforeach()
    # Imported targets must not silently map a Debug consumer to release DLLs.
    if("Debug" IN_LIST CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE STREQUAL "Debug")
        message(FATAL_ERROR "The OCCT Windows SDK has no Debug binaries. Use Release or RelWithDebInfo configurations.")
    endif()
    set(OAP_OCCT_INCLUDE_DIR "${include_dir}" PARENT_SCOPE)
    set(OAP_OCCT_LIBRARY_DIR "${library_dir}" PARENT_SCOPE)
    set(OAP_OCCT_RUNTIME_DIR "${runtime_dir}" PARENT_SCOPE)
endfunction()

function(oap_load_occt_package)
    # Pin the same API baseline as Windows. OCCT's version config uses exact
    # compatibility; do not silently accept the incompatible OCCT 8 API.
    find_package(OpenCASCADE 7.9.3 EXACT CONFIG REQUIRED COMPONENTS
        FoundationClasses ModelingData ModelingAlgorithms Visualization
        ApplicationFramework DataExchange)
    set(toolkits ${OAP_OCCT_CORE_LIBRARIES})
    if(OAP_BUILD_GUI)
        list(APPEND toolkits ${OAP_OCCT_VIEWER_LIBRARIES})
    endif()
    foreach(toolkit IN LISTS toolkits)
        if(TARGET OCCT::${toolkit})
            continue()
        endif()
        if(TARGET OpenCASCADE::${toolkit})
            set(upstream_target OpenCASCADE::${toolkit})
        elseif(TARGET ${toolkit})
            set(upstream_target ${toolkit})
        else()
            message(FATAL_ERROR "The OpenCASCADE package does not export ${toolkit}")
        endif()
        if(WIN32 AND ("Debug" IN_LIST CMAKE_CONFIGURATION_TYPES OR CMAKE_BUILD_TYPE STREQUAL "Debug"))
            get_target_property(debug_library ${upstream_target} IMPORTED_IMPLIB_DEBUG)
            if(NOT debug_library)
                message(FATAL_ERROR "${upstream_target} has no Debug import library; use release-compatible configurations")
            endif()
        endif()
        add_library(OCCT::${toolkit} INTERFACE IMPORTED)
        set_target_properties(OCCT::${toolkit} PROPERTIES
            INTERFACE_LINK_LIBRARIES "${upstream_target}")
    endforeach()
    set(OAP_OCCT_INCLUDE_DIR "${OpenCASCADE_INCLUDE_DIR}" PARENT_SCOPE)
    set(OAP_OCCT_LIBRARY_DIR "${OpenCASCADE_LIBRARY_DIR}" PARENT_SCOPE)
    set(OAP_OCCT_RUNTIME_DIR "${OpenCASCADE_BINARY_DIR}" PARENT_SCOPE)
endfunction()

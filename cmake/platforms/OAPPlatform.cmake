if(WIN32)
    set(oap_occt_provider_default WINDOWS_SDK)
else()
    set(oap_occt_provider_default PACKAGE)
endif()
set(OAP_OCCT_PROVIDER "${oap_occt_provider_default}" CACHE STRING
    "OCCT dependency provider: WINDOWS_SDK or PACKAGE")
set_property(CACHE OAP_OCCT_PROVIDER PROPERTY STRINGS WINDOWS_SDK PACKAGE)
if(NOT OAP_OCCT_PROVIDER MATCHES "^(WINDOWS_SDK|PACKAGE)$")
    message(FATAL_ERROR "OAP_OCCT_PROVIDER must be WINDOWS_SDK or PACKAGE")
endif()
if(NOT oap_multi_config AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build configuration" FORCE)
endif()

if(WIN32)
    include("${CMAKE_CURRENT_LIST_DIR}/OAPWindows.cmake")
elseif(UNIX)
    if(OAP_OCCT_PROVIDER STREQUAL "WINDOWS_SDK")
        message(FATAL_ERROR "WINDOWS_SDK cannot be used on this platform. Use OAP_OCCT_PROVIDER=PACKAGE.")
    endif()
    if(OAP_BUILD_GUI)
        message(FATAL_ERROR
            "Desktop GUI porting is not complete on this platform. Use OAP_BUILD_GUI=OFF for core libraries and tests.")
    endif()
else()
    message(FATAL_ERROR "Supported build platforms are Windows, Linux and macOS")
endif()

# Only this prebuilt Windows SDK requires release-compatible configurations.
# Native package builds keep their compiler's Debug/Release choices.
if(OAP_OCCT_PROVIDER STREQUAL "WINDOWS_SDK")
    if(oap_multi_config)
        set(oap_active_configs ${CMAKE_CONFIGURATION_TYPES})
    else()
        set(oap_active_configs ${CMAKE_BUILD_TYPE})
    endif()
    foreach(oap_config IN LISTS oap_active_configs)
        if(NOT oap_config MATCHES "^(Release|RelWithDebInfo|MinSizeRel)$")
            message(FATAL_ERROR
                "Configuration '${oap_config}' is unsupported by the release-only OCCT SDK. "
                "Use RelWithDebInfo, Release or MinSizeRel; set CMAKE_CONFIGURATION_TYPES "
                "(multi-config) or CMAKE_BUILD_TYPE (single-config).")
        endif()
    endforeach()
endif()

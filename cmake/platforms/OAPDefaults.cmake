# Applied before project() initializes multi-configuration defaults.
get_property(oap_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(CMAKE_HOST_WIN32 AND oap_multi_config AND
   (NOT DEFINED OAP_OCCT_PROVIDER OR OAP_OCCT_PROVIDER STREQUAL "WINDOWS_SDK"))
    set(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release;MinSizeRel" CACHE STRING
        "Configurations compatible with the bundled release-only Windows SDK")
endif()

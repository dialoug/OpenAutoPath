set(oap_thirdparty_root_default "$ENV{OAP_THIRDPARTY_ROOT}")
if(NOT oap_thirdparty_root_default)
    set(oap_thirdparty_root_default
        "${PROJECT_SOURCE_DIR}/external/3rdparty-vc14-64")
endif()
set(OAP_THIRDPARTY_ROOT "${oap_thirdparty_root_default}"
    CACHE PATH "OCCT third-party runtime root")
if(NOT OAP_THIRDPARTY_ROOT)
    message(FATAL_ERROR
        "OAP_THIRDPARTY_ROOT is required for the OCCT runtime dependencies.")
endif()

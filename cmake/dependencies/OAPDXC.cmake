# Deploy the reviewed DXC release, independent of the host Windows SDK version.
set(OAP_DXC_ROOT "$ENV{OAP_DXC_ROOT}" CACHE PATH
    "Optional extracted dxc_2026_02_20.zip root; binaries must match the reviewed release")
set(oap_dxc_root "${OAP_DXC_ROOT}")
if(NOT oap_dxc_root)
    include(FetchContent)
    cmake_policy(PUSH)
    if(POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif()
    FetchContent_Declare(oap_dxc_runtime
        URL https://github.com/microsoft/DirectXShaderCompiler/releases/download/v1.9.2602/dxc_2026_02_20.zip
        URL_HASH SHA256=a1e89031421cf3c1fca6627766ab3020ca4f962ac7e2caa7fab2b33a8436151e
        SOURCE_SUBDIR bin)
    FetchContent_MakeAvailable(oap_dxc_runtime)
    cmake_policy(POP)
    set(oap_dxc_root "${oap_dxc_runtime_SOURCE_DIR}")
endif()

set(oap_dxc_dxcompiler_sha256 b86a738ece4c05dbe2d9bbb29668a2ccb28a0740773c9b027cdc61e8d07b4d75)
set(oap_dxc_dxil_sha256 058f2f52a680c38b223a5615b7df969def21f77e577562e5099d971dede992de)
set(OAP_DXC_RUNTIME_DLLS)
foreach(library dxcompiler dxil)
    set(dll "${oap_dxc_root}/bin/x64/${library}.dll")
    if(NOT EXISTS "${dll}")
        message(FATAL_ERROR "Reviewed DXC runtime missing: ${dll}")
    endif()
    file(SHA256 "${dll}" actual_sha256)
    if(NOT actual_sha256 STREQUAL oap_dxc_${library}_sha256)
        message(FATAL_ERROR "DXC runtime does not match the reviewed 1.9.2602 release: ${dll}")
    endif()
    list(APPEND OAP_DXC_RUNTIME_DLLS "${dll}")
endforeach()

set(OAP_FREEIMAGE_ROOT "${OAP_THIRDPARTY_ROOT}/freeimage-3.18.0-x64" CACHE PATH
    "FreeImage runtime SDK; may point to an SDK built by Build-WindowsFreeImage.ps1")
set(OAP_THIRDPARTY_DLLS
    "${OAP_THIRDPARTY_ROOT}/freetype-2.13.3-x64/bin/freetype.dll"
    "${OAP_FREEIMAGE_ROOT}/bin/FreeImage.dll"
    "${OAP_THIRDPARTY_ROOT}/tbb-2021.13.0-x64/bin/tbb12.dll"
    "${OAP_THIRDPARTY_ROOT}/jemalloc-vc14-64/bin/jemalloc.dll"
    "${OAP_THIRDPARTY_ROOT}/openvr-1.14.15-64/bin/win64/openvr_api.dll"
    "${OAP_THIRDPARTY_ROOT}/ffmpeg-3.3.4-64/bin/avcodec-57.dll"
    "${OAP_THIRDPARTY_ROOT}/ffmpeg-3.3.4-64/bin/avformat-57.dll"
    "${OAP_THIRDPARTY_ROOT}/ffmpeg-3.3.4-64/bin/avutil-55.dll"
    "${OAP_THIRDPARTY_ROOT}/ffmpeg-3.3.4-64/bin/swscale-4.dll")

# FreeImage 使用 Visual C++ OpenMP 运行库。优先使用显式路径，其次只在当前
# CMake 选择的 VS 实例或开发者命令行公布的 Redist 根下查找。
set(OAP_VCOMP_RUNTIME_DLL "$ENV{OAP_VCOMP_RUNTIME_DLL}" CACHE FILEPATH
    "Optional explicit path to the redistributable vcomp140.dll")
if(OAP_VCOMP_RUNTIME_DLL AND NOT EXISTS "${OAP_VCOMP_RUNTIME_DLL}")
    message(FATAL_ERROR
        "OAP_VCOMP_RUNTIME_DLL does not exist: ${OAP_VCOMP_RUNTIME_DLL}")
endif()

if(NOT OAP_VCOMP_RUNTIME_DLL)
    set(oap_vcomp_search_roots)
    if(DEFINED ENV{VCToolsRedistDir})
        list(APPEND oap_vcomp_search_roots "$ENV{VCToolsRedistDir}")
    endif()
    if(CMAKE_GENERATOR_INSTANCE)
        list(APPEND oap_vcomp_search_roots
            "${CMAKE_GENERATOR_INSTANCE}/VC/Redist/MSVC")
    endif()

    set(oap_vcomp_candidates)
    foreach(redist_root IN LISTS oap_vcomp_search_roots)
        file(GLOB oap_vcomp_root_candidates
            "${redist_root}/x64/Microsoft.VC*.OpenMP/vcomp140.dll"
            "${redist_root}/*/x64/Microsoft.VC*.OpenMP/vcomp140.dll")
        list(APPEND oap_vcomp_candidates ${oap_vcomp_root_candidates})
    endforeach()
    if(oap_vcomp_candidates)
        list(SORT oap_vcomp_candidates COMPARE NATURAL ORDER DESCENDING)
        list(GET oap_vcomp_candidates 0 OAP_VCOMP_RUNTIME_DLL)
    endif()
endif()

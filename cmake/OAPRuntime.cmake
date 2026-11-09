# GUI 目标的运行时部署配置。将部署逻辑与目标定义分离，便于根构建文件
# 清晰表达项目架构。

# 仅部署实际参与链接的 OCCT 工具包。旧逻辑复制整个 SDK bin，连同 DRAWEXE、
# Inspector 和数十个未使用 DLL 一起进入应用目录，既混乱又放大分发体积。
# 部分 OCCT 工具包通过 DLL 导入表间接依赖持久化、XCAF 和隐藏线模块；
# 它们不会出现在本项目的链接行中，但 Windows 加载器仍要求随应用部署。
set(OAP_OCCT_TRANSITIVE_RUNTIME_LIBRARIES
    TKBinTObj TKHLR TKTObj TKVCAF
    TKXml TKXmlL TKXmlTObj TKXmlXCAF)

set(OAP_OCCT_RUNTIME_DLLS)
foreach(toolkit IN LISTS
        OAP_OCCT_CORE_LIBRARIES
        OAP_OCCT_VIEWER_LIBRARIES
        OAP_OCCT_TRANSITIVE_RUNTIME_LIBRARIES)
    set(runtime_dll "${OAP_OCCT_RUNTIME_DIR}/${toolkit}.dll")
    if(NOT EXISTS "${runtime_dll}")
        message(FATAL_ERROR "Required OpenCASCADE runtime is missing: ${runtime_dll}")
    endif()
    list(APPEND OAP_OCCT_RUNTIME_DLLS "${runtime_dll}")
endforeach()
list(REMOVE_DUPLICATES OAP_OCCT_RUNTIME_DLLS)

add_custom_command(TARGET OpenAutoPathApp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${OAP_OCCT_RUNTIME_DLLS} "$<TARGET_FILE_DIR:OpenAutoPathApp>"
    COMMENT "Copying required OpenCASCADE DLLs")

# Qt6::qmake 与已链接的 Qt targets 来自同一个 package。由它确定部署工具目录，
# 避免 PATH 中的 Anaconda 或其他 Qt 安装被 find_program 误选。
get_target_property(OAP_QMAKE_EXECUTABLE Qt6::qmake IMPORTED_LOCATION)
if(OAP_QMAKE_EXECUTABLE)
    get_filename_component(OAP_QT_BIN_DIR "${OAP_QMAKE_EXECUTABLE}" DIRECTORY)
    find_program(OAP_QT_DEPLOY_TOOL
        NAMES windeployqt
        PATHS "${OAP_QT_BIN_DIR}"
        NO_DEFAULT_PATH
        NO_CACHE)
endif()

if(OAP_QT_DEPLOY_TOOL)
    add_custom_command(TARGET OpenAutoPathApp POST_BUILD
        COMMAND "${OAP_QT_DEPLOY_TOOL}" --no-translations
                "$<TARGET_FILE:OpenAutoPathApp>"
        COMMENT "Deploying Qt runtime")
else()
    message(FATAL_ERROR
        "windeployqt was not found beside the Qt installation selected by "
        "find_package(Qt6): ${OAP_QT_BIN_DIR}")
endif()

set(OAP_THIRDPARTY_DLLS
    "${OAP_THIRDPARTY_ROOT}/freetype-2.13.3-x64/bin/freetype.dll"
    "${OAP_THIRDPARTY_ROOT}/freeimage-3.18.0-x64/bin/FreeImage.dll"
    "${OAP_THIRDPARTY_ROOT}/tbb-2021.13.0-x64/bin/tbb12.dll"
    "${OAP_THIRDPARTY_ROOT}/jemalloc-vc14-64/bin/jemalloc.dll"
    "${OAP_THIRDPARTY_ROOT}/openvr-1.14.15-64/bin/win64/openvr_api.dll"
    "${OAP_THIRDPARTY_ROOT}/ffmpeg-3.3.4-64/bin/avcodec-57.dll"
    "${OAP_THIRDPARTY_ROOT}/ffmpeg-3.3.4-64/bin/avformat-57.dll"
    "${OAP_THIRDPARTY_ROOT}/ffmpeg-3.3.4-64/bin/avutil-55.dll"
    "${OAP_THIRDPARTY_ROOT}/ffmpeg-3.3.4-64/bin/swscale-4.dll")
foreach(thirdparty_dll IN LISTS OAP_THIRDPARTY_DLLS)
    if(NOT EXISTS "${thirdparty_dll}")
        message(FATAL_ERROR
            "Required OCCT third-party runtime is missing: ${thirdparty_dll}")
    endif()
endforeach()
add_custom_command(TARGET OpenAutoPathApp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${OAP_THIRDPARTY_DLLS} "$<TARGET_FILE_DIR:OpenAutoPathApp>"
    COMMENT "Copying OCCT third-party runtime DLLs")

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

if(OAP_VCOMP_RUNTIME_DLL)
    add_custom_command(TARGET OpenAutoPathApp POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${OAP_VCOMP_RUNTIME_DLL}"
                "$<TARGET_FILE_DIR:OpenAutoPathApp>"
        COMMENT "Copying vcomp140.dll")
else()
    message(WARNING
        "vcomp140.dll was not found. Set OAP_VCOMP_RUNTIME_DLL when packaging "
        "a redistributable build that uses FreeImage.")
endif()

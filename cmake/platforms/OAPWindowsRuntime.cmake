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
    VERBATIM
    COMMENT "Copying required OpenCASCADE DLLs")

# Qt6::qmake 与已链接的 Qt targets 来自同一个 package。由它确定部署工具目录，
# 避免 PATH 中的 Anaconda 或其他 Qt 安装被 find_program 误选。
find_program(OAP_QT_DEPLOY_TOOL
    NAMES windeployqt
    PATHS "${OAP_QT_BIN_DIR}"
    NO_DEFAULT_PATH
    NO_CACHE)

if(OAP_QT_DEPLOY_TOOL)
    # A normal PowerShell/CMake Tools session need not have run VsDevCmd.
    # Give windeployqt the same VS instance CMake selected for compilation.
    set(oap_qt_deploy_environment)
    if(CMAKE_GENERATOR_INSTANCE)
        list(APPEND oap_qt_deploy_environment
            "VCINSTALLDIR=${CMAKE_GENERATOR_INSTANCE}/VC")
    endif()
    # Keep deployment independent of optional modules installed on the build host.
    # We use Windows' D3D compiler and native OpenGL drivers; no SVG assets/plugins.
    add_custom_command(TARGET OpenAutoPathApp POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E env ${oap_qt_deploy_environment}
                "${OAP_QT_DEPLOY_TOOL}" --no-translations --no-system-d3d-compiler
                --no-opengl-sw --exclude-plugins qsvg,qsvgicon
                "$<TARGET_FILE:OpenAutoPathApp>"
        VERBATIM
        COMMENT "Deploying Qt runtime")
else()
    message(FATAL_ERROR
        "windeployqt was not found beside the Qt installation selected by "
        "find_package(Qt6): ${OAP_QT_BIN_DIR}")
endif()

foreach(thirdparty_dll IN LISTS OAP_THIRDPARTY_DLLS)
    if(NOT EXISTS "${thirdparty_dll}")
        message(FATAL_ERROR
            "Required OCCT third-party runtime is missing: ${thirdparty_dll}")
    endif()
endforeach()
add_custom_command(TARGET OpenAutoPathApp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${OAP_THIRDPARTY_DLLS} "$<TARGET_FILE_DIR:OpenAutoPathApp>"
    VERBATIM
    COMMENT "Copying OCCT third-party runtime DLLs")

if(OAP_VCOMP_RUNTIME_DLL)
    add_custom_command(TARGET OpenAutoPathApp POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${OAP_VCOMP_RUNTIME_DLL}"
                "$<TARGET_FILE_DIR:OpenAutoPathApp>"
        VERBATIM
        COMMENT "Copying vcomp140.dll")
else()
    message(WARNING
        "vcomp140.dll was not found. Set OAP_VCOMP_RUNTIME_DLL when packaging "
        "a redistributable build that uses FreeImage.")
endif()

# OpenAutoPath 的第三方依赖查找配置。
#
# 直接从根 CMakeLists.txt 配置时优先使用环境变量；未设置环境变量则回退到
# 当前仓库的标准 Windows 开发目录布局。所有路径仍是缓存变量，可用 -D 覆盖。

set(oap_qt_root_default "$ENV{OAP_QT_ROOT}")
if(NOT oap_qt_root_default)
    set(oap_qt_root_default "C:/Qt/6.11.1/msvc2022_64")
endif()
set(OAP_QT_ROOT "${oap_qt_root_default}" CACHE PATH
    "Qt installation prefix containing lib/cmake/Qt6")
if(OAP_QT_ROOT)
    list(PREPEND CMAKE_PREFIX_PATH "${OAP_QT_ROOT}")
endif()
find_package(Qt6 6.11 REQUIRED COMPONENTS
    Widgets Gui Core OpenGL OpenGLWidgets ShaderTools GuiPrivate)

# Native Vulkan compute/indirect draw interop needs headers only. Device entry
# points come from Qt's Vulkan instance, so no separate loader library is linked.
find_path(OAP_VULKAN_HEADERS_DIR vulkan/vulkan.h HINTS "$ENV{VULKAN_SDK}/Include")
if(NOT OAP_VULKAN_HEADERS_DIR)
    include(FetchContent)
    cmake_policy(PUSH)
    if(POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif()
    FetchContent_Declare(oap_vulkan_headers
        URL https://codeload.github.com/KhronosGroup/Vulkan-Headers/zip/refs/tags/v1.3.290
        URL_HASH SHA256=918a8bb31b8312fd649fd1d1826443da870f33f3187c16c4528aa94927409bab
        # Only populate headers; do not add the dependency's install/test targets.
        SOURCE_SUBDIR include)
    FetchContent_MakeAvailable(oap_vulkan_headers)
    cmake_policy(POP)
    set(OAP_VULKAN_HEADERS_DIR "${oap_vulkan_headers_SOURCE_DIR}/include")
endif()

set(oap_occt_root_default "$ENV{OAP_OCCT_ROOT}")
if(NOT oap_occt_root_default)
    set(oap_occt_root_default
        "${PROJECT_SOURCE_DIR}/external/opencascade-7.9.3-vc14-64")
endif()
set(OAP_OCCT_ROOT "${oap_occt_root_default}"
    CACHE PATH "OpenCASCADE 7.9.3 binary SDK root")
if(NOT OAP_OCCT_ROOT)
    message(FATAL_ERROR
        "OAP_OCCT_ROOT is required. Set the environment variable or pass "
        "-DOAP_OCCT_ROOT=<OpenCASCADE SDK root>.")
endif()
set(OAP_OCCT_INCLUDE_DIR "${OAP_OCCT_ROOT}/inc")
set(OAP_OCCT_LIBRARY_DIR "${OAP_OCCT_ROOT}/win64/vc14/lib")
set(OAP_OCCT_RUNTIME_DIR "${OAP_OCCT_ROOT}/win64/vc14/bin")

foreach(required_path
        OAP_OCCT_INCLUDE_DIR
        OAP_OCCT_LIBRARY_DIR
        OAP_OCCT_RUNTIME_DIR)
    if(NOT EXISTS "${${required_path}}")
        message(FATAL_ERROR
            "${required_path} does not exist: ${${required_path}}. "
            "Set OAP_OCCT_ROOT to a valid OpenCASCADE SDK.")
    endif()
endforeach()

# 导入、XCAF 持久化和几何分析所需的核心工具包。核心库只能传播这组
# 依赖，避免无界面命令行工具因为静态库链接而携带 OpenGL/查看器组件。
set(OAP_OCCT_CORE_LIBRARIES
    TKDESTEP TKXSBase TKDE TKDECascade
    TKXCAF TKLCAF TKCAF TKCDF
    TKBinXCAF TKBin TKBinL TKStd TKStdL
    TKService
    TKMesh TKShHealing TKTopAlgo TKGeomAlgo
    TKBO TKBool TKFillet TKOffset TKPrim
    TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel)

# 仅桌面查看器需要的渲染工具包。
set(OAP_OCCT_VIEWER_LIBRARIES
    TKV3d TKOpenGl)

# Clipper2 is independent from AnalysisSitus. An explicit root may point to an
# official checkout, CPP/Clipper2Lib, or the former flat include/source layout.
# With no root, FetchContent obtains the exact audited 2.0.1 commit.
set(oap_clipper2_root_default "$ENV{OAP_CLIPPER2_ROOT}")
if(NOT oap_clipper2_root_default AND
   EXISTS "${PROJECT_SOURCE_DIR}/external/AnalysisSitus-master/src/asiAlgo/clipper")
    set(oap_clipper2_root_default
        "${PROJECT_SOURCE_DIR}/external/AnalysisSitus-master/src/asiAlgo/clipper")
endif()
set(OAP_CLIPPER2_ROOT "${oap_clipper2_root_default}" CACHE PATH
    "Optional Clipper2 2.0.1 checkout or Clipper2Lib root")
if(OAP_CLIPPER2_ROOT)
    if(EXISTS "${OAP_CLIPPER2_ROOT}/CPP/Clipper2Lib/include/clipper2/clipper.h")
        set(OAP_CLIPPER2_INCLUDE_DIR
            "${OAP_CLIPPER2_ROOT}/CPP/Clipper2Lib/include")
        set(OAP_CLIPPER2_SOURCE_DIR
            "${OAP_CLIPPER2_ROOT}/CPP/Clipper2Lib/src")
    elseif(EXISTS "${OAP_CLIPPER2_ROOT}/include/clipper2/clipper.h")
        set(OAP_CLIPPER2_INCLUDE_DIR "${OAP_CLIPPER2_ROOT}/include")
        set(OAP_CLIPPER2_SOURCE_DIR "${OAP_CLIPPER2_ROOT}/src")
    elseif(EXISTS "${OAP_CLIPPER2_ROOT}/clipper2/clipper.h")
        # Compatibility with the previously used flat AnalysisSitus copy.
        set(OAP_CLIPPER2_INCLUDE_DIR "${OAP_CLIPPER2_ROOT}")
        set(OAP_CLIPPER2_SOURCE_DIR "${OAP_CLIPPER2_ROOT}/clipper2")
    else()
        message(FATAL_ERROR
            "OAP_CLIPPER2_ROOT does not contain a supported Clipper2 layout: "
            "${OAP_CLIPPER2_ROOT}")
    endif()
else()
    include(FetchContent)
    FetchContent_Declare(OAPClipper2
        GIT_REPOSITORY https://github.com/AngusJohnson/Clipper2.git
        GIT_TAG 21ebba05db8894f0c7217ad35ea518080f324946
        GIT_PROGRESS TRUE
        SOURCE_SUBDIR CPP/Clipper2Lib)
    FetchContent_MakeAvailable(OAPClipper2)
    set(OAP_CLIPPER2_INCLUDE_DIR
        "${oapclipper2_SOURCE_DIR}/CPP/Clipper2Lib/include")
    set(OAP_CLIPPER2_SOURCE_DIR
        "${oapclipper2_SOURCE_DIR}/CPP/Clipper2Lib/src")
endif()

foreach(required_clipper_file
        "${OAP_CLIPPER2_INCLUDE_DIR}/clipper2/clipper.h"
        "${OAP_CLIPPER2_SOURCE_DIR}/clipper.engine.cpp"
        "${OAP_CLIPPER2_SOURCE_DIR}/clipper.offset.cpp")
    if(NOT EXISTS "${required_clipper_file}")
        message(FATAL_ERROR
            "Required Clipper2 2.0.1 file is missing: "
            "${required_clipper_file}")
    endif()
endforeach()

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

unset(oap_qt_root_default)
unset(oap_occt_root_default)
unset(oap_clipper2_root_default)
unset(oap_thirdparty_root_default)

include("${CMAKE_CURRENT_LIST_DIR}/OAPOpenCamLib.cmake")

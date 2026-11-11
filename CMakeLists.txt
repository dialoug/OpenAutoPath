# 根构建入口：声明项目选项并装配领域模块与桌面 GUI。
cmake_minimum_required(VERSION 3.22)
include(cmake/platforms/OAPDefaults.cmake)
project(OpenAutoPath LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
option(OAP_BUILD_GUI "Build desktop application, presentation libraries and their tests" ${WIN32})
option(OAP_DEPLOY_RUNTIME "Deploy desktop runtime DLLs after linking" ${WIN32})
include(cmake/platforms/OAPPlatform.cmake)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(OAP_ENABLE_STRICT_WARNINGS
       "Enable strict compiler warnings for first-party targets" ON)

include(cmake/OAPDependencies.cmake)
include(cmake/OAPSources.cmake)
include(cmake/OAPModuleDependencies.cmake)
include(cmake/OAPTargetHelpers.cmake)

# 所有配置共用同一层次，避免 Visual Studio 把 lib、PDB 和测试
# 全部堆到 build/<Config> 根目录。根目录仅保留用户启动的入口程序和运行缓存；
# UI 主体及其 DLL 位于 bin/。路径含 $<CONFIG>，多配置生成器不会再追加配置名。
set(OAP_OUTPUT_ROOT "${CMAKE_BINARY_DIR}/$<CONFIG>")
set(OAP_APP_RUNTIME_OUTPUT_DIR "${OAP_OUTPUT_ROOT}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OAP_APP_RUNTIME_OUTPUT_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OAP_APP_RUNTIME_OUTPUT_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OAP_OUTPUT_ROOT}/lib")
set(CMAKE_PDB_OUTPUT_DIRECTORY "${OAP_OUTPUT_ROOT}/symbols")
# Keep compiler PDBs in CMake's per-target intermediate directories. Unlike
# linker PDBs, they can share the default name (vc143.pdb) across targets.

# ---------------------------------------------------------------------------
# 可复用的核心模块
# ---------------------------------------------------------------------------
# 将与 UI 无关的代码放入库中，供应用与回归测试复用。所有第一方直接依赖
# 由 OAPModuleDependencies.cmake 统一提供给构建和架构门禁。

add_library(oap_analysis STATIC ${OAP_ANALYSIS_SOURCES})
add_library(OAP::Analysis ALIAS oap_analysis)
oap_configure_occt_core_target(oap_analysis PUBLIC)
oap_enable_utf8(oap_analysis)
oap_enable_core_guideline_warnings(oap_analysis)
oap_group_sources(oap_analysis)

add_library(oap_recognition STATIC ${OAP_RECOGNITION_SOURCES})
add_library(OAP::Recognition ALIAS oap_recognition)
target_link_libraries(oap_recognition
    PUBLIC ${OAP_INTERNAL_DEPS_oap_recognition} Threads::Threads)
target_compile_definitions(oap_recognition PRIVATE _USE_MATH_DEFINES)
oap_enable_utf8(oap_recognition)
oap_enable_core_guideline_warnings(oap_recognition)
oap_group_sources(oap_recognition)

add_library(oap_toolpath_core STATIC ${OAP_TOOLPATH_CORE_SOURCES})
add_library(OAP::ToolpathCore ALIAS oap_toolpath_core)
target_include_directories(oap_toolpath_core
    SYSTEM PRIVATE "${OAP_CLIPPER2_INCLUDE_DIR}")
target_link_libraries(oap_toolpath_core
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_core} Threads::Threads)

add_library(oap_toolpath_program STATIC ${OAP_TOOLPATH_PROGRAM_SOURCES})
add_library(OAP::ToolpathProgram ALIAS oap_toolpath_program)
target_link_libraries(oap_toolpath_program
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_program})

add_library(oap_toolpath_hole STATIC ${OAP_TOOLPATH_HOLE_SOURCES})
add_library(OAP::HoleToolpath ALIAS oap_toolpath_hole)
target_link_libraries(oap_toolpath_hole
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_hole})

add_library(oap_toolpath_pocket STATIC ${OAP_TOOLPATH_POCKET_SOURCES})
add_library(OAP::PocketToolpath ALIAS oap_toolpath_pocket)
target_include_directories(oap_toolpath_pocket
    SYSTEM PUBLIC
        "$<BUILD_INTERFACE:${OAP_CLIPPER2_INCLUDE_DIR}>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/OpenAutoPath/thirdparty>")
target_link_libraries(oap_toolpath_pocket
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_pocket})

add_library(oap_toolpath_profile STATIC ${OAP_TOOLPATH_PROFILE_SOURCES})
add_library(OAP::ProfileToolpath ALIAS oap_toolpath_profile)
target_include_directories(oap_toolpath_profile
    SYSTEM PRIVATE "${OAP_CLIPPER2_INCLUDE_DIR}")
target_link_libraries(oap_toolpath_profile
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_profile})

add_library(oap_toolpath_finishing STATIC ${OAP_TOOLPATH_FINISHING_SOURCES})
add_library(OAP::FinishingToolpath ALIAS oap_toolpath_finishing)
target_link_libraries(oap_toolpath_finishing
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_finishing}
    PRIVATE OAP::OpenCAMLib)

add_library(oap_toolpath_algorithms STATIC ${OAP_TOOLPATH_ALGORITHM_SOURCES})
add_library(OAP::ToolpathAlgorithms ALIAS oap_toolpath_algorithms)
target_link_libraries(oap_toolpath_algorithms
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_algorithms})

add_library(oap_toolpath_workflow STATIC ${OAP_TOOLPATH_WORKFLOW_SOURCES})
add_library(OAP::ToolpathWorkflow ALIAS oap_toolpath_workflow)
target_link_libraries(oap_toolpath_workflow
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_workflow})

foreach(toolpath_target
        oap_toolpath_core
        oap_toolpath_hole
        oap_toolpath_pocket
        oap_toolpath_profile
        oap_toolpath_finishing
        oap_toolpath_algorithms
        oap_toolpath_program
        oap_toolpath_workflow)
    oap_enable_utf8(${toolpath_target})
    oap_enable_core_guideline_warnings(${toolpath_target})
    oap_group_sources(${toolpath_target})
endforeach()

add_library(oap_toollibrary STATIC ${OAP_TOOLING_CORE_SOURCES})
add_library(OAP::ToolLibrary ALIAS oap_toollibrary)
target_link_libraries(oap_toollibrary PUBLIC Qt6::Core)
oap_enable_utf8(oap_toollibrary)
oap_enable_core_guideline_warnings(oap_toollibrary)
oap_group_sources(oap_toollibrary)

# 适配器是 tooling 与 toolpath 的唯一依赖交汇点。核心刀库可独立用于
# 导入、编辑和持久化；仅需要生成 CAM 参数的调用方才链接这个目标。
add_library(oap_toolpath_adapter STATIC ${OAP_TOOLPATH_ADAPTER_SOURCES})
add_library(OAP::ToolpathAdapter ALIAS oap_toolpath_adapter)
target_link_libraries(oap_toolpath_adapter
    PUBLIC ${OAP_INTERNAL_DEPS_oap_toolpath_adapter})
oap_enable_utf8(oap_toolpath_adapter)
oap_enable_core_guideline_warnings(oap_toolpath_adapter)
oap_group_sources(oap_toolpath_adapter)

add_library(oap_manufacturing STATIC ${OAP_MANUFACTURING_SOURCES})
add_library(OAP::Manufacturing ALIAS oap_manufacturing)
target_link_libraries(oap_manufacturing
    PUBLIC Qt6::Core ${OAP_INTERNAL_DEPS_oap_manufacturing})
oap_enable_utf8(oap_manufacturing)
oap_enable_core_guideline_warnings(oap_manufacturing)
oap_group_sources(oap_manufacturing)

if(OAP_BUILD_GUI)
add_library(oap_app_view STATIC ${OAP_APP_VIEW_SOURCES})
add_library(OAP::AppView ALIAS oap_app_view)
target_link_libraries(oap_app_view
    PUBLIC ${OAP_INTERNAL_DEPS_oap_app_view})
oap_configure_occt_core_target(oap_app_view PUBLIC)
oap_enable_utf8(oap_app_view)
oap_enable_core_guideline_warnings(oap_app_view)
oap_group_sources(oap_app_view)

# 该层协调桌面端状态与后台领域服务，通过交互端口发布进度和忙碌状态，
# 不依赖具体 Widget、MainWindow 或 OCCT 查看器。
function(oap_add_app_workflow_target target alias_name sources_var)
    add_library(${target} STATIC ${${sources_var}})
    add_library(OAP::${alias_name} ALIAS ${target})
    target_link_libraries(${target}
        PUBLIC Qt6::Core ${OAP_INTERNAL_DEPS_${target}})
    get_target_property(workflow_link_libraries ${target} LINK_LIBRARIES)
    if(Qt6::Widgets IN_LIST workflow_link_libraries)
        message(FATAL_ERROR
            "${target} must use WorkflowInteraction instead of Qt Widgets")
    endif()
    oap_enable_utf8(${target})
    oap_enable_core_guideline_warnings(${target})
    oap_group_sources(${target})
endfunction()

oap_add_app_workflow_target(oap_app_workflow_base AppWorkflowBase
    OAP_APP_WORKFLOW_BASE_SOURCES)
oap_add_app_workflow_target(oap_app_workflow_playback AppWorkflowPlayback
    OAP_APP_WORKFLOW_PLAYBACK_SOURCES)
oap_add_app_workflow_target(oap_app_workflow_cam AppWorkflowCam
    OAP_APP_WORKFLOW_CAM_SOURCES)
qt_add_resources(oap_app_workflow_cam "simulation_tool_library"
    PREFIX "/tool-libraries"
    BASE "resources/tool-libraries"
    FILES resources/tool-libraries/general-simulation.oaptl)
oap_add_app_workflow_target(oap_app_workflow_simulation AppWorkflowSimulation
    OAP_APP_WORKFLOW_SIMULATION_SOURCES)
set_target_properties(
    oap_app_workflow_playback
    oap_app_workflow_simulation
    PROPERTIES AUTOMOC ON)

add_library(oap_app_dialogs STATIC ${OAP_APP_DIALOG_SOURCES})
add_library(OAP::AppDialogs ALIAS oap_app_dialogs)
target_link_libraries(oap_app_dialogs
    PUBLIC Qt6::Widgets ${OAP_INTERNAL_DEPS_oap_app_dialogs})
oap_enable_utf8(oap_app_dialogs)
oap_enable_core_guideline_warnings(oap_app_dialogs)
oap_group_sources(oap_app_dialogs)

add_library(oap_app_presentation STATIC ${OAP_APP_PRESENTATION_SOURCES})
add_library(OAP::AppPresentation ALIAS oap_app_presentation)
oap_configure_occt_viewer_target(oap_app_presentation PUBLIC)
target_link_libraries(oap_app_presentation
    PUBLIC ${OAP_INTERNAL_DEPS_oap_app_presentation}
    PRIVATE Qt6::Core)
oap_enable_utf8(oap_app_presentation)
oap_enable_core_guideline_warnings(oap_app_presentation)
oap_group_sources(oap_app_presentation)

add_library(oap_app_playback STATIC ${OAP_APP_PLAYBACK_SOURCES})
add_library(OAP::AppPlayback ALIAS oap_app_playback)
oap_configure_occt_viewer_target(oap_app_playback PUBLIC)
target_link_libraries(oap_app_playback
    PUBLIC Qt6::Core ${OAP_INTERNAL_DEPS_oap_app_playback})
set_target_properties(oap_app_playback PROPERTIES AUTOMOC ON)
oap_enable_utf8(oap_app_playback)
oap_enable_core_guideline_warnings(oap_app_playback)
oap_group_sources(oap_app_playback)

add_library(oap_app_playback_ui STATIC ${OAP_APP_PLAYBACK_UI_SOURCES})
add_library(OAP::AppPlaybackUi ALIAS oap_app_playback_ui)
oap_configure_occt_viewer_target(oap_app_playback_ui PUBLIC)
target_link_libraries(oap_app_playback_ui
    PUBLIC Qt6::Widgets ${OAP_INTERNAL_DEPS_oap_app_playback_ui})
set_target_properties(oap_app_playback_ui PROPERTIES AUTOMOC ON)
oap_enable_utf8(oap_app_playback_ui)
oap_enable_core_guideline_warnings(oap_app_playback_ui)
oap_group_sources(oap_app_playback_ui)

add_library(oap_app_simulation STATIC ${OAP_APP_SIMULATION_SOURCES})
add_library(OAP::AppSimulation ALIAS oap_app_simulation)
oap_configure_occt_viewer_target(oap_app_simulation PUBLIC)
target_link_libraries(oap_app_simulation
    PUBLIC Qt6::Core ${OAP_INTERNAL_DEPS_oap_app_simulation})
set_target_properties(oap_app_simulation PROPERTIES AUTOMOC ON)
oap_enable_utf8(oap_app_simulation)
oap_enable_core_guideline_warnings(oap_app_simulation)
oap_group_sources(oap_app_simulation)

add_library(oap_app_analysis STATIC ${OAP_APP_ANALYSIS_SOURCES})
add_library(OAP::AppAnalysis ALIAS oap_app_analysis)
set_target_properties(oap_app_analysis PROPERTIES AUTOMOC ON)
target_link_libraries(oap_app_analysis
    PUBLIC Qt6::Core ${OAP_INTERNAL_DEPS_oap_app_analysis})
oap_enable_utf8(oap_app_analysis)
oap_enable_core_guideline_warnings(oap_app_analysis)
oap_group_sources(oap_app_analysis)

add_library(oap_app_recognition STATIC ${OAP_APP_RECOGNITION_SOURCES})
add_library(OAP::AppRecognition ALIAS oap_app_recognition)
oap_configure_occt_viewer_target(oap_app_recognition PUBLIC)
target_link_libraries(oap_app_recognition
    PUBLIC Qt6::Core ${OAP_INTERNAL_DEPS_oap_app_recognition})
oap_enable_utf8(oap_app_recognition)
oap_enable_core_guideline_warnings(oap_app_recognition)
oap_group_sources(oap_app_recognition)

add_library(oap_app_scene STATIC ${OAP_APP_SCENE_SOURCES})
add_library(OAP::AppScene ALIAS oap_app_scene)
oap_configure_occt_viewer_target(oap_app_scene PUBLIC)
target_link_libraries(oap_app_scene
    PUBLIC ${OAP_INTERNAL_DEPS_oap_app_scene})
oap_enable_utf8(oap_app_scene)
oap_enable_core_guideline_warnings(oap_app_scene)
oap_group_sources(oap_app_scene)

add_library(oap_app_document STATIC ${OAP_APP_DOCUMENT_SOURCES})
add_library(OAP::AppDocument ALIAS oap_app_document)
oap_configure_occt_core_target(oap_app_document PUBLIC)
target_link_libraries(oap_app_document
    PUBLIC Qt6::Core ${OAP_INTERNAL_DEPS_oap_app_document})
set_target_properties(oap_app_document PROPERTIES AUTOMOC ON)
oap_enable_utf8(oap_app_document)
oap_enable_core_guideline_warnings(oap_app_document)
oap_group_sources(oap_app_document)

add_library(oap_app_machining STATIC ${OAP_APP_MACHINING_SOURCES})
add_library(OAP::AppMachining ALIAS oap_app_machining)
target_link_libraries(oap_app_machining
    PUBLIC Qt6::Core ${OAP_INTERNAL_DEPS_oap_app_machining})
oap_enable_utf8(oap_app_machining)
oap_enable_core_guideline_warnings(oap_app_machining)
oap_group_sources(oap_app_machining)

add_library(oap_app_orchestration STATIC ${OAP_APP_ORCHESTRATION_SOURCES})
add_library(OAP::AppOrchestration ALIAS oap_app_orchestration)
target_link_libraries(oap_app_orchestration
    PUBLIC Qt6::Core ${OAP_INTERNAL_DEPS_oap_app_orchestration})
oap_enable_utf8(oap_app_orchestration)
oap_enable_core_guideline_warnings(oap_app_orchestration)
oap_group_sources(oap_app_orchestration)

add_library(oap_app_panels STATIC ${OAP_APP_PANEL_SOURCES})
add_library(OAP::AppPanels ALIAS oap_app_panels)
set_target_properties(oap_app_panels PROPERTIES AUTOMOC ON)
target_link_libraries(oap_app_panels
    PUBLIC Qt6::Widgets ${OAP_INTERNAL_DEPS_oap_app_panels})
oap_enable_utf8(oap_app_panels)
oap_enable_core_guideline_warnings(oap_app_panels)
oap_group_sources(oap_app_panels)

function(oap_add_ui_slice target alias_name source_variable)
    add_library(${target} STATIC ${${source_variable}})
    add_library(OAP::${alias_name} ALIAS ${target})
    oap_configure_occt_viewer_target(${target} PUBLIC)
    target_link_libraries(${target}
        PUBLIC Qt6::Widgets ${OAP_INTERNAL_DEPS_${target}})
    set_target_properties(${target} PROPERTIES AUTOMOC ON)
    oap_enable_utf8(${target})
    oap_enable_core_guideline_warnings(${target})
    oap_group_sources(${target})
endfunction()

oap_add_ui_slice(oap_app_recognition_ui AppRecognitionUi
    OAP_APP_RECOGNITION_UI_SOURCES)
oap_add_ui_slice(oap_app_document_ui AppDocumentUi
    OAP_APP_DOCUMENT_UI_SOURCES)
oap_add_ui_slice(oap_app_cam_ui AppCamUi
    OAP_APP_CAM_UI_SOURCES)
oap_add_ui_slice(oap_app_simulation_ui AppSimulationUi
    OAP_APP_SIMULATION_UI_SOURCES)

# ---------------------------------------------------------------------------
# 桌面应用程序
# ---------------------------------------------------------------------------

# QRhi stays private to simulation presentation; geometry/CAM remains API-neutral.
function(oap_configure_cam_rhi target)
    target_link_libraries(${target} PRIVATE Qt6::GuiPrivate)
    target_include_directories(${target} SYSTEM PRIVATE "${OAP_VULKAN_HEADERS_DIR}")
    qt_add_shaders(${target} cam_scene_shaders
        PREFIX "/cam"
        BASE "${PROJECT_SOURCE_DIR}/resources/shaders/cam"
        FILES resources/shaders/cam/scene.vert resources/shaders/cam/scene.frag
              resources/shaders/cam/post.vert resources/shaders/cam/post.frag
              resources/shaders/cam/dexel_cut.comp resources/shaders/cam/dexel_surface.comp
              resources/shaders/cam/dexel.vert resources/shaders/cam/dexel.frag
        NOGLSL NOHLSL NOMSL)
endfunction()

add_library(oap_app_cam_renderer STATIC ${OAP_APP_CAM_RENDERER_SOURCES})
add_library(OAP::AppCamRenderer ALIAS oap_app_cam_renderer)
target_link_libraries(oap_app_cam_renderer PUBLIC Qt6::Widgets Qt6::OpenGLWidgets ${OAP_INTERNAL_DEPS_oap_app_cam_renderer})
set_target_properties(oap_app_cam_renderer PROPERTIES AUTOMOC ON)
oap_enable_utf8(oap_app_cam_renderer)
oap_enable_core_guideline_warnings(oap_app_cam_renderer)
oap_configure_cam_rhi(oap_app_cam_renderer)

add_executable(OpenAutoPathApp WIN32 ${OAP_APP_SOURCES})
set_target_properties(OpenAutoPathApp PROPERTIES AUTOMOC ON)
qt_add_resources(OpenAutoPathApp "app_translations"
    PREFIX "/i18n"
    BASE "resources/i18n"
    FILES resources/i18n/en.json
          resources/i18n/diagnostics_zh_CN.json
          resources/i18n/logs_zh_CN.json
          resources/i18n/labels.json
          resources/i18n/qt/qtbase_zh_CN.ts)
target_link_libraries(OpenAutoPathApp PRIVATE
    Qt6::Widgets
    Qt6::Gui
    Qt6::Core
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    ${OAP_INTERNAL_DEPS_OpenAutoPathApp})
oap_configure_occt_viewer_target(OpenAutoPathApp PRIVATE)
oap_enable_utf8(OpenAutoPathApp)
oap_enable_core_guideline_warnings(OpenAutoPathApp)
oap_group_sources(OpenAutoPathApp)
endif()

foreach(first_party_target IN LISTS OAP_FIRST_PARTY_TARGETS)
    if(TARGET ${first_party_target})
        oap_configure_first_party_includes(${first_party_target})
        if(first_party_target IN_LIST OAP_FIRST_PARTY_LIBRARY_TARGETS)
            oap_require_cxx20(${first_party_target} PUBLIC)
        else()
            oap_require_cxx20(${first_party_target} PRIVATE)
        endif()
    endif()
endforeach()

if(OAP_BUILD_GUI AND WIN32)
    # Windows 在进入 main() 前就会解析导入 DLL，根目录 EXE 无法自行把 bin/
    # 加入搜索路径。因此用一个无第三方依赖的小启动器作为用户入口，实际 UI
    # 与 Qt/OCCT 运行库继续整齐地留在 bin/。
    add_executable(OpenAutoPath WIN32
        src/launcher/OpenAutoPathLauncher.cpp)
    oap_require_cxx20(OpenAutoPath PRIVATE)
    set_target_properties(OpenAutoPath PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${OAP_OUTPUT_ROOT}"
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
        VS_DEBUGGER_COMMAND "$<TARGET_FILE:OpenAutoPathApp>"
        VS_DEBUGGER_WORKING_DIRECTORY "${OAP_OUTPUT_ROOT}"
        VS_DEBUGGER_ENVIRONMENT
            "OAP_APP_ROOT=${OAP_OUTPUT_ROOT}\nPATH=$<TARGET_FILE_DIR:OpenAutoPathApp>;%PATH%\nQT_PLUGIN_PATH=$<TARGET_FILE_DIR:OpenAutoPathApp>")
    add_dependencies(OpenAutoPath OpenAutoPathApp)
    oap_enable_utf8(OpenAutoPath)
    oap_enable_core_guideline_warnings(OpenAutoPath)
    oap_group_sources(OpenAutoPath)
endif()

include(cmake/OAPRuntimePaths.cmake)
if(WIN32 AND OAP_BUILD_GUI AND OAP_DEPLOY_RUNTIME)
    include(cmake/OAPRuntime.cmake)
endif()

if(BUILD_TESTING)
    include(cmake/OAPTests.cmake)
endif()

include(cmake/OAPInstall.cmake)
if(BUILD_TESTING AND OAP_INSTALL_DEVELOPMENT)
    include(cmake/tests/OAPInstallTests.cmake)
endif()

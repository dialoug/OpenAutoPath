# 公共目标配置。这些辅助函数用于统一 GUI、可复用库和回归测试的
# Windows/OCCT 编译选项及 IDE 目录布局。

function(oap_configure_occt_target_with_libraries target visibility)
    foreach(toolkit IN LISTS ARGN)
        target_link_libraries(${target} ${visibility} OCCT::${toolkit})
    endforeach()
    if(WIN32)
        target_compile_definitions(${target} ${visibility} WNT _CRT_SECURE_NO_WARNINGS)
    endif()
endfunction()

# Repository-relative first-party includes are available while compiling each
# target, but are not propagated through a base module as an interface detail.
function(oap_configure_first_party_includes target)
    target_include_directories(${target} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
endfunction()

# Language requirements belong to targets, independently of SDK installation.
# Libraries publish the requirement so consumers also compile as C++20.
function(oap_require_cxx20 target visibility)
    target_compile_features(${target} ${visibility} cxx_std_20)
    set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
endfunction()

# 无界面算法与命令行工具只链接 OCCT 核心组件。
function(oap_configure_occt_core_target target visibility)
    oap_configure_occt_target_with_libraries(
        ${target} ${visibility} ${OAP_OCCT_CORE_LIBRARIES})
endfunction()

# 视图目标在核心组件基础上追加 OpenGL/交互式查看器组件。
function(oap_configure_occt_viewer_target target visibility)
    oap_configure_occt_target_with_libraries(
        ${target} ${visibility}
        ${OAP_OCCT_CORE_LIBRARIES} ${OAP_OCCT_VIEWER_LIBRARIES})
endfunction()

function(oap_enable_utf8 target)
    if(MSVC)
        # Visual Studio 的 MultiToolTask 用根配置中的共享进程预算约束并行度。
        # /FS 会串行写入目标 PDB；否则 Visual Studio 生成器在并行编译
        # 静态库时可能报 C1041 错误。
        target_compile_options(${target} PRIVATE /utf-8 /MP /FS)
    endif()
endfunction()

# 将可由编译器持续验证的 C++ Core Guidelines 要求固化到所有项目目标。
# 第三方头文件按外部头处理，避免其诊断掩盖项目自身的问题。
function(oap_enable_core_guideline_warnings target)
    if(NOT OAP_ENABLE_STRICT_WARNINGS)
        return()
    endif()

    if(MSVC)
        target_compile_options(${target} PRIVATE
            /W4
            /permissive-
            /Zc:__cplusplus
            /external:anglebrackets
            /external:W0)
    else()
        target_compile_options(${target} PRIVATE
            -Wall
            -Wextra
            -Wpedantic)
    endif()
endfunction()

function(oap_group_sources target)
    get_target_property(target_sources ${target} SOURCES)
    if(NOT target_sources)
        return()
    endif()
    set(project_sources)
    set(external_sources)
    foreach(source IN LISTS target_sources)
        # Generated sources may be represented by generator expressions.
        if(source MATCHES "^\\$<")
            continue()
        endif()
        get_filename_component(source_absolute "${source}" ABSOLUTE
            BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
        file(RELATIVE_PATH source_relative "${CMAKE_CURRENT_SOURCE_DIR}" "${source_absolute}")
        if(IS_ABSOLUTE "${source_relative}" OR source_relative MATCHES "^\\.\\.(/|$)")
            list(APPEND external_sources "${source_absolute}")
        else()
            list(APPEND project_sources "${source_absolute}")
        endif()
    endforeach()
    if(project_sources)
        source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${project_sources})
    endif()
    if(external_sources)
        source_group("External" FILES ${external_sources})
    endif()
endfunction()

function(oap_set_runtime_subdirectory target subdirectory)
    set_target_properties(${target} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY
            "${OAP_OUTPUT_ROOT}/${subdirectory}")
endfunction()

function(oap_place_test target)
    oap_set_runtime_subdirectory(${target} "tests")
endfunction()

# 测试 EXE 位于 tests/，运行时从已配置的 SDK 目录加载 Qt/OCCT DLL；测试目标
# 不依赖桌面应用，因此只构建某个快速测试时不会触发 GUI 链接和部署。
function(oap_add_test_with_runtime test_name target)
    if(WIN32)
        add_test(NAME ${test_name}
            COMMAND "${CMAKE_COMMAND}" -E env
                "PATH=${OAP_TEST_RUNTIME_PATH}"
                "$<TARGET_FILE:${target}>" ${ARGN})
    else()
        add_test(NAME ${test_name}
            COMMAND "$<TARGET_FILE:${target}>" ${ARGN})
    endif()
endfunction()

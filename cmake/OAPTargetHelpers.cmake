# 公共目标配置。这些辅助函数用于统一 GUI、可复用库和回归测试的
# Windows/OCCT 编译选项及 IDE 目录布局。

function(oap_configure_occt_target_with_libraries target visibility)
    target_include_directories(${target} ${visibility}
        "${OAP_OCCT_INCLUDE_DIR}")
    target_link_directories(${target} ${visibility} "${OAP_OCCT_LIBRARY_DIR}")
    target_link_libraries(${target} ${visibility} ${ARGN})
    target_compile_definitions(${target} ${visibility} WNT _CRT_SECURE_NO_WARNINGS)
endfunction()

# Repository-relative first-party includes are available while compiling each
# target, but are not propagated through a base module as an interface detail.
function(oap_configure_first_party_includes target)
    target_include_directories(${target} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
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
        # /MP 让同一目标内的多个翻译单元并行编译；配合 CMake Tools 的
        # cmake.parallelJobs，可同时利用项目级和目标内并行。
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
    if(target_sources)
        source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${target_sources})
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

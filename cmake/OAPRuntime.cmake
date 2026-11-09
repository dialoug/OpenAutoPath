# Platform-specific desktop deployment.
if(WIN32)
    include("${CMAKE_CURRENT_LIST_DIR}/platforms/OAPWindowsRuntime.cmake")
endif()

if(NOT MSVC OR NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "The Windows build requires x64 MSVC and matching dependencies")
endif()

if(CMAKE_GENERATOR MATCHES "Visual Studio")
    include(ProcessorCount)
    ProcessorCount(oap_detected_jobs)
    if(NOT oap_detected_jobs)
        set(oap_detected_jobs 2)
    endif()
    set(OAP_BUILD_PARALLEL_JOBS "${oap_detected_jobs}" CACHE STRING
        "Maximum concurrent MSVC compiler processes across projects")
    if(NOT OAP_BUILD_PARALLEL_JOBS MATCHES "^[1-9][0-9]*$")
        message(FATAL_ERROR "OAP_BUILD_PARALLEL_JOBS must be a positive integer")
    endif()
    list(APPEND CMAKE_VS_GLOBALS
        "UseMultiToolTask=true"
        "EnforceProcessCountAcrossBuilds=true"
        "MultiProcMaxCount=${OAP_BUILD_PARALLEL_JOBS}")
endif()

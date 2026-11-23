# Source ownership shared by normal configuration and script-mode checks.
include_guard(GLOBAL)

# Store the variable name, not a copy: module manifests may filter/split lists
# after declaring them.
macro(oap_module_sources target source_variable)
    if(DEFINED OAP_SOURCE_VARIABLE_${target})
        message(FATAL_ERROR "Duplicate source declaration for ${target}")
    endif()
    list(APPEND OAP_SOURCE_TARGETS "${target}")
    set(OAP_SOURCE_VARIABLE_${target} "${source_variable}")
    set(${source_variable} ${ARGN})
endmacro()
include(${CMAKE_CURRENT_LIST_DIR}/modules/OAPAnalysisSources.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/modules/OAPRecognitionSources.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/modules/OAPToolpathSources.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/modules/OAPToolingSources.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/modules/OAPManufacturingSources.cmake)
oap_module_sources(oap_support OAP_SUPPORT_SOURCES
    src/support/Files.cpp src/support/Files.h src/support/Identity.cpp src/support/Identity.h
    src/support/Text.h src/support/Json.h src/support/Utf8Path.h)
oap_module_sources(oap-worker OAP_WORKER_SOURCES
    src/worker/main.cpp src/worker/Project.cpp src/worker/Project.h
    src/worker/NcReview.cpp src/worker/NcReview.h
    src/worker/Planning.cpp src/worker/Planning.h
    src/worker/Simulation.cpp src/worker/Simulation.h)

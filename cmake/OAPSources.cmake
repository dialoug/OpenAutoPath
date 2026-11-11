# Source ownership shared by normal configuration and script-mode checks.
include_guard(GLOBAL)

# Store the variable name, not a copy: module manifests may filter/split lists
# after declaring them (notably the desktop workflow and renderer sources).
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
include(${CMAKE_CURRENT_LIST_DIR}/modules/OAPAppSources.cmake)

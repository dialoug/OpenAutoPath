# First-party module dependency graph.
#
# These lists are the single source of truth for both target_link_libraries()
# and CheckModuleBoundaries.cmake. Every source-level cross-target include must
# name a target in the owner's direct dependency list; relying only on another
# target's PUBLIC/transitive dependencies is intentionally forbidden.

include(${CMAKE_CURRENT_LIST_DIR}/OAPSources.cmake)
set(OAP_FIRST_PARTY_TARGETS ${OAP_SOURCE_TARGETS})
set(OAP_FIRST_PARTY_LIBRARY_TARGETS ${OAP_FIRST_PARTY_TARGETS})
list(REMOVE_ITEM OAP_FIRST_PARTY_LIBRARY_TARGETS oap-worker)
set(OAP_INTERNAL_DEPS_oap-worker
    oap_support oap_analysis oap_recognition oap_toollibrary oap_toolpath_adapter oap_toolpath_core
    oap_toolpath_hole oap_toolpath_pocket oap_toolpath_profile oap_toolpath_finishing
    oap_toolpath_workflow oap_toolpath_program oap_manufacturing)

set(OAP_INTERNAL_DEPS_oap_support)
set(OAP_INTERNAL_DEPS_oap_analysis oap_support)
set(OAP_INTERNAL_DEPS_oap_recognition
    oap_analysis)
set(OAP_INTERNAL_DEPS_oap_toolpath_core
    oap_analysis)
set(OAP_INTERNAL_DEPS_oap_toolpath_program
    oap_analysis
    oap_toolpath_core)
set(OAP_INTERNAL_DEPS_oap_toolpath_hole
    oap_analysis
    oap_toolpath_core)
set(OAP_INTERNAL_DEPS_oap_toolpath_pocket
    oap_analysis
    oap_toolpath_core
    oap_toolpath_program)
set(OAP_INTERNAL_DEPS_oap_toolpath_profile
    oap_analysis
    oap_toolpath_core
    oap_toolpath_pocket
    oap_toolpath_program)
set(OAP_INTERNAL_DEPS_oap_toolpath_algorithms
    oap_toolpath_hole
    oap_toolpath_pocket
    oap_toolpath_profile)
set(OAP_INTERNAL_DEPS_oap_toolpath_finishing
    oap_toolpath_core)
set(OAP_INTERNAL_DEPS_oap_toolpath_workflow
    oap_toolpath_algorithms
    oap_toolpath_core
    oap_toolpath_hole
    oap_toolpath_pocket
    oap_toolpath_profile)
set(OAP_INTERNAL_DEPS_oap_toollibrary oap_support)
set(OAP_INTERNAL_DEPS_oap_toolpath_adapter
    oap_support
    oap_analysis
    oap_toollibrary
    oap_toolpath_core
    oap_toolpath_hole
    oap_toolpath_pocket
    oap_toolpath_profile
    oap_toolpath_workflow)
set(OAP_INTERNAL_DEPS_oap_manufacturing
    oap_support
    oap_toolpath_finishing
    oap_analysis
    oap_toollibrary
    oap_toolpath_adapter
    oap_toolpath_core
    oap_toolpath_hole
    oap_toolpath_pocket
    oap_toolpath_profile
    oap_toolpath_program
    oap_toolpath_workflow)

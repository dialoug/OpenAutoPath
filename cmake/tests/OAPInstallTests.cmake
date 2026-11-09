# Exercise the installed interface from an independent CMake project.
configure_file("${CMAKE_CURRENT_LIST_DIR}/CheckInstalledPackage.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/CheckInstalledPackage.cmake.in" @ONLY)
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/CheckInstalledPackage-$<CONFIG>.cmake"
    INPUT "${CMAKE_CURRENT_BINARY_DIR}/CheckInstalledPackage.cmake.in")
add_test(NAME oap_installed_package COMMAND "${CMAKE_COMMAND}" -P
    "${CMAKE_CURRENT_BINARY_DIR}/CheckInstalledPackage-$<CONFIG>.cmake")
set_tests_properties(oap_installed_package PROPERTIES LABELS "packaging" TIMEOUT 600)

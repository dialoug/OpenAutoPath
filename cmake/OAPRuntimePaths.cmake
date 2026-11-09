# Runtime paths shared by tests and optional desktop deployment.
# Only deployment requires the complete DLL bundle at configure time.
get_target_property(OAP_QMAKE_EXECUTABLE Qt6::qmake IMPORTED_LOCATION)
get_filename_component(OAP_QT_BIN_DIR "${OAP_QMAKE_EXECUTABLE}" DIRECTORY)
get_filename_component(oap_qt_prefix "${OAP_QT_BIN_DIR}" DIRECTORY)
set(OAP_QT_PLUGIN_DIR "${oap_qt_prefix}/plugins")

if(WIN32)
    include("${CMAKE_CURRENT_LIST_DIR}/platforms/OAPWindowsRuntimePaths.cmake")
endif()

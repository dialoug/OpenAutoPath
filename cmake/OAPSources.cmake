# Central source manifest. Module-local manifests retain the authoritative target ownership.
include(${CMAKE_CURRENT_LIST_DIR}/modules/OAPAnalysisSources.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/modules/OAPRecognitionSources.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/modules/OAPToolpathSources.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/modules/OAPToolingSources.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/modules/OAPManufacturingSources.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/modules/OAPAppSources.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/tests/OAPTestSources.cmake)

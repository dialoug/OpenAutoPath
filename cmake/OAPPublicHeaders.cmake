# Explicit cross-target API manifest, split by owning module.
set(OAP_PUBLIC_TARGET_HEADERS)
include(${CMAKE_CURRENT_LIST_DIR}/public/OAPAnalysisPublicHeaders.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/public/OAPRecognitionPublicHeaders.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/public/OAPToolingPublicHeaders.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/public/OAPToolpathPublicHeaders.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/public/OAPManufacturingPublicHeaders.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/public/OAPAppPublicHeaders.cmake)

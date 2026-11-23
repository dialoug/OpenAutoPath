# Explicit cross-target API manifest, split by owning module.
set(OAP_PUBLIC_TARGET_HEADERS)
include(${CMAKE_CURRENT_LIST_DIR}/public/OAPAnalysisPublicHeaders.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/public/OAPRecognitionPublicHeaders.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/public/OAPToolingPublicHeaders.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/public/OAPToolpathPublicHeaders.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/public/OAPManufacturingPublicHeaders.cmake)
list(APPEND OAP_PUBLIC_TARGET_HEADERS
    src/support/Files.h src/support/Identity.h src/support/Json.h src/support/Text.h src/support/Utf8Path.h)

# Core SDK APIs without an internal cross-target caller.
set(OAP_SDK_ONLY_HEADERS
    src/recognition/DihedralAngle.h
    src/tooling/library/ToolAssetCatalog.h
    src/toolpath/hole/HoleMoldEnvelopeEntryResolver.h
    src/toolpath/hole/HoleStageWallHighlightResolver.h
    src/toolpath/pocket/simulation/DexelDisplayMesh.h
    src/toolpath/workflow/hole/HoleSimulationAdapter.h
    src/toolpath/workflow/ToolpathProgramAdapters.h
    src/toolpath/workflow/ToolpathAsyncExecutor.h)

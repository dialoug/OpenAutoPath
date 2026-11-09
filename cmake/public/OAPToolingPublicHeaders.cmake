# Public headers owned by the tooling module.
list(APPEND OAP_PUBLIC_TARGET_HEADERS
    src/tooling/library/ToolAssetCatalog.h
    src/tooling/library/ToolLibrary.h
    src/tooling/library/ToolLibraryStorage.h
    src/tooling/adapter/AssignedToolContext.h
    src/tooling/adapter/hole/HoleStageIdentity.h
    src/tooling/adapter/hole/HoleAutomaticToolpath.h
    src/tooling/adapter/hole/HoleMachiningInputMapper.h
    src/tooling/adapter/hole/HoleToolSelectionPlanner.h
    src/tooling/adapter/pocket/PocketAutomaticToolpath.h
    src/tooling/adapter/profile/ProfileAutomaticToolpath.h
)

# Exact test/header pairs for storage ownership tests. This does not expose the
# header as public API or permit other tests to include module internals.
set(OAP_TEST_PRIVATE_HEADER_ACCESS
    "tests/smoke/app/language_smoke.cpp|app/ui/LanguageSettings.h"
    "tests/smoke/app/cam_renderer_smoke.cpp|app/ui/dialogs/detail/PocketCamVulkanWidget.h"
    "tests/smoke/app/cam_renderer_smoke.cpp|app/ui/dialogs/detail/PocketCamOpenGLWidget.h"
    "tests/smoke/app/cam_renderer_smoke.cpp|app/ui/dialogs/detail/PocketCamDialog.h"
    "tests/smoke/toolpath/dexel_interval_smoke.cpp|toolpath/pocket/simulation/detail/DexelCellIntervals.h")

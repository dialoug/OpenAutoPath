# OpenAutoPath architecture

OpenAutoPath is a modular C++20 desktop application. Production code is
compiled into first-party static libraries and assembled by
`OpenAutoPathApp`; the libraries are internal modules, not independently
versioned or ABI-stable packages.

The application name is `OpenAutoPath`, the organization identifier is `OAP`,
and first-party C++ namespaces, build targets and environment options use
`oap`, `oap_` and `OAP_` respectively. The launcher is `OpenAutoPath.exe`,
which starts `bin/OpenAutoPathApp.exe`. User settings and the tool catalog use
the new OAP/OpenAutoPath identity; previous application settings are not
automatically imported. The embedded synthetic simulation library is added
at startup if its library ID is missing.

New project and tool-library files use `.oapjob` and `.oaptl`, with `oap-`
format identifiers. Explicitly opening older `.ffjob` and `.fftl` files remains
supported; saving writes the new format identifier. Legacy identifiers remain
only in the compatibility readers, import filters and regression fixtures.

## Source layout

The physical source tree follows responsibility boundaries while CMake target
lists remain the authoritative ownership map:

| Directory | Contents |
|---|---|
| `src/analysis` | reusable model and geometry analysis |
| `src/recognition` | feature recognition and shared geometry caches; see [recognition notes](src/recognition/README.md) |
| `src/tooling/library` | tool data, import and persistence |
| `src/tooling/adapter` | hole/pocket/profile process planning and CAM input mapping |
| `src/toolpath/core` | shared moves, parameters, validation and stock simulation primitives |
| `src/toolpath/program` | program validation, review, post-processing and export |
| `src/toolpath/hole` | hole machining kernels |
| `src/toolpath/pocket/geometry` | pocket topology conversion and machining-region construction |
| `src/toolpath/pocket/planning` | pocket layer, entry, linking and cutting-pattern planning |
| `src/toolpath/pocket/simulation` | dexel, Z-map and fixed-axis stock simulation |
| `src/toolpath/pocket/program` / `validation` | program adaptation/export and safety validation |
| `src/toolpath/profile` | fixed-axis external clearing, contour finishing, validation and program adaptation |
| `src/toolpath/finishing` | OpenCAMLib raster relief engraving, private mesh conversion and neutral program adaptation |
| `src/toolpath/algorithms` / `src/toolpath/workflow` | replaceable algorithms and UI-independent use cases |
| `src/manufacturing` | machining Job, rebuild and IPW lifecycle services |
| `src/app/workflows/hole` / `pocket` / `profile` | feature-specific desktop CAM workflows |
| `src/app/workflows/simulation` / `playback` | stock simulation and playback workflows |
| `src/app/workflows/project` / `shared` | project state and shared application workflows |
| `src/app/dialogs` | reusable Qt dialog adapters |
| `src/app/panels` | reusable analysis and recognition result panels |
| `src/app/view` | window-independent display geometry |
| `src/app/presentation` | OCCT/AIS presentation objects and overlay ownership |
| `src/app/<workspace>` | focused document, scene, playback, recognition, machining and simulation coordinators |
| `src/app/ui/shell` | thin main-window composition and residual event adapters |
| `src/app/ui/{recognition,document,cam,simulation,playback}` | independently buildable UI slices and callback ports |
| `src/app/ui/dialogs` / `panels` / `view` | concrete UI components and OCCT view widget |
| `tests/smoke/<module>` | deterministic smoke tests grouped by owning module |
| `cmake/modules` / `public` / `tests` | module sources, public API declarations and test manifests |
| `external` | ignored local third-party SDKs and source dependencies |
| `local-data` / `artifacts` | ignored local inputs, reference material and generated output |

Application workflow, dialog, panel and display files must not be placed in a
shared `src/app/toolpath` catch-all directory. New files should use the focused
directory matching their responsibility and be registered in the matching
manifest under `cmake/modules`. Headers used only inside one target belong in a
`detail/` directory and must not be listed as public API.

中文说明：模块源码通过 `oap_module_sources(目标名 源码变量 ...)` 声明，
构建和架构检查共同使用这份归属清单；无需再在边界检查脚本中重复登记目标。
直接依赖仍在 `cmake/OAPModuleDependencies.cmake` 中显式维护，公共头文件仍由
`cmake/public` 控制。C++20 要求由目标声明，与是否安装开发 SDK 无关。

测试配置集中在 `cmake/tests/OAPCoreTests.cmake` 和
`cmake/tests/OAPAppTests.cmake`：一个 `oap_add_smoke_test` 同时声明源码、
链接库、标签、超时和环境变量。同一可执行文件的不同参数或线程预算用
`oap_add_smoke_test_case` 紧邻声明；架构检查和安装验证保留独立入口。
测试访问私有头文件的例外单独记录在 `OAPTestPrivateHeaders.cmake`，不扩大公共 API。

Development installation follows the core subset of this public API manifest.
`cmake/OAPSdkHeaders.cmake` maintains an explicit exception list for support
types embedded in public interfaces, without changing cross-target include
permissions. The installed header set is checked exactly, and each header is
compiled independently against the installed package. Unrelated implementation
headers and desktop headers are excluded from the core SDK.

Within toolpath workflows, private lifecycle helpers share parameter preparation
while feature services retain domain-specific failure and cancellation policy.
Profile planning separates cached geometry generation (`ProfileContour`) from
travel optimization, safe linking and move assembly (`detail/ProfileMotionPlanning`).
Both stages belong to the existing profile target; internal stage headers do not
expand its public API or change the module dependency graph.

Implementation fragments have been replaced by independently compiled sources.
Pocket CAM UI separates dialog composition, mesh conversion, camera interaction,
rendering, GPU programs/material/surface processing and playback. Hole Job services
separate capture, selection and regeneration from identity/order/IPW helpers.
Multi-Dexel and ZMap expose private stage contracts for rasterization, updates,
comparison and display; analytic BRep separates geometry, sweeps and booleans.
Shared implementation types belong in private headers, not textual source includes.

Native material updates consume `core/simulation/FixedAxisToolpathView`: a borrowed
operation ID, cutting axis and authoritative moves. Hole and Profile IPW updates
do not construct Pocket operations. The Profile module's simulation adapter returns
this neutral view. Legacy BRep and display consumers still use the explicitly named
`workflow/compatibility/LegacyProfileSimulationAdapter`; this bridge is owned by
the orchestration layer that can depend on both feature domains. No view is stored
in an asynchronous request or persisted checkpoint.

## Dependency layers

The dependency graph is acyclic and points from orchestration toward reusable
geometry and domain modules:

```text
OpenAutoPathApp
  -> app_analysis / app_panels / app_playback / app_recognition / app_simulation
  -> app_{recognition,document,cam,simulation,playback}_ui
  -> app_scene / app_document / app_machining / app_orchestration
  -> app_presentation / app_workflow_{base,playback,cam,simulation}
  -> manufacturing / tooling / toolpath / recognition

app_playback
  -> analysis / app_presentation / app_view / app_workflow_playback
  -> app_workflow_cam / hole toolpath / tool library

app_scene / app_document
  -> their focused app services / app_presentation / app_workflow_base

app_machining
  -> app_document / app_recognition / app_simulation
  -> app_workflow_cam / app_workflow_simulation / manufacturing / toolpath

app_simulation
  -> app_presentation / app_workflow_simulation / manufacturing

app_recognition
  -> analysis / app_analysis / app_presentation
  -> app_workflow_base / app_workflow_cam / tooling adapter

app_workflow_simulation
  -> app_workflow_cam / app_workflow_base / manufacturing / toolpath / app_view

app_workflow_cam
  -> app_workflow_playback / app_workflow_base
  -> manufacturing / tooling adapter / toolpath / recognition

app_workflow_playback
  -> app_workflow_base / recognition / toolpath

app_workflow_base
  -> analysis / app_view / recognition / toolpath

app_presentation
  -> recognition / analysis / app_view

manufacturing
  -> analysis / tool library / tooling adapter
  -> toolpath core / hole / pocket / profile / program / workflow

tooling adapter
  -> analysis / tool library / toolpath core
  -> hole / pocket / profile toolpath / toolpath workflow

toolpath workflow
  -> algorithms / toolpath core
  -> hole / pocket / profile toolpath

toolpath algorithms
  -> hole / pocket / profile toolpath

toolpath program
  -> toolpath core / analysis

recognition / toolpath core
  -> analysis
```

The exact direct edges live in
[`cmake/OAPModuleDependencies.cmake`](cmake/OAPModuleDependencies.cmake).
CMake target linking and the source boundary checker consume the same lists, so
a dependency cannot be made legal only in one of those places.

Face pairing, nearest-neighbor distance/interference analysis, mating-face
recognition and their JSONL result export have been removed. Model browsing,
geometric properties, feature recognition and CAM retain the shared topology
data. The binary Model cache uses format version 2; older cached models are
invalidated and rebuilt when loading STEP files.

## Module responsibilities

| Target | Responsibility |
|---|---|
| `oap_analysis` | STEP loading, model topology indexing, shared geometry and cache serialization |
| `oap_recognition` | feature and three-axis structure recognition |
| `oap_toolpath_core` | common moves, parameters, validation and stock primitives |
| `oap_toolpath_program` | program validation, review, post-processing and export |
| `oap_toolpath_hole` | fixed-axis hole drilling/milling kernels, structure validation and stock-simulation adapters |
| `oap_toolpath_pocket` | fixed-axis pocket geometry, clearing, entry/linking, safety validation and Multi-Dexel stock primitives |
| `oap_toolpath_profile` | fixed-axis external clearing, contour finishing, safety validation and program/simulation adapters |
| `oap_toolpath_algorithms` | replaceable algorithm implementations and registry |
| `oap_toolpath_workflow` | UI-independent toolpath use cases |
| `oap_toollibrary` | global ToolBit assets, materialized library references, import and versioned persistence without CAM dependencies |
| `oap_toolpath_adapter` | translation between tool-library decisions and CAM inputs |
| `oap_manufacturing` | persistent machining Job, rebuild services, formal project-program assembly, conservative output partitioning and IPW lifecycle |
| `oap_app_view` | viewer-independent display geometry |
| `oap_app_presentation` | OCCT/AIS scene, overlay ownership and presentation objects |
| `oap_app_playback` | playback plus full/current toolpath and hole-stage presentation lifecycle, geometry policy and visibility coordination |
| `oap_app_playback_ui` | playback dock ownership, signal routing, frame presentation and tool filtering |
| `oap_app_recognition_ui` | model/recognition/selection dock family and panel callback boundary |
| `oap_app_document_ui` | file-navigation dock ownership and activation boundary |
| `oap_app_cam_ui` | CAM workflow dock, operation tree, coverage and command callback boundary |
| `oap_app_simulation_ui` | simulation-result selection to presentation binding |
| `oap_app_scene` | OCCT pick mapping, selection-source policy and visibility/undo transactions |
| `oap_app_document` | active model document, analysis cache and load-task lifecycle |
| `oap_app_machining` | facade owning machining runtime, configuration and tool-library state and their cross-state transactions |
| `oap_app_simulation` | stock/IPW session, async display derivation and OCCT presentation coordination |
| `oap_app_recognition` | recognition session/task lifecycle, atomic result installation, derived colors/boundaries and inspection Overlay coordination |
| `oap_app_analysis` | Qt task lifecycle for analysis and recognition |
| `oap_app_panels` | result panels that consume domain results |
| `oap_app_workflow_base` | shared document, recognition, selection and interaction contracts |
| `oap_app_workflow_playback` | reusable playback session, assembly, controller and worker |
| `oap_app_workflow_cam` | hole, pocket, profile, project and tool-library application workflows, including stock-definition geometry reconstruction |
| `oap_app_workflow_simulation` | stock/IPW workflow, execution and display-mesh derivation |
| `oap_app_orchestration` | cross-workspace desktop transactions and lifecycle coordination |
| `OpenAutoPathApp` | UI composition root; wires actions, dialogs, panels and controllers |

## Desktop state ownership

`MainWindow` is an adapter/composition root, not the owner of long-lived
model, CAM or OCCT overlay state. The desktop layer assigns that state as
follows:

STEP loading computes exact face bounds through an injected, synchronous
`StepIndexBuildControl` executor. The application adapter uses at most four
workers and borrows additional slots from the existing process budget; insufficient capacity falls
back to serial execution. Complex faces use isolated geometry copies, indexed
result slots, and serial publication of stable topology IDs. `oap_analysis`
does not depend on the CAM scheduler. Cancellation or executor failure never
publishes a partial `Model` or topology index.

For complex, uncached models, the loader publishes a separately copied and
meshed preview before exact indexing finishes. `StepLoadPreviewPresenter` owns
only this temporary presentation and restores previous visibility/camera on
failure or cancellation. Preview geometry never becomes authoritative model,
stock or machining input. The document workspace gates preview/progress by
request generation and source path, and atomically installs only completed
analysis. Cached loads skip both preview preparation and exact indexing.
Logs distinguish cache lookup, transfer, preview preparation, exact bounds,
cache persistence, preview visibility, exact model visibility and machining
readiness. Display preparation and indexing never mutate the same geometry.

| Owner | State / policy |
|---|---|
| `ModelDocumentWorkspaceCoordinator` | owns the active document Session, STEP analysis cache/Controller, pending path and load generation; publishes read-only model values and explicit setup-frame installs |
| `RecognitionSession` | recognition result, derived feature sets and reusable recognition caches |
| `RecognitionWorkspaceCoordinator` | owns recognition Session, background Controller, color/boundary Coordinator and inspection Presenter; atomically installs completed recognition results |
| `MachiningWorkspaceCoordinator` | owns machining Session, configuration and tool libraries; publishes model reset, hole/pocket/profile install, project restore and invalidation as cross-state transactions |
| `MachiningWorkspaceSession` | facade-internal persistent machining Job plus editable hole/pocket/profile preview state |
| `MachiningConfigurationSession` | facade-internal hole/pocket/profile parameters, machining coordinates and program options |
| `MachiningProjectController` | `.oapjob` initialization, capture, validation, synchronization and regeneration |
| `MachiningProjectInstaller` | stages regenerated project values for atomic publication by workspace facades |
| `StockSimulationWorkspaceCoordinator` | owns stock/IPW Session, task/display Coordinator and Presenter; publishes the ordered Job IPW display state atomically |
| `StockSimulationSession` | workspace-internal unified IPW results, comparison data and stock-source configuration |
| `StockSimulationCoordinator` | workspace-internal task cancellation, display-worker/cache ownership, stale-result gates and publication policy |
| `StockSimulationWorkflowController` | stock-source validation, Hole/Pocket/Profile result installation, shared operation selection and simulation reporting |
| `StockSimulationDialogs` | value-driven stock/IPW settings and timeline dialogs without `MainWindow`, Job or OCCT access |
| stock-simulation use cases | stock loading/fingerprints, simulation task lifecycle, failure gates, result installation and OpenGL launch values |
| `Hole/Pocket/ProfileToolpathWorkflowController` | geometry preparation and atomic generated-program commit |
| `Hole/Pocket/ProfileToolpathGenerationUseCase` | generation orchestration, validation/statistics, diagnostic value results, preview publishing and commit gates |
| `PlaybackProgramAssembler` | `ToolpathProgram` flattening, semantic move colors, preview decimation and stable setup/tool/phase batches |
| `PlaybackSession` | flattened moves, tool batches, request coalescing, filtering and throttling |
| `PlaybackWorkspaceCoordinator` | owns playback/full/current/highlight Presenters; coordinates program requests, preview geometry, tool filtering, display layers and static/remaining-path invariants |
| `PlaybackPanelViewModelBuilder` | pure projection of playback position into step, phase, setup, tool and navigation panel values |
| `PlaybackCameraFollower` | null-safe camera recentering that preserves the eye/at offset |
| `HoleToolpathWorkbenchCoordinator` | hole-stage selection/tree values, manual-plan definition sync, atomic generated-program installation and undo restoration |
| `CamWorkflowPanel` | FreeCAD-style project→setup→tooling→operations→verification→output UI projection; embeds the existing operation tree, coverage audit and feature-allowance views without owning CAM state |
| `ToolLibraryController` | machining-facade-internal global ToolBit catalog loading, legacy-ID migration, active-library selection and v3 persistence |
| `LibraryToolSlot` / `AssignedToolContext` | separates reusable T-slot cutting defaults from immutable CAM/Job tool snapshots shared by the feature-planning workflows |
| `ToolLibraryDialog` / `ToolLibrarySelectionDialog` | FreeCAD-inspired global asset manager plus independent CAM library selector; edits canonical ToolBits, attaches/detaches library references, transfers slots, and previews cutter-holder-spindle envelopes while retaining this project's automatic-planning model |
| `SceneInteractionCoordinator` | owns selection-source policy and SceneController; maps picks and performs visibility/history transactions |
| `SceneController` | model-face AIS ownership, display modes and visibility state |
| presentation presenters | semantic OCCT objects for playback, toolpaths, inspection, hole-stage highlights and stock/IPW |
| `IpwVisualAcceptanceCapture` | optional OCCT final/ROI evidence capture with camera and displayed-object restoration |
| `RecognitionPresentationCoordinator` | derived recognition colors and feature boundaries |
| `OverlayController` | replacement/removal and visibility of semantic OCCT overlay groups |
| `ToolpathExportController` | active-program assembly and export validation |
| `CommandHistory` | time-ordered visibility and manual-hole undo records |

Model switches reset the model-scoped sessions through coordinated session
entry points. Presenters own their displayed AIS handles and clear semantic
groups through
`OverlayController`, so a deleted or regenerated operation cannot survive as
an untracked object in the OCCT context. Stock-simulation dialogs exchange value
snapshots and callbacks; `StockSimulationWorkspaceCoordinator` owns the Session,
worker/cache and Presenter, and rejects stale display results before the window
can publish UI feedback. `PlaybackWorkspaceCoordinator` likewise owns every
toolpath presentation group: full preview, current-tool preview, remaining path,
trail, cursor and hole-stage highlight. It builds preview/highlight geometry and
publishes filter or playback visibility as one transaction, so the window cannot
leave two competing static-path states. `MainWindow` consumes
`RecognitionWorkspaceCoordinator` query values and commands; cancelled or failed
recognition generations never replace the last valid Session, while model resets
cancel the task and clear recognition/inspection presentation through one entry
point. Hole-only recognition, pocket-region caching and loaded-project hole-source
synchronization also publish through this facade instead of exposing its Session. The
document, playback and stock workspaces likewise expose no Session accessor. Setup-frame,
stock-definition and loaded-hole updates are explicit cross-workspace transactions. The
window retains widget/action pointers, top-level user notifications and the thin glue needed to
translate UI events into session/controller calls. Hole program replacement and
undo publish through one window adapter entry after the workbench transaction has
committed, so every related semantic Overlay is rebuilt from the same Job state.
`MainWindowUiBuilder` owns the chrome construction order. `MainWindowUiAssembly.cpp`
instantiates the focused UI controllers and supplies callback ports; the individual
slice targets own their concrete docks, panels and signal routing. `MainWindow*.cpp`
files retain only command adapters that still span multiple workspaces.

The current manufacturing entry points and persistence/runtime split are
documented in [`src/manufacturing/README.md`](src/manufacturing/README.md).
Toolpath target ownership, canonical development plans and shared release gates
are indexed in [`src/toolpath/README.md`](src/toolpath/README.md). Application
workflow source ownership and transaction rules are documented in
[`src/app/workflows/README.md`](src/app/workflows/README.md).
Current UI state rules and manual release checks are documented in
[`docs/16_ui_coordination_audit.md`](docs/16_ui_coordination_audit.md).

## Enforced boundary rules

1. Every production `.cpp` and `.h` belongs to exactly one target through
   `cmake/OAPSources.cmake`.
2. Cross-target includes must follow a direct edge in the dependency graph;
   transitive-link access is rejected.
3. Cross-target includes may use only headers in
   `cmake/OAPPublicHeaders.cmake`. Unlisted and `detail/` headers are
   private.
4. Repository source include paths are build-private and are not propagated by
   a base library's CMake interface.
5. Tests link production modules. They must not copy production implementation
   files into a test executable.
6. Domain, recognition, toolpath, tooling and manufacturing targets do not
   depend on desktop UI targets.
7. Cross-module paths stored in `std::string` are UTF-8. Code that enters
   `std::filesystem` must use `analysis/Utf8Path.h`; Qt passes paths to domain
   services with `QString::toUtf8()`, while Qt-native file APIs keep `QString`.

`cmake/CheckModuleBoundaries.cmake` enforces the source ownership, direct-edge
and public-header rules. Negative fixtures verify rejection of a forbidden
direction, a private header, a dependency that is available only transitively
and a dependency cycle.

The architecture-only GitHub Actions workflow runs this checker and the
documentation-link checker without downloading OCCT or compiling the desktop
application. It gives pull requests fast feedback on dependency and layout
regressions; native application builds remain a separate, dependency-heavy
verification step.

## Verification

### Simulation graphics backends

`OAP::AppCamRenderer` owns the independent simulation dialog and both graphics
backends. Its public entry is `showPocketCamSimulatorDialog`; the renderer classes,
camera controls and QRhi details remain private. Tests link this same production
module. No graphics API was added to geometry, toolpath or manufacturing code.

The simulation window's backend selector offers **Vulkan** and **OpenGL（兼容）**.
OpenGL remains the initial default; explicit selections are remembered through
`simulation/renderBackend` in QSettings. `OAP_CAM_RENDER_BACKEND=vulkan` or `opengl`
overrides the initial selection for a process. Switching rebuilds the independent
top-level window and returns playback to its initial, paused frame. A reported
Vulkan initialization, mesh or pipeline failure recreates the OpenGL window with a
diagnostic. The main OCCT viewport continues using its existing OpenGL driver.

| Capability | Vulkan | OpenGL |
| --- | --- | --- |
| Presentation | QRhiWidget with explicit Vulkan API; SPIR-V shaders | QOpenGLWidget |
| Material updates | Native Vulkan compute for fixed-axis display playback; authoritative CPU IPW replay for project playback | GPU compute; authoritative IPW replay for project playback |
| Play, pause, seek, reset, final frame | Publishes cutter, progress and material together, including partial original moves | Existing behavior |
| Mixed setup axes and tool profiles | Uses each operation's authoritative IPW source and cutter frame | Existing behavior |
| Camera, path, target overlay, projection | Shared camera and controls | Shared camera and controls |
| Lighting, FXAA and SSAO | Matching palette/light and offscreen color/depth postprocessing | Retained |
| GPU TSDF reconstruction | Ported cut/classification/Hermite surface shaders; compact coarse and fine indirect draws | Same compact coarse/fine lists and 12-vertex coarse draws |

In authoritative IPW playback, Vulkan never drives material removal with simplified display chords. The original
moves, checkpoints, explicit profiles and per-operation parameters remain the
material authority. Requests supersede older worker generations; closing a view
cancels and joins its worker. Both explicit tool profiles and legacy pocket sources
support incremental replay. A separate fully completed prefix checkpoint prevents
partial-move cuts from leaking into an earlier seek. Unknown cutting lengths are
resolved from the full source operation before slicing. Simplified display motions
map their fractional progress back to cumulative distances along original moves.
Project playback requests material-only worker results and uploads intervals through
reusable, coherent staging buffers per Qt frame slot. It reuses GPU allocations and
derives surfaces on the GPU, avoiding CPU triangulation and mesh conversion/upload.
Changes to grid orientation, dimensions or interval capacity rebuild allocations.
Device/resource limits fall back to the existing bounded mesh path; its quality
follows the scene quality and does not change underlying stock or NC data.

Fixed-axis display playback now keeps interval/count/TSDF data in device-local
buffers. Cut compute dispatches cover only each new motion's expanded bounding box,
including directed arc extrema. Push constants carry motion parameters; tool profiles
and grid metadata stay resident. Surface classification runs once after a batch of
cuts, compacts both coarse and fine entries, and writes two indirect draw commands.
Homogeneous coarse tiles emit only their 12 top/bottom vertices in both backends,
avoiding the 24 side vertices discarded by the original implementation.
Vulkan rewind restores a device-local initial
snapshot and replays to the requested step; it does not read material back to the CPU.
Long backward seeks still replay from the start and can cost more than forward steps.

OpenGL now shares the coarse-list compaction and indirect command layout with
Vulkan (fine command at byte 0, coarse command at byte 24). GL storage binding 7
maps to Vulkan binding 9 in the shader generator. Grid/surface/render constants
are refreshed on stock upload, and the compute cutter profile only on tool change.
The model-view-projection matrix is multiplied once on the CPU, as in Vulkan.
Same-size stock uploads reuse buffers and the VAO, updating contents instead of
destroying and reallocating storage; capacity changes still reallocate. Buffer-update,
shader-storage and indirect-command barriers preserve the producer/consumer ordering.

`PocketCamVulkanDexel` owns the native Vulkan buffers, descriptors and pipelines.
QRhi still owns the device, queue, frame scheduling and render targets. Native work
is recorded only inside `ExternalContent` passes between `beginExternal/endExternal`;
explicit barriers connect transfer, compute, indirect command reads and vertex reads.
Normal frames contain no queue-idle wait or material readback. Initialization,
resource teardown/recreation and explicit volume diagnostics may synchronize.
The native code stays private to `oap_app_cam_renderer`. The established OpenGL
algorithms are ported reproducibly by `node tools/Generate-CamVulkanShaders.cjs`;
generated GLSL is checked in and compiled by `qsb`, so builds do not require Node.
GPU display simulation, including its existing profile/grid approximation, remains
separate from authoritative verification and NC output.

Building requires Qt **6.11 or later**, its **Shader Tools** module (`qsb`), and
matching GuiPrivate headers. `qt_add_shaders` embeds the SPIR-V bundles; no loose
shader files or Vulkan SDK are required at runtime. Build-time Vulkan headers come
from `OAP_VULKAN_HEADERS_DIR`, the Vulkan SDK, or the pinned, SHA-256-verified
Vulkan-Headers v1.3.290 archive. Vulkan entry points come from Qt; no separate loader
library is linked. The GPU driver must provide
Vulkan. QRhi has [limited compatibility guarantees](https://doc.qt.io/qt-6/qrhiwidget.html),
so changing the Qt minor version requires rebuilding and native GPU validation.
The tested toolchain is Qt 6.11.1/MSVC, OCCT 7.9.3, on Intel UHD Graphics.
The full RelWithDebInfo application build and all 68 CTest cases passed with
native GPU tests enabled (165.22 seconds). The GPU port passed native
OpenGL/Vulkan volume and image comparisons for flat/ball tools, plunge,
linear and trusted circular cuts, rapid motion, completion and in-move rewind.
The flat-tool fixture's mean channel difference was below 0.0001/255. Authoritative
IPW final-frame comparisons were below 0.52/255 with AO on/off. These visual checks
are separate from the timing benchmark. Native render/switch tests also passed
with Khronos synchronization validation enabled and no validation errors.

The native acceptance test checks real Vulkan framebuffer pixels, reference IPW
volumes, partial cuts and in-move rewind, rapid moves, cancellation, completion,
mixed-axis partial ball-tool cuts, AO toggling, and matching-view OpenGL/Vulkan
target pixels. It also covers resize/projection changes, injected Vulkan failure and actual
window replacement while a native OCCT viewport remains open. Separate fresh-process
tests start with OpenGL and switch to Vulkan both before and after playback. The
Vulkan widget must select its API **before** receiving its QWidget parent; a native
OCCT ancestor can otherwise create a surface for Qt's default graphics API too early,
leading to a null Vulkan surface and an access violation during device initialization.
The original 65-test run used a Qt OpenGL parent and missed this native-window case.
The tests write screenshots to
`build/cam-renderer-artifacts`. It requires a desktop and a Vulkan/OpenGL-capable
driver; ordinary headless runs explicitly skip this test with code 77:

```powershell
$env:OAP_TEST_NATIVE_GPU = '1'
ctest --test-dir build -C RelWithDebInfo -R '^cam_renderer' --output-on-failure
```

GPU playback timing can be reproduced after building `oap_test_cam_renderer`:

```powershell
./tools/Measure-CamGpu.ps1 -BuildDirectory build -Configuration RelWithDebInfo
```

On 2026-11-09, Intel UHD Graphics / Qt 6.11.1 / RelWithDebInfo, both backends
used a 960 × 720 material grid (0.1 mm), the same flat-tool path and camera,
and a 1200 × 900 framebuffer. Each run warmed 40 frames and measured 120;
the second process reversed backend order. Timings include `setCurrentStep`
and synchronous framebuffer capture, excluding initialization, file writes and
material-volume diagnostics. No validation layer was enabled during timing.

The first measurement, **before applying equivalent optimizations to OpenGL**, was:

| Backend order | OpenGL median / p95 (ms) | Vulkan median / p95 (ms) | Median reduction |
| --- | --- | --- | --- |
| OpenGL, Vulkan | 46.77 / 51.59 | 29.70 / 35.83 | 36.5% |
| Vulkan, OpenGL | 45.83 / 51.97 | 30.15 / 36.82 | 34.2% |

The dense final-frame GPU volumes matched, and mean image channel difference was
below 0.000001/255. The original logs were retained as
`build/gpu-benchmark-before-opengl-optimization.log` and
`build/gpu-benchmark-vulkan-first-before-opengl-optimization.log`.
This measures fixed-axis GPU display
playback on one device, not arbitrary STEP workloads, authoritative CPU IPW
computation, or presentation-only interactive FPS. Project playback can still be
limited by authoritative material calculation and interval packing/transfers.

After applying the common optimizations to OpenGL, both the application and
renderer test executable rebuilt successfully. All four native renderer tests
passed (23.02 seconds), including additional profile-change/rewind coverage;
Khronos synchronization validation reported no errors. Shader regeneration was
checked with `node tools/Generate-CamVulkanShaders.cjs --check`.

The first two post-optimization runs varied substantially: GL-first measured
40.73 ms vs 29.54 ms, while Vulkan-first measured 32.16 ms vs 34.42 ms (GL vs VK
medians). To avoid drawing conclusions from that pair, the unchanged workload was
repeated three times in each order:

```powershell
./tools/Measure-CamGpu.ps1 -Repetitions 3
```

| Repetition / order | OpenGL median / p95 (ms) | Vulkan median / p95 (ms) | Vulkan median reduction |
| --- | --- | --- | --- |
| 1 / OpenGL first | 39.86 / 52.09 | 31.80 / 38.81 | 20.2% |
| 1 / Vulkan first | 32.75 / 38.70 | 30.88 / 37.11 | 5.7% |
| 2 / OpenGL first | 32.09 / 37.96 | 30.81 / 37.08 | 4.0% |
| 2 / Vulkan first | 32.84 / 41.38 | 33.27 / 39.26 | -1.3% |
| 3 / OpenGL first | 40.14 / 49.03 | 30.29 / 36.04 | 24.5% |
| 3 / Vulkan first | 34.87 / 40.24 | 31.65 / 40.25 | 9.3% |

Each backend has 720 measured frames across these six processes. Taking the median
of the six per-process medians gives **33.858 ms OpenGL vs 31.265 ms Vulkan**, a
**7.7% reduction** for Vulkan. This is a median-of-run-medians statistic, not a
pooled frame median or an estimate of intrinsic API overhead. Individual paired
Vulkan/OpenGL median ratios range from 0.755 to 1.013, so the remaining advantage
is not stable across runs. The
earlier 34–36% figure compared different optimization levels and should not be
used to characterize the current implementations.

All six dense final-frame volume comparisons passed and mean image channel
difference remained 0.000000617/255. Logs are `build/gpu-benchmark-run-{1,2,3}.log`
and `build/gpu-benchmark-vulkan-first-run-{1,2,3}.log`. The initial, variable pair
is retained in `build/gpu-benchmark.log` and `build/gpu-benchmark-vulkan-first.log`.

### Main viewport migration assessment

The independent simulation window is a suitable first migration boundary because
it consumes meshes, tool profiles and a motion timeline. The main view is coupled
to `OpenGl_GraphicDriver`, `V3d_Viewer` and `AIS_InteractiveContext` in
[`OcctView.cpp`](src/app/ui/view/OcctView.cpp), together with AIS-based model,
feature, stock and toolpath presentations under `src/app/presentation`.
Replacing its driver is therefore substantially broader than changing the
simulation widget's graphics API.

A custom Vulkan main view would need a render-independent scene snapshot with
stable face/edge/operation identities, mesh and wire presentations, depth-aware
selection and hover highlighting, transparency, overlays, camera/fit behavior and
their interaction with workspace state. Geometry, recognition and CAM can remain
on OCCT; the main-view presentation and selection contracts require an adapter
and their own regression suite. The current Vulkan simulation renderer does not
provide those AIS interaction semantics.

Keep the OCCT main view for now. Before authorizing a main-view replacement, compare
OpenGL and Vulkan on the same representative STEP models and IPW states: first
visible frame, stationary/rotating frame-time percentiles, upload time and memory,
then validate picking/highlighting and camera results. The small native acceptance
fixture proves rendering and material correctness, not performance on
`rounded_relief.step`. A useful next graphics experiment is GPU material-update
parity inside the isolated simulator, followed by measurements; a full main-view
rewrite is not justified by this first-stage result alone.

### General regression

From the repository root:

```powershell
cmake -S . -B build
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

The fast test suite covers module boundaries plus representative analysis,
recognition, toolpath, manufacturing, IPW, presentation and task-lifecycle
behavior. Large ignored STEP files and archived experiment counts are not part
of the deterministic test contract.

See [BUILDING.md](BUILDING.md) for dependency setup, supported configurations,
platform presets, core-only builds, installation and cross-platform CI.

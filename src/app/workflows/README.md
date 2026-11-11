# Application workflows

`src/app/workflows` contains UI-independent application orchestration that
connects model/selection values, toolpath/manufacturing services and desktop
workspace transaction ports. Concrete dialogs and widgets live elsewhere.
Source-to-target ownership is enforced by
[`cmake/CheckWorkflowBoundaries.cmake`](../../../cmake/CheckWorkflowBoundaries.cmake).

## CMake ownership

| Target | Owned workflow sources | Responsibility |
| --- | --- | --- |
| `oap_app_workflow_base` | most of `shared/` | document, recognition, selection and interaction value contracts |
| `oap_app_workflow_playback` | `playback/` | program flattening, playback tasks and filter state |
| `oap_app_workflow_simulation` | `simulation/` plus hole stock helpers | IPW execution, comparison, display-mesh work and simulation sessions |
| `oap_app_workflow_cam` | `hole/`, `pocket/`, `profile/`, `engraving/`, `project/` and CAM-specific shared files | feature generation, project/parameter/tool-library coordination and export |

Do not infer ownership only from the physical folder. The CMake source lists,
public-header manifest and workflow-boundary checker are authoritative.

## Layer pattern

```text
MainWindow / UI callback
  -> WorkflowController or command policy
      -> GenerationUseCase
          -> Runner / domain workflow / manufacturing service
      -> value result
  -> workspace transaction port
      -> atomic Job / preview / playback / IPW publication
```

- **Runner** adapts progress/cancellation and invokes a UI-independent domain
  service. It does not own long-lived state.
- **GenerationUseCase** validates a request, orchestrates generation/program
  validation/preview and returns a value result with a stable failure category.
- **WorkflowController/command policy** prepares geometry and coordinates an
  atomic commit boundary; it does not render OCCT objects directly.
- **Session** is owned by its workspace/facade. A UseCase may receive values or
  ports, but does not expose mutable session state to `MainWindow`.
- **Presenter/UI** consumes a completed value result. It does not decide whether
  geometry or IPW evidence is valid.

## Transaction rules

1. Capture all request values before starting background work; workers do not
   read mutable widgets, selection containers or an active document by pointer.
2. Cancellation, generation failure, stale generation ID, validation failure or
   preview failure preserves the last valid Job/runtime/presentation state.
3. Preview publication is reversible and distinct from committing a formal Job
   operation. A preview cannot be saved/exported as formal merely because it is
   visible.
4. Commit related Job, program, preview, playback and IPW state through one
   workspace transaction after every required gate succeeds.
5. Controllers return structured failure categories and diagnostics; top-level
   windows only choose user-facing presentation.
6. Model switches cancel tasks and invalidate model-scoped sessions through
   workspace entry points before publishing new document state.
7. App workflows do not bypass Toolpath Formal gates, Job fingerprints or IPW
   parent-state checks.
8. Cross-view selection uses the formal Job operation ID. Runtime expansion
    suffixes are playback-only identities; stale, disabled and missing-IPW
    operations remain selectable with an explicit no-replay state.

## Feature ownership

| Workflow folder | Domain | Current formal state |
| --- | --- | --- |
| `hole/` | [`toolpath/hole`](../../toolpath/README.md) | formal Job, program and IPW |
| `pocket/` | [`toolpath/pocket`](../../toolpath/README.md) | formal Job, program and IPW |
| `profile/` | [`toolpath/profile`](../../toolpath/README.md) | formal Job, program and IPW |
| `engraving/` | [`toolpath/finishing`](../../toolpath/finishing/README.md) | full-model raster engraving, all six faces or any individual face, dedicated library/direct tool, formal Job and IPW |
| `project/` | [`manufacturing/job`](../../manufacturing/README.md) | schema v12 lifecycle, project loading and shared regeneration |
| `simulation/` | [`manufacturing/ipw`](../../manufacturing/README.md) | unified Hole/Pocket/Profile/Engraving material chain |
| shared export | [`toolpath/program`](../../toolpath/README.md) | active-program reference NC export |

Engraving shares the normal parameter editor and workflow panel. One admitted heavy
job generates the requested faces sequentially, then replaces the engraving family
atomically, preserving preceding operations and stable identities for retained faces.
An explicit single-tool snapshot needs no library. Local process defaults are stored
separately from the older hole/pocket/profile preferences; saved Job operations retain
their actual tool/Setup/parameter snapshots using the compatible surface-finishing schema.

Hole, Pocket and Profile generation requests also accept `singleTool`. When present,
this snapshot takes precedence over `automaticLibrary`; invalid input fails without
fallback. `project/DirectToolSource` validates the tool and creates a temporary
one-tool source for the existing planning and safety pipeline. Hole accepts drills
and flat end mills; Pocket/Profile accept flat, ball-end and bull-nose mills. No manual hole-segment or
strategy-override API is restored. Capture-only `operationToolLibraryId` overrides
the controller source to an empty ID while retaining the project's active-library
preference. The controller's complete snapshot supports save/reopen/regeneration,
and the installer restores a family's SingleTool mode when its operations share
one standalone snapshot. Failed or cancelled candidates never replace the current Job.
Existing hole strategy ownership is preserved: through holes require drilling;
flat end mills handle planar-bottom blind holes and millable counterbore shoulders.
An explicit end mill does not force a through-hole milling strategy.

Engraving defaults to all six faces and allows an individual ±X/±Y/±Z selection.
Reopening restores the stored face set, including older multi-face subsets.
`single_tool_workflow` covers the three feature families, direct-tool validation,
cancellation, mixed-project capture and regeneration without a user library.

Project IPW animation prefers GPU Multi-Dexel rendering. If an authoritative
project or engraving IPW run completes, the UI first opens the same checkpoint
timeline used for pocket IPW. Closing it retains the final material checkpoint;
only the explicit animation button opens the animation viewer, initially paused.
An empty timeline never requests animation. If an authoritative
checkpoint exceeds the dense GPU cell/interval or SSBO block budget, or upload
fails, playback switches to a bounded display mesh from that same checkpoint.
Subsequent meshes are prepared by the display worker; material and cutter positions
are published together. This fallback lowers display detail only, leaving removal,
audit and NC data unchanged. The status reports the fallback reason. Empty material
checkpoints are valid. `IpwDisplayMeshController::meshOptionsForQuality` exposes the
shared mesh budget policy without exposing cache implementation headers.

Profile generation accepts an optional mold-local setup face. Filtering precedes
tool selection and Job capture; an empty or unresolved selection fails without
changing the current Job. The desktop scope selector shows each direction's region
count and maximum depth. Successful generation replaces the Profile family for the
selected scope. Automatic tools are selected independently per direction/depth group;
any failed selected group rejects the entire candidate instead of publishing a partial
program. Other operation families remain intact.

## Verification

Run from the repository root after dependency setup and preset configuration in [BUILDING.md](../../../BUILDING.md). The example uses the Windows desktop `windows-vs2022` preset; for VS 2026, change both the preset name and build directory to `windows-vs2026`. See the build guide for core configurations and full-suite commands.

```powershell
cmake --build --preset windows-vs2022
ctest --test-dir build/windows-vs2022 -C RelWithDebInfo --output-on-failure --parallel 1 --no-tests=error `
  -R "^(app_workflow_controllers|machining_project_controller|machining_workspace|desktop_workspace|stock_simulation_workspace|playback_workspace|task_lifecycle|workflow_internal_boundaries|module_boundaries)$"
```

Changes to ownership or public ports must update
`cmake/modules/OAPAppSources.cmake`,
`cmake/public/OAPAppPublicHeaders.cmake`, direct dependencies and the
workflow-boundary test in the same change.

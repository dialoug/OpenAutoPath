# Toolpath modules

`src/toolpath` contains the controller-independent CAM kernel, replaceable
algorithm registry, workflow services and neutral program boundary. CMake target
lists are the source of truth for source ownership and direct dependencies; see
[`ARCHITECTURE.md`](../../ARCHITECTURE.md) and
[`cmake/OAPModuleDependencies.cmake`](../../cmake/OAPModuleDependencies.cmake).

## Module map

| Target | Directory | Responsibility | Development plan |
| --- | --- | --- | --- |
| `oap_toolpath_core` | `core/` | common moves, primitives, parameters, tools, Setup, validation and stock primitives | [`core/DEVELOPMENT_PLAN.md`](core/DEVELOPMENT_PLAN.md) |
| `oap_toolpath_program` | `program/` | neutral-program validation, post-processing, NC review and atomic export | [`program/DEVELOPMENT_PLAN.md`](program/DEVELOPMENT_PLAN.md) |
| `oap_toolpath_hole` | `hole/` | fixed-axis drilling and hole-milling kernels | [`hole/DEVELOPMENT_PLAN.md`](hole/DEVELOPMENT_PLAN.md) |
| `oap_toolpath_pocket` | `pocket/` | pocket geometry, clearing, safety and Multi-Dexel primitives | [`pocket/DEVELOPMENT_PLAN.md`](pocket/DEVELOPMENT_PLAN.md) |
| `oap_toolpath_profile` | `profile/` | fixed-axis external clearing and contour finishing | [`profile/DEVELOPMENT_PLAN.md`](profile/DEVELOPMENT_PLAN.md) |
| `oap_toolpath_finishing` | `finishing/` | OpenCAMLib fixed-axis raster relief engraving and neutral program adaptation | [`finishing/README.md`](finishing/README.md) |
| `oap_toolpath_algorithms` | `algorithms/` | stable algorithm interfaces, built-ins and immutable registry | [`algorithms/DEVELOPMENT_PLAN.md`](algorithms/DEVELOPMENT_PLAN.md) |
| `oap_toolpath_workflow` | `workflow/` | UI-independent feature services, batch orchestration and async execution | [`workflow/DEVELOPMENT_PLAN.md`](workflow/DEVELOPMENT_PLAN.md) |

The persistent operation model and ordered material-state chain are owned by
[`manufacturing/job`](../manufacturing/job/DEVELOPMENT_PLAN.md) and
[`manufacturing/ipw`](../manufacturing/ipw/DEVELOPMENT_PLAN.md). Desktop
orchestration rules are documented in
[`app/workflows`](../app/workflows/README.md).

## Dependency direction

```text
app / manufacturing / tooling adapter
  -> toolpath workflow
      -> algorithms
          -> feature kernels
              -> core
  -> program
      -> core
```

Feature kernels do not depend on Job, Qt widgets, presentation objects or app
sessions. `program` consumes only the neutral core program model. `algorithms`
selects feature kernels but does not own project state. `workflow` composes
complete UI-independent batches but does not persist or display them.

The integrated engraving entry calls `oap_toolpath_finishing` through the shared
heavy-job scheduler. Its formal Job definition stores complete target units,
setup, tool snapshot and per-operation parameters. Schema 12 rebuilds these
operations on reopen, and the ordered Job IPW chain consumes their native
fixed-axis moves and actual cutter profiles. Finishing retains its own operation
kind throughout the tree, playback, program assembly and reference NC export.
The project IPW entry opens continuous animation after the ordered simulation
completes, including engraving-only projects and mixed setups on all six faces.
Project animation derives material frames asynchronously from original move
ranges with each operation's cutter profile; forward playback reuses the previous
immutable frame, while seeking restores a formal parent checkpoint. OpenGL renders
these material states and the operation's tool axis. Display simplification does
not drive removal, and the final frame uses the authoritative project checkpoint.
The single-setup GPU cutting path remains separate and retains its fixed-axis guard.
The dedicated engraving IPW action offers all engraving directions or one setup.
It simulates the global prefix through the last selected engraving operation to
preserve preceding material removal, then plays only the selected engraving paths
from their parent checkpoints. Later unrelated operations are excluded; stale
prerequisite operations still block the run. Job definitions remain unchanged.
The OpenCAMLib shared library has OpenMP disabled and adds no worker pool.
The shared engraving parameter page offers finish-only (the compatible default)
or stock roughing followed by finishing. Stock mode uses the formal project stock's
conservative envelope and bounded layer depths, then finishes with the same cutter.
It does not trim paths against exact previous IPW residual material. Simulation-only
stock overrides do not change the planning stock. Legacy preferences/projects retain
finish-only behavior; malformed new mode, depth, allowance or movement-budget fields
are rejected without replacing current settings.

Hole/Pocket/Profile offer active-library selection or an explicitly entered single
tool in their standard parameter pages. Hole supports drills and flat end mills;
Pocket/Profile support flat, ball-end and bull-nose mills. A direct tool uses the same geometry,
automatic strategy selection, reach, coverage and safety checks. It does not
restore manual hole segments or per-stage strategy overrides. Invalid or unsuitable
direct tools fail without falling back to the active library.
The current automatic hole policy drills through holes and mills planar-bottom
blind holes or suitable counterbore shoulders; choosing a direct end mill does
not override that policy to mill an arbitrary through hole.

`DirectToolSource` validates and adapts a T1–T9999 snapshot to a temporary one-tool
planning source; it never adds a catalog entry. Formal controllers store an empty
library ID and their complete snapshot, so saved projects regenerate without that
library. Active-library preferences remain unchanged. Engraving also supports a
dedicated library or direct tool; its standard parameter page defaults to all six
Setup faces and can select any individual ±X/±Y/±Z face. Process depth, allowance
and linking settings remain editable.

## Service lifecycle and planning stages

Hole, Pocket and Profile share private parameter preparation in `workflow/detail`.
Each service retains its own input mapping, cancellation precedence, failure
records, coverage audit and publication policy. Profile generation and evaluation
materialization reuse region execution and safety-result handling. These helpers
are implementation details, not a new public operation base class.

Profile planning separates geometry from motion:

```text
ProfileContour: cached sections -> layers -> offsets/clearing loops
  -> detail/ProfileMotionPlanning: travel ordering -> safe links -> moves/primitives
  -> ProfileToolpathService: safety validation -> ordered batch merge
```

The motion stage is independently compiled within `oap_toolpath_profile` and
exposes only travel optimization, distance evaluation and move assembly to its
caller. Evaluation and assembly share the same link-safety implementation.
`ProfileContour` continues to own caching, stage timing, cancellation and layer
budgets; the split does not introduce another scheduler. Job persistence and IPW
publication remain in manufacturing and application workflows.

The former stock simulation `.inc` fragments now compile independently behind
private stage headers. `MultiDexelStock` consumes `FixedAxisToolpathView` for native
material updates; Pocket overloads are compatibility wrappers. Hole and Profile
IPW updates pass their own authoritative moves directly, without copying them into
a Pocket operation. The view borrows storage only for the synchronous apply call.
Legacy BRep and display adapters remain explicit in
`workflow/compatibility/LegacyProfileSimulationAdapter`, outside the Profile kernel.
Module checks reject new `.inc` files and textual `.cpp`/`.inc` includes.

The 2026-11-08 migration removed all 32 first-party `.inc` files (14,847 lines of
source fragments). RelWithDebInfo builds successfully and all 54 CTest entries
pass. `fixed_axis_execution` compares neutral and compatibility adapters by
material fingerprint, including prefix replay, oriented sweeps and cancellation.
Existing IPW, Job and parallel-budget regressions remain enabled. Extraction
checks also preserve 105 stock helper bodies and 65 GL methods/shader strings;
these checks do not substitute for manual interactive OpenGL validation.
Local verification logs are `build/inc-refactor-final-build.log` and
`build/inc-refactor-ctest.log`.

## Canonical documentation ownership

To prevent several roadmaps from becoming competing sources of truth:

| Decision | Canonical document |
| --- | --- |
| common move/primitive/parameter/tool contracts | [`core/DEVELOPMENT_PLAN.md`](core/DEVELOPMENT_PLAN.md) |
| algorithm IDs, registry and default-admission policy | [`algorithms/DEVELOPMENT_PLAN.md`](algorithms/DEVELOPMENT_PLAN.md) |
| batch, cancellation, async and publication orchestration | [`workflow/DEVELOPMENT_PLAN.md`](workflow/DEVELOPMENT_PLAN.md) |
| Hole/Pocket/Profile geometry and safety semantics | the corresponding feature plan |
| `.oapjob` schema, migrations, stable references and runtime invalidation | [`manufacturing/job/DEVELOPMENT_PLAN.md`](../manufacturing/job/DEVELOPMENT_PLAN.md) |
| Multi-Dexel truth, node/checkpoint identity and engine versions | [`manufacturing/ipw/DEVELOPMENT_PLAN.md`](../manufacturing/ipw/DEVELOPMENT_PLAN.md) |
| neutral program packaging and post-processor capability | [`program/DEVELOPMENT_PLAN.md`](program/DEVELOPMENT_PLAN.md) |
| desktop Runner/UseCase/Controller transactions | [`app/workflows/README.md`](../app/workflows/README.md) |

Feature plans may summarize a cross-module dependency, but version numbers,
field definitions and engine IDs must be changed in the canonical document
first and linked from the summary.

## Shared release gate

Every production toolpath change must preserve:

1. deterministic ordering, diagnostics and fingerprints for identical input;
2. cooperative cancellation without partial batch publication;
3. equivalence between semantic primitives and authoritative fallback moves;
4. complete tool/Setup snapshots at formal publication boundaries;
5. program validation before preview/export and material validation before IPW publication;
6. module ownership, public-header and direct-dependency checks;
7. explicit sample/parallel budgets rather than unbounded work.

From the repository root:

```powershell
cmake --build build --config Release --parallel 16 -- /verbosity:minimal
ctest --test-dir build -C Release --output-on-failure --parallel 4
cmake -DOAP_SOURCE_DIR="$PWD" -P cmake/CheckDocumentationLinks.cmake
```

## Parallel execution and optimized builds

Current scheduling and budget contracts are maintained in
[`core/DEVELOPMENT_PLAN.md`](core/DEVELOPMENT_PLAN.md) and
[`workflow/DEVELOPMENT_PLAN.md`](workflow/DEVELOPMENT_PLAN.md).
Generated STEP inputs, reproducible benchmark commands and measured policy
comparisons are recorded in
[`core/PARALLEL_BENCHMARK_RESULTS.md`](core/PARALLEL_BENCHMARK_RESULTS.md).

Use `RelWithDebInfo` for performance measurements; it enables compiler optimization
while retaining symbols. With the Visual Studio generator, `OAP_BUILD_PARALLEL_JOBS`
defaults to the detected logical CPU count. MSBuild MultiToolTask shares this
compiler-process limit across projects, so project and file parallelism do not
multiply into an uncontrolled number of compiler processes.

Hole batches, automatic Pocket planning, and Profile candidates, regions and
layer patterns share the process-wide budget in `core/ToolpathParallel.h`.
Pocket reserves a parent lease and subdivides only its granted slots; the calling
thread participates, and zero extra slots means serial computation on that thread.
The parent lease is returned after the region workers join. Synchronous budget
scopes also cover legacy move/primitive validation and safety preparation; worker
wrappers restore their previous scopes even when a callback throws.
Independent tasks use up to the
available logical CPU count; nested Profile tasks may borrow unused slots.
Explicit caller budgets are preserved, and ordered merges retain deterministic
program output. Material-envelope accumulation and the IPW operation chain remain
ordered because later states depend on earlier states.

`OAP_CAM_MAX_THREADS` optionally caps this toolpath worker budget per process
(e.g. `1`, `4`, `14` for comparative runs); unset it for hardware-based allocation.
Root workflow threads and memory samplers are outside the additional-worker budget.
It is not a CPU-affinity setting or a hard cap on OCCT, Qt, rendering or IPW threads.
Boolean-only OCCT operations now share a nonblocking admission gate and require
the verified native backend's complete pool demand. Unknown/TBB backends or a pool
larger than the configured budget fall back to serial, without reconfiguring the
process pool. Multi-Dexel Tile updates now use the same worker budget and retain
transactional cancellation/failure semantics. Recognition receives a synchronous
range-execution callback from the application; standalone recognition defaults to
serial execution without depending on toolpath. The unused hole-preview parallel
builder has been removed; playback retains its existing request coalescing.

`core/HeavyJobScheduler.h` admits independent expensive jobs before execution:
one persistent worker, at most 32 pending jobs, FIFO start order, and immediate
queued cancellation through `cancelHeavyJob`. Pocket/Hole/Profile async services,
automatic desktop runners, recognition and stock/IPW simulation share this queue.
Pending jobs do not create waiting threads. Queue-full and shutdown results are
explicit failures; callers can cancel without waiting for a preceding job.
Input captures are released before futures become ready. Synchronous runners use
join guards, and async task handles cancel and wait on destruction. Nested jobs
must call synchronous services, not submit and wait on this same queue.
This is admission control for these entry points, not a hard process-wide CPU cap:
direct synchronous API calls, custom budgets, model-loading workers and rendering
remain outside root-job admission. FIFO does not provide preemption or time slices.
The Profile regression executable accepts `--profile-sample <STEP path>` to run
recognition, generation, safety validation and solid intrusion checks and report
generation time plus layer worker counts. Customer CAD is supplied locally and
not committed to the repository.

## Documentation maintenance

IPW target auditing retains a session-owned `StockTargetCache` in the machining
job runtime. Exact indexed world bounds are reused only for complete indexed
units; unknown geometry falls back to `AddOptimal(false, false)`. Model cache
format V3 excludes old triangulation-derived bounds. Target raster reuse checks
shape/location/orientation, model revision and units, grid axes/origin/extents,
resolution, dimensions and tolerances. The cache holds at most one raster with a
256 MiB limit and is discarded with the job runtime. In-place geometry mutation
is unsupported: replacing model geometry must invalidate the runtime.
Raster interval extraction consolidates crossings in place and reuses per-column
scratch storage, avoiding millions of temporary allocations during feature audits.

Multi-Dexel statistics are recomputed for touched tiles and summed in stable tile
order. Material fingerprints retain their original byte algorithm but are computed
once on demand, safely across concurrent readers. GPU-only transient frames do
not request them; formal IPW checkpoints still use full content fingerprints.
Volume summation now follows tile order, so last-bit floating point differences
from older builds are possible without any change to material intervals.

Multi-Dexel cells keep one material interval inline in a 24-byte private container
on 64-bit builds. Empty and single-interval cells need no interval heap allocation;
split cells own a contiguous heap buffer and return to inline storage when
subtraction leaves at most one interval. Tile copies deep-copy this storage, so
published checkpoints remain immutable and cancelled updates discard their child
state. Public vector copies and read-only span views retain their existing contracts.
Initial raster conversion reuses one column buffer rather than retaining a vector
for every ray. This optimizes allocation and locality; it does not change the shared
worker budget or imply a measured hardware cache hit rate. The interval smoke test
has an exact private-header exception for ownership and lifetime checks; this
container is not part of the module's public API.

- Every plan records its current stage, task key, code baseline, maintenance
  role and latest verified test evidence.
- Update a plan in the same change that alters its public contract, roadmap
  status or default algorithm behavior.
- Keep raw experiment logs in ignored local artifacts; retain reproducible benchmark
  methods and results beside the owning module. Active contracts and next work
  remain beside production code.
- Add a full development plan only for a target or independently owned domain.
  Small implementation folders should link to their owner instead of copying a
  roadmap.

## Verification record

| Date | Code baseline | Evidence |
| --- | --- | --- |
| 2026-11-09 | `d3278da` + current working changes | RelWithDebInfo application build and CTest 62/62 passed. Exact target caching, six-axis audit equivalence, incremental tile statistics and concurrent lazy fingerprints verified. Local rounded_relief STEP (-Z setup, simulation library): first audit 78.49→2.48 s (1M cells), 90.37→10.74 s (~8M cells); both material fingerprints and allowance/overcut volumes match the previous baseline. |
| 2026-11-08 | `ee6f2cd` + current working changes | RelWithDebInfo full application build and CTest 51/51 passed after Tile/recognition budgeting and heavy-job admission |
| 2026-08-24 | `612ab76` | Release CTest 51/51 passed, including feature, manufacturing, app-workflow, architecture and documentation-link suites |

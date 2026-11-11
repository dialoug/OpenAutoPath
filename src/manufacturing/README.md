# Manufacturing module

`oap_manufacturing` owns the persistent machining Job and the runtime lifecycle
needed to rebuild, execute and compare hole, pocket, profile and surface-finishing operations.
The persistent format is schema v12; `surface_finishing_operations` stores the
OpenCAMLib operation definitions, including engraving mode, maximum step-down
and roughing allowance. Older finishing-only definitions retain that mode when
migrated; layered roughing must not silently become finishing-only. Legacy files with nonempty or malformed
`surface_operations` are rejected before the active definition changes; older
files without those retired operations remain loadable. Projects with retired manual hole plans, local batches, operation overrides or
manual tool controllers are rejected before installation; automatic library-backed
projects migrate without discarding operations. The module is a
UI-independent application/domain module; dialogs, progress presentation and
scene updates belong to the focused `oap_app_workflow_*` targets or
`OpenAutoPathApp`.

The repository-wide dependency rules are defined in
[`ARCHITECTURE.md`](../../ARCHITECTURE.md). The component map and boundary contracts below describe the current Job and IPW responsibilities.

## Current components

- `job/MachiningJob.*` defines the Job aggregate. Its definition contains the
  serializable inputs needed to rebuild operations, while its runtime contains
  generated toolpaths and IPW caches.
- `job/MachiningJobStorage.*` serializes the persistent definition and computes
  input/execution fingerprints.
- `job/MachiningJobApplicationService.*` synchronizes model/setup state before
  load and save operations.
- `program/MachiningProgramAssembler.*` validates formal runtime freshness and
  assembles Hole/Pocket/Profile/SurfaceFinishing child operations in the Job's global
  sequence while retaining a traceable formal-to-runtime mapping.
- `program/MachiningProgramPartitioner.*` requires the current complete Ready
  IPW/checkpoint chain and partitions only consecutive operations across hard
  Setup/WCS, immutable tool-controller and hole-phase boundaries.
- `hole/HoleMachiningJobService.*` captures and regenerates hole operations.
- `pocket/PocketMachiningJobService.*` captures and regenerates pocket
  operations.
- `profile/ProfileMachiningJobService.*` captures and regenerates profile
  operations.
- `finishing/SurfaceFinishingMachiningJobService.*` captures and regenerates
  OpenCAMLib finishing operations with stable unit references and frozen tools.
- `ipw/IpwSession.*`, `IpwExecutor.*` and `IpwCheckpointStore.*` manage the
  ordered in-process-workpiece plan and reusable checkpoints.
- `ipw/HoleMultiDexelUpdater.*` applies hole results to the shared stock model; the executor also applies
  pocket/profile semantic toolpaths and finishing moves to the same material chain.

## Related documentation

- [Toolpath modules](../toolpath/README.md) describe generation, validation and the program boundary.
- [Surface finishing](../toolpath/finishing/README.md) describes engraving parameters and integration.
- [Application workflows](../app/workflows/README.md) describe desktop transactions and project coordination.

## Boundary contract

The module directly depends on `analysis`, the tool library, the
tooling/toolpath adapter, toolpath core/program, hole/pocket/profile/finishing
kernels, and the UI-independent toolpath workflow. These edges are explicit because
public Job and service contracts currently expose the corresponding value
types.

The module must not:

- include desktop widgets, `MainWindow`, `OcctView` or presentation objects;
- store widget state in the persistent Job definition;
- persist generated OCCT toolpaths, display meshes or IPW cache objects as Job
  source data;
- access another module through a dependency that is only transitively linked.

Changes to public manufacturing contracts must also update the explicit API
manifest and, when a new first-party module is referenced, the canonical direct
dependency graph.

## Regression coverage

The deterministic fast suite exercises Job validation and storage projections,
project-level neutral-program assembly/partitioning and IPW publication gates, hole batch identity,
Hole/Pocket/Profile/SurfaceFinishing IPW behavior, project regeneration and DTO mapping.
After configuring dependencies as described in [BUILDING.md](../../BUILDING.md),
run the fast suite from the repository root (use `windows-vs2026` for VS 2026).
For full regression, use the unfiltered commands in the build guide:

```powershell
cmake --build --preset windows-vs2022
ctest --preset windows-vs2022 --parallel 1
```

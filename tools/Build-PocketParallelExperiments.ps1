param([string]$BuildDirectory = 'build')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$buildPath = [IO.Path]::GetFullPath((Join-Path $root $BuildDirectory))
$destination = Join-Path $buildPath 'parallel-experiments'
[IO.Directory]::CreateDirectory($destination) | Out-Null
$schedulerPath = Join-Path $root 'src/tooling/adapter/pocket/detail/PocketPlanningScheduler.cpp'
$plannerPath = Join-Path $root 'src/tooling/adapter/pocket/PocketAutomaticToolpath.cpp'
$scheduler = [IO.File]::ReadAllText($schedulerPath)
$planner = [IO.File]::ReadAllText($plannerPath)
$encoding = [Text.UTF8Encoding]::new($false)
# Retain recoverable source snapshots if the host terminates before finally.
[IO.File]::WriteAllText((Join-Path $destination 'PocketPlanningScheduler.original.cpp'), $scheduler, $encoding)
[IO.File]::WriteAllText((Join-Path $destination 'PocketAutomaticToolpath.original.cpp'), $planner, $encoding)
$experiments = @(
    @{Name='dynamic'; Feedback='true'; Track='true'; Nested='true'; Prefetch='4u'},
    @{Name='fixed'; Feedback='false'; Track='false'; Nested='true'; Prefetch='4u'},
    @{Name='no-prefetch'; Feedback='true'; Track='true'; Nested='true'; Prefetch='1u'},
    @{Name='regions-only'; Feedback='true'; Track='true'; Nested='false'; Prefetch='4u'}
)
foreach ($name in @('kDynamicMemoryFeedback', 'kTrackRegionMemory', 'kBorrowNestedWorkers')) {
    if ($scheduler -notmatch "constexpr bool $name = true;") { throw "Unexpected baseline policy: $name" }
}
if ($planner -notmatch 'kMaximumFullStagePrefetch = 4u;') { throw 'Unexpected prefetch baseline.' }
try {
    foreach ($experiment in $experiments) {
        $source = $scheduler.Replace('kDynamicMemoryFeedback = true;', "kDynamicMemoryFeedback = $($experiment.Feedback);").
            Replace('kTrackRegionMemory = true;', "kTrackRegionMemory = $($experiment.Track);").
            Replace('kBorrowNestedWorkers = true;', "kBorrowNestedWorkers = $($experiment.Nested);")
        [IO.File]::WriteAllText($schedulerPath, $source, $encoding)
        [IO.File]::WriteAllText($plannerPath,
            $planner.Replace('kMaximumFullStagePrefetch = 4u;', "kMaximumFullStagePrefetch = $($experiment.Prefetch);"), $encoding)
        Write-Output "Building experiment $($experiment.Name)"
        $log = Join-Path $destination "$($experiment.Name)-build.log"
        & cmake --build $buildPath --config RelWithDebInfo --target oap_test_pocket_automatic_toolpath --parallel 6 *> $log
        if ($LASTEXITCODE -ne 0) { Get-Content $log -Tail 40; throw 'Experiment build failed.' }
        Copy-Item -LiteralPath (Join-Path $buildPath 'RelWithDebInfo/tests/oap_test_pocket_automatic_toolpath.exe') `
            -Destination (Join-Path $destination "$($experiment.Name).exe")
        $experiment | ConvertTo-Json | Set-Content -Encoding UTF8 (Join-Path $destination "$($experiment.Name)-policy.json")
    }
} finally {
    # Restore the exact input sources even if a compiler invocation fails.
    [IO.File]::WriteAllText($schedulerPath, $scheduler, $encoding)
    [IO.File]::WriteAllText($plannerPath, $planner, $encoding)
}
& cmake --build $buildPath --config RelWithDebInfo --target oap_test_pocket_automatic_toolpath --parallel 6 `
    *> (Join-Path $destination 'restored-build.log')
if ($LASTEXITCODE -ne 0) { throw 'Could not rebuild the restored production policy.' }

param(
    [string]$Directory = 'artifacts/parallel-benchmark',
    [string[]]$Tags = @('baseline', 'baseline-stress', 'dynamic', 'fixed', 'no-prefetch', 'regions-only')
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$directoryPath = [IO.Path]::GetFullPath((Join-Path $root $Directory))
$records = foreach ($tag in $Tags) {
    Get-Content -Raw -LiteralPath (Join-Path $directoryPath "$tag.json") | ConvertFrom-Json
}
foreach ($sample in ($records | Group-Object sample)) {
    if (@($sample.Group.hash | Sort-Object -Unique).Count -ne 1) {
        throw "Geometry/semantic output differs for $($sample.Name)."
    }
}
$summary = foreach ($group in ($records | Where-Object iteration -gt 0 | Group-Object tag,sample,thread_limit)) {
    $rows = @($group.Group)
    if ($rows.Count -ne 3) { throw "Expected three measured repetitions: $($group.Name)" }
    $times = @($rows.milliseconds | Sort-Object)
    [PSCustomObject]@{
        tag = $rows[0].tag
        sample = $rows[0].sample
        thread_limit = $rows[0].thread_limit
        median_ms = [Math]::Round($times[1], 3)
        minimum_ms = [Math]::Round($times[0], 3)
        maximum_ms = [Math]::Round($times[2], 3)
        peak_working_set_mib = [Math]::Round(($rows.peak_working_set_bytes | Measure-Object -Maximum).Maximum / 1MB, 2)
        sampled_peak_threads = ($rows.sampled_peak_threads | Measure-Object -Maximum).Maximum
        minimum_planning_peak = ($rows.planning_peak | Measure-Object -Minimum).Minimum
        maximum_planning_peak = ($rows.planning_peak | Measure-Object -Maximum).Maximum
        moves = $rows[0].moves
        primitives = $rows[0].primitives
        hash = $rows[0].hash
    }
}
$summary | ConvertTo-Json -Depth 4 | Set-Content -Encoding UTF8 (Join-Path $directoryPath 'summary.json')
$summary | Where-Object thread_limit -eq 8 | Sort-Object sample,tag |
    Format-Table tag,sample,median_ms,peak_working_set_mib,sampled_peak_threads,maximum_planning_peak -AutoSize
Write-Output "All $($records.Count) runs have matching per-sample output hashes."

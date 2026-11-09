param(
    [Parameter(Mandatory = $true)]
    [int]$TargetProcessId,

    [Parameter(Mandatory = $true)]
    [string]$CsvPath,

    [Parameter(Mandatory = $true)]
    [string]$JsonPath,

    [int]$DurationSeconds = 600,
    [int]$IntervalMilliseconds = 500,
    [double]$MinimumAvailableMiB = 2300.0,
    [double]$MaximumPrivateMiB = 7500.0,
    [string]$StopSignalPath = '',
    [switch]$StopOnLimit
)

$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName Microsoft.VisualBasic

$csvFullPath = [System.IO.Path]::GetFullPath($CsvPath)
$jsonFullPath = [System.IO.Path]::GetFullPath($JsonPath)
$csvDirectory = [System.IO.Path]::GetDirectoryName($csvFullPath)
$jsonDirectory = [System.IO.Path]::GetDirectoryName($jsonFullPath)
[System.IO.Directory]::CreateDirectory($csvDirectory) | Out-Null
[System.IO.Directory]::CreateDirectory($jsonDirectory) | Out-Null

$computerInfo = [Microsoft.VisualBasic.Devices.ComputerInfo]::new()
$samples = [System.Collections.Generic.List[object]]::new()
$started = Get-Date
$stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
$reason = 'duration_limit'
$stoppedProcess = $false
$processorCount = [Math]::Max(1, [Environment]::ProcessorCount)

$process = Get-Process -Id $TargetProcessId
$previousCpuSeconds = $process.TotalProcessorTime.TotalSeconds
$previousElapsedSeconds = 0.0

while ($stopwatch.Elapsed.TotalSeconds -lt $DurationSeconds) {
    Start-Sleep -Milliseconds $IntervalMilliseconds
    try {
        $process = Get-Process -Id $TargetProcessId
        $process.Refresh()
    } catch {
        $reason = 'process_exit'
        break
    }

    $elapsedSeconds = $stopwatch.Elapsed.TotalSeconds
    $cpuSeconds = $process.TotalProcessorTime.TotalSeconds
    $wallDelta = [Math]::Max(
        0.001, $elapsedSeconds - $previousElapsedSeconds)
    $logicalCores = [Math]::Max(
        0.0, ($cpuSeconds - $previousCpuSeconds) / $wallDelta)
    $workingSetMiB = $process.WorkingSet64 / 1MB
    $privateMiB = $process.PrivateMemorySize64 / 1MB
    $availableMiB = $computerInfo.AvailablePhysicalMemory / 1MB

    $samples.Add([pscustomobject]@{
        Timestamp = (Get-Date).ToString('o')
        ElapsedSeconds = [Math]::Round($elapsedSeconds, 3)
        LogicalCores = [Math]::Round($logicalCores, 3)
        NormalizedCpuPercent = [Math]::Round(
            100.0 * $logicalCores / $processorCount, 3)
        WorkingSetMiB = [Math]::Round($workingSetMiB, 3)
        PrivateMiB = [Math]::Round($privateMiB, 3)
        SystemAvailableMiB = [Math]::Round($availableMiB, 3)
        Responding = $process.Responding
    })

    $previousCpuSeconds = $cpuSeconds
    $previousElapsedSeconds = $elapsedSeconds

    if ($StopSignalPath -and (Test-Path -LiteralPath $StopSignalPath)) {
        $reason = 'stop_signal'
        break
    }

    if ($availableMiB -lt $MinimumAvailableMiB) {
        $reason = 'available_memory_limit'
    } elseif ($privateMiB -gt $MaximumPrivateMiB) {
        $reason = 'private_memory_limit'
    } else {
        continue
    }

    if ($StopOnLimit) {
        Stop-Process -Id $TargetProcessId -Force -ErrorAction SilentlyContinue
        $stoppedProcess = $true
    }
    break
}

$stopwatch.Stop()
$samples | Export-Csv -LiteralPath $csvFullPath -NoTypeInformation -Encoding UTF8

$averageCores = 0.0
$maximumCores = 0.0
$averageWorkingSet = 0.0
$maximumWorkingSet = 0.0
$maximumPrivate = 0.0
$minimumAvailable = 0.0
if ($samples.Count -gt 0) {
    $averageCores = ($samples | Measure-Object LogicalCores -Average).Average
    $maximumCores = ($samples | Measure-Object LogicalCores -Maximum).Maximum
    $averageWorkingSet =
        ($samples | Measure-Object WorkingSetMiB -Average).Average
    $maximumWorkingSet =
        ($samples | Measure-Object WorkingSetMiB -Maximum).Maximum
    $maximumPrivate =
        ($samples | Measure-Object PrivateMiB -Maximum).Maximum
    $minimumAvailable =
        ($samples | Measure-Object SystemAvailableMiB -Minimum).Minimum
}

[ordered]@{
    ProcessId = $TargetProcessId
    Started = $started.ToString('o')
    Finished = (Get-Date).ToString('o')
    Reason = $reason
    StoppedProcess = $stoppedProcess
    SampleCount = $samples.Count
    DurationSeconds = [Math]::Round($stopwatch.Elapsed.TotalSeconds, 3)
    AverageLogicalCores = [Math]::Round($averageCores, 3)
    MaximumLogicalCores = [Math]::Round($maximumCores, 3)
    AverageWorkingSetMiB = [Math]::Round($averageWorkingSet, 3)
    MaximumWorkingSetMiB = [Math]::Round($maximumWorkingSet, 3)
    MaximumPrivateMiB = [Math]::Round($maximumPrivate, 3)
    MinimumSystemAvailableMiB = [Math]::Round($minimumAvailable, 3)
} | ConvertTo-Json | Set-Content -LiteralPath $jsonFullPath -Encoding UTF8

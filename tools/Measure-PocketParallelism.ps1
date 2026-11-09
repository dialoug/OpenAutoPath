param(
    [string]$BuildDirectory = 'build',
    [string]$Executable = '',
    [string]$Tag = 'current',
    [string]$OutputDirectory = 'artifacts/parallel-benchmark',
    [string[]]$Samples = @('small', 'multi', 'deep', 'islands', 'many_tools', 'stress'),
    [int[]]$ThreadLimits = @(1, 4, 8),
    [int]$Repetitions = 3
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$buildPath = [IO.Path]::GetFullPath((Join-Path $root $BuildDirectory))
$outputPath = [IO.Path]::GetFullPath((Join-Path $root $OutputDirectory))
[IO.Directory]::CreateDirectory($outputPath) | Out-Null
$samplePath = Join-Path $outputPath 'step'
[IO.Directory]::CreateDirectory($samplePath) | Out-Null
$testList = (& ctest --test-dir $buildPath -C RelWithDebInfo --show-only=json-v1 | Out-String) | ConvertFrom-Json
if ($LASTEXITCODE -ne 0) { throw 'Cannot discover the test runtime.' }
$testCommand = ($testList.tests | Where-Object name -eq 'pocket_automatic_toolpath').command
$runtimePath = ($testCommand | Where-Object { $_.StartsWith('PATH=') }).Substring(5)
if (!$Executable) { $Executable = Join-Path $buildPath 'RelWithDebInfo/tests/oap_test_pocket_automatic_toolpath.exe' }
$Executable = [IO.Path]::GetFullPath($Executable)
$records = [Collections.Generic.List[object]]::new()
foreach ($sample in $Samples) {
    foreach ($limit in $ThreadLimits) {
        # First run ensures all STEP assets exist and is excluded from summaries.
        for ($iteration = 0; $iteration -le $Repetitions; ++$iteration) {
            $start = [Diagnostics.ProcessStartInfo]::new()
            $start.FileName = $Executable
            $start.Arguments = '--parallel-benchmark ' + $sample + ' "' + $samplePath + '"'
            $start.UseShellExecute = $false
            $start.CreateNoWindow = $true
            $start.RedirectStandardOutput = $true
            $start.RedirectStandardError = $true
            $start.EnvironmentVariables['PATH'] = $runtimePath
            $start.EnvironmentVariables['OAP_CAM_MAX_THREADS'] = [string]$limit
            $process = [Diagnostics.Process]::new()
            $process.StartInfo = $start
            $timer = [Diagnostics.Stopwatch]::StartNew()
            $process.Start() | Out-Null
            $stdout = $process.StandardOutput.ReadToEndAsync()
            $stderr = $process.StandardError.ReadToEndAsync()
            $peakThreads = 0
            $peakBytes = 0L
            $cpuMilliseconds = 0.0
            while (!$process.HasExited) {
                if ($timer.Elapsed.TotalMinutes -gt 3) {
                    $process.Kill()
                    throw "Benchmark timeout: $sample, $limit threads"
                }
                try {
                    $process.Refresh()
                    $peakThreads = [Math]::Max($peakThreads, $process.Threads.Count)
                    $peakBytes = [Math]::Max($peakBytes, $process.PeakWorkingSet64)
                    $cpuMilliseconds = $process.TotalProcessorTime.TotalMilliseconds
                } catch [InvalidOperationException] { }
                Start-Sleep -Milliseconds 10
            }
            $process.WaitForExit()
            $outText = $stdout.GetAwaiter().GetResult()
            $errText = $stderr.GetAwaiter().GetResult()
            $logStem = "$Tag-$sample-$limit-$iteration"
            [IO.File]::WriteAllText((Join-Path $outputPath "$logStem.log"), $outText + $errText)
            if ($process.ExitCode -ne 0) { throw "Benchmark failed ($logStem): $errText" }
            $line = @($outText -split '\r?\n' | Where-Object { $_.StartsWith('BENCHMARK ') })[-1]
            if (!$line) { throw "No benchmark record: $logStem" }
            $record = $line.Substring(10) | ConvertFrom-Json
            $record | Add-Member tag $Tag
            $record | Add-Member thread_limit $limit
            $record | Add-Member iteration $iteration
            $record | Add-Member peak_working_set_bytes $peakBytes
            $record | Add-Member sampled_peak_threads $peakThreads
            $record | Add-Member process_cpu_ms $cpuMilliseconds
            $record | Add-Member process_wall_ms $timer.Elapsed.TotalMilliseconds
            $records.Add($record)
            $records | ConvertTo-Json -Depth 4 | Set-Content -Encoding UTF8 (Join-Path $outputPath "$Tag.json")
            if ($iteration -gt 0) {
                Write-Output ("{0} {1} threads={2} run={3} planning={4:N1}ms peak={5:N1}MiB threads~{6}" -f
                    $Tag, $sample, $limit, $iteration, $record.milliseconds, ($peakBytes / 1MB), $peakThreads)
            }
            $process.Dispose()
        }
    }
}

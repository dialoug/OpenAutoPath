param(
    [string]$BuildDirectory = 'build',
    [string]$Configuration = 'RelWithDebInfo',
    [ValidateRange(1, 10)][int]$Repetitions = 1
)
$ErrorActionPreference = 'Stop'
$buildPath = (Resolve-Path -LiteralPath $BuildDirectory).Path
$previousNative = $env:OAP_TEST_NATIVE_GPU
$previousPlugins = $env:QT_PLUGIN_PATH
try {
    $env:OAP_TEST_NATIVE_GPU = '1'
    $inventory = & ctest --test-dir $buildPath -C $Configuration --show-only=json-v1 | ConvertFrom-Json
    if ($LASTEXITCODE -ne 0) { throw 'CTest inventory failed.' }
    $test = $inventory.tests | Where-Object name -eq 'cam_renderer_gpu_material'
    if (!$test) { throw 'Configure and build oap_test_cam_renderer first.' }
    $properties = $test.properties | Where-Object name -eq 'ENVIRONMENT'
    $pluginSetting = $properties.value | Where-Object { $_ -like 'QT_PLUGIN_PATH=*' }
    $env:QT_PLUGIN_PATH = $pluginSetting.Substring('QT_PLUGIN_PATH='.Length)
    Push-Location -LiteralPath $buildPath
    try {
        # Both orders reduce warm-cache/thermal bias. Each process warms 40 frames
        # and measures 120 per backend with the same grid, camera and readback.
        for ($iteration = 1; $iteration -le $Repetitions; ++$iteration) {
            foreach ($mode in @('--gpu-benchmark', '--gpu-benchmark-vulkan-first')) {
                $runArguments = @($test.command | Select-Object -Skip 1)
                $runArguments[$runArguments.Length - 1] = $mode
                $suffix = if ($Repetitions -eq 1) { '' } else { "-run-$iteration" }
                & $test.command[0] @runArguments 2>&1 | Tee-Object -FilePath ($mode.TrimStart('-') + $suffix + '.log')
                if ($LASTEXITCODE -ne 0) { throw "GPU benchmark failed: $mode, run $iteration" }
            }
        }
    } finally { Pop-Location }
} finally {
    $env:OAP_TEST_NATIVE_GPU = $previousNative
    $env:QT_PLUGIN_PATH = $previousPlugins
}

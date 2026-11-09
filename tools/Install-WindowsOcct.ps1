# Install the same release-only SDK used by local builds and Windows CI.
param(
    [string]$Destination = (Join-Path $PSScriptRoot '../external')
)

$ErrorActionPreference = 'Stop'
$Destination = [IO.Path]::GetFullPath($Destination)
New-Item -ItemType Directory -Path $Destination -Force | Out-Null
$packages = @(
    @{
        Name = 'opencascade-7.9.3-vc14-64'
        Sha256 = '2a4f5e187992427674869e89477882d1ace4d67695d4b27173830d93f8cf11be'
        Sentinel = 'win64/vc14/lib/TKernel.lib'
    },
    @{
        Name = '3rdparty-vc14-64'
        Sha256 = 'f25be51af6160a2db74d19a4adc0c08728517822167b01459f8e775927a2a2f6'
        Sentinel = 'tbb-2021.13.0-x64/bin/tbb12.dll'
    }
)

foreach ($package in $packages) {
    $packageRoot = Join-Path $Destination $package.Name
    if (Test-Path -LiteralPath (Join-Path $packageRoot $package.Sentinel)) {
        Write-Host "Using existing $packageRoot"
        continue
    }
    if (Test-Path -LiteralPath $packageRoot) {
        throw "Incomplete SDK at $packageRoot. Select an empty destination or repair it before retrying."
    }
    $archive = Join-Path $Destination ($package.Name + '.zip')
    $url = 'https://github.com/Open-Cascade-SAS/OCCT/releases/download/V7_9_3/' + $package.Name + '.zip'
    if (-not (Test-Path -LiteralPath $archive)) {
        Invoke-WebRequest -Uri $url -OutFile $archive
    }
    if ((Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash -ne $package.Sha256) {
        throw "SHA256 mismatch for $archive"
    }
    Expand-Archive -LiteralPath $archive -DestinationPath $Destination
    if (-not (Test-Path -LiteralPath (Join-Path $packageRoot $package.Sentinel))) {
        throw "Unexpected SDK layout after extracting $archive"
    }
}

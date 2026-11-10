# Rebuild the existing FFmpeg ABI from the pinned upstream source, without
# modifying the vendor SDK. GNU Make is a build tool, not a runtime dependency.
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$SourceArchive,
    [Parameter(Mandatory)][string]$OutputDir,
    [Parameter(Mandatory)][string]$MakePackage,
    [string]$VisualStudioRoot,
    [ValidateRange(1, 8)][int]$Parallel = 4
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$root = Split-Path $PSScriptRoot -Parent
$manifest = Get-Content (Join-Path $root 'licenses/windows-runtime.json') -Raw -Encoding UTF8 | ConvertFrom-Json
$source = $manifest.sources | Where-Object component -eq 'ffmpeg'
$archive = (Resolve-Path -LiteralPath $SourceArchive).Path
$makeArchive = (Resolve-Path -LiteralPath $MakePackage).Path
$makeHash = '2408af61717dae87b00c855b132769a125c708907fc94a46bb16dae076113e5c'
if ((Get-FileHash $archive).Hash -ne $source.sha256) { throw 'FFmpeg source archive SHA256 mismatch' }
if ((Get-FileHash $makeArchive).Hash -ne $makeHash) { throw 'GNU Make package SHA256 mismatch' }
$output = [IO.Path]::GetFullPath($OutputDir)
if (Test-Path -LiteralPath $output) { throw 'OutputDir must not exist' }
$gitRoot = Split-Path (Split-Path (Get-Command git -ErrorAction Stop).Source -Parent) -Parent
$bash = Join-Path $gitRoot 'bin/bash.exe'
if (-not (Test-Path $bash)) { throw 'Git for Windows Bash is required' }
if (-not $VisualStudioRoot) {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
    $VisualStudioRoot = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if ($LASTEXITCODE -ne 0 -or -not $VisualStudioRoot) { throw 'Visual Studio C++ tools were not found' }
}
$vcvars = Join-Path $VisualStudioRoot 'VC/Auxiliary/Build/vcvars64.bat'
if ($vcvars -match '["%\r\n]') { throw 'Unsupported Visual Studio path' }
New-Item -ItemType Directory -Path $output | Out-Null
$work = Join-Path $output 'source'
$makeRoot = Join-Path $output 'build-tools/make'
New-Item -ItemType Directory -Path $work, $makeRoot -Force | Out-Null
& cmake -E chdir $work cmake -E tar xf $archive
if ($LASTEXITCODE -ne 0) { throw 'FFmpeg extraction failed' }
& cmake -E chdir $makeRoot cmake -E tar xf $makeArchive
if ($LASTEXITCODE -ne 0) { throw 'GNU Make extraction failed' }
$script = @'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/source/ffmpeg-3.3.4"
./configure --toolchain=msvc --arch=amd64 --enable-shared --disable-static --disable-asm --disable-yasm --disable-swresample --disable-avfilter --disable-avdevice --disable-ffmpeg --disable-ffplay --disable-ffserver --disable-doc --prefix="$(pwd)/../.."
make -j__JOBS__
make install
'@
$scriptPath = Join-Path $output 'build.sh'
[IO.File]::WriteAllText($scriptPath, $script.Replace('__JOBS__', "$Parallel").Replace("`r`n", "`n"), [Text.UTF8Encoding]::new($false))
$names = @('PATH', 'INCLUDE', 'LIB', 'LIBPATH', 'VCToolsVersion', 'VCINSTALLDIR', 'WindowsSDKVersion', 'CL', '_CL_')
$saved = @{}
foreach ($name in $names) { $saved[$name] = [Environment]::GetEnvironmentVariable($name, 'Process') }
try {
    # Capture only the required compiler variables; never log the full environment.
    $vars = & cmd /d /s /c ('"' + $vcvars + '" >nul && set')
    if ($LASTEXITCODE -ne 0) { throw 'Visual Studio environment setup failed' }
    foreach ($line in $vars) {
        if ($line -match '^(PATH|INCLUDE|LIB|LIBPATH|VCToolsVersion|VCINSTALLDIR|WindowsSDKVersion)=(.*)$') {
            [Environment]::SetEnvironmentVariable($matches[1], $matches[2], 'Process')
        }
    }
    $env:CL = $null
    $env:_CL_ = $null
    $compiler = $env:VCToolsVersion
    $sdk = $env:WindowsSDKVersion
    $env:PATH = (Join-Path $makeRoot 'usr/bin') + ';' + $env:PATH + ';' + (Join-Path $gitRoot 'usr/bin')
    & $bash --noprofile --norc $scriptPath.Replace('\', '/') > (Join-Path $output 'build.log') 2>&1
    if ($LASTEXITCODE -ne 0) { throw "FFmpeg build failed; see $output/build.log" }
} finally {
    foreach ($name in $names) { [Environment]::SetEnvironmentVariable($name, $saved[$name], 'Process') }
}
& (Join-Path $root 'tools/tests/Test-FFmpegRuntime.ps1') -BinDir (Join-Path $output 'bin') -OutputDir (Join-Path $output 'smoke')
$outputs = @()
foreach ($file in @('avcodec-57.dll', 'avformat-57.dll', 'avutil-55.dll', 'swscale-4.dll')) {
    $outputs += @{ file = $file; sha256 = (Get-FileHash (Join-Path $output "bin/$file")).Hash.ToLowerInvariant() }
}
$record = [ordered]@{
    schemaVersion = 1; component = 'ffmpeg'; version = '3.3.4'
    sourceSha256 = $source.sha256; sourceUrl = $source.url
    recipe = 'tools/Build-WindowsFFmpeg.ps1'
    recipeSha256 = (Get-FileHash $PSCommandPath).Hash.ToLowerInvariant()
    patches = @()
    compilerVersion = $compiler; windowsSdkVersion = $sdk
    gitVersion = (& git --version)
    makePackage = 'make-4.4.1-2-x86_64.pkg.tar.zst'; makePackageSha256 = $makeHash
    configureArguments = @('--toolchain=msvc', '--arch=amd64', '--enable-shared', '--disable-static',
        '--disable-asm', '--disable-yasm', '--disable-swresample', '--disable-avfilter', '--disable-avdevice',
        '--disable-ffmpeg', '--disable-ffplay', '--disable-ffserver', '--disable-doc', '--prefix=<output>')
    outputs = $outputs
    smokeTest = 'PPM demux/decode, packet SHA256 and RGB/BGR pixel conversion'
}
$record | ConvertTo-Json -Depth 8 | Set-Content (Join-Path $output 'build-record.json') -Encoding UTF8
Copy-Item (Join-Path $work 'ffmpeg-3.3.4/config.h') (Join-Path $output 'config.h')
Copy-Item (Join-Path $work 'ffmpeg-3.3.4/config.mak') (Join-Path $output 'config.mak')
Write-Host "Built and tested FFmpeg 3.3.4: $output"

# Validate deployed binaries, then include license evidence and dependency sources.
# No binaries are rebuilt or replaced by this script.
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Stage,
    [Parameter(Mandatory)][string]$BuildDir,
    [string]$SourceCache,
    [switch]$Offline,
    [switch]$VerifyOnly,
    [switch]$RequireSourceBuilds
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$sourceRoot = Split-Path $PSScriptRoot -Parent
$stagePath = (Resolve-Path -LiteralPath $Stage).Path
$buildPath = (Resolve-Path -LiteralPath $BuildDir).Path
$licensePath = Join-Path $stagePath 'share/licenses/OpenAutoPath'
$manifestPath = Join-Path $sourceRoot 'licenses/windows-runtime.json'
$manifest = Get-Content -LiteralPath $manifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
if ($manifest.schemaVersion -ne 1) { throw 'Unsupported runtime license manifest' }

# Compare every bundled notice, not only a directory or a marker file.
foreach ($file in Get-ChildItem -LiteralPath (Join-Path $sourceRoot 'licenses/runtime') -Recurse -File) {
    $relative = $file.FullName.Substring((Join-Path $sourceRoot 'licenses').Length + 1)
    $installed = Join-Path $licensePath $relative
    if (-not (Test-Path -LiteralPath $installed -PathType Leaf)) {
        throw "Missing third-party notice: $relative"
    }
    if ((Get-FileHash -LiteralPath $file.FullName).Hash -ne (Get-FileHash -LiteralPath $installed).Hash) {
        throw "Changed third-party notice: $relative"
    }
}

$known = @{}
foreach ($component in $manifest.components) {
    if (-not (Test-Path -LiteralPath (Join-Path $licensePath $component.notice) -PathType Leaf)) {
        throw "Missing component notice: $($component.notice)"
    }
    foreach ($file in $component.files) {
        if ($known.ContainsKey($file.path)) { throw "Duplicate runtime mapping: $($file.path)" }
        $known[$file.path] = @{ Component = $component; File = $file }
    }
}
$inventory = @()
$sourceBuildRecords = @()
foreach ($file in Get-ChildItem -LiteralPath (Join-Path $stagePath 'bin') -Recurse -File) {
    if ($file.Extension -notin @('.dll', '.exe')) { continue }
    $relative = $file.FullName.Substring($stagePath.Length + 1).Replace('\', '/')
    if ($relative -eq 'bin/OpenAutoPathApp.exe') { continue }
    if ($file.Name -match '^(avcodec|avformat|avutil|swscale|avdevice|avfilter|swresample)-[0-9]+\.dll$') {
        throw "FFmpeg runtime is disabled; remove stale DLL: $relative"
    }
    if (-not $known.ContainsKey($relative)) {
        throw "Unreviewed runtime file: $relative. Update licenses/windows-runtime.json and its notices."
    }
    $entry = $known[$relative]
    $hash = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($entry.File.PSObject.Properties['sha256'] -and $hash -ne $entry.File.sha256) {
        if ($entry.Component.id -eq 'occt') {
            $id = $entry.Component.id
            $recordPath = Join-Path $licensePath "build-evidence/$id-build-record.json"
            if (-not (Test-Path -LiteralPath $recordPath)) { throw "Changed $id runtime has no source build record" }
            $record = Get-Content -LiteralPath $recordPath -Raw -Encoding UTF8 | ConvertFrom-Json
            $source = $manifest.sources | Where-Object component -eq $id
            $recipe = 'tools/Build-WindowsOCCT.ps1'
            $output = @($record.outputs | Where-Object file -eq $file.Name)
            if ($record.schemaVersion -ne 1 -or $record.component -ne $id -or
                $record.version -ne $entry.Component.version -or $record.sourceSha256 -ne $source.sha256 -or
                $record.recipe -ne $recipe -or $record.recipeSha256 -ne (Get-FileHash (Join-Path $sourceRoot $recipe)).Hash -or
                $output.Count -ne 1 -or $output[0].sha256 -ne $hash -or @($record.patches).Count -ne 0) {
                throw "$id source build record does not match the runtime/source/recipe"
            }
            if (@($sourceBuildRecords | Where-Object component -eq $id).Count -eq 0) { $sourceBuildRecords += $record }
        } elseif ($entry.Component.id -ne 'freeimage') {
            throw "Runtime changed since license review: $relative. Review the new SDK before updating its hash."
        } else {
        # A source build is accepted only with the pinned recipe, source archive,
        # compatibility patch and a record matching the actual deployed DLL.
        $recordPath = Join-Path $licensePath 'build-evidence/FreeImage-build-record.json'
        if (-not (Test-Path -LiteralPath $recordPath)) { throw 'Changed FreeImage runtime has no source build record' }
        $record = Get-Content -LiteralPath $recordPath -Raw -Encoding UTF8 | ConvertFrom-Json
        $fiSource = $manifest.sources | Where-Object component -eq 'freeimage'
        if ($record.schemaVersion -ne 1 -or $record.component -ne 'freeimage' -or
            $record.version -ne '3.18.0' -or $record.sourceSha256 -ne $fiSource.sha256 -or
            $record.dllSha256 -ne $hash -or
            $record.recipe -ne 'tools/Build-WindowsFreeImage.ps1' -or
            $record.patch -ne 'tools/dependencies/freeimage-openexr-modern-cxx.patch' -or
            $record.recipeSha256 -ne (Get-FileHash (Join-Path $sourceRoot $record.recipe)).Hash -or
            $record.patchSha256 -ne (Get-FileHash (Join-Path $sourceRoot $record.patch)).Hash) {
            throw 'FreeImage source build record does not match the runtime/source/recipe/patch'
        }
        $sourceBuildRecords += $record
        }
    }
    # DXC is obtained from the Windows SDK, not Qt. Signed SDK and GitHub
    # distributions can have different hashes for the same product version.
    if ($entry.Component.id -eq 'dxc' -and
        $file.VersionInfo.ProductVersion -ne $entry.Component.version) {
        throw "Unreviewed DirectX shader compiler version: $($file.VersionInfo.ProductVersion)"
    }
    $inventory += [ordered]@{
        file = $relative
        sha256 = $hash
        fileVersion = $file.VersionInfo.FileVersion
        component = $entry.Component.id
        version = $entry.Component.version
        license = $entry.Component.license
        notice = 'share/licenses/OpenAutoPath/' + $entry.Component.notice
    }
}
# Optional Qt plugins may be absent; the indispensable runtime files are checked
# by Package-WindowsRelease.ps1. Never allow an empty inventory to pass.
if ($inventory.Count -eq 0) { throw 'No runtime binaries found' }
if ($RequireSourceBuilds) {
    foreach ($id in @('freeimage', 'occt')) {
        if (@($sourceBuildRecords | Where-Object component -eq $id).Count -ne 1) {
            throw "Source-built release requires a verified $id build record"
        }
        $component = $manifest.components | Where-Object id -eq $id
        foreach ($requiredFile in $component.files) {
            if (@($inventory | Where-Object file -eq $requiredFile.path).Count -ne 1) {
                throw "Source-built release is missing runtime: $($requiredFile.path)"
            }
        }
    }
}
$occtRecord = $sourceBuildRecords | Where-Object component -eq 'occt'
if ($occtRecord) {
    if (-not $occtRecord.PSObject.Properties['ffmpegEnabled'] -or
        $occtRecord.ffmpegEnabled -isnot [bool] -or $occtRecord.ffmpegEnabled -or
        'FFmpeg' -in $occtRecord.features) {
        throw 'OCCT source build must disable FFmpeg'
    }
    foreach ($dependency in $occtRecord.dependencyBinaries) {
        $deployed = @($inventory | Where-Object { [IO.Path]::GetFileName($_.file) -eq $dependency.file })
        if ($deployed.Count -ne 1 -or $deployed[0].sha256 -ne $dependency.sha256) {
            throw "OCCT source build dependency mismatch: $($dependency.file)"
        }
    }
    if ($occtRecord.memoryManager -eq 'NATIVE' -and @($inventory | Where-Object component -eq 'jemalloc').Count) {
        throw 'Native-allocator OCCT package contains a stale jemalloc.dll; use a clean runtime directory'
    }
}
$oclNotice = Get-Content -LiteralPath (Join-Path $licensePath 'OpenCAMLib-NOTICE.txt') -Raw
$ocl = $manifest.components | Where-Object id -eq 'opencamlib'
if (-not $oclNotice.Contains($ocl.version)) { throw 'OpenCAMLib source revision does not match the license manifest' }
if ($VerifyOnly) {
    Write-Output "Runtime license verification passed ($($inventory.Count) files)."
    return
}

$cache = Get-Content -LiteralPath (Join-Path $buildPath 'CMakeCache.txt') -Raw
$qtMatch = [regex]::Match($cache, '(?m)^Qt6_DIR:PATH=(.+)\r?$')
if (-not $qtMatch.Success) { throw 'Qt6_DIR is missing from the CMake cache' }
$qtRoot = [IO.Path]::GetFullPath((Join-Path $qtMatch.Groups[1].Value.Trim() '../../..'))
$evidencePath = Join-Path $licensePath 'build-evidence'
New-Item -ItemType Directory -Force -Path $evidencePath | Out-Null
foreach ($relative in @('sbom/qtbase-6.11.1.spdx.json', 'config_qtbase.opt', 'config_qtbase.summary')) {
    $inputFile = Join-Path $qtRoot $relative
    if (-not (Test-Path -LiteralPath $inputFile -PathType Leaf)) { throw "Missing Qt SDK evidence: $relative" }
    Copy-Item -LiteralPath $inputFile -Destination $evidencePath
}

if (-not $SourceCache) { $SourceCache = Join-Path $buildPath 'release-source-cache' }
$SourceCache = [IO.Path]::GetFullPath($SourceCache)
New-Item -ItemType Directory -Force -Path $SourceCache | Out-Null
$sourcesPath = Join-Path $stagePath 'share/sources/OpenAutoPath'
New-Item -ItemType Directory -Force -Path $sourcesPath | Out-Null
foreach ($source in $manifest.sources) {
    # Manifest filenames must remain single filenames within our cache/stage.
    if ([IO.Path]::GetFileName($source.file) -ne $source.file -or $source.file -match '[:\\/]') {
        throw "Invalid source archive name: $($source.file)"
    }
    $archive = Join-Path $SourceCache $source.file
    if (-not (Test-Path -LiteralPath $archive -PathType Leaf)) {
        if ($Offline) { throw "Source archive unavailable offline: $($source.file)" }
        $partial = $archive + '.download'
        for ($attempt = 1; $attempt -le 3; $attempt++) {
            try {
                Write-Host "Downloading dependency source: $($source.component)"
                Invoke-WebRequest -UseBasicParsing -Uri $source.url -OutFile $partial -TimeoutSec 180
                if ((Get-FileHash -LiteralPath $partial).Hash -ne $source.sha256) {
                    throw "Source archive SHA256 mismatch: $($source.file)"
                }
                Move-Item -LiteralPath $partial -Destination $archive
                break
            } catch {
                if ($attempt -eq 3) { throw }
                Start-Sleep -Seconds $attempt
            }
        }
    }
    if ((Get-FileHash -LiteralPath $archive).Hash -ne $source.sha256) {
        throw "Cached source archive SHA256 mismatch: $($source.file)"
    }
    Copy-Item -LiteralPath $archive -Destination $sourcesPath
}
Copy-Item -LiteralPath (Join-Path $sourceRoot 'licenses/DEPENDENCY_SOURCES.md') -Destination $sourcesPath
Copy-Item -LiteralPath $manifestPath -Destination $licensePath
Copy-Item -LiteralPath $manifestPath -Destination $sourcesPath
$support = Join-Path $sourcesPath 'OpenAutoPath-build-support'
foreach ($relative in @('LICENSE', 'licenses/windows-runtime.json', 'tools/Build-WindowsFreeImage.ps1',
        'tools/tests/Test-FreeImageRuntime.ps1', 'tools/dependencies/freeimage-openexr-modern-cxx.patch',
        'tools/Build-WindowsOCCT.ps1',
        'tools/Install-WindowsOcct.ps1')) {
    $destination = Join-Path $support $relative
    New-Item -ItemType Directory -Force -Path (Split-Path $destination -Parent) | Out-Null
    Copy-Item -LiteralPath (Join-Path $sourceRoot $relative) -Destination $destination
}
$result = [ordered]@{
    schemaVersion = 1
    scope = 'Deployed third-party DLLs and Microsoft redistributable; excludes project executables'
    binaries = @($inventory | Sort-Object { $_.file })
    sources = @($manifest.sources)
    sourceBuilds = @($sourceBuildRecords)
    sourceDirectory = 'share/sources/OpenAutoPath'
}
$result | ConvertTo-Json -Depth 10 | Set-Content -LiteralPath (Join-Path $licensePath 'runtime-inventory.json') -Encoding UTF8
Write-Output "Collected licenses, build evidence and $($manifest.sources.Count) source archives for $($inventory.Count) runtime files."

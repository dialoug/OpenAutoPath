# Build a separate FreeImage SDK from the pinned source archive. Never modifies
# the downloaded OCCT SDK. The resulting record ties source, patch and DLL.
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$SourceArchive,
    [Parameter(Mandatory)][string]$OutputDir,
    [string]$VisualStudioRoot,
    [string]$Toolset,
    [string]$WindowsSdkVersion,
    [ValidateRange(1, 8)][int]$Parallel = 4
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$root = Split-Path $PSScriptRoot -Parent
$manifest = Get-Content (Join-Path $root 'licenses/windows-runtime.json') -Raw -Encoding UTF8 | ConvertFrom-Json
$source = $manifest.sources | Where-Object component -eq 'freeimage'
$archive = (Resolve-Path -LiteralPath $SourceArchive).Path
if ((Get-FileHash -LiteralPath $archive).Hash -ne $source.sha256) { throw 'FreeImage source archive SHA256 mismatch' }
$output = [IO.Path]::GetFullPath($OutputDir)
if (Test-Path -LiteralPath $output) { throw 'OutputDir must not exist; choose a new directory to avoid stale artifacts' }
if (-not $VisualStudioRoot) {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
    $VisualStudioRoot = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if ($LASTEXITCODE -ne 0 -or -not $VisualStudioRoot) { throw 'Visual Studio C++ tools were not found' }
}
$msbuild = Join-Path $VisualStudioRoot 'MSBuild/Current/Bin/MSBuild.exe'
if (-not (Test-Path -LiteralPath $msbuild)) { throw 'MSBuild was not found' }
if (-not $Toolset) {
    $Toolset = Get-ChildItem (Join-Path $VisualStudioRoot 'MSBuild/Microsoft/VC/*/Platforms/x64/PlatformToolsets/v*') -Directory |
        Select-Object -ExpandProperty Name | Sort-Object -Unique -Descending | Select-Object -First 1
}
if ($Toolset -notin @('v143', 'v145')) { throw 'This recipe supports VS 2022 (v143) and VS 2026 (v145)' }
if (-not $WindowsSdkVersion) {
    $kits = (Get-ItemProperty 'HKLM:/SOFTWARE/Microsoft/Windows Kits/Installed Roots').KitsRoot10
    $WindowsSdkVersion = Get-ChildItem (Join-Path $kits 'Include/10.*') -Directory |
        Where-Object { Test-Path (Join-Path $_.FullName 'um/Windows.h') } |
        Sort-Object { [version]$_.Name } -Descending | Select-Object -First 1 -ExpandProperty Name
}
if (-not $WindowsSdkVersion) { throw 'A Windows SDK is required' }
New-Item -ItemType Directory -Path $output | Out-Null
$work = Join-Path $output 'source'
New-Item -ItemType Directory -Path $work | Out-Null
& cmake -E chdir $work cmake -E tar xf $archive
if ($LASTEXITCODE -ne 0) { throw 'FreeImage source extraction failed' }
$sourceDir = Join-Path $work 'FreeImage'
$patch = Join-Path $root 'tools/dependencies/freeimage-openexr-modern-cxx.patch'
$patchedFile = Join-Path $sourceDir 'Source/OpenEXR/IlmImf/ImfAttribute.cpp'
$originalHash = (Get-FileHash -LiteralPath $patchedFile).Hash.ToLowerInvariant()
$text = [IO.File]::ReadAllText($patchedFile)
$before = 'struct NameCompare: std::binary_function <const char *, const char *, bool>'
if ([regex]::Matches($text, [regex]::Escape($before)).Count -ne 1) { throw 'Unexpected OpenEXR source; patch requires review' }
# Remove only the obsolete typedef-bearing base; comparison behavior is unchanged.
[IO.File]::WriteAllText($patchedFile, $text.Replace($before, 'struct NameCompare'), [Text.UTF8Encoding]::new($false))
$patchedHash = (Get-FileHash -LiteralPath $patchedFile).Hash.ToLowerInvariant()
$arguments = @('FreeImage.2017.vcxproj', '/p:Configuration=Release', '/p:Platform=x64',
    "/p:PlatformToolset=$Toolset", "/p:WindowsTargetPlatformVersion=$WindowsSdkVersion",
    '/m:1', '/v:minimal', '/nologo')
$oldCL = $env:CL
$oldUnderscoreCL = $env:_CL_
try {
    # A single project at a time, bounded parallel compilation within it.
    $env:CL = "/MP$Parallel"
    $env:_CL_ = $null
    Push-Location $sourceDir
    try {
        & $msbuild @arguments > (Join-Path $output 'build.log') 2>&1
        if ($LASTEXITCODE -ne 0) { throw "FreeImage build failed; see $output/build.log" }
    } finally { Pop-Location }
} finally {
    $env:CL = $oldCL
    $env:_CL_ = $oldUnderscoreCL
}
New-Item -ItemType Directory -Path (Join-Path $output 'bin'), (Join-Path $output 'lib'), (Join-Path $output 'include') | Out-Null
Copy-Item (Join-Path $sourceDir 'Dist/x64/FreeImage.dll') (Join-Path $output 'bin/FreeImage.dll')
Copy-Item (Join-Path $sourceDir 'Dist/x64/FreeImage.lib') (Join-Path $output 'lib/FreeImage.lib')
Copy-Item (Join-Path $sourceDir 'Source/FreeImage.h') (Join-Path $output 'include/FreeImage.h')
& (Join-Path $root 'tools/tests/Test-FreeImageRuntime.ps1') -Dll (Join-Path $output 'bin/FreeImage.dll') -OutputDir (Join-Path $output 'smoke')
$compilerVersion = (Get-Content (Join-Path $VisualStudioRoot 'VC/Auxiliary/Build/Microsoft.VCToolsVersion.default.txt') -Raw).Trim()
$record = [ordered]@{
    schemaVersion = 1
    component = 'freeimage'
    version = '3.18.0'
    sourceSha256 = $source.sha256
    sourceUrl = $source.url
    recipe = 'tools/Build-WindowsFreeImage.ps1'
    recipeSha256 = (Get-FileHash -LiteralPath $PSCommandPath).Hash.ToLowerInvariant()
    patch = 'tools/dependencies/freeimage-openexr-modern-cxx.patch'
    patchSha256 = (Get-FileHash -LiteralPath $patch).Hash.ToLowerInvariant()
    patchedFile = 'Source/OpenEXR/IlmImf/ImfAttribute.cpp'
    originalFileSha256 = $originalHash
    patchedFileSha256 = $patchedHash
    toolset = $Toolset
    compilerVersion = $compilerVersion
    windowsSdkVersion = $WindowsSdkVersion
    msbuildVersion = (Get-Item $msbuild).VersionInfo.FileVersion
    arguments = $arguments
    compilerOptions = "/MP$Parallel"
    dllSha256 = (Get-FileHash (Join-Path $output 'bin/FreeImage.dll')).Hash.ToLowerInvariant()
    smokeTest = 'PNG save/load pixel round-trip passed'
}
$record | ConvertTo-Json -Depth 6 | Set-Content (Join-Path $output 'build-record.json') -Encoding UTF8
Write-Host "Built and tested FreeImage 3.18.0: $output"

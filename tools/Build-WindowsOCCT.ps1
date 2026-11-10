# Build OCCT's application toolkits from the pinned upstream source archive.
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$SourceArchive,
    [Parameter(Mandatory)][string]$OutputDir,
    [Parameter(Mandatory)][string]$ThirdPartyRoot,
    [Parameter(Mandatory)][string]$FreeImageRoot,
    [ValidateSet('Visual Studio 17 2022', 'Visual Studio 18 2026')]
    [string]$Generator = 'Visual Studio 17 2022',
    [ValidateRange(1, 8)][int]$Parallel = 4,
    [switch]$ReuseExistingBuild
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$root = Split-Path $PSScriptRoot -Parent
$manifest = Get-Content (Join-Path $root 'licenses/windows-runtime.json') -Raw -Encoding UTF8 | ConvertFrom-Json
$source = $manifest.sources | Where-Object component -eq 'occt'
$archive = (Resolve-Path -LiteralPath $SourceArchive).Path
if ((Get-FileHash $archive).Hash -ne $source.sha256) { throw 'OCCT source archive SHA256 mismatch' }
$output = [IO.Path]::GetFullPath($OutputDir)
$thirdparty = (Resolve-Path -LiteralPath $ThirdPartyRoot).Path
$freeimage = (Resolve-Path -LiteralPath $FreeImageRoot).Path
$sourceDir = Join-Path $output 'OCCT-7_9_3'
$build = Join-Path $output 'build'
$install = Join-Path $output 'install'
if (Test-Path -LiteralPath $output) {
    if (-not $ReuseExistingBuild) { throw 'OutputDir exists; use a new directory or explicitly request a verified incremental build' }
    # Check the entire existing source tree against a fresh extraction, not a
    # mutable marker file. Changed or extra source files require a fresh build.
    $verification = Join-Path $output ('source-verification-' + [guid]::NewGuid().ToString('N'))
    New-Item -ItemType Directory -Path $verification | Out-Null
    & cmake -E chdir $verification cmake -E tar xf $archive
    if ($LASTEXITCODE -ne 0) { throw 'OCCT verification extraction failed' }
    $reference = Join-Path $verification 'OCCT-7_9_3'
    # Keep all-file verification, but avoid tens of thousands of PowerShell
    # pipeline invocations. Hash directly from streams with a bounded buffer.
    $verifier = Add-Type -PassThru -TypeDefinition @'
using System;
using System.IO;
using System.Security.Cryptography;
public static class OCCTSourceVerifier {
    public static int Verify(string reference,string existing) {
        string[] files=Directory.GetFiles(reference,"*",SearchOption.AllDirectories);
        if(files.Length!=Directory.GetFiles(existing,"*",SearchOption.AllDirectories).Length)
            throw new Exception("Existing OCCT source tree has missing or extra files");
        using(var sha=SHA256.Create()) {
            int count=0;
            foreach(string file in files) {
                string relative=file.Substring(reference.Length+1);
                string other=Path.Combine(existing,relative);
                if(!File.Exists(other)) throw new Exception("Missing OCCT source file: "+relative);
                byte[] a,b;
                using(var stream=File.OpenRead(file)) a=sha.ComputeHash(stream);
                using(var stream=File.OpenRead(other)) b=sha.ComputeHash(stream);
                if(Convert.ToBase64String(a)!=Convert.ToBase64String(b))
                    throw new Exception("Changed OCCT source file: "+relative);
                if(++count%5000==0) Console.WriteLine("Verified {0}/{1} OCCT source files",count,files.Length);
            }
        }
        return files.Length;
    }
}
'@
    $verifiedCount = $verifier::Verify($reference, $sourceDir)
    Write-Host "Verified $verifiedCount pristine OCCT source files before incremental build."
} else {
    New-Item -ItemType Directory -Path $output | Out-Null
    & cmake -E chdir $output cmake -E tar xf $archive
    if ($LASTEXITCODE -ne 0) { throw 'OCCT extraction failed' }
}
# Match the application's toolkit closure, rather than building DRAW, samples,
# VTK integrations or unused exchange formats. Keep image/VR support, no video.
$toolkits = 'TKDESTEP TKDECascade TKBinXCAF TKStd TKStdL TKMesh TKBO TKBool TKFillet TKOffset TKPrim TKV3d TKOpenGl TKBinTObj TKHLR TKTObj TKVCAF TKXml TKXmlL TKXmlTObj TKXmlXCAF'
$options = @('-S', $sourceDir, '-B', $build, '-G', $Generator, '-A', 'x64',
    "-DCMAKE_INSTALL_PREFIX=$install", "-D3RDPARTY_DIR=$thirdparty", "-D3RDPARTY_FREEIMAGE_DIR=$freeimage",
    "-DBUILD_ADDITIONAL_TOOLKITS=$toolkits", '-DBUILD_MODULE_FoundationClasses=OFF',
    '-DBUILD_MODULE_ModelingData=OFF', '-DBUILD_MODULE_ModelingAlgorithms=OFF',
    '-DBUILD_MODULE_Visualization=OFF', '-DBUILD_MODULE_ApplicationFramework=OFF',
    '-DBUILD_MODULE_DataExchange=OFF', '-DBUILD_MODULE_Draw=OFF', '-DBUILD_MODULE_DETools=OFF',
    '-DBUILD_DOC_Overview=OFF', '-DBUILD_USE_PCH=ON', '-DUSE_TBB=ON', '-DUSE_FREEIMAGE=ON',
    '-DUSE_FREETYPE=ON', '-DUSE_OPENVR=ON', '-DUSE_FFMPEG=OFF', '-DUSE_MMGR_TYPE=NATIVE',
    '-DINSTALL_DIR_BIN=win64/vc14/bin', '-DINSTALL_DIR_LIB=win64/vc14/lib', '-DINSTALL_DIR_INCLUDE=inc')
$oldErrorAction = $ErrorActionPreference
try {
    $ErrorActionPreference = 'Continue'
    & cmake @options > (Join-Path $output 'configure.log') 2>&1
    $configureExit = $LASTEXITCODE
} finally { $ErrorActionPreference = $oldErrorAction }
if ($configureExit -ne 0) { throw "OCCT configuration failed; see $output/configure.log" }
$oldCL = $env:CL
$oldUnderscoreCL = $env:_CL_
try {
    $env:CL = $null
    # One MSBuild project at a time; limit the /MP compiler workers explicitly.
    $env:_CL_ = "/MP$Parallel"
    & cmake --build $build --config Release --parallel 1 > (Join-Path $output 'build.log') 2>&1
    if ($LASTEXITCODE -ne 0) { throw "OCCT build failed; see $output/build.log" }
} finally {
    $env:CL = $oldCL
    $env:_CL_ = $oldUnderscoreCL
}
& cmake --install $build --config Release > (Join-Path $output 'install.log') 2>&1
if ($LASTEXITCODE -ne 0) { throw "OCCT install failed; see $output/install.log" }
$outputs = @(Get-ChildItem (Join-Path $install 'win64/vc14/bin') -Filter '*.dll' | Sort-Object Name | ForEach-Object {
    @{ file = $_.Name; sha256 = (Get-FileHash -LiteralPath $_.FullName).Hash.ToLowerInvariant() }
})
if ($outputs.Count -lt 39) { throw 'Incomplete OCCT toolkit installation' }
$dependencies = @()
foreach ($filePath in @((Join-Path $thirdparty 'freetype-2.13.3-x64/bin/freetype.dll'),
        (Join-Path $thirdparty 'tbb-2021.13.0-x64/bin/tbb12.dll'),
        (Join-Path $thirdparty 'openvr-1.14.15-64/bin/win64/openvr_api.dll'),
        (Join-Path $freeimage 'bin/FreeImage.dll'))) {
    $file = Get-Item -LiteralPath $filePath
    $dependencies += @{ file = $file.Name; sha256 = (Get-FileHash -LiteralPath $file.FullName).Hash.ToLowerInvariant() }
}
$cache = Get-Content (Join-Path $build 'CMakeCache.txt') -Raw
$instance = [regex]::Match($cache, '(?m)^CMAKE_GENERATOR_INSTANCE:INTERNAL=(.+)\r?$').Groups[1].Value.Trim()
$compilerVersion = (Get-Content (Join-Path $instance 'VC/Auxiliary/Build/Microsoft.VCToolsVersion.default.txt') -Raw).Trim()
$record = [ordered]@{
    schemaVersion = 1; component = 'occt'; version = '7.9.3'
    sourceSha256 = $source.sha256; sourceUrl = $source.url
    recipe = 'tools/Build-WindowsOCCT.ps1'; recipeSha256 = (Get-FileHash $PSCommandPath).Hash.ToLowerInvariant()
    patches = @(); generator = $Generator; compilerVersion = $compilerVersion
    incrementalBuild = [bool]$ReuseExistingBuild
    toolkits = $toolkits; configuration = 'Release'; memoryManager = 'NATIVE'
    ffmpegEnabled = $false
    features = @('TBB', 'FreeImage', 'FreeType', 'OpenVR')
    dependencyBinaries = $dependencies
    outputs = $outputs
}
$record | ConvertTo-Json -Depth 8 | Set-Content (Join-Path $install 'build-record.json') -Encoding UTF8
Write-Host "Built $($outputs.Count) OCCT 7.9.3 toolkits: $install"

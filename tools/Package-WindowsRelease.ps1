# Package the same desktop binaries that passed CTest. No rebuild or source changes.
# 打包已通过测试的桌面二进制；本脚本不重新编译，也不修改源码。
# BuildDir：已有构建目录；Version：版本名；Configuration：已构建的配置；
# OutputDir：ZIP 与 SHA-256 校验文件的输出目录。
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$BuildDir,
    [Parameter(Mandatory)]
    [ValidatePattern('^(v[0-9]+\.[0-9]+\.[0-9]+(-[0-9A-Za-z]+([.-][0-9A-Za-z]+)*)?|preview-[0-9a-f]{7,40})$')]
    [string]$Version,
    [ValidateSet('RelWithDebInfo', 'Release', 'MinSizeRel')]
    [string]$Configuration = 'RelWithDebInfo',
    [string]$OutputDir = 'artifacts/release',
    [string]$SourceCache,
    [switch]$OfflineSources
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$sourceRoot = Split-Path $PSScriptRoot -Parent
$buildPath = (Resolve-Path -LiteralPath $BuildDir).Path
$cache = Get-Content -LiteralPath (Join-Path $buildPath 'CMakeCache.txt') -Raw
# 必须启用 GUI 和运行库部署，否则不能生成可分发的桌面包。
foreach ($option in @('OAP_BUILD_GUI', 'OAP_DEPLOY_RUNTIME')) {
    if ($cache -notmatch "(?m)^${option}:BOOL=(ON|TRUE|1)\r?$") {
        throw "Packaging requires $option=ON in $buildPath"
    }
}
$commit = & git -C $sourceRoot rev-parse HEAD
if ($LASTEXITCODE -ne 0) { throw 'Cannot determine source commit' }
$packageName = "OpenAutoPath-$Version-windows-x64"
$outputPath = [IO.Path]::GetFullPath($OutputDir)
New-Item -ItemType Directory -Force -Path $outputPath | Out-Null
$archivePath = Join-Path $outputPath "$packageName.zip"
$checksumPath = Join-Path $outputPath 'SHA256SUMS.txt'
if ((Test-Path -LiteralPath $archivePath) -or (Test-Path -LiteralPath $checksumPath)) {
    throw 'Output already contains a release. Use a fresh output directory.'
}
# A unique stage prevents stale DLLs from hiding deployment errors. No deletion.
# 每次使用独立安装目录，避免旧 DLL 掩盖缺失依赖；保留该目录便于排查。
$stageRoot = Join-Path $buildPath ('release-stage-' + [guid]::NewGuid().ToString('N'))
$stage = Join-Path $stageRoot $packageName
& cmake --install $buildPath --config $Configuration --component Runtime --prefix $stage
if ($LASTEXITCODE -ne 0) { throw 'Runtime installation failed' }

# 检查主程序、关键依赖、Qt 平台插件和现有许可声明是否已安装。
$required = @(
    'OpenAutoPath.exe', 'bin/OpenAutoPathApp.exe', 'bin/oap_opencamlib.dll',
    'bin/Qt6Core.dll', 'bin/Qt6Gui.dll', 'bin/Qt6Widgets.dll',
    'bin/Qt6OpenGL.dll', 'bin/Qt6OpenGLWidgets.dll',
    'bin/platforms/qwindows.dll', 'bin/TKernel.dll', 'bin/TKV3d.dll',
    'bin/vcomp140.dll',
    'share/licenses/OpenAutoPath/LICENSE',
    'share/licenses/OpenAutoPath/THIRD_PARTY_NOTICES.md',
    'share/licenses/OpenAutoPath/AnalysisSitus-BSD-3-Clause.txt',
    'share/licenses/OpenAutoPath/qttranslations/qtbase_zh_CN.ts',
    'share/licenses/OpenAutoPath/qttranslations/README.md',
    'share/licenses/OpenAutoPath/qttranslations/licenseRule.json',
    'share/licenses/OpenAutoPath/qttranslations/LGPL-3.0-only.txt',
    'share/licenses/OpenAutoPath/qttranslations/GPL-3.0-only.txt',
    'share/licenses/OpenAutoPath/OpenCAMLib-COPYING.txt',
    'share/licenses/OpenAutoPath/OpenCAMLib-NOTICE.txt',
    'share/licenses/OpenAutoPath/Boost-LICENSE_1_0.txt',
    'share/licenses/OpenAutoPath/Clipper2-LICENSE.txt'
)
foreach ($relative in $required) {
    if (-not (Test-Path -LiteralPath (Join-Path $stage $relative) -PathType Leaf)) {
        throw "Required runtime file missing: $relative"
    }
}
if (-not (Test-Path -LiteralPath (Join-Path $stage 'bin/vc_redist.x64.exe'))) {
    throw 'The x64 Visual C++ runtime installer is missing'
}
# Verify deployed libraries and include their notices, SDK evidence and sources.
& (Join-Path $PSScriptRoot 'Complete-WindowsReleaseMetadata.ps1') -Stage $stage -BuildDir $buildPath -SourceCache $SourceCache -Offline:$OfflineSources -RequireSourceBuilds:($Version.StartsWith('v'))
Copy-Item -LiteralPath (Join-Path $sourceRoot 'BUILDING.md') -Destination $stage
@{
    version = $Version
    commit = $commit.Trim()
    configuration = $Configuration
    platform = 'windows-x64'
    createdUtc = [DateTime]::UtcNow.ToString('o')
} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $stage 'build-info.json') -Encoding UTF8
@'
OpenAutoPath for Windows x64

中文使用说明

请先完整解压，再运行包根目录的 OpenAutoPath.exe。
保留 bin 和 share 目录及其相对位置，不要只复制启动程序。
如果系统尚未安装 Visual C++ 运行库，请先运行 bin/vc_redist.x64.exe。
版本、源码提交号和构建配置记录在 build-info.json 中。本软件包未进行代码签名。
项目自有代码采用 MIT；第三方组件保留原许可。
许可与声明位于 share/licenses/OpenAutoPath，分发范围见 THIRD_PARTY_NOTICES.md。
依赖源码归档位于 share/sources/OpenAutoPath，构建说明见 DEPENDENCY_SOURCES.md。
构建与发布流程的中英文说明见同目录的 BUILDING.md。

English instructions

Extract the entire archive before starting OpenAutoPath.exe at the package root.
Keep the bin and share directories beside it. If the Visual C++ runtime is not
installed, run bin/vc_redist.x64.exe and then start OpenAutoPath.exe.
Build identity is recorded in build-info.json. This package is not code-signed.
Project-owned code is MIT-licensed; third-party components retain their licenses.
See share/licenses/OpenAutoPath/LICENSE and THIRD_PARTY_NOTICES.md for scope.
Dependency sources and build notes are under share/sources/OpenAutoPath.
'@ | Set-Content -LiteralPath (Join-Path $stage 'README.txt') -Encoding UTF8

# ZIP 保留版本命名的顶层目录；校验文件可用于核对下载后的文件完整性。
Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem
# Use ZIP-standard forward slashes even under Windows PowerShell/.NET Framework.
$archive = [IO.Compression.ZipFile]::Open($archivePath, [IO.Compression.ZipArchiveMode]::Create)
try {
    foreach ($file in Get-ChildItem -LiteralPath $stage -Recurse -File | Sort-Object FullName) {
        $relative = $file.FullName.Substring($stage.Length + 1).Replace('\', '/')
        [IO.Compression.ZipFileExtensions]::CreateEntryFromFile(
            $archive, $file.FullName, "$packageName/$relative",
            [IO.Compression.CompressionLevel]::Optimal) | Out-Null
    }
} finally {
    $archive.Dispose()
}
$hash = (Get-FileHash -LiteralPath $archivePath -Algorithm SHA256).Hash.ToLowerInvariant()
"$hash  $packageName.zip" | Set-Content -LiteralPath $checksumPath -Encoding ascii
Write-Host "Created $archivePath"
Write-Host "SHA256: $hash"

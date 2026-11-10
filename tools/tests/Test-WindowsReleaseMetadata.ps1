# Exercise release guards with an isolated fixture. No release or network access.
[CmdletBinding()]
param([Parameter(Mandatory)][string]$BuildDir, [string]$FreeImageRoot, [string]$FFmpegRoot, [string]$OCCTRoot)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$root = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$collector = Join-Path $root 'tools/Complete-WindowsReleaseMetadata.ps1'
$buildPath = (Resolve-Path -LiteralPath $BuildDir).Path
$fixture = Join-Path $buildPath ('license-tests-' + [guid]::NewGuid().ToString('N'))
$notices = Join-Path $fixture 'share/licenses/OpenAutoPath'
New-Item -ItemType Directory -Force -Path $notices, (Join-Path $fixture 'bin') | Out-Null
Copy-Item -LiteralPath (Join-Path $root 'licenses/runtime') -Destination $notices -Recurse
Copy-Item -LiteralPath (Join-Path $buildPath 'generated/opencamlib/OpenCAMLib-NOTICE.txt') -Destination $notices
$cache = Get-Content -LiteralPath (Join-Path $buildPath 'CMakeCache.txt') -Raw
$match = [regex]::Match($cache, '(?m)^Qt6_DIR:PATH=(.+)\r?$')
if (-not $match.Success) { throw 'Qt6_DIR is missing' }
$qtRoot = [IO.Path]::GetFullPath((Join-Path $match.Groups[1].Value.Trim() '../../..'))
$qtDll = Join-Path $fixture 'bin/Qt6Core.dll'
Copy-Item -LiteralPath (Join-Path $qtRoot 'bin/Qt6Core.dll') -Destination $qtDll
& $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly

function Assert-Rejected([scriptblock]$Action, [string]$Expected) {
    $caught = $null
    try { & $Action } catch { $caught = $_.Exception.Message }
    if (-not $caught -or $caught -notlike "*$Expected*") {
        throw "Expected rejection '$Expected', got '$caught'"
    }
    Write-Host "PASS: $Expected"
}

Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly -RequireSourceBuilds } 'Source-built release requires'

$unknown = Join-Path $fixture 'bin/unreviewed.dll'
[IO.File]::WriteAllText($unknown, 'unknown dependency')
Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly } 'Unreviewed runtime file'
# Rename fixture files, retaining evidence for inspection without recursive deletion.
Move-Item -LiteralPath $unknown -Destination ($unknown + '.fixture')
$notice = Join-Path $notices 'runtime/Qt-6.11.1-NOTICES.txt'
Move-Item -LiteralPath $notice -Destination ($notice + '.fixture')
Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly } 'Missing third-party notice'
Move-Item -LiteralPath ($notice + '.fixture') -Destination $notice
[IO.File]::AppendAllText($qtDll, 'changed SDK')
Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly } 'Runtime changed since license review'
Copy-Item -LiteralPath (Join-Path $qtRoot 'bin/Qt6Core.dll') -Destination $qtDll -Force

$sourceCache = Join-Path $fixture 'source-cache'
New-Item -ItemType Directory -Force -Path $sourceCache | Out-Null
Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -SourceCache $sourceCache -Offline } 'Source archive unavailable offline'
[IO.File]::WriteAllText((Join-Path $sourceCache 'qtbase-6.11.1.tar.xz'), 'not a source archive')
Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -SourceCache $sourceCache -Offline } 'Cached source archive SHA256 mismatch'
Write-Host "Release metadata guard tests passed. Fixture: $fixture"
if ($FreeImageRoot) {
    $evidence = Join-Path $notices 'build-evidence'
    New-Item -ItemType Directory -Force -Path $evidence | Out-Null
    $recordFile = Join-Path $evidence 'FreeImage-build-record.json'
    Copy-Item (Join-Path $FreeImageRoot 'bin/FreeImage.dll') (Join-Path $fixture 'bin/FreeImage.dll')
    Copy-Item (Join-Path $FreeImageRoot 'build-record.json') $recordFile
    & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly
    $recordText = Get-Content $recordFile -Raw -Encoding UTF8
    $record = $recordText | ConvertFrom-Json
    $record.dllSha256 = 'invalid'
    $record | ConvertTo-Json -Depth 6 | Set-Content $recordFile -Encoding UTF8
    Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly } 'FreeImage source build record does not match'
    $record = $recordText | ConvertFrom-Json
    $record.sourceSha256 = 'invalid'
    $record | ConvertTo-Json -Depth 6 | Set-Content $recordFile -Encoding UTF8
    Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly } 'FreeImage source build record does not match'
    $recordText | Set-Content $recordFile -Encoding UTF8
    Move-Item $recordFile ($recordFile + '.fixture')
    Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly } 'Changed FreeImage runtime has no source build record'
    Move-Item ($recordFile + '.fixture') $recordFile
    Write-Host 'FreeImage source build record guard tests passed.'
}
foreach ($sdk in @(@{ id = 'ffmpeg'; root = $FFmpegRoot; bin = 'bin' },
        @{ id = 'occt'; root = $OCCTRoot; bin = 'win64/vc14/bin' })) {
    if (-not $sdk.root) { continue }
    $id = $sdk.id
    $evidence = Join-Path $notices 'build-evidence'
    New-Item -ItemType Directory -Force -Path $evidence | Out-Null
    $recordFile = Join-Path $evidence "$id-build-record.json"
    Copy-Item (Join-Path $sdk.root 'build-record.json') $recordFile
    $recordText = Get-Content $recordFile -Raw -Encoding UTF8
    $record = $recordText | ConvertFrom-Json
    foreach ($binary in $record.outputs) {
        Copy-Item (Join-Path $sdk.root "$($sdk.bin)/$($binary.file)") (Join-Path $fixture "bin/$($binary.file)")
    }
    if ($id -eq 'occt') {
        # The application has already deployed exactly the dependencies used by OCCT.
        foreach ($dependency in $record.dependencyBinaries) {
            if (-not (Test-Path (Join-Path $fixture "bin/$($dependency.file)"))) {
                Copy-Item (Join-Path $buildPath "RelWithDebInfo/bin/$($dependency.file)") (Join-Path $fixture "bin/$($dependency.file)")
            }
        }
    }
    & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly
    foreach ($field in @('sourceSha256', 'recipeSha256')) {
        $record = $recordText | ConvertFrom-Json
        $record.$field = 'invalid'
        $record | ConvertTo-Json -Depth 8 | Set-Content $recordFile -Encoding UTF8
        Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly } "$id source build record does not match"
    }
    $record = $recordText | ConvertFrom-Json
    $record.outputs[0].sha256 = 'invalid'
    $record | ConvertTo-Json -Depth 8 | Set-Content $recordFile -Encoding UTF8
    Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly } "$id source build record does not match"
    $recordText | Set-Content $recordFile -Encoding UTF8
    Move-Item $recordFile ($recordFile + '.fixture')
    Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly } "Changed $id runtime has no source build record"
    Move-Item ($recordFile + '.fixture') $recordFile
    if ($id -eq 'occt') {
        $record = $recordText | ConvertFrom-Json
        $record.dependencyBinaries[0].sha256 = 'invalid'
        $record | ConvertTo-Json -Depth 8 | Set-Content $recordFile -Encoding UTF8
        Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly } 'OCCT source build dependency mismatch'
        $recordText | Set-Content $recordFile -Encoding UTF8
        $jemalloc = Join-Path $fixture 'bin/jemalloc.dll'
        Copy-Item (Join-Path $root 'external/3rdparty-vc14-64/jemalloc-vc14-64/bin/jemalloc.dll') $jemalloc
        Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly } 'stale jemalloc.dll'
        Move-Item $jemalloc ($jemalloc + '.fixture')
        & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly -RequireSourceBuilds
        $missing = Join-Path $fixture 'bin/avformat-57.dll'
        Move-Item $missing ($missing + '.fixture')
        Assert-Rejected { & $collector -Stage $fixture -BuildDir $buildPath -VerifyOnly -RequireSourceBuilds } 'Source-built release is missing runtime'
        Move-Item ($missing + '.fixture') $missing
    }
    Write-Host "$id source build record guard tests passed."
}

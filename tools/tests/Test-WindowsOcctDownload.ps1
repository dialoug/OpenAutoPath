# Exercise retry and integrity failures without network access or SDK extraction.
$ErrorActionPreference = 'Stop'
$tokens = $null
$parseErrors = $null
$ast = [System.Management.Automation.Language.Parser]::ParseFile(
    (Join-Path (Get-Location) 'tools/Install-WindowsOcct.ps1'),
    [ref]$tokens, [ref]$parseErrors)
if ($parseErrors.Count) { throw $parseErrors[0] }
$definition = $ast.Find({ param($node)
    $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and
    $node.Name -eq 'Get-VerifiedArchive'
}, $true)
Invoke-Expression $definition.Extent.Text
$fixture = Join-Path (Get-Location) ('build/download-check-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $fixture | Out-Null
$seed = Join-Path $fixture 'seed'
[IO.File]::WriteAllText($seed, 'verified archive fixture')
$expectedHash = (Get-FileHash -LiteralPath $seed -Algorithm SHA256).Hash
function Start-Sleep { param([int]$Seconds) }
function Invoke-WebRequest {
    param([string]$Uri, [string]$OutFile, [switch]$UseBasicParsing)
    $script:attempts++
    if ($script:mode -eq 'transient' -and $script:attempts -eq 1) {
        [IO.File]::WriteAllText($OutFile, 'partial')
        throw 'Simulated HTTP failure'
    }
    if ($script:mode -eq 'corrupt') {
        [IO.File]::WriteAllText($OutFile, 'invalid archive')
    } else {
        [IO.File]::WriteAllText($OutFile, 'verified archive fixture')
    }
}
$script:mode = 'transient'
$script:attempts = 0
$archive = Join-Path $fixture 'success.zip'
Get-VerifiedArchive -Uri 'https://example.invalid/sdk.zip' -Archive $archive -Sha256 $expectedHash
if ($script:attempts -ne 2 -or (Test-Path -LiteralPath ($archive + '.download')) -or
    (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash -ne $expectedHash) {
    throw 'Transient download retry did not produce a verified archive'
}
$script:mode = 'corrupt'
$script:attempts = 0
$archive = Join-Path $fixture 'failure.zip'
$rejected = $false
try {
    Get-VerifiedArchive -Uri 'https://example.invalid/sdk.zip' -Archive $archive -Sha256 $expectedHash
} catch {
    if ($_.Exception.Message -notmatch 'SHA256 mismatch') { throw }
    $rejected = $true
}
if (-not $rejected -or $script:attempts -ne 3 -or
    (Test-Path -LiteralPath $archive) -or (Test-Path -LiteralPath ($archive + '.download'))) {
    throw 'Corrupt archive was not safely rejected after bounded retries'
}
Write-Output 'Download retry and checksum-rejection checks passed'

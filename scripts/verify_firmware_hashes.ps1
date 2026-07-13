$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$manifestPath = Join-Path $repoRoot "FIRMWARE_SHA256SUMS.txt"
$entries = Get-Content -LiteralPath $manifestPath -Encoding utf8
$verified = 0

foreach ($line in $entries) {
    if ([string]::IsNullOrWhiteSpace($line)) { continue }
    if ($line -notmatch '^([0-9a-fA-F]{64})\s{2}(.+)$') {
        throw "Malformed checksum entry: $line"
    }

    $expected = $Matches[1].ToLowerInvariant()
    $relativePath = $Matches[2]
    $fullPath = Join-Path $repoRoot $relativePath

    if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
        throw "Firmware file is missing: $relativePath"
    }

    $actual = (Get-FileHash -LiteralPath $fullPath -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actual -ne $expected) {
        throw "Checksum mismatch: $relativePath"
    }

    Write-Output "OK  $relativePath"
    $verified++
}

if ($verified -eq 0) {
    throw "No firmware checksums were found."
}

Write-Output "Firmware checksum verification passed: $verified files."

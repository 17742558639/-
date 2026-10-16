$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$header = Get-Content -LiteralPath (Join-Path $root "Models\Inc\hmi.h") -Raw -Encoding UTF8
$source = Get-Content -LiteralPath (Join-Path $root "Models\Src\hmi.c") -Raw -Encoding UTF8

function Assert-Pattern([string]$name, [string]$text, [string]$pattern) {
    if ($text -notmatch $pattern) {
        throw "FAILED: $name"
    }
    Write-Host "PASS: $name"
}

Assert-Pattern "start zone enum exists" $header 'StartZone1[\s\S]*StartZone2'
Assert-Pattern "coarse test mode exists" $header 'CoarseModeTest'
Assert-Pattern "S1 command is exact" $source '"##s1"'
Assert-Pattern "S2 command is exact" $source '"##s2"'
Assert-Pattern "test command is exact" $source '"##test"'
Assert-Pattern "S1 only selects zone" $source 'HmiSelectStartZone\s*\(\s*StartZone1\s*\)'
Assert-Pattern "S2 only selects zone" $source 'HmiSelectStartZone\s*\(\s*StartZone2\s*\)'
Assert-Pattern "test requires selected zone" $source 'Task\.StartZone\s*!=\s*StartZone1\s*&&\s*Task\.StartZone\s*!=\s*StartZone2'
Assert-Pattern "test starts coarse state machine" $source 'StartCoarseTestMode\s*\(\s*\)'
Assert-Pattern "formal flow is disabled" $header '#define\s+FORMAL_FLOW_ENABLE\s+0'

$activeSource = [regex]::Replace($source, '(?s)#if\s+0.*?#endif', '')
if ($activeSource -match 'StartMode\s*\(\s*1\s*\)') {
    throw "FAILED: legacy command can still start formal motion"
}
Write-Host "PASS: legacy commands cannot start formal motion"

Write-Host "HMI coarse-mode regression checks passed."

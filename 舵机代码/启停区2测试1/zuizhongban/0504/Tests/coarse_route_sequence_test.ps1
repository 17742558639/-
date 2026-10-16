$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$hmiHeader = Get-Content -LiteralPath (Join-Path $root "Models\Inc\hmi.h") -Raw -Encoding UTF8
$bfsHeader = Get-Content -LiteralPath (Join-Path $root "Models\Inc\bfs.h") -Raw -Encoding UTF8
$moveHeader = Get-Content -LiteralPath (Join-Path $root "Models\Inc\move.h") -Raw -Encoding UTF8
$moveSource = Get-Content -LiteralPath (Join-Path $root "Models\Src\move.c") -Raw -Encoding UTF8

function Assert-Pattern([string]$name, [string]$text, [string]$pattern) {
    if ($text -notmatch $pattern) {
        throw "FAILED: $name"
    }
    Write-Host "PASS: $name"
}

function Remove-DisabledBlocks([string]$text) {
    return [regex]::Replace($text, '(?s)#if\s+0.*?#endif', '')
}

$activeMove = Remove-DisabledBlocks $moveSource
$coarseBlock = [regex]::Match(
    $activeMove,
    '(?s)void\s+CoarseProcesing\s*\(\s*void\s*\).*?(?=void\s+RunTask)'
).Value

Assert-Pattern "QR coordinate is 0,2" $bfsHeader '#define\s+QR_SCAN_X\s+0[\s\S]*#define\s+QR_SCAN_Y\s+2'
Assert-Pattern "QR arm faces -X" $moveHeader '#define\s+QR_SCAN_YAW\s+-90'
Assert-Pattern "material arm faces -Y" $moveHeader '#define\s+MATERIAL_STATION_YAW\s+0'
Assert-Pattern "rough arm faces +Y" $moveHeader '#define\s+ROUGH_STATION_YAW\s+180'
Assert-Pattern "storage arm faces +X" $moveHeader '#define\s+STORAGE_STATION_YAW\s+90'

Assert-Pattern "QR target is selectable" $activeMove 'case\s+CoarseTargetQr[\s\S]*?QR_SCAN_X[\s\S]*?QR_SCAN_Y[\s\S]*?QR_SCAN_YAW'
Assert-Pattern "return target uses selected start-zone exit" $activeMove 'case\s+CoarseTargetReturn[\s\S]*?StartZoneGetPoint'
Assert-Pattern "route advances QR to material" $activeMove 'case\s+CoarseTargetQr\s*:[\s\S]*?CoarseTargetMaterial'
Assert-Pattern "route advances material to rough" $activeMove 'case\s+CoarseTargetMaterial\s*:[\s\S]*?CoarseTargetRough'
Assert-Pattern "route advances rough to storage" $activeMove 'case\s+CoarseTargetRough\s*:[\s\S]*?CoarseTargetStorage'
Assert-Pattern "route advances storage to return" $activeMove 'case\s+CoarseTargetStorage\s*:[\s\S]*?CoarseTargetReturn'

Assert-Pattern "station alignment stops chassis" $coarseBlock 'case\s+CoarseStationAlign[\s\S]*?Rotation\s*\(\s*Task\.StationYaw\s*\)[\s\S]*?Stop\s*\(\s*\)'
Assert-Pattern "station dwell uses configured 1000ms" $coarseBlock 'case\s+CoarseStationDwell[\s\S]*?HAL_Delay\s*\(\s*COARSE_TEST_DWELL_MS\s*\)'
Assert-Pattern "return runs stop correction" $coarseBlock 'CoarseTargetReturn[\s\S]*?StopCorrection\s*\(\s*\)'
Assert-Pattern "successful test is recorded" $activeMove 'Task\.CoarseTestPassed\s*=\s*1'
Assert-Pattern "successful test clears start-zone selection" $activeMove 'Task\.StartZone\s*=\s*StartZoneNone'
Assert-Pattern "successful test enters finished state" $activeMove 'Task\.CoarseStage\s*=\s*CoarseFinished'
Assert-Pattern "successful test prevents further motion" $activeMove 'CoarseFinishTest[\s\S]*?Task\.TaskFlag\s*=\s*None'

if ($coarseBlock -match 'WaitPlace|SetCameraTask|\bCatch\s*\(|\bPlace\s*\(|\bRunTask\s*\(') {
    throw "FAILED: coarse route calls camera or arm work"
}

Write-Host "Coarse route sequence regression checks passed."

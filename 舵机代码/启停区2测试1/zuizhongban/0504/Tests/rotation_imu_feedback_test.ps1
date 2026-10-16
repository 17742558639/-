$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$move = Get-Content -LiteralPath (Join-Path $root "Models\Src\move.c") -Raw -Encoding UTF8
$header = Get-Content -LiteralPath (Join-Path $root "Models\Inc\move.h") -Raw -Encoding UTF8

function Remove-DisabledAndComments([string]$text) {
    $text = [regex]::Replace($text, '(?s)#if\s+0.*?#endif', '')
    $text = [regex]::Replace($text, '(?s)/\*.*?\*/', '')
    return [regex]::Replace($text, '(?m)//.*$', '')
}

function Assert-Pattern([string]$name, [string]$text, [string]$pattern) {
    if ($text -notmatch $pattern) { throw "FAILED: $name" }
    Write-Host "PASS: $name"
}

function Assert-NoPattern([string]$name, [string]$text, [string]$pattern) {
    if ($text -match $pattern) { throw "FAILED: $name" }
    Write-Host "PASS: $name"
}

$active = Remove-DisabledAndComments $move
$rotation = [regex]::Match($active, '(?s)uint8_t\s+Rotation\s*\([^)]*\).*?(?=uint8_t\s+TurnIMU)').Value
$turn = [regex]::Match($active, '(?s)uint8_t\s+TurnIMU\s*\([^)]*\).*?(?=static\s+void\s+PosTarSetVal)').Value
$runMap = [regex]::Match($active, '(?s)uint8_t\s+RunMap\s*\(\s*void\s*\).*?(?=/\*|#define\s+START_CORRECT)').Value

Assert-Pattern "Rotation returns status" $header 'uint8_t\s+Rotation\s*\('
Assert-Pattern "TurnIMU returns status" $header 'uint8_t\s+TurnIMU\s*\('
Assert-Pattern "Rotation implementation found" $rotation 'uint8_t\s+Rotation'
Assert-Pattern "Rotation waits for fresh IMU before control" $rotation 'Imu_Wait_Ready\s*\('
Assert-Pattern "Rotation waits for fresh IMU after motor step" $rotation 'Rotate\s*\([^;]+;[\s\S]*?Imu_Wait_Ready\s*\('
Assert-Pattern "Rotation caps correction step" $active '#define\s+ROTATION_MAX_STEP_DEGREES\s+30'
Assert-Pattern "Rotation uses reduced speed" $active '#define\s+ROTATE_VEL\s+45'
Assert-Pattern "Rotation stops on stale or unresponsive IMU" $rotation 'RotationStopAll\s*\(\s*\)[\s\S]*?return\s+0'
Assert-NoPattern "Rotation no longer permanently caches IMU failure" $rotation 'static\s+uint8_t\s+imu_ok'
Assert-Pattern "TurnIMU delegates to feedback Rotation" $turn 'return\s+Rotation\s*\('
Assert-Pattern "RunMap rejects failed turn" $runMap 'if\s*\(\s*!TurnIMU\s*\('
Assert-Pattern "start correction rejects failed alignment" $active 'StartCorrection[\s\S]*?if\s*\(\s*!Rotation\s*\(\s*0\s*\)\s*\)[\s\S]*?return\s+0'
Assert-Pattern "stop correction rejects failed alignment" $active 'StopCorrection[\s\S]*?if\s*\(\s*!Rotation\s*\(\s*0\s*\)\s*\)[\s\S]*?return\s+0'
Assert-Pattern "station alignment enters IMU fault on failure" $active 'case\s+CoarseStationAlign\s*:[\s\S]*?if\s*\(\s*!Rotation\s*\(\s*Task\.StationYaw\s*\)\s*\)[\s\S]*?CoarseEnterFault\s*\(\s*CoarseFaultImuTimeout\s*\)'

Write-Host "Rotation IMU feedback regression checks passed."

$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$imuSource = Get-Content -LiteralPath (Join-Path $root "Models\Src\imu.c") -Raw -Encoding UTF8
$imuHeader = Get-Content -LiteralPath (Join-Path $root "Models\Inc\imu.h") -Raw -Encoding UTF8
$moveSource = Get-Content -LiteralPath (Join-Path $root "Models\Src\move.c") -Raw -Encoding UTF8

function Assert-Pattern([string]$name, [string]$source, [string]$pattern) {
    if ($source -notmatch $pattern) {
        throw "FAILED: $name"
    }
    Write-Host "PASS: $name"
}

Assert-Pattern "IMU exposes ready wait API" $imuHeader "Imu_Wait_Ready\s*\(\s*uint32_t\s+timeout_ms\s*\)"
Assert-Pattern "IMU requires consecutive valid frames" $imuSource "IMU_READY_FRAME_COUNT"
Assert-Pattern "startup waits for fresh IMU data" $moveSource "CoarseStartCorrect:[\s\S]*?Imu_Wait_Ready\s*\(\s*1500\s*\)[\s\S]*?Rotation\s*\(\s*0\s*\)"
Assert-Pattern "return correction waits for fresh IMU data" $moveSource "BackCheck\s*\([^)]*\)[\s\S]*?Imu_Wait_Ready\s*\(\s*1500\s*\)[\s\S]*?Rotation\s*\(\s*0\s*\)"

Write-Host "IMU ready regression checks passed."

$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$imu = Get-Content -LiteralPath (Join-Path $root "Models\Src\imu.c") -Raw -Encoding UTF8

function Assert-Pattern([string]$name, [string]$text, [string]$pattern) {
    if ($text -notmatch $pattern) {
        throw "FAILED: $name"
    }
    Write-Host "PASS: $name"
}

Assert-Pattern "IMU still requires the complete fixed frame" $imu 'ImuCount\s*==\s*IMU_CMD'
Assert-Pattern "IMU idle path parses a complete buffered frame" $imu 'Imu_Idle_Func[\s\S]*?ImuCount\s*=\s*IMU_CMD[\s\S]*?Imu_Get\s*\(\s*\)'

$activeImu = [regex]::Match($imu, '(?s)void\s+Imu_Idle_Func\s*\(\s*void\s*\).*').Value
$activeImu = [regex]::Replace($activeImu, '(?s)/\*.*?\*/|//[^\r\n]*', '')
if ($activeImu -match 'ImuCount\s*=\s*IMU_CMD\s*-\s*__HAL_DMA_GET_COUNTER') {
    throw "FAILED: active IMU path still derives frame length from circular DMA NDTR"
}
Write-Host "PASS: active IMU path does not derive frame length from circular DMA NDTR"

Assert-Pattern "IMU frame validity still checks header" $imu 'ImuCmd\[0\]\s*==\s*0x55[\s\S]*?ImuCmd\[1\]\s*==\s*0x53'
Assert-Pattern "IMU frame validity still checks checksum" $imu 'Checksum\s*=\s*ImuCmd\[9\][\s\S]*?ImuCmd\[10\]'
Assert-Pattern "IMU ready protection remains" $imu 'ImuValidFrameCount\s*>=\s*IMU_READY_FRAME_COUNT[\s\S]*?ImuDataReady\s*=\s*1'

Write-Host "IMU DMA fixed-frame regression checks passed."

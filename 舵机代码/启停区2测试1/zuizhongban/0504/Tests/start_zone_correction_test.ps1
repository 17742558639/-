$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$bfs = Get-Content -LiteralPath (Join-Path $root "Models\Inc\bfs.h") -Raw -Encoding UTF8
$move = Get-Content -LiteralPath (Join-Path $root "Models\Src\move.c") -Raw -Encoding UTF8

function Remove-Comments([string]$text) {
    $withoutBlock = [regex]::Replace($text, '(?s)/\*.*?\*/', '')
    return [regex]::Replace($withoutBlock, '(?m)//.*$', '')
}

$activeMove = Remove-Comments $move
$startCorrection = [regex]::Match(
    $activeMove,
    '(?s)static\s+uint8_t\s+StartCorrection\s*\(\s*void\s*\).*?(?=static\s+uint8_t\s+StopCorrection)'
).Value
$stopCorrection = [regex]::Match(
    $activeMove,
    '(?s)static\s+uint8_t\s+StopCorrection\s*\(\s*void\s*\).*?(?=static\s+uint8_t\s+CoarseTargetSet)'
).Value

function Assert-Pattern([string]$name, [string]$text, [string]$pattern) {
    if ($text -notmatch $pattern) {
        throw "FAILED: $name"
    }
    Write-Host "PASS: $name"
}

Assert-Pattern "S1 BFS entry is 0,0" $bfs '#define\s+START_ZONE1_BFS_X\s+0[\s\S]*#define\s+START_ZONE1_BFS_Y\s+0'
Assert-Pattern "S2 BFS entry is 0,4" $bfs '#define\s+START_ZONE2_BFS_X\s+0[\s\S]*#define\s+START_ZONE2_BFS_Y\s+4'
Assert-Pattern "StartCorrection function found" $startCorrection 'StartCorrection'
Assert-Pattern "StopCorrection function found" $stopCorrection 'StopCorrection'
Assert-Pattern "S1 startup validates LEFT for 5cm before remaining 20cm" $startCorrection 'case\s+StartZone1\s*:[\s\S]*?RunMove\s*\(\s*LEFT\s*,\s*5\s*,\s*0\s*\)[\s\S]*?HAL_Delay\s*\(\s*1500\s*\)[\s\S]*?RunMove\s*\(\s*LEFT\s*,\s*20\s*,\s*0\s*\)[\s\S]*?HAL_Delay\s*\(\s*600\s*\)[\s\S]*?RunMove\s*\(\s*FRONT\s*,\s*25\s*,\s*0\s*\)'
Assert-Pattern "S2 startup validates LEFT for 5cm before remaining 20cm" $startCorrection 'case\s+StartZone2\s*:[\s\S]*?RunMove\s*\(\s*LEFT\s*,\s*5\s*,\s*0\s*\)[\s\S]*?HAL_Delay\s*\(\s*1500\s*\)[\s\S]*?RunMove\s*\(\s*LEFT\s*,\s*20\s*,\s*0\s*\)[\s\S]*?HAL_Delay\s*\(\s*600\s*\)[\s\S]*?RunMove\s*\(\s*BACK\s*,\s*25\s*,\s*0\s*\)'
Assert-Pattern "S1 stop is inverse BACK then RIGHT" $stopCorrection 'case\s+StartZone1\s*:[\s\S]*?RunMove\s*\(\s*BACK\s*,\s*25\s*,\s*0\s*\)[\s\S]*?HAL_Delay\s*\(\s*600\s*\)[\s\S]*?RunMove\s*\(\s*RIGHT\s*,\s*25\s*,\s*0\s*\)'
Assert-Pattern "S2 stop is inverse FRONT then RIGHT" $stopCorrection 'case\s+StartZone2\s*:[\s\S]*?RunMove\s*\(\s*FRONT\s*,\s*25\s*,\s*0\s*\)[\s\S]*?HAL_Delay\s*\(\s*600\s*\)[\s\S]*?RunMove\s*\(\s*RIGHT\s*,\s*25\s*,\s*0\s*\)'
Assert-Pattern "startup registers selected BFS entry" $activeMove 'BfsResetStartZone\s*\(\s*\)'
Assert-Pattern "startup waits for fresh IMU" $startCorrection 'Imu_Wait_Ready\s*\(\s*1500\s*\)'
Assert-Pattern "stop waits for fresh IMU" $stopCorrection 'Imu_Wait_Ready\s*\(\s*1500\s*\)'

Write-Host "Start-zone correction regression checks passed."

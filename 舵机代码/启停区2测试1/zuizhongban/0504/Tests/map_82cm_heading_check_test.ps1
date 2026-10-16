$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$move = Get-Content -LiteralPath (Join-Path $root "Models\Src\move.c") -Raw -Encoding UTF8

function Remove-DisabledAndComments([string]$text) {
    $text = [regex]::Replace($text, '(?s)#if\s+0.*?#endif', '')
    $text = [regex]::Replace($text, '(?s)/\*.*?\*/', '')
    return [regex]::Replace($text, '(?m)//.*$', '')
}

function Assert-Pattern([string]$name, [string]$text, [string]$pattern) {
    if ($text -notmatch $pattern) { throw "FAILED: $name" }
    Write-Host "PASS: $name"
}

$active = Remove-DisabledAndComments $move
$runMap = [regex]::Match(
    $active,
    '(?s)uint8_t\s+RunMap\s*\(\s*void\s*\).*?(?=#define\s+START_CORRECT_X_CM)'
).Value

Assert-Pattern "82cm heading-check distance is defined from two 41cm grids" $active '#define\s+MAP_HEADING_CHECK_CM\s+\(\s*UNIT_DIS\s*\*\s*2\s*\)'
Assert-Pattern "one movement chunk is capped at two grids" $runMap 'chunk_steps\s*=\s*\(\s*remaining_steps\s*>\s*2\s*\)\s*\?\s*2\s*:\s*remaining_steps'
Assert-Pattern "chunk distance cannot exceed 82cm" $runMap 'map_distance\s*=\s*UNIT_DIS\s*\*\s*chunk_steps'
<#
后续版断言（已回退）：82cm修正目标可能选择保持的工位朝向。
Assert-Pattern "exact 82cm selects an absolute-yaw correction" $runMap 'map_distance\s*==\s*MAP_HEADING_CHECK_CM[\s\S]*?heading_target\s*=\s*CoarseTravelYawValid\s*\?\s*CoarseTravelYaw\s*:\s*DirToYaw\s*\(\s*dir\s*\)'
#>
Assert-Pattern "exact 82cm corrects to the current BFS road yaw" $runMap 'map_distance\s*==\s*MAP_HEADING_CHECK_CM[\s\S]*?heading_target\s*=\s*DirToYaw\s*\(\s*dir\s*\)'
Assert-Pattern "failed 82cm correction stops movement" $runMap 'if\s*\(\s*!Rotation\s*\(\s*heading_target\s*\)\s*\)[\s\S]*?RotationStopAll\s*\(\s*\)[\s\S]*?return\s+0'
Assert-Pattern "path state advances only after the chunk moved" $runMap 'RunMove\s*\([^;]+;[\s\S]*?for\s*\([^)]*chunk_steps[^)]*\)[\s\S]*?PathStateUpdate\s*\(\s*\)'

Write-Host "82cm heading correction regression checks passed."

$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$move = Get-Content -LiteralPath (Join-Path $root "Models\Src\move.c") -Raw -Encoding UTF8

function Remove-Comments([string]$text) {
    $withoutBlock = [regex]::Replace($text, '(?s)/\*.*?\*/', '')
    return [regex]::Replace($withoutBlock, '(?m)//.*$', '')
}

function Remove-DisabledBlocks([string]$text) {
    return [regex]::Replace($text, '(?s)#if\s+0.*?#endif', '')
}

function Assert-Pattern([string]$name, [string]$text, [string]$pattern) {
    if ($text -notmatch $pattern) {
        throw "FAILED: $name"
    }
    Write-Host "PASS: $name"
}

$activeMove = Remove-Comments (Remove-DisabledBlocks $move)
$runMove = [regex]::Match(
    $activeMove,
    '(?s)void\s+RunMove\s*\([^)]*\).*?(?=void\s+PidMove)'
).Value
$mapBlock = [regex]::Match(
    $runMove,
    '(?s)if\s*\(\s*isrunmap\s*\).*?(?=\}\s*else)'
).Value

Assert-Pattern "RunMove map block found" $mapBlock 'isrunmap'

# 实测基准：yaw=0且车尾朝-Y时，代码LEFT对应地图+X。
# RunDir中RIGHT/LEFT分别表示地图x增/减，FRONT/BACK分别表示地图y增/减。
Assert-Pattern "map +X uses physical LEFT" $mapBlock 'case\s+RIGHT\s*:\s*x_sign\s*=\s*-1\s*;\s*y_sign\s*=\s*0'
Assert-Pattern "map -X uses physical RIGHT" $mapBlock 'case\s+LEFT\s*:\s*x_sign\s*=\s*1\s*;\s*y_sign\s*=\s*0'

# 实测基准：从S2顺时针转到yaw=-90后，旧BACK映射驶向+Y；反转横移符号后才是-Y。
Assert-Pattern "map +Y after yaw -90 uses opposite lateral direction" $mapBlock 'case\s+FRONT\s*:\s*x_sign\s*=\s*1\s*;\s*y_sign\s*=\s*0'
Assert-Pattern "map -Y after yaw -90 uses opposite lateral direction" $mapBlock 'case\s+BACK\s*:\s*x_sign\s*=\s*-1\s*;\s*y_sign\s*=\s*0'

Assert-Pattern "X-axis map travel keeps yaw zero" $activeMove 'DirToYaw[\s\S]*?RIGHT\s*\|\|\s*dir\s*==\s*LEFT[\s\S]*?return\s+0'
Assert-Pattern "Y-axis map travel keeps tested clockwise yaw" $activeMove 'DirToYaw[\s\S]*?return\s+-90'
Assert-Pattern "map travel starts with 5cm direction check" $activeMove 'map_distance\s*>\s*5[\s\S]*?RunMove\s*\(\s*dir\s*,\s*5\s*,\s*1\s*\)[\s\S]*?HAL_Delay\s*\(\s*1500\s*\)[\s\S]*?RunMove\s*\(\s*dir\s*,\s*map_distance\s*-\s*5\s*,\s*1\s*\)'

Write-Host "Map direction mapping regression checks passed."

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
$runMap = [regex]::Match($active, '(?s)uint8_t\s+RunMap\s*\(\s*void\s*\).*?(?=#define\s+START_CORRECT_X_CM)').Value
$coarse = [regex]::Match($active, '(?s)void\s+CoarseProcesing\s*\(\s*void\s*\).*?(?=void\s+RunTask)').Value
$mapping = [regex]::Match($active, '(?s)static\s+uint8_t\s+MapDirToBodyDir\s*\([^)]*\).*?(?=uint8_t\s+RunMap)').Value

Assert-Pattern "coarse travel heading state exists" $active 'static\s+uint8_t\s+CoarseTravelYawValid'
Assert-Pattern "coarse travel heading value exists" $active 'static\s+short\s+CoarseTravelYaw'
Assert-Pattern "destination-aligned state exists" $active 'static\s+uint8_t\s+CoarseDestinationYawReady'
Assert-Pattern "new test start clears held heading" $active 'StartCorrection[\s\S]*?CoarseTravelYawValid\s*=\s*0'
Assert-Pattern "new route clears destination-aligned state" $active 'CoarseAdvanceTarget[\s\S]*?CoarseDestinationYawReady\s*=\s*0'
Assert-Pattern "held BFS heading maps world direction to body direction" $runMap 'CoarseTravelYawValid[\s\S]*?MapDirToBodyDir\s*\(\s*dir\s*,\s*CoarseTravelYaw'
Assert-Pattern "held BFS movement uses body coordinate mode" $runMap 'RunMove\s*\(\s*body_dir\s*,[^,]+,\s*0\s*\)'
Assert-Pattern "82cm correction selects station heading" $runMap 'heading_target\s*=\s*CoarseTravelYawValid\s*\?\s*CoarseTravelYaw\s*:\s*DirToYaw\s*\(\s*dir\s*\)'
Assert-Pattern "82cm correction applies selected heading" $runMap 'Rotation\s*\(\s*heading_target\s*\)'
Assert-Pattern "first straight segment keeps departure heading" $runMap 'segment_index\s*=\s*0[\s\S]*?segment_index\s*>\s*0'
Assert-Pattern "BFS corner aligns to destination heading once" $runMap 'segment_index\s*>\s*0[\s\S]*?!Rotation\s*\(\s*Task\.StationYaw\s*\)[\s\S]*?CoarseTravelYaw\s*=\s*Task\.StationYaw[\s\S]*?CoarseDestinationYawReady\s*=\s*1'
Assert-Pattern "arrival skips duplicate station turn when corner already aligned" $coarse 'case\s+CoarseStationAlign[\s\S]*?if\s*\(\s*!CoarseDestinationYawReady\s*\)[\s\S]*?Rotation\s*\(\s*Task\.StationYaw\s*\)'
Assert-Pattern "station alignment saves final heading" $coarse 'CoarseTravelYaw\s*=\s*Task\.StationYaw[\s\S]*?CoarseTravelYawValid\s*=\s*1'

Assert-Pattern "yaw 180 map +X uses body RIGHT" $mapping 'case\s+180\s*:[\s\S]*?case\s+RIGHT\s*:\s*\*body_dir\s*=\s*RIGHT'
Assert-Pattern "yaw 180 map -X uses body LEFT" $mapping 'case\s+180\s*:[\s\S]*?case\s+LEFT\s*:\s*\*body_dir\s*=\s*LEFT'
Assert-Pattern "yaw 180 map +Y uses body BACK" $mapping 'case\s+180\s*:[\s\S]*?case\s+FRONT\s*:\s*\*body_dir\s*=\s*BACK'
Assert-Pattern "yaw 180 map -Y uses body FRONT" $mapping 'case\s+180\s*:[\s\S]*?case\s+BACK\s*:\s*\*body_dir\s*=\s*FRONT'
Assert-Pattern "yaw 90 map -Y uses body RIGHT" $mapping 'case\s+90\s*:[\s\S]*?case\s+BACK\s*:\s*\*body_dir\s*=\s*RIGHT'

Write-Host "Station heading hold regression checks passed."

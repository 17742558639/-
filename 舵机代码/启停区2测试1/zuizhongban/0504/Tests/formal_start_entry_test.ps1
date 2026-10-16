$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$movePath = Join-Path $root "Models\Src\move.c"
$taskPath = Join-Path $root "Models\Src\task.c"

function Remove-CComments([string]$source) {
    return [regex]::Replace($source, '(?s)/\*.*?\*/|//[^\r\n]*', '')
}

function Assert-Pattern([string]$name, [string]$source, [string]$pattern) {
    if ($source -notmatch $pattern) {
        throw "FAILED: $name"
    }
    Write-Host "PASS: $name"
}

$move = Remove-CComments (Get-Content -LiteralPath $movePath -Raw -Encoding UTF8)
$task = Remove-CComments (Get-Content -LiteralPath $taskPath -Raw -Encoding UTF8)

$formalStart = [regex]::Match($move, 'if\s*\(\s*Task\.TaskRunIndex\s*==\s*0(?:\s*&&\s*Task\.StartCorrected\s*==\s*0)?\s*\)\s*\{(?<body>[\s\S]*?)PosTarSet\s*\(\s*\)', 'Singleline')
if (-not $formalStart.Success) {
    throw "FAILED: formal startup block found"
}

$body = $formalStart.Groups['body'].Value
Assert-Pattern "formal startup waits for IMU" $body 'Imu_Wait_Ready\s*\(\s*1500\s*\)'
# 旧断言检查花括号内部，无法看到位于花括号前的 if 条件：
# Assert-Pattern "formal startup runs only once" $body 'Task\.TaskRunIndex\s*==\s*0\s*&&\s*Task\.StartCorrected\s*==\s*0'
Assert-Pattern "formal startup runs only once" $formalStart.Value 'Task\.TaskRunIndex\s*==\s*0\s*&&\s*Task\.StartCorrected\s*==\s*0'
Assert-Pattern "formal startup aligns yaw before movement" $body 'Rotation\s*\(\s*0\s*\)'
Assert-Pattern "formal startup performs RIGHT correction" $body 'RunMove\s*\(\s*RIGHT\s*,\s*25\s*,\s*0\s*\)'
Assert-Pattern "formal startup performs BACK correction" $body 'RunMove\s*\(\s*BACK\s*,\s*25\s*,\s*0\s*\)'
Assert-Pattern "formal startup resets BFS entry" $body 'BfsResetStart\s*\(\s*\)'

$rightIndex = $body.IndexOf('RunMove(RIGHT')
$backIndex = $body.IndexOf('RunMove(BACK')
if ($rightIndex -lt 0 -or $backIndex -lt 0 -or $rightIndex -ge $backIndex) {
    throw "FAILED: startup correction order"
}
if ($body.Substring($rightIndex, $backIndex - $rightIndex) -notmatch 'HAL_Delay\s*\(\s*600\s*\)') {
    throw "FAILED: startup correction waits between segments"
}
Assert-Pattern "formal startup waits after BACK correction" $body 'RunMove\s*\(\s*BACK\s*,\s*25\s*,\s*0\s*\)[\s\S]*?HAL_Delay\s*\(\s*600\s*\)'
Assert-Pattern "formal startup marks correction complete" $body 'Task\.StartCorrected\s*=\s*1'

Assert-Pattern "coarse and formal state machines are mutually exclusive" $task 'if\s*\(\s*Task\.CoarseStage\s*!=\s*CoarseIdle\s*\)\s*\{[\s\S]*?CoarseProcesing\s*\(\s*\)\s*;[\s\S]*?return\s*;'

Write-Host "Formal BFS entry regression checks passed."

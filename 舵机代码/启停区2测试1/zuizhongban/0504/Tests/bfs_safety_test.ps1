$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$bfsSource = Get-Content -LiteralPath (Join-Path $root "Models\Src\bfs.c") -Raw -Encoding UTF8
$moveSource = Get-Content -LiteralPath (Join-Path $root "Models\Src\move.c") -Raw -Encoding UTF8
$taskSource = Get-Content -LiteralPath (Join-Path $root "Models\Src\task.c") -Raw -Encoding UTF8
$hmiSource = Get-Content -LiteralPath (Join-Path $root "Models\Src\hmi.c") -Raw -Encoding UTF8

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
$runMapBlock = [regex]::Match(
    $activeMove,
    '(?s)uint8_t\s+RunMap\s*\(\s*void\s*\).*?(?=/\*\s*\r?\n\s*\*\s*===\s*粗定位参数)'
).Value

Assert-Pattern "BFS validates start coordinate before map access" $bfsSource 'start\.x\s*<\s*0[\s\S]*?start\.x\s*>=\s*rows[\s\S]*?start\.y\s*<\s*0[\s\S]*?start\.y\s*>=\s*cols'
Assert-Pattern "BFS validates goal coordinate before map access" $bfsSource 'goal\.x\s*<\s*0[\s\S]*?goal\.x\s*>=\s*rows[\s\S]*?goal\.y\s*<\s*0[\s\S]*?goal\.y\s*>=\s*cols'
Assert-Pattern "path calculation clears stale path" $bfsSource 'void\s+CalPath[\s\S]*?memset\s*\(\s*Path\.path'
Assert-Pattern "path calculation resets path index" $bfsSource 'void\s+CalPath[\s\S]*?Path\.path_index\s*=\s*1'

Assert-Pattern "RunMap validates grid points" $activeMove 'static\s+uint8_t\s+GridPointValid'
Assert-Pattern "RunMap validates complete BFS result" $activeMove 'static\s+uint8_t\s+PathResultValid'
Assert-Pattern "RunMap rejects invalid path length" $activeMove 'Path\.path_len\s*<\s*2[\s\S]*?Path\.path_len\s*>\s*ROWS\s*\*\s*COLS'
Assert-Pattern "RunMap stops all motors on failure" $runMapBlock 'RotationStopAll\s*\(\s*\)'
Assert-Pattern "RunMap returns failure" $runMapBlock 'return\s+0\s*;'
$gridCheckIndex = $runMapBlock.IndexOf('if (!GridPointValid')
$positionCheckIndex = $runMapBlock.IndexOf('if (PosCp())')
if ($gridCheckIndex -lt 0 -or $positionCheckIndex -lt 0 -or $gridCheckIndex -ge $positionCheckIndex) {
    throw "FAILED: RunMap must validate coordinates before accepting an equal position"
}
Write-Host "PASS: RunMap validates coordinates before accepting an equal position"
Assert-Pattern "coarse route converts RunMap failure to BFS fault" $activeMove 'if\s*\(\s*!RunMap\s*\(\s*\)\s*\)[\s\S]*?CoarseEnterFault\s*\(\s*CoarseFaultBfsPath\s*\)'

Assert-Pattern "formal processing is compile-time gated" $taskSource '#if\s+FORMAL_FLOW_ENABLE[\s\S]*?Procesing\s*\(\s*\)\s*;[\s\S]*?#endif'
Assert-Pattern "fault can only reset through start-zone selection" $hmiSource 'CoarseStage\s*!=\s*CoarseFault[\s\S]*?Task\.CoarseStage\s*=\s*CoarseIdle'
Assert-Pattern "test command only starts from idle" $hmiSource 'Task\.TaskFlag\s*==\s*None\s*&&\s*Task\.CoarseStage\s*==\s*CoarseIdle'

Write-Host "BFS safety and fault-lock regression checks passed."

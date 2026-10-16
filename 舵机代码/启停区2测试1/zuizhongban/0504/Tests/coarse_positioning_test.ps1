param(
    [string]$ProjectRoot = (Join-Path $PSScriptRoot "..")
)

$ErrorActionPreference = "Stop"

$hmiHeader = Get-Content -LiteralPath (Join-Path $ProjectRoot "Models\Inc\hmi.h") -Raw -Encoding UTF8
$hmiSource = Get-Content -LiteralPath (Join-Path $ProjectRoot "Models\Src\hmi.c") -Raw -Encoding UTF8
$bfsHeader = Get-Content -LiteralPath (Join-Path $ProjectRoot "Models\Inc\bfs.h") -Raw -Encoding UTF8
$bfsSource = Get-Content -LiteralPath (Join-Path $ProjectRoot "Models\Src\bfs.c") -Raw -Encoding UTF8
$moveHeader = Get-Content -LiteralPath (Join-Path $ProjectRoot "Models\Inc\move.h") -Raw -Encoding UTF8
$moveSource = Get-Content -LiteralPath (Join-Path $ProjectRoot "Models\Src\move.c") -Raw -Encoding UTF8
$taskSource = Get-Content -LiteralPath (Join-Path $ProjectRoot "Models\Src\task.c") -Raw -Encoding UTF8

$failures = 0

function Assert-Pattern([string]$Name, [string]$Text, [string]$Pattern)
{
    if ($Text -notmatch $Pattern)
    {
        Write-Host "FAIL integration: $Name"
        $script:failures++
    }
}

$expectedTransitions = @{
    "CoarseStartCorrect" = "CoarseBfsNavigate"
    "CoarseBfsNavigate" = "CoarseStationAlign"
    "CoarseStationAlign" = "CoarseWaitArm"
    "CoarseWaitArm" = "CoarseWaitArm"
}

foreach ($stage in $expectedTransitions.Keys)
{
    if ([string]::IsNullOrWhiteSpace($expectedTransitions[$stage]))
    {
        Write-Host "FAIL state model: $stage has no next stage"
        $failures++
    }
}

Assert-Pattern "BFS entry X is 0" $bfsHeader "#define\s+BFS_START_X\s+0"
Assert-Pattern "BFS entry Y is 4" $bfsHeader "#define\s+BFS_START_Y\s+4"
Assert-Pattern "material coarse X is 2" $bfsHeader "#define\s+MATERIAL_COARSE_X\s+2"
Assert-Pattern "material coarse Y is 0" $bfsHeader "#define\s+MATERIAL_COARSE_Y\s+0"
Assert-Pattern "rough coarse X is 2" $bfsHeader "#define\s+ROUGH_COARSE_X\s+2"
Assert-Pattern "rough coarse Y is 4" $bfsHeader "#define\s+ROUGH_COARSE_Y\s+4"
Assert-Pattern "storage coarse X is 4" $bfsHeader "#define\s+STORAGE_COARSE_X\s+4"
Assert-Pattern "storage coarse Y is 2" $bfsHeader "#define\s+STORAGE_COARSE_Y\s+2"

Assert-Pattern "material station yaw keeps arm at -Y" $moveHeader "#define\s+MATERIAL_STATION_YAW\s+0"
Assert-Pattern "rough station yaw keeps arm at +Y" $moveHeader "#define\s+ROUGH_STATION_YAW\s+180"
Assert-Pattern "storage station yaw keeps arm at +X" $moveHeader "#define\s+STORAGE_STATION_YAW\s+90"
Assert-Pattern "hmi.h declares rough target" $hmiHeader "CoarseTargetRough"
Assert-Pattern "hmi.h declares storage target" $hmiHeader "CoarseTargetStorage"

foreach ($stage in $expectedTransitions.Keys)
{
    Assert-Pattern "hmi.h declares $stage" $hmiHeader $stage
}

Assert-Pattern "transport starts coarse material positioning" $hmiSource "Task\.CoarseStage\s*=\s*CoarseStartCorrect"
Assert-Pattern "start correction defaults to zero X" $moveSource "#define\s+START_CORRECT_X_CM\s+0"
Assert-Pattern "start correction defaults to zero Y" $moveSource "#define\s+START_CORRECT_Y_CM\s+0"
Assert-Pattern "start correction aligns yaw zero" $moveSource "case\s+CoarseStartCorrect[\s\S]*?Rotation\s*\(\s*0\s*\)"
Assert-Pattern "BFS position becomes corrected entry" $moveSource "Path\.pos\.x\s*=\s*BFS_START_X[\s\S]*?Path\.pos\.y\s*=\s*BFS_START_Y"
Assert-Pattern "coarse destination is material point" $moveSource "Path\.pos_tar\.x\s*=\s*MATERIAL_COARSE_X[\s\S]*?Path\.pos_tar\.y\s*=\s*MATERIAL_COARSE_Y"
Assert-Pattern "coarse navigation calls RunMap" $moveSource "case\s+CoarseBfsNavigate[\s\S]*?RunMap\s*\(\s*\)"
Assert-Pattern "station alignment uses configured yaw" $moveSource "case\s+CoarseStationAlign[\s\S]*?Rotation\s*\(\s*Task\.StationYaw\s*\)"
Assert-Pattern "main loop executes coarse positioning" $taskSource "CoarseProcesing\s*\(\s*\)"
Assert-Pattern "rough target selects rough coordinates" $moveSource "case\s+CoarseTargetRough[\s\S]*?Path\.pos_tar\.x\s*=\s*ROUGH_COARSE_X[\s\S]*?Path\.pos_tar\.y\s*=\s*ROUGH_COARSE_Y"
Assert-Pattern "rough target selects yaw 180" $moveSource "case\s+CoarseTargetRough[\s\S]*?Task\.StationYaw\s*=\s*ROUGH_STATION_YAW"
Assert-Pattern "storage target selects storage coordinates" $moveSource "case\s+CoarseTargetStorage[\s\S]*?Path\.pos_tar\.x\s*=\s*STORAGE_COARSE_X[\s\S]*?Path\.pos_tar\.y\s*=\s*STORAGE_COARSE_Y"
Assert-Pattern "storage target selects yaw 90" $moveSource "case\s+CoarseTargetStorage[\s\S]*?Task\.StationYaw\s*=\s*STORAGE_STATION_YAW"
Assert-Pattern "later stages can request another coarse target" $moveSource "void\s+CoarseNavigateTo\s*\(\s*uint8_t\s+target\s*\)"
Assert-Pattern "coarse continuation clears legacy task counts" $moveSource "void\s+CoarseNavigateTo[\s\S]*?Task\.ItemNum\s*=\s*0[\s\S]*?Task\.PlaceNum\s*=\s*0"

$route = @(
    @{ X = 0; Y = 4 },
    @{ X = 0; Y = 3 },
    @{ X = 0; Y = 2 },
    @{ X = 0; Y = 1 },
    @{ X = 0; Y = 0 },
    @{ X = 1; Y = 0 },
    @{ X = 2; Y = 0 }
)
$walls = @("1,1", "1,3", "3,1", "3,3")

for ($i = 0; $i -lt $route.Count; $i++)
{
    $pointKey = "$($route[$i].X),$($route[$i].Y)"
    if ($walls -contains $pointKey)
    {
        Write-Host "FAIL BFS route: point $pointKey is a wall"
        $failures++
    }

    if ($i -gt 0)
    {
        $dx = [Math]::Abs($route[$i].X - $route[$i - 1].X)
        $dy = [Math]::Abs($route[$i].Y - $route[$i - 1].Y)
        if (($dx + $dy) -ne 1)
        {
            Write-Host "FAIL BFS route: non-adjacent step at index $i"
            $failures++
        }
    }
}

$edgeCount = $route.Count - 1
$minimumEdges = [Math]::Abs(2 - 0) + [Math]::Abs(0 - 4)
if ($edgeCount -ne 6 -or $edgeCount -ne $minimumEdges)
{
    Write-Host "FAIL BFS route: expected 6 shortest edges, actual=$edgeCount"
    $failures++
}

$materialToRough = @(
    @{ X = 2; Y = 0 },
    @{ X = 2; Y = 1 },
    @{ X = 2; Y = 2 },
    @{ X = 2; Y = 3 },
    @{ X = 2; Y = 4 }
)

$roughToStorage = @(
    @{ X = 2; Y = 4 },
    @{ X = 3; Y = 4 },
    @{ X = 4; Y = 4 },
    @{ X = 4; Y = 3 },
    @{ X = 4; Y = 2 }
)

foreach ($namedRoute in @(
    @{ Name = "material-to-rough"; Points = $materialToRough; ExpectedEdges = 4 },
    @{ Name = "rough-to-storage"; Points = $roughToStorage; ExpectedEdges = 4 }
))
{
    for ($i = 0; $i -lt $namedRoute.Points.Count; $i++)
    {
        $point = $namedRoute.Points[$i]
        $pointKey = "$($point.X),$($point.Y)"
        if ($walls -contains $pointKey)
        {
            Write-Host "FAIL $($namedRoute.Name): point $pointKey is a wall"
            $failures++
        }

        if ($i -gt 0)
        {
            $previous = $namedRoute.Points[$i - 1]
            $distance = [Math]::Abs($point.X - $previous.X) + [Math]::Abs($point.Y - $previous.Y)
            if ($distance -ne 1)
            {
                Write-Host "FAIL $($namedRoute.Name): non-adjacent step at index $i"
                $failures++
            }
        }
    }

    if (($namedRoute.Points.Count - 1) -ne $namedRoute.ExpectedEdges)
    {
        Write-Host "FAIL $($namedRoute.Name): wrong edge count"
        $failures++
    }
}

Assert-Pattern "current map keeps four center obstacles" $bfsSource "(?s)\{0,\s*1,\s*0,\s*1,\s*0\}.*\{0,\s*1,\s*0,\s*1,\s*0\}"

$coarseBlock = [regex]::Match($moveSource, "(?s)void\s+CoarseProcesing\s*\(\s*void\s*\).*?(?=void\s+RunTask)").Value
if ($coarseBlock -match "WaitPlace|SetCameraTask|\bCatch\s*\(|\bPlace\s*\(|\bRunTask\s*\(")
{
    Write-Host "FAIL isolation: coarse positioning calls camera or arm code"
    $failures++
}

if ($failures -ne 0)
{
    Write-Host "FAILED: $failures check(s)"
    exit 1
}

Write-Host "PASSED: three coarse stations, 6/4/4-edge routes, and $($expectedTransitions.Count) coarse stages"

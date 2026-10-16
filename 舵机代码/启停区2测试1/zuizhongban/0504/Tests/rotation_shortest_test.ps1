param(
    [string]$SourcePath = (Join-Path $PSScriptRoot "..\Models\Src\move.c")
)

$ErrorActionPreference = "Stop"

function Normalize-AngleError([int]$ErrorValue)
{
    while ($ErrorValue -gt 180)  { $ErrorValue -= 360 }
    while ($ErrorValue -lt -180) { $ErrorValue += 360 }
    return $ErrorValue
}

$cases = @(
    @{ Current = 350;  Target = 0;    Expected = 10 },
    @{ Current = 540;  Target = 0;    Expected = -180 },
    @{ Current = -270; Target = 90;   Expected = 0 },
    @{ Current = 720;  Target = -90;  Expected = -90 },
    @{ Current = 180;  Target = -180; Expected = 0 },
    @{ Current = 450;  Target = 0;    Expected = -90 },
    @{ Current = 0;    Target = 90;   Expected = 90 },
    @{ Current = 90;   Target = 0;    Expected = -90 }
)

$failures = 0
foreach ($case in $cases)
{
    $actual = Normalize-AngleError ($case.Target - $case.Current)
    if ($actual -ne $case.Expected)
    {
        Write-Host "FAIL math: current=$($case.Current), target=$($case.Target), expected=$($case.Expected), actual=$actual"
        $failures++
    }
}

$source = Get-Content -LiteralPath $SourcePath -Raw -Encoding UTF8
$requiredPatterns = @(
    "static\s+int32_t\s+NormalizeAngleError\s*\(",
    "error\s*=\s*NormalizeAngleError\s*\(\s*\(int32_t\)yaw_desired\s*-\s*\(int32_t\)yaw\s*\)",
    "ROTATION_MAX_CORRECTIONS",
    "ROTATION_MAX_TOTAL_DEGREES"
)

foreach ($pattern in $requiredPatterns)
{
    if ($source -notmatch $pattern)
    {
        Write-Host "FAIL integration: move.c is missing pattern: $pattern"
        $failures++
    }
}

if ($failures -ne 0)
{
    Write-Host "FAILED: $failures check(s)"
    exit 1
}

Write-Host "PASSED: $($cases.Count) signed angle cases and source integration checks"

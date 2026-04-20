$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
$pat = [regex]'\[STAGE4-V5-2B\] off=(-?\d+) v5_score=(\d+) legacy_seed=\((-?\d+),(-?\d+)\)'
$scores = [System.Collections.Generic.List[int64]]::new()
$offs = [System.Collections.Generic.List[int]]::new()
$legI = [System.Collections.Generic.List[int]]::new()
$legQ = [System.Collections.Generic.List[int]]::new()
$reader = [System.IO.File]::OpenText("stepB6_harq_on.log")
try {
    while ($null -ne ($line = $reader.ReadLine())) {
        $m = $pat.Match($line)
        if ($m.Success) {
            [void]$offs.Add([int]$m.Groups[1].Value)
            [void]$scores.Add([int64]$m.Groups[2].Value)
            [void]$legI.Add([int]$m.Groups[3].Value)
            [void]$legQ.Add([int]$m.Groups[4].Value)
        }
    }
}
finally { $reader.Close() }

$csvRows = Import-Csv -Path "HARQ_Matrix_Results_STEPB_ON.csv"
$trialsPerCell = 100
$nExpected = $csvRows.Count * $trialsPerCell
if ($scores.Count -ne $nExpected) {
    Write-Warning "score lines $($scores.Count) vs expected $nExpected"
}

function Get-CellScores([int]$cellIndex) {
    $base = $cellIndex * $trialsPerCell
    $end = [Math]::Min($base + $trialsPerCell, $scores.Count)
    for ($i = $base; $i -lt $end; $i++) { $scores[$i] }
}

function Write-Bucket([string]$title, [scriptblock]$pred) {
    $acc = [System.Collections.Generic.List[int64]]::new()
    for ($i = 0; $i -lt $csvRows.Count; $i++) {
        $row = $csvRows[$i]
        if (& $pred $row) {
            foreach ($s in (Get-CellScores $i)) { [void]$acc.Add($s) }
        }
    }
    Write-Output "## $title"
    if ($acc.Count -eq 0) {
        Write-Output "(no rows)"
        Write-Output ""
        return
    }
    $avg = ($acc | Measure-Object -Average).Average
    $min = ($acc | Measure-Object -Minimum).Minimum
    $max = ($acc | Measure-Object -Maximum).Maximum
    $mean = ($acc | Measure-Object -Average).Average
    $var = 0.0
    foreach ($x in $acc) { $d = [double]$x - $mean; $var += $d * $d }
    $var /= $acc.Count
    $sd = [Math]::Sqrt($var)
    Write-Output "- trial 수: $($acc.Count)"
    Write-Output ("- 평균 v5_score: {0:N1}" -f $avg)
    Write-Output "- 최소: $min"
    Write-Output "- 최대: $max"
    Write-Output ("- 표준편차: {0:N1}" -f $sd)
    Write-Output ""
}

Write-Output "# Step B — v5 Walsh-row DIAG (CSV 행 순서 == 셀 순서 가정)"
Write-Output ""
Write-Bucket "Clean (전체)" { param($r) $r.channel -eq "Clean" }
Write-Bucket "Swept 22 dB" { param($r) $r.channel -eq "Swept" -and $r.intensity -eq "22.00" }
Write-Bucket "Multi 17 dB" { param($r) $r.channel -eq "MultiTone" -and $r.intensity -eq "17.00" }
Write-Bucket "AWGN -16 dB" { param($r) $r.channel -eq "AWGN" -and $r.intensity -eq "-16.00" }
Write-Bucket "Barrage 18 dB_JSR" { param($r) $r.channel -eq "Barrage" -and $r.intensity -eq "18.00" }

$n = [Math]::Min($scores.Count, $legI.Count)
$mag2 = New-Object "double[]" $n
for ($i = 0; $i -lt $n; $i++) {
    $li = [double]$legI[$i]
    $lq = [double]$legQ[$i]
    $mag2[$i] = $li * $li + $lq * $lq
}
$meanS = ($scores[0..($n-1)] | Measure-Object -Average).Average
$meanM = ($mag2 | Measure-Object -Average).Average
$vS = 0.0; $vM = 0.0
for ($i = 0; $i -lt $n; $i++) {
    $ds = [double]$scores[$i] - $meanS
    $dm = $mag2[$i] - $meanM
    $vS += $ds * $ds; $vM += $dm * $dm
}
$vS /= $n; $vM /= $n
$sdS = [Math]::Sqrt($vS); $sdM = [Math]::Sqrt($vM)
$cov = 0.0
for ($i = 0; $i -lt $n; $i++) {
    $cov += ([double]$scores[$i] - $meanS) * ($mag2[$i] - $meanM)
}
$cov /= $n
$r = if ($sdS -gt 1e-12 -and $sdM -gt 1e-12) { $cov / ($sdS * $sdM) } else { [double]::NaN }
Write-Output "## Legacy seed vs v5_score (전체 trial)"
Write-Output ("- 피어슨 상관 (v5_score, |legacy|^2): {0:N4}" -f $r)

$vmin = ($scores | Measure-Object -Minimum).Minimum
$vmax = ($scores | Measure-Object -Maximum).Maximum
Write-Output ""
Write-Output "## 전체 v5_score 범위"
Write-Output "- 최소: $vmin"
Write-Output "- 최대: $vmax"
Write-Output "- 라인 수: $($scores.Count)"

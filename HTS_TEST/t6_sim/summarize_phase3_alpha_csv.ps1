param([Parameter(Mandatory=$true)][string]$CsvPath)
$ErrorActionPreference = 'Stop'
$rows = Import-Csv $CsvPath
function Zone([double]$p) {
    if ($p -ge 0.90) { 'PASS' } elseif ($p -ge 0.50) { 'RISK' } else { 'FAIL' }
}
$nPass = 0; $nRisk = 0; $nFail = 0
$jamPass = @{}
foreach ($r in $rows) {
    $p = [double]$r.success_rate
    $z = Zone $p
    if ($z -eq 'PASS') { $nPass++ } elseif ($z -eq 'RISK') { $nRisk++ } else { $nFail++ }
    if ($z -eq 'PASS') {
        $j = $r.jam
        if (-not $jamPass.ContainsKey($j)) { $jamPass[$j] = 0 }
        $jamPass[$j]++
    }
}
$jsrMean = @{}
foreach ($jsr in ($rows | Select-Object -ExpandProperty jsr_db -Unique)) {
    $ji = [int]$jsr
    $sub = $rows | Where-Object { [int]$_.jsr_db -eq $ji }
    $vals = @($sub | ForEach-Object { [double]$_.success_rate })
    $m = 0.0
    if ($vals.Count -gt 0) { $m = ($vals | Measure-Object -Average).Average }
    $jsrMean[$ji] = $m
}
Write-Host "cells=$($rows.Count) PASS>=0.9: $nPass RISK: $nRisk FAIL: $nFail"
Write-Host "per_jam PASS_cells:"
$jamPass.GetEnumerator() | Sort-Object Name | ForEach-Object { Write-Host "  $($_.Key): $($_.Value)" }
Write-Host "JSR mean success_rate:"
$jsrMean.GetEnumerator() | Sort-Object Name | ForEach-Object { Write-Host "  $($_.Key) dB: $([math]::Round($_.Value, 4))" }

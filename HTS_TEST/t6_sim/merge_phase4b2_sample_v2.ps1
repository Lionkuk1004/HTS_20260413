$ErrorActionPreference = 'Stop'
$base = Import-Csv -Path 'stepC4_sample_v2_baseline.csv'
$mask = Import-Csv -Path 'stepC4_sample_v2_mask.csv'
$rows = @()
$maxDelta = 0.0
$maxDeltaNone = 0.0
foreach ($b in $base) {
    $m = $mask | Where-Object {
        $_.jam -eq $b.jam -and
        [int]$_.cov_pct -eq [int]$b.cov_pct -and
        [int]$_.jsr_db -eq [int]$b.jsr_db -and
        $_.lpi -eq $b.lpi
    } | Select-Object -First 1
    if (-not $m) { throw "No mask row for $($b.jam) $($b.cov_pct) $($b.jsr_db) $($b.lpi)" }
    $pb = [double]$b.success_rate
    $pm = [double]$m.success_rate
    $delta = ($pm - $pb) * 100.0
    $ad = [math]::Abs($delta)
    if ($ad -gt $maxDelta) { $maxDelta = $ad }
    if ($b.lpi -eq 'NONE' -and $ad -gt $maxDeltaNone) { $maxDeltaNone = $ad }
    $rows += [pscustomobject]@{
        jam                 = $b.jam
        cov_pct             = [int]$b.cov_pct
        jsr_db              = [int]$b.jsr_db
        lpi                 = $b.lpi
        baseline_pass_rate  = $pb
        mask_pass_rate      = $pm
        delta_pct_points    = [math]::Round($delta, 4)
        zone_baseline       = $b.zone
        zone_mask           = $m.zone
    }
}
$rows | Export-Csv -Path 'sample_v2_compare.csv' -NoTypeInformation -Encoding UTF8
Write-Host "v2 ALL max |delta|: $([math]::Round($maxDelta, 4)) %p"
Write-Host "v2 LPI=NONE max |delta|: $([math]::Round($maxDeltaNone, 4)) %p"

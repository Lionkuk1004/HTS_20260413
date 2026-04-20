# Step C-1: S3 SNR waterfall — v5_score distribution from T6 V5 log (streaming).
# Rule: row() prints "SNR ... dB" AFTER each SNR's trials; STAGE4 lines before
# that row belong to that SNR. Flush pending on each SNR row; on S4 hdr flush to last SNR.

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$LOG_PATH = "stepC1_4_t6_v5.log"
$OUT_TXT = "stepC1_5_s3_v5_stats.txt"
$OUT_CSV = "stepC1_5_s3_v5_stats.csv"

if (-not (Test-Path $LOG_PATH)) {
    Write-Error "Missing $LOG_PATH"
    exit 1
}

$reS3Hdr = [regex]'│\s*S3\s'
$reS4Hdr = [regex]'│\s*S4\s'
$reSnrRow = [regex]'SNR\s+([+-]?\d+)\s*dB'
$reV5 = [regex]'\[STAGE4-V5-2B\]\s+off=(\d+)\s+v5_score=(\d+)\s+legacy_seed=\(([+-]?\d+),([+-]?\d+)\)'

$data = @{}  # snr -> list of [off, score, si, sq]
function Add-Pending([System.Collections.Generic.List[object]]$pending, [int]$snr) {
    if (-not $data.ContainsKey($snr)) { $data[$snr] = [System.Collections.Generic.List[object]]::new() }
    foreach ($p in $pending) { [void]$data[$snr].Add($p) }
    $pending.Clear()
}

$inS3 = $false
$pending = [System.Collections.Generic.List[object]]::new()
$reader = [System.IO.File]::OpenText($LOG_PATH)
try {
    while ($null -ne ($line = $reader.ReadLine())) {
        if ($reS3Hdr.IsMatch($line)) {
            $inS3 = $true
            continue
        }
        if ($reS4Hdr.IsMatch($line)) {
            if ($inS3 -and $pending.Count -gt 0) {
                # Should not happen if last SNR row flushed; assign to +10 if missing
                Add-Pending $pending 10
            }
            $inS3 = $false
            continue
        }
        if (-not $inS3) { continue }

        $mS = $reSnrRow.Match($line)
        if ($mS.Success) {
            $snrVal = [int]$mS.Groups[1].Value
            Add-Pending $pending $snrVal
            continue
        }

        $mV = $reV5.Match($line)
        if ($mV.Success) {
            [void]$pending.Add(@(
                    [int][int64]$mV.Groups[1].Value,
                    [int64]$mV.Groups[2].Value,
                    [int][int64]$mV.Groups[3].Value,
                    [int][int64]$mV.Groups[4].Value))
        }
    }
}
finally { $reader.Close() }

if ($data.Count -eq 0) {
    Write-Error "No S3 v5 data parsed (markers or log path)."
    exit 1
}

function Mean([double[]]$xs) {
    if ($xs.Length -eq 0) { return 0.0 }
    ($xs | Measure-Object -Average).Average
}

function StDevPop([double[]]$xs) {
    if ($xs.Length -le 1) { return 0.0 }
    $m = (Mean $xs)
    $v = 0.0
    foreach ($x in $xs) { $d = $x - $m; $v += $d * $d }
    [Math]::Sqrt($v / $xs.Length)
}

function MedianSorted([double[]]$sorted) {
    $n = $sorted.Length
    if ($n -eq 0) { return 0.0 }
    if (($n % 2) -eq 1) { return $sorted[[int](($n - 1) / 2)] }
    $mid = [int]($n / 2)
    return 0.5 * ($sorted[$mid - 1] + $sorted[$mid])
}

$snrKeys = ($data.Keys | ForEach-Object { [int]$_ } | Sort-Object)
$results = @()
foreach ($snr in $snrKeys) {
    $vals = $data[$snr]
    $scores = @($vals | ForEach-Object { [double]$_[1] })
    $nonZero = @($scores | Where-Object { $_ -gt 0 })
    $nTot = $scores.Length
    $nNz = $nonZero.Length
    $meanNz = (Mean $nonZero)
    $sorted = @($nonZero | Sort-Object)
    $med = (MedianSorted $sorted)
    $sd = (StDevPop $nonZero)
    $maxS = if ($nNz -gt 0) { ($nonZero | Measure-Object -Maximum).Maximum } else { 0.0 }

    $xsCorr = [System.Collections.Generic.List[double]]::new()
    $ysCorr = [System.Collections.Generic.List[double]]::new()
    foreach ($v in $vals) {
        $sc = [double][int64]$v[1]
        if ($sc -gt 0) {
            $si = [double][int]$v[2]; $sq = [double][int]$v[3]
            [void]$xsCorr.Add($sc)
            [void]$ysCorr.Add($si * $si + $sq * $sq)
        }
    }
    $corr = 0.0
    if ($xsCorr.Count -ge 2) {
        $xs = $xsCorr.ToArray()
        $ys = $ysCorr.ToArray()
        $mx = (Mean $xs); $my = (Mean $ys)
        $sx = (StDevPop $xs); $sy = (StDevPop $ys)
        if ($sx -gt 1e-12 -and $sy -gt 1e-12) {
            $cov = 0.0
            for ($i = 0; $i -lt $xs.Length; $i++) {
                $cov += ($xs[$i] - $mx) * ($ys[$i] - $my)
            }
            $cov /= $xs.Length
            $corr = $cov / ($sx * $sy)
        }
    }

    $results += [pscustomobject]@{
        snr          = $snr
        n_total      = $nTot
        n_nonzero    = $nNz
        mean_nonzero = $meanNz
        median_nz    = $med
        stdev_nz     = $sd
        max_score    = $maxS
        corr_legacy  = $corr
    }
}

# --- text report ---
$sb = [System.Text.StringBuilder]::new()
[void]$sb.AppendLine(("=" * 80))
[void]$sb.AppendLine("Step C-1 S3 SNR Waterfall - v5_score (non-zero stats)")
[void]$sb.AppendLine(("=" * 80))
[void]$sb.AppendLine(("`n{0,7} {1,9} {2,9} {3,14} {4,14} {5,14} {6,14} {7,12}" -f `
            "SNR", "n_total", "n_nz", "mean_nz", "median_nz", "stdev_nz", "max", "corr"))
[void]$sb.AppendLine(("-") * 80)
foreach ($r in $results) {
    $line = "{0,7} {1,9} {2,9} {3,14:E3} {4,14:E3} {5,14:E3} {6,14:E3} {7,12:F3}" -f `
        $r.snr, $r.n_total, $r.n_nonzero, $r.mean_nonzero, $r.median_nz, $r.stdev_nz, $r.max_score, $r.corr_legacy
    [void]$sb.AppendLine($line)
}

$ref = $results | Where-Object { $_.mean_nonzero -gt 0 } | Sort-Object { $_.snr } | Select-Object -First 1
[void]$sb.AppendLine("`n" + ("=" * 80))
[void]$sb.AppendLine("LPI: ratio vs lowest-SNR reference mean (non-zero; first SNR with mean>0)")
[void]$sb.AppendLine(("=" * 80))
if ($ref -and $ref.mean_nonzero -gt 0) {
    [void]$sb.AppendLine("Reference SNR=$($ref.snr) dB mean_nz=$($ref.mean_nonzero.ToString('E3'))")
    [void]$sb.AppendLine(("{0,7} {1,16} {2,18} {3,10}" -f "SNR", "mean_nz", "ratio_vs_ref", "loss_dB"))
    [void]$sb.AppendLine(("-") * 60)
    $refm = $ref.mean_nonzero
    foreach ($r in $results) {
        $ratio = if ($refm -gt 0) { $r.mean_nonzero / $refm } else { 0 }
        $loss = if ($ratio -gt 0) { -10.0 * [Math]::Log10($ratio) } else { [double]::PositiveInfinity }
        [void]$sb.AppendLine(("{0,7} {1,16:E3} {2,18:F4} {3,10:F2}" -f $r.snr, $r.mean_nonzero, $ratio, $loss))
    }
}

[System.IO.File]::WriteAllText((Join-Path $PSScriptRoot $OUT_TXT), $sb.ToString(), [System.Text.UTF8Encoding]::new($false))

# CSV
$csv = New-Object System.Text.StringBuilder
[void]$csv.AppendLine("snr_dB,n_total,n_nonzero,mean_nonzero,median_nonzero,stdev_nonzero,max_score,corr_legacy")
$cInv = [System.Globalization.CultureInfo]::InvariantCulture
foreach ($r in $results) {
    $line = "{0},{1},{2},{3},{4},{5},{6},{7}" -f $r.snr, $r.n_total, $r.n_nonzero,
    $r.mean_nonzero.ToString("E3", $cInv), $r.median_nz.ToString("E3", $cInv),
    $r.stdev_nz.ToString("E3", $cInv), $r.max_score.ToString("E3", $cInv),
    $r.corr_legacy.ToString("F3", $cInv)
    [void]$csv.AppendLine($line)
}
[System.IO.File]::WriteAllText((Join-Path $PSScriptRoot $OUT_CSV), $csv.ToString(), [System.Text.UTF8Encoding]::new($false))

Write-Host "Wrote $OUT_TXT and $OUT_CSV"
foreach ($r in $results) {
    Write-Host ("  SNR {0,4} dB: nz={1}/{2} mean_nz={3:E2}" -f $r.snr, $r.n_nonzero, $r.n_total, $r.mean_nonzero)
}

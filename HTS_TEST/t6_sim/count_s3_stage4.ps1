$log = "stepC1_4_t6_v5.log"
$in = $false
$n = 0
foreach ($line in Get-Content $log) {
    if ($line -match '│\s*S3\s') { $in = $true; continue }
    if ($in -and ($line -match '│\s*S4\s')) { break }
    if ($in -and ($line -match '\[STAGE4-V5-2B\]')) { $n++ }
}
Write-Host "S3_STAGE4_lines=$n"

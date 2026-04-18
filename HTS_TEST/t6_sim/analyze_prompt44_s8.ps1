$ErrorActionPreference = 'Stop'
$log = 'D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\t6_v34_fixa3.log'
$c = Get-Content -LiteralPath $log

$bP8_10  = ($c | Select-String -Pattern 'JSR\+10\s+P=8'  | Select-Object -First 1).LineNumber
$bP8_15  = ($c | Select-String -Pattern 'JSR\+15\s+P=8'  | Select-Object -First 1).LineNumber
$bP8_20  = ($c | Select-String -Pattern 'JSR\+20\s+P=8'  | Select-Object -First 1).LineNumber
$bP12_15 = ($c | Select-String -Pattern 'JSR\+15\s+P=12' | Select-Object -First 1).LineNumber
$bP16_20 = ($c | Select-String -Pattern 'JSR\+20\s+P=16' | Select-Object -First 1).LineNumber

Write-Host "P=8  +10=$bP8_10  +15=$bP8_15  +20=$bP8_20"
Write-Host "P=12 +15=$bP12_15"
Write-Host "P=16 +20=$bP16_20"

function Get-Seg($a, $b) {
    $start = $a + 1
    $cnt = $b - $a - 1
    if ($cnt -le 0) { return @() }
    return @($c | Select-Object -Skip ($start - 1) -First $cnt)
}

$seg_p8_20  = Get-Seg $bP8_15 $bP8_20
$seg_p12_15 = Get-Seg $bP8_20 $bP12_15
$seg_p16_20 = Get-Seg $bP12_15 $bP16_20

Write-Host "`n=== S8 JSR+15 P=12: [P0-FWHT-RAW] block2 non-all-zero top5 (first 8) ==="
$seg_p12_15 | Select-String -Pattern '\[P0-FWHT-RAW\]\s+block2:' |
    Where-Object { $_.Line -notmatch 'bin0=0 bin1=0 bin2=0 bin3=0 bin4=0' } |
    Select-Object -First 8 | ForEach-Object { $_.Line }

Write-Host "`n=== S8 JSR+15 P=12: [P0-BLOCK-TOTAL] block=2 non-zero (first 8) ==="
$seg_p12_15 | Select-String -Pattern '\[P0-BLOCK-TOTAL\]\s+block=2' |
    Where-Object { $_.Line -notmatch 'total_e=0' } |
    Select-Object -First 8 | ForEach-Object { $_.Line }

Write-Host "`n=== S8 JSR+15 P=12: [P0-PINPOINT] block=2 non-all-zero (first 8) ==="
$seg_p12_15 | Select-String -Pattern '\[P0-PINPOINT\]\s+block=2' |
    Where-Object { $_.Line -notmatch 'e_bin0=0 e_bin6=0 e_bin63=0' } |
    Select-Object -First 8 | ForEach-Object { $_.Line }

Write-Host "`n=== S8 JSR+15 P=12: [FIXA3] (first 8) ==="
$seg_p12_15 | Select-String -Pattern '\[FIXA3\]' | Select-Object -First 8 | ForEach-Object { $_.Line }

Write-Host "`n=== S8 JSR+15 P=12: [STAGE4-P0] (first 8) ==="
$seg_p12_15 | Select-String -Pattern '\[STAGE4-P0\]' | Select-Object -First 8 | ForEach-Object { $_.Line }

Write-Host "`n=== S8 JSR+20 P=16: [P0-FWHT-RAW] block2 non-all-zero (first 8) ==="
$seg_p16_20 | Select-String -Pattern '\[P0-FWHT-RAW\]\s+block2:' |
    Where-Object { $_.Line -notmatch 'bin0=0 bin1=0 bin2=0 bin3=0 bin4=0' } |
    Select-Object -First 8 | ForEach-Object { $_.Line }

Write-Host "`n=== S8 JSR+20 P=16: [P0-BLOCK-TOTAL] block=2 non-zero (first 8) ==="
$seg_p16_20 | Select-String -Pattern '\[P0-BLOCK-TOTAL\]\s+block=2' |
    Where-Object { $_.Line -notmatch 'total_e=0' } |
    Select-Object -First 8 | ForEach-Object { $_.Line }

Write-Host "`n=== S8 JSR+20 P=16: [FIXA3] (first 8) ==="
$seg_p16_20 | Select-String -Pattern '\[FIXA3\]' | Select-Object -First 8 | ForEach-Object { $_.Line }

Write-Host "`n=== S8 JSR+20 P=16: [STAGE4-P0] (first 8) ==="
$seg_p16_20 | Select-String -Pattern '\[STAGE4-P0\]' | Select-Object -First 8 | ForEach-Object { $_.Line }

Write-Host "`n=== Ref S8 JSR+20 P=8 PASS: [P0-FWHT-RAW] block2 non-all-zero (first 5) ==="
$seg_p8_20 | Select-String -Pattern '\[P0-FWHT-RAW\]\s+block2:' |
    Where-Object { $_.Line -notmatch 'bin0=0 bin1=0 bin2=0 bin3=0 bin4=0' } |
    Select-Object -First 5 | ForEach-Object { $_.Line }

$tags = @('P0-FWHT-RAW','P0-BLOCK-TOTAL','P0-PINPOINT','FIXA3','STAGE4-P0','P1-NC','HDR-TMPL','HDR-SHIFT','PAYLOAD-SHIFT')

Write-Host "`n=== DIAG counts by segment ==="
foreach ($pair in @(
    @('S8 JSR+15 P=12', $seg_p12_15),
    @('S8 JSR+20 P=16', $seg_p16_20),
    @('S8 JSR+20 P=8 (ref)', $seg_p8_20)
)) {
    $name = $pair[0]
    $seg = $pair[1]
    Write-Host "`n--- $name ---"
    foreach ($tag in $tags) {
        $cnt = ($seg | Select-String -Pattern "\[$tag").Count
        Write-Host ("    [{0,-16}] = {1}" -f $tag, $cnt)
    }
}

$ErrorActionPreference = 'Stop'
$log = 'D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\t6_v33_fixa2.log'
$c = Get-Content -LiteralPath $log

function Analyze-Segment {
    param(
        [int] $startLine,
        [int] $endLine,
        [string] $name
    )
    if ($endLine -le $startLine + 1) {
        Write-Host ("{0,-24}  (empty range)" -f $name)
        return
    }
    $start = $startLine + 1
    $cnt = $endLine - $startLine - 1
    $seg = $c | Select-Object -Skip ($start - 1) -First $cnt
    $fixa2 = ($seg | Select-String '\[FIXA2\]').Count
    $fixa2cj = ($seg | Select-String '\[FIXA2\].*cj=1').Count
    $p0raw = ($seg | Select-String '\[P0-FWHT-RAW\]').Count
    $payshift = ($seg | Select-String '\[PAYLOAD-SHIFT\]').Count
    $hdrtmpl = ($seg | Select-String '\[HDR-TMPL\]').Count
    $p1nc = ($seg | Select-String '\[P1-NC\]').Count
    Write-Host (
        "{0,-24}  FIXA2={1,4} cj1={2,4}  P0raw={3,5}  P1NC={4,6}  HDR-TMPL={5,5}  PAYSHIFT={6,5}" -f `
            $name, $fixa2, $fixa2cj, $p0raw, $p1nc, $hdrtmpl, $payshift
    )
}

Write-Host '=== V33 fixa2 log: segment DIAG counts (lines strictly between waterfall boundaries) ==='
Write-Host ''
Analyze-Segment 49682 50683 'S7 +5dB (ref)'
Analyze-Segment 50683 51843 'S7 +10dB'
Analyze-Segment 51843 53902 'S7 +15dB'
Analyze-Segment 53902 57401 'S7 +20/+25dB'
Analyze-Segment 64161 65184 'S8 JSR+10 P=8 PASS'
Analyze-Segment 65184 73067 'S8 JSR+15 P=8'
Analyze-Segment 73067 80950 'S8 JSR+20 P=8'
Analyze-Segment 80950 88833 'S8 JSR+15 P=12'
Analyze-Segment 96718 98995 'S9 Full'

Write-Host ''
Write-Host '=== S7 +15dB [P0-FWHT-RAW] first 9 lines (block triplets) ==='
$s715s = 51843 + 1
$s715c = 53902 - 51843 - 1
($c | Select-Object -Skip ($s715s - 1) -First $s715c | Select-String '\[P0-FWHT-RAW\]' | Select-Object -First 9).Line

Write-Host ''
Write-Host '=== S8 JSR+15 P=8 [P0-FWHT-RAW] first 9 lines ==='
$s815s = 65184 + 1
$s815c = 73067 - 65184 - 1
($c | Select-Object -Skip ($s815s - 1) -First $s815c | Select-String '\[P0-FWHT-RAW\]' | Select-Object -First 9).Line

Write-Host ''
Write-Host '=== S8 JSR+15 P=8 [FIXA2] first 5 lines ==='
($c | Select-Object -Skip ($s815s - 1) -First $s815c | Select-String '\[FIXA2\]' | Select-Object -First 5).Line

Write-Host ''
Write-Host '=== S9 Full [P0-FWHT-RAW] first 9 lines ==='
$s9s = 96718 + 1
$s9c = 98995 - 96718 - 1
($c | Select-Object -Skip ($s9s - 1) -First $s9c | Select-String '\[P0-FWHT-RAW\]' | Select-Object -First 9).Line

Write-Host ''
Write-Host '=== S9 Full [FIXA2] first 5 lines ==='
($c | Select-Object -Skip ($s9s - 1) -First $s9c | Select-String '\[FIXA2\]' | Select-Object -First 5).Line

Write-Host ''
Write-Host '=== S9 Full [PAYLOAD-SHIFT] first 5 lines ==='
($c | Select-Object -Skip ($s9s - 1) -First $s9c | Select-String '\[PAYLOAD-SHIFT\]' | Select-Object -First 5).Line

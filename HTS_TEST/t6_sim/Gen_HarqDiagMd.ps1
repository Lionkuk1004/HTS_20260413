#Requires -Version 5.1
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$csvPath = Join-Path $here 'HARQ_Matrix_Results.csv'
$outMd = Join-Path $here 'HARQ_DIAG_ANALYSIS.md'
$logA3 = Join-Path $here 'stepA3_t6.log'
$logA6 = Join-Path $here 'stepA6_t6_final.log'

function Get-QuantLine([string]$path) {
    if (-not (Test-Path $path)) { return '(log missing)' }
    $m = Select-String -Path $path -Pattern '정량 합계:\s*(\d+)\s*/\s*(\d+)' -Encoding UTF8 |
        Select-Object -Last 1
    if (-not $m) { return '(pattern not found)' }
    return $m.Matches[0].Groups[1].Value + ' / ' + $m.Matches[0].Groups[2].Value
}

$rows = Import-Csv -Path $csvPath -Encoding UTF8
if ($rows.Count -ne 260) { Write-Warning "Expected 260 data rows, got $($rows.Count)" }

function Md-Escape([string]$s) {
    if ($null -eq $s) { return '' }
    return ($s -replace '\|', '\|')
}

$sb = [System.Text.StringBuilder]::new()
[void]$sb.AppendLine('# HARQ Matrix DIAG 분석 (Step A 실측 결과)')
[void]$sb.AppendLine('')
[void]$sb.AppendLine('## 1. 회귀 기준')
$g0 = '10927 / 11200'
$gA3 = Get-QuantLine $logA3
$gA6 = if (Test-Path $logA6) { Get-QuantLine $logA6 } else { '(stepA6 not run yet)' }
[void]$sb.AppendLine("- Gate 0: $g0")
$a3ok = if ($gA3 -eq $g0) { '유지' } else { '비유지' }
$a6ok = if ($gA6 -eq $g0) { '유지' } else { '비유지' }
[void]$sb.AppendLine("- Gate A-3: $gA3 ($a3ok)")
[void]$sb.AppendLine("- Gate A-6: $gA6 ($a6ok)")
[void]$sb.AppendLine('')

[void]$sb.AppendLine('## 2. 260 cell 원본 결과 (CSV 전부)')
[void]$sb.AppendLine('')
$cols = $rows[0].PSObject.Properties.Name
[void]$sb.AppendLine('| ' + ($cols -join ' | ') + ' |')
[void]$sb.AppendLine('|' + (($cols | ForEach-Object { '---' }) -join '|') + '|')
foreach ($r in $rows) {
    $cells = foreach ($c in $cols) { Md-Escape([string]$r.$c) }
    [void]$sb.AppendLine('| ' + ($cells -join ' | ') + ' |')
}
[void]$sb.AppendLine('')

function Add-ChannelTable([string]$name, [scriptblock]$filterScript) {
    [void]$sb.AppendLine("### $name")
    [void]$sb.AppendLine('| fec_path | mode | intensity | crc_ok | total | pre | hdr | dec | P_s_pay | P_s_pre | P_after | jsr_meas_db | 해석 |')
    [void]$sb.AppendLine('| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |')
    $sub = @($rows | Where-Object -FilterScript $filterScript | Sort-Object { [double]$_.intensity })
    foreach ($x in $sub) {
        [void]$sb.AppendLine("| $($x.fec_path) | $($x.mode) | $($x.intensity) | $($x.crc_ok) | $($x.total) | $($x.pre) | $($x.hdr) | $($x.dec) | $($x.P_s_pay) | $($x.P_s_pre) | $($x.P_after) | $($x.jsr_meas_db) | - |")
    }
    [void]$sb.AppendLine('')
}

[void]$sb.AppendLine('## 3. 채널별 성능 한계 (cliff 식별)')
[void]$sb.AppendLine('')
Add-ChannelTable 'AWGN (SNR, intensity_unit=dB_SNR)' { $_.channel -eq 'AWGN' }
Add-ChannelTable 'Barrage (dB_JSR)' { $_.channel -eq 'Barrage' }
Add-ChannelTable 'CW (dB_JSR)' { $_.channel -eq 'CW' }
Add-ChannelTable 'Pulse (dB_JSR)' { $_.channel -eq 'Pulse' }
Add-ChannelTable 'Multi (dB_JSR)' { $_.channel -eq 'Multi' }
Add-ChannelTable 'Swept (dB_JSR)' { $_.channel -eq 'Swept' }
Add-ChannelTable 'Partial_Barrage (dB_JSR)' { $_.channel -eq 'Partial_Barrage' }
Add-ChannelTable 'Clean' { $_.channel -eq 'Clean' }

[void]$sb.AppendLine('## 4. 병목 단계 맵')
[void]$sb.AppendLine('')
$pre0 = @($rows | Where-Object { [int]$_.pre -eq 0 } | ForEach-Object { "$($_.fec_path),$($_.mode),$($_.channel),$($_.intensity)" })
$ph = @($rows | Where-Object { [int]$_.pre -ge 1 -and [int]$_.hdr -eq 0 } | ForEach-Object { "$($_.fec_path),$($_.mode),$($_.channel),$($_.intensity)" })
$hd = @($rows | Where-Object { [int]$_.hdr -ge 1 -and [int]$_.dec -eq 0 } | ForEach-Object { "$($_.fec_path),$($_.mode),$($_.channel),$($_.intensity)" })
$dc = @($rows | Where-Object { [int]$_.dec -ge 1 -and [int]$_.crc_ok -lt [int]$_.total } | ForEach-Object { "$($_.fec_path),$($_.mode),$($_.channel),$($_.intensity) crc=$($_.crc_ok)/$($_.total)" })
[void]$sb.AppendLine('- pre=0/10 cell (pre 열=0): ' + $(if ($pre0.Count) { ($pre0 -join '; ') } else { '(없음)' }))
[void]$sb.AppendLine('- pre>=1 hdr=0 cell: ' + $(if ($ph.Count) { ($ph -join '; ') } else { '(없음)' }))
[void]$sb.AppendLine('- hdr>=1 dec=0 cell: ' + $(if ($hd.Count) { ($hd -join '; ') } else { '(없음)' }))
[void]$sb.AppendLine('- dec>=1 crc<total cell: ' + $(if ($dc.Count) { ($dc -join '; ') } else { '(없음)' }))
[void]$sb.AppendLine('')

[void]$sb.AppendLine('## 5. 재밍 실효값 검증')
[void]$sb.AppendLine('')
$jsrRows = @($rows | Where-Object { $_.intensity_unit -match 'JSR' })
$diffList = @()
foreach ($x in $jsrRows) {
    $intd = [double]$x.intensity
    $meas = [double]$x.jsr_meas_db
    if ($meas -le -90.0) { continue }
    $d = [math]::Abs($intd - $meas)
    if ($d -ge 3.0) {
        $diffList += "$($x.fec_path),$($x.mode),$($x.channel),intended=$intd dB, jsr_meas=$meas dB, delta=$([math]::Round($d,2)) dB"
    }
}
[void]$sb.AppendLine('| intended vs measured |delta|>=3 dB cell 목록')
[void]$sb.AppendLine('| --- |')
foreach ($line in $diffList) { [void]$sb.AppendLine("| $line |") }
if (-not $diffList.Count) { [void]$sb.AppendLine('| (해당 없음) |') }
[void]$sb.AppendLine('')
$preVals = @($rows | ForEach-Object { [double]$_.P_s_pre })
$preMin = ($preVals | Measure-Object -Minimum).Minimum
$preMax = ($preVals | Measure-Object -Maximum).Maximum
[void]$sb.AppendLine("- P_s_pre (프리앰블 구간 전력 평균) 전 cell min=$preMin max=$preMax (동일 cell 간 변동: min!=max → $($preMin -ne $preMax))")
[void]$sb.AppendLine('')

[void]$sb.AppendLine('## 6. T6 S3/S7/S8 교차 검증 (로그에서 원문 추출)')
[void]$sb.AppendLine('')
function Grab-Section([string]$log, [string]$tag) {
    if (-not (Test-Path $log)) { return "(missing $log)" }
    $all = Get-Content -Path $log -Encoding UTF8 -ErrorAction SilentlyContinue
    $lines = @($all | Select-String -Pattern $tag | Select-Object -First 25)
    if (-not $lines.Count) { return "(no lines matching $tag)" }
    return ($lines | ForEach-Object { $_.Line }) -join "`n"
}
[void]$sb.AppendLine('### stepA3_t6.log — S3')
[void]$sb.AppendLine('```')
[void]$sb.AppendLine((Grab-Section $logA3 'S3'))
[void]$sb.AppendLine('```')
[void]$sb.AppendLine('')
[void]$sb.AppendLine('### stepA3_t6.log — S7')
[void]$sb.AppendLine('```')
[void]$sb.AppendLine((Grab-Section $logA3 'S7'))
[void]$sb.AppendLine('```')
[void]$sb.AppendLine('')
[void]$sb.AppendLine('### stepA3_t6.log — S8')
[void]$sb.AppendLine('```')
[void]$sb.AppendLine((Grab-Section $logA3 'S8'))
[void]$sb.AppendLine('```')
[void]$sb.AppendLine('')
[void]$sb.AppendLine('### HARQ Matrix — chase + DATA 만 (강도별 crc_ok/total)')
$aw = @($rows | Where-Object { $_.fec_path -eq 'chase' -and $_.mode -eq 'DATA' -and $_.channel -eq 'AWGN' } | Sort-Object { [double]$_.intensity })
foreach ($x in $aw) { [void]$sb.AppendLine("- SNR=$($x.intensity) dB: crc $($x.crc_ok)/$($x.total), pre=$($x.pre), hdr=$($x.hdr), dec=$($x.dec), jsr_meas_db=$($x.jsr_meas_db)") }
$br = @($rows | Where-Object { $_.fec_path -eq 'chase' -and $_.mode -eq 'DATA' -and $_.channel -eq 'Barrage' } | Sort-Object { [double]$_.intensity })
[void]$sb.AppendLine('')
foreach ($x in $br) { [void]$sb.AppendLine("- Barrage JSR=$($x.intensity) dB: crc $($x.crc_ok)/$($x.total), jsr_meas_db=$($x.jsr_meas_db)") }
$cw = @($rows | Where-Object { $_.fec_path -eq 'chase' -and $_.mode -eq 'DATA' -and $_.channel -eq 'CW' } | Sort-Object { [double]$_.intensity })
[void]$sb.AppendLine('')
foreach ($x in $cw) { [void]$sb.AppendLine("- CW JSR=$($x.intensity) dB: crc $($x.crc_ok)/$($x.total), jsr_meas_db=$($x.jsr_meas_db)") }
[void]$sb.AppendLine('')

[void]$sb.AppendLine('## 7. 실측 근거 있는 발견')
[void]$sb.AppendLine('')
$clean = @($rows | Where-Object { $_.channel -eq 'Clean' })
$all10 = ($clean | ForEach-Object { [int]$_.crc_ok -eq 10 -and [int]$_.pre -eq 10 -and [int]$_.hdr -eq 10 -and [int]$_.dec -eq 10 } | Where-Object { $_ } | Measure-Object).Count
[void]$sb.AppendLine("- Clean 채널 cell 수=$($clean.Count), 그 중 crc=10 pre=10 hdr=10 dec=10 동시 만족 cell 수=$all10")
[void]$sb.AppendLine("- 전체 cell 수=$($rows.Count), CSV 데이터 행 기준")
[void]$sb.AppendLine('')

[void]$sb.AppendLine('## 8. 추가 조사 필요 항목')
[void]$sb.AppendLine('')
[void]$sb.AppendLine('- 본 CSV의 jsr_meas_db는 (P_after-P_s)/P_s 의 10*log10 추정이며, AWGN의 intensity(dB_SNR)와 동일 스케일 아님.')
[void]$sb.AppendLine('- T6 S3/S7/S8 로그 원문과 HARQ Matrix 수치는 섹션 6에 병기됨(자동 비교 수식 없음).')
[void]$sb.AppendLine('')

[void]$sb.AppendLine('## 9. 파일 목록')
[void]$sb.AppendLine('')
[void]$sb.AppendLine('- HARQ_Matrix_Results.csv')
[void]$sb.AppendLine('- stepA4_harq.log')
[void]$sb.AppendLine('- stepA3_t6.log, stepA6_t6_final.log')
[void]$sb.AppendLine('')

[System.IO.File]::WriteAllText($outMd, $sb.ToString(), [System.Text.UTF8Encoding]::new($true))
Write-Host "Wrote $outMd"

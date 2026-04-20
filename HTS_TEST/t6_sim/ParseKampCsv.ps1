$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$newCsv = Join-Path $here 'HARQ_Matrix_Results.csv'
$oldCsv = Join-Path $here 'HARQ_Matrix_Results_phaseA_k1000.csv'

function Read-Rows([string]$path) {
    if (-not (Test-Path $path)) { return @() }
    return @(Import-Csv -Path $path -Encoding UTF8)
}

$n = Read-Rows $newCsv
Write-Host "new_rows $($n.Count)"
$clean = @($n | Where-Object { $_.channel -eq 'Clean' })
foreach ($c in $clean) {
    Write-Host "Clean $($c.fec_path) $($c.mode) crc=$($c.crc_ok)/$($c.total) pre=$($c.pre) hdr=$($c.hdr) dec=$($c.dec) P_s_pay=$($c.P_s_pay) kAmp=$($c.kAmp)"
}
$maxP = ($n | ForEach-Object { [double]$_.P_after } | Measure-Object -Maximum).Maximum
Write-Host "P_after_max $maxP"
$cw50 = @($n | Where-Object {
        $_.fec_path -eq 'chase' -and $_.mode -eq 'DATA' -and $_.channel -eq 'CW' -and
        [math]::Abs([double]$_.intensity - 50.0) -lt 0.01
    } | Select-Object -First 1)
if ($cw50.Count) {
    Write-Host "chase DATA CW 50: jsr=$($cw50[0].jsr_meas_db) P_after=$($cw50[0].P_after) crc=$($cw50[0].crc_ok)"
}
$b50 = @($n | Where-Object {
        $_.fec_path -eq 'chase' -and $_.mode -eq 'DATA' -and $_.channel -eq 'Barrage' -and
        [math]::Abs([double]$_.intensity - 50.0) -lt 0.01
    } | Select-Object -First 1)
if ($b50.Count) {
    Write-Host "chase DATA Barrage 50: jsr=$($b50[0].jsr_meas_db) P_after=$($b50[0].P_after)"
}
$cd = @($n | Where-Object {
        $_.fec_path -eq 'chase' -and $_.mode -eq 'DATA' -and $_.channel -eq 'Clean'
    } | Select-Object -First 1)
if ($cd.Count) { Write-Host "chase DATA Clean P_s_pay=$($cd[0].P_s_pay)" }
# smoke rows
foreach ($pair in @(
        @{ ch = 'Clean'; i = 0.0 }
        @{ ch = 'Barrage'; i = 5.0 }
        @{ ch = 'CW'; i = 5.0 }
        @{ ch = 'Swept'; i = 5.0 }
    )) {
    $row = @($n | Where-Object {
            $_.fec_path -eq 'chase' -and $_.mode -eq 'DATA' -and $_.channel -eq $pair.ch -and
            [math]::Abs([double]$_.intensity - $pair.i) -lt 0.001
        } | Select-Object -First 1)
    if ($row.Count) {
        Write-Host "smoke $($pair.ch) $($pair.i): crc=$($row[0].crc_ok)/10"
    }
}

$o = Read-Rows $oldCsv
if ($o.Count) {
    Write-Host "old_rows $($o.Count)"
    $omax = ($o | ForEach-Object { [double]$_.P_after } | Measure-Object -Maximum).Maximum
    Write-Host "old_P_after_max $omax"
}

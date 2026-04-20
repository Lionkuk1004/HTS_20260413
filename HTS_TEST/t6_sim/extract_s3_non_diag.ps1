param([string]$InLog, [string]$OutTxt)
$lines = Get-Content -Path $InLog -ReadCount 0
$in = $false
$out = [System.Collections.Generic.List[string]]::new()
foreach ($line in $lines) {
    if ($line -match '│\s*S3\s') { $in = $true; [void]$out.Add($line); continue }
    if ($in -and ($line -match '│\s*S4\s')) { break }
    if (-not $in) { continue }
    if ($line -match '\[STAGE4-V5-2B\]') { continue }
    [void]$out.Add($line)
}
Set-Content -Path $OutTxt -Value $out -Encoding utf8

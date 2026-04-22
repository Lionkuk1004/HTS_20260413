# Step S: pre-split vs post-split — LPI / Real Sync / T6 logs (fc)
# Requires: MSVC cl on PATH (or run from Developer PowerShell).
$ErrorActionPreference = "Stop"
$utf8 = New-Object System.Text.UTF8Encoding $false

function Read-Utf8([string]$p) { return [System.IO.File]::ReadAllText($p, $utf8) }
function Write-Utf8([string]$p, [string]$c) { [System.IO.File]::WriteAllText($p, $c, $utf8) }
function Normalize-Lf([string]$s) { return ($s -replace "`r`n", "`n").Replace("`r", "`n") }

$LIM = (Resolve-Path (Join-Path $PSScriptRoot "..\..\HTS_LIM")).Path
$SIM = (Resolve-Path $PSScriptRoot).Path
$Backup = Join-Path $LIM "HTS_V400_Dispatcher.cpp.pre_split_backup"
$T6src = Join-Path $SIM "HTS_T6_SIM_Test.cpp"
$Snippet = Join-Path $SIM "stepS_t6_post_split_includes.snippet"

$splitFiles = @(
  "HTS_V400_Dispatcher_Internal.hpp",
  "HTS_V400_Dispatcher_Core.cpp",
  "HTS_V400_Dispatcher_Sync_PSLTE.cpp",
  "HTS_V400_Dispatcher_Payload.cpp",
  "HTS_V400_Dispatcher_TX.cpp",
  "HTS_V400_Dispatcher_Decode.cpp"
)

$postBlock = @"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Core.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Sync_PSLTE.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Payload.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_TX.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Decode.cpp"
"@ + "`n"

$preBlock = @"
#include "../../HTS_LIM/HTS_V400_Dispatcher.cpp"
"@ + "`n"

$postBlock = Normalize-Lf $postBlock
$preBlock = Normalize-Lf $preBlock

if (-not (Test-Path $Backup)) { throw "missing backup: $Backup" }
if (-not (Test-Path $T6src)) { throw "missing: $T6src" }

$t6Orig = Normalize-Lf (Read-Utf8 $T6src)
if ($t6Orig.IndexOf("HTS_V400_Dispatcher_Core.cpp", [StringComparison]::Ordinal) -lt 0) {
  throw "T6: expected post-split dispatcher includes"
}
Write-Utf8 $Snippet $postBlock

$renamed = $false
$t6Patched = $false
try {
  $t6Pre = $t6Orig.Replace($postBlock, $preBlock)
  if ($t6Pre -eq $t6Orig) { throw "T6: include replace (post->pre) had no effect" }
  Write-Utf8 $T6src $t6Pre
  $t6Patched = $true

  Push-Location $LIM
  try {
    foreach ($f in $splitFiles) {
      $p = Join-Path $LIM $f
      if (-not (Test-Path $p)) { throw "missing split file: $p" }
      Rename-Item -LiteralPath $p -NewName ($f + ".tmp") -Force
    }
    $renamed = $true
    Copy-Item -LiteralPath $Backup -Destination (Join-Path $LIM "HTS_V400_Dispatcher.cpp") -Force
  } finally {
    Pop-Location
  }

  Push-Location $SIM
  try {
    $commonLpi = @(
      "/nologo", "/O2", "/std:c++17", "/EHsc", "/MD", "/W3",
      "/I`"..\..\HTS_LIM`"",
      "/DHTS_ALLOW_HOST_BUILD", "/DHTS_FEC_SIMULATE_M4_RAM_LAYOUT", "/DHTS_FEC_POLAR_DISABLE"
    )
    & cl.exe @commonLpi /FeHTS_FEC_Layer_LPI_Isolation_Test_PRE.exe HTS_FEC_Layer_LPI_Isolation_Test.cpp /link /nologo
    if ($LASTEXITCODE -ne 0) { throw "cl LPI PRE failed" }
    cmd /c "HTS_FEC_Layer_LPI_Isolation_Test_PRE.exe > stepS_pre_split_lpi_iso.log 2>&1"

    & cl.exe @commonLpi /FeHTS_FEC_Layer_Real_Sync_Test_PRE.exe HTS_FEC_Layer_Real_Sync_Test.cpp /link /nologo
    if ($LASTEXITCODE -ne 0) { throw "cl RealSync PRE failed" }
    cmd /c "HTS_FEC_Layer_Real_Sync_Test_PRE.exe > stepS_pre_split_real_sync.log 2>&1"

    $t6cl = @(
      "/nologo", "/O2", "/std:c++17", "/EHsc", "/MD", "/W3",
      "/I`".`"", "/I`"..\..\HTS_LIM`"",
      "/DHTS_ALLOW_HOST_BUILD", "/DHTS_FEC_SIMULATE_M4_RAM_LAYOUT",
      "/DHTS_DIAG_PRINTF", "/D_CRT_SECURE_NO_WARNINGS",
      "/FeHTS_T6_SIM_Test_PRE.exe",
      "HTS_T6_SIM_Test.cpp", "HTS_Session_Derive_Stub.cpp",
      "..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp",
      "/link", "/nologo"
    )
    & cl.exe @t6cl
    if ($LASTEXITCODE -ne 0) { throw "cl T6 PRE failed" }
    cmd /c "HTS_T6_SIM_Test_PRE.exe > stepS_pre_split_t6.log 2>&1"
  } finally {
    Pop-Location
  }

  Push-Location $LIM
  try {
    Remove-Item -LiteralPath (Join-Path $LIM "HTS_V400_Dispatcher.cpp") -Force
    foreach ($f in $splitFiles) {
      Rename-Item -LiteralPath (Join-Path $LIM ($f + ".tmp")) -NewName $f -Force
    }
    $renamed = $false
  } finally {
    Pop-Location
  }

  $postDisk = Read-Utf8 $Snippet
  $t6Post = (Read-Utf8 $T6src).Replace($preBlock, $postDisk)
  if ($t6Post -eq (Read-Utf8 $T6src)) { throw "T6: include restore (pre->post) had no effect" }
  Write-Utf8 $T6src $t6Post
  $t6Patched = $false

  Push-Location $SIM
  try {
    & cl.exe @commonLpi /FeHTS_FEC_Layer_LPI_Isolation_Test_POST.exe HTS_FEC_Layer_LPI_Isolation_Test.cpp /link /nologo
    if ($LASTEXITCODE -ne 0) { throw "cl LPI POST failed" }
    cmd /c "HTS_FEC_Layer_LPI_Isolation_Test_POST.exe > stepS_post_split_lpi_iso.log 2>&1"

    & cl.exe @commonLpi /FeHTS_FEC_Layer_Real_Sync_Test_POST.exe HTS_FEC_Layer_Real_Sync_Test.cpp /link /nologo
    if ($LASTEXITCODE -ne 0) { throw "cl RealSync POST failed" }
    cmd /c "HTS_FEC_Layer_Real_Sync_Test_POST.exe > stepS_post_split_real_sync.log 2>&1"

    cmd /c "build.bat"
    if ($LASTEXITCODE -ne 0) { throw "build.bat failed" }
    cmd /c "HTS_T6_SIM_Test.exe > stepS_post_split_t6.log 2>&1"
  } finally {
    Pop-Location
  }

  Push-Location $SIM
  try {
    cmd /c "fc stepS_pre_split_lpi_iso.log stepS_post_split_lpi_iso.log > stepS_diff_lpi_iso.txt"
    cmd /c "fc stepS_pre_split_real_sync.log stepS_post_split_real_sync.log > stepS_diff_real_sync.txt"
    cmd /c "fc stepS_pre_split_t6.log stepS_post_split_t6.log > stepS_diff_t6.txt"
  } finally {
    Pop-Location
  }

  Write-Host "OK — inspect stepS_diff_*.txt in $SIM"
}
catch {
  Write-Host "ERROR: $_"
  if ($renamed) {
    Push-Location $LIM
    try {
      if (Test-Path (Join-Path $LIM "HTS_V400_Dispatcher.cpp")) {
        Remove-Item -LiteralPath (Join-Path $LIM "HTS_V400_Dispatcher.cpp") -Force -ErrorAction SilentlyContinue
      }
      foreach ($f in $splitFiles) {
        $tmp = Join-Path $LIM ($f + ".tmp")
        if (Test-Path $tmp) { Rename-Item -LiteralPath $tmp -NewName $f -Force }
      }
    } finally { Pop-Location }
  }
  if ($t6Patched) { Write-Utf8 $T6src $t6Orig }
  exit 1
}

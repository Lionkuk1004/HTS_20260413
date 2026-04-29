@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"

mkdir diag_results\v5a_apply_fix_2026-04-29 2>nul

set "EXE_BASE=%~dp0x64\Release\HTS_TEST.exe"
set "EXE_FIX=%~dp0x64\Release_ApplyFix\HTS_TEST_ApplyFix.exe"

if not exist "%EXE_BASE%" (
  echo ERROR: not found: "%EXE_BASE%"
  echo Build HTS_T6_SIM_Test with Configuration Release ^| x64 first.
  exit /b 1
)
if not exist "%EXE_FIX%" (
  echo ERROR: not found: "%EXE_FIX%"
  echo Build HTS_T6_SIM_Test with Configuration Release_ApplyFix ^| x64 first.
  exit /b 1
)

echo [1/2] baseline (Apply LEGACY) ...
"%EXE_BASE%" > diag_results\v5a_apply_fix_2026-04-29\baseline.txt 2>&1

echo [2/2] Apply Fix ...
"%EXE_FIX%" > diag_results\v5a_apply_fix_2026-04-29\apply_fix.txt 2>&1

echo === done ===
findstr /C:"정량 합계" diag_results\v5a_apply_fix_2026-04-29\baseline.txt
findstr /C:"정량 합계" diag_results\v5a_apply_fix_2026-04-29\apply_fix.txt

endlocal

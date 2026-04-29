@echo off
REM Run CFO Bank v4 sweeps (b1..b13, bug, random, unified) into diag_results\cfo_sweep_20260428
REM Prereq: build b1..b5 via cursor_lab_v4_batch1.cmd .. batch5.cmd; b6+ via cursor_lab_v4_batch6_13_bug_random_unified.cmd
REM Double-click: pause at end. Automation: ...cmd nopause
setlocal
cd /d "%~dp0"
set "OUT=%~dp0..\diag_results\cfo_sweep_20260428"
mkdir "%OUT%" 2>nul
for %%E in (1 2 3 4 5 6 7 8 9 10 11 12 13) do (
  echo Running batch %%E ...
  "%~dp0HTS_CFO_Bank_Test_v4_b%%E.exe" > "%OUT%\batch_%%E_2026-04-28.txt" 2>&1
)
echo Running BUG_FOCUS ...
"%~dp0HTS_CFO_Bank_Test_v4_bug.exe" > "%OUT%\batch_bug_2026-04-28.txt" 2>&1
echo Running RANDOM ...
"%~dp0HTS_CFO_Bank_Test_v4_random.exe" > "%OUT%\batch_random_2026-04-28.txt" 2>&1
echo Running UNIFIED (long) ...
"%~dp0HTS_CFO_Bank_Test_v4_unified.exe" > "%OUT%\batch_unified_2026-04-28.txt" 2>&1
echo Done. Logs in "%OUT%"
set RC=0
if /i not "%~1"=="nopause" pause
exit /b %RC%

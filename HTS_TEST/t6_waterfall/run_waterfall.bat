@echo off
setlocal

cd /d "%~dp0"

set SEED=0xABCDEF01
if not "%~1"=="" set SEED=%~1

echo === HTS Waterfall Matrix (Double Statistics 50x50) ===
echo Seed=%SEED%
echo Expected runtime: several hours ^(152500 trials^). Use build_waterfall_smoke for a short run.
echo Press Ctrl-C to abort.

HTS_T6_Waterfall.exe %SEED% 1>waterfall_stdout.log 2>waterfall_stderr.log

echo.
echo === DONE ===
echo   waterfall_stdout.log
echo   waterfall_results.csv ^(written by harness^)
echo   waterfall_per_meta.csv
echo   waterfall_stderr.log ^(FEC/host diagnostics^)
exit /b 0

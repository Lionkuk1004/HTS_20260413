@echo off
setlocal
cd /d "%~dp0"

call build_phase4b2_sample.cmd baseline
if errorlevel 1 exit /b 1
call build_phase4b2_sample.cmd mask
if errorlevel 1 exit /b 1

HTS_Phase4B2_Sample_BASELINE.exe 1>stepC4_sample_baseline.csv 2>stepC4_sample_baseline.err
if errorlevel 1 exit /b 1
HTS_Phase4B2_Sample_MASK.exe 1>stepC4_sample_mask.csv 2>stepC4_sample_mask.err
if errorlevel 1 exit /b 1

powershell -NoProfile -ExecutionPolicy Bypass -File merge_phase4b2_sample.ps1
if errorlevel 1 exit /b 1
echo Wrote sample_compare.csv
exit /b 0

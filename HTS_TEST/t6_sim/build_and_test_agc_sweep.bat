@echo off
setlocal EnableDelayedExpansion

for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2^>nul`) do set "VSPATH=%%i"
if not defined VSPATH (
    echo vswhere failed.
    exit /b 1
)
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat"

cd /d D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim

set "SRC=HTS_T6_SIM_Test.cpp HTS_Session_Derive_Stub.cpp ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp"
set "CLF=/nologo /O2 /std:c++17 /EHsc /MD /W3 /I..\..\HTS_LIM /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT /DHTS_FEC_POLAR_DISABLE /DHTS_AMP_DIAG /DHTS_WRC_METHOD=0"

echo =================================== baseline 256/256
cl %CLF% /DHTS_AGC_GAIN_SCALE_NUM=256 /DHTS_AGC_GAIN_SCALE_DEN=256 /FeHTS_T6_AGC_baseline.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1
HTS_T6_AGC_baseline.exe 1>agc_baseline_stdout.log 2>agc_baseline_stderr.log

echo =================================== half 128/256
cl %CLF% /DHTS_AGC_GAIN_SCALE_NUM=128 /DHTS_AGC_GAIN_SCALE_DEN=256 /FeHTS_T6_AGC_half.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1
HTS_T6_AGC_half.exe 1>agc_half_stdout.log 2>agc_half_stderr.log

echo =================================== quarter 64/256
cl %CLF% /DHTS_AGC_GAIN_SCALE_NUM=64 /DHTS_AGC_GAIN_SCALE_DEN=256 /FeHTS_T6_AGC_quarter.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1
HTS_T6_AGC_quarter.exe 1>agc_quarter_stdout.log 2>agc_quarter_stderr.log

echo =================================== double 512/256
cl %CLF% /DHTS_AGC_GAIN_SCALE_NUM=512 /DHTS_AGC_GAIN_SCALE_DEN=256 /FeHTS_T6_AGC_double.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1
HTS_T6_AGC_double.exe 1>agc_double_stdout.log 2>agc_double_stderr.log

echo =================================== small 32/256
cl %CLF% /DHTS_AGC_GAIN_SCALE_NUM=32 /DHTS_AGC_GAIN_SCALE_DEN=256 /FeHTS_T6_AGC_small.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1
HTS_T6_AGC_small.exe 1>agc_small_stdout.log 2>agc_small_stderr.log

echo.
echo ===== Grand Pass =====
for %%L in (baseline half quarter double small) do (
    echo --- %%L ---
    findstr /C:"정량 합계" agc_%%L_stdout.log
)

echo.
echo ===== Saturation / RMS =====
for %%L in (baseline half quarter double small) do (
    echo --- %%L ---
    findstr /C:"saturated" /C:"RMS |I|" agc_%%L_stderr.log
)

exit /b 0

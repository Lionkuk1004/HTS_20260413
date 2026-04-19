@echo off
setlocal EnableDelayedExpansion

for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2^>nul`) do set "VSPATH=%%i"
if not defined VSPATH (
    echo vswhere failed.
    exit /b 1
)
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat"

cd /d D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim

cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
   /I..\..\HTS_LIM ^
   /DHTS_ALLOW_HOST_BUILD ^
   /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /DHTS_FEC_POLAR_DISABLE ^
   /DHTS_CW_DETECT_DIAG ^
   /DHTS_WRC_METHOD=0 ^
   /FeHTS_T6_CW_DETECT.exe ^
   HTS_T6_SIM_Test.cpp ^
   HTS_Session_Derive_Stub.cpp ^
   ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp ^
   ..\..\HTS_LIM\HTS_CW_Detect_Diag.cpp ^
   /link /nologo

if errorlevel 1 (
    echo BUILD FAIL
    exit /b 1
)

HTS_T6_CW_DETECT.exe 1>cw_detect_stdout.log 2>cw_detect_stderr.log

echo.
echo === Grand Pass ===
findstr /C:"정량 합계" cw_detect_stdout.log

echo.
echo === CW Detect Phase1 (stderr) ===
findstr /C:"CW Detect" /C:"per_sym" /C:"win8" /C:"pkt mean" /C:"conc_mean" /C:"ent_mean" cw_detect_stderr.log

exit /b 0

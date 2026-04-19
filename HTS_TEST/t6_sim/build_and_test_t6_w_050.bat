@echo off
setlocal EnableDelayedExpansion

for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2^>nul`) do set "VSPATH=%%i"
if not defined VSPATH (
    echo vswhere failed.
    exit /b 1
)
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat"

cd /d "%~dp0"

cl /nologo /O2 /std:c++17 /EHsc /MD /W3 /I..\..\HTS_LIM ^
   /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT /DHTS_FEC_POLAR_DISABLE /DHTS_WRC_METHOD=0 ^
   /DHTS_CW_LLR_WEIGHT_V1 /DHTS_CW_W_FACTOR=0.5 /DHTS_CW_W_TOP_K=2 ^
   /FeHTS_T6_W_050.exe HTS_T6_SIM_Test.cpp HTS_Session_Derive_Stub.cpp ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp ^
   /link /nologo

if errorlevel 1 (
    echo BUILD FAIL
    exit /b 1
)

HTS_T6_W_050.exe 1>t6_w_050_stdout.log 2>t6_w_050_stderr.log
findstr /c:"정량 합계" t6_w_050_stdout.log
exit /b 0

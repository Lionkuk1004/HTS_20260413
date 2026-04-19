@echo off
setlocal EnableDelayedExpansion

for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2^>nul`) do set "VSPATH=%%i"
if not defined VSPATH (
    echo vswhere failed.
    exit /b 1
)
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat"

cd /d "%~dp0"

cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
   /I..\..\HTS_LIM ^
   /DHTS_ALLOW_HOST_BUILD ^
   /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /DHTS_FEC_POLAR_DISABLE ^
   /DHTS_LLR_DIAG ^
   /DHTS_ROW_CONSISTENCY_DIAG ^
   /DHTS_WRC_METHOD=0 ^
   /DHTS_CW_LLR_WEIGHT_V1 ^
   /DHTS_CW_W_FACTOR=0.25 ^
   /DHTS_CW_W_TOP_K=2 ^
   /DHTS_CW_W_DIAG ^
   /FeHTS_T6_MC_W_025.exe ^
   HTS_T6_MC_Harness.cpp ^
   ..\t6_sim\HTS_Session_Derive_Stub.cpp ^
   ..\t6_sim\HTS_BER_PER_Measure.cpp ^
   ..\t6_sim\HTS_Jammer_STD.cpp ^
   ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp ^
   ..\..\HTS_LIM\HTS_LLR_Diag.cpp ^
   ..\..\HTS_LIM\HTS_Row_Consistency_Diag.cpp ^
   /link /nologo

if errorlevel 1 (
    echo BUILD FAIL
    exit /b 1
)

echo Built HTS_T6_MC_W_025.exe ^(w=0.25^)
exit /b 0

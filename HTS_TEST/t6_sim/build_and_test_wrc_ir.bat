@echo off
setlocal EnableDelayedExpansion

for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2^>nul`) do set "VSPATH=%%i"
if not defined VSPATH (
    echo vswhere failed; ensure VS Build Tools installed.
    exit /b 1
)
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat"

cd /d D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim

for %%M in (0 1 2 3) do (
    echo ===================================
    echo Method %%M build
    echo ===================================
    cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
       /I..\..\HTS_LIM ^
       /DHTS_ALLOW_HOST_BUILD ^
       /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
       /DHTS_FEC_POLAR_DISABLE ^
       /DHTS_WRC_METHOD=%%M ^
       /FeHTS_T6_WRC_IR_%%M.exe ^
       HTS_T6_SIM_Test.cpp ^
       HTS_Session_Derive_Stub.cpp ^
       ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp ^
       /link /nologo
    if errorlevel 1 (
        echo BUILD FAIL %%M
        exit /b 1
    )
    HTS_T6_WRC_IR_%%M.exe 1>wrc_ir_method_%%M.log 2>wrc_ir_diag_%%M.log
)

echo.
echo ===== Quantitative summary (findstr) =====
for %%M in (0 1 2 3) do (
    echo --- Method %%M ---
    findstr /C:"정량 합계" wrc_ir_method_%%M.log
    type wrc_ir_diag_%%M.log
)

exit /b 0

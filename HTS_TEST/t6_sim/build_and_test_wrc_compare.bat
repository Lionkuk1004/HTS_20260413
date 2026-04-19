@echo off
setlocal

for /f "usebackq tokens=*" %%i in (`vswhere.exe -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set VSPATH=%%i
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat"

cd /d D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim

set CLFLAGS=/nologo /O2 /std:c++17 /EHsc /MD /W3 /I"..\..\HTS_LIM" ^
  /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
  /DHTS_FEC_POLAR_DISABLE

set SRC=HTS_T6_SIM_Test.cpp HTS_Session_Derive_Stub.cpp ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp

echo ===================================
echo [0/4] Baseline (HTS_WRC_METHOD=0)
echo ===================================
cl %CLFLAGS% /DHTS_WRC_METHOD=0 /FeHTS_T6_WRC_baseline.exe %SRC% /link /nologo
if %ERRORLEVEL% NEQ 0 (echo BASELINE FAIL & exit /b 1)
HTS_T6_WRC_baseline.exe > wrc_baseline.log 2>&1

echo ===================================
echo [1/4] Method A (zero-out)
echo ===================================
cl %CLFLAGS% /DHTS_WRC_METHOD=1 /FeHTS_T6_WRC_A.exe %SRC% /link /nologo
if %ERRORLEVEL% NEQ 0 (echo METHOD A FAIL & exit /b 1)
HTS_T6_WRC_A.exe > wrc_method_A.log 2>&1

echo ===================================
echo [2/4] Method B (attenuate)
echo ===================================
cl %CLFLAGS% /DHTS_WRC_METHOD=2 /FeHTS_T6_WRC_B.exe %SRC% /link /nologo
if %ERRORLEVEL% NEQ 0 (echo METHOD B FAIL & exit /b 1)
HTS_T6_WRC_B.exe > wrc_method_B.log 2>&1

echo ===================================
echo [3/4] Method C (top1-only)
echo ===================================
cl %CLFLAGS% /DHTS_WRC_METHOD=3 /FeHTS_T6_WRC_C.exe %SRC% /link /nologo
if %ERRORLEVEL% NEQ 0 (echo METHOD C FAIL & exit /b 1)
HTS_T6_WRC_C.exe > wrc_method_C.log 2>&1

echo.
echo ===== 정량 합계 비교 =====
echo Baseline:
findstr "정량 합계" wrc_baseline.log
echo Method A (zero-out):
findstr "정량 합계" wrc_method_A.log
echo Method B (attenuate):
findstr "정량 합계" wrc_method_B.log
echo Method C (top1-only):
findstr "정량 합계" wrc_method_C.log

endlocal

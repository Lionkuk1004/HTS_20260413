@echo off
setlocal
cd /d "%~dp0"

set LIM=..\..\HTS_LIM
set MODE=%~1
if "%MODE%"=="" set MODE=baseline

if /i "%MODE%"=="baseline" (
  set DEFMASK=/DHTS_WALSH_ROW_MASK_SAMPLE=0
  set OUTEXE=HTS_Phase4B2_Sample_BASELINE.exe
) else if /i "%MODE%"=="mask" (
  set DEFMASK=/DHTS_WALSH_ROW_MASK_SAMPLE=1 /DHTS_WALSH_ROW_MASK_SHIFT=3
  set OUTEXE=HTS_Phase4B2_Sample_MASK.exe
) else (
  echo Usage: %~nx0 [baseline^|mask]
  exit /b 2
)

cl /nologo /O2 /std:c++17 /EHsc /MD /W3 /I. /I%LIM% ^
  /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
  /DHTS_FEC_POLAR_DISABLE /D_CRT_SECURE_NO_WARNINGS /DNOMINMAX ^
  /D_USE_MATH_DEFINES %DEFMASK% ^
  /Fe%OUTEXE% ^
  ..\HTS_Jammer_STD\HTS_Phase4B2_Sample_Harness.cpp ^
  HTS_V400_Dispatcher_Local.cpp ^
  %LIM%\HTS_FEC_HARQ.cpp ^
  %LIM%\HTS_Walsh_Row_Permuter.cpp ^
  %LIM%\HTS_Walsh_Row_Converter.cpp ^
  %LIM%\HTS_Holo_LPI.cpp ^
  %LIM%\HTS_Secure_Memory.cpp ^
  %LIM%\HTS_Polar_Codec.cpp ^
  HTS_Session_Derive_Stub.cpp ^
  /link /nologo /STACK:16000000

if errorlevel 1 exit /b 1
echo Built: %CD%\%OUTEXE%
exit /b 0

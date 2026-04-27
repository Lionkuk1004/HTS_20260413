@echo off
REM Lab: HTS_CFO_Bank_Test_v4 — 양산 CFO_V5a + 128-chip preamble + optional pipeline DIAG.
REM Usage: Developer Command Prompt x64, then run this script.
REM   기본: sweep only (파이프라인 printf 없음)
REM   set PIPELINE=1 → HTS_PIPELINE_DIAG + HTS_V5A_BUG9_DIAG (Stage3 gate_mag)
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"

set "HTS_LIM=%~dp0..\HTS_LIM"
set "CLBASE=/nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS"
set "CLBASE=%CLBASE% /DHTS_USE_HOLO_TENSOR_4D /DHTS_ALLOW_HOST_BUILD"
set "CLBASE=%CLBASE% /DHTS_CFO_V5A_ENABLE=1 /DNOGDI /I"%HTS_LIM%""

set "OUTEXE=HTS_CFO_Bank_Test_v4.exe"
if "%PIPELINE%"=="1" (
  set "CLBASE=%CLBASE% /DHTS_PIPELINE_DIAG /DHTS_V5A_BUG9_DIAG"
  set "OUTEXE=HTS_CFO_Bank_Test_v4_pipeline.exe"
)

cl %CLBASE% /Fe:%OUTEXE% ^
   HTS_CFO_Bank_Test_v4.cpp ^
   "%HTS_LIM%\HTS_CFO_V5a.cpp" ^
   "%HTS_LIM%\HTS_Rx_CFO_SinCos_Table.cpp" ^
   "%HTS_LIM%\HTS_Holo_Tensor_4D_Common.cpp" ^
   "%HTS_LIM%\HTS_Holo_Tensor_4D_TX.cpp" ^
   "%HTS_LIM%\HTS_Holo_Tensor_4D_RX.cpp" ^
   "%HTS_LIM%\HTS_Secure_Memory.cpp" ^
   "%HTS_LIM%\HTS_Preamble_Holographic.cpp" ^
   /link /nologo

exit /b %ERRORLEVEL%

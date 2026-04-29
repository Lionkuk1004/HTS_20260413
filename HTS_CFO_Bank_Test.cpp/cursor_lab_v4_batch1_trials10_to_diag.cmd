@echo off
REM One-shot: build BATCH_1 exe with 10 trials/CFO, run, write diag_results\cfo_sweep_10trials_20260428\*.txt
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || (
  echo [ERROR] vcvars64.bat failed.
  pause
  exit /b 1
)
cd /d "%~dp0"
if not exist "HTS_CFO_Bank_Test_v4.cpp" (
  echo [ERROR] Run from HTS_CFO_Bank_Test.cpp folder.
  pause
  exit /b 1
)
set "HTS_LIM=%~dp0..\HTS_LIM"
set "CLBASE=/nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS /DHTS_USE_HOLO_TENSOR_4D /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DNOGDI /DHTS_BATCH_1 /DHTS_CFO_SWEEP_TRIALS=10 /I"%HTS_LIM%""
cl %CLBASE% /Fe:HTS_CFO_Bank_Test_v4_b1_t10.exe ^
   HTS_CFO_Bank_Test_v4.cpp ^
   "%HTS_LIM%\HTS_CFO_V5a.cpp" ^
   "%HTS_LIM%\HTS_Rx_CFO_SinCos_Table.cpp" ^
   "%HTS_LIM%\HTS_Holo_Tensor_4D_Common.cpp" ^
   "%HTS_LIM%\HTS_Holo_Tensor_4D_TX.cpp" ^
   "%HTS_LIM%\HTS_Holo_Tensor_4D_RX.cpp" ^
   "%HTS_LIM%\HTS_Secure_Memory.cpp" ^
   "%HTS_LIM%\HTS_Preamble_Holographic.cpp" ^
   /link /nologo
if errorlevel 1 (
  echo BUILD FAILED
  pause
  exit /b 1
)
set "OUT=%~dp0..\diag_results\cfo_sweep_10trials_20260428"
mkdir "%OUT%" 2>nul
(
  echo CFO Bank v4 BATCH_1, HTS_CFO_SWEEP_TRIALS=10, 0..1000 Hz @50 Hz
  echo Log file: batch1_0_1000hz_hz50_step_trials10.txt
) > "%OUT%\README_10trials.txt"
"%~dp0HTS_CFO_Bank_Test_v4_b1_t10.exe" > "%OUT%\batch1_0_1000hz_hz50_step_trials10.txt" 2>&1
set RC=%ERRORLEVEL%
echo Done. Exit=%RC% log="%OUT%\batch1_0_1000hz_hz50_step_trials10.txt"
if /i not "%~1"=="nopause" pause
exit /b %RC%

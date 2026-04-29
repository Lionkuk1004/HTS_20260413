@echo off
REM Build CFO Bank v4 exes: BATCH 6..13, BUG_FOCUS, RANDOM, UNIFIED (same TU as cursor_lab_v4_batch1.cmd).
REM Double-click: pause at end. Automation: ...cmd nopause
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || (
  echo [ERROR] vcvars64.bat failed.
  pause
  exit /b 1
)
cd /d "%~dp0"
if not exist "HTS_CFO_Bank_Test_v4.cpp" (
  echo [ERROR] HTS_CFO_Bank_Test_v4.cpp not found in "%CD%"
  pause
  exit /b 1
)
set "HTS_LIM=%~dp0..\HTS_LIM"
set "CLBASE=/nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS /DHTS_USE_HOLO_TENSOR_4D /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DNOGDI /I"%HTS_LIM%""

cl %CLBASE% /DHTS_BATCH_6 /Fe:HTS_CFO_Bank_Test_v4_b6.exe ^
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
  echo BUILD FAILED (see messages above)
  pause
  exit /b 1
)

cl %CLBASE% /DHTS_BATCH_7 /Fe:HTS_CFO_Bank_Test_v4_b7.exe ^
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
  echo BUILD FAILED (see messages above)
  pause
  exit /b 1
)

cl %CLBASE% /DHTS_BATCH_8 /Fe:HTS_CFO_Bank_Test_v4_b8.exe ^
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
  echo BUILD FAILED (see messages above)
  pause
  exit /b 1
)

cl %CLBASE% /DHTS_BATCH_9 /Fe:HTS_CFO_Bank_Test_v4_b9.exe ^
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
  echo BUILD FAILED (see messages above)
  pause
  exit /b 1
)

cl %CLBASE% /DHTS_BATCH_10 /Fe:HTS_CFO_Bank_Test_v4_b10.exe ^
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
  echo BUILD FAILED (see messages above)
  pause
  exit /b 1
)

cl %CLBASE% /DHTS_BATCH_11 /Fe:HTS_CFO_Bank_Test_v4_b11.exe ^
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
  echo BUILD FAILED (see messages above)
  pause
  exit /b 1
)

cl %CLBASE% /DHTS_BATCH_12 /Fe:HTS_CFO_Bank_Test_v4_b12.exe ^
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
  echo BUILD FAILED (see messages above)
  pause
  exit /b 1
)

cl %CLBASE% /DHTS_BATCH_13 /Fe:HTS_CFO_Bank_Test_v4_b13.exe ^
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
  echo BUILD FAILED (see messages above)
  pause
  exit /b 1
)

cl %CLBASE% /DHTS_BATCH_BUG_FOCUS /Fe:HTS_CFO_Bank_Test_v4_bug.exe ^
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
  echo BUILD FAILED (see messages above)
  pause
  exit /b 1
)

cl %CLBASE% /DHTS_BATCH_RANDOM /Fe:HTS_CFO_Bank_Test_v4_random.exe ^
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
  echo BUILD FAILED (see messages above)
  pause
  exit /b 1
)

cl %CLBASE% /DHTS_BATCH_UNIFIED /Fe:HTS_CFO_Bank_Test_v4_unified.exe ^
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
  echo BUILD FAILED (see messages above)
  pause
  exit /b 1
)

echo OK: b6..b13, bug, random, unified built in %CD%
set RC=0
if /i not "%~1"=="nopause" pause
exit /b %RC%

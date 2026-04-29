@echo off
REM Batch 4: CFO 3000..4000 Hz @50 Hz (21 points)
REM Run as .cmd file from this folder (do not paste into interactive cmd).
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
  echo Run: cd /d "...\HTS_CFO_Bank_Test.cpp" ^&^& cursor_lab_v4_batch4.cmd
  pause
  exit /b 1
)
set "HTS_LIM=%~dp0..\HTS_LIM"
set "CLBASE=/nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS /DHTS_USE_HOLO_TENSOR_4D /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DNOGDI /DHTS_BATCH_4 /I"%HTS_LIM%""
cl %CLBASE% /Fe:HTS_CFO_Bank_Test_v4_b4.exe ^
   HTS_CFO_Bank_Test_v4.cpp ^
   "%HTS_LIM%\HTS_CFO_V5a.cpp" ^
   "%HTS_LIM%\HTS_Rx_CFO_SinCos_Table.cpp" ^
   "%HTS_LIM%\HTS_Holo_Tensor_4D_Common.cpp" ^
   "%HTS_LIM%\HTS_Holo_Tensor_4D_TX.cpp" ^
   "%HTS_LIM%\HTS_Holo_Tensor_4D_RX.cpp" ^
   "%HTS_LIM%\HTS_Secure_Memory.cpp" ^
   "%HTS_LIM%\HTS_Preamble_Holographic.cpp" ^
   /link /nologo
set RC=%ERRORLEVEL%
if %RC% neq 0 echo BUILD FAILED, exit code %RC%
if /i not "%~1"=="nopause" pause
exit /b %RC%

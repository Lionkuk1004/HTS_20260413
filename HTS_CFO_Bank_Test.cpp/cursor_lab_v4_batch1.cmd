@echo off
REM Batch 1: CFO 0..1000 Hz @50 Hz (21 points). Same TU as cursor_lab_v5a_production.cmd + /DHTS_CFO_SWEEP_TRIALS=10
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
set "HTS_LIM=%~dp0..\HTS_LIM"
set "CLBASE=/nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS /DHTS_USE_HOLO_TENSOR_4D /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DNOGDI /DHTS_BATCH_1 /I"%HTS_LIM%""
cl %CLBASE% /Fe:HTS_CFO_Bank_Test_v4_b1.exe ^
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

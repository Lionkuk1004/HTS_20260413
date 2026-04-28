@echo off
REM Option 4: v4 LR_AB 9-point — 인프라 매크로 없음 (HOLO Phase-B derotate OFF, 영준님 Step 2A baseline 정합)
REM 출력: v4_mode_baseline_lr_ab_result.txt
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
set "HTS_LIM=%~dp0..\HTS_LIM"
set "SRC=HTS_CFO_Bank_Test_v4.cpp "%HTS_LIM%\HTS_CFO_V5a.cpp" "%HTS_LIM%\HTS_Rx_CFO_SinCos_Table.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_Common.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_TX.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_RX.cpp" "%HTS_LIM%\HTS_Secure_Memory.cpp" "%HTS_LIM%\HTS_Preamble_Holographic.cpp""
set "CLBASE=/nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS /DHTS_USE_HOLO_TENSOR_4D /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DNOGDI /DHTS_BATCH_LR_AB /I"%HTS_LIM%""

cl %CLBASE% /Fe:v4_mode_baseline.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1

v4_mode_baseline.exe > v4_mode_baseline_lr_ab_result.txt 2>&1
endlocal
exit /b 0

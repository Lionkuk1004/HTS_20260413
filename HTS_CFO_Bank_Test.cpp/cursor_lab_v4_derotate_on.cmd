@echo off
REM Step 1 Lab: HTS_BATCH_LR_AB (9 cliff CFO) + HTS_HOLO_RX_PHASE_B_DEROTATE ON
REM 비교용으로 derotate OFF 빌드도 같은 스크립트에서 실행 (출력: *_off / *_on).
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
set "HTS_LIM=%~dp0..\HTS_LIM"
set "SRC=HTS_CFO_Bank_Test_v4.cpp "%HTS_LIM%\HTS_CFO_V5a.cpp" "%HTS_LIM%\HTS_Rx_CFO_SinCos_Table.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_Common.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_TX.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_RX.cpp" "%HTS_LIM%\HTS_Secure_Memory.cpp" "%HTS_LIM%\HTS_Preamble_Holographic.cpp""
set "CLBASE=/nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS /DHTS_USE_HOLO_TENSOR_4D /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DNOGDI /DHTS_BATCH_LR_AB /I"%HTS_LIM%""

echo === Build A: HOLO derotate OFF (baseline, same TU) ===
cl %CLBASE% /Fe:HTS_CFO_Bank_Test_v4_lr_ab_derotate_off.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1

echo === Build B: HOLO derotate ON (HTS_HOLO_RX_PHASE_B_DEROTATE) ===
cl %CLBASE% /DHTS_HOLO_RX_PHASE_B_DEROTATE /Fe:HTS_CFO_Bank_Test_v4_lr_ab_derotate_on.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1

HTS_CFO_Bank_Test_v4_lr_ab_derotate_off.exe > v4_derotate_off_lr_ab_result.txt 2>&1
HTS_CFO_Bank_Test_v4_lr_ab_derotate_on.exe  > v4_derotate_on_lr_ab_result.txt 2>&1

echo === derotate OFF ===
type v4_derotate_off_lr_ab_result.txt
echo.
echo === derotate ON ===
type v4_derotate_on_lr_ab_result.txt

endlocal
exit /b 0

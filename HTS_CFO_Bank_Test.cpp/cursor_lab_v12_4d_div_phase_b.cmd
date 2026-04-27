@echo off
REM Step 2-3d-3: Lab v12 + Phase B (잔여 dφ derotate in 4D decode)
REM   HTS_HOLO_RX_PHASE_B: 양산 Dispatcher 경로용 (Lab TU에선 미링크)
REM   HTS_HOLO_RX_PHASE_B_DEROTATE: HTS_Holo_Tensor_4D_RX.cpp 에서
REM     holo4d_estimate_residual_dphi_q16 결과를 derotate 에 반영 (필수)
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
set "HTS_LIM=%~dp0..\HTS_LIM"
set "SRC=HTS_CFO_Bank_Test_v12_4d_diversity.cpp "%HTS_LIM%\HTS_CFO_V5a.cpp" "%HTS_LIM%\HTS_Rx_CFO_SinCos_Table.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_Common.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_TX.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_RX.cpp" "%HTS_LIM%\HTS_Secure_Memory.cpp" "%HTS_LIM%\HTS_Preamble_Holographic.cpp""
set "CLBASE=/nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS /DHTS_USE_HOLO_TENSOR_4D /DHTS_USE_4D_DIVERSITY /DHTS_HOLO_RX_PHASE_B /DHTS_HOLO_RX_PHASE_B_DEROTATE /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DNOGDI /I"%HTS_LIM%""

cl %CLBASE% /Fe:HTS_CFO_Bank_Test_v12_4d_div_phase_b.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1

HTS_CFO_Bank_Test_v12_4d_div_phase_b.exe > v12_phase_b_result.txt 2>&1
type v12_phase_b_result.txt

endlocal
exit /b 0

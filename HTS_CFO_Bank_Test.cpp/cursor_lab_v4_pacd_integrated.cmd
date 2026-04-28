@echo off
REM C-3: v4 LR_AB + PaCD (V5a Apply 후 Decode 전), HTS_V400_Dispatcher_PaCD.cpp 링크
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
set "HTS_LIM=%~dp0..\HTS_LIM"
set "SRC=HTS_CFO_Bank_Test_v4.cpp "%HTS_LIM%\HTS_CFO_V5a.cpp" "%HTS_LIM%\HTS_Rx_CFO_SinCos_Table.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_Common.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_TX.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_RX.cpp" "%HTS_LIM%\HTS_Secure_Memory.cpp" "%HTS_LIM%\HTS_Preamble_Holographic.cpp" "%HTS_LIM%\HTS_V400_Dispatcher_PaCD.cpp""
set "CLBASE=/nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS /DHTS_USE_HOLO_TENSOR_4D /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DNOGDI /DHTS_BATCH_LR_AB /DHTS_USE_PACD /I"%HTS_LIM%""

REM C-3 동작(무게이트): threshold 0 = 항상 PaCD 적용
cl %CLBASE% /DHTS_PACD_PSI_THRESHOLD_Q15=0 /Fe:v4_pacd_integrated.exe %SRC% /link /nologo
if errorlevel 1 (
    echo BUILD FAILED
    exit /b 1
)
v4_pacd_integrated.exe > v4_pacd_integrated_lr_ab_result.txt 2>&1
type v4_pacd_integrated_lr_ab_result.txt
endlocal
exit /b 0

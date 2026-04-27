@echo off
REM Step B-A: I!=Q TX + phase sweep (HTS_CFO_Bank_Test_v11_phase_iq.cpp)
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
set "HTS_LIM=%~dp0..\HTS_LIM"
set "SRC=HTS_CFO_Bank_Test_v11_phase_iq.cpp "%HTS_LIM%\HTS_CFO_V5a.cpp" "%HTS_LIM%\HTS_Rx_CFO_SinCos_Table.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_Common.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_TX.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_RX.cpp" "%HTS_LIM%\HTS_Secure_Memory.cpp" "%HTS_LIM%\HTS_Preamble_Holographic.cpp""
set "CLBASE=/nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS /DHTS_USE_HOLO_TENSOR_4D /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DNOGDI /I"%HTS_LIM%""

cl %CLBASE% /Fe:HTS_CFO_Bank_Test_v11_phase_iq.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1

HTS_CFO_Bank_Test_v11_phase_iq.exe > v11_phase_iq_result.txt 2>&1
type v11_phase_iq_result.txt

endlocal
exit /b 0

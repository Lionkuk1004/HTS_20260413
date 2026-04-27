@echo off
REM Lab: HTS_BATCH_LR_AB — Normal / No PTE / No LR / Pure bank (PTE+LR off).
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
set "HTS_LIM=%~dp0..\HTS_LIM"
set "SRC=HTS_CFO_Bank_Test_v4.cpp "%HTS_LIM%\HTS_CFO_V5a.cpp" "%HTS_LIM%\HTS_Rx_CFO_SinCos_Table.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_Common.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_TX.cpp" "%HTS_LIM%\HTS_Holo_Tensor_4D_RX.cpp" "%HTS_LIM%\HTS_Secure_Memory.cpp" "%HTS_LIM%\HTS_Preamble_Holographic.cpp""
set "CLBASE=/nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS /DHTS_USE_HOLO_TENSOR_4D /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DNOGDI /DHTS_BATCH_LR_AB /I"%HTS_LIM%""

echo === Build 1 Normal (PTE+LR ON) ===
cl %CLBASE% /Fe:HTS_CFO_Bank_Test_v4_normal.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1

echo === Build 2 No PTE ===
cl %CLBASE% /DHTS_V5A_DISABLE_PTE /Fe:HTS_CFO_Bank_Test_v4_no_pte.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1

echo === Build 3 No LR ===
cl %CLBASE% /DHTS_V5A_DISABLE_LR /Fe:HTS_CFO_Bank_Test_v4_no_lr.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1

echo === Build 4 Pure bank (PTE+LR OFF) ===
cl %CLBASE% /DHTS_V5A_DISABLE_PTE /DHTS_V5A_DISABLE_LR /Fe:HTS_CFO_Bank_Test_v4_pure_bank.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1

echo.
echo ========== Normal ==========
HTS_CFO_Bank_Test_v4_normal.exe
echo.
echo ========== No PTE ==========
HTS_CFO_Bank_Test_v4_no_pte.exe
echo.
echo ========== No LR ==========
HTS_CFO_Bank_Test_v4_no_lr.exe
echo.
echo ========== Pure bank ==========
HTS_CFO_Bank_Test_v4_pure_bank.exe

endlocal
exit /b 0

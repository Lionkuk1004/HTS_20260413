@echo off
REM Step 4: CFO_V5a::Estimate LR vs MnM (surrogate rows — 원본 V9 소스 미탑재 시 보조 측정)
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
set "LIM=%~dp0..\HTS_LIM"
set "SRC=HTS_CFO_Bank_Test_step4_v9_surrogate_full_est.cpp "%LIM%\HTS_CFO_V5a.cpp" "%LIM%\HTS_Rx_CFO_SinCos_Table.cpp""
set "CLBASE=cl /nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /I"%LIM%""

%CLBASE% /Fe:step4_v9_surrogate_LR.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1
%CLBASE% /DHTS_USE_MNM_WALSH /Fe:step4_v9_surrogate_MnM.exe %SRC% /link /nologo
if errorlevel 1 exit /b 1

step4_v9_surrogate_LR.exe  > step4_v9_surrogate_lr.txt 2>&1
step4_v9_surrogate_MnM.exe > step4_v9_surrogate_mnm.txt 2>&1

echo === LR ===
type step4_v9_surrogate_lr.txt
echo === MnM ===
type step4_v9_surrogate_mnm.txt

endlocal
exit /b 0

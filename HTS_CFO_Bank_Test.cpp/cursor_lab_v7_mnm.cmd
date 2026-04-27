@echo off
REM Lab v7: 27 CFO x 100 trials — LR_Estimate vs MnM_Walsh_Estimate_Dpte_Table
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
set "HTS_LIM=%~dp0..\HTS_LIM"
set "SRC=HTS_CFO_Bank_Test_v7_mnm.cpp "%HTS_LIM%\HTS_CFO_V5a.cpp" "%HTS_LIM%\HTS_Rx_CFO_SinCos_Table.cpp""

cl /nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS ^
   /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DHTS_USE_MNM_WALSH ^
   /I"%HTS_LIM%" ^
   /Fe:HTS_CFO_Bank_Test_v7_mnm.exe ^
   %SRC% /link /nologo
if errorlevel 1 exit /b 1

HTS_CFO_Bank_Test_v7_mnm.exe > v7_mnm_result.txt 2>&1
type v7_mnm_result.txt

endlocal
exit /b 0

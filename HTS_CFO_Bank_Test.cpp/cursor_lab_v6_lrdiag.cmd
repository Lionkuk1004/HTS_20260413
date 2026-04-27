@echo off
REM Lab: HTS_LR_DIAG — per-trial L&R stage dump (cfo_true=1000 Hz).
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
set "HTS_LIM=%~dp0..\HTS_LIM"
set "SRC=HTS_CFO_Bank_Test_v6_lrdiag.cpp "%HTS_LIM%\HTS_CFO_V5a.cpp" "%HTS_LIM%\HTS_Rx_CFO_SinCos_Table.cpp""

cl /nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS ^
   /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DHTS_LR_DIAG ^
   /I"%HTS_LIM%" ^
   /Fe:HTS_CFO_Bank_Test_v6_lrdiag.exe ^
   %SRC% /link /nologo
if errorlevel 1 exit /b 1

HTS_CFO_Bank_Test_v6_lrdiag.exe > v6_lrdiag_result.txt 2>&1
type v6_lrdiag_result.txt

endlocal
exit /b 0

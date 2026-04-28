@echo off
REM C-3 A: v4 Walsh preamble (kAmp=500) vs PaCD static TX preamble — sign/ratio check
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
set "HTS_LIM=%~dp0..\HTS_LIM"
cl /nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS ^
   /DHTS_USE_PACD /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DNOGDI ^
   /I"%HTS_LIM%" ^
   /Fe:pacd_compare.exe ^
   pacd_preamble_compare.cpp ^
   "%HTS_LIM%\HTS_V400_Dispatcher_PaCD.cpp" ^
   "%HTS_LIM%\HTS_CFO_V5a.cpp" ^
   "%HTS_LIM%\HTS_Rx_CFO_SinCos_Table.cpp" ^
   "%HTS_LIM%\HTS_Secure_Memory.cpp" ^
   /link /nologo
if errorlevel 1 exit /b 1
pacd_compare.exe > pacd_compare_result.txt 2>&1
type pacd_compare_result.txt
endlocal
exit /b 0

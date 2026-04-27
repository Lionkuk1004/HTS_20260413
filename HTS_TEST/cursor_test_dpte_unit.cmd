@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
set "LIM=%~dp0..\HTS_LIM"
cl /nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS ^
   /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DHTS_USE_MNM_WALSH ^
   /I"%LIM%" ^
   /Fe:test_cfo_v5a_mnm_walsh_dpte.exe ^
   test_cfo_v5a_mnm_walsh_dpte.cpp "%LIM%\HTS_CFO_V5a.cpp" "%LIM%\HTS_Rx_CFO_SinCos_Table.cpp" ^
   /link /nologo
if errorlevel 1 exit /b 1
test_cfo_v5a_mnm_walsh_dpte.exe
endlocal
exit /b 0

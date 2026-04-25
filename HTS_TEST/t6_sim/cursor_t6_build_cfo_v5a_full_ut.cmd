@echo off
REM Phase 1-5: Full CFO_V5a::Estimate() unit test (host)
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"

cl /nologo /O2 /std:c++17 /EHsc /MD /W4 /WX /wd4324 /DNDEBUG ^
   /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD ^
   /D_CRT_SECURE_NO_WARNINGS ^
   /Fetest_cfo_v5a_full.exe ^
   ..\..\HTS_TEST\test_cfo_v5a_full.cpp ^
   ..\..\HTS_LIM\HTS_CFO_V5a.cpp ^
   ..\..\HTS_LIM\HTS_Rx_CFO_SinCos_Table.cpp ^
   /link /nologo

if errorlevel 1 exit /b 1
.\test_cfo_v5a_full.exe
exit /b %ERRORLEVEL%

@echo off
REM Phase A: cfo_ vs V5a sin/cos diagnostic (host)
setlocal
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" 2>nul
if errorlevel 1 call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"

cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
   /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD ^
   /D_CRT_SECURE_NO_WARNINGS ^
   /Fetest_sincos_diff.exe ^
   ..\..\HTS_TEST\test_sincos_diff.cpp ^
   ..\..\HTS_LIM\HTS_CFO_V5a.cpp ^
   ..\..\HTS_LIM\HTS_Rx_CFO_SinCos_Table.cpp ^
   /link /nologo

if errorlevel 1 exit /b 1
.\test_sincos_diff.exe
exit /b %ERRORLEVEL%

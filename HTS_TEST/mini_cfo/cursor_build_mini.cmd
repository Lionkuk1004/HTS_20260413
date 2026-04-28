@echo off
REM test_mini_cfo.exe — Mini + V5a 격리 링크 (HTS_TEST/mini_cfo)
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"

cl /nologo /std:c++17 /O2 /EHsc /MD /W3 /DNDEBUG ^
   /DHTS_ALLOW_HOST_BUILD=1 ^
   /I"..\..\HTS_LIM" ^
   /Fe:test_mini_cfo.exe ^
   test_mini_cfo.cpp ^
   HTS_Mini_CFO.cpp ^
   ..\..\HTS_LIM\HTS_CFO_V5a.cpp ^
   ..\..\HTS_LIM\HTS_Rx_CFO_SinCos_Table.cpp ^
   /link /nologo

if errorlevel 1 exit /b %ERRORLEVEL%
echo Build OK

exit /b 0

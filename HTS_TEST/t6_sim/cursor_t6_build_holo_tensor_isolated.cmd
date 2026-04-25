@echo off
REM Isolated HOLO/tensor CFO sweep lab scaffold
setlocal
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" 2>nul
if errorlevel 1 call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"

cl /nologo /O2 /std:c++17 /EHsc /MD /W4 /WX /wd4324 /DNDEBUG ^
   /I"." /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD ^
   /D_CRT_SECURE_NO_WARNINGS ^
   /FeHTS_CFO_Bank_Test.exe ^
   ..\..\HTS_TEST\HTS_CFO_Bank_Test.cpp ^
   ..\..\HTS_LIM\HTS_CFO_V5a.cpp ^
   ..\..\HTS_LIM\HTS_Rx_CFO_SinCos_Table.cpp ^
   ..\..\HTS_LIM\HTS_Holo_Tensor_4D.cpp ^
   ..\..\HTS_LIM\HTS_Secure_Memory.cpp ^
   ..\..\HTS_LIM\HTS_Preamble_Holographic.cpp ^
   /link /nologo

if errorlevel 1 exit /b 1
.\HTS_CFO_Bank_Test.exe
exit /b %ERRORLEVEL%

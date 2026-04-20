@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
  /I"..\..\HTS_LIM" /I"..\..\HTS_Jammer_STD" ^
  /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT /DHTS_FEC_POLAR_DISABLE ^
  /FeHTS_Block_Message_Test.exe ^
  HTS_Block_Message_Test.cpp /link /nologo
exit /b %ERRORLEVEL%

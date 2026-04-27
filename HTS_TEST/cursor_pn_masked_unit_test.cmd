@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"

cl /nologo /O2 /std:c++17 /EHsc /MD /W4 /WX /DNDEBUG ^
   /I"..\HTS_LIM" ^
   /DHTS_USE_PN_MASKED ^
   /Fe:pn_masked_unit_test.exe ^
   pn_masked_unit_test.cpp ^
   /link /nologo

exit /b %ERRORLEVEL%

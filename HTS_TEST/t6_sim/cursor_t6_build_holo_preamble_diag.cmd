@echo off
REM HOLO preamble (CMYK=0) + HTS_DIAG_HOLO_CMYK — Phase 2.8 LEGACY-STATE 로그
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"

set HOLO_FLAG=
REM set HOLO_FLAG=/DHTS_USE_HOLOGRAPHIC_SYNC

cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
   /I"." /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD ^
   /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /DHTS_HOLO_PREAMBLE ^
   /DHTS_DIAG_HOLO_CMYK=1 ^
   /DHTS_DIAG_PRINTF ^
   %HOLO_FLAG% ^
   /D_CRT_SECURE_NO_WARNINGS ^
   /FeHTS_T6_SIM_Test_holo_pre_diag.exe ^
   HTS_T6_SIM_Test.cpp HTS_Session_Derive_Stub.cpp ^
   ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp ^
   ..\..\HTS_LIM\HTS_Preamble_Holographic.cpp ^
   /link /nologo

exit /b %ERRORLEVEL%

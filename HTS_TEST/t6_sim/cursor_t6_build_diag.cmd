@echo off
REM Holo preamble + DIAG printf + S5-only single CFO (HTS_DIAG_SINGLE_CFO).
REM Usage: cursor_t6_build_diag.cmd [cfo_hz_int]
REM   Default cfo_hz_int=5000. Example: cursor_t6_build_diag.cmd 0
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"

set "HZ=%~1"
if "%HZ%"=="" set "HZ=5000"

set HOLO_FLAG=
REM set HOLO_FLAG=/DHTS_USE_HOLOGRAPHIC_SYNC

cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
   /I"." /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD ^
   /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /DHTS_HOLO_PREAMBLE ^
   /DHTS_DIAG_PRINTF ^
   /DHTS_DIAG_SINGLE_CFO ^
   /DHTS_DIAG_SINGLE_CFO_HZ=%HZ% ^
   /DHTS_DIAG_SINGLE_CFO_TRIALS=10 ^
   %HOLO_FLAG% ^
   /D_CRT_SECURE_NO_WARNINGS ^
   /FeHTS_T6_SIM_Test_diag.exe ^
   HTS_T6_SIM_Test.cpp HTS_Session_Derive_Stub.cpp ^
   ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp ^
   ..\..\HTS_LIM\HTS_Preamble_Holographic.cpp ^
   /link /nologo

exit /b %ERRORLEVEL%

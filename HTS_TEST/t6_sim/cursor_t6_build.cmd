@echo off
REM HTS_T6_SIM_Test.exe - Cursor cl flags (same as build.bat stage 1)
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"

REM Holographic: default OFF (Step2 baseline). S5-HOLO/DIAG: uncomment next line.
set HOLO_FLAG=
REM set HOLO_FLAG=/DHTS_USE_HOLOGRAPHIC_SYNC

cl /nologo /O2 /std:c++17 /EHsc /MD /W4 /WX /wd4324 /DNDEBUG ^
   /I"." /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD ^
   /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /DHTS_USE_HOLO_TENSOR_4D ^
   %HOLO_FLAG% ^
   /D_CRT_SECURE_NO_WARNINGS ^
   /FeHTS_T6_SIM_Test.exe ^
   HTS_T6_SIM_Test.cpp HTS_Session_Derive_Stub.cpp ^
   ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp ^
   ..\..\HTS_LIM\HTS_Preamble_Holographic.cpp ^
   ..\..\HTS_LIM\HTS_Rx_CFO_SinCos_Table.cpp ^
   ..\..\HTS_LIM\HTS_CFO_V5a.cpp ^
   ..\..\HTS_LIM\HTS_Holo_Tensor_4D.cpp ^
   /link /nologo

exit /b %ERRORLEVEL%

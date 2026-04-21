@echo off
setlocal
cd /d "%~dp0"

set LIM=..\..\HTS_LIM

cl /nologo /O2 /std:c++17 /EHsc /MD /W3 /I. /I%LIM% ^
  /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
  /DHTS_FEC_POLAR_DISABLE /D_CRT_SECURE_NO_WARNINGS /DNOMINMAX ^
  /D_USE_MATH_DEFINES /DHTS_PHASE3_ALPHA_TX_CFO=0 /DHTS_WALSH_ROW_MASK_SAMPLE=0 ^
  /FeHTS_Phase4B2_Sample_v2_BASELINE.exe ^
  ..\HTS_Jammer_STD\HTS_Phase4B2_Sample_Harness.cpp ^
  HTS_V400_Dispatcher_Local.cpp ^
  %LIM%\HTS_FEC_HARQ.cpp ^
  %LIM%\HTS_Walsh_Row_Permuter.cpp ^
  %LIM%\HTS_Walsh_Row_Converter.cpp ^
  %LIM%\HTS_Holo_LPI.cpp ^
  %LIM%\HTS_Secure_Memory.cpp ^
  %LIM%\HTS_Polar_Codec.cpp ^
  HTS_Session_Derive_Stub.cpp ^
  /link /nologo /STACK:16000000
if errorlevel 1 exit /b 1

cl /nologo /O2 /std:c++17 /EHsc /MD /W3 /I. /I%LIM% ^
  /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
  /DHTS_FEC_POLAR_DISABLE /D_CRT_SECURE_NO_WARNINGS /DNOMINMAX ^
  /D_USE_MATH_DEFINES /DHTS_PHASE3_ALPHA_TX_CFO=0 /DHTS_WALSH_ROW_MASK_SAMPLE=1 /DHTS_WALSH_ROW_MASK_SHIFT=3 ^
  /FeHTS_Phase4B2_Sample_v2_MASK.exe ^
  ..\HTS_Jammer_STD\HTS_Phase4B2_Sample_Harness.cpp ^
  HTS_V400_Dispatcher_Local.cpp ^
  %LIM%\HTS_FEC_HARQ.cpp ^
  %LIM%\HTS_Walsh_Row_Permuter.cpp ^
  %LIM%\HTS_Walsh_Row_Converter.cpp ^
  %LIM%\HTS_Holo_LPI.cpp ^
  %LIM%\HTS_Secure_Memory.cpp ^
  %LIM%\HTS_Polar_Codec.cpp ^
  HTS_Session_Derive_Stub.cpp ^
  /link /nologo /STACK:16000000
if errorlevel 1 exit /b 1

HTS_Phase4B2_Sample_v2_BASELINE.exe 1>stepC4_sample_v2_baseline.csv 2>stepC4_sample_v2_baseline.err
if errorlevel 1 exit /b 1
HTS_Phase4B2_Sample_v2_MASK.exe 1>stepC4_sample_v2_mask.csv 2>stepC4_sample_v2_mask.err
if errorlevel 1 exit /b 1

powershell -NoProfile -ExecutionPolicy Bypass -File merge_phase4b2_sample_v2.ps1
if errorlevel 1 exit /b 1
echo Wrote sample_v2_compare.csv
exit /b 0

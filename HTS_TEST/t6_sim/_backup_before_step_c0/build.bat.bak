@echo off
del /F /Q *.obj 2>nul
del /F /Q HTS_T6_SIM_Test.exe 2>nul
del /F /Q HTS_Harq_Matrix_Test.exe 2>nul
del /F /Q HTS_Harq_Matrix_Test_V5.exe 2>nul

REM -- 1. T6 SIM (vcxproj 동일: Session stub + WRC 호스트 TU) --
cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
   /I"." /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /DHTS_FEC_POLAR_DISABLE /DHTS_DIAG_PRINTF /D_CRT_SECURE_NO_WARNINGS ^
   /FeHTS_T6_SIM_Test.exe HTS_T6_SIM_Test.cpp HTS_Session_Derive_Stub.cpp ^
   ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp /link /nologo
if errorlevel 1 goto :err

REM -- 2. HARQ Matrix (v5 OFF — 기본 동작) --
cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
   /I"." /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /DHTS_FEC_POLAR_DISABLE /D_CRT_SECURE_NO_WARNINGS ^
   /FeHTS_Harq_Matrix_Test.exe HTS_Harq_Matrix_Test.cpp HTS_V400_Dispatcher_Local.cpp /link /nologo
if errorlevel 1 goto :err

REM -- 3. HARQ Matrix v5 ON (DIAG 전용, 동일 소스 + 매크로) --
cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
   /I"." /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /DHTS_FEC_POLAR_DISABLE /DHTS_WALSH_V5_PREAMBLE /DHTS_DIAG_PRINTF ^
   /D_CRT_SECURE_NO_WARNINGS ^
   /FeHTS_Harq_Matrix_Test_V5.exe HTS_Harq_Matrix_Test.cpp HTS_V400_Dispatcher_Local.cpp /link /nologo
if errorlevel 1 goto :err

echo [OK] T6 + HARQ Matrix (OFF) + HARQ Matrix V5 (ON) built successfully
goto :eof

:err
echo [FAIL] Build error
exit /b 1

@echo off
REM HTS_T6_SIM_Test_ami.exe — AMI 경로 dry-run 빌드
REM 기반: cursor_t6_build.cmd + /DHTS_TARGET_AMI 추가
REM 출력: HTS_T6_SIM_Test_ami.exe (baseline 덮어쓰지 않음)
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"

cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
   /I"." /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD ^
   /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /DHTS_DIAG_PRINTF ^
   /DHTS_TARGET_AMI ^
   /D_CRT_SECURE_NO_WARNINGS ^
   /FeHTS_T6_SIM_Test_ami.exe ^
   HTS_T6_SIM_Test.cpp HTS_Session_Derive_Stub.cpp ^
   ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp ^
   /link /nologo

exit /b %ERRORLEVEL%

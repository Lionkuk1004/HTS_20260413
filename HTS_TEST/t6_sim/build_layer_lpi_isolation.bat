@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
REM Step C-4b: matched-filter 힌트 경로 켜려면 아래 줄에 /DHTS_SYNC_USE_MATCHED_FILTER 추가
cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
  /I"..\..\HTS_LIM" ^
  /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT /DHTS_FEC_POLAR_DISABLE ^
  /FeHTS_FEC_Layer_LPI_Isolation_Test.exe ^
  HTS_FEC_Layer_LPI_Isolation_Test.cpp /link /nologo
exit /b %ERRORLEVEL%

@echo off
REM Holographic util 단독 단위 테스트 빌드
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"

cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
  /I"..\..\HTS_LIM" ^
  /D_CRT_SECURE_NO_WARNINGS ^
  /FeHTS_Holographic_Unit_Test.exe ^
  HTS_Holographic_Unit_Test.cpp ^
  ..\..\HTS_LIM\HTS_Preamble_Holographic.cpp ^
  /link /nologo

if exist HTS_Holographic_Unit_Test.exe (
    echo === Build OK ===
) else (
    echo === Build FAILED ===
    exit /b 1
)

exit /b %ERRORLEVEL%

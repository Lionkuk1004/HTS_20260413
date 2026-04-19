@echo off
setlocal EnableExtensions
cd /d "%~dp0"

echo ========================================
echo  HTS Phase 3 (PROMPT 49 v3) — Jammer UnitTest single-TU
echo  cwd: %CD%
echo ========================================

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  echo ERROR: vswhere.exe not found. Install VS or Build Tools.
  exit /b 1
)
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSINSTALL=%%i"
if not defined VSINSTALL (
  echo ERROR: No VS installation with VC Tools found.
  exit /b 1
)
call "%VSINSTALL%\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1

if defined BOOST_ROOT (
  set BOOST_I=/I"%BOOST_ROOT%"
) else (
  set BOOST_I=
)

echo [1/3] Building HTS_Jammer_STD_UnitTest.exe (Phase1+Phase3) ...
echo      (V400 로그 비대 방지: HTS_DIAG_PRINTF 미정의 — 필요 시 수동 추가^)
cl /nologo /O2 /std:c++17 /EHsc /MD /W3 %BOOST_I% /I"..\..\HTS_LIM" ^
  /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
  /DHTS_FEC_POLAR_DISABLE /D_CRT_SECURE_NO_WARNINGS ^
  /DHTS_ENABLE_PHASE3_MIL /DHTS_LINK_JAMMER_STD_IN_UNITTEST ^
  /FeHTS_Jammer_STD_UnitTest.exe HTS_Jammer_STD_UnitTest.cpp /link /nologo
if errorlevel 1 (
  echo BUILD FAILED
  exit /b 1
)

echo.
echo [2/3] Running (S1 default; add /DHTS_PHASE3_FULL_MATRIX to cl for S2-S6^) ...
HTS_Jammer_STD_UnitTest.exe > phase3_output.log 2>&1
set RUNERR=%ERRORLEVEL%

echo.
echo [3/3] Artifacts: phase3_output.log, phase3_all_scenarios.csv, phase3_mil_profile.txt, phase3_analysis.md
echo NOTE: HTS_DIAG_PRINTF makes phase3_output.log very large.
exit /b %RUNERR%

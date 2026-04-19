@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  echo ERROR: vswhere.exe not found.
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

echo Phase3 noise DIAG: HTS_Phase3_Diag.exe -^> phase3_diag_output.log
cl /nologo /O2 /std:c++17 /EHsc /MD /W3 %BOOST_I% /I"..\..\HTS_LIM" ^
  /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
  /DHTS_FEC_POLAR_DISABLE /D_CRT_SECURE_NO_WARNINGS ^
  /DHTS_ENABLE_PHASE3_MIL /DHTS_LINK_JAMMER_STD_IN_UNITTEST ^
  /DHTS_PHASE3_DEBUG_S1_MINI /DHTS_PHASE3_NOISE_DIAG ^
  /FeHTS_Phase3_Diag.exe HTS_Jammer_STD_UnitTest.cpp /link /nologo
if errorlevel 1 (
  echo BUILD FAILED
  exit /b 1
)

set START=%TIME%
HTS_Phase3_Diag.exe > phase3_diag_output.log 2>&1
set RUNERR=%ERRORLEVEL%
set END=%TIME%

echo Done. Results: phase3_diag_output.log
echo Start: %START%  End: %END%  EXITCODE=%RUNERR%
exit /b %RUNERR%

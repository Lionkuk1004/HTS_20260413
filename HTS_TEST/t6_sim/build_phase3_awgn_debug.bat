@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSINSTALL=%%i"
call "%VSINSTALL%\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1

echo === Phase3 AWGN debug: no V400 printf flood, J1 DIAG, mini S1 ===
cl /nologo /O2 /std:c++17 /EHsc /MD /W3 /I"..\..\HTS_LIM" ^
  /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
  /DHTS_FEC_POLAR_DISABLE /D_CRT_SECURE_NO_WARNINGS ^
  /DHTS_ENABLE_PHASE3_MIL /DHTS_LINK_JAMMER_STD_IN_UNITTEST ^
  /DHTS_PHASE3_DEBUG_S1_MINI ^
  /DHTS_PHASE3_NOISE_DIAG ^
  /FeHTS_Jammer_STD_UnitTest.exe HTS_Jammer_STD_UnitTest.cpp /link /nologo
if errorlevel 1 exit /b 1

HTS_Jammer_STD_UnitTest.exe > phase3_awgn_debug.log 2>&1
echo Wrote phase3_awgn_debug.log (and phase3_*.csv/md/txt if Phase3 ran^)
exit /b %ERRORLEVEL%

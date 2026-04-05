@echo off
setlocal EnableExtensions
REM Tensor unit test: locate MSVC (vswhere) then cl + run.
REM Run from anywhere; working directory becomes repo root (parent of this folder).

set "REPO=%~dp0.."
set "VCVARS="

if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" (
  for /f "usebackq delims=" %%I in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2^>nul`) do (
    if exist "%%I\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=%%I\VC\Auxiliary\Build\vcvars64.bat"
  )
)

if not defined VCVARS (
  if exist "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" (
    set "VCVARS=C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
  )
)

if not defined VCVARS (
  echo [ERROR] vcvars64.bat not found. Install "Desktop development with C++" or set VCVARS to your vcvars64.bat
  exit /b 1
)

call "%VCVARS%" || exit /b 1
cd /d "%REPO%" || exit /b 1

cl /nologo /EHsc /std:c++17 /O2 /W3 /DNDEBUG /I"HTS_LIM" verify_tensor_unit_test\test_hts_tensor_engine.cpp /Fe:verify_tensor_unit_test\test_hts_tensor_engine.exe /Fo:verify_tensor_unit_test\test_hts_tensor_engine.obj
if errorlevel 1 exit /b 1

verify_tensor_unit_test\test_hts_tensor_engine.exe
exit /b %ERRORLEVEL%

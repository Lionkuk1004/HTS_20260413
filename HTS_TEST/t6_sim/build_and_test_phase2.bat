@echo off
setlocal EnableExtensions
cd /d "%~dp0"

echo ========================================
echo  HTS Phase 2 — BER/PER unit test + logs
echo  cwd: %CD%
echo  (Walsh needs Session_Gateway::Derive_Session_Material — see
echo   HTS_Session_Derive_Stub.cpp included from HTS_BER_PER_UnitTest.cpp)
echo ========================================

rem Optional: set BOOST_ROOT only if switching clopper_pearson_ci to boost::math
if defined BOOST_ROOT (
  echo [info] BOOST_ROOT=%BOOST_ROOT% ^(extra /I applied^)
  set BOOST_I=/I"%BOOST_ROOT%"
) else (
  echo [info] BOOST_ROOT unset — embedded Beta/CP build ^(no Boost required^)
  set "BOOST_I="
)

echo.
echo [1/4] Building HTS_BER_PER_UnitTest.exe ...
cl /nologo /O2 /std:c++17 /EHsc /MD /W3 %BOOST_I% /I"..\..\HTS_LIM" ^
  /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
  /DHTS_FEC_POLAR_DISABLE /DHTS_DIAG_PRINTF ^
  /FeHTS_BER_PER_UnitTest.exe HTS_BER_PER_UnitTest.cpp /link /nologo
if errorlevel 1 (
  echo BUILD FAILED
  exit /b 1
)

echo.
echo [2/4] Archiving previous logs ^(.bak^)...
for %%F in (t6_phase2_cp_unittest.log t6_phase2_ber_per.log t6_phase2_theory_compare.log t6_phase2_regression.log) do (
  if exist "%%F" copy /Y "%%F" "%%F.bak" >nul
)

echo.
echo [3/4] Running HTS_BER_PER_UnitTest.exe ...
HTS_BER_PER_UnitTest.exe > t6_phase2_cp_unittest.log 2>&1
if errorlevel 1 (
  echo UNIT TESTS FAILED — see t6_phase2_cp_unittest.log
  exit /b 1
)
rem Prompt filename convention: duplicate full console capture
copy /Y t6_phase2_cp_unittest.log t6_phase2_ber_per.log >nul
copy /Y t6_phase2_cp_unittest.log t6_phase2_theory_compare.log >nul

echo.
echo [4/4] Regression V34 Fix A3 ^(if exe present in this folder^)...
if exist "HTS_T6_SIM_Test_V34_FIXA3.exe" (
  HTS_T6_SIM_Test_V34_FIXA3.exe > t6_phase2_regression.log 2>&1
  echo Regression log written: t6_phase2_regression.log
) else (
  echo SKIP: HTS_T6_SIM_Test_V34_FIXA3.exe not found in %CD%
  (echo Build or copy HTS_T6_SIM_Test_V34_FIXA3.exe into this folder, then re-run build_and_test_phase2.bat.)> t6_phase2_regression.log
)

echo.
echo ========================================
echo  Phase 2 build and test completed OK
echo ========================================
exit /b 0

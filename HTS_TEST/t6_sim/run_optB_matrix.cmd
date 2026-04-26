@echo off
REM Phase B opt B: baseline vs REF-only vs REF+APPLY — builds, runs T6, writes out_lab\optB_matrix\SUMMARY.md
REM Usage:  cmd /c "D:\...\HTS_TEST\t6_sim\run_optB_matrix.cmd"
REM Optional: set RUN_OPTB_PAUSE=1 for pause at end on success/failure.
setlocal enabledelayedexpansion
chcp 65001 >nul 2>&1

echo.
echo ============================================================
echo   Phase B option B - automated matrix (Build A/B/C + T6)
echo ============================================================
echo.

set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"

set "OUT_DIR=%SCRIPT_DIR%out_lab\optB_matrix"
if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"

REM ------------------------------------------------------------------
REM Build A: baseline (no REF/APPLY macros)
REM ------------------------------------------------------------------
echo [1/3] Build A - Baseline (cursor_t6_build_holo.cmd^)...
call "%SCRIPT_DIR%cursor_t6_build_holo.cmd" >"%OUT_DIR%\build_A.log" 2>&1
if errorlevel 1 (
    echo   Build A FAILED - see "%OUT_DIR%\build_A.log"
    goto :ERROR
)
echo       Build A OK
echo       Running T6 A...
if not exist "%SCRIPT_DIR%HTS_T6_SIM_Test_holo.exe" (
    echo   HTS_T6_SIM_Test_holo.exe not found after build A
    goto :ERROR
)
"%SCRIPT_DIR%HTS_T6_SIM_Test_holo.exe" >"%OUT_DIR%\result_A.log" 2>&1
if errorlevel 1 (
    echo   T6 A returned errorlevel !ERRORLEVEL! - see result_A.log
    goto :ERROR
)
echo       T6 A done

REM ------------------------------------------------------------------
REM Build B: REF only (Step 1)
REM ------------------------------------------------------------------
echo.
echo [2/3] Build B - Step 1 REF only (cursor_t6_build_holo_optB_step1.cmd^)...
call "%SCRIPT_DIR%cursor_t6_build_holo_optB_step1.cmd" >"%OUT_DIR%\build_B.log" 2>&1
if errorlevel 1 (
    echo   Build B FAILED - see "%OUT_DIR%\build_B.log"
    goto :ERROR
)
echo       Build B OK
echo       Running T6 B...
if not exist "%SCRIPT_DIR%HTS_T6_SIM_Test_holo_optB_step1.exe" (
    echo   HTS_T6_SIM_Test_holo_optB_step1.exe not found after build B
    goto :ERROR
)
"%SCRIPT_DIR%HTS_T6_SIM_Test_holo_optB_step1.exe" >"%OUT_DIR%\result_B.log" 2>&1
if errorlevel 1 (
    echo   T6 B returned errorlevel !ERRORLEVEL! - see result_B.log
    goto :ERROR
)
echo       T6 B done

REM ------------------------------------------------------------------
REM Build C: REF + APPLY (Step 2)
REM ------------------------------------------------------------------
echo.
echo [3/3] Build C - REF+APPLY (cursor_t6_build_holo_optB.cmd^)...
call "%SCRIPT_DIR%cursor_t6_build_holo_optB.cmd" >"%OUT_DIR%\build_C.log" 2>&1
if errorlevel 1 (
    echo   Build C FAILED - see "%OUT_DIR%\build_C.log"
    goto :ERROR
)
echo       Build C OK
echo       Running T6 C...
if not exist "%SCRIPT_DIR%HTS_T6_SIM_Test_holo_optB.exe" (
    echo   HTS_T6_SIM_Test_holo_optB.exe not found after build C
    goto :ERROR
)
"%SCRIPT_DIR%HTS_T6_SIM_Test_holo_optB.exe" >"%OUT_DIR%\result_C.log" 2>&1
if errorlevel 1 (
    echo   T6 C returned errorlevel !ERRORLEVEL! - see result_C.log
    goto :ERROR
)
echo       T6 C done

REM ------------------------------------------------------------------
REM Python summary (primary SUMMARY.md + scores_* excerpt)
REM ------------------------------------------------------------------
echo.
echo Generating SUMMARY.md (Python^)...
set "PY_OK=0"
where py >nul 2>&1
if not errorlevel 1 (
    py -3 "%SCRIPT_DIR%analyze_optB_results.py" >"%OUT_DIR%\analyze_py.log" 2>&1
    if not errorlevel 1 set "PY_OK=1"
)
if "!PY_OK!"=="0" (
    where python >nul 2>&1
    if not errorlevel 1 (
        python "%SCRIPT_DIR%analyze_optB_results.py" >"%OUT_DIR%\analyze_py.log" 2>&1
        if not errorlevel 1 set "PY_OK=1"
    )
)
if "!PY_OK!"=="0" (
    echo   WARNING: py/python failed or not in PATH - writing minimal SUMMARY.md
    (
        echo # Phase B opt B matrix
        echo.
        echo Run manually after installing Python 3:
        echo   py -3 "%SCRIPT_DIR%analyze_optB_results.py"
        echo.
        echo Raw logs: result_A.log, result_B.log, result_C.log
    ) >"%OUT_DIR%\SUMMARY.md"
    echo   Fallback: scores_*.txt via findstr ...
    for %%T in (A B C) do (
        findstr /i /c:"S1" /c:"S5" /c:"S5H" /c:"PASS" /c:"FAIL" /c:"PART" "%OUT_DIR%\result_%%T.log" >"%OUT_DIR%\scores_%%T.txt" 2>nul
    )
)

echo.
echo ============================================================
echo   Done.
echo   - "%OUT_DIR%\SUMMARY.md"
echo   - "%OUT_DIR%\result_*.log"
echo   - "%OUT_DIR%\build_*.log"
echo   - "%OUT_DIR%\scores_*.txt"
echo ============================================================

if "%RUN_OPTB_PAUSE%"=="1" pause
exit /b 0

:ERROR
echo.
echo *** FAILED ***  Logs: "%OUT_DIR%\"
if "%RUN_OPTB_PAUSE%"=="1" pause
exit /b 1

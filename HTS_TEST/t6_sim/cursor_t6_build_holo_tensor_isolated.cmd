@echo off
REM ============================================================================
REM cursor_t6_build_holo_tensor_isolated.cmd
REM 영준님 Phase H Lab 격리 빌드 스크립트
REM ============================================================================
REM 사용법:
REM   1. VS Developer Command Prompt 에서 실행
REM   2. 또는 vcvarsall.bat 후 이 스크립트 실행
REM ============================================================================

setlocal

REM 작업 디렉토리 (이 .cmd 파일 위치 기준)
set SCRIPT_DIR=%~dp0
set HTS_LIM=%SCRIPT_DIR%..\..\HTS_LIM
set HTS_TEST=%SCRIPT_DIR%..

echo ============================================================
echo  Phase H Lab Build (격리 환경)
echo ============================================================
echo HTS_LIM:  %HTS_LIM%
echo HTS_TEST: %HTS_TEST%
echo [build] Lab single-file: "%HTS_TEST%\HTS_CFO_Bank_Test_lab.cpp"
echo [run]   Capture log: "%SCRIPT_DIR%out_lab\holo_tensor_lab.exe" ^> "%SCRIPT_DIR%out_lab\lab_p_run2.txt" 2^>^&1
echo.

REM 출력 디렉토리
if not exist "%SCRIPT_DIR%out_lab" mkdir "%SCRIPT_DIR%out_lab"

REM 컴파일 옵션
set CL_OPTS=/std:c++17 /O2 /W4 /EHsc /nologo /D_CRT_SECURE_NO_WARNINGS
set CL_OPTS=%CL_OPTS% /DHTS_USE_HOLO_TENSOR_4D
set CL_OPTS=%CL_OPTS% /DHTS_ALLOW_HOST_BUILD
set CL_OPTS=%CL_OPTS% /DHTS_CFO_V5A_ENABLE=1
set CL_OPTS=%CL_OPTS% /DNOGDI
set CL_OPTS=%CL_OPTS% /I"%HTS_LIM%"

REM 소스 파일
set SRCS=
set SRCS=%SRCS% "%HTS_TEST%\HTS_CFO_Bank_Test_lab.cpp"
set SRCS=%SRCS% "%HTS_LIM%\HTS_CFO_V5a.cpp"
set SRCS=%SRCS% "%HTS_LIM%\HTS_Rx_CFO_SinCos_Table.cpp"
set SRCS=%SRCS% "%HTS_LIM%\HTS_Holo_Tensor_4D.cpp"
set SRCS=%SRCS% "%HTS_LIM%\HTS_Preamble_Holographic.cpp"
set SRCS=%SRCS% "%HTS_LIM%\HTS_Secure_Memory.cpp"

REM 빌드
cl %CL_OPTS% %SRCS% /Fe:"%SCRIPT_DIR%out_lab\holo_tensor_lab.exe" /Fo:"%SCRIPT_DIR%out_lab\\"
if errorlevel 1 (
    echo.
    echo *** BUILD FAILED ***
    exit /b 1
)

echo.
echo ============================================================
echo  Build OK. 실행:
echo ============================================================
echo.

"%SCRIPT_DIR%out_lab\holo_tensor_lab.exe"

endlocal

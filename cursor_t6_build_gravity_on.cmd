@echo off
REM Delegates to HTS_TEST\t6_sim\cursor_t6_build_gravity_on.cmd (repo root helper)
setlocal
call "%~dp0HTS_TEST\t6_sim\cursor_t6_build_gravity_on.cmd"
exit /b %ERRORLEVEL%

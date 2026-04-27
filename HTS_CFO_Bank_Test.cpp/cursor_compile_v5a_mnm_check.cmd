@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM
cl /nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /c HTS_CFO_V5a.cpp
if errorlevel 1 exit /b 1
cl /nologo /O2 /std:c++20 /EHsc /MD /W3 /DNDEBUG /D_CRT_SECURE_NO_WARNINGS /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1 /DHTS_USE_MNM_WALSH /Fo:HTS_CFO_V5a_mnm.obj /c HTS_CFO_V5a.cpp
exit /b 0

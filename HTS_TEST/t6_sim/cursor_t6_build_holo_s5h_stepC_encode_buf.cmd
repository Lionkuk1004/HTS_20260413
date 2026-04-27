@echo off
REM Step C: S5H 200Hz 1 trial + [ENC] + [V5A-APPLY] + full [ENC-CHIP]/[V5A-BUF] dump.
REM HTS_DIAG_STEPC_MAIN_ONLY: skip S1-S10 except S5-HOLO.
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"

cl /nologo /O2 /std:c++17 /EHsc /MD /W4 /WX /wd4324 /DNDEBUG ^
   /I"." /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD ^
   /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /DHTS_USE_HOLO_TENSOR_4D ^
   /DHTS_USE_HOLOGRAPHIC_SYNC ^
   /DHTS_CFO_V5A_S5H_TRIAL_DIAG ^
   /DHTS_CFO_V5A_S5H_APPLY_DIAG ^
   /DHTS_DIAG_STEPB_S5H_CFO_HZ=200 ^
   /DHTS_CFO_V5A_S5H_ENCODE_DIAG ^
   /DHTS_CFO_V5A_S5H_STEPC_FULL ^
   /DHTS_DIAG_STEPC_S5H_200HZ_ONE_TRIAL ^
   /DHTS_DIAG_STEPC_MAIN_ONLY ^
   /D_CRT_SECURE_NO_WARNINGS ^
   /FeHTS_T6_SIM_Test_holo_s5h_stepC.exe ^
   HTS_T6_SIM_Test.cpp HTS_Session_Derive_Stub.cpp ^
   ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp ^
   ..\..\HTS_LIM\HTS_Preamble_Holographic.cpp ^
   ..\..\HTS_LIM\HTS_Rx_CFO_SinCos_Table.cpp ^
   ..\..\HTS_LIM\HTS_CFO_V5a.cpp ^
   ..\..\HTS_LIM\HTS_Holo_Tensor_4D_Common.cpp ^
   ..\..\HTS_LIM\HTS_Holo_Tensor_4D_TX.cpp ^
   ..\..\HTS_LIM\HTS_Holo_Tensor_4D_RX.cpp ^
   /link /nologo

exit /b %ERRORLEVEL%

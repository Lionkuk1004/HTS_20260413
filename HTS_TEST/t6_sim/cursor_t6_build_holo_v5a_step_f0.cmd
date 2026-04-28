@echo off

REM HTS_T6_SIM_Test_holo_v5a_step_f0.exe — step_d1 동일 + HTS_V5A_STEP_F0_DIAG (S5 0/100/500Hz 입력 dump)

setlocal

call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1

cd /d "%~dp0"



cl /nologo /O2 /std:c++17 /EHsc /MD /W4 /WX /wd4324 /DNDEBUG ^
   /I"." /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD ^
   /DHTS_UT2_STEP1_DIAG ^
   /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /DHTS_USE_HOLO_TENSOR_4D ^
   /DHTS_USE_HOLOGRAPHIC_SYNC ^
   /DHTS_CFO_V5A_S5H_TRIAL_DIAG ^
   /DHTS_V5A_DIAG=1 ^
   /DHTS_V5A_STEP_A_DIAG=1 ^
   /DHTS_V5A_STEP_B1_DIAG=1 ^
   /DHTS_V5A_STEP_C1_DIAG=1 ^
   /DHTS_V5A_STEP_D1_DIAG=1 ^
   /DHTS_V5A_STEP_F0_DIAG=1 ^
   /DHTS_DIAG_PRINTF=1 ^
   /D_CRT_SECURE_NO_WARNINGS ^
   /FeHTS_T6_SIM_Test_holo_v5a_step_f0.exe ^
   HTS_T6_SIM_Test.cpp HTS_Session_Derive_Stub.cpp ^
   ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp ^
   ..\..\HTS_LIM\HTS_Preamble_Holographic.cpp ^
   ..\..\HTS_LIM\HTS_Rx_CFO_SinCos_Table.cpp ^
   ..\..\HTS_LIM\HTS_V400_Dispatcher_PaCD.cpp ^
   ..\..\HTS_LIM\HTS_CFO_V5a.cpp ^
   ..\..\HTS_LIM\HTS_Holo_Tensor_4D_Common.cpp ^
   ..\..\HTS_LIM\HTS_Holo_Tensor_4D_TX.cpp ^
   ..\..\HTS_LIM\HTS_Holo_Tensor_4D_RX.cpp ^
   /link /nologo



exit /b %ERRORLEVEL%

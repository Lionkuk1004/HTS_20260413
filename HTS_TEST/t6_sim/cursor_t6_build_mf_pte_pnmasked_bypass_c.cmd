@echo off
REM HTS_T6_SIM_Test_mf_pte_bypass_c.exe — bypass_c + MF/PN + 링크 TU
REM   (/DHTS_SYNC_USE_MATCHED_FILTER /DHTS_USE_PN_MASKED + Matched_Filter + Dynamic_Config)
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"
cl /nologo /O2 /std:c++17 /EHsc /MD /W4 /WX /wd4324 /DNDEBUG ^
   /I"." /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD ^
   /DHTS_UT2_STEP1_DIAG ^
   /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /DHTS_USE_HOLO_TENSOR_4D ^
   /DHTS_SYNC_USE_MATCHED_FILTER ^
   /DHTS_USE_PN_MASKED ^
   /DHTS_BYPASS_GROUP_C=1 ^
   /D_CRT_SECURE_NO_WARNINGS ^
   /FeHTS_T6_SIM_Test_mf_pte_bypass_c.exe ^
   HTS_T6_SIM_Test.cpp HTS_Session_Derive_Stub.cpp ^
   ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp ^
   ..\..\HTS_LIM\HTS_Preamble_Holographic.cpp ^
   ..\..\HTS_LIM\HTS_Rx_CFO_SinCos_Table.cpp ^
   ..\..\HTS_LIM\HTS_Rx_Matched_Filter.cpp ^
   ..\..\HTS_LIM\HTS_Dynamic_Config.cpp ^
   ..\..\HTS_LIM\HTS_V400_Dispatcher_PaCD.cpp ^
   ..\..\HTS_LIM\HTS_CFO_V5a.cpp ^
   ..\..\HTS_LIM\HTS_Holo_Tensor_4D_Common.cpp ^
   ..\..\HTS_LIM\HTS_Holo_Tensor_4D_TX.cpp ^
   ..\..\HTS_LIM\HTS_Holo_Tensor_4D_RX.cpp ^
   /link /nologo
exit /b %ERRORLEVEL%

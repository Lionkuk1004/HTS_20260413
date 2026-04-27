@echo off
REM HTS_T6_SIM_Test_mf_pte.exe — MF + PTE (Sync) + PN-masked LUT 기준
REM   HTS_SYNC_USE_MATCHED_FILTER: MF
REM   PTE: mf_pte_refine_ (Sync_PSLTE / Sync_AMI, MF 경로 내장)
REM   HTS_USE_PN_MASKED: MF reference = PN-masked 첫 64칩 (양산 LUT)
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"

cl /nologo /O2 /std:c++17 /EHsc /MD /W4 /WX /wd4324 /DNDEBUG ^
   /I"." /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD ^
   /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /DHTS_USE_HOLO_TENSOR_4D ^
   /DHTS_USE_PACD ^
   /DHTS_USE_PN_MASKED=1 ^
   /DHTS_SYNC_USE_MATCHED_FILTER=1 ^
   /D_CRT_SECURE_NO_WARNINGS ^
   /FeHTS_T6_SIM_Test_mf_pte.exe ^
   HTS_T6_SIM_Test.cpp HTS_Session_Derive_Stub.cpp ^
   ..\..\HTS_LIM\HTS_Walsh_Row_Converter.cpp ^
   ..\..\HTS_LIM\HTS_Preamble_Holographic.cpp ^
   ..\..\HTS_LIM\HTS_Rx_CFO_SinCos_Table.cpp ^
   ..\..\HTS_LIM\HTS_V400_Dispatcher_PaCD.cpp ^
   ..\..\HTS_LIM\HTS_Rx_Matched_Filter.cpp ^
   ..\..\HTS_LIM\HTS_Dynamic_Config.cpp ^
   ..\..\HTS_LIM\HTS_CFO_V5a.cpp ^
   ..\..\HTS_LIM\HTS_Holo_Tensor_4D_Common.cpp ^
   ..\..\HTS_LIM\HTS_Holo_Tensor_4D_TX.cpp ^
   ..\..\HTS_LIM\HTS_Holo_Tensor_4D_RX.cpp ^
   /link /nologo

exit /b %ERRORLEVEL%

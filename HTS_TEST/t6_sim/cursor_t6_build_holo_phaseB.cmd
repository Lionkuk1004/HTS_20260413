@echo off
REM HTS_T6_SIM_Test_holo_phaseB.exe — cursor_t6_build_holo.cmd + HTS_HOLO_RX_PHASE_B
REM 선택: 잔여 dφ 칩 derotate 실험 시 `/DHTS_HOLO_RX_PHASE_B_DEROTATE` 추가
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1
cd /d "%~dp0"

cl /nologo /O2 /std:c++17 /EHsc /MD /W4 /WX /wd4324 /DNDEBUG ^
   /I"." /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD ^
   /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /DHTS_USE_HOLO_TENSOR_4D ^
   /DHTS_USE_HOLOGRAPHIC_SYNC ^
   /DHTS_HOLO_RX_PHASE_B ^
   /D_CRT_SECURE_NO_WARNINGS ^
   /FeHTS_T6_SIM_Test_holo_phaseB.exe ^
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

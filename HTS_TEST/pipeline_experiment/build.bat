@echo off
del /F /Q *.obj 2>nul
del /F /Q HTS_T6_SIM_Test_exp.exe 2>nul

cl /nologo /O2 /std:c++17 /EHsc /MD /W3 ^
   /I"." /I"..\..\HTS_LIM" ^
   /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
   /FeHTS_T6_SIM_Test_exp.exe HTS_T6_SIM_Test_exp.cpp /link /nologo

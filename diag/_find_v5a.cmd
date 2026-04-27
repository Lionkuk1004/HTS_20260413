@echo off
cd /d D:\HTS_ARM11_Firmware\HTS_LIM
findstr /n /i "V5A-PTE-DIAG" HTS_TEST\t6_sim\out_lab\%~1 > diag\%~2
find /c /v "" diag\%~2

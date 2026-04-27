@echo off
findstr /n /c:"[V5A-S5H-TRIAL]" D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\out_lab\phaseA2_5_trial_on.log > D:\HTS_ARM11_Firmware\HTS_LIM\diag\phaseA2_5_trial_lines.txt
find /c /v "" D:\HTS_ARM11_Firmware\HTS_LIM\diag\phaseA2_5_trial_lines.txt

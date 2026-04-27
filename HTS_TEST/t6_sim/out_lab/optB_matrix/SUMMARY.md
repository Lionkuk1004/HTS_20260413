# Phase B 옵션 B — 자동 매트릭스 결과


| 시나리오 | Build A (baseline) | Build B (REF only) | Build C (REF+APPLY) | 비고 |
|----------|--------------------|--------------------|---------------------|------|
| S1 | 100/100 (PASS) | 100/100 (PASS) | 100/100 (PASS) | |
| S5 200Hz | 100/100 (PASS) | 100/100 (PASS) | 100/100 (PASS) | |
| S5H 200Hz | 100/100 (PASS) | 100/100 (PASS) | 100/100 (PASS) | |
| S5H 500Hz | 0/100 (FAIL) | 0/100 (FAIL) | 0/100 (FAIL) | |
| S5H 2000Hz | 0/100 (FAIL) | 0/100 (FAIL) | 0/100 (FAIL) | |
| S5H 25000Hz | 0/100 (FAIL) | 0/100 (FAIL) | 0/100 (FAIL) | |

## 자동 판정

- **Step1 (B vs A)**: S1 동일 또는 로그 누락.
- **회귀 게이트 (C vs A)**: S1 / S5H 200Hz 유지 (또는 로그 없음).

## 로그

- `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\out_lab\optB_matrix\result_A.log`
- `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\out_lab\optB_matrix\result_B.log`
- `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\out_lab\optB_matrix\result_C.log`

※ 현재 T6 `test_S5_holographic` CFO 상한은 **25000Hz** (30000Hz 행 없음).
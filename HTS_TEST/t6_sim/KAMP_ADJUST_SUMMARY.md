# kAmp 조정 결과 요약 (v1000 → v100 시도)

## 회귀 기준

- Gate B-1: 10927 / 11200
- Gate B-4: 10927 / 11200
- Gate B-7: 10927 / 11200

## P_s / P_after 비교 (Clean + 주요 cell)

| cell | Phase A (kAmp=1000) P_s_pay | Phase B (kAmp=100) P_s_pay | Phase A P_after max (해당 행) | Phase B P_after max (해당 행) |
| --- | --- | --- | --- | --- |
| Clean (모든 mode, 4 cell) | 2000000.000000 | 20000.000000 | 2000000.000000 | 20000.000000 |
| AWGN -50 chase DATA | 2000000.000000 | 20000.000000 | 2028683542.742678 | 1080852261.850491 |
| CW 50 chase DATA | 2000000.000000 | 20000.000000 | 2147418113.000000 | 1320228599.900000 |
| Barrage 50 chase DATA | 2000000.000000 | 20000.000000 | 2029890909.446430 | 1076233470.314344 |

전 matrix 기준 `P_after` 최댓값: Phase A = 2147418113.000000, Phase B (kAmp=100 실행) = 1559201854.293020 (`HARQ_Matrix_Results_kAmp100_attempt.csv`).

## jsr_meas_db 비교 (chase DATA Barrage, intended = intensity dB_JSR)

| intended (dB) | Phase A jsr_meas | Phase B jsr_meas |
| --- | --- | --- |
| 30 | 27.302029 | 29.997492 |
| 40 | 29.484891 | 40.019527 |
| 50 | 30.060141 | 47.308626 |

## crc_rate 비교 (chase DATA, crc_ok/total)

| cell | Phase A crc | Phase B crc |
| --- | --- | --- |
| Clean 0.00 | 10/10 | 0/10 |
| Barrage 5 | 10/10 | 0/10 |
| Barrage 15 | 2/10 | 0/10 |
| Barrage 20 | 0/10 | 0/10 |
| Multi 20 | 8/10 | 0/10 |
| Multi 25 | 2/10 | 0/10 |
| CW 20 | 5/10 | 0/10 |
| CW 25 | 0/10 | 0/10 |

## Gate B-5/B-6 결과 (실측)

- kAmp=100 빌드 산출물: `HARQ_Matrix_Results_kAmp100_attempt.csv`, `stepB6_full.log`.
- Clean 4 cell 전부 `crc_ok=0`, `pre=0`, `hdr=0`, `dec=0` (Phase B).
- `P_s_pay=20000` (4 Clean cell 동일).
- 지시서 Gate B-5·B-6 합격 기준 미달 → `HTS_Harq_Matrix_Test.cpp` 는 **kAmp=1000** 으로 복구, CSV `kAmp` 열 제거 상태로 되돌림.
- 현재 워킹트리 `HARQ_Matrix_Results.csv` 는 kAmp=1000 재실행으로 재생성됨.

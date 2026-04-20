# Phase C HARQ 매트릭스 결과 요약

## 1. 회귀 기준

- Gate C-1: 10927 / 11200 (`stepC1_t6.log`)
- Gate C-6: 10927 / 11200 (`stepC6_t6.log`)
- Gate C-9: 10927 / 11200 (`stepC9_t6_final.log`)

## 2. HARQ 루프 정상성 (Smoke 1 - Clean)

| fec_path | mode | crc_ok | harq_k_avg | harq_k_max | s_k1 |
| --- | --- | --- | --- | --- | --- |
| chase | DATA | 100/100 | 1.000000 | 1 | 100 |
| chase | VOICE | 100/100 | 1.000000 | 1 | 100 |
| ir_harq | DATA | 100/100 | 1.000000 | 1 | 100 |
| ir_harq | VOICE | 100/100 | 1.000000 | 1 | 100 |

(출처: `HARQ_Matrix_Results.csv` Clean 행 4개)

## 3. Transition 영역 HARQ (chase DATA, Barrage dB_JSR)

| JSR | crc/100 | harq_k_avg | harq_k_max | latency_avg_chips | s_k1 | s_k2 | s_k3 | s_k4 | s_k5 | s_k6 | s_k7 | s_k8 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 10 | 100/100 | 1.000000 | 1 | 11968.0 | 100 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 11 | 100/100 | 1.000000 | 1 | 11968.0 | 100 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 12 | 100/100 | 1.000000 | 1 | 11968.0 | 100 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 13 | 100/100 | 1.000000 | 1 | 11968.0 | 100 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 14 | 95/100 | 1.350000 | 8 | 15820.8 | 95 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 15 | 17/100 | 6.810000 | 8 | 75924.5 | 17 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 16 | 0/100 | 8.000000 | 8 | 89024.0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 17 | 0/100 | 8.000000 | 8 | 89024.0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 18 | 0/100 | 8.000000 | 8 | 89024.0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 19 | 0/100 | 8.000000 | 8 | 89024.0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 20 | 0/100 | 8.000000 | 8 | 89024.0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

## 4. chase vs ir_harq 비교 (동일 channel / intensity)

| channel | intensity | chase crc/100 | ir_harq crc/100 | chase harq_k_avg | ir_harq harq_k_avg |
| --- | --- | --- | --- | --- | --- |
| Barrage | 15 | 17/100 | 18/100 | 6.810000 | 6.740000 |
| Barrage | 18 | 0/100 | 0/100 | 8.000000 | 8.000000 |
| CW | 20 | 35/100 | 44/100 | 5.550000 | 4.920000 |
| CW | 22 | 4/100 | 5/100 | 7.720000 | 7.650000 |
| Multi | 15 | 86/100 | 87/100 | 1.980000 | 1.910000 |

## 5. Phase A (n=10, 단일샷) 대비 (지시서에 제시된 Phase A 수치 vs Phase C)

| cell | Phase A crc/10 | Phase C crc/100 | Phase C harq_k_avg |
| --- | --- | --- | --- |
| chase DATA Barrage 15 | 2/10 | 17/100 | 6.810000 |
| chase DATA CW 20 | 5/10 | 35/100 | 5.550000 |
| chase DATA Multi 20 | 8/10 | 51/100 | 4.430000 |

## 6. 실행 규모·시간

- total_cells: 352, N_MC: 100, total_trials: 35200
- `stepC7_smoke.log` / `stepC_harq_full.log` 마지막 줄: Elapsed: 97.2 s

## 7. 파일 목록

- `HARQ_Matrix_Results.csv` (353행 = 헤더 + 352 cell)
- `stepC1_t6.log`, `stepC5_build.log`, `stepC6_t6.log`, `stepC7_smoke.log`, `stepC_harq_full.log`, `stepC9_t6_final.log`

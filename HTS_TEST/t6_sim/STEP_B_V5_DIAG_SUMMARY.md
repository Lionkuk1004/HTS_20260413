# Step B — v5 Walsh-row DIAG 분포

로그: `stepB6_harq_on.log` (약 111 MB, `STAGE4-V5-2B` **35200** 라인)  
CSV 정렬 가정: `HARQ_Matrix_Results_STEPB_ON.csv` 행 순서 = 매트릭스 셀 순서, 셀당 **100** trial (`kTrials`).

## 관측 요약 (실측)

HARQ Matrix PC 하네스에서 **모든** `STAGE4-V5-2B` 라인이 `off=0`, `v5_score=0`, `legacy_seed=(0,0)` 이었고, 같은 구간의 `[P0-SEED] dot=(0,0) est=(0,0)` 과 일치한다.  
즉 v5 DIAG 코드는 호출되나, 이 빌드/경로에서 `p0_buf128_*` 기준 legacy 시드·FWHT 입력 에너지가 **상수 0**으로 보이며, **셀별(Clean / Swept / Multi / AWGN / Barrage) 분산 비교는 무의미**(평균·분산 모두 0).

| 구간 | trial 수 | 평균 v5_score | 최소 | 최대 | 비고 |
|------|----------|---------------|------|------|------|
| Clean | 400 | 0 | 0 | 0 | DATA+VOICE×IR+CHASE Clean 4셀×100 |
| Swept 22 dB_JSR | 400 | 0 | 0 | 0 | intensity 22.00 행 4셀×100 |
| Multi 17 dB_JSR | 400 | 0 | 0 | 0 | channel `Multi`, intensity 17.00 |
| AWGN -16 dB_SNR | 400 | 0 | 0 | 0 | |
| Barrage 18 dB_JSR | 400 | 0 | 0 | 0 | |

## Legacy seed (walsh63_dot_) 대비

- `v5_score` 분산이 0 → 피어슨 상관 **정의 불가 (NaN)**.
- Step C 이전에 **T6_SIM 또는 IQ 주입 경로**에서 P0 버퍼·`best_off`·`seed_dot_*` 가 0이 아닌지 대조 권장 (본 Step B 목표는 “비개입 DIAG + 회귀·CSV 동일” 충족).

## 게이트 관점

- **Gate B-6-a**: `STAGE4-V5-2B` **35200** 라인, `v5_score` 범위 **[0, 0]** (양의 정수 조건은 “overflow 없음” 수준으로만 충족; 신호 의미 해석은 상기 제약).
- **Gate B-6-b**: `HARQ_Matrix_Results_STEPB_OFF.csv` vs `STEPB_ON.csv` **바이너리 동일** (`fc /b`).

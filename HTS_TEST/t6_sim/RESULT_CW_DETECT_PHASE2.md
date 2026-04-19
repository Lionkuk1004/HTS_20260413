# CW 탐지 Phase 2 — 2D Joint + Band 검증

일자: 2026-04-19  
매크로: `HTS_CW_DETECT_DIAG_V2` (계측만, 복조·Excision 로직 변경 없음)  
실행: `build_and_test_cw_detect_v2.bat` → `cw_v2_stdout.log` / `cw_v2_stderr.log`

## 빌드

- 결과: **PASS**

## 회귀

- stdout: **정량 합계 10927 / 11200 (97.6%)**

## 라벨 정의 (T6)

| 라벨 | 시나리오 |
|------|-----------|
| CLEAN | S1, S2, S4, S5, S6, **S10**(내구 10000 + LPI 일부) |
| LOWSNR | S3 |
| BARRAGE | S7 |
| CW | S8 |
| MIXED | S9 |

**주의:** S10 내구 구간이 CLEAN 라벨에 포함되어 누적 심볼 수가 CLEAN에 과대 반영된다 (라벨 요약 해석 시 참고).

## 2D “CW_medium” 영역 비율 (누적 joint, conc 0.85–1.0 & ent 1.5–2.5 bit)

| 라벨 | 심볼 수 | `frac_in_CW_medium_2D` |
|------|---------|-------------------------|
| CLEAN | 3,705,045 | **0.005582** (약 0.56%) |
| LOWSNR | 55,764 | **0.003640** |
| BARRAGE | 45,502 | **0.000044** |
| CW | 20,640 | **1.000000** |
| MIXED | 7,054 | **0.000000** |

→ Q1/Q2: **CW 라벨만** 2D CW_medium 셀에 완전히 몰리고, Barrage·Mixed는 사실상 0에 가깝다. CLEAN/LOWSNR은 소량 침범(단일 row + 저엔트로피 클린 지문과 일치).

## Band 후보별 심볼 비율 (라벨별 %, stderr 표 발췌)

### CW 지향 후보 (TP = CW 열)

| Band | CLEAN | BARRAGE | CW | LOWSNR | MIXED | max non-CW FP | TP−maxFP (심볼 %) |
|------|-------|---------|-----|--------|-------|----------------|-------------------|
| CW_tight | 0.558 | 0.000 | **100** | 0.360 | 0.000 | 0.558 | **99.442** |
| CW_loose | 0.846 | 0.057 | **100** | 7.641 | 0.071 | 7.641 | **92.359** |
| CW_medium | 0.558 | 0.004 | **100** | 0.364 | 0.000 | 0.558 | **99.442** |
| CW_hiConc | 0.865 | 0.004 | **100** | 2.310 | 0.000 | 2.310 | **97.690** |
| CW_midEnt | 0.559 | 0.015 | **100** | 12.162 | 0.071 | 12.162 | **87.838** |

### 참조용 non-CW 밴드

| Band | CLEAN | BARRAGE | … |
|------|-------|---------|---|
| Clean_tight | 99.056 | 0.068 | … |
| Barrage | 0.000 | 69.120 | … |
| LowSNR | 0.000 | 43.438 | … |

## 패킷 단위 (심볼 ≥50%가 밴드 안이면 hit)

| Band | CLEAN 패킷% | CW 패킷% |
|------|-------------|----------|
| CW_tight | 0.375 | **100** |
| CW_medium | 0.375 | **100** |
| CW_loose | 0.741 | **100** |
| CW_hiConc | 0.891 | **100** |
| CW_midEnt | 0.375 | **100** |

(LOWSNR 일부 패킷은 CW_loose / CW_midEnt 등에서 12.903%까지 올라가 밴드가 넓을수록 FP가 커짐.)

## Window 크기 (CW_medium 2D 윈도 히트, per-scenario 출력)

대표적으로 S1·S2 등 per-scenario 블록에서 **win4 / win8 / win16** CW_medium 구간 집계가 **0**인 경우가 많다. 이유: **심볼 단위** 엔트로피가 클린에서 ~0 bit에 가깝고, **슬라이딩 평균** 엔트로피가 1.5–2.5 구간에 들어가기 어렵기 때문이다.  
→ Q4: “윈도 평균 2D”만으로는 클린/CW 분리 지표로 쓰기엔 **심볼 단위 2D**가 더 직접적이다. 윈도 크기 비교는 **저엔트로피 구간**을 포함한 별도 ROI에서 추가 검토하는 편이 낫다.

## 해석 (판정)

### A에 가까움 (심볼 단위, 중·좁은 CW 밴드)

- **CW_medium / CW_tight**: CW TP 100%, CLEAN·BARRAGE·MIXED 대비 max FP ≈ **0.56%** 수준 → Phase 1에서 예상한 “2D joint로 CW 영역 분리”가 **심볼 단위**에서는 강하게 지지된다.
- **CW_loose / CW_midEnt**: LOWSNR 침범이 커져 **B (부분 분리)** 쪽 — 임계를 넓히면 FP가 급증.

### 윈도 3종

- 동일 ROI `[0.85,1]×[1.5,2.5]`에서 win4/8/16 히트가 거의 0 → **이 ROI는 윈도 평균용이 아님** (Q4 답: 본 ROI 기준 최적은 “윈도 미사용 또는 ROI 재정의”).

## 최종 판별 수식 초안 (심볼 단위, 계측 기반)

```
CW_detect_sym =
  (CONC_SYM >= 0.85) AND (CONC_SYM <= 1.00)
  AND (H_SYM >= 1.50) AND (H_SYM <= 2.50)
```

- `CONC_SYM`: FWHT 행 에너지 top4 / 총에너지 (본 구현과 동일).
- `H_SYM`: 동일 64(또는 nc) bin에 대한 Shannon entropy (bit).
- **윈도**: 위 ROI로는 유효 hit가 거의 없으므로, 별도 저·중엔트로피 ROI를 정의한 뒤 win8을 재검증할 것.

**수치 초안 (본 로그 기준):** `CONC_TH_LO=0.85`, `CONC_TH_HI=1.00`, `ENT_LOW=1.50`, `ENT_HIGH=2.50`, **window = 보류** (또는 ent ROI 재설계 후 8칩 재측정).

## 다음 단계

- Phase 3 Excision 전: **S10을 CLEAN 요약에서 분리**하거나, 내구 trial 샘플링으로 계측량 축소.
- 윈도 기반 판정을 쓸 경우: **ent ROI를 심볼 분포에 맞게** (예: 클린 low-H vs CW mid-H) 별도 2D 맵 설계.
- Phase 2.5: `CW_loose` 대비 **LOWSNR** FP(7.6%)를 줄이는 복합 조건(예: conc 상한 + LLR) 검토.

## 자진 고지

- 패킷 판정: 심볼의 **≥50%**가 밴드 안일 때 패킷 hit — 임의 근사이며 Excision과 1:1 대응은 아님.
- `count_joint_region`은 conc/ent **bin 경계**에 기반한 근사 포함.
- CLEAN 라벨에 **S10 10000회** 포함으로 라벨 합산 비중이 CLEAN으로 쏠림 — FP “0.56%”는 이 혼합에 대한 관측치.

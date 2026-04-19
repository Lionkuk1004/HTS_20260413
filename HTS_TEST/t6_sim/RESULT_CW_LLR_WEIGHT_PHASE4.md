# CW LLR Weighting Phase 4 — 구현 + MC 교차 측정

일자: 2026-04-19  
매크로: `HTS_CW_LLR_WEIGHT_V1` (미정의 시 코드 경로 없음 → baseline 동일)  
빌드 파라미터: `HTS_CW_W_TOP_K` (기본 2), `HTS_CW_W_FACTOR` (0.5 / 0.25 / 0.1)  
진단: `HTS_CW_W_DIAG` (MC 전용 권장)

## 구현 요약

- **신규:** `HTS_LIM/HTS_CW_LLR_Weight.hpp`, `HTS_CW_LLR_Weight.cpp` — Phase 2 판별 재사용, **엔트로피는 FWHT 행 순서(0…nc−1) 에너지**로 계산 (Phase 3 excision과 동일).
- **`Bin_To_LLR`:** `fI`/`fQ`는 변경하지 않고, `corr = fI[m]+fQ[m]`에 대해 CW로 판정된 **top-K 행**만 Q8 고정소수 `weight_factor` 배로 스케일 후 기존 Polar max-log / Conv sum 루프에 반영.
- **링크:** `HTS_FEC_HARQ.cpp` 파일 끝 `#if defined(HTS_CW_LLR_WEIGHT_V1)` 로 `HTS_CW_LLR_Weight.cpp` include (단일 TU 패턴, Phase 3 excision과 동일).
- **Polar IR (`Decode64_IR` 내 에너지 max-log):** `Bin_To_LLR`를 거치지 않으므로 **본 Phase 4 변경은 적용되지 않음** (`HTS_FEC_POLAR_DISABLE` MC / 최소 T6 빌드에서는 Conv `Bin_To_LLR` 경로만 해당).

## 빌드

| 산출물 | 스크립트 | 결과 |
|--------|-----------|------|
| `HTS_T6_MC_W_OFF.exe` | `t6_mc/build_mc_w_off.bat` | **PASS** |
| `HTS_T6_MC_W_050.exe` | `t6_mc/build_mc_w_050.bat` | **PASS** |
| `HTS_T6_MC_W_025.exe` | `t6_mc/build_mc_w_025.bat` | **PASS** |
| `HTS_T6_MC_W_010.exe` | `t6_mc/build_mc_w_010.bat` | **PASS** |

## T6 회귀 (Polar 비활성, WRC=0, 최소 TU)

`build_and_test_t6_w_off.bat` … `build_and_test_t6_w_010.bat` 실행.

| Weight | grand_pass (stdout `정량 합계`) | Baseline 대비 |
|--------|----------------------------------|----------------|
| OFF | **10927 / 11200** | +0 |
| 0.50 | **10927 / 11200** | +0 |
| 0.25 | **10927 / 11200** | +0 |
| 0.10 | **10927 / 11200** | +0 |

→ 매크로 OFF baseline **10927 유지**; ON(세 가중치)에서도 **동일** — 비-CW 경로에서는 판정 미발화 또는 가중치가 디코드 결과에 반영되지 않는 realization 위주로 해석 가능.

## MC 비교 (N=1000, base_seed=`0xABCDEF01`)

실행 예: `HTS_T6_MC_W_OFF.exe 1000 0xABCDEF01 > mc_w_off_1000.log 2>mc_w_off_1000.err` (가중치 ON 빌드는 `HTS_CW_W_DIAG`로 stderr에 통계).

### 시나리오별 pass rate

모든 가중치(0.5 / 0.25 / 0.1)에서 **OFF와 pass 수·95% CI가 동일** (아래는 공통 값).

| Scenario | OFF / w=0.5 / w=0.25 / w=0.1 (동일) | 95% CI (동일) |
|----------|--------------------------------------|----------------|
| MC_S1_Clean | 612 / 1000 (61.2%) | [0.5810, 0.6423] |
| MC_S3_LowSNR | 17 / 1000 (1.7%) | [0.0099, 0.0271] |
| MC_S7_Barrage | 279 / 1000 (27.9%) | [0.2514, 0.3079] |
| MC_S8_CW_Freq | 0 / 1000 (0%) | [0, 0.00368] |
| MC_S8_CW_Multi | 24 / 1000 (2.4%) | [0.0154, 0.0355] |
| MC_S9_Combined | 0 / 1000 (0%) | [0, 0.00368] |

**최적 Δ:** MC pass 기준으로는 **전 시나리오 Δ=0** (개선 없음).

### CI overlap

OFF vs 각 weight: **완전 동일** → 유의한 pass 차이 **없음**.

### 보조: LLR_BIN 평균 (가중치에 따라 소폭 변화)

예: MC_S1 — OFF `31.113` vs w=0.1 `30.374`; MC_S8_CW_Multi — OFF `1.477` vs w=0.1 `1.331`. pass 수는 변하지 않음.

### Weight 통계 (`HTS_CW_W_DIAG`, w=0.5 / 0.25 / 0.1 공통 수치)

`detect_per_symbol` 호출은 가중치와 무관 → 세 ON 빌드 모두 동일:

- `total_symbols` = **834,082**
- `cw_detected` = **7,986**
- `weighted_rows_total` = **15,972** (= 7,986 × top_k 2)
- 상위 row hit (일부): `0:771 1:260 2:742 3:285 4:253 5:278 6:231 7:276` …

## 해석 (분기)

- **A (기대 성공):** 해당 없음 — MC_S8 pass **개선 없음**.
- **B (부분):** LLR_BIN 등 2차 지표만 미세 변화, pass 불변.
- **C (효과 없음):** MC pass 기준 **Phase 3와 유사** — 병목이 `Bin_To_LLR` 이전/이후 또는 시나리오 정의 쪽일 가능성.

**최적 weight (실측):** pass 기준으로는 **임의** (차이 없음). 후속은 **Polar IR 경로 정렬**, **Pre-FWHT/다른 관측점**, 또는 **더 강한 가중/게이팅(수식은 유지)** 등 Phase 4.5 검토.

## 다음 단계

- Polar 켜진 제품 빌드: IR 에너지 max-log 루프에 **동일 판별·스케일** 이식 여부 결정.
- MC S8_Freq의 `LLR_BIN=0` 등 harness 특이점 별도 확인(Phase 3 기록과 동일 계열).

## 자진 고지

- `HTS_CW_W_FACTOR` 는 컴파일 타임 매크로; 런타임 튜닝은 미구현.
- `top_k=2` 고정 샘플.
- `tpe_clamp_llr` 등 후단과의 상호작용은 기존과 동일.
- MEDIAN/Excision(`HTS_CW_EXCISION_V1`)과 독립; 둘 다 OFF가 기본.

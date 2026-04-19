# CW Excision Phase 3 — 구현 + MC 교차 측정

일자: 2026-04-19  
매크로: `HTS_CW_EXCISION_V1` (기본 미정의 = OFF, baseline 영향 없음)  
탐지·적용: Phase 2 고정 규칙 (top-4 concentration ≥ 0.85, Shannon H ∈ [1.50, 2.50] bit, **행 순서** 에너지로 H 계산)  
구현: `HTS_LIM/HTS_CW_Excision.hpp`, `HTS_LIM/HTS_CW_Excision.cpp` — `HTS_FEC_HARQ.cpp` 끝에서 `#if defined(HTS_CW_EXCISION_V1)` 로 단일 TU에 `#include "HTS_CW_Excision.cpp"` (별도 링크 소스 불필요).

## 빌드

| 산출물 | 정의 | 결과 |
|--------|------|------|
| `HTS_TEST/t6_mc/HTS_T6_MC_EX_OFF.exe` | `HTS_CW_EXCISION_V1` 없음 | **PASS** (`build_mc_excision_off.bat`) |
| `HTS_TEST/t6_mc/HTS_T6_MC_EX_ON.exe` | `/DHTS_CW_EXCISION_V1 /DHTS_CW_EX_MODE=1` + `/DHTS_CW_EX_DIAG` | **PASS** (`build_mc_excision_on.bat`) |

모드: **HARD_NULL**, `excise_top_k=2` (`default_build_options()`).

## 회귀 (T6 SIM, Polar 비활성·WRC=0, 최소 TU)

동일 플래그로 `HTS_T6_SIM_Test.cpp` + `HTS_Session_Derive_Stub.cpp` + `HTS_Walsh_Row_Converter.cpp` 만 링크한 로컬 검증:

| 구성 | 정량 합계 (stdout) |
|------|---------------------|
| `HTS_CW_EXCISION_V1` **미정의** (baseline) | **10927 / 11200** (97.6%) |
| `HTS_CW_EXCISION_V1` **정의** (HARD_NULL) | **10907 / 11200** (97.4%) |

→ 매크로 OFF 시 **10927 유지** 확인. ON 시 소폭 하락(−20건) — Clean FP 유사 오탐에 가까운 국면 가능.

## MC 비교 (N=1000, base_seed=`0xABCDEF01`)

실행:

- `HTS_T6_MC_EX_OFF.exe 1000 0xABCDEF01 > mc_ex_off_1000.log`
- `HTS_T6_MC_EX_ON.exe  1000 0xABCDEF01 > mc_ex_on_1000.log`  
  (ON 빌드는 `HTS_CW_EX_DIAG` 포함 → excision 요약은 stderr)

### 시나리오별 pass rate

| Scenario | OFF pass | OFF 95% CI | ON pass | ON 95% CI | Δ | 판정 |
|----------|----------|--------------|---------|-------------|---|------|
| MC_S1_Clean | 612/1000 (61.2%) | [0.5810, 0.6423] | 612/1000 (61.2%) | [0.5810, 0.6423] | 0 | CI 동일 → **차이 확정 못함** (실질 동일) |
| MC_S3_LowSNR | 17/1000 (1.7%) | [0.0099, 0.0271] | 17/1000 (1.7%) | 동일 | 0 | 동일 |
| MC_S7_Barrage | 279/1000 (27.9%) | [0.2514, 0.3079] | 279/1000 (27.9%) | 동일 | 0 | 동일 |
| MC_S8_CW_Freq | 0/1000 (0%) | [0, 0.00368] | 0/1000 (0%) | 동일 | 0 | 동일 |
| MC_S8_CW_Multi | 24/1000 (2.4%) | [0.0154, 0.0355] | 24/1000 (2.4%) | 동일 | 0 | 동일 |
| MC_S9_Combined | 0/1000 (0%) | [0, 0.00368] | 0/1000 (0%) | 동일 | 0 | 동일 |

### 보조 지표 (로그 요약)

- **MC_S1:** OFF `LLR_BIN mean=31.113` vs ON `30.368` — 소폭 변화, pass 수는 동일.
- **MC_S8_CW_Multi:** OFF `LLR_BIN mean=1.477` vs ON `1.332` — 미세 변화, pass 수는 동일.
- **Row H / Row conc (S1,S3,S7):** OFF/ON **동일** (진단 샘플이 excision 전 FWHT 경로와 정렬된 경우 기대 가능).

### Excision 통계 (`HTS_CW_EX_DIAG`, ON 실행 stderr)

- `total_symbols` = **1,081,128**
- `detected` = **15,453** (≈ **1.43%** of symbols)
- `excised` = **15,453** (탐지 시 항상 적용)
- 상위 excised row (idx:count 일부): `0:1059 1:514 2:1002 3:569 4:497 5:550 6:455 7:544` …

### 3모드 비교 (선택)

이번 실행에서는 **HARD_NULL만** 측정. SOFT_ATTEN / MEDIAN_SUB 는 `HTS_CW_EX_MODE=2|3` 및 필요 시 `/DHTS_CW_EX_ATTEN=…` 로 동일 harness 재빌드하면 됨.

## 해석 (Phase 3 분기)

- **MC 관점:** S8/S9 pass는 **OFF/ON 동일** → MC pass 지표만으로는 “CW 시나리오 대폭 개선”은 **관측되지 않음** (템플릿 C 요소: 개선 미미 / 시나리오·지표가 병목일 수 있음).
- **T6 관점:** 전체 회귀에서 ON 시 **10927 → 10907** — **Clean 유사 손실**이 MC S1 pass 수에는 나타나지 않았으나, 긴 시나리오 스위트에서는 누적될 수 있음 → **B(Trade-off) 성분** 검토: `SOFT_ATTEN` 또는 FP 억제용 추가 게이트(Phase 3.5) 후보.
- **S7 Barrage:** MC pass 동일 — 본 탐지가 주로 CW-like FWHT 분포에서만 발화한다는 방향과 정성적으로 일치.

## 다음 단계

| 분기 | 조건 | 제안 |
|------|------|------|
| **A** | MC S8·S9 pass 유의 개선 + Clean 유지 | Phase 4 AJC 통합 검토 |
| **B** | T6/ Clean 측면 손실 + MC는 둔감 | Phase 3.5: SOFT_ATTEN, `excise_top_k`, 또는 탐지 임계는 유지하되 **적용 조건** 재검토 (수식 변경 없이 게이팅만) |
| **C** | 구현/위치 의심 | `Decode64_IR` conv·polar·`Decode16_IR` 삽입 순서·`nc` 경로 재확인, S8_Freq LLR=0 별도 harness 이슈 여부 확인 |

본 결과는 **B + C 혼합**에 가깝게 정리됨.

## 자진 고지

- `excise_top_k=2` 는 스펙상 임의 기본값.
- Phase 2의 non-CW FP (~0.56%)는 **심볼 단위**; T6/MC의 pass/bit은 **상위 계층** 지표라 직접 대응하지 않을 수 있음.
- CW 에너지가 여러 FWHT row에 퍼지면 top-2 null만으로는 제한적일 수 있음.
- MEDIAN_SUB 모드는 이번 배치에서 미실행.

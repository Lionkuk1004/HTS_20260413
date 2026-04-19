# LLR 분포 계측 결과 (전략 A)

## Discovery (착수 전 원문 확인 요약)

### DISC-L1 — `Bin_To_LLR` (`HTS_FEC_HARQ.cpp`)

- **위치:** `FEC_HARQ::Bin_To_LLR` (약 391행대).
- **입력:** `const int32_t *fI`, `const int32_t *fQ`, `nc`, `bps`, 출력 `int32_t *llr`.
- **시프트:** `#if defined(HTS_FEC_POLAR_ENABLE)` 분기에서 **Polar: `raw >> 10`**, 그렇지 않으면 **Conv: `raw >> 12`**.
- **클램프:** 함수 **내부에는 없음**. 호출부(`Decode64_IR` 등)에서 `tpe_clamp_llr` → **±500000** (`tpe_clamp_llr` / `tpe_sat_add_llr` 주석 176–192행).

### DISC-L2 — `Decode64_IR` IR 누적

- **Conv 경로 (`#else`):** `Bit_Deinterleave` 후 `ir_state.llr_accum[i] = tpe_sat_add_llr(slot, wb.ru.all_llr[i])` (688 슬롯, `TOTAL_CODED`).
- **Polar 경로 (`HTS_FEC_POLAR_ENABLE`):** 심볼 루프에서 `polar_llr_coded` 합산 → 동적 시프트 후 **int16 `polar_llr16`**으로 폴드(±32767) → `HTS_Polar_Codec::Decode_SCL`.

### DISC-L3 — Polar SCL PM (`HTS_Polar_Codec.cpp`)

- **`scl_PM[]`:** 경로별 누적 패널티(낮을수록 유리).
- **종료:** `order[]`로 PM 정렬 후 전 경로 CRC 검사, **CRC 통과 경로 중 PM 최소**를 상수시간 스캔으로 선택.

---

## 빌드

- **결과:** PASS (`build_and_test_llr_diag.bat`, `HTS_T6_LLR_DIAG.exe`).
- **정의:** `/DHTS_LLR_DIAG`, 단일 TU + `HTS_LLR_Diag.cpp` 링크.

## 회귀

- **grand_pass:** **10927 / 11200 (97.6%)** — baseline 10927 유지.

---

## 4 지점 정의 (계측 의미)

| 지점 | 위치 | 기록 값 |
|------|------|---------|
| **FWHT_OUT** | `Decode64_IR` 심볼 루프, `fec_ir_fwht_bin_unshift` 직후 | `1e6 × (E_row63 / (mean_bin_energy+1))` 정수 스케일(“soft 품질” 프록시) |
| **BIN_TO_LLR** | `Bin_To_LLR` 끝, 시프트 직후 각 `llr[b]` | 실제 비트 LLR(클램프 전) |
| **IR_ACCUM** | Polar: `polar_llr16[]` 폴드 직후 / Conv: `llr_accum[]` HARQ 누적 직후 | 스테이지 입력에 가까운 누적 LLR |
| **POLAR_PM** | `Decode_SCL` CRC 검사 직후 | **CRC 통과** 경로들 중 **최소 PM** 1샘플/디코드 호출 |

---

## 시나리오별 요약 (로그: `llr_diag_stderr.log`)

### S1 (Clean)

| 지점 | E[\|LLR\|] | Var[\|LLR\|] | clamp_hits | 비고 |
|------|-----------|---------------|------------|------|
| FWHT_OUT | ~0.046 | ~0.044 | 0 | 다수 샘플 ratio→0(평균 에너지 대비 row63 미약) |
| BIN_TO_LLR | ~124.2 | ~0.83 | 0 | mag 히스토그램 bin4 집중(100–500) |
| IR_ACCUM | ~5202 | ~3.38e6 | 0 | 1K–5K·5K–10K 구간 위주 |
| POLAR_PM | 0 | 0 | 0 | CRC 통과 PM 샘플이 0이거나 기록값 0 |

### S3 (저 SNR 워터폴)

| 지점 | E[\|LLR\|] | Var[\|LLR\|] | clamp_hits |
|------|-----------|---------------|------------|
| FWHT_OUT | ~5.52e5 | ~5.12e11 | 0 |
| BIN_TO_LLR | ~51.7 | ~2.19e3 | 0 |
| IR_ACCUM | ~2037 | ~1.47e6 | 0 |
| POLAR_PM | ~73.4 | ~2.41e5 | 0 |

### S7 (재밍 계열)

| 지점 | E[\|LLR\|] | Var[\|LLR\|] | clamp_hits |
|------|-----------|---------------|------------|
| FWHT_OUT | ~7.58e5 | ~6.53e11 | 0 |
| BIN_TO_LLR | ~36.6 | ~1.53e3 | 0 |
| IR_ACCUM | ~1898 | ~1.59e6 | 0 |
| POLAR_PM | ~165 | ~2.25e5 | 0 |

### S10 (내구 10000 + LPI)

- **FWHT_OUT** E[\|LLR\|] ~108, **BIN** ~124.2, **IR** ~5200, **POLAR_PM**은 다수 호출에서 PM=0만 기록되어 E[\|LLR\|]=0에 가깝게 보일 수 있음(경로 선택·CRC 실패 혼재).

원문 히스토그램·전 시나리오(S2–S9)는 `llr_diag_stderr.log`의 `=== LLR Diag [Sx] ===` 블록 참고.

---

## 해석

### A. Clean(S1/S4/S8 등)에서 BIN/IR이 안정·FWHT 프록시가 낮거나 희소

- **BIN** E[\|LLR\|] ~124, 분산 작음 → 양자화(>>10/12) 이후에도 **좁은 마그니튜드 밴드**에 머무름.
- **IR** E[\|LLR\|] ~5200대 → HARQ 누적 후 스케일이 크게 증가(기대 현상).

### B. S3/S7에서 FWHT_OUT·IR 분산 급증

- 채널 열화에 따라 **FWHT row63 대비 평균 에너지 비율**과 **IR 누적 분산**이 함께 커짐 → BER 없이도 **열화 정량** 가능.

### C. clamp_hits (본 빌드)

- **BIN / IR / Polar int16** 구간에서 `clamp_hits`는 **0**에 가까움(±500000 / ±32767 대비 미포화).

### D. 단계 간 E[\|LLR\|] 변화

- **BIN → IR:** 평균 절대값이 크게 증가 → 누적이 **신뢰도 스케일**을 키움.
- **FWHT_OUT → BIN:** 서로 다른 스케일(에너지 비율 ×1e6 vs 시프트 LLR)이므로 **절대값 비교는 프록시 간 비교용**으로만 사용.

---

## 다음 단계

1. **FWHT_OUT**과 **BIN/IR** 스케일을 동일 단위로 맞춘 파생 지표(예: dB 스케일)를 정의하면 체인 비교가 쉬워짐.
2. **`record_llr_with_truth` + 하네스**로 clean 시나리오 부호 정합률 측정.
3. **`HTS_FEC_POLAR_DISABLE` 실효성:** `HTS_FEC_HARQ.hpp`가 `HTS_FEC_POLAR_ENABLE`을 강제 정의하면 CLI `/D`만으로는 Conv 경로로 전환되지 않을 수 있음 — Polar/ Conv A·B 비교 시 **헤더 가드 정합** 필요.

---

## 자진 고지

- **계측 위치:** 위 표 및 소스 `#if defined(HTS_LLR_DIAG)` 주석 인근.
- **샘플 수:** 시나리오·trial·성공 디코드 횟수에 따라 지점별로 크게 다름; `POLAR_PM`은 **CRC 통과 경로가 있을 때만** 1회 기록.
- **FWHT_OUT:** 정수 비율 프록시라 **S1처럼 0이 많은 구간**이 생길 수 있음(평균 대비 row63이 상대적으로 작을 때).
- **POLAR_PM=0:** “패널티 없음” 또는 **선택 로직과의 정합** 추가 검토 여지.
- **선택 (b):** 본 세션에서는 `record_llr_with_truth`를 API로만 두고 T6에서는 호출하지 않음(`correct_sign`은 0).

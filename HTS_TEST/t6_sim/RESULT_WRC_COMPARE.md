# WRC 3방식 비교 결과

**일자:** 2026-04-19
**측정:** `HTS_TEST/t6_sim` — `HTS_T6_SIM_Test.cpp` + `HTS_Session_Derive_Stub.cpp` + `HTS_LIM/HTS_Walsh_Row_Converter.cpp`
**플래그:** `/I..\..\HTS_LIM` `/DHTS_ALLOW_HOST_BUILD` `/DHTS_FEC_SIMULATE_M4_RAM_LAYOUT` `/DHTS_FEC_POLAR_DISABLE`

## 빌드 결과

| 구성 | 결과 |
|------|------|
| Baseline (`/DHTS_WRC_METHOD=0`) | **PASS** (C4996 `strncpy` 4건 — 기존과 동일) |
| Method A (`/DHTS_WRC_METHOD=1`) | **PASS** |
| Method B (`/DHTS_WRC_METHOD=2`) | **PASS** |
| Method C (`/DHTS_WRC_METHOD=3`) | **PASS** |

## 회귀 결과 (grand_pass / grand_total)

로그: `wrc_baseline.log`, `wrc_method_A.log`, `wrc_method_B.log`, `wrc_method_C.log`

| 빌드 | 정량 합계 (원문) |
|------|------------------|
| Baseline | `10927 / 11200` (97.6%), BER 0.02438 |
| Method A | `10927 / 11200` (97.6%), BER 0.02438 |
| Method B | `10927 / 11200` (97.6%), BER 0.02438 |
| Method C | `10927 / 11200` (97.6%), BER 0.02438 |

**증감 (대 Baseline):** A/B/C 모두 **0** (합계·BER 동일).

## 시나리오별 pass 증감 (Baseline 기준)

종합 표의 **pass 숫자**는 네 로그에서 **동일**함 (`fc /b` 시 파일 전체는 소요 시간 등 일부 바이트만 차이).

| 시나리오 | Method A | Method B | Method C |
|----------|----------|----------|----------|
| S1 ~ S10 (종합 표 pass) | 0 | 0 | 0 |

## 최고 성적 방식

**전체 pass 기준:** 네 구성 **동률** (10927 / 11200).

## 구현 요약 (지시 대비)

- **신규:** `HTS_LIM/HTS_Walsh_Row_Converter.hpp`, `HTS_LIM/HTS_Walsh_Row_Converter.cpp` — 지시문 알고리즘·매크로 분기 반영.
- **`HTS_FEC_HARQ.cpp`:** `Decode_Core` 루프에서 `memcpy`로 `fI`/`fQ` 채운 직후, **`FWHT` 호출 전**에만 `#if HTS_WRC_METHOD != 0` 블록 추가. `Viterbi`·`Bin_To_LLR`·Polar·IR 전용 루프 본문은 변경 없음.
- **배치:** `build_and_test_wrc_compare.bat` — `HTS_Session_Derive_Stub.cpp` 및 위 호스트/FEC 매크로 포함(링크·T6 관례).

## 자진 고지

1. **`Decode_Core` 시그니처**는 지시의 `const int16_t* sym_I`가 아니라 실제 코드대로 **`const int32_t *accI`, `const int32_t *accQ`** 입니다. WRC API는 **`int16_t*`** 이므로, 적용 시 **`fI`/`fQ`를 int16 범위로 포화한 뒤 `clean_chips` 호출 → 다시 int32로 되돌림** 했습니다 (HARQ 누적 int32 경로 보존).

2. **T6 SIM 기본 경로:** `HTS_T6_SIM_Test.cpp`의 `setup()`이 **`Set_IR_mode(true)`** 이고, `Build_Packet`은 **`PayloadMode::DATA`** 입니다. `try_decode_`의 DATA 분기는 **`ir_mode_`일 때 `Decode64_IR`** 를 사용하며, **`Decode_Core`는 `ir_mode_`가 꺼진 비-IR DATA 경로**에서만 사용됩니다. 따라서 **이번 T6 실측에서는 `Decode_Core` 내 WRC가 거의/전혀 실행되지 않아**, A/B/C와 Baseline의 **정량 합계가 동일**한 것으로 해석됩니다. WRC 효과를 T6로 보고하려면 IR 끄기·또는 `Decode64_IR` 전용 삽입 등 **별도 하네스**가 필요합니다.

3. **`HTS_Walsh_Row_Converter.hpp`**의 공개 함수명은 지시문이 `clean_chips_N`을 언급했으나, 본문 시그니처는 **`clean_chips`** 로 통일되어 있어 **`clean_chips`로 구현**했습니다.

4. **WRC `fwht_local` / FEC `FWHT`:** 지시대로 별도 구현(격리). 이중 FWHT 정의가 존재함(의도된 side-effect 격리).

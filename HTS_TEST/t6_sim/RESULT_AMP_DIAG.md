# Chip 진폭 분포 측정 결과

## 빌드

- **결과:** PASS (`build_and_test_amp_diag.bat`, C4996 `strncpy` 4건 — 기존 T6 SIM과 동일)
- **플래그:** `HTS_ALLOW_HOST_BUILD`, `HTS_FEC_SIMULATE_M4_RAM_LAYOUT`, `HTS_FEC_POLAR_DISABLE`, `HTS_AMP_DIAG`, `HTS_WRC_METHOD=0`

## 회귀

- **grand_pass:** `10927 / 11200` (baseline 유지)

## 측정 결과 (stderr 원문)

출처: `amp_diag_stderr.log` (WRC diag·`Decode64_IR` 진입 로그 포함).

```
[Decode64_IR entry] nsym=172 nc=64 bps=4
[Decode64_IR entry] nsym=172 nc=64 bps=4
[Decode64_IR entry] nsym=172 nc=64 bps=4

=== WRC DIAG ===
  method         = 0
  call_count_16  = 0
  call_count_64  = 3775262
  total_delta    = 0
================

=== Chip Amplitude Distribution ===
  (Feed_Chip 진입, DC/AGC 이전 raw rx_I/rx_Q)
  total_samples    = 134046746
  saturated        = 293267  (0.2188%)
  near saturation  = 343792  (0.2565%)
  max |I|          = 32768
  max |Q|          = 32768
  RMS |I|          = 2128.64
  RMS |Q|          = 2129.82
  PAR (max/rms) I  = 15.39
  PAR (max/rms) Q  = 15.39

--- Histogram (|I|, |Q|) ---
  Range |x|              Count I         Count Q
         0        6226031( 4.64%)    6226028( 4.64%)
      1- 1023   122253118(91.20%)  122232121(91.19%)
   1024- 4095     3459948( 2.58%)    3478782( 2.60%)
   4096- 8191     1001840( 0.75%)    1003021( 0.75%)
   8192-16383      615823( 0.46%)     615944( 0.46%)
  16384-24575      207049( 0.15%)     207134( 0.15%)
  24576-32766      114584( 0.09%)     115592( 0.09%)
  32767 (SAT)      168353( 0.13%)     168124( 0.13%)
===================================
```

## 핵심 지표

| 항목 | 값 |
|------|-----|
| total_samples | 134046746 (`Feed_Chip` 호출 1회 = I·Q 1쌍) |
| saturated (\|x\| ≥ 32767) | **293267 (0.2188%)** |
| near saturation (\|x\| ≥ 30000) | **343792 (0.2565%)** |
| max \|I\|, max \|Q\| | **32768** (`int16_t` 최소값 −32768의 절댓값) |
| RMS \|I\|, RMS \|Q\| | **2128.64**, **2129.82** |
| PAR (max/RMS) | **~15.4** (I/Q 동일) |

## 히스토그램 요약

- **0 근처:** ~4.64% (양 채널)
- **1–1023 (저·중저):** ~91.2% — 대부분 소·중간 진폭
- **1024–8191:** ~2.6%
- **8192–32766:** 합 ~0.55% 내외 (빈별 합)
- **SAT 빈 (|x| ≥ 32767):** **~0.13%** 채널별 (I 168353 / Q 168124)

`saturated_count`는 **한 샘플(I,Q) 쌍** 기준으로 I 또는 Q 중 하나라도 ≥32767이면 1회 증가(0.2188%)이고, 히스토그램 SAT 빈은 **I·Q 각각** 별도 집계이므로 비율이 다르게 보일 수 있음.

## 판정 (지시 §7 기준)

- **포화 비율 0.2188% > 0.1%** → 지시문 기준 **「int16 포화 구간이 유의미하게 존재」** 쪽에 해당. **가설(AGC/풀스케일 근접 → 클리핑 → SNR 손실)** 과 **정성적으로 합치**.
- RMS는 **~2130** 수준으로 **풀스케일(32767) 근처 평균 진폭은 아님** — “전 샘플이 포화”는 아니고, **꼬리에서 포화**가 발생하는 형태.
- **PAR ≈ 15.4** — 피크가 RMS보다 크게 튀는 분포(버스트/간섭 시 tail).

## 자진 고지

- **계측 위치:** `HTS_V400_Dispatcher::Feed_Chip` **함수 최상단**, 인자 `rx_I`/`rx_Q` — **DC 제거·`pre_agc_.Apply`·홀로 LPI 역보정 이전** (하네스가 넣는 **RF 입력에 가까운 raw 칩**).
- **AGC 이후/Despread 입력:** 이번 빌드에서는 **미측정**; 필요 시 동일 `record_chip`을 DC 후 또는 `orig_I_`/`orig_Q_` 기록 직전에 **별도 통계 객체**로 추가하는 편이 안전함(현 구조는 단일 전역 `AmpStats`).
- **시나리오 분해:** 미실시 — 전체 T6 한 번에 누적.
- **max \|I\|=32768:** `int16` 표현 한계; 실코드값은 **−32768 또는 32767**.

## 추가된 파일

- `HTS_LIM/HTS_Amp_Diag.hpp`, `HTS_LIM/HTS_Amp_Diag.cpp`
- `HTS_V400_Dispatcher.cpp` — `#if defined(HTS_AMP_DIAG)` 한 곳
- `HTS_T6_SIM_Test.cpp` — reset/print 및 조건부 `#include` of `HTS_Amp_Diag.cpp`
- `HTS_TEST/t6_sim/build_and_test_amp_diag.bat`

`HTS_AMP_DIAG` **미정의** 시 기존 동작·바이너리에 계측 코드 없음.

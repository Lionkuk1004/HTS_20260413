# HARQ Retry 카운터 측정 결과

## 빌드

- **결과:** PASS (`build_and_test_harq_diag.bat`, 경고 C4996 `strncpy` 4건 — 기존 T6 SIM과 동일)
- **플래그:** `HTS_ALLOW_HOST_BUILD`, `HTS_FEC_SIMULATE_M4_RAM_LAYOUT`, `HTS_FEC_POLAR_DISABLE`, `HTS_HARQ_DIAG`, `HTS_WRC_METHOD=0`

## 실행 결과

- **grand_pass:** `10927 / 11200` (baseline 유지)

- **통계 (stderr 원문):**

```
=== WRC DIAG ===
  method         = 0
  call_count_16  = 0
  call_count_64  = 3775262
  total_delta    = 0
================

=== HARQ RETRY STATS ===
  max_harq_limit         = 800
  total_trials           = 10975
  total_retries          = 10975
  avg retries per trial  = 1.00
  max observed           = 1
  trials hit limit       = 0  (0.0%)
========================

--- 분포 ---
  0 retry        : 0
  1-5 retry      : 10975
  6-10 retry     : 0
  11-20 retry    : 0
  21-50 retry    : 0
  51-100 retry   : 0
  100+ retry     : 0
```

## 해석

### 1) max_harq_limit 실제 값

- 런타임·전처리 기준 **`FEC_HARQ::DATA_K` = 800** (`max_harq_limit = 800`).
- `HTS_V400_Dispatcher.cpp` 내 구 주석「모드에서 32」는 **`DATA_K`와 불일치**였음 → 주석을 **800**에 맞게 정리함 (`max_harq_ = FEC_HARQ::DATA_K`).

### 2) 제한 도달 비율

- **`trials_hit_limit / total_trials = 0 / 10975 = 0.0%`**
- **실패 273 trial(11200−10927)이 전부 `max_harq_` 소진 때문이라는 가설은 본 계측으로는 지지되지 않음** — 최소한 **finish 분기에 도달한 PDU**에 한해서는 **800라운드 소진 실패가 0건**.

### 3) 분포 특성

- **관측된 `harq_round_`(PDU 종료 시점)은 전부 1** → `1-5` bin에만 10975건, 그 외 0.
- 의미(구조): `try_decode_`는 **매 PDU마다 `harq_round_++` 후 1회 디코드**하고, `finish = dec_ok | harq_ex`일 때만 `note_packet_finished`를 호출함. 이번 T6 실행에서는 **그 시점의 `harq_round_`가 항상 1** — 즉 **첫 번째 try_decode 완료에서 PDU가 종료**(성공 또는 `finish`가 성립하는 다른 경로)되었고, **다중 HARQ 라운드(2…800) 누적이 통계에 잡히지 않음**.

### 4) total_trials vs grand_total (11200)

- **`total_trials` = 10975 < 11200`** → 일부 trial은 **`try_decode_`의 `finish != 0` 분기에 도달하지 않아** 계측되지 않았을 수 있음(예: 동기/페이로드 미완·하네스가 다음 trial로 넘어가며 `full_reset_`만 반복하는 경우 등). **계측은 “PDU가 HARQ 완료 조건으로 종료된 경우”에만 증가**한다는 한계를 둠.

## 판정

- **`max_harq_`(800) 한도에 도달한 실패는 0%** → **지금 잡힌 종료 PDU들에 한해** “제한 때문에 깨진다”는 설명은 **성립하기 어렵다**.
- **다중 retry(>1)가 관측되지 않음** → T6 하네스/프레이밍이 **IR 연속 재전송을 800회까지 밀어 넣지 않거나**, 실패 trial이 **첫 디코드 이후 finish 없이 끊기는** 패턴일 가능성 — **Stage B(재전송 주입·연속 모드) 별도 실험**이 있으면 retry 분포가 달라질 수 있음.

## 자진 고지

- **계측 위치:** `HTS_V400_Dispatcher::try_decode_()` — `PayloadMode::VIDEO_1` 종료 직전(`full_reset_` 전), `VIDEO_16`/`VOICE`·`DATA` 각각에서 `finish != 0`일 때 **`HARQ_Diag::note_packet_finished(harq_round_, max_harq_, dec_ok)`** (`VIDEO_1`은 시도 수 **1**으로 고정).
- **정확도:** `harq_round_`는 **“이번 PDU에 대해 `try_decode_`가 성공적으로 디코드를 시도한 횟수”**(코멘트상 HARQ 라운드 인덱스)로 해석하는 것이 맞고, **finish 미도달 PDU는 미집계**.
- **의심:** T6에서 **273 FAIL이 “800회 HARQ 소진”이 아닌 다른 메커니즘**(첫 프레임 비트에러·타임아웃·상위 trial 종료 등)일 수 있음 — 본 로그는 **그 가설을 약화**시킴.

## 변경 파일 (계측용)

- 신규: `HTS_LIM/HTS_HARQ_Diag.hpp`, `HTS_LIM/HTS_HARQ_Diag.cpp`
- `HTS_V400_Dispatcher.cpp`: `#if defined(HTS_HARQ_DIAG)` 호출 + 주석 정리
- `HTS_T6_SIM_Test.cpp`: `HTS_HARQ_DIAG` 시 reset/print 및 `HTS_HARQ_Diag.cpp` TU include
- `build_and_test_harq_diag.bat`

`HTS_HARQ_DIAG` **미정의** 빌드에서는 기존과 동일(계측·추가 TU 없음).

# WRC in Decode64_IR 재측정 결과

## 빌드

- Method 0: **PASS** (경고 C4996 `strncpy` 4건 — 기존 T6 SIM과 동일 계열)
- Method 1: **PASS**
- Method 2: **PASS**
- Method 3: **PASS**

실행: `build_and_test_wrc_ir.bat` (VS 2026 vcvars64, `HTS_TEST\t6_sim`)

## grand_pass (stdout `정량 합계` 원문)

- Method 0: `10927 / 11200`
- Method 1: `10911 / 11200`
- Method 2: `10927 / 11200`
- Method 3: `10867 / 11200`

## WRC 실행 통계 (진단 카운터, stderr 전문)

### Method 0 — `wrc_ir_diag_0.log`

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
```

### Method 1 — `wrc_ir_diag_1.log`

```
[Decode64_IR entry] nsym=172 nc=64 bps=4
[Decode64_IR entry] nsym=172 nc=64 bps=4
[Decode64_IR entry] nsym=172 nc=64 bps=4

=== WRC DIAG ===
  method         = 1
  call_count_16  = 0
  call_count_64  = 3775262
  total_delta    = 4027925520
================
```

### Method 2 — `wrc_ir_diag_2.log`

```
[Decode64_IR entry] nsym=172 nc=64 bps=4
[Decode64_IR entry] nsym=172 nc=64 bps=4
[Decode64_IR entry] nsym=172 nc=64 bps=4

=== WRC DIAG ===
  method         = 2
  call_count_16  = 0
  call_count_64  = 3775262
  total_delta    = 4004499519
================
```

### Method 3 — `wrc_ir_diag_3.log`

```
[Decode64_IR entry] nsym=172 nc=64 bps=4
[Decode64_IR entry] nsym=172 nc=64 bps=4
[Decode64_IR entry] nsym=172 nc=64 bps=4

=== WRC DIAG ===
  method         = 3
  call_count_16  = 0
  call_count_64  = 3775262
  total_delta    = 16658704142
================
```

## 3단계 조사 (수치로 확정)

| 단계 | 내용 | 결과 |
|------|------|------|
| **조사 1** | `Decode64_IR`이 실제 호출되는가? | **통과.** `[Decode64_IR entry] nsym=172 nc=64 bps=4`가 stderr에 3회(상한 캡) 출력됨. |
| **조사 2** | `WRC::clean_chips` N=64 호출 수 | **통과.** `call_count_64 = 3775262` (0 초과). 심볼당 I·Q 각 1회 × IR 재시도 누적으로 해석 가능. |
| **조사 3** | 칩 변화량 `total_delta_sum` | **Method 1~3 통과** (`total_delta` > 0). **Method 0**은 설계대로 변화 없음(`0`). |

## 해석

- **A.** `call_count_64 > 0` → WRC 삽입 위치(`Decode64_IR` 직후 복사 루프)는 T6 DATA/IR 경로에서 **정상 동작**함.
- **B.** Method 1~3에서 `total_delta > 0` → 비-OFF에서 **칱 배열이 실제로 수정**됨(WRC 로직 실행).
- **C.** `grand_pass`는 Method 0·2가 **10927/11200**으로 이전 `Decode_Core` 삽입 시와 동일한 합계선이 유지되고, Method 1·3에서는 **소폭 감소**(10911, 10867). 즉 “실행은 되나 효과가 항상 이득은 아님”이 수치로 드러남. Method 2는 이번 하네스에서 baseline과 동일 pass.

## 자진 고지

- **변경 위치**
  - `HTS_Walsh_Row_Converter.hpp`: `DiagCounters`, `get_diag`, `reset_diag`, `print_diag` 선언.
  - `HTS_Walsh_Row_Converter.cpp`: 진단 전역·`clean_chips` 내 호출/델타 집계(Method 0은 호출 수만).
  - `HTS_FEC_HARQ.cpp`: `Decode_Core` 내 WRC 블록 **삭제**; `Decode64_IR`에서 `llr_slots` 검증 직후 **호스트 진입 로그**, **`use_I`/`use_Q`**(복사 + 심볼별 `clean_chips`) 후 conv 루프 및 Polar 분기에서 `sym_I`/`sym_Q` 대신 **`use_I`/`use_Q`** 사용. 비호스트·WRC OFF 빌드에서는 복사/WRC 생략(`use_I` = 원 포인터).
  - `HTS_T6_SIM_Test.cpp`: `reset_diag()` / `print_diag()` 및 `HTS_Walsh_Row_Converter.hpp` include.
  - `build_and_test_wrc_ir.bat`: Method 0~3 빌드·실행·로그 수집.
- **원본과 다른 점:** WRC 적용 지점이 **`Decode_Core` → `Decode64_IR`**; T6 기본 경로에 맞춤. 진단용으로 **`HTS_ALLOW_HOST_BUILD`에서만** `Decode64_IR` stderr 3회 진입 출력.
- **의심/한계:** `call_count_64`는 “심볼×I/Q×디코드 시도” 누적이라 절대값만으로는 라운드 수를 단정하기 어려움. `total_delta`는 L1 합으로 스케일이 큼(방식 C가 특히 큼).

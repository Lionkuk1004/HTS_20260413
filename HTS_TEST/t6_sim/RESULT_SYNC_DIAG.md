# Preamble 동기 정밀 계측 결과 (`HTS_SYNC_DIAG`)

- **일자:** 2026-04-19  
- **빌드:** `build_and_test_sync_diag.bat` → `HTS_T6_SYNC_DIAG.exe`  
- **로그:** `sync_diag_stdout.log`, `sync_diag_stderr.log` (동일 실행)

---

## 빌드

- **결과:** PASS (`cl` exit code 0)

---

## 회귀

- **정량 합계 (stdout):** `10927 / 11200` (97.6%) — `sync_diag_stdout.log` 내 `정량 합계` 행과 일치.
- **baseline:** 사용자 기준 **10927 유지** — 본 빌드에서 동일.

---

## 계측 의미 (주의)

- `record_scan`은 **`phase0_scan_()`가 호출될 때마다** 1회 집계됩니다. `pass == false`일 때 `memcpy`로 버퍼를 밀고 `p0_chip_count_ = 64`로 두어 **동일 패킷 시도 중 재스캔**이 이어질 수 있으므로, **`total_scans` ≫ 성공 디코드 trial 수`**인 것은 정상입니다.
- `passed_scans`는 **`pass == true`인 phase0 스캔** 횟수입니다(성공적으로 임계 통과한 스냅샷).
- `best_e63`은 `phase0_scan_` 내부에서 누적·비교되는 **`accum` 스케일**(FWHT 에너지 등에 시프트가 포함됨)입니다. 사용자 메모의 “칩 진폭 2128 → 64합 → 제곱” 등 **이론 에너지와 숫자 단위가 1:1로 대응한다고 가정하면 안 됩니다.**

---

## 시나리오별 동기 품질

아래 수치는 **`sync_diag_stderr.log` 원문**에서 발췌했습니다.

### S1 (Clean)

- `total_scans = 60`, `passed = 20 (33.33%)`, `failed = 40 (66.67%)`
- **best_off:** min=0, max=0, avg=0.000 — **분포:** `off=0` 만 20회 (100%)
- **best_e63 (pass만):** min=82668, max=82668, avg=8.267e+04
- **best_e63 분포:** `10K ~ 100K` : 20
- **second/best:** `0.25 ~ 0.50 (fair)` : 20

### S2 (위상 스윕)

- `total_scans = 480`, `passed = 160 (33.33%)`, `failed = 320 (66.67%)`
- **best_off:** min=0, max=0, avg=0.000 — **off=0** 만 160 (100%)
- **best_e63:** min=82602, max=82668, avg=8.264e+04
- **second/best:** `0.25 ~ 0.50 (fair)` : 160

### S3 (AWGN 스윕)

- `total_scans = 547`, `passed = 197 (36.01%)`, `failed = 350 (63.99%)`
- **best_off:** min=0, max=63, avg=16.426 — **0에 65%**, 나머지 다수 bin에 흩어짐
- **best_e63:** min=38967, max=27710753, avg=2.107e+06
- **second/best:** good 7, fair 80, poor 41, bad 69

### S4 (타이밍 `pre_guard + off`)

- `total_scans = 620`, `passed = 140 (22.58%)`, `failed = 480 (77.42%)`
- **best_off:** min=0, max=63, avg=25.714 — off=0,1,5,17,31 각 20회, **off=63 에 40회(28.57%)**
- **best_e63:** min=82668, max=248004, avg=2.067e+05
- **second/best:** good 20, fair 20, poor 100

### S5 (CFO)

- `total_scans = 504`, `passed = 164 (32.54%)`, `failed = 340 (67.46%)`
- **best_off:** min=0, max=0, avg=0.000 — **off=0** 만 164 (100%)
- **best_e63:** min=38062, max=108708, avg=7.626e+04

### S6 (다중경로)

- `total_scans = 240`, `passed = 80 (33.33%)`, `failed = 160 (66.67%)`
- **best_off:** min=0, max=0, avg=0.000 — **off=0** 만 80 (100%)
- **best_e63:** min=72256, max=93821, avg=8.270e+04

### S7 (Barrage 재밍)

- `total_scans = 470`, `passed = 158 (33.62%)`, `failed = 312 (66.38%)`
- **best_off:** min=0, max=63, avg=17.728 — **0에 63%**, 고 off 쪽 비중 증가
- **best_e63:** min=49767, max=25920282, avg=2.740e+06
- **second/best:** good 4, fair 54, poor 49, bad 51

### S8 (CW 재밍)

- `total_scans = 280`, `passed = 100 (35.71%)`, `failed = 180 (64.29%)`
- **best_off:** min=0, max=63, avg=12.600 — **off=0: 80%**, **off=63: 20%**
- **best_e63:** min=48862, max=192663, avg=9.922e+04

### S9 (복합 스트레스)

- `total_scans = 87`, `passed = 32 (36.78%)`, `failed = 55 (63.22%)`
- **best_off:** min=0, max=62, avg=23.969 — 다점 분포
- **best_e63:** min=81001, max=861920, avg=4.187e+05
- **second/best:** fair 3, poor 10, bad 19

### S10 (내구 10000 + LPI)

- `total_scans = 30600`, `passed = 10200 (33.33%)`, `failed = 20400 (66.67%)`
- **best_off:** min=0, max=0, avg=0.000 — **off=0** 만 10200 (100%)
- **best_e63:** min=74726, max=82948, avg=8.265e+04

---

## 핵심 해석 (가설 분기 — 로그 근거만)

### A. S1/S2/S5/S6/S10에서 `best_off` 가 항상 0

- 클린·위상 회전·CFO·MP·대량 내구 구간에서 **성공한 phase0 스캔의 `best_off`는 전부 0**으로 관측됨.
- **“칩 정수 슬라이드 동기만으로 클린에서 위치가 흔들린다”**는 가설은 **본 로그의 pass된 스캔 기준으로는 뒷받침되지 않음**.

### B. S3/S7/S9에서 `best_off` 다점 분포

- 저 SNR·바라지·복합 스트레스에서 **pass된 스캔의 `best_off`가 여러 값**으로 퍼짐.
- **재밍·강한 왜곡 하에서 동기 피크가 alias/잡음에 끌린다**는 정성 결론에 부합.

### C. S4는 시나리오가 의도적으로 `pre_guard` 칩 오프셋을 바꿈

- `off=63` 비중이 큰 것은 **실험 조건(타이밍 오프셋)** 과 일치할 수 있음 → “알고리즘 결함” 단정과 분리해 해석 필요.

### D. `best_e63` vs 사용자 적은 이론 스케일

- 본 계측의 `best_e63`는 **`phase0_scan_`의 `accum` 값**이며, 사용자가 적어 둔 \(136192^2\) 류의 **raw 에너지와는 단위·정의가 다름** — **직접 비율 산출은 “확인 못함”**(별도 `accum` ↔ 물리 에너지 매핑 필요).

---

## stderr 전체 (동일 실행, 파일 복사본)

`sync_diag_stderr.log` 파일을 열면 **S1~S10 블록 전체 + WRC DIAG**까지 동일 내용이 있습니다. (상단 `[Decode64_IR entry]` 줄은 다른 경로의 stderr 출력입니다.)

---

## 자진 고지

- **`failed_scans` 비율**은 “패킷 디코드 실패율”이 아니라 **phase0 `pass` 실패(슬라이드 재시도) 횟수**입니다.
- **S4**는 채널이 아니라 **`feed_raw_ext(..., pre_guard = kGuard + off)`** 로 인한 **의도적 칩 정렬 변화**가 `best_off` 분포에 섞입니다.
- **이론 8 dB / Python `frac_off`** 와의 수치 연결은 본 문서 범위 밖(별도 모델 정합 필요).

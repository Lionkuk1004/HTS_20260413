# Walsh Row Consistency 계측 결과

## Discovery 결과

### DISC-RC1 — Payload 심볼 FWHT (`Decode64_IR`)

- **위치:** `HTS_FEC_HARQ.cpp` — `for (int sym = 0; sym < nsym; ++sym)` 루프.
- **처리:** 칩 `use_I/use_Q` → `fI[c]`, `fQ[c]` (마스크) → `FWHT(fI, nc)`, `FWHT(fQ, nc)` → `fec_ir_fwht_bin_unshift` → `Bin_To_LLR`.
- **심볼 수:** T6 DATA 경로에서 로그 기준 `nsym=172`, `nc=64`, `bps=4` (호스트 진입 로그 3회 상한).

### DISC-RC2 — “허용 row” 가정 (계측용)

- `Bin_To_LLR`는 `valid = min(2^bps, nc)` 개의 **인덱스 0 … valid−1**에서 `corr = fI[m]+fQ[m]`로 LLR을 쌓는다 (`HTS_FEC_HARQ.cpp` 396–399행대).
- 본 계측의 **허용 row**는 사용자 예시와 정합되도록 **직교 세트 격자**로 정의했다:  
  **`nc % 2^bps == 0`** 일 때 **`step = nc / 2^bps`**, **`top1 % step == 0`** 이면 `in_allowed`.  
  - 예: `nc=64`, `bps=4` → `step=4` → row **0,4,8,…,60** (16개).  
  - `2^bps ≥ nc` 이거나 나누어 떨어지지 않으면 **전 row 허용**(보수적으로 `true`).

> **주의:** 에너지 **argmax** row가 이 집합에 속한다는 것은 “정상 수신”과 **동치가 아님**. 무작위로 64 row 중 하나를 고르면 `P(in allowed)=16/64=0.25`**(bps=4, nc=64)**.

---

## 빌드

- **결과:** PASS (`build_and_test_row_consistency.bat`).

## 회귀

- **grand_pass:** **10927 / 11200 (97.6%)** — baseline 유지.

---

## 시나리오별 결과 (요약)

원문 로그: `row_consist_stderr.log`.

### S1 (Clean)

| 항목 | 값 |
|------|-----|
| total_symbols | 6880 |
| allowed_ratio | **0.254651** |
| entropy_H | **3.996 bit** |
| concentration_top4 | 0.274 |
| 상위 row | r2,r4,r8,r14,… (다점 분산) |
| jump | [0]=374, [1]=2944, [2]=2063, [3]=1498 |

### S3 (저 SNR)

- **allowed_ratio 0.614** — 무작위 null ~0.25보다 **높음**.
- **entropy ~4.70 bit** — S1 대비 증가.
- 상위: r0,r1,r2,… 다수.

### S7 (Barrage)

- **allowed_ratio 0.716** — null보다 **현저히 높음**.
- **entropy ~4.67 bit**.
- r0 다수(4089) 등 저번대 집중.

### S8 (CW)

- **allowed_ratio 0.794**
- **entropy_H 0.734 bit** — **붕괴**.
- **concentration_top4 1.000** — **r28=16382**, r30=4258 만 존재 → **재밍 fingerprint**에 해당.

### S9

- **allowed_ratio 1.000**, out_allowed=0 — 해당 시나리오에서 top row가 항상 step 격자 위(표본 7054).

### S10 (내구)

- total_symbols **3,508,800** (10000 trial × 부하).
- allowed_ratio **0.249** ≈ **이론 null 0.25**.

---

## 해석 (가설 대비)

### 무작위 null (bps=4, nc=64)

- **allowed_ratio ≈ 0.25** 이면 “에너지 최대 row가 16개 격자 중 하나일 확률”과 일치 → **S1/S2/S4/S6/S10** 패턴.
- 따라서 **“Clean 이면 1.0”** 기대는 **본 허용 정의 + 에너지 argmax** 조합으로는 성립하지 않음 → 지시서 **B**에 가깝지만, **“구조 오류”가 아니라 지표 정의 한계”**로 해석 가능.

### 재밍·열화에서의 이탈

- **S7/S3:** allowed_ratio가 **0.25를 상회** → 단순 무작위보다 **격자(row %4==0) 쪽으로 에너지 누출/정렬**되는 경향.
- **S8:** **entropy 급락 + top4=100%** → **row fingerprint**로 활용 가치 높음 (가설의 “재밍 구분” **부분 충족**).

### 판정 (다음 단계)

- **A’ (수정):** “allowed_ratio 단독”보다 **entropy + concentration + 상위 row 지문**이 재밍 구분에 유효 — **특히 S8**.
- **B:** “허용 집합 = 비트 LLR에 쓰인 첫 `valid`개 row” 등 **다른 정의**를 쓰면 allowed_ratio 의미가 바뀜 → Discovery 재정의 후 재측정 권장.
- **C:** allowed_ratio만으로는 Clean/재밍 분리 **불충분**(null 근처 유지).

---

## 자진 고지

- **허용 row**는 `nc/(2^bps)` 격자 가정이며, **TX 매핑·역 Walsh shift 후 도메인**과 1:1 대응함을 증명하지는 않음.
- **Polar 분기**에서도 동일 `fec_row_consistency_record_from_fwht` 호출(동일 `nc`,`bps`).
- **last_top_row**는 시나리오 시작 시 `reset_stats()`로 -1 초기화; **Decode 호출 사이**에는 유지되므로 “패킷 경계”가 아닌 **연속 심볼** 기준 jump임.
- 샘플 수는 시나리오·성공 디코드 횟수에 비례.

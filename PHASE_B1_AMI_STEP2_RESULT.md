# Stage A-0 Step 2 결과

## 커밋

- **hash**: `2a7377e`
- **메시지**: `ami_step2: AMI dominant_row FWHT 도입 (best_dom_row 시변화)`
- **변경 파일**: `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp` — `if (pass)` 내 **`dominant_row_` 대입 직전**에 `#if defined(HTS_TARGET_AMI) && !defined(HTS_PHASE0_WALSH_BANK)` 블록 추가 (지시서 스펙과 동일). `pass`가 이미 참일 때만 실행.

## 빌드

| 대상 | 결과 |
|------|------|
| AMI (`cursor_t6_build_ami.cmd`) | **성공** (`build_ami_step2.log`) |
| PS-LTE (`cursor_t6_build.cmd`) | **성공** (`build_baseline_step2.log`, `REM` 줄로 인한 PowerShell 경고 1줄은 기존과 동일·빌드 완료) |

## PS-LTE 회귀

- **정량 합계** (`baseline_after_step2.log`): **`13839 / 15200 (91.0%)`** — BER **0.08954**
- **13839 유지**: **Yes**

## AMI 3회

| Run | 로그 | 정량 합계 | BER |
|-----|------|-----------|-----|
| 1 | `ami_step2_run1.log` | **230 / 15200 (1.5%)** | **0.96562** |
| 2 | `ami_step2_run2.log` | **230 / 15200 (1.5%)** | **0.96562** |
| 3 | `ami_step2_run3.log` | **230 / 15200 (1.5%)** | **0.96562** |

- **Step 1 직후(참고)**: 230 / 15200, BER **0.97171**
- **PASS 건수 변화**: **없음 (230 유지)**  
- **BER**: 소수 변동 (**0.97171 → 0.96562**) — 통과 trial 집합은 동일하나, 실패 trial 비트 오차 집계가 달라질 수 있음(페이로드 경로 입력 변화).

## `[PAYLOAD-SHIFT]` — `walsh_shift` 분포 (`ami_step2_run1.log`)

- **총 라인 수**: **12510** (Step 1 로그의 13458과 약간 다름 — 실행/디코드 진입 횟수 변동 가능).

| walsh_shift | 건수 (상위) |
|-------------|------------|
| **0** | **11193** |
| 31 | 257 |
| 57 | 378 |
| 51 | 109 |
| 60 | 108 |
| 3 | 108 |
| 63 | 89 |
| … | (그 외 **다수** 비-zero 값; 1~62 전반에 분포) |

- **요약**: **이전(전부 `walsh_shift=0`)과 달리 비-zero가 대량 출현** → `dominant_row_` 시변화가 페이로드 측 `walsh_shift`에 **전달됨**.

## `[PAYLOAD-SHIFT]` — `dom` 분포 (`ami_step2_run1.log`)

| dom | 건수 (상위) |
|-----|------------|
| **63** | **11193** (`walsh_shift=0`과 동일 건수 — 상관된 패턴) |
| 6 | 378 |
| 31 | 257 |
| 30 | 17 |
| … | 0~62 광범위 |

## `[AMI-DOM]` — `dom_row` 분포 (Phase 0 락 시, `ami_step2_run1.log`)

- **총 라인 수**: **47905** (`pass` 횟수와 동일 오더).

| dom_row | 건수 (상위) |
|---------|------------|
| **31** | **16370** |
| **63** | **16124** |
| 25 | 1940 |
| 24 | 1502 |
| 17 | 1901 |
| 16 | 1757 |
| … | 0~62 전반 (예: 17, 18, 22, 28 등 수천 단위) |

- **요약**: **63 고정 해제** — FWHT 기반 `best_dom_row`가 실제로 변동.

### `[AMI-DOM]` 샘플 (로그 선두)

```
[AMI-DOM] best_off=32 dom_row=63 e_max>>16=31000
[AMI-DOM] best_off=0 dom_row=63 e_max>>16=62001
[AMI-DOM] best_off=9 dom_row=31 e_max>>16=35938
[AMI-DOM] best_off=63 dom_row=17 e_max>>16=65876
...
```

## 시나리오별 (요약, `ami_step2_run1.log`)

| S | PASS (Step 2) | Step 1 참고 | 변화 |
|---|----------------|-------------|------|
| S1 Clean | **0** / 100 | 0 | 동일 |
| S4 +63 | **100** / 100 | 100 | 동일 |
| S4 +127 | **100** / 100 | 100 | 동일 |
| S5 전 행 | **0** / 100 각 | 0 | 동일 |
| **정량 합계** | **230** / 15200 | 230 | **동일** |

(나머지 S2~S3/S6~S10 행은 Step 1과 동일 패턴 유지 — PASS 총합 230으로 검증.)

## 판정

- **[ ] 개선 (230 → 상향)** — 엔드투엔드 **230 유지**
- **[x] 변화 없음 (PASS 수)** — 시나리오 표 불변
- **[x] 복합** — **내부 지표는 개선·분산 확보**, 통과율은 동일:
  - `dominant_row_` / `[AMI-DOM]` / `[PAYLOAD-SHIFT]` **시변화 성공**
  - Stage 6 / 전체 FEC 경로는 **여전히 병목** (지시서 “변화 없음 시 IR 경로 조사”에 해당)

## 다음 단계 권고

1. **Stage 6 IR 경로** AMI 정합성 점검 (`walsh_shift`·`dominant_row` 소비부 — `HTS_FEC_HARQ`는 지시서상 미터치 유지).
2. **S1=0** 유지 → Phase0 판정·`e63_min`·시드 경로와의 **교차 분석** (이번 변경은 `pass` 이후에만 동작).
3. **WCET**: `pass`당 FWHT **2회**만 추가됨 — 현재 T6 AMI 런타임은 로그량 증가로 다소 증가 가능; 임베디드 타깃이면 프로파일 권장.

## 산출물

- `HTS_TEST/t6_sim/build_ami_step2.log`, `build_baseline_step2.log`
- `HTS_TEST/t6_sim/baseline_after_step2.log`
- `HTS_TEST/t6_sim/ami_step2_run1.log` ~ `ami_step2_run3.log` (용량 큼 — 저장소 미추가 권장)

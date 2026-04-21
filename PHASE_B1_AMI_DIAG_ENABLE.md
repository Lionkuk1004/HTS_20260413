# Stage A-0 Step 1: AMI DIAG 활성화 결과

## 환경

- **branch**: `ami_dry_run` (`git branch --show-current` 확인).
- **Working tree**: 추적 파일 기준 변경은 `HTS_V400_Dispatcher_Sync.cpp` 한 파일만 커밋됨. `HTS_TEST/t6_sim` 아래 기존과 같이 다수 **untracked** 문서·exe가 있음 (이번 작업에서 정리하지 않음).
- **수정 커밋**: `249b288` — `ami_diag: enable HTS_DIAG_PRINTF Phase0/Feed paths under HTS_TARGET_AMI (Sync.cpp)`
- **DIAG 활성화 방식**: `HTS_V400_Dispatcher_Sync.cpp`에서  
  `#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK) && !defined(HTS_TARGET_AMI)`  
  → `!defined(HTS_TARGET_AMI)` **한 줄만 제거** (9곳).  
  추가로 `#if defined(HTS_DIAG_PRINTF) && !defined(HTS_TARGET_AMI)` (V5 프리앰블 진단 1곳) → `#if defined(HTS_DIAG_PRINTF)` 로 동일 원칙 적용.
- **로직·파이프라인**: `#elif !defined(HTS_TARGET_AMI)` 등 **수신 알고리즘 분기는 미변경**. Polar/LPI/CFO/AGC 비활성 매크로 **추가 없음**.

`11b2894..HEAD --stat`:

- `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp` — **10 insertions, 19 deletions** (전처리 줄 정리 포함).

---

## 빌드 결과

| 대상 | 결과 | 비고 |
|------|------|------|
| AMI (`cursor_t6_build_ami.cmd`) | 성공 | `HTS_T6_SIM_Test_ami.exe` 갱신 (로컬 표시 **115712** bytes, 2026-04-22 빌드) |
| PS-LTE (`cursor_t6_build.cmd`) | 성공 | `HTS_T6_SIM_Test.exe` 갱신 (**126464** bytes) |

- **PS-LTE 정량 합계** (`baseline_after_diag.log`): **`13839 / 15200 (91.0%)`** — BER **0.08954**.  
  지시서 기준 **13839 유지**: **Yes** (DIAG 전처리 완화만으로 수치 변화 없음).

---

## AMI 실측 (3회 재현성)

| Run | 로그 파일 | 정량 합계 |
|-----|-------------|-----------|
| 1 | `HTS_TEST/t6_sim/ami_diag_run1.log` | **230 / 15200 (1.5%)**, BER **0.97171** |
| 2 | `ami_diag_run2.log` | 동일 |
| 3 | `ami_diag_run3.log` | 동일 |

- **이전 (STAGE4 printf 비활성)**: 230 / 15200, BER 0.97171  
- **현재**: **동일** → printf 추가만으로 **판정/통계 경로는 변화 없음** (재현성: **동일**).

---

## Phase 0 — `[STAGE4-SCAN]` / `[STAGE4-JUDGE]` (관측 한계 포함)

### 총 건수 (전체 T6 1회 기준)

| 태그 | `ami_diag_run1.log` 내 건수 |
|------|------------------------------|
| `[STAGE4-SCAN#` | **3** |
| `[STAGE4-JUDGE#` | **3** |

**한계 (중요)**: `phase0_scan_()` 초입에 있는 **`diag_this_scan = (s_stage4_scan_idx <= 3)`** 및 관련 정적 카운터 때문에, **프로그램 전체 수명에서 STAGE4-SCAN/JUDGE printf는 최대 3회만** 출력된다. 따라서 지시서에 있던 **“시나리오별 STAGE4 히스토그램”**은 **이번 변경만으로는 불가**이며, Step 2에서 **스캔 진단 한도 상향**(로직 외 정책) 또는 **로그 구간 마킹**이 필요하다.

### 전역 첫 3회 샘플 (로그 선두, `ami_diag_run1.log`)

**SCAN**

1. `#1` — `final_best_off=-1`, `best_e63=0`  
2. `#2` — `final_best_off=32`, `best_e63=31000`, `second_e63=9687`  
3. `#3` — `final_best_off=0`, `best_e63=62001`, `second_e63=62001`

**JUDGE**

1. `#1` — `best_off=-1`, `best_e63=0`, `e63_min=19000`, `r_avg_ok=1`, `sep_ok=1`, **`pass=0`**  
2. `#2` — `best_off=32`, `best_e63=31000`, `e63_min=19000`, `r_avg_ok=1`, `sep_ok=1`, **`pass=1`**  
3. `#3` — `best_off=0`, `best_e63=62001`, `e63_min=39125`, `r_avg_ok=1`, `sep_ok=1`, **`pass=1`**

- **pass=1**: 2 / 3  
- **pass=0**: 1 / 3  
- 위 3회는 **하네스 전체 중 가장 이른 `phase0_scan_` 호출**에 해당하며, **S1 Clean 100 trial 전부**를 대표하지는 않는다.

---

## Phase 0 — `[P0-SCAN]` 기반 `best_off` 분포 (전체 로그, 관측)

`[P0-SCAN]` 라인 수: **173984** (`ami_diag_run1.log`).

| 지표 | 값 |
|------|-----|
| `off=-1` 빈도 | **17688** |
| `off >= 0`만 (156296회) min / max / 평균 | min **0**, max **63**, 평균 **29.98** |
| 전체(−1 포함) min/max/평균 | min **−1**, max **63**, 평균 **≈26.83** |

**10-chip 버킷** (`Floor((off+5)/10)*10`, −1은 0 버킷에 섞일 수 있음):

| Name (버킷) | Count |
|---------------|-------|
| 0 | 38714 |
| 10 | 25845 |
| 20 | 16255 |
| 30 | 33163 |
| 40 | 15322 |
| 50 | 18062 |
| 60 | 26623 |

- **`sep=0`**: 1545 / 173984  
- **`sep=1`**: 172439 / 173984  

→ 대부분 구간에서 **sep_ok 쪽은 1**로 관측되는 비율이 높다. 실패는 **`pass`/`e63_min`/오프셋 −1** 등 다른 조건과 결합될 가능성이 크다.

---

## Payload / HDR (기존 태그)

| 태그 | 건수 (run1) |
|------|-------------|
| `[PAYLOAD-SHIFT]` | **13458** |
| 샘플 | 여전히 **`walsh_shift=0 dom=63`** 패턴 |
| `[HDR-SHIFT]` | (필요 시 동일 로그에서 `Select-String`으로 확장 가능) |
| `CRC.*PASS` / `crc_ok` 등 | **유의미한 전용 태그 거의 없음** (grep 1건 수준 — 기존과 동일 계열) |

---

## S1 Clean — 이번 로그에서 바로 말할 수 있는 것

- **엔드투엔드**: 여전히 **0 / 100** (정량 표는 `ami_diag_run*.log` 하단 `S1` 행과 동일).  
- **STAGE4-JUDGE**: S1 전용으로 잘라낸 출력 **없음** (상단 3줄만 존재).  
- **해석**: S1에서 “Phase0만 실패 vs Payload만 실패”를 **STAGE4만으로 분리할 수 없음** → Step 2에서 **진단 한도 확대** 또는 **시나리오 라벨과 동기화된 printf** 검토.

---

## 원인 후보 정렬 (이번 **관측** 범위 내)

### A. `best_off` 후보가 −1 또는 비정상 구간

- **근거**: `[P0-SCAN] off=-1` **17688**회; STAGE4 첫 판정도 `best_off=-1`, `pass=0`.  
- **강도**: 중~상 — 특히 **동기 미확정** 구간과 일치.

### B. `e63_min` 대비 에너지 여유

- **근거**: `#2`에서 `best_e63=31000`, `e63_min=19000` (여유); `#3`에서 `e63_min=39125`로 상향.  
- **강도**: 중 — **적응형 `e63_min`**이 스캔 간격에 따라 변동.

### C. Phase0 `pass=1` 이후에도 엔드투엔드 실패

- **근거**: 전역 첫 스캔 중 2회는 `pass=1`이나, **대부분의 trial**에 대한 STAGE4는 로그에 없어 **검증 불가**.  
- **강도**: 미정 — Step 2 측정 필요.

### D. 기타

- `[HARNESS]` / `[DC-AGC]` 등 Feed 측 진단은 **이제 AMI에서도 컴파일됨** (상한 300 chip 등). 로그 용량 증가에 유의.

---

## Stage A-0 Step 2 권고

| 항목 | 권고 |
|------|------|
| STAGE4 스캔 로그 한도 | `diag_this_scan` 또는 `s_stage4_scan_idx` 상한을 **S1 전용 N회** 또는 **시나리오 ID별**으로 완화해 **통계 유효 샘플** 확보 (로직·판정식 변경 아님). |
| `best_off` 분포 | 이미 **`[P0-SCAN]`**으로 전역 분포는 확보됨 → Step 2에서는 **S1 구간 라인 번호/마커**만 추가해도 상관 분석 가능. |
| `e63_min` / amp× 계수 | `#3`에서 `e63_min=39125` 등 — **S4 +63/+127 성공 행**과 동일 로그에서 **교차표** 작성. |

---

## 산출물 경로

- 빌드 로그: `HTS_TEST/t6_sim/build_ami_diag.log`, `build_baseline_after_diag.log`  
- 실측 로그: `ami_diag_run1.log` ~ `ami_diag_run3.log`, `baseline_after_diag.log`  
- (용량 큼) 저장소에 **git add 하지 않음**.

---

## 요약

1. **AMI에서도** `HTS_DIAG_PRINTF` 경로의 **STAGE4-SCAN/JUDGE**, **BUF/P0-DUMP 경로(동일 #if 블록)**, **Feed `[HARNESS]`/`[DC-AGC]`**, **V5 2블록 진단**이 컴파일·링크된다.  
2. **PS-LTE 13839/15200 유지**, **AMI 230/15200·BER 동일** — 실행 경로에 영향 없음을 실측으로 확인.  
3. **STAGE4 라인은 코드상 전역 3회 제한**이라, 시나리오별 Phase0 표본은 **`[P0-SCAN]`** 위주로 분석하는 것이 현실적이다.

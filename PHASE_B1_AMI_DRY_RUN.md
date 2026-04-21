# AMI 경로 Dry-Run 실측 결과

**작업일**: 2026-04-22  
**원칙**: `ami_dry_run` 브랜치만 수정; `#error` 원문은 주석 보존; `HTS_T6_SIM_Test.exe`(PS-LTE)는 AMI 전용 출력명으로 비덮어쓰기 방지(실측 후 PS-LTE 재빌드로 갱신됨).

## 환경

| 항목 | 값 |
|------|-----|
| branch | `ami_dry_run` |
| HEAD (보고서 커밋 포함) | `44aab13` (`PHASE_B1_AMI_DRY_RUN.md` 실측 보고) |
| 기반 커밋 | `a556e57` (`CFO복구`) — 브랜치는 `step_c1_s3_v5_lpi`에서 분기 후 위 커밋이 부모 |
| `#error` 해제 커밋 | `10ebfc9` (`ami_dry_run: #error 임시 해제 (실측 위한 조건부 차단 변경)`) |
| 빌드 스크립트 커밋 | `2fee54f` |
| 사전 조치 | 작업 시작 시 **추적 파일** `HTS_TEST/t6_sim/HTS_T6_SIM_Test.exe` 가 modified 로 표시되어 `git restore` 로 인덱스 상태로 복원 후 브랜치 생성 |

## 변경 요약 (코드)

**파일** `HTS_LIM/HTS_V400_Dispatcher_Internal.hpp`  
- 기존 `#if defined(HTS_TARGET_AMI)` + 무조건 `#error` 제거.  
- 원 `#error` 문자열은 **주석으로 보존**.  
- `#if defined(HTS_TARGET_AMI) && defined(HTS_AMI_FORCE_DISABLE)` 일 때만 새 `#error` (강제 차단용).

**파일** `HTS_TEST/t6_sim/cursor_t6_build_ami.cmd` (신규)  
- `/DHTS_TARGET_AMI` 추가, 출력 `/FeHTS_T6_SIM_Test_ami.exe`.

---

## 빌드 결과

- [x] **성공**
  - `HTS_T6_SIM_Test_ami.exe` 크기: **111104** bytes  
  - 빌드 소요(1회): **약 3.8 s** (`build_ami.log` / 콘솔 `elapsed_ms`)  
  - 경고: 본 로그에서 **fatal / error C** 없음  

- [ ] 실패 — 해당 없음  

**산출 로그**: `HTS_TEST/t6_sim/build_ami.log`

---

## T6 실측 결과 (AMI)

### 정량 합계

| Run | Pass / Total | 비율 | 전체 BER (로그 표기) |
|-----|----------------|------|----------------------|
| 1 | **230 / 15200** | **1.5%** | 0.97171 |
| 2 | **230 / 15200** | **1.5%** | 0.97171 |
| 3 | **230 / 15200** | **1.5%** | 0.97171 |
| **평균** | **230 / 15200** | **1.5%** | — |
| **편차** | **0** trial (3회 동일) | | |

**로그 파일**: `HTS_TEST/t6_sim/ami_run1.log`, `ami_run2.log`, `ami_run3.log`  
**단일 런 소요**: 약 **250 s** (run1 로그 `총 소요: 250.1s`).

### 기존 "붕괴" 기록과 비교

- 주석/`#error` 기록: **43 / 11200** (약 **0.38%**, trial 총합 11200 가정)  
- 본 실측 (현 T6 하네스): **230 / 15200** (약 **1.51%**)  

**판정** (비율만 보면):

- [ ] 주석 기록과 **동일 수치** 재현 — **아님** (trial 수·합계 다름)  
- [x] **동일 패턴**(극소수 pass, 대다수 FAIL) — **유사**  
- [ ] 주석 대비 **개선되어 정상** — **아님**  

**추정:** 과거 **11200** trial 집계와 현재 **15200** 집계는 **하네스 버전 차이** 가능 — 직접 수치 비교는 부적절, **비율·형태**만 참고.

### baseline (PS-LTE) 과 비교

| 구분 | Pass / Total | 비율 | BER |
|------|----------------|------|-----|
| PS-LTE (`HTS_T6_SIM_Test.exe`, `ami_dry_run` 위에서 재빌드 후 1회) | **13839 / 15200** | **91.0%** | 0.08954 |
| AMI dry-run (평균) | **230 / 15200** | **1.5%** | 0.97171 |

- **격차**: 약 **89.5 pp** (pass 비율)  
- **해석 (관측)**: 동일 소스 트리·동일 T6 스위트에서 **AMI 매크로만 켠 빌드**는 **PS-LTE 대비 극단적 저하**.  
- **로그**: `HTS_TEST/t6_sim/baseline_on_ami_branch.log`

---

## 시나리오별 분포 (ami_run1.log 표 기준)

종합 표에 **S0 행 없음** (S1부터). `범주 PASS: 2 / 53`.

| Scenario | Pass / Total (예시 행) | 비고 |
|----------|-------------------------|------|
| S1 | 0 / 100 | FAIL |
| S2 (8각) | 각 0 / 100 | 전부 FAIL |
| S3 | -30~-15dB: 0; -10dB: 4; -5dB: 8; 0dB: 3; 5dB: 1; 10dB: 0 | PART 일부 |
| S4 | +0~+31: 0; **+63: 100**; **+127: 100** | **PASS 2행** |
| S5 (CFO) | **0Hz~5000Hz 전부 0/100** | **2000Hz 포함 전 FAIL** (`ami_run1.log` `1254516`) |
| S6 | 전부 0 / 100 | FAIL |
| S7 | 0dB:4, 5dB:7, 10dB:2; 15~30dB: 0 | 일부 PART |
| S8~S9 | 0 | FAIL |
| S10a/b/c | Endurance 0; LPI-OFF 0; LPI-ON 1 | 대부분 FAIL/PART |

### 특이 관측

- **가장 큰 기여**: **S4 +63, +127** 각 100 pass (합 200) + S3/S7/S10c 등 소량 → **총 230**과 일치.  
- **완전 붕괴에 가까움**: S1, S2, S5 전역, S6 등.  
- **CFO 의존성**: S5 **전 주파수 0 pass** — **H2(CFO/주파수)** 와 **일치하는 관측**이나, S2/S6 등 CFO 외 시나리오도 전면 FAIL 이므로 **단일 원인으로는 부족**.

---

## 가설 검증 (실측 기반)

### H1: `dominant_row_` AMI 미갱신 (`best_dom_row` 미갱신)

- 실측: **증거 부족** — 본 로그만으로 `dominant_row_` 값 분포 미추출.  
- **참고(코드 관측)**: `PHASE_B1_AMI_COLLAPSE_DIAGNOSIS.md` 대로 AMI는 `best_dom_row` 갱신 분기 밖에 있음.

### H2: `block_chips=64` 의미 mismatch

- 실측: **부분 지지** — S5 **전 구간 FAIL**은 CFO 경로 민감성과 **합치나**, S2 등 **CFO 무관** FAIL 다수로 **반증만으로는 불충분**.

### H5: 실제로는 동작 가능(과도 차단)

- 실측: **반증** — **1.5%** pass, **BER ~0.97** → 지시서 **시나리오 γ~δ**(저 pass 붕괴)에 해당.

---

## 종합 판정

- [ ] AMI **80%+** 실동작 확인 → **해당 없음**  
- [ ] AMI **50~80%** 부분 동작 → **해당 없음**  
- [x] AMI **심각 저하 (~1.5%)** → **주석 기록과 같은 계열의 “실질 붕괴”** 로 판단 가능  
- [ ] 예상 외: **3회 완전 동일(230)** — 결정론적 시뮬/고정 시드 특성일 수 있음 (**추정**)

---

## 다음 단계 권고 (구현 아님)

- **방향 A**: `#error` **완전 해제가 아닌** 현재와 같이 **`HTS_AMI_FORCE_DISABLE` 옵션 차단** 유지 + AMI 전용 회귀만 별도 CI.  
- **방향 B**: `best_dom_row` / Phase1 / CFO **AMI 전용 분기 설계** 지시서 (코드 변경은 별도).  
- **방향 C**: 과거 **11200** trial 하네스와의 **집계 정의 diff** 문서화 후, 43 vs 230 **정규화 비교**.  

**추천**: **방향 B + C** — 실측으로 **“과도한 보수적 차단(H5)”은 반증**되었고, **구조적 미스매치(H1/H2) 가설**을 계측·설계 단계에서 계속 파고들 것.

---

## Git 상태

- 브랜치 **`ami_dry_run`** 유지; 커밋: `10ebfc9`, `2fee54f`, `44aab13`.  
- `master` 와는 히스토리가 크게 갈라져 있음 (`git diff main ami_dry_run --stat` 매우 큼 — **브랜치가 `main`이 아닌 다른 라인에서 이미 분기된 상태**). **main 직접 수정 없음.**  

---

## 체크리스트 (지시서 대응)

| 체크 | 결과 |
|------|------|
| baseline exe 덮어쓰기 | AMI는 **`HTS_T6_SIM_Test_ami.exe`** 로 분리 빌드. 이후 PS-LTE 검증을 위해 **`HTS_T6_SIM_Test.exe` 재빌드** 수행 → **의도된 갱신** (`LastWriteTime` 변경). |
| PS-LTE 정량 유지 | **`13839 / 15200 (91.0%)`** 재확인 (`baseline_on_ami_branch.log`). |
| `HTS_FEC_HARQ` 수정 | **없음** |

---

## 산출물 경로

| 파일 |
|------|
| `HTS_TEST/t6_sim/cursor_t6_build_ami.cmd` |
| `HTS_TEST/t6_sim/build_ami.log` |
| `HTS_TEST/t6_sim/build_baseline_on_ami_branch.log` |
| `HTS_TEST/t6_sim/ami_run1.log` / `ami_run2.log` / `ami_run3.log` |
| `HTS_TEST/t6_sim/baseline_on_ami_branch.log` |

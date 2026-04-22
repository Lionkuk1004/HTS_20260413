# S5 CFO 실패 원인 단계 분리 보고 (DIAG bisect)

## 조사 목적

- **배경:** `sin_block>>6` / `atan2` 치환 가설은 실험으로 반증됨. S5 실패는 phase0·Phase1·헤더·CRC 등 **여러 단계** 중 어디에서도 발생 가능.
- **목표:** CFO **3500 Hz**(실패) vs **4000 Hz**(성공), **동일 시드 `t=0`**, Legacy `phase0_scan_` 경로에서 **기존 `printf` 마커만**으로 단계 분리.
- **원칙:** Dispatcher / CFO / HARQ / Sync / Holographic **소스 수정 없음**. 새 `printf` 추가 없음. 본 보고서는 **실측 로그가 없는 문장을 사실로 적지 않음**.

---

## 0. 실행 환경 제한 (Cursor 에이전트)

- `cursor_t6_build_ami.cmd` 실행 시 **`vcvars64.bat` 경로 미인식**으로 본 환경에서 **빌드·실행·`bisect_full.log` 생성 불가**.
- 따라서 **§3~§6의 수치 표는 비워 두었음** (로컬 PC에서 동일 절차 재실행 후 채워야 함).
- **`HTS_T6_SIM_Test.cpp`:** bisect용으로 잠시 수정했다가 **`git checkout -- HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp` 로 원복 완료** (`git diff` 출력 없음 확인).

---

## 1. 기존 DIAG `printf` 마커 (AMI Sync — 정적 전수)

**파일:** `HTS_LIM/HTS_V400_Dispatcher_Sync_AMI.cpp`  
**조건:** `HTS_DIAG_PRINTF` 정의, `HTS_PHASE0_WALSH_BANK` **미정의** (기본 T6 AMI와 동일 계열).

### 1.1 시간 순에 가까운 순서 (한 패킷 `feed_raw_ext` 관점)

| 순서 | 라벨 | 출력 필드(코드 기준) | 비고 |
|------|------|----------------------|------|
| 1 | `[HARNESS]` | `chip#`, `rx_I`, `rx_Q` | `Feed_Chip` 초입, 상한 300회 (`s_lab_feed_chip_idx`) |
| 2 | `[DC-AGC]` | `chip#`, `rx_I`, `after_DC`, `dc_est_I`, `dc_est_Q` | 특정 구간·16칩 간격 |
| 3 | `[STAGE4-P0]` | `off`, `r0`, `r1`, `r2`, `xor_legacy`, `accum`, `bdr` | PS-LTE 분기용; AMI `HTS_TARGET_AMI` 경로에서는 `#if !HTS_TARGET_AMI` 블록 |
| 4 | `[STAGE4B]` | `off`, `full3`, `r0`…`accum` | `diag` + `!WALSH_BANK` 한정 |
| 5 | **`[P0-SCAN]`** | `off`, `e63`, `avg_o`, `r_avg`, `r_sep`, `emin`, `sep` | **매 `phase0_scan_()` 호출마다** 출력 (`HTS_DIAG_PRINTF`) |
| 6 | **`[P0-SEED]`** | `dot`, `est`, `n` | `phase0` **`pass==true`** 이후 블록 |
| 7 | **`[P0-ALIGNED]`** | `off`, `carry` | `psal_commit_align_()` 직전 |
| 8 | `[P1-E]` | `call#`, `e63_sh`, `e0_sh`, `max_e`, `k_P1_MIN`, `pass`, `dom` | `!WALSH_BANK` + 카운터 상한 |
| 9 | **`[P1-NC]`** | `sym`, `e63`, `e0`, `est`, `n`, `carry_pend` | Phase1 매 심볼 |
| 10 | **`[P1-GATE]`** | `n`, `e63_sh`, `e0_sh`, `max_e`, `k_P1_MIN_E`, `gate_*`, `enter_hdr` | 카운터 상한 50000 |
| 11 | **`[P1→HDR]`** | `est` | `PRE_SYM1` 확정 시 |
| 12 | `[SILENT]` | `total`, `last_e` | `max_e_sh < k_P1_MIN_E`, 10회마다 |
| 13 | `[HDR-FAIL]` / `[HDR-SOFT]` / `[HDR-RETRY]` | 문맥별 | 헤더 파싱 루프 |

### 1.2 `pass` 가 `[P0-SCAN]` 에 **없음** (중요)

- `phase0` 최종 판정 `pass` 는 코드 **807–808행** (`best_off>=0 && r_avg_ok && best_e63>=e63_min && sep_ok`).
- **`[P0-SCAN]`** (916–918행) 는 `off,e63,avg_o,r_avg,r_sep,emin,sep` 만 출력. **`sep` 는 `sep_ok` 의 bool**이다.
- **`[STAGE4-JUDGE#…] pass=%d`** 는 **807행 `pass`** 를 출력하나, **`diag_this_scan == true` 인 경우만**이고, `diag_this_scan` 은 **`s_stage4_scan_idx <= 3`** (전역 static, **프로세스 전체에서 `phase0_scan_` 4번째 호출부터 false**).

```78:81:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync_AMI.cpp
#if defined(HTS_DIAG_PRINTF) && !defined(HTS_PHASE0_WALSH_BANK)
    static int s_stage4_scan_idx = 0;
    ++s_stage4_scan_idx;
    const bool diag_this_scan = (s_stage4_scan_idx <= 3);
```

**사실:** 긴 세션에서는 **`[STAGE4-JUDGE]` 의 `pass` 필드를 3500/4000 비교에 믿을 수 없을 수 있음.** 비교에는 `[P0-SCAN]` + (가능하면) 동일 실행의 앞선 3회 `[STAGE4-JUDGE]`, 또는 `HTS_SYNC_DIAG` 빌드의 `SyncDiag` 요약을 병행하는 것이 안전.

### 1.3 `test_S5` 구조 (라인 번호 — 원본 파일 기준)

| 항목 | 위치 |
|------|------|
| 함수 | `static void test_S5()` — **751행** |
| `cfos[]` | **775–780행** |
| 시드 | `mk_seed(0x500000u, t)` — **785행** |
| trial 수 | `kTrials` — **79행**, `constexpr int kTrials = 100` |
| `record_ext` | **807행**, `param` 은 `"%.0fHz", cfo` (**805행**) |

---

## 2. 재현 환경 (지시서 — 로컬에서 적용)

| 항목 | 값 |
|------|-----|
| branch | `opt/holographic_sync` (지시서) |
| 매크로 | `HOLO_FLAG` 비움 → `HTS_USE_HOLOGRAPHIC_SYNC` 없음 |
| 빌드 | `HTS_TEST/t6_sim/cursor_t6_build_ami.cmd` (`HTS_DIAG_PRINTF` 포함) |
| 임시 수정 | `cfos[] = { 3500.0, 4000.0 }`; `main()` 에서 **`test_S5()` 만** 호출; (권장) `t` 루프를 `t<1` 로 두어 **3500·4000 각 1회**만 로그 분리 |

**원복:** `git checkout -- HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp`

---

## 3. CFO 3500 Hz, `t=0` — DIAG 타임라인

**상태:** 본 Cursor 실행에서 로그 **미생성**. 로컬 `bisect_full.log` 에서 **첫 번째 `feed_raw_ext`(3500, t=0)** 구간을 잘라 붙일 것.

---

## 4. CFO 4000 Hz, `t=0` — DIAG 타임라인

**상태:** 동일 **미생성**.

---

## 5. `[P0-SCAN]` 비교 표 (실측 값 — 미기입)

| 항목 | CFO 3500 `t=0` | CFO 4000 `t=0` |
|------|----------------|----------------|
| `[P0-SCAN]` 출력 횟수 (해당 trial 구간) | (로컬 측정) | (로컬 측정) |
| `off` (마지막 줄 기준 등, 정의 후 기입) | | |
| `e63` | | |
| `avg_o` | | |
| `r_avg` | | |
| `r_sep` | | |
| `emin` | | |
| `sep` (= `sep_ok`) | | |
| `[STAGE4-JUDGE]` 의 `pass` (최대 3회만 출력될 수 있음) | | |

---

## 6. 이후 단계 표 (실측 값 — 미기입)

| 단계 | CFO 3500 `t=0` | CFO 4000 `t=0` |
|------|----------------|----------------|
| `[P0-SEED]` 출력 여부 | | |
| `[P0-ALIGNED]` 출력 여부 | | |
| `[P1-NC]` / `[P1-GATE]` / `[P1→HDR]` | | |
| `[SILENT]` | | |
| `[HDR-*]` | | |
| T6 `feed_raw_ext` 결과 (`m.pass`, `m.crc_passed`) | | |

---

## 7. 판정 (시나리오 A–D)

**상태:** 로그 없이 **시나리오 문자 부여 안 함.**

로컬에서 표 §5–§6을 채운 뒤, 지시서 규칙으로만 선택:

- **A:** `[STAGE4-JUDGE]` `pass=0` 또는 `[P0-SCAN]` 만 반복·`[P0-SEED]` 부재로 **phase0 미통과** 추적 가능할 때  
- **B:** `[P0-SEED]`·`[P0-ALIGNED]` 존재 후 **`[P1→HDR]` 없음** 또는 `[SILENT]`·`[HDR-*]` 집중  
- **C:** 헤더 이후 **`m.pass==false`** 이지만 CRC 등 상위 지표는 별도 확인  
- **D:** §5 표의 **수치 차이**가 인접 CFO 패턴과 일관될 때 (숫자만으로 서술)

---

## 8. 원복 확인

| 검사 | 결과 |
|------|------|
| `git diff HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp` | **없음** (원복 확인됨) |
| AMI baseline `14939/18400` | 본 환경 **미재실행** — 로컬에서 `cursor_t6_build_ami.cmd` 후 `Select-String 정량 합계` 로 확인 권장 |

---

## 9. 로컬 재현 체크리스트 (복붙용)

```powershell
Set-Location D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim
# (1) HTS_T6_SIM_Test.cpp: cfos = 3500,4000 / main 에 test_S5 만 / 필요 시 t<1
cmd /c ".\cursor_t6_build_ami.cmd"
.\HTS_T6_SIM_Test_ami.exe 2>&1 | Tee-Object bisect_full.log
Select-String -Path bisect_full.log -Pattern '\[P0-SCAN\]|\[P0-SEED\]|\[P0-ALIGNED\]|\[STAGE4-JUDGE\]|\[P1-|\[SILENT\]|\[HDR'
git checkout -- HTS_T6_SIM_Test.cpp
```

---

## 10. Payload 측 단일 마커

`HTS_V400_Dispatcher_Payload.cpp` 에서 `HTS_DIAG_PRINTF` 기준 검색 시 **`[IR-DIAG]`** 등 소수만 확인됨 (본 bisect의 phase0/P1·HDR와는 별층).

---

*보고 일자: 2026-04-22 · 코드 변경: 없음(임시 bisect는 적용 후 원복). · 실측 로그: 에이전트 환경에서 **수집 실패** — §3–§7 수치는 로컬 보완 필요.*

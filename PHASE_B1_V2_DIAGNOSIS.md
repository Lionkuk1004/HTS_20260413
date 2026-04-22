# V2 Walsh Bank 경로 붕괴 진단

**조사 범위**: 읽기 전용. 코드 수정·추가 빌드·수정 제안 없음.

**전제(관측)**: E1 빌드(`/DHTS_PHASE0_WALSH_BANK`)에서 T6 **100 / 15200**, BER **0.98026**, 3회 동일 재현. Baseline(PS-LTE Phase0) **13839 / 15200**.

아래 **Q1–Q7**은 지시서 통합 양식과 동일 구조로 정리함. 상세 표·라인 인용은 동일 파일 하단에 유지.

---

## Q1. `HTS_PHASE0_WALSH_BANK` 블록 구조

**파일**: `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp`

| 항목 | 라인 (관측) |
|------|-------------|
| `#if defined(HTS_PHASE0_WALSH_BANK)` 시작 | `238` |
| 동일 `#if` 블록 종료 (`#else` 직전) | `274` (`#else` at `275`) |
| 내부 단계 | `242`–`249`: `p0_buf128_*` → `T_I`/`T_Q` 복사. `251`: `fwht_64_complex_inplace_` per `blk`. `252`–`258`: 각 행 `r`에 대해 `e_I=T_I[blk][r]^2`, `e_Q=T_Q[blk][r]^2`, `row_e[r] += e_I+e_Q`. `260`–`271`: `row_max_e` = `row_e[r]`의 최대(행 63은 타이브레이크로 기본 후보). `272`: **`accum = static_cast<int32_t>(row_max_e >> 16)`**. `273`–`274`: `seed_I_fwht` / `seed_Q_fwht` = 두 블록에서 동일 `max_row` 빈의 `T_I`/`T_Q` 합. |
| `best_off` / `best_e63` 갱신 | 공통: `583` `sum_all += accum`. `584`–`587`: `if (accum > best_e63)` → `second_e63`, `best_e63 = accum`, **`best_off = off`**. WALSH_BANK 전용: `588`–`591` `best_dom_row = max_row`, `best_seed_I/Q` 저장. |
| `row_e` 수식 | **이진 제곱합**: 각 FWHT 빈 `r`에 대해 **양 블록 합산** `Σ_blk (T_I²+T_Q²)` (`253`–`257`). |
| `pass`에 쓰이는 값 | `best_e63`는 위 `accum`의 최대값(`584`–`587`). `pass`는 `793`–`794`(아래 Q3). |

**추정(관측과 분리)**: PS-LTE 경로의 `accum`은 동일 `off` 루프 안 `#else` 분기(`304`~)에서 **다른 스케일·다른 조합식**(예: `accum_8x8` `>>14`, `accum_cohr` `>>16`, `max_e0+max_e1b` 등)으로 계산됨. WALSH_BANK의 `accum`과 **동일 숫자 범위를 가정할 근거는 코드상 없음**.

---

## Q2. 매크로 의존성

| 항목 | 관측 |
|------|------|
| WALSH_BANK 블록 **내부**의 `#if` / `#elif` | `238`–`274` 구간 안에는 **추가 `#if` 없음** — 단일 직선 코드. |
| `HTS_TARGET_AMI` | `275`에서 `#else` → `276` `#if defined(HTS_TARGET_AMI)` — **WALSH_BANK 정의 시 AMI 분기는 전처리상 도달 불가**(상호 배타). |
| `Internal.hpp` | `24`–`26`: `#if defined(HTS_PHASE0_WALSH_BANK)` 일 때 **`extern uint8_t k_W63_FWHT_ROW;`** 선언. |
| `Core.cpp` | `37`–`61`: `#if defined(HTS_PHASE0_WALSH_BANK)` 에서 **`uint8_t k_W63_FWHT_ROW = 255u`**, `calc_kw63_fwht_row_()`(FWHT로 `k_w63` 패턴의 **피크 행** 계산). `full_reset_()` `347`–`355`: `k_W63_FWHT_ROW==255`이면 **한 번** `calc_kw63_fwht_row_()`로 채움. |

**동반 매크로(관측)**: `HTS_PHASE0_WALSH_BANK` 켜짐 → **`k_W63_FWHT_ROW` 런타임 심볼** 및 `full_reset_` 내 초기화 경로가 활성. `HTS_TARGET_AMI`와 **동시 정의는 `Internal.hpp` `13`–`15`에서 `#error`**.

**추정(분리)**: “필수 동반 매크로”가 더 있는지 여부는 **본 파일만으로는 추가 매크로 필수 목록 미확인**.

---

## Q3. `pass` 판정

**파일**: `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp`

**4개 조건(구현 위치 `793`–`794`)**

| 조건 | 라인 | 계산(관측) |
|------|------|------------|
| `best_off >= 0` | `793`, `54`, `584`–`587` | `int best_off = -1` (`54`). `accum > best_e63`일 때 `best_off = off` (`587`). |
| `r_avg_ok` | `751`–`754` | `best_x63 = (best_e63<<6) - best_e63` (`751`–`752`), `so_x5 = (sum_others<<2)+sum_others` (`753`), `r_avg_ok = (sum_others<=0) || (best_x63 >= so_x5)`. |
| `best_e63 >= e63_min` | `793`, `768`–`791` | `e63_min = max(adaptive_min, k_E63_ALIGN_MIN)` 등(`768`–`771`). `k_E63_ALIGN_MIN`: PS-LTE 분기에서 **`(amp32<<5)+(amp32<<2)+(amp32<<1)`** (`786`–`788`, `amp*38`). |
| `sep_ok` | `756`–`766` | `r_avg_high = (sum_others>0) && (best_x63 >= (sum_others<<3))` (`756`–`759`). `sec_x5 = (second_e63<<2)+second_e63` (`763`–`764`). `sep_ok = (!r_avg_high) OR (second_e63==0) OR (best_x4 > sec_x5)` (`765`–`766`). `second_e63` 갱신: `585`–`586`, `699`–`700`. |

| 조건 | 라인 (요약) |
|------|------|
| `pass =` | `793`–`794`: `(best_off >= 0 && r_avg_ok && best_e63 >= e63_min && sep_ok)` |
| `e63_min` | `768`–`791`: `adaptive_min` vs `k_E63_ALIGN_MIN` 중 큰 값; `k_E63_ALIGN_MIN`은 PS-LTE에서 **`(amp32<<5)+(amp32<<2)+(amp32<<1)`** (`786`–`788`) (= `amp*38`) |

**스케일 관련 관측**:

- `pass`의 에너지 축은 **`best_e63`** (= WALSH_BANK에서 **`row_max_e >> 16`** 최대, Q1).
- 임계 하한 **`e63_min`**은 **`tx_amp_`** 기반 **PS-LTE 설계 주석**과 동일 식(`786`–`788`).
- WALSH_BANK `accum`과 PS-LTE `accum`의 **단위·정규화가 동일하다는 주석·assert는 없음**.

**추정(분리)**: `best_e63`가 `e63_min`보다 **체계적으로 작으면** `pass`가 거의 항상 false → `1084`–`1105` 실패 분기(버퍼 `memcpy` 슬라이드)만 반복 → 동기 실패 체인 가능.

---

## Q4. `dominant_row_`

| 항목 | 라인 |
|------|------|
| 초기값 | `full_reset_()` `365`: `dominant_row_ = 63u` (`HTS_LIM/HTS_V400_Dispatcher_Core.cpp`) |
| `pass` 성공 시 | `phase0_scan_` `981`: `dominant_row_ = static_cast<uint8_t>(best_dom_row);` |
| WALSH_BANK에서 `best_dom_row` 갱신 | `588`–`591`: `best_dom_row = max_row` (**`accum`이 갱신될 때만**, `584` 조건 충족 시) |

**PS-LTE와 차이(관측)**: PS-LTE는 `597`–`690`에서 `best_dom_pick` 등 **별도** dominant 후보 로직 존재. WALSH_BANK는 **`max_row` 직접**이 `best_dom_row`로 전달.

---

## Q5. 다운스트림 (`pass` 이후 / `Feed_Chip`)

**`pass` true (`909`~) 순서(관측)**:

1. `981` `dominant_row_` 확정  
2. `988`–`1012`: `seed_dot_*` — WALSH_BANK는 **`990`–`992`** `best_seed_I/Q` 복사  
3. `1028`–`1033` `est_*`, `update_derot_shift_from_est_()`  
4. `1038`–`1050`: `walsh63_dot_` 두 번, `cfo_.Estimate_From_Preamble`, `cfo_.Advance_Phase_Only(192)`  
5. `1052`–`1064` `pre_agc_.Set_From_Peak`  
6. `1081`–`1083` `psal_*`, `psal_commit_align_()`

**`pass` false (`1084`~)**: `p0_buf128_*` 앞 64칩을 `+128`에서 `memcpy`, `p0_chip_count_=64`.

**Payload 관점(`pass` false, 관측)**: 위 슬라이드 경로만 실행되고 `909`~의 `dominant_row_`/`seed_dot`/`walsh63_dot_`/CFO/`pre_agc_` 체인은 **`if (pass)`** 블록 밖에서 분기되므로 **동기 확립 분기로 진입하지 않음** → 이후 `Feed_Chip` 등에서 누적 오류 가능(구체 BER 연계는 **추정**; 로그 미첨부).

**`Feed_Chip`**: `1108`~; `phase0_scan_` 호출은 `1226` 근처(이전 조사와 동일 파일 내 `phase0_scan_()` 호출).

**Phase 1 WALSH_BANK** (`1267`–`1300`): `dominant_row_`로 `dot63`, `dominant_row_ XOR k_W63_FWHT_ROW`로 `sym1_row` 후 `dot0` — **`k_W63_FWHT_ROW`는 PS-LTE의 `k_W63_FWHT_ROW_NATURAL`(`Internal.hpp` `22`)과 다른 심볼**.

---

## Q6. 커밋 히스토리

| 명령 | 결과(관측) |
|------|------------|
| `git log --all --oneline --grep=WALSH_BANK` | **출력 없음** (메시지에 문자열 `WALSH_BANK` 없음) |
| `git log --all --oneline --grep=WALSH` | **`8b6bce9`** `Step B: v5 Walsh DIAG...`, **`fdb0846`** `test(t6): Walsh_Row_Permuter...`, **`dd10598`** `디스패쳐분활활`, **`722faf0`**, **`71405e5`** 등 |
| `git log --oneline -S"HTS_PHASE0_WALSH_BANK" -- HTS_LIM/` | **`dd10598`**, **`722faf0`**, **`71405e5`** (동일 저장소에서 실행) |
| `git log -1 --oneline -- HTS_LIM/HTS_V400_Dispatcher_Sync.cpp` | **`ae8f643`** `feat(CFO 4-3): call Advance_Phase_Only(192) after Estimate in phase0_scan_` |

**도입 시점(문자열 추가, 관측)**: **`dd10598`** — 메시지 **`디스패쳐분활활`** — `HTS_V400_Dispatcher_Sync.cpp` 신규 추가(대규모 분할 커밋, `git show dd10598 --stat`에서 Sync.cpp **+2171** 확인).

**`Sync.cpp` 마지막 커밋(파일 기준, 관측)**: **`ae8f643`** — 위 메시지. (WALSH_BANK 전용 변경인지는 **해당 diff 미열어 확인**.)

**추정(분리)**: `dd10598`은 디스패처 **파일 분할** 중심으로 보이며, `HTS_PHASE0_WALSH_BANK`는 그때 **함께 들어온 분기**로 관측됨. 제품 의도·완성도는 메시지만으로 **확정 불가**.

---

## Q7. 100 / 15200 의미

**출처**: `HTS_TEST/t6_sim/e1_run1.log` **종합 표 마지막 부분** (`Get-Content -Tail`).

| 관측 | 내용 |
|------|------|
| `grand_pass` / `grand_total` 정의 | `HTS_T6_SIM_Test.cpp` `1230`–`1244`, `1255`–`1257`: 시나리오별 `r.pass` 합 / `r.total` 합 |
| 시나리오별 | **대부분** `Pass=0`, `total=100`(또는 S10a `10000`). **유일하게** `║ S4      │ +127             │   100 │ ... │ PASS  ║` |
| `100`의 구성 | **단일 시나리오(S4/+127)에서 100 trial 전부 pass** → 합계 pass 수 **100**. `15200`은 전 시나리오 trial 총합. |

**추정(분리)**: S4가 “칩 슬립/오프셋” 류 실험이라면, **+127** 조건에서만 우연히 `phase0` 에너지·`pass` 조건이 맞아떨어졌을 가능성 — **시뮬 정의 추가 확인은 본 로그만으로는 미완료**.

---

## 종합 진단 (지시서 체크리스트)

체크박스는 “**원인 후보로 채택**” 여부. `[x]` = 코드/로그로 **우선 검증할 가설**. `[ ]` = **반증** 또는 **미확인**.

- [ ] V2 경로 미완성 (스텁, `best_off` 미설정 등) — **반증**: `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp` `238`–`274` 완전 구현; `best_off` 갱신 `584`–`587`, 초기값 `-1`은 `54`.
- [ ] V2 경로 다른 매크로 의존 (동반 매크로 필수) — **부분 관측만**: `k_W63_FWHT_ROW` / `full_reset_`(`HTS_LIM/HTS_V400_Dispatcher_Core.cpp` `347`–`355`)와 결합. 블록 내부 중첩 `#if` 없음(Q2). “AMI 등 **필수** 동반”은 **미확인**.
- [x] V2 경로 스케일 불일치 (`best_e63` vs `e63_min` 등) — **추정**(수치 검증은 본 조사 범위 외). **근거(관측)**: WALSH `accum`(`272`) vs PS-LTE `#else` 누적식·시프트 상이; `e63_min`은 `786`–`788` PS-LTE amp 기반.
- [ ] V2 경로 완전 재설계 필요 — **미판정**.
- [ ] 기타: (비움)

---

## 정비 vs 재설계 판단

| 항목 | 내용 |
|------|------|
| V2 정비 예상 작업량 | **중~대 불확실**: `accum`/`pass`/Phase1 `k_W63_FWHT_ROW` 정합 및 T6 S4만 통과 현상과의 연계 **추가 계측·시나리오 정의** 필요 시 범위 증가. |
| V3 신규 설계 예상 작업량 | **미측정**. |
| 권고 방향 | **다음 지시서**: WALSH 경로에서 `best_e63`·`e63_min`·`r_avg_ok`·`sep_ok`의 **런타임 샘플 또는 단위 테스트**로 스케일 가설을 **관측으로 확정**한 뒤, V2 정비 vs 구조 변경을 선택. **본 문서는 수정 코드 제안 없음.** |

---

## 참고 파일 경로

- `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp` — Phase0 `238`–`274`, `pass` `793`–`794`, `pass` 분기 `909`~, Phase1 `1267`–`1300`  
- `HTS_LIM/HTS_V400_Dispatcher_Core.cpp` — `k_W63_FWHT_ROW`, `full_reset_` `347`–`355`  
- `HTS_LIM/HTS_V400_Dispatcher_Internal.hpp` — `k_W63_FWHT_ROW` / `k_W63_FWHT_ROW_NATURAL`  
- `HTS_TEST/t6_sim/e1_run1.log` — 종합 표(마지막 ~120줄)

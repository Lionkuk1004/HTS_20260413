# Phase B-1 Investigation — Walsh-row Bank-parallel CFO 이식 전 조사

조사 시점: `HEAD` = `a556e57` (CFO복구). `HTS_LIM` 트리 및 `HTS_ARM11_Firmware` 일부 검색. 코드 수정 없음.

---

## Part 0 (롤백 결과)

| 항목 | 관측 |
|------|------|
| 롤백 명령 | `git checkout -- HTS_TEST/t6_sim/` 후 `git reset --hard a556e57` 실행됨 |
| 현재 `HEAD` | `a556e57` — 메시지 `CFO복구` (`git log -3` 확인) |
| 작업 트리 | 추적 파일 변경 없음. 미추적: `HTS_TEST/t6_sim/cursor_t6_build.cmd` |
| T6 로그 | `HTS_TEST/t6_sim/rollback_t6_final.log` |
| T6 정량 합계 (로그 라인) | `477961`: `13839 / 15200 (91.0%)` — 전체 BER `0.08954` |

---

## Q1. Phase 0 스캐너 구조

| 항목 | 관측 (파일: `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp`) |
|------|------------------------------------------------------|
| `phase0_scan_` 함수 범위 | 시작 `52`, 종료 `1107` (`1108`행부터 `Feed_Chip` 정의) |
| 메인 offset 루프 | `232`: `for (int off = 0; off < 64; ++off)` — 종료 `702` 직전(`}`) |
| offset 스캔 범위 | `off ∈ [0, 63]` 정수 칩 인덱스. 코드에 `search_range` / `scan_offset` / `off_scan` 식별자 **없음** (동일 파일 `Select-String` 검색 결과 0건). “±N chip” 형태의 별도 변수는 **관측되지 않음**. |
| FWHT / 상관 | `#if defined(HTS_PHASE0_WALSH_BANK)` (`238`~): 블록 2개에 대해 `p0_buf128_*` 복사 → `fwht_64_complex_inplace_` (`251`), `row_e` 합산 (`252`–`258`), `max_row`/`accum` (`260`–`272`). `#else` / `#if defined(HTS_TARGET_AMI)` (`276`~): 32칩×2 `k_w63` 가중 합 비코히런트 에너지. `#else` PS-LTE (`304`~): `T0`/`T1`/`T2` 각 64칩 `fwht_64_complex_inplace_` (`322`, `327`, 조건부 `364`), `energy_8x8_noncoh_row7_` 호출 구간 존재(동 루프 내, `Internal.hpp`의 `energy_8x8_noncoh_row7_` 선언 `133`–`147`). `walsh63_dot_`: `pass` 후 시드/CFO 구간 (`1004`–`1046` 등). |
| `best_off` 결정 | 루프 내 `accum` 계산 후 `583`: `sum_all += accum`. `584`–`587`: `if (accum > best_e63)` 이면 `second_e63`/`best_e63`/`best_off = off` 갱신. `#elif !defined(HTS_TARGET_AMI)` (`597`–`690`): `best_dom_row` 후보(`best_dom_pick`) 갱신. |
| `dominant_row_` 확정 | `pass` 분기 내 `981`: `dominant_row_ = static_cast<uint8_t>(best_dom_row);` |
| `pass` 판정 | `793`–`794`: `pass = (best_off >= 0 && r_avg_ok && best_e63 >= e63_min && sep_ok);` — `r_avg_ok`/`sep_ok`/`e63_min`은 `746`–`791` |
| 실패 시 버퍼 시프트 | `1084`–`1105`: `else`에서 `memcpy`로 `p0_buf128_*` 앞 64칩을 `+128`에서 복사, `p0_chip_count_ = 64` |
| Bank-parallel로 교체 시 **후보가 되는 연속 블록** (라인만 명시) | **(a)** `232`–`702` per-`off` 에너지·행 탐색 본문. **(b)** `746`–`794` 스코어 대비·임계. **(c)** `909`–`1083` `if (pass)` 확정 후 시드·`walsh63_dot_`·`cfo_.Estimate_From_Preamble`·`Advance_Phase_Only`·`pre_agc_.Set_From_Peak`. **(d)** `704`–`743` `#if defined(HTS_WALSH_ROW_DIAG)` 진단 FWHT (매크로 정의 시에만). |

---

## Q2. Walsh-row v20 (Bank-parallel)

| 항목 | 관측 |
|------|------|
| `bank_parallel`, `Bank_Parallel`, `v20_`, `13.*banks`, `500.*Hz.*step` | `D:\HTS_ARM11_Firmware\HTS_LIM` 및 `D:\HTS_ARM11_Firmware\CORE` 대상 `*.py;*.cpp;*.h;*.hpp;*.md` 재귀 검색 **0건** |
| `HTS_PHASE0_WALSH_BANK` | 정의 위치: 매크로 자체는 소스에 “정의 파일” 없음(빌드 플래그로 주입). **사용** 예: `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp` 다수 행(`59`, `210`, `238`, …), `HTS_LIM/HTS_V400_Dispatcher_Internal.hpp` `24`, `HTS_LIM/HTS_V400_Dispatcher.hpp` `337`, `HTS_LIM/HTS_V400_Dispatcher_Core.cpp` `37`, `347`, `HTS_TEST/t6_sim/HTS_V400_Dispatcher_Local.cpp` `250`(주석: 실험 빌드에 `/DHTS_PHASE0_WALSH_BANK`), `449` 등 |
| 이식 가능한 별도 v20 시뮬 레포/스크립트 | **본 워크스페이스 검색 범위에서는 경로 미발견** |

---

## Q3. Walsh-row v22 (PTE fine CFO)

| 항목 | 관측 |
|------|------|
| `PTE.*CFO`, `fine.*CFO`, `v22`, `186.*Hz`, `25.*Hz.*RMS`, `PTE_fine` | `HTS_LIM` 동일 glob 검색 **0건** (다만 `HTS_CFO_Compensator.h`, `HTS_Phase3_Alpha_Harness.cpp` 등에 `CFO` 문자열은 존재하나 v22/PTE 패턴과는 불일치) |
| v20+v22 통합 파일 | **미발견** |

---

## Q4. Walsh Bank 인프라

### `HTS_Walsh_Row_Converter.hpp` (`HTS_LIM/HTS_Walsh_Row_Converter.hpp`)

네임스페이스 `ProtectedEngine::WRC` 내 **파일 수준** API(클래스 `public` 블록 없음):

- `void clean_chips(int16_t* chips, int N) noexcept;` — `38`
- `DiagCounters` 구조체 — `43`–`48`
- `DiagCounters& get_diag() noexcept;` — `51`
- `void reset_diag() noexcept;` / `void print_diag() noexcept;` — `54`–`55`

구현: `HTS_LIM/HTS_Walsh_Row_Converter.cpp` — `clean_chips` 정의 `202`행 근처(파일 헤더 `1`).

### `HTS_Walsh_Row_Permuter.hpp` (`HTS_LIM/HTS_Walsh_Row_Permuter.hpp`)

클래스 `Walsh_Row_Permuter` (`62`~) `public`:

- `Initialize` — `81`–`82`
- `Update_Key` — `85`–`86`
- `Encode_6bit` / `Decode_6bit` — `90`–`94`
- `Encode_4bit` / `Decode_4bit` — `96`–`98`
- `Is_Initialized` — `100`–`102`
- `Get_Update_Count` — `104`

`HTS_LIM/HTS_V400_Dispatcher.hpp` `494`: 멤버 `Walsh_Row_Permuter walsh_permuter_;`

### FWHT (메인 `HTS_LIM`)

| 항목 | 경로 / 라인 |
|------|----------------|
| `fwht_64_complex_inplace_` | `HTS_LIM/HTS_V400_Dispatcher_Internal.hpp` `78`–`94` |
| `fwht_8_complex_inplace_` | 동 파일 `113`–`128` |
| `fwht_raw` | 동 파일 `168`~ |
| `HTS_V400_Dispatcher_Sync.cpp` 내 호출 | 예: `145`–`146`(진단), `251`, `322`, `327`, `364`, `723`, `863`, `1277`, `1360` 등 |

### `clean_chips` 사용처 (검색 일부)

- `HTS_LIM/HTS_FEC_HARQ.cpp` `1612`–`1613`: `WRC::clean_chips(...)`

---

## Q5. CFO_Compensator 공존

### `cfo_.` 호출 (`HTS_LIM/HTS_LIM`만)

| 파일 | 라인 | 호출 |
|------|------|------|
| `HTS_V400_Dispatcher_Sync.cpp` | `1047` | `cfo_.Estimate_From_Preamble(...)` |
| 동일 | `1050` | `cfo_.Advance_Phase_Only(192)` |
| 동일 | `1167` | `cfo_.Apply(chip_I, chip_Q)` |
| `HTS_V400_Dispatcher_Core.cpp` | `344`, `502` | `cfo_.Init()` |

### `HTS_CFO_Compensator` API (헤더 `HTS_LIM/HTS_CFO_Compensator.h`)

- `Init` — `25`, 구현 `65`–`72`
- `Estimate_From_Preamble` — `27`–`31`, 주석 `27`: “Walsh-63 dot 결과로 CFO 추정”
- `Estimate_From_Preamble_MCE` — `38`–`39` 선언; **호출처는 `HTS_LIM` 트리 grep 결과 없음**
- `Apply` — `41`
- `Advance_Phase_Only` — `43`–`48`
- `Reset` — `52`, `74` (`Init` 위임)

### Walsh 도메인 CFO “흔적” (문자열)

- `HTS_V400_Dispatcher_Sync.cpp` `1034`, `1164`: 주석 “CFO 는 Walsh 도메인에서 처리 예정”
- `HTS_CFO_Compensator.h` `27`: `Estimate_From_Preamble` 주석에 Walsh-63 dot 명시
- `cfo.*walsh` 형태 grep: 위 항목 외 **추가 매칭 없음**

### 동일 심볼의 다른 트리 복사본 (참고만)

`Core/Src/HTS_V400_Dispatcher.cpp`, `HTS_TEST/HTS_V400_Dispatcher.cpp`, `HTS_TEST/PLUTO_SDR/HTS_V400_Dispatcher.cpp` 등에 `cfo_.Init` / `Estimate_From_Preamble` / `Apply` — **본 조사는 `HTS_LIM/HTS_LIM` 생산 경로 위주**.

### 공존 방식

**불명확** — 본 문서는 호출 순서·API 목록만 기록. `Estimate_From_Preamble`은 `phase0_scan_`의 `if (pass)` 내부(`1047`)에서만 관측됨.

---

## 종합 관측 (사실만)

| 항목 | 내용 |
|------|------|
| 예상 작업 범위 | `phase0_scan_` 집중 시 단일 파일 `HTS_V400_Dispatcher_Sync.cpp` 내 `52`–`1107` 및 `HTS_V400_Dispatcher_Internal.hpp`의 FWHT/에너지 유틸 재사용 가능성. **정확한 라인 수·일정은 미측정**. |
| 재사용 vs 신규 | FWHT: `fwht_64_complex_inplace_` 등 **기존 인라인** 존재. v20/v22 **외부 시뮬 소스는 검색 미발견**. |
| 잠재 리스크 (사실 기반) | (1) `HTS_PHASE0_WALSH_BANK`와 PS-LTE `#else`가 **별 컴파일 분기**로 공존. (2) `Estimate_From_Preamble_MCE`는 헤더에만 있고 **현 `HTS_LIM` 호출 없음**. (3) `Dispatcher`의 복수 트리 복사본 존재 — 변경 시 동기 필요 여부는 **본 조사에서 확인 안 함**. |

---

## 조사 한계

- Python/C++ 외 바이너리·외부 드라이브 미포함.
- `pipeline_audit_raw` 등은 사용자 지시에 따라 검색 제외하지 않았으나, v20/v22 키워드는 `HTS_LIM`/`CORE` 소스 위주로만 수행.

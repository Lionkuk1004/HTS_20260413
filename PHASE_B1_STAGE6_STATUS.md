# Stage 6 (V29b) 반영 상태 확정

**조사 방식**: 읽기 전용. **`git checkout main`** 시점의 `HTS_LIM` 트리만 검색·열람(ami_dry_run 아님).  
**조사 후**: `git checkout ami_dry_run` 로 복귀.

**`main` tip (관측)**: `2494e03` (`HARQ_MATRIX baseline before debug`).  
**분기 관계 (관측)**: `git merge-base --is-ancestor main dd10598` → exit **0** (`main` 은 `dd10598` 의 조상 → **분할 커밋보다 이전** 선형).

---

## Q1. `fec_ir_fwht_bin_unshift`

| 항목 | 결과 |
|------|------|
| 함수 정의 | **있음** — `HTS_LIM/HTS_FEC_HARQ.cpp` **`108`–`134`** 근처 `static inline void fec_ir_fwht_bin_unshift(int32_t* fI, int32_t* fQ, int nc, uint8_t walsh_shift)`; 주석 **`// Stage 6B:`** (`107`–`108`). |
| 호출 | **있음** — 동 파일 **`1207`**, **`1318`**, **`1587`** (grep). |
| `Decode64_IR` / `Decode16_IR` 의 `walsh_shift` | **있음** — `HTS_LIM/HTS_FEC_HARQ.hpp` **`279`–`286`**, **`293`–`300`** (`uint8_t walsh_shift = 0u` 기본 인자). |
| `walsh_shift` 문자열 참조(개략) | **`main` 단일 디스패처 트리**에서 `HTS_V400_Dispatcher.hpp` **1** + `HTS_V400_Dispatcher.cpp` **19** + `HTS_FEC_HARQ.hpp` **2** + `HTS_FEC_HARQ.cpp` **15** ≈ **37**회 (이전 `grep` `output_mode:count` 합산). |

**판정**: [x] **V29b 핵심 함수·IR 시그니처 존재** (Stage 6 코드 레벨 단서)  
[ ] 일부만 / [ ] 완전 부재

---

## Q2. `best_dom_row` (Fix A1 / 3-block consistency)

| 항목 | `main` (`HTS_V400_Dispatcher.cpp`) |
|------|--------------------------------------|
| 변수 선언 | **`2174`**: `int best_dom_row = 63;` |
| `best_dom_row` 갱신 | **`2706`–`2707`**: `HTS_PHASE0_WALSH_BANK` → `best_dom_row = max_row`. **`2715`–`2808`**: `#elif !defined(HTS_TARGET_AMI)` → Fix A1/A3 블록 후 **`best_dom_row = best_dom_pick`**. |
| `dominant_row_` 최종 반영 | **`3099`**: `dominant_row_ = static_cast<uint8_t>(best_dom_row);` (grep). 초기화 **`1017`**: `dominant_row_ = 63u`. |
| `cons_pass` / 3-block | **`2626`** `++cons_pass_count`; **`2923`** 근처 `[STAGE4-SCAN#%d] cons_pass=...` printf. |
| `first_c63_` / V29b 주석 | **`3789`–`3793`** `// V29b: linear floor best_e > first_c63_` |

**AMI 경로 (관측)**: `accum > best_e63` 블록에서 **`#elif !defined(HTS_TARGET_AMI)`** 만 `best_dom_row` 갱신 → **AMI는 해당 `#elif` 제외** → **AMI에서 `best_dom_row` 미갱신**(초기 `63` 유지). WALSH_BANK·PS-LTE는 위 분기들에서 갱신.

**판정**: [ ] 전 경로 갱신  
[x] **`best_dom_row` 일부 경로만 (AMI 누락)**  
[ ] 완전 부재

---

## Q3. HDR shift XOR / `current_walsh_shift_`

| 항목 | 결과 |
|------|------|
| `current_walsh_shift_()` | **있음** — `HTS_LIM/HTS_V400_Dispatcher.hpp` **`328`–`329`**: `return static_cast<uint8_t>(dominant_row_ ^ 63u);` 주석 **Stage 6A**. |
| `[HDR-SHIFT]` / 심볼 XOR | **있음** — `HTS_V400_Dispatcher.cpp` **`3673`**, **`3847`**, **`3888`** 등 `printf` 및 인근 **`sym ^ walsh_shift`** 패턴 (grep). |
| `walsh_dec_full_` 내부 | **관측**: `HTS_V400_Dispatcher.cpp` **`334`–`402`** — **FWHT 피크 탐색만**, **`walsh_shift` 인자·XOR 없음**. |

**판정**: [ ] HDR XOR가 `walsh_dec_full_` **내부**에 존재  
[x] **부분 존재** — HDR·템플릿 경로에서는 XOR·`walsh_shift` 사용; **`walsh_dec_full_` 본문에는 없음**  
[ ] 완전 부재

---

## Q4. `t6_cfo_final.txt` vs 현재 실측

| 항목 | 결과 |
|------|------|
| `HTS_TEST/t6_sim/t6_cfo_final.txt` | **워크스페이스에서 미발견** (`Glob **/t6_cfo*.txt` 0건). |
| `stepC2pre4_baseline_summary.txt` | 본 조사에서 **미확인**(지시 명령만으로는 파일 없음). |
| `baseline_run1.log` 의 S5 +2000Hz | **관측**: `║ S5 │ 2000Hz │ 100 │ ... │ PASS ║` (`477938`행 부근). → **100/100 PASS**(현 하네스는 trial **100**). |
| `20/20` 흔적 | **있음** — `HTS_TEST/t6_sim/stepC0_7_t6_v5.log` 등에 **`CFO +2000 Hz ... 20/20 ... PASS`** (다른 로그·**trial 수 20**). |

**추정:** 영준님 기억의 **「S5 20/20」** 과 현 **「100/100」** 은 **동일 PASS이나 trial 정의·하네스 버전 차이** 가능.

---

## Q5. Git 히스토리 (요약)

| 명령 | 관측 |
|------|--------|
| `--grep=V29b` | (실행) **일치 커밋 없음** |
| `-S 'fec_ir_fwht_bin_unshift' -- HTS_LIM/HTS_FEC_HARQ.cpp` | **`a556e57`**, **`71405e5`**, **`dd10598`** 등(문자열 터치 이력). |
| `-S 'best_dom_row' -- HTS_LIM/HTS_V400_Dispatcher.cpp` | **`71405e5`**, **`a556e57`**, `cd5d9da`, `6ef5e96`, … |
| `-S '10867'` (전체) | **저장소 텍스트에서 `10867/11200` 리터럴 미검색**; `10810/11200` 등은 `HTS_TEST/pipeline_experiment*` 등에 다수. |
| `HTS_FEC_HARQ.cpp` 최근 로그 | `71405e5`, `68e4cd3`, `24b79be`, `3f61545`, … (위 `git log` 출력 일부). |

**추정:** **「V29b」 문자열 태그**는 커밋 메시지에 없을 수 있으나, **코드 주석·기능(`Stage 6B`, Fix A1, `fec_ir_fwht_bin_unshift`)은 `main`에 존재**.

---

## Q6. Stage 6 없이 13839?

**관측:** `main` (`2494e03`)에도 **Stage 6 계열 코드가 이미 포함**되어 있음(Q1–Q3). 따라서 **「Stage 6이 빠져서 13839가 나왔다」** 는 설명은 **`main` 기준으로는 성립하기 어려움**.

**`13839/15200` (PS-LTE, 분할 브랜치 실측)** 은 **CFO 복구(`a556e57`)·디스패처 분할(`dd10598`) 이후 트리**에서 측정된 값으로, 이번 `main` 조사와는 **브랜치·커밋이 다름**.

**추정:** 수치 차이(**10810/11200**, **10867/11200** 기억, **13839/15200**)는 **하네스 trial 수·시나리오 세트·측정 브랜치** 차이와 **CFO/스캔 후속 수정**의 합성 가능성이 큼.

---

## Q7. AMI ↔ Stage 6

**관측 (`HTS_V400_Dispatcher.cpp` on `main`, AMI 분기와 동일 구조는 `Sync.cpp` 분할에도 존재):**

- **`best_dom_row`**: AMI는 **`#elif !defined(HTS_TARGET_AMI)`** 밖 → **갱신 없음**(Q2).  
- **`current_walsh_shift_()`**: `dominant_row_ ^ 63` — **`dominant_row_` 가 63 고정이면 시프트 0** (`63^63=0`).  
- **`fec_ir_fwht_bin_unshift`**: **타깃 분기와 무관**하게 `Decode64_IR` / `Decode16_IR` 내부에서 `walsh_shift` 인자로 호출 가능 — **AMI 빌드에서도 IR 경로가 쓰이면 호출됨**(호출 그래프는 별도 빌드로 확정 필요).

**추정:** AMI 저하의 직접 원인으로 **「Stage 6 자체가 리포지토리에서 삭제됨」** 보다는 **`best_dom_row`/dominant 고정 → `walsh_shift` 퇴화 + Phase0 AMI 메트릭** 쪽이 유력.

---

## 종합 판정 (`main` @ `2494e03`)

현재 **로컬 `main`** 에서 Stage 6 (V29b) 계열 반영 상태:

- [ ] 완전 반영 (태그·문서까지 동일) — **미확인**  
- [x] **대부분 반영** — FEC IR unshift·`walsh_shift` API·Fix A1·HDR XOR·Stage 6A/B 주석 **모두 코드에 존재**  
- [ ] 부분 반영(절반) / 대부분 누락 / 완전 부재 / 롤백됨 — **해당 없음** (`main` 기준)

**복원 전략 권고 (조사만, 구현 아님):**

- [ ] cherry-pick V29b 전용 커밋 — **커밋 메시지 `V29b` 미발견** → **hash 특정 불가**  
- [ ] **부분 복원** — `main` 에서 이미 존재 → **「Stage 6 복원」보다 `step_c1…` / `a556e57` 와의 diff 정리**가 우선일 수 있음  
- [ ] 수동 재작성 — **우선순위 낮음** (`main` 에 기능 존재)  
- [ ] 복원 위험 — **분할·FEC 병행 시 충돌**은 별도 병합 검토  

**예상 효과:** 본 문서는 **수치 예측 없음** (가설).

---

## CURSOR_URGENT_STAGE6_VERIFY.md

**관측:** 저장소 내 **`CURSOR_URGENT_STAGE6_VERIFY.md` 파일 미발견** (glob 0건).

---

## 다음 단계 (지시서 시나리오 매핑)

- **시나리오 R에 가까움**: **「Stage 6 대부분 부재(P)」** 는 **`main` 기준으로는 지지되기 어려움** — 오히려 **코드는 존재**.  
- **남은 갭**: (1) **AMI·`best_dom_row`**, (2) **`main` vs 통합 브랜치** 기능 동등성, (3) **과거 10867/11200 근거 로그·커밋** 재수집.

---

## 참고 경로 (`main` 조사 시)

- `HTS_LIM/HTS_FEC_HARQ.cpp` `107`–`134`, `1207`, `1318`, `1587`  
- `HTS_LIM/HTS_FEC_HARQ.hpp` `279`–`300`  
- `HTS_LIM/HTS_V400_Dispatcher.hpp` `326`–`329`  
- `HTS_LIM/HTS_V400_Dispatcher.cpp` `334`–`402`, `2700`–`2816`, `3099`, `3673`/`3847`/`3888`, `3789`–`3793`  
- `HTS_TEST/t6_sim/baseline_run1.log` `477938` (S5 2000Hz)  
- `HTS_TEST/t6_sim/stepC0_7_t6_v5.log` (20/20 CFO 행 다수)

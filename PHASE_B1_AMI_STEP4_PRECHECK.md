# PHASE_B1_AMI_STEP4_PRECHECK.md

**전제**: 읽기 전용 조사, 브랜치 `ami_dry_run`, 빌드·소스 수정 없음. 근거 코드는 `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp`, `HTS_V400_Dispatcher_Core.cpp`, `HTS_V400_Dispatcher_Payload.cpp`, `HTS_V400_Dispatcher_Decode.cpp`, `HTS_FEC_HARQ.cpp` 및 로그 `HTS_TEST/t6_sim/ami_step2_run1.log`.

---

## Q1. AMI Phase 1 est 누적

- **수식 (AMI, PRE_SYM0 분기에서만 누적)**  
  - `dot63a_*`, `dot63b_*`: 각각 칩 `0..31`, `32..63`에 대해 `k_w63[j]` 부호를 곱한 **32칩 walsh63 상관** (non-coherent half).  
  - **에너지 판정** `e63_64`: 반쪽별 에너지 **합**  
    `(dot63a_I²+dot63a_Q²) + (dot63b_I²+dot63b_Q²)`  
  - **est 누적**: `est_I_ += dot63_I`, `est_Q_ += dot63_Q` 여기서  
    `dot63_I = dot63a_I + dot63b_I`, `dot63_Q = dot63a_Q + dot63b_Q` (**두 half의 벡터 합**, 에너지 합이 아님).

- **PS-LTE 수식 (동일 블록의 `#else`)**  
  - 64칩을 FWHT한 뒤 `dominant_row_` 및 `sym1_row`에 해당하는 **단일 64칩 도메인** `dot63_I/Q`, `dot0_I/Q`로 `e63_64`, `e0_64` 계산.  
  - PRE_SYM0일 때 동일하게 `est_I_ += dot63_I`, `est_Q_ += dot63_Q` (**한 번의 64칩 FWHT 빈**).

- **주석 "CFO phase cancel" 전문** (`HTS_V400_Dispatcher_Sync.cpp` AMI 분기 직전):

  ```
  // est 누적용 단일 dot (두 half 합, CFO phase cancel 영향 있음)
  ```

- **차이**  
  - AMI: 에너지는 **반쪽 독립 제곱합**으로 비교하되, **위상 정보(est)** 는 **두 half 상관 벡터를 선형 합** — half 사이 CFO로 위상이 달라지면 벡터 합이 **줄어들거나 회전**할 수 있음(주석이 지적).  
  - PS-LTE: **전체 64칩**에 대한 단일 FWHT 도메인 dot로 est·에너지 일관.

---

## Q2. `update_derot_shift_from_est_()` 동작

- **수식** (`HTS_V400_Dispatcher_Core.cpp`):  
  - `abs_eI = |est_I_|`, `abs_eQ = |est_Q_|` (분기 없는 절대값).  
  - `est_mag64 = (uint64_t)abs_eI + (uint64_t)abs_eQ` (32비트씩 캐스트 후 합; 합이 `UINT32_MAX` 초과 시 상한 클램프).  
  - `est_mag64 == 0` 이면 `derot_shift_ = 17`.  
  - 아니면 `mag_u`의 **가장 높은 세트 비트 인덱스** `b`에 대해 `derot_shift_ = b` (즉 **L1 노름의 order of magnitude**에 해당하는 비트 위치).

- **est 왜곡 시 효과 (설계 의도 관점)**  
  - `|est_I|+|est_Q|` 가 커지면 `derot_shift_` 비트 위치가 커지고, 작아지면 작아짐 → **나중에 이 값을 쓰는 정규화/스케일 경로**가 있다면 칩 스케일링이 달라질 수 있음.

- **현 HTS_LIM 분할 소스에서의 사실**  
  - `derot_shift_`는 **멤버에 대입만** 되고, `HTS_LIM` 트리 내 **다른 `.cpp`/`.hpp`에서 읽는 참조가 없음** (리포 전체 `grep derot_shift_` 기준). 따라서 **현재 바이너리 관점에서 est→derot→페이로드 경로는 연결되지 않은 상태(사실상 dead store)**.

---

## Q3. derot 가 payload 에 사용되는 위치

- **`derot_cos` / `derot_sin` / `apply_derot`**: `HTS_LIM` 하위 **매칭 없음**.

- **사용 위치**  
  - `derot_shift_` 필드 및 `update_derot_shift_from_est_()` 정의·호출: `HTS_V400_Dispatcher.hpp`, `Core.cpp`, `Sync.cpp` (Phase 0 시드 est 확정 후, Phase 1 PRE_SYM0 누적 후, PRE_SYM1→HDR 전이 시 등 **갱신 호출만**).

- **페이로드·디코드와의 관계**  
  - `on_sym_()` (`Payload.cpp`): Walsh permuter, FWHT 피크 기반 `sh`, IR 버퍼 적재, **non-IR HARQ 누적** 등 — **`derot_shift_` 미사용**.  
  - IR DATA 경로: 주석상 **Dispatcher 레벨 derotation 제거** — `Decode64_IR`는 위상 불변·에너지 기반 경로 (`Payload.cpp` 해당 블록 주석).  
  - `walsh_dec_dot_proj_full_` (`Decode.cpp`): **est를 디코드 분기에 쓰지 않음** (주석: non-coherent 전용, est는 derot 등 **다른 경로**).

- **AMI에서 실제 적용 여부**  
  - 칩 단위 위상 보정은 **`cfo_.Apply(chip_I, chip_Q)`** (`Sync.cpp` Feed 경로) 등 **CFO 객체 쪽**으로 보이며, **`derot_shift_` 값을 소비하는 코드는 현 분할 트리에 없음** → **가설 X의 “derot_shift 오류 → payload decode” 연결은 이 코드 스냅샷으로는 성립하기 어려움**.

---

## Q4. Decode64_IR 호출 건수 추정

- **`[PAYLOAD-SHIFT]`** (`ami_step2_run1.log`): **12510**건 (`grep` 카운트).  
  - 코드상 동일 태그는 **`try_decode_()`** 에서 (1) `VIDEO_16`/`VOICE` + `ir_mode_` + `Decode16_IR` 성공 여부와 무관하게 `ir16_ok`이면 출력, (2) **`DATA` + `ir_mode_` + `Decode64_IR` 호출 직후** 출력.  
  - 로그만으로 **DATA vs VOICE/VIDEO_16 비율 분해는 불가** (동일 문자열).

- **`Decode64_IR` 로그**  
  - `HTS_FEC_HARQ.cpp`: `HTS_ALLOW_HOST_BUILD`에서 **`s_decode64_ir_entry_cap < 3` 일 때만** `stderr`에 `[Decode64_IR entry]` 출력 → **프로세스 전역 최대 3줄**. 본 로그에서 해당 패턴 **4행** 수준(스트림에 PowerShell `CategoryInfo` 줄이 섞일 수 있음) — **실제 호출 수의 하한/상한 지표가 아님**.

- **보조**: `[FEC-FWHT-BEFORE]`(Decode64_IR 내부 진단)도 **초기 소수 호출만** 찍히도록 되어 있어, **전체 호출 수 추정에는 부적합**.

- **정리 (추정 방식)**  
  - **상한 참고**: 모든 `[PAYLOAD-SHIFT]`가 `DATA`+`Decode64_IR`라면 디코드 시도 **≤ 12510** (라운드마다 1줄).  
  - **하한/실측**: 현 로그의 `Decode64_IR entry` 줄 수는 **의도적으로 3회로 캡**되어 있어 **실 호출 수를 반영하지 않음**.  
  - **Payload 모드 문자열**: 로그에 `DATA`/`VOICE`/`VIDEO_16` **리터럴 태그는 거의 없음**; `AJC-STATE`의 `ir_mode=0`은 **ctor/초기 상태 샘플**에 가깝고 런타임 `ir_mode_`와 1:1 대응한다고 보기 어려움.

---

## Q5. est 값 실측

- **로그 태그** (`HTS_DIAG_PRINTF` 가정, `Sync.cpp`):  
  - **`[P1-NC]`**: 매 Phase1 심볼 처리 시 `est=(%d,%d) n=%d` 등.  
  - **`[P1→HDR]`**: PRE_SYM1로 헤더 진입 시 `est=(%d,%d) n=%d`.

- **`[P1-NC]` / `[P1→HDR]` 샘플 (AMI, `ami_step2_run1.log` 앞부분)**:

  ```
  [P1-NC] sym=63 e63=62001 e0=0 est=(-95616,-95616) n=3 carry_pend=0
  [P1-NC] sym=63 e63=62001 e0=0 est=(-159360,-159360) n=4 carry_pend=0
  ...
  [P1→HDR] est=(-254975,-254975) n=6
  [P1-NC] sym=63 e63=601 e0=127 est=(-1983,-1983) n=3 carry_pend=0
  ...
  [P1→HDR] est=(2,2) n=15
  [P1→HDR] est=(1985,1985) n=17
  ```

- **PS-LTE 대비 수치 범위**  
  - 동일 하네스에서 PS-LTE 로그를 이번 스텝에서 **수치 대조하지는 않음** (요청 범위는 AMI 코드·AMI 로그). 구조적으로 PS-LTE는 **64칩 단일 dot** 누적이라 **크기·n 진행이 AMI와 직접 비교되기 어려움**.

- **왜곡 의심**  
  - 샘플에서 **`est_I_ == est_Q_`** 패턴이 반복됨 — AMI 시뮬/채널이 **대각선(I=Q) 에너지**에 가깝게 놓이면 자연스러울 수 있음.  
  - **“두 half 벡터 합”** 특성상 CFO가 크면 **진폭 감소·회전**이 가능하나, **그 결과가 `derot_shift_`를 통해 페이로드에 전달되는 경로는 현 소스에서 확인되지 않음** → **왜곡만으로 Decode64_IR 실패를 설명하는 단서는 약함**.

---

## 종합 가설 X 판정

- **[ ] est 왜곡 확정 (derot 엉망 → decode 실패)**  
  - `update_derot_shift_from_est_()`는 **`derot_shift_`만 설정**하고, **HTS_LIM 분할 소스 어디에서도 읽히지 않음**. IR 페이로드는 **Dispatcher derotation 제거** 주석·구조. → **이 스냅샷에서 가설 X의 인과 고리는 성립하기 어려움.**

- **[ ] est 정상 (derot 정상 → 다른 원인)**  
  - est 자체의 “정상/비정상”은 **별도 기준(PS-LTE 동일 조건 로그)** 없이 단정 불가. 다만 **derot가 디코드에 연결되지 않으므로** “derot 정상” 프레이밍은 의미가 제한적.

- **[x] 불명확 → Step 4 본 조사 필요** *(또는: 가설 X는 **현 코드 경로로는 기각**, 실패 원인은 **CFO/칩 파이프라인·FWHT·walsh_shift·IR 버퍼 등 다른 축** 우선)*  

**권장**: Step 4 본 조사에서는 **`derot_shift_` 소비처 부재**를 전제로, **Decode64_IR 입력(`ir_chip_*`)·`walsh_shift_payload`·CFO 누적** 등으로 범위를 좁히는 것이 타당.

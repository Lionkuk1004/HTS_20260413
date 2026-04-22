# PHASE_B1_T6_CHIP_RATE_PARAM.md

**조사 범위**: `HTS_TEST` 트리 (`t6_sim` 중심, 동일 상수를 쓰는 인접 하네스 참고). 읽기 전용, 빌드·코드 변경 없음.

---

## Q1. `kChipRate` 사용

### `HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp` (T6 SIM 본체)

| 항목 | 내용 |
|------|------|
| **선언** | **L74**: `static constexpr double kChipRate = 200000.0;` (`namespace {` 내부) |
| **사용** | **L337** (`ch_cfo`): `ph = 2.0 * kPi * cfo_hz * i / kChipRate` — **유일 사용처**(동 파일 `rg kChipRate` **2건**). |
| **200000 / 1M 리터럴 (T6 단일 파일)** | **200000.0** 는 **L74 한 곳**만. **1_000_000** 리터럴은 **본 파일에 없음**. |
| **수정 대상 (T6 SIM만 “파라미터화” 시)** | **선언 1곳**을 조건부 `constexpr` 로 바꾸면 되고, **호출부 추가 수정 불필요**(이미 심볼 참조). |

### `HTS_TEST` 기타 (참고 — 별도 TU/하네스)

| 파일 | 상수명 | 값 |
|------|--------|-----|
| `t6_waterfall/HTS_T6_Waterfall_Harness.cpp` | `kChipRate` | 200000.0 (+ `kChipRate/f_hz` 등) |
| `t6_mc/HTS_T6_MC_Harness.cpp` | 동일 패턴 | 200000.0 |
| `t6_sim/HTS_Jammer_STD_UnitTest.cpp` | `kChipRateHz` | 200000.0 |
| `t6_sim/HTS_Block_Message_Test.cpp` | `kChipRate_Hz` | 200000.0 |
| `t6_sim/HTS_Jammer_STD.hpp` | `F_CHIP_HZ` | 200000.0 |
| 기타 | `HTS_Phase3_MIL.hpp`, `HTS_BER_PER_Measure.hpp` 등 | 200000.0 계열 |

**요약**: T6 **정량 합계**에 쓰이는 실행 파일은 **`HTS_T6_SIM_Test.cpp` 단일 TU**이므로, **“AMI 13839 유지”만 보면 우선 이 파일 L74(+분기)만 맞추면 됨**. 다만 **동일 200k 가정**을 쓰는 다른 하네스는 **별도 정합**이 필요할 수 있음.

---

## Q2. CFO 외 수식

### `HTS_T6_SIM_Test.cpp` 채널 모델 (L307 이후)

| 함수 | `kChipRate` 사용 | 비고 |
|------|------------------|------|
| **`ch_cfo`** | **Y** (L337) | `ph = 2π · cfo_hz · i / kChipRate` — **칩 인덱스 `i`에 대한 누적 위상** |
| **`ch_awgn`** | **N** | `sigma`는 신호 전력·`snr_db`만 사용 |
| **`ch_rotate`** | **N** | 고정 각도 |
| **`ch_multipath_3tap`** | **N** | |
| **`ch_barrage`** | **N** | `kAmp`·JSR dB |
| **`ch_cw`** | **N** | `ph = 2π · i / period_chips` — **`period_chips` 인자**(S8 테이블에 **8, 12, 16** 등 **고정**, `kChipRate` 무관) |

### `1 / kChipRate` (chip period)

- **T6 SIM 본 파일**: **`1.0/kChipRate` 형태의 명시적 chip period 계산 없음**.

### 시간·주파수 의존 (CFO 외)

- **SNR/AWGN**: `ch_awgn` — **칩레이트 미사용**.
- **S5**: `cfos[] = {0,50,100,200,500,1000,2000,5000}` Hz (L718 근처) — **Hz 단위만** `ch_cfo`에 전달, **레이트는 `kChipRate`로만 반영**.

---

## Q3. 빌드 스크립트·조건부 컴파일

### 현재 매크로

| 스크립트 | 주요 `/D` |
|----------|-----------|
| **`cursor_t6_build.cmd`** (PS-LTE용 바이너리) | `HTS_ALLOW_HOST_BUILD`, `HTS_FEC_SIMULATE_M4_RAM_LAYOUT`, `HTS_DIAG_PRINTF`, `_CRT_SECURE_NO_WARNINGS` — **`HTS_TARGET_AMI` 없음** |
| **`cursor_t6_build_ami.cmd`** | 위 + **`/DHTS_TARGET_AMI`** |

### `HTS_T6_SIM_Test.cpp` 내 타깃 분기 예시

- **L1308 근처**: `#if defined(HTS_TARGET_AMI)` 로 **include할 Sync TU** 분기(이미 존재).

### `kChipRate` 를 매크로 분기로 바꿀 때의 함정 (Q4와 연결)

- **`HTS_V400_Dispatcher.hpp`는 `HTS_V400_Dispatcher_Internal.hpp`를 include 하지 않음.**  
- `kChipRate` 가 있는 **L74 시점**에는 아직 **`HTS_TARGET_PSLTE`가 자동으로 정의되지 않을 수 있음** (`Internal.hpp`의 “둘 다 없으면 `HTS_TARGET_PSLTE`” 규칙은 **그 헤더를 include한 TU 이후**에만 적용).

**실무적으로 안전한 분기 (제안)**:

```cpp
#if defined(HTS_TARGET_AMI)
static constexpr double kChipRate = 200000.0;   // AMI 빌드: 200 kcps
#else
static constexpr double kChipRate = 1000000.0;  // PS-LTE 빌드(AMI 미정의): 1 Mcps
#endif
```

- **`cursor_t6_build.cmd`**: `HTS_TARGET_AMI` 미정의 → **`#else` → 1 Mcps** (의도와 일치).
- **`cursor_t6_build_ami.cmd`**: `/DHTS_TARGET_AMI` → **200 kcps 유지** → **기존 AMI 실측 조건 유지 가능**.

**`HTS_TARGET_PSLTE` 단독 분기**를 쓰려면: **`Internal.hpp` 선행 include** 또는 **`cursor_t6_build.cmd`에 `/DHTS_TARGET_PSLTE` 명시** 중 하나가 필요(현재 cmd에는 **없음**).

---

## Q4. 제안 수정안 (개념 검증)

| 항목 | 내용 |
|------|------|
| **수정 파일 (최소)** | `HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp` **1파일** |
| **수정 규모** | **`kChipRate` 선언부 1줄 → `#if`/`#else` 블록**(통상 3~5줄). **`ch_cfo` 등 로직 변경 없음**. |
| **빌드 스크립트** | **AMI / PS-LTE cmd 모두 “그대로” 가능** — 위 **`#else` = AMI 아님 = PS-LTE 빌드`** 패턴이면 **추가 `/D` 불필요**. |
| **별도 매크로** (`HTS_T6_CHIPRATE_PSLTE` 등) | **필수는 아님**. 다만 “호스트 범용 빌드”처럼 **AMI도 아니고 PS-LTE도 아닌** 커스텀 cl이 생기면 기본값 정책을 **문서화**할 필요 있음. |

---

## Q5. PS-LTE 환경(1 Mcps) 영향

### 위상 / 칩 (동일 CFO Hz)

- **칩당 위상 증가**: \(\Delta\phi = 2\pi \cdot f_\mathrm{CFO} / R_\mathrm{chip}\).  
- **S5 예: 2000 Hz**  
  - **200 kcps**: \(\Delta\phi \approx 0.0628\) rad/chip → **64칩 ≈ 4.0 rad**.  
  - **1 Mcps**: \(\Delta\phi \approx 0.01257\) rad/chip → **64칩 ≈ 0.80 rad** (**약 5배 완만**).

→ **동일 Hz CFO 스윕이 “물리적으로 훨씬 약한 회전”**이 되어, **S5·S7 등 CFO 스트레스의 상대 난이도가 크게 낮아질 가능성이 큼**. (사용자 지시서의 “매우 여유” 방향과 일치.)

### 시나리오 유효성

- **수치·라벨(“2000 Hz”)은 그대로**이나, **의미(칩당 회전량)** 가 바뀌므로 **“기존 S5가 PS-LTE 1 Mcps에서도 동일한 스트레스인가?” → N에 가깝다**고 보는 것이 타당.

### CFO 범위 재조정

- **1 Mcps에서 동등한 난이도**를 원하면 **CFO Hz를 레이트에 비례해 올리는** 식(예: **×5**)의 **시나리오 재설계**가 **필요할 수 있음** — 별도 측정·튜닝 작업.

---

## 권고 (체크박스)

- **[x] 매크로 분기로 선언부만 수정 가능** — **`HTS_TARGET_AMI` 유무**가 가장 단순하고 **현재 빌드 스크립트와 1:1 대응**.
- **[ ] 다른 파일 수정 필요** — **T6 SIM “합계 회귀”만**이면 **불필요**. Waterfall/MC 등 **다른 벤치를 PS-LTE 1 Mcps에 맞출 경우**에는 해당 파일의 **200k 상수**도 검토.
- **[ ] 더 복잡한 구조가 필요한 경우** — 런타임으로 chip rate를 바꾸고 싶다면 **`constexpr` 제거·설정 API**로 가며 **메모리/결정성**이 달라짐(이번 목표 “컴파일 타임 분기”와 다름).

---

## 다음 작업 설계 (수정 지시서에 넣을 요지)

| 항목 | 제안 |
|------|------|
| **파일** | `HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp` |
| **변경** | `kChipRate` 를 **`#if defined(HTS_TARGET_AMI)` / `#else`** 로 **200k / 1M** 분기 |
| **PS-LTE 빌드 스크립트** | **그대로** 가능 (`HTS_TARGET_AMI` 없음 → 1M) |
| **AMI 빌드 스크립트** | **그대로** 가능 (`HTS_TARGET_AMI` → 200k) |
| **PS-LTE용 CFO 시나리오 조정** | **[ ] 필요** — 1 Mcps에서 **동일 스트레스**를 유지하려면 S5/S7 CFO **Hz 스윕 재스케일** 권장. |
| | **[ ] 불필요** — “완화된 환경에서의 절대 성능”만 보려면 **당장은 불필요**할 수 있음. |

**예상 결과 (개념)**:

- **AMI 빌드**: **`kChipRate` 불변(200k)** 이면 **13839/15200 유지** 가능성 높음(로직 동일).  
- **PS-LTE 빌드**: **1 Mcps 채널 모델**로 **CFO 합성만 완화** → **합계·S5 패스율이 오를 수 있음**(“회귀 숫자 고정” 목표와 충돌 시, 시나리오 조정은 별선).

---

## 검색에 사용한 명령 요약

- `rg kChipRate|chip_rate|200000|1000000` under `HTS_TEST/**/*.cpp|hpp|h`  
- `HTS_T6_SIM_Test.cpp` 수동 확인: `ch_*`, `test_S5`, `CwCase`, include 순서  
- `cursor_t6_build*.cmd`, `HTS_V400_Dispatcher_Internal.hpp` L10–15

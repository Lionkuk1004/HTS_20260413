# AMI 32-chip 경로 붕괴 진단 (문서화 43/11200, 실험일 2026-04-18)

**조사 범위**: 읽기 전용. 코드·스크립트 수정, `#error` 해제, 신규 빌드 없음.  
**관측**과 **추정:** 으로 구분.

---

## Q1. 커밋 이력

### 2026-04-18 당일(캘린더, 커밋 타임스탬프 기준)

**관측** (`git log --all --format="%h %ci %s" --since='2026-04-18T00:00' --until='2026-04-19T00:00'`):

- `cda65f3` `2026-04-18 02:33:12 +0900` `1000HZ 20/20`

(그 외 동일 하루 구간에 추가 커밋은 위 조회 결과에 없음.)

### 2026-04-15 ~ 2026-04-20 인근(이전 조사에서 넓게 본 목록 일부)

**관측**: `d6550da` `2026-04-17 12:34:13 +0900` `fix(dispatcher): P0-FIX-001 non-coherent 2-block phase0 scan` — `phase0_scan_` 비코히런트 2블록 스캔 관련. `dd10598`(`디스패쳐분활활`)의 조상임 (`merge-base --is-ancestor d6550da dd10598` → exit 0).

### `#error` 문자열 도입(파일 내용 기준)

**관측**: `git show dd10598:HTS_LIM/HTS_V400_Dispatcher_Internal.hpp` 최상단에 이미 다음이 존재함.

```cpp
#if defined(HTS_TARGET_AMI)
#  error "HTS_TARGET_AMI 32-chip 경로는 2026-04-18 T6 붕괴 (43/11200) 로 비활성. 원인 규명 전 빌드 금지."
#endif
```

즉 **`#error` 블록은 분할 커밋 `dd10598`(커밋일 `2026-04-21 03:18:13 +0900`) 시점의 `Internal.hpp`에 포함**되어 있음. 주석·문구의 **「2026-04-18」은 실험/기록 일자**이고, **저장소 타임스탬프와는 하루 차이**.

**관측** (`git log --all --oneline -S "43/11200" -- .`): 저장소 전체에서 **해당 문자열을 추가/삭제한 커밋은 조회 결과 없음**(문자열이 커밋 메시지·diff에 안 잡혔을 수 있음).

**관측** (`git log --all --oneline -S "HTS_TARGET_AMI 32-chip" -- HTS_LIM/HTS_V400_Dispatcher_Internal.hpp`): `dd10598` 등(히스토리에 `Internal.hpp`가 등장하는 구간).

### `a556e57` (지시서 언급 CFO 복구)

**관측**: `a556e57` `2026-04-21 22:15:57 +0900` `CFO복구` — `HTS_V400_Dispatcher_Internal.hpp`에는 **FWHT 스케일 헬퍼 include/inline 추가** 등(`git show a556e57 -- Internal.hpp` 상단 diff). **`#error` 라인 자체의 추가/삭제는 본 diff에 없음**(이미 `dd10598`에 존재).

### 붕괴를 유발한 단일 커밋

**추정:** 현재 저장소만으로 **「43/11200을 재현한 바이너리 = 특정 hash」** 를 단정할 증거 없음. 후보로는 (1) **`d6550da`** Phase0 스캔 변경, (2) **`dd10598`** 대규모 분할·통합, (3) 그 사이 **`71405e5`**(`cfo 2000hz달성`) 등 CFO 관련 변경이 **가설 후보**로 남음 — **bisect/재빌드 없이는 확정 불가**.

---

## Q2. 분할 파일 AMI 코드 분포

**관측** (`Get-ChildItem … | Select-String 'HTS_TARGET_AMI'`):

| 파일 | 내용 |
|------|------|
| `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp` | 다수 `#if defined(HTS_TARGET_AMI)` / `!defined(HTS_TARGET_AMI)` |
| `HTS_LIM/HTS_V400_Dispatcher_Internal.hpp` | `16`–`17` `#error`로 AMI 빌드 차단 |
| `HTS_LIM/HTS_V400_Dispatcher.cpp.pre_split_backup` | 동일 패턴 백업 |
| `HTS_LIM/HTS_V400_Dispatcher_TX.cpp` | **매치 없음** |
| `HTS_LIM/HTS_V400_Dispatcher_Payload.cpp` | **매치 없음** |
| `HTS_LIM/HTS_V400_Dispatcher_Core.cpp` | **매치 없음** |
| `HTS_LIM/HTS_V400_Dispatcher_Decode.cpp` | **매치 없음** |

**관측**: AMI 타깃 분기는 **사실상 `Sync.cpp` + `Internal.hpp`(차단) + 테스트용 로컬 복제**에 집중.

**관측**: `HTS_TEST/t6_sim/HTS_V400_Dispatcher_Local.cpp` `232`–`246` — `Internal.hpp`와 유사한 **AMI/PS-LTE 선택 주석 + 동일 `#error` 문구**(중복 정의 위험은 별도 이슈).

**추정:** 분할로 **TX/Payload에서 AMI 전용 코드가 사라진 것**은 본 grep으로는 **확인되지 않음**(애초 없을 수 있음).

---

## Q3. AMI Phase 0 scan

### Phase 0 `accum` (AMI 분기)

**파일** `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp` **`276`–`302`** (그대로 인용):

```cpp
#if defined(HTS_TARGET_AMI)
        // ── AMI: 32-chip × 2 non-coherent per block (기존) ──
        int64_t e_nc = 0;
        for (int blk = 0; blk < 2; ++blk) {
            const int base = off + (blk << 6);
            for (int half = 0; half < 2; ++half) {
                int32_t dI = 0, dQ = 0;
                for (int j = 0; j < 32; ++j) {
                    const int idx = base + (half << 5) + j;
                    const int32_t sI =
                        static_cast<int32_t>(p0_buf128_I_[idx]);
                    const int32_t sQ =
                        static_cast<int32_t>(p0_buf128_Q_[idx]);
                    const int widx = (half << 5) + j;
                    if (k_w63[static_cast<std::size_t>(widx)] > 0) {
                        dI += sI;
                        dQ += sQ;
                    } else {
                        dI -= sI;
                        dQ -= sQ;
                    }
                }
                e_nc += static_cast<int64_t>(dI) * dI +
                        static_cast<int64_t>(dQ) * dQ;
            }
        }
        accum = static_cast<int32_t>(e_nc >> 16);
```

**관측**: 블록 `blk=0,1` 각각에 대해 **half 0/1** 두 번(각 **32칩**) `k_w63` 가중 합 `dI,dQ` 후 **에너지 제곱합**을 `e_nc`에 누적, 최종 **`accum = e_nc >> 16`**.

### `best_dom_row` / `dominant_row_`

**관측**: `best_dom_row` 초기화 — `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp` **`56`**: `int best_dom_row = 63;`

`if (accum > best_e63)` 내부 (`583`~):

- `HTS_PHASE0_WALSH_BANK`: `best_dom_row = max_row` (`588`–`589`).
- `#elif !defined(HTS_TARGET_AMI)` (즉 **PS-LTE 등**): `best_dom_row = best_dom_pick` (`597`–`690`).
- **`HTS_TARGET_AMI`이면 위 두 분기 모두 제외** → **`best_dom_row` 갱신 없음** → **`63` 유지**.

`pass` 성공 시 `dominant_row_ = static_cast<uint8_t>(best_dom_row);` (`981`).

**추정:** AMI는 Phase1에서 별도 32+32 도트(`1305`~)를 쓰므로 **FWHT `dominant_row` 의미가 PS-LTE와 다름**; 그러나 **`dominant_row_=63` 고정**이 다른 공통 경로(게이트·로그·후속 심볼)에 영향을 줄 수 있는지는 **추가 추적 필요**.

### PS-LTE와 구조적 차이(관측)

- PS-LTE Phase0: `304`~ **FWHT 64×N**, `best_dom_pick` 등 **다중 블록 일관성** (`597`~).
- AMI Phase0: **32-chip XOR 누적 비코히런트**, **dominant row 미설정**.

---

## Q4. AMI TX Preamble

**관측**: `HTS_V400_Dispatcher_TX.cpp`에서 **`HTS_TARGET_AMI` 문자열 없음** — 본 조사 범위에서 **AMI 전용 TX 분기는 발견되지 않음**.

**추정:** AMI vs PS-LTE는 **수신 스캔/판정 측 매크로**로 분리되고, TX 패킷 생성은 **공통 또는 상위 계층**일 가능성 — **TX/RX 정합 여부는 본 grep만으로 판정 불가**.

---

## Q5. AMI 전용 상수

**파일** `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp`

| 항목 | 라인 | 관측 |
|------|------|------|
| `k_E63_ALIGN_MIN` (AMI) | `781`–`784` | `(amp32 << 4) + (amp32 << 1) + amp32` 주석 **`// amp × 19`** |
| `k_E63_ALIGN_MIN` (PS-LTE) | `786`–`788` | `(amp32<<5)+(amp32<<2)+(amp32<<1)` **`// amp × 38`** |
| `e63_min` | `790`–`791` | `max(adaptive_min, k_E63_ALIGN_MIN)` |
| `k_P1_MIN_E` (AMI) | `1380`–`1381` | **`500`** 주석 `32-chip × 2 non-coherent (peak 1/2)` |
| `k_P1_MIN_E` (PS-LTE) | `1382`–`1383` | **`1000`** |
| `k_P1_MIN_E` (WALSH_BANK) | `1378`–`1379` | **`1000`** |

**관측**: AMI 전용 **amp×19**, **P1 에너지 하한 500**이 코드에 명시되어 있음.

---

## Q6. CFO 경로 정합

**관측** (`HTS_V400_Dispatcher_Sync.cpp`):

- `1041`–`1047`: 두 블록 각각 `walsh63_dot_(…, d0I,d0Q)` / `d1I,d1Q` 후  
  `cfo_.Estimate_From_Preamble(d0I, d0Q, d1I, d1Q, 64);` (`1047`)
- `1050`: `cfo_.Advance_Phase_Only(192);`

**관측**: `Estimate_From_Preamble`의 **네 번째 인자 `block_chips`는 항상 `64`** (AMI/PS-LTE 분기 없음).

**관측** (`HTS_LIM/HTS_CFO_Compensator.h` `76`–`105`): `block_chips == 64` → `sin_per_chip_ = sin_block >> 6`; `16` 분기 별도; 그 외는 `sin_block / block_chips`.

**관측** (`HTS_V400_Dispatcher_Payload.cpp`): `cfo_.` 패턴 **미검색**(grep AMI와 별도로 이전 CFO grep에서 `Sync.cpp`·`Core.cpp`만).

**추정:** AMI 물리칩 간격이 32-chip 기준이라면 **128-chip 프리앰블에 대해 `64` 고정 스케일이 최적인지**는 코드만으로 **확인 불가**; 다만 **AMI/PS-LTE 공통 경로**임은 **관측**.

---

## Q7. AMI 이전 동작 버전

**관측**: 현재 **`#error`로 AMI 타깃 빌드 불가** → 워킹트리에서 **AMI ON + T6 통과**를 재현하는 커밋을 **실행 검증 없이** 특정할 수 없음.

**관측** (`git log --oneline --grep=AMI`): `251b98b`, `886c5eb`, `817846f`, `495472a`, `3c0ede4`, `a5d197f`, `0525e95` 등 **AMI Barrage30 / AMI 테스트** 관련 커밋 존재(메시지 기준).

**추정:** **「마지막으로 AMI가 정상이었다」** 는 커밋은 **로그·CI 기록 없이는 추정만 가능**; 복구 시 **`#error` 직전 조상에서 bisect** 권장(구현은 별도 지시서).

---

## Q8. 43/11200 분포

**관측**: `HTS_TEST/t6_sim/*.log` 에서 **`43/11200` 리터럴 문자열 grep 결과 없음**.

**관측**: `e1_run*.log` 등에 **`[DIAG-SCAN-STAT] total=11200`** 형태는 존재하나, **`43/` 단독 패턴**과의 일치는 본 검색에서 **특정하지 못함**.

**관측**: `43/11200` 및 동일 `#error` 문구는 **소스 주석**에 존재: `HTS_V400_Dispatcher_Internal.hpp` `17`; `HTS_V400_Dispatcher.cpp.pre_split_backup` `46`–`48`; `HTS_TEST/t6_sim/HTS_V400_Dispatcher_Local.cpp` `243`–`245`.

**관측**: 파일명 `*ami*`, `*collapse*` glob — **0건**(대소문자 무관 패턴으로도 미발견).

**추정:** **시나리오별 43 성분 분해표는 로그 소실/미보관**으로 본 조사에서는 **불가**.

---

## 종합 원인 추정 (가설 + 근거)

체크는 **「다음 단계에서 우선 검증할 가설」** (증명 아님).

- [ ] **분할 과정에서 AMI 로직 누락** — **반증 경향**: `pre_split_backup`에 동일 AMI/`#error` 패턴 존재; grep상 **TX/Payload에 AMI 분기 원래 없음**.
- [x] **AMI 전용 상수·에너지 스케일 vs 공통 `pass`/CFO 경로** — **근거(관측)**: AMI `accum`(`276`–`302`)·`amp×19`(`781`–`784`)·`k_P1_MIN_E=500`(`1380`–`1381`) vs PS-LTE FWHT·`×38`·`1000`; **재현 수치 없이 붕괴 직접 원인 단정은 불가**.
- [ ] **CFO 복구 `a556e57` 단독 AMI 비정합** — **부분 근거**: `a556e57`는 `#error`와 무관한 `Internal.hpp` 확장; **CFO 호출은 AMI에서도 `64` 고정**(`1047`). **AMI 전용 CFO 스케일 문제는 추정만 가능**.
- [ ] **TX/RX 정합 깨짐** — **근거 부족**(TX.cpp AMI 분기 없음).
- [x] **`best_dom_row` 미갱신(AMI에서 항상 초기값 63)** — **관측**: `583`–`691` 구조. **추정:** 후속 공통 코드가 `dominant_row_`를 **64-chip FWHT 의미로 가정**하는 경우 리스크 — **영향 범위 추가 조사 필요**.

---

## 복구 전략 권고 (관측 기반, 구현 아님)

- **방향 A (측정 우선)**: `#error` 해제는 **별도 지시서**에서만. 해제 후 **동일 T6 하네스에서 `best_e63`/`e63_min`/`pass`/`dominant_row_` 샘플 로그** 확보해 **43/11200 재현 여부와 시나리오 분포**를 **관측으로 고정**.
- **방향 B (구조)**: AMI에서 **`best_dom_row`/`dominant_row_` 의미**를 PS-LTE와 동등하게 쓸지, **AMI 전용 심볼**로 분리할지 결정 — 현재 **`best_dom_row` 비갱신**은 **코드 결함 후보**(관측).
- **방향 C (CFO)**: `Estimate_From_Preamble(..., 64)` 및 `Advance_Phase_Only(192)`가 **32-chip 타깃 스펙**과 위상 누적 길이가 맞는지 **스펙 대조**(문서/주파수 계획) 후 검증.
- **방향 D (역사)**: `d6550da` 이후 `dd10598`까지 **AMI 빌드 가능했던 구간 bisect**(빌드 허용 시).

**추천:** **방향 A + B**를 병행해 **관측 데이터 확보 후** C/D 선택 — 본 문서는 **원인 단정 없이** 증거 수집 순서만 제안.

---

## 참고 경로

- `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp` — AMI P0 `276`–`302`, `e63_min` `768`–`794`, seed `993`–`1001`, CFO `1038`–`1050`, P1 AMI `1305`–`1349`, `k_P1_MIN_E` `1378`–`1384`
- `HTS_LIM/HTS_V400_Dispatcher_Internal.hpp` — `#error` `16`–`17`
- `HTS_LIM/HTS_CFO_Compensator.h` — `Estimate_From_Preamble` `76`–`105`
- `HTS_TEST/t6_sim/HTS_V400_Dispatcher_Local.cpp` — 중복 AMI 차단 `243`–`245`

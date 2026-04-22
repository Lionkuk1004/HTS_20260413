# PHASE_B1_AMI_STEP2_PRECHECK.md

**전제**: `ami_dry_run` · `HTS_LIM/HTS_V400_Dispatcher_Sync.cpp` 중심 · 코드 수정 없음 · 읽기 전용.

---

## Q1. AMI `best_off` 확정 위치

### AMI 전용 **스코어(accum)** 블록

- **시작**: L274 `#if defined(HTS_TARGET_AMI)` — `phase0_scan_()` 안 `for (int off = 0; off < 64; ++off)` 루프 내부.
- **끝**: L300 `accum = static_cast<int32_t>(e_nc >> 16);` 직후, L301 `#else` (PS-LTE 경로) 전까지.

```274:301:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync.cpp
#if defined(HTS_TARGET_AMI)
        // ── AMI: 32-chip × 2 non-coherent per block (기존) ──
        int64_t e_nc = 0;
        for (int blk = 0; blk < 2; ++blk) {
            ...
        }
        accum = static_cast<int32_t>(e_nc >> 16);
#else
```

### `best_off` **최종 갱신** (AMI/PS-LTE 공통)

- 루프 본문에서 `accum > best_e63`일 때 **`best_off = off`** 한 곳에서 갱신된다.

```578:582:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync.cpp
        sum_all += static_cast<int64_t>(accum);
        if (accum > best_e63) {
            second_e63 = best_e63;
            best_e63 = accum;
            best_off = off;
```

**정리**: AMI는 **L274~L300**에서 `accum`만 정의하고, **`best_off` 확정은 L582** (공통 루프)에서 이루어진다.

### 참고: 다른 `#if defined(HTS_TARGET_AMI)` (L776 근처)

- L776~L779: **`k_E63_ALIGN_MIN` (amp×19)** 등 적응형 하한 — `best_off`와 무관.

---

## Q2. FWHT 헬퍼

| 항목 | 내용 |
|------|------|
| **함수명** | `fwht_64_complex_inplace_` |
| **선언 위치** | `HTS_LIM/HTS_V400_Dispatcher_Internal.hpp` |
| **시그니처** | `void fwht_64_complex_inplace_(int32_t* __restrict T_I, int32_t* __restrict T_Q) noexcept` — **64점 복소 I/Q in-place**, 버퍼는 호출부에서 **길이 64** `int32_t` 배열로 준비. |
| **호출 예** | `phase0_scan_()` 내 `HTS_PHASE0_WALSH_BANK` 분기: 블록별로 `p0_buf128_*` → `T_I[blk]` 복사 후 L249 `fwht_64_complex_inplace_(T_I[blk], T_Q[blk]);` |

```83:98:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Internal.hpp
inline void fwht_64_complex_inplace_(int32_t* __restrict T_I,
                                            int32_t* __restrict T_Q) noexcept {
    for (int stride = 1; stride < 64; stride <<= 1) {
        ...
    }
}
```

```243:249:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync.cpp
        for (int blk = 0; blk < 2; ++blk) {
            const int base = off + (blk << 6);
            for (int i = 0; i < 64; ++i) {
                T_I[blk][i] = static_cast<int32_t>(p0_buf128_I_[base + i]);
                T_Q[blk][i] = static_cast<int32_t>(p0_buf128_Q_[base + i]);
            }
            fwht_64_complex_inplace_(T_I[blk], T_Q[blk]);
```

---

## Q3. `dominant_row_` 설정 지점 (AMI에서의 공백)

### 멤버에 대한 **유일한 대입** (`phase0_scan_` 기준)

```975:975:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync.cpp
        dominant_row_ = static_cast<uint8_t>(best_dom_row);
```

- 위치: **`if (pass)`** 블록 안, L903 이후.

### `best_dom_row` 갱신 구조 (핵심)

- 초기값: L56 `int best_dom_row = 63;`
- 새 최대 `accum`일 때:
  - **WALSH_BANK**: L583~L584 `best_dom_row = max_row;`
  - **PS-LTE** (`#elif !defined(HTS_TARGET_AMI)`): L685 `best_dom_row = best_dom_pick;`
  - **AMI (`HTS_TARGET_AMI` + 비 WALSH_BANK)**: 위 두 분기 **모두 제외** → 루프 전체에서 **`best_dom_row`가 갱신되지 않음** → 항상 **63**.

```583:593:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync.cpp
#if defined(HTS_PHASE0_WALSH_BANK)
            best_dom_row = max_row;
            best_seed_I = seed_I_fwht;
            best_seed_Q = seed_Q_fwht;
...
#elif !defined(HTS_TARGET_AMI)
```

### β-1a 관점에서의 **삽입 후보**

1. **락 직전 (권장 후보)**: L975 **`dominant_row_ = ...` 직전**에, `best_off`·`p0_buf128_*`가 이미 확정된 상태에서 **FWHT로 산출한 row**를 `best_dom_row`에 넣은 뒤 기존 대입 유지.  
   - 장점: 스캔 루프의 AMI `accum` 판정은 그대로, **락 성공 시에만** 부가 비용.
2. **루프 내 `if (accum > best_e63)`**: AMI 전용으로 `best_dom_row`를 같이 갱신 — WALSH_BANK와 대칭적이나, 스캔마다 FWHT를 돌리면 **비용 증가** (β-1a “최소 침습”이면 락 1회 쪽이 유리).

---

## Q4. `p0_buf128_` 활용

### 크기·타입

```324:325:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.hpp
        int16_t p0_buf128_I_[192] = {};
        int16_t p0_buf128_Q_[192] = {};
```

- **192칩** 윈도우, `int16_t` I/Q 각각.

### `best_off` 기준 64칩 추출

- AMI 시드 경로에서 이미 **`best_off + (blk<<6)`** 로 블록 0/1 (각 64칩) 접근:

```986:994:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher_Sync.cpp
#elif defined(HTS_TARGET_AMI)
            for (int blk = 0; blk < 2; ++blk) {
                int32_t di = 0, dq = 0;
                walsh63_dot_(&p0_buf128_I_[best_off + (blk << 6)],
                             &p0_buf128_Q_[best_off + (blk << 6)],
                             di, dq);
```

- `best_off ∈ [0,63]` 이면 `best_off + 64 + 63 ≤ 190 < 192` → **블록 2개 모두 버퍼 내**.

**결론**: **`best_off` 확정 후, 동일 인덱스로 int32 스크래치에 64칩 복사 → `fwht_64_complex_inplace_` 호출**은 구조상 **Yes**.

---

## Q5. WALSH_BANK 경로 차용

### 로직 요약 (라인 범위)

- **L236~L272** (`#if defined(HTS_PHASE0_WALSH_BANK)`): 오프셋 `off`마다 2블록 × 64칩을 `T_I/T_Q`에 올리고 **블록마다 `fwht_64_complex_inplace_`**, 행별 에너지 합 `row_e[r]` 후 **argmax(63 tie-break)** → `max_row`, `accum`, `seed_I_fwht`/`seed_Q_fwht`.
- 새 최적일 때 **L584**에서 `best_dom_row = max_row`.

### AMI 축소 버전 차용 가능 여부

- **Yes** — 동일 헬퍼(`fwht_64_complex_inplace_`) + 동일 버퍼(`p0_buf128_*` @ `best_off`) 패턴을 **`off` 루프 밖·`best_off` 확정 후**에만 수행하면 됨.
- **“FWHT 1회”만** 고집할 경우: **한 블록(64칩)만** 변환해 row 추정하는 초소형 변형도 가능하나, WALSH_BANK와 정합하려면 **블록 2회 + row 에너지 결합**(L243~L256 스타일)이 참고본과 가장 가깝다.
- **필요 변경(설계 메모)**: `Internal.hpp`의 `fwht_64_complex_inplace_` / `find_argmax_64` 등은 이미 `Sync.cpp`에서 include 경로로 사용 중. AMI 전용으로 **`best_dom_row`만 산출**하는 지역 블록 추가면 되고, **`HTS_PHASE0_WALSH_BANK` 전역 전환 없이** β-1a 가능.

---

## Step 2 설계 초안 (구현 전 메모)

| 항목 | 제안 |
|------|------|
| **추가 코드 위치** | `if (pass)` 내부, **L975 `dominant_row_ = ...` 직전** (또는 직후 대입만 `best_dom_row`를 덮어쓰기). `#elif defined(HTS_TARGET_AMI)`로 기존 PS-LTE/WALSH_BANK와 분리. |
| **추가 라인 수 (추정)** | 지역 `int32_t T_I[64], T_Q[64]` 2세트 또는 WALSH_BANK와 동일 `T_I[2][64]` + 루프: 대략 **40~80** 라인 (DIAG·주석 포함 시 더 증가). |
| **예상 연산 증가** | **성공 락(`pass`)당** `fwht_64_complex_inplace_` **2회**(블록 0/1) + row 에너지 합산/argmax — 스캔 64×에는 넣지 않는 것이 WCET에 유리. |
| **예상 효과** | `best_dom_row`가 63 고정에서 벗어나 **`dominant_row_`가 시변** → Payload의 `[PAYLOAD-SHIFT]` 및 Stage 6 IR 경로가 **퇴화하지 않을** 여지 (실측으로 검증 필요). |

### 관측으로 확정된 사전 결론

- AMI는 **`best_dom_row`를 루프에서 한 번도 갱신하지 않아** L975에서 **`dominant_row_`가 사실상 항상 63**이다.  
- Step 2 β-1a는 이 공백을 메우는 것이 정합하다.

---

## 금지 사항 준수

- 본 문서 작성까지 **코드 수정·빌드·실측 없음**.

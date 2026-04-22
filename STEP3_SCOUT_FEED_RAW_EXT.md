# Step 3 사전 조사 보고

근거 파일: `HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp` (정적 읽기만, 빌드/실행 없음).

**참고 (워킹 트리 상태)**: 동일 TU에 Step 3에서 이미 추가된 `feed_raw_ext_holo` / `test_S5_holographic` / `main` 내 `#ifdef HTS_USE_HOLOGRAPHIC_SYNC` 호출이 존재한다. 아래 Q1~Q5는 **현재 파일 내용**을 그대로 인용한다.

---

## Q1. `feed_raw_ext`

### 함수 시그니처

```cpp
static TrialMetrics feed_raw_ext(uint32_t ds, const int16_t* rxI,
                                 const int16_t* rxQ, int n,
                                 const uint8_t* expected,
                                 int pre_guard = kGuard) noexcept;
```

- **위치**: L203–L206 (정의 시작 L203).
- **추가 파라미터**: `pre_guard` (기본값 `kGuard`). 일부 시나리오(S4 등)는 `feed_raw_ext(..., kGuard + off)` 로 명시 전달.

### `TrialMetrics` 필드 (동일 파일 L88–L94)

- `bool pass`
- `bool crc_passed`
- `bool length_correct`
- `int bit_errors`, `int byte_errors`  
- 지시서 초안에 있던 `crc_ok` / `decode_called` 명칭은 **실제 코드에 없음**.

### 내부 구조 요약 (L203–L248)

```203:248:HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp
static TrialMetrics feed_raw_ext(uint32_t ds, const int16_t* rxI,
                                 const int16_t* rxQ, int n,
                                 const uint8_t* expected,
                                 int pre_guard = kGuard) noexcept {
    TrialMetrics m{};
    m.pass = false;
    m.crc_passed = false;
    m.length_correct = false;
    m.bit_errors = 64;
    m.byte_errors = 8;

    g_last = DecodedPacket{};
    HTS_V400_Dispatcher rx;
    setup(rx, ds);

    for (int i = 0; i < pre_guard; ++i) rx.Feed_Chip(0, 0);
    for (int i = 0; i < n; ++i)         rx.Feed_Chip(rxI[i], rxQ[i]);
    for (int i = 0; i < kGuard; ++i)    rx.Feed_Chip(0, 0);

    m.crc_passed = (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
    m.length_correct = (g_last.data_len == 8);

    if (m.crc_passed && m.length_correct) {
        // ... payload 대비 bit/byte 오차 ...
        m.pass = (bit_err == 0);
    }
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::mark_packet_boundary_v2();
#elif defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::mark_packet_boundary();
#endif
    return m;
}
```

핵심 구조:

| 항목 | 실제 |
|------|------|
| Dispatcher 생성 | 스택 `HTS_V400_Dispatcher rx;` |
| `setup` | `setup(rx, ds);` 직후 feed |
| Callback | `setup` 내부에서 `Set_Packet_Callback(on_pkt)` (Q2 참고) |
| 선행 가드 | `pre_guard`회 `Feed_Chip(0,0)` (기본 `pre_guard == kGuard`) |
| 본문 | `n`회 `Feed_Chip(rxI[i], rxQ[i])` |
| 후행 가드 | `kGuard`회 `Feed_Chip(0,0)` — 상수 `kGuard` (L72: `256`) |
| `Set_Packet_Callback`을 `feed_raw_ext` 본문에서 직접 호출하는가? | **아니오** — `setup` 안에서 호출됨 |

### `feed_raw_harq`

- `HTS_T6_SIM_Test.cpp` 내 **`feed_raw_harq` 심볼 없음** (검색 결과 0건).

---

## Q2. `setup`

### 정의 (L169–L177)

```169:177:HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp
static void setup(HTS_V400_Dispatcher& d, uint32_t seed) noexcept {
    d.Set_Seed(seed);
    d.Set_IR_Mode(true);
    d.Set_Preamble_Boost(kPreBoost);
    d.Set_Preamble_Reps(kPreReps);
    d.Set_Packet_Callback(on_pkt);
    d.Update_Adaptive_BPS(1000);
    d.Set_Lab_IQ_Mode_Jam_Harness();
}
```

`setup_t6` 이름의 함수는 **없음**.

Dispatcher 설정 순서 (호출 순):

1. `Set_Seed(seed)`
2. `Set_IR_Mode(true)`
3. `Set_Preamble_Boost(kPreBoost)` — `kPreBoost`는 L70 `static constexpr int kPreBoost = 1;`
4. `Set_Preamble_Reps(kPreReps)` — L69 `kPreReps = 4`
5. `Set_Packet_Callback(on_pkt)`
6. `Update_Adaptive_BPS(1000)`
7. `Set_Lab_IQ_Mode_Jam_Harness()`

**없음**: `Set_Tx_Amp` — RX `setup`에는 없고, TX는 `build_tx` 안에서 `Build_Packet(..., kAmp, ...)` (L194–L195, `kAmp` L68).

### `Set_Holographic_Sync(true)` 삽입 위치

- `setup` 수정 없이, **`feed_raw_ext` 복제 함수에서 `setup(rx, ds);` 직후** 한 줄 추가하는 패턴이 코드와 일치한다. (현재 트리의 `feed_raw_ext_holo`는 L266 `setup` 직후 L267 `rx.Set_Holographic_Sync(true);`.)

---

## Q3. `test_S5` / S5F 명칭

### `test_S5` (L751–L824)

- **헬퍼**: `hdr`, `mk_seed`, `build_tx`, `ch_cfo`, `feed_raw_ext`, `row`, `record_ext` 및 조건부 diag `reset_*` / `print_*`.
- **`kTrials`**: L79 `static constexpr int kTrials = 100;`
- **CFO 배열**: L775–L780, **20개** 포인트 `0 … 25000` Hz (주석: 저역 + AMI 한계 + PS-LTE 상한).
- **seed**: `mk_seed(0x500000u, t)` — 지시서 가정 `0x500000u` **일치**.
- **집계**: 각 CFO마다 `record_ext("S5", param, ok, crc_only, kTrials, total_bits);` — `param`은 `snprintf`로 만든 `"%.0fHz"` 문자열.

### `test_S5F` 심볼

- **`static void test_S5F` 없음**.
- 동일 역할에 가까운 시나리오는 **`test_S5_seed_fixed`** (L910–L986), 테이블 태그는 **`"S5F"`** (`record_ext("S5F", ...)` L969).

### `test_S5` vs `test_S5_seed_fixed` 차이 (코드 근거)

| 항목 | `test_S5` | `test_S5_seed_fixed` |
|------|-----------|------------------------|
| CFO 배열 | L775–L780과 동일 20포인트 | L933–L938 **동일 배열** |
| TX | 매 trial `build_tx(mk_seed(0x500000u, t), t)` | **고정** `ds_fixed = mk_seed(0x500000u, 0)`, `tx_fixed = build_tx(ds_fixed, 0)` |
| 루프 | `t`로 페이로드/시드 변화 | `(void)t;` — 통계만 `kTrials` 유지, IQ는 동일 TX에 CFO만 가함 |
| `feed_raw_ext` | `feed_raw_ext(ds, rI, rQ, tx.n, tx.info)` | `feed_raw_ext(ds_fixed, rI, rQ, tx_fixed.n, tx_fixed.info)` |

---

## Q4. `hdr` / `row` / `record_ext` / 집계

### `hdr` (L444–L448)

```444:448:HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp
static void hdr(const char* id, const char* title) {
    std::printf("\n  ┌───────────────────────────────────────────────┐\n");
    std::printf("  │ %-3s %-43s │\n", id, title);
    std::printf("  └───────────────────────────────────────────────┘\n");
}
```

- 인자: `const char* id`, `const char* title` (지시서의 `tag`/`desc`에 해당).

### `row` (L450–L458)

- 시그니처: `static void row(const char* label, int pass, int total)` — 콘솔 진행 바 출력.

### `record_ext` (L120–L131)

```120:131:HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp
static void record_ext(const char* name, const char* param,
                       int pass, int crc_only, int total,
                       long long bit_errors) {
    if (g_n_results >= kMaxScenario) return;
    auto& r = g_results[g_n_results++];
    std::strncpy(r.name, name, 63);  r.name[63] = '\0';
    std::strncpy(r.param, param, 31); r.param[31] = '\0';
    r.pass = pass;
    r.crc_only_pass = crc_only;
    r.total = total;
    r.total_bit_errors = bit_errors;
}
```

- 두 번째 인자는 **`double cfo`가 아니라 `const char* param`**.
- `record_ext("S5", param, ok, crc_only, kTrials, total_bits)` 형태가 `test_S5`와 일치.

### 정량 합계

- **`print_summary` 함수 없음** — `main` 안에서 `g_results[0..g_n_results)` 루프로 `grand_pass` / `grand_total` / `grand_bits` 누적 후 `printf` (L1458–L1486 근처).

---

## Q5. `main`

### 호출 순서 (L1429–L1442)

```1429:1442:HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp
    test_S1();
    test_S2();
    test_S3();
    test_S4();
    test_S5();
    test_S5_seed_fixed();
#ifdef HTS_USE_HOLOGRAPHIC_SYNC
    test_S5_holographic();
#endif
    test_S6();
    test_S7();
    test_S8();
    test_S9();
    test_S10();
```

1. `test_S1` … `test_S4`  
2. `test_S5`  
3. `test_S5_seed_fixed()` (표시명 S5F)  
4. `#ifdef HTS_USE_HOLOGRAPHIC_SYNC` 일 때만 `test_S5_holographic()`  
5. `test_S6` … `test_S10`

### `test_S5_holographic` 추가 위치 (이미 반영된 경우)

- **`test_S5_seed_fixed()` 직후**, `test_S6()` 직전 — 위 인용과 동일.

### 정량 합계 출력 위치

- `main` 내부, 모든 `test_S*` 종료 후 — L1451 이후 테이블 및 L1483 근처 `정량 합계` 한 줄.

---

## 종합 평가

### Step 3 지시서 가정 대비 실제

| 가정 | 실제 | 일치 |
|------|------|------|
| `feed_raw_ext`가 Dispatcher 스택 생성 + `setup` 호출 | `HTS_V400_Dispatcher rx;` + `setup(rx, ds)` | OK |
| `setup` 이후 `Set_Holographic_Sync` 삽입 가능 | `setup`에 홀로 없음 → 별도 헬퍼에서 `setup` 직후 추가 가능 | OK |
| `test_S5`가 `kTrials` × CFO sweep | 이중 루프, 외부 CFO 20, 내부 `kTrials` | OK |
| `record_ext("S5", ...)` 로 집계 | `name`/`param` 문자열 + `ok`/`crc_only`/`kTrials`/`total_bits` | OK (`param`은 CFO double이 아님) |
| `main`에서 `test_S5()` 직접 호출 | 있음 | OK |
| 시험 지시서의 `test_S5F` 함수명 | 실제는 `test_S5_seed_fixed`, 태그 `"S5F"` | **명칭 불일치** — 지시서는 이름/호출을 `test_S5_seed_fixed` 기준으로 적는 것이 정확 |
| `TrialMetrics`에 `crc_ok` / `decode_called` | `crc_passed` / `length_correct` + `pass` | **필드명 불일치** — 지시서 예시 코드 수정 필요 |
| `feed_raw_harq` | 파일 내 없음 | 해당 가정 제거 |

### 조정 필요 사항 (지시서 v3 반영용)

1. **`TrialMetrics`**: 판정 시 `m.crc_ok` 대신 `m.crc_passed && m.length_correct`, `m.pass` 사용 (실제 `test_S5` L796–L800).  
2. **`record_ext`**: 두 번째 인자는 `const char* param` (CFO는 `snprintf`로 문자열화).  
3. **S5 고정 시드 시나리오**: `test_S5F()`가 아니라 **`test_S5_seed_fixed()`** 및 `record_ext("S5F", ...)`.  
4. **`pre_guard`**: 기본 `kGuard`(256); S5는 기본 인자만 사용.  
5. **`Set_Packet_Callback`**: `feed_raw_ext` 본문이 아니라 **`setup` 내부**에서 등록됨 — 설명 문구만 정확히 하면 됨.

### Step 3 지시서 수정 제안

| 항목 | 제안 |
|------|------|
| `feed_raw_ext_holo` 헬퍼 구조 변경 필요? | **No** — `setup` 직후 `Set_Holographic_Sync(true)` 한 줄 추가 패턴이 실제와 일치 (이미 구현된 형태와 동일). |
| `test_S5_holographic` 구조 변경 필요? | **No** — `test_S5`와 동일 이중 루프·동일 CFO 배열·`record_ext`만 태그/시드/feed 함수 분리면 됨. |
| seed domain 충돌? | S5는 `0x500000u`; 별도 시나리오는 다른 `mk_seed` 베이스 사용 가능 — **충돌 회피는 별도 베이스로 충분** (예: 이미 사용 중인 `0x5A5A5Au`). |

---

## 다음 단계

- 지시서 예시 코드의 **`TrialMetrics` 필드명**·**`record_ext` 시그니처**·**`test_S5F` 명칭**을 위 근거에 맞게 수정한 **Step 3 지시서 v3**를 쓰면 된다.
- 논리 가정(Dispatcher + `setup` + feed, CFO sweep, `main` 등록)은 **실제 코드와 일치**한다.

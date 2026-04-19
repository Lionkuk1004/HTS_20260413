# Conv + REP4 경로 Discovery (VOICE 16칩 / 공용 FEC)

**범위:** `HTS_LIM` 소스 읽기 전용. 일자 2026-04-19.  
**원칙:** 아래 인용은 저장소 원문 복사. 해석·추측으로 채우지 않은 항목은 **「확인 못함」**으로 표기.

---

## === DISC-V1 Conv Encoder ===

**File:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.cpp`

**Function signature (원문):**

```cpp
void FEC_HARQ::Conv_Encode(const uint8_t *in, int n, uint8_t *out) noexcept {
```

**File:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.hpp` (상수·생성 다항식)

**원문:**

```cpp
        static constexpr int CONV_K = 7;
        static constexpr int TAIL = CONV_K - 1;
        static constexpr int CONV_IN = INFO_BITS + TAIL;
        static constexpr int CONV_OUT = CONV_IN * 2;
        static constexpr int REP = 4;
        static constexpr int TOTAL_CODED = CONV_OUT * REP;
...
        static constexpr uint8_t G0 = 0x79u;  // Conv 생성 다항식 (공개 표준)
        static constexpr uint8_t G1 = 0x5Bu;
        static constexpr int NSTATES = 64;
```

**Code rate:** `CONV_OUT = CONV_IN * 2` 이므로 **인코더 출력은 입력 비트 수의 2배(쌍으로 출력)** — 원문에 “1/2” 문자열은 없음.

**Constraint length:** `CONV_K = 7` (원문).

**Generator polynomial (원문):** `G0 = 0x79u`, `G1 = 0x5Bu` 및 주석 `// Conv 생성 다항식 (공개 표준)`.

**입출력 비트 매핑 (Conv_Encode 본문 원문):**

```cpp
void FEC_HARQ::Conv_Encode(const uint8_t *in, int n, uint8_t *out) noexcept {
    uint8_t sr = 0u;
    for (int i = 0; i < n; ++i) {
        uint8_t r = static_cast<uint8_t>(((in[i] & 1u) << 6u) | sr);
        out[2 * i] = static_cast<uint8_t>(pc7(r & G0) & 1);
        out[2 * i + 1] = static_cast<uint8_t>(pc7(r & G1) & 1);
        sr = static_cast<uint8_t>((r >> 1u) & 0x3Fu);
    }
}
```

**Encode_Core에서의 호출·REP 배치 (원문):**

```cpp
        Conv_Encode(in_bits.data(), CONV_IN, conv.data());
        for (int r = 0; r < REP; ++r)
            for (int i = 0; i < CONV_OUT; ++i)
                wb.ru.rep[r * CONV_OUT + i] = conv[static_cast<std::size_t>(i)];
        Bit_Interleave(wb.ru.rep, TOTAL_CODED, il);
```

**Zero-tail:** `Encode_Core`에서 `in_bits`는 `std::array<uint8_t, static_cast<std::size_t>(CONV_IN)> in_bits{};` 초기화 후 `INFO_BITS`만 루프로 채움(원문 617–621행). 나머지 `TAIL` 비트는 배열 제로 초기화에 따름(원문만으로 비트별 의도 문장은 없음).

---

## === DISC-V2 Viterbi Decoder ===

**File:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.hpp`

**Function declaration (원문):**

```cpp
        static void Viterbi_Decode(const int32_t* soft, int nc,
            uint8_t* out, int no, WorkBuf& wb) noexcept;
```

**File:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.cpp`

**Function definition (원문):**

```cpp
void FEC_HARQ::Viterbi_Decode(const int32_t *soft, int nc, uint8_t *out, int no,
                              WorkBuf &wb) noexcept {
```

**State count:** 루프 `for (int st = 0; st < 64; ++st)` 및 `wb.pm[0][s]` / `wb.surv[t][ns]` — **64 states**. `HTS_FEC_HARQ.hpp`의 `NSTATES = 64`와 일치.

**PM 타입:** `WorkBuf`의 `int32_t pm[2][64];` (hpp 원문).

**Traceback length:** `const int T = nc >> 1;` … `const int steps = (T & T_lt) | (VIT_STEPS & ~T_lt);` 후 `for (int t = 0; t < steps; ++t)` 및 `for (int t = steps - 1; t >= 0; --t)`. `Decode_Core`에서 `Viterbi_Decode(wb.ru.all_llr, CONV_OUT, dec.data(), CONV_IN, wb);` 호출이므로 **`nc = CONV_OUT`**, **`no = CONV_IN`** (cpp 731행). `CONV_OUT/2`와 `VIT_STEPS` 관계는 hpp 주석·`static_assert`에 있음.

**Soft metric 입력:**

- **타입:** `const int32_t *soft` — 루프 내 `int32_t s0 = soft[2 * t], s1 = soft[2 * t + 1];`
- **범위(클램프):** `Viterbi_Decode` 본문에는 soft에 대한 `tpe_clamp_llr` 호출 없음. 별도 주석:

```cpp
//  tpe_sat_add_llr: ±500000 범위 — Viterbi 경로 메트릭 오버플로 방지
```

(위는 `tpe_sat_add_llr`에 대한 설명이며, `Viterbi_Decode` 입력 soft에 대한 직접 클램프 문장은 **확인 못함**.)

- **의미(부호):** 분기 메트릭 `int32_t bm = s0 * (1 - 2 * e0) + s1 * (1 - 2 * e1);` (원문). “양수=?, 음수=?”에 대한 주석 문장은 **확인 못함**.

**Decode16 과의 관계:** `Decode16`은 `Decode_Core` 호출만 수행(cpp 786–789행). `Decode_Core` 끝에서 `Viterbi_Decode` 호출 — REP4 합산은 **Viterbi 전**에 같은 함수 내에서 수행(아래 DISC-V3/V6).

**핵심 루프 원문 (Viterbi 일부, 약 20줄):**

```cpp
    const int T = nc >> 1;
    const int32_t T_lt =
        static_cast<int32_t>(0u - static_cast<uint32_t>(T < VIT_STEPS));
    const int steps = (T & T_lt) | (VIT_STEPS & ~T_lt);
    static constexpr int32_t DEAD_STATE = -1000000000;
    for (int s = 0; s < 64; ++s)
        wb.pm[0][s] = DEAD_STATE;
    wb.pm[0][0] = 0;
    int cur = 0;
    for (int t = 0; t < steps; ++t) {
        int nxt = 1 - cur;
        for (int s = 0; s < 64; ++s)
            wb.pm[nxt][s] = DEAD_STATE;
        int32_t s0 = soft[2 * t], s1 = soft[2 * t + 1];
        for (int st = 0; st < 64; ++st) {
            const int32_t pm_st = wb.pm[cur][st];
            const uint32_t m_alive =
                0u - static_cast<uint32_t>(pm_st > DEAD_STATE);
            for (int bit = 0; bit <= 1; ++bit) {
                uint8_t r =
                    static_cast<uint8_t>((static_cast<uint8_t>(bit) << 6u) |
                                         static_cast<uint8_t>(st));
                int ns = static_cast<int>((r >> 1u) & 0x3Fu);
                int e0 = pc7(static_cast<uint8_t>(r & G0)) & 1;
                int e1 = pc7(static_cast<uint8_t>(r & G1)) & 1;
                int32_t bm = s0 * (1 - 2 * e0) + s1 * (1 - 2 * e1);
                int32_t np = pm_st + bm;
```

---

## === DISC-V3 REP4 Structure ===

**인코더 REP4 위치**

- **File:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.cpp`
- **Function:** `FEC_HARQ::Encode_Core` (내부)
- **원문:**

```cpp
        Conv_Encode(in_bits.data(), CONV_IN, conv.data());
        for (int r = 0; r < REP; ++r)
            for (int i = 0; i < CONV_OUT; ++i)
                wb.ru.rep[r * CONV_OUT + i] = conv[static_cast<std::size_t>(i)];
        Bit_Interleave(wb.ru.rep, TOTAL_CODED, il);
```

→ **Conv 후** 동일 `conv[i]`를 `r=0..REP-1` 슬롯에 복사한 뒤, **`Bit_Interleave(wb.ru.rep, TOTAL_CODED, il)`** 호출.

**디코더 REP4 합산 위치**

- **File:** 동상 `HTS_FEC_HARQ.cpp`
- **Function:** `FEC_HARQ::Decode_Core`
- **원문:**

```cpp
        Bit_Deinterleave(wb.ru.all_llr, TOTAL_CODED, il, wb);
        for (int i = 0; i < CONV_OUT; ++i) {
            int32_t acc = wb.ru.all_llr[i];
            for (int r = 1; r < REP; ++r) {
                acc = tpe_sat_add_llr(acc, wb.ru.all_llr[r * CONV_OUT + i]);
            }
            wb.ru.all_llr[i] = acc;
        }
        dec.fill(static_cast<uint8_t>(0));
        Viterbi_Decode(wb.ru.all_llr, CONV_OUT, dec.data(), CONV_IN, wb);
```

→ **Bit_Deinterleave 이후**, **Viterbi_Decode 이전**.

**동일 비트 4번 배치**

- 인코더: `r * CONV_OUT + i`에 동일 `conv[i]` — **연속 블록 4개(인덱스 기준)** 로 복제 후 `Bit_Interleave`가 전체 `TOTAL_CODED`에 셔플.
- **방식:** 원문에 “연속/분산” 한글 용어는 없음. 위 코드만으로 **복제는 블록 단위 동일**, 이후 **인터리빙** 적용.

**hpp 주석 (원문):**

```cpp
        // rep(TX)·all_llr(RX) 동시 미사용 → 공용 저장 + REP 합산 후 all_llr[0..CONV_OUT) 가 Viterbi 입력
```

**16칩 심볼과의 매핑:** `Encode16`은 `Encode_Core(..., BPS16, NSYM16, wb)` 호출(cpp 768–770행). `BPS16`·`NSYM16`·`TOTAL_CODED` 관계는 hpp 상수. **심볼↔비트 위치를 한 문장으로 설명하는 별도 주석은 확인 못함**(루프는 `Encode_Core` 629–641행의 `sym`/`b`/`bi`).

---

## === DISC-V4 16-chip to Bit LLR ===

**Function:** `FEC_HARQ::Bin_To_LLR` (공용; `nc`, `bps` 인자)

**File:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.cpp`

**시그니처 (원문):**

```cpp
void FEC_HARQ::Bin_To_LLR(const int32_t *fI, const int32_t *fQ, int nc, int bps,
                          int32_t *llr) noexcept {
```

**16칩 경로에서의 호출 (Decode16_IR 내부, 원문):**

```cpp
        FWHT(fI.data(), nc);
        FWHT(fQ.data(), nc);
...
        Bin_To_LLR(fI.data(), fQ.data(), nc, bps, llr.data());
...
        for (int b = 0; b < bps; ++b) {
            const int bi = sym * bps + b;
            const uint32_t in_range =
                0u - static_cast<uint32_t>(bi < TOTAL_CODED);
            wb.ru.all_llr[static_cast<std::size_t>(bi)] =
                tpe_clamp_llr(llr[static_cast<std::size_t>(b)]) &
                static_cast<int32_t>(in_range);
        }
```

**FWHT 호출 (원문):** 위 인용. `nc == C16`일 때 `FWHT(..., 16)` — `FWHT` 본문에서 `if (n == 16) { FWHT_Unroll16(d); return; }` (cpp 287–289행).

**16칩 → 비트 매핑 로직 (Decode_Core 루프 일부, 15~30줄 분량 원문):**

```cpp
        for (int sym = 0; sym < nsym; ++sym) {
            const int base = sym * nc;
            std::memcpy(fI.data(), accI + base,
                        static_cast<std::size_t>(nc) * sizeof(int32_t));
            std::memcpy(fQ.data(), accQ + base,
                        static_cast<std::size_t>(nc) * sizeof(int32_t));
            FWHT(fI.data(), nc);
            FWHT(fQ.data(), nc);
            Bin_To_LLR(fI.data(), fQ.data(), nc, bps, llr.data());
            for (int b = 0; b < bps; ++b) {
                const int bi = sym * bps + b;
                // TPE: bi < TOTAL_CODED 마스크 — OOB 방지
                const uint32_t in_range =
                    0u - static_cast<uint32_t>(bi < TOTAL_CODED);
                const int32_t val =
                    tpe_clamp_llr(llr[static_cast<std::size_t>(b)]) &
                    static_cast<int32_t>(in_range);
                wb.ru.all_llr[static_cast<std::size_t>(bi)] = val;
            }
        }
```

**Walsh row 별 기여:** `Bin_To_LLR` 내부에서 `for (int m = 0; m < valid; ++m)` 및 `corr = fI[m] + fQ[m]` (Conv 분기, `HTS_FEC_POLAR_ENABLE` 미정의 시, cpp 415–425행). **“Walsh row”라는 단어가 이 함수에 있는지:** 본 검색 구간 주석에는 `//  Conv: 비트별 LLR = Σ corr(bit=0) − Σ corr(bit=1)` (cpp 385행) — **행 인덱스 명명은 “m”** .

---

## === DISC-V5 Decode16 Call Site ===

**File:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp`

**Line:** `try_decode_` 분기 — 본 파일에서 `Decode16` / `Decode16_IR` 문자열 출현 근처 **약 1446–1488행** (에디터/빌드에 따라 ±수행).

**Context (원문):**

```cpp
    } else if (cur_mode_ == PayloadMode::VIDEO_16 ||
               cur_mode_ == PayloadMode::VOICE) {
        if (ir_mode_) {
            harq_round_++;
            const int rv = ir_rv_;
...
            pkt.success_mask = static_cast<uint32_t>(
                0u -
                static_cast<uint32_t>(
                    ir16_ok &&
                    FEC_HARQ::Decode16_IR(
                        ir_chip_I_, ir_chip_Q_, FEC_HARQ::NSYM16,
                        FEC_HARQ::C16, FEC_HARQ::BPS16, il, rv, *ir_state_,
                        pkt.data, &pkt.data_len, wb_, walsh_shift_payload)));
...
        } else {
            FEC_HARQ::Advance_Round_16(rx_.m16);
            harq_round_++;
            pkt.success_mask = static_cast<uint32_t>(
                0u - static_cast<uint32_t>(FEC_HARQ::Decode16(
                         rx_.m16, pkt.data, &pkt.data_len, il, wb_)));
        }
```

**분기 조건 (원문):** `cur_mode_ == PayloadMode::VIDEO_16 || cur_mode_ == PayloadMode::VOICE` 및 내부 `if (ir_mode_)`.

**Decode16_IR 존재:** **Yes**

**시그니처 (hpp 원문):**

```cpp
        [[nodiscard]] static bool Decode16_IR(
            const int16_t* sym_I, const int16_t* sym_Q,
            int nsym, int nc, int bps,
            uint32_t il_seed, int rv,
            IR_RxState& ir_state,
            uint8_t* out, int* olen,
            WorkBuf& wb,
            uint8_t walsh_shift = 0u) noexcept;
```

---

## === DISC-V6 REP4 Soft Sum ===

**Location:** `HTS_FEC_HARQ.cpp` — `Decode_Core` 내 (본 리포 읽기 기준 **약 718–724행**).

**Code (원문):**

```cpp
        for (int i = 0; i < CONV_OUT; ++i) {
            int32_t acc = wb.ru.all_llr[i];
            for (int r = 1; r < REP; ++r) {
                acc = tpe_sat_add_llr(acc, wb.ru.all_llr[r * CONV_OUT + i]);
            }
            wb.ru.all_llr[i] = acc;
        }
```

**`tpe_sat_add_llr` (원문):**

```cpp
static inline int32_t tpe_sat_add_llr(int32_t a, int32_t b) noexcept {
    const int64_t s = static_cast<int64_t>(a) + static_cast<int64_t>(b);
    const int64_t lim = 500000;
    const int64_t m_pos = -(s > lim);
    const int64_t m_neg = -(s < -lim);
    const int64_t out = (lim & m_pos) | (-lim & m_neg) | (s & ~(m_pos | m_neg));
    return static_cast<int32_t>(out);
}
```

**중간 타입:** 누적 변수 `acc`는 `int32_t`; 내부 합은 `int64_t s`.

**Overflow 처리:** **포화(saturation)** — `±500000` 클램프(원문).

**합 vs 평균:** 루프는 가산만 수행 — **sum** (원문에 평균·스케일 문구 없음).

**Decode16_IR 동일 패턴 (원문, 약 1381–1388행):**

```cpp
    for (int i = 0; i < CONV_OUT; ++i) {
        int32_t acc = ir_state.llr_accum[static_cast<std::size_t>(i)];
        for (int r = 1; r < REP; ++r) {
            acc = tpe_sat_add_llr(
                acc,
                ir_state.llr_accum[static_cast<std::size_t>(r * CONV_OUT + i)]);
        }
        wb.ru.all_llr[static_cast<std::size_t>(i)] = acc;
    }
```

---

## === DISC-V7 S5/S7 Scenario ===

**File:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp`

**S5 2000Hz — `test_S5` 원문 (발췌):**

```cpp
static void test_S5() {
    hdr("S5", "극한 CFO 추종 (블라인드)");
    const double cfos[] = {0, 50, 100, 200, 500, 1000, 2000, 5000};
    for (double cfo : cfos) {
...
            ch_cfo(rI, rQ, tx.n, cfo);
            const TrialMetrics m = feed_raw_ext(ds, rI, rQ, tx.n, tx.info);
```

**CFO 값:** 루프 변수 `cfo`; 2000Hz는 배열 원소 **2000** (원문 `const double cfos[]`).

**패킷 모드:** `build_tx` (원문):

```cpp
    pkt.n = tx.Build_Packet(PayloadMode::DATA, pkt.info, 8, kAmp,
                            pkt.I, pkt.Q, kMaxC);
```

→ **DATA** (원문 `PayloadMode::DATA`).

**S7 — `test_S7` 원문 (발췌):**

```cpp
static void test_S7() {
    hdr("S7", "바라지 재밍 (JSR +10 ~ +30 dB)");
    std::mt19937 rng(0x70000000u);
    const double jsrs[] = {0, 5, 10, 15, 20, 25, 30};
    for (double jsr : jsrs) {
...
            ch_barrage(rI, rQ, tx.n, jsr, rng);
            const TrialMetrics m = feed_raw_ext(ds, rI, rQ, tx.n, tx.info);
```

**재밍 파라미터:** `jsr` 및 `ch_barrage(rI, rQ, tx.n, jsr, rng)` (원문). `ch_barrage` 본체 상세는 **이 파일의 다른 줄** — 필요 시 별도 열람.

**사용 경로:** 동일하게 `build_tx` → **DATA**.

**`setup` (원문):**

```cpp
static void setup(HTS_V400_Dispatcher& d, uint32_t seed) noexcept {
...
    d.Set_IR_Mode(true);
}
```

**Decode16 호출 여부 (T6 SIM 이 파일만으로):** `Build_Packet(PayloadMode::DATA, ...)` 이므로 **`try_decode_`의 VOICE/VIDEO_16 분기(따라서 `Decode16`/`Decode16_IR`)는 이 하네스의 S5/S7 경로에서 호출된다고 단정할 근거는 원문에 없음.** → **이 시나리오 정의만 놓고 보면 No (DATA 빌드)**.

**실패 유형:** `feed_raw_ext`는 `success_mask`, `data_len`, `memcmp`류 비트오류만 측정(원문 196–217행). **timeout** 필드는 **확인 못함**.

---

## === DISC-V8 Walsh Row Mapping (16-chip) ===

**FWHT size:** `FWHT(fI.data(), nc);` 에서 `Decode16_IR`는 `nc != C16`이면 실패 분기(1299–1301행). **`nc == 16`**.

**관련 함수:** `FEC_HARQ::FWHT`, `FWHT_Unroll16` (cpp 198–232, 283–289행).

**원문 (FWHT 분기):**

```cpp
void FEC_HARQ::FWHT(int32_t *d, int n) noexcept {
    if (d == nullptr || n <= 1) {
        return;
    }
    if (n == 16) {
        FWHT_Unroll16(d);
        return;
    }
```

**신호 row (16칩에서 “어느 row가 신호인지”):** 본 Discovery에서 **해당 문구/인덱스 상수는 검색하지 않음** → **확인 못함**.

**REP4 ↔ row 관계:** REP4 합산은 **디인터리브 후 인덱스 `i` 및 `r * CONV_OUT + i`** (위 DISC-V3/V6 원문). **Walsh bin `m`과 `i`의 1:1 대응을 적은 한 줄 주석으로는 확인 못함** → **확인 못함**.

---

## === DISC-V9 Decode16 Performance ===

**`HTS_T6_SIM_Test.cpp` 검색 결과:** `PayloadMode::VOICE` / `VIDEO_16` 문자열 **없음**. `Build_Packet`은 **`PayloadMode::DATA`** 만 사용(원문 168행, 765행 근처).

**VOICE 경로 사용 시나리오:** 이 테스트 파일 원문만으로는 **없음** — **pass 합계 산출: 확인 못함**.

**Baseline 10927/11200 중 Decode16 기여:** T6 SIM 소스만으로 **수치 분해 불가** — **확인 못함**.

---

## === DISC-V10 Insertion Candidates ===

**후보 1:** `HTS_FEC_HARQ.cpp` — `Decode_Core`, `Bit_Deinterleave` 직후·REP 합산 루프 직전.

**기존 코드 (원문):**

```cpp
        Bit_Deinterleave(wb.ru.all_llr, TOTAL_CODED, il, wb);
        for (int i = 0; i < CONV_OUT; ++i) {
            int32_t acc = wb.ru.all_llr[i];
            for (int r = 1; r < REP; ++r) {
                acc = tpe_sat_add_llr(acc, wb.ru.all_llr[r * CONV_OUT + i]);
            }
            wb.ru.all_llr[i] = acc;
        }
```

**삽입 가능성:** “가중치”를 **`all_llr[...]` 또는 `tpe_sat_add_llr` 인자**에 곱하기 전에 넣는 형태는 코드 구조상 가능하나, **설계·회귀 영향은 본 문서 범위 밖** — 등급화는 **확인 못함**.

**후보 2:** `HTS_FEC_HARQ.cpp` — `Decode16_IR`, `Bit_Deinterleave` 이후 동일 패턴(원문 1373–1388행 인용, DISC-V6).

**후보 3:** `Bin_To_LLR` 출력 이후 `wb.ru.all_llr[bi] = tpe_clamp_llr(...)` 대입 직전 — **FWHT 도메인 `fI`/`fQ`의 인덱스 `m`과 `bi`의 대응**을 추가로 정의해야 함(원문만으로는 단정 없음).

**권장:** **확인 못함** (요구된 Step 1 설계 없이 번호 지정은 추측에 해당).

---

## === DISC-V11 VOICE/DATA Separation ===

**공유 함수 (원문 근거):**

- `Decode16` / `Decode64` / `Decode64_A` 모두 **`Decode_Core`** 호출(cpp 786–789, 816–819, 892–893행).
- `Encode16` / `Encode64`(Polar 비활성 시) **`Encode_Core`** (cpp 768–770, 799행 등).
- **`Bin_To_LLR`**, **`FWHT`**, **`Bit_Interleave` / `Bit_Deinterleave`**, **`Viterbi_Decode`**, **`Conv_Encode`** — `Encode_Core`/`Decode_Core`/IR 경로에서 공용.

**공유 버퍼 (hpp 원문):**

```cpp
        struct alignas(32) WorkBuf {
            int32_t  pm[2][64];
            uint8_t  surv[VIT_STEPS][64];
            uint8_t  tb[VIT_STEPS];
            uint16_t perm[TOTAL_CODED];
            int32_t  tmp_soft[TOTAL_CODED];
            union {
                uint8_t  rep[TOTAL_CODED];
                int32_t  all_llr[TOTAL_CODED];
            } ru;
        };
```

**`HTS_V400_Dispatcher`의 `wb_`:** `FEC_HARQ::WorkBuf wb_{};` (hpp — 본 Discovery에서 인용 생략 가능하나 **호출 경로상 try_decode_가 wb_ 사용**).

**분기 조건 (원문):** `try_decode_`에서 `VIDEO_1` / `VIDEO_16|VOICE` / `DATA` 등 모드별 분기(`HTS_V400_Dispatcher.cpp` 인용, DISC-V5).

**분리된 함수:** `Decode16` vs `Decode64` / `Decode64_A` — hpp 선언은 별도(cpp는 `Decode_Core` 위임).

**VOICE만 수정 시 DATA 영향:** **동일 `Decode_Core`·`WorkBuf` 레이아웃·`TOTAL_CODED`를 공유**하므로, **`Decode_Core` 본문을 바꾸면 DATA 경로도 동일 코드 경로** — 원문 구조상 **영향 가능성 있음**. 정량적 회귀는 **확인 못함**.

---

## 자진 고지

- **라인 번호**는 조사 시점의 워크스페이스 기준이며, 편집으로 변동 가능.
- **S5/S7 실패 “원인”**은 채널·동기·재밍 물리를 코드 밖에서 단정하지 않음(지시: 추측 금지).
- **`ch_cfo` / `ch_barrage` 내부**는 DISC-V7에 필요한 최소만 인용; 상세는 미포함.

---

## 다음 단계 (사용자 메모 전달만)

1. 각 경로 특성 정리  
2. Step 1 (REP4 row_weight) 구체 설계  
3. 삽입 위치 확정  
4. 회귀 위험 평가  
5. `cursor_rep4_row_weight.cpp` 작성  

(코드 변경 없음 — 본 파일은 Discovery 기록만.)

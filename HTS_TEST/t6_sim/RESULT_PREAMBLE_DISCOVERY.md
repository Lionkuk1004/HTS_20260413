# Preamble 동기 구조 Discovery (읽기 전용, 2026-04-19)

코드 기준: `HTS_LIM/HTS_V400_Dispatcher.hpp`, `HTS_LIM/HTS_V400_Dispatcher.cpp`, `HTS_TEST/t6_sim/HTS_T6_SIM_Test.cpp`. 추측 없이 **파일·라인·원문**만 정리. `HTS_TARGET_AMI` / `HTS_PHASE0_WALSH_BANK` 분기는 T6 기본(PS-LTE)과 다름 — 해당 분기는 **원문 존재만** 표기.

---

## === DISC-P1 Preamble Definition ===

**File:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.hpp`

**PRE_SYM0 / PRE_SYM1 정의 (원문):**

```270:273:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.hpp
        /// @brief 프리앰블 심볼 0 (0x3F = Walsh row 63)
        static constexpr uint8_t PRE_SYM0 = 0x3Fu;
        /// @brief 프리앰블 심볼 1 (0x00 = Walsh row 0)
        static constexpr uint8_t PRE_SYM1 = 0x00u;
```

**TX 조립 (Build_Packet, 원문):**

```1761:1862:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
    const int16_t pre_amp =
        static_cast<int16_t>(static_cast<int32_t>(amp) * pre_boost_);
    // [BUG-FIX-PRE2] 프리앰블 반복 전송 — pre_reps_ × PRE_SYM0 + 1 × PRE_SYM1
    const int pre_chips = (pre_reps_ + 1) * 64;
    ...
    int pos = 0;
    for (int r = 0; r < pre_reps_; ++r) {
        walsh_enc(PRE_SYM0, 64, pre_amp, bp_dst_i(oI, pos, okm),
                  bp_dst_q(oQ, pos, okm));
        pos += 64 * inc;
    }
    walsh_enc(PRE_SYM1, 64, pre_amp, bp_dst_i(oI, pos, okm),
              bp_dst_q(oQ, pos, okm));
    pos += 64 * inc;
```

**전체 preamble 구조 (코드에서 읽히는 범위):**

- **심볼 개수(논리):** `pre_reps_`회의 **PRE_SYM0** + **1회 PRE_SYM1** → **pre_reps_ + 1** 심볼.
- **칩 개수:** `pre_chips = (pre_reps_ + 1) * 64`.
- **패턴:** 각 심볼은 `walsh_enc(..., 64, pre_amp, ...)` 로 **64칩**씩 출력; PRE_SYM0=Walsh row **63**, PRE_SYM1=row **0** (헤더 주석).

---

## === DISC-P2 phase0_scan_ ===

**File:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp`  
**Line:** **2164–3140** (`phase0_scan_` 본문 전체; `#if` 분기로 길이 변동).

**시그니처 (원문):**

```2164:2167:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
void HTS_V400_Dispatcher::phase0_scan_() noexcept {
    int32_t best_e63 = 0, second_e63 = 0;
    int best_off = -1;
    int64_t sum_all = 0;
```

**입력 버퍼:** `p0_buf128_I_[192]`, `p0_buf128_Q_[192]` (`HTS_V400_Dispatcher.hpp` 멤버, Phase 0 주석).

**반환값:** 함수는 `void`. **`best_off` / `best_e63` / `second_e63` 등은 지역 변수**이며, 성공 시 **`dominant_row_`·`psal_off_`·`psal_e63_`·`pre_agc_` 등 멤버에 반영** (P7·P4 인용).

**본문 (연속 61줄, PS-LTE 경로 `#else` — `HTS_PHASE0_WALSH_BANK`·`HTS_TARGET_AMI` 미정의 시 컴파일되는 분기, 원문):**

```2416:2476:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
        // Stage 4b (PS-LTE): 3×64 FWHT 동일 row 일관성 (pre_reps_>1 시 PRE_SYM0 반복)
        // p0_buf128_*_[192]: off+191<192 일 때만 3블록 안전 → off==0 만 4b 전체.
        // 그 외 off 는 기존 2블록 XOR+fallback (슬라이드 OOB 방지).
        alignas(32) int32_t T0_I[64];
        alignas(32) int32_t T0_Q[64];
        alignas(32) int32_t T1_I[64];
        alignas(32) int32_t T1_Q[64];
        alignas(32) int32_t T2_I[64];
        alignas(32) int32_t T2_Q[64];
        int r0 = 63;
        int r1 = 63;
        int r2 = 63;
        int max_row0 = 63;
        int max_row1 = 63;
        for (int i = 0; i < 64; ++i) {
            T0_I[i] = static_cast<int32_t>(p0_buf128_I_[off + i]);
            T0_Q[i] = static_cast<int32_t>(p0_buf128_Q_[off + i]);
        }
        fwht_64_complex_inplace_(T0_I, T0_Q);
        for (int i = 0; i < 64; ++i) {
            T1_I[i] = static_cast<int32_t>(p0_buf128_I_[off + 64 + i]);
            T1_Q[i] = static_cast<int32_t>(p0_buf128_Q_[off + 64 + i]);
        }
        fwht_64_complex_inplace_(T1_I, T1_Q);
        int64_t max_e0 = static_cast<int64_t>(T0_I[63]) * T0_I[63] +
                         static_cast<int64_t>(T0_Q[63]) * T0_Q[63];
        max_row0 = 63;
        for (int r = 0; r < 64; ++r) {
            if (r == 63) {
                continue;
            }
            const int64_t e = static_cast<int64_t>(T0_I[r]) * T0_I[r] +
                              static_cast<int64_t>(T0_Q[r]) * T0_Q[r];
            if (e > max_e0) {
                max_e0 = e;
                max_row0 = r;
            }
        }
        int64_t max_e1 = static_cast<int64_t>(T1_I[63]) * T1_I[63] +
                         static_cast<int64_t>(T1_Q[63]) * T1_Q[63];
        max_row1 = 63;
        for (int r = 0; r < 64; ++r) {
            if (r == 63) {
                continue;
            }
            const int64_t e = static_cast<int64_t>(T1_I[r]) * T1_I[r] +
                              static_cast<int64_t>(T1_Q[r]) * T1_Q[r];
            if (e > max_e1) {
                max_e1 = e;
                max_row1 = r;
            }
        }
        r0 = max_row0;
        r1 = max_row1;
        int64_t max_e2 = 0;
        if ((off + 128 + 63) < 192) {
            for (int i = 0; i < 64; ++i) {
                T2_I[i] = static_cast<int32_t>(p0_buf128_I_[off + 128 + i]);
                T2_Q[i] = static_cast<int32_t>(p0_buf128_Q_[off + 128 + i]);
            }
            fwht_64_complex_inplace_(T2_I, T2_Q);
```

(이후 동일 `if` 블록 안에 `HTS_DIAG_PRINTF` 전용 dump 코드가 이어짐 — **2477행~**.)

**내부 FWHT (동일 분기):** 위 블록 내 `fwht_64_complex_inplace_(T0_I, T0_Q)` 등.

---

## === DISC-P3 Correlation Algorithm ===

**방식 (PS-LTE, `HTS_PHASE0_WALSH_BANK`·`HTS_TARGET_AMI` 미정의 시):**

- **Time-domain** 샘플 `p0_buf128_*`를 블록 단위로 잘라 **`fwht_64_complex_inplace_`** 적용.
- **에너지:** 각 FWHT 빈 `r`에 대해 **`T*_I[r]*T*_I[r] + T*_Q[r]*T*_Q[r]`** (`int64_t`) — **I²+Q² 합**(복소 상관이 아닌 **에너지·비일관(noncoh) 스타일**).

**핵심 루프 예 (블록0 최대 행 탐색, 원문):**

```2443:2452:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
        for (int r = 0; r < 64; ++r) {
            if (r == 63) {
                continue;
            }
            const int64_t e = static_cast<int64_t>(T0_I[r]) * T0_I[r] +
                              static_cast<int64_t>(T0_Q[r]) * T0_Q[r];
            if (e > max_e0) {
                max_e0 = e;
                max_row0 = r;
            }
        }
```

**Offset 순회 (원문):**

```2344:2344:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
    for (int off = 0; off < 64; ++off) {
```

---

## === DISC-P4 Peak Threshold ===

**best vs second 갱신 (원문, `if (accum > best_e63)` 직후에는 전처리 분기로 길어짐 — **2699행 다음~2810행**은 소스 파일 직접 참조):**

```2695:2699:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
        sum_all += static_cast<int64_t>(accum);
        if (accum > best_e63) {
            second_e63 = best_e63;
            best_e63 = accum;
            best_off = off;
```

```2811:2813:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
        } else if (accum > second_e63) {
            second_e63 = accum;
        }
```

**`r_avg_ok` / `r_avg_high` / `sep_ok` / `e63_min` / `pass` (원문):**

```2816:2864:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
    const int64_t sum_others = sum_all - static_cast<int64_t>(best_e63);

    // r_avg >= 5: (best<<6)-best >= (so<<3)+(so<<1)+(so) = so*5...
    // 아니, 5 = (1<<2)+1: so*5 = (so<<2)+so
    // r_avg >= 5 ↔ best*63 >= so*5
    const int64_t best_x63 = (static_cast<int64_t>(best_e63) << 6) -
                              static_cast<int64_t>(best_e63);
    const int64_t so_x5    = (sum_others << 2) + sum_others;
    const bool r_avg_ok = (sum_others <= 0) || (best_x63 >= so_x5);

    // r_avg >= 8 판정 (sep_ok 활성화용, 곱셈 0)
    // best*63 >= so*8 ↔ best_x63 >= so<<3
    const bool r_avg_high = (sum_others <= 0) ||
        (best_x63 >= (sum_others << 3));

    // 적응형 sep_ok: r_avg≥8일 때만 alias 방어
    const int64_t best_x4 = static_cast<int64_t>(best_e63) << 2;
    const int64_t sec_x5  = (static_cast<int64_t>(second_e63) << 2) +
                             static_cast<int64_t>(second_e63);
    const bool sep_ok = (!r_avg_high) ||
        (second_e63 == 0) || (best_x4 > sec_x5);

    // 적응형 e63_min: noise_floor × 5 (하한 5000), amp×38 하한 (TPC tx_amp_ 연동)
    const int32_t avg_others =
        (sum_others > 0)
            ? static_cast<int32_t>((sum_others * 1040LL) >> 16)
            : 0;
    // avg_others * 5 = (avg<<2)+avg [SHIFT]
    const int32_t adaptive_min =
        (avg_others > 0)
            ? static_cast<int32_t>((static_cast<int64_t>(avg_others) << 2) +
                                    avg_others)
            : 5000;
    // 적응형: AMI amp×19 / PS-LTE amp×38 (shift 조합)
    const int32_t amp32 = static_cast<int32_t>(tx_amp_);
#if defined(HTS_TARGET_AMI)
    // AMI 32-chip × 4 non-coherent: clean peak 약 1/2 → amp × 19
    const int32_t k_E63_ALIGN_MIN =
        (amp32 << 4) + (amp32 << 1) + amp32;  // amp × 19
#else
    // PS-LTE 64-chip × 2 non-coherent: 기존 amp × 38
    const int32_t k_E63_ALIGN_MIN =
        (amp32 << 5) + (amp32 << 2) + (amp32 << 1);  // amp × 38
#endif
    const int32_t e63_min =
        (adaptive_min > k_E63_ALIGN_MIN) ? adaptive_min : k_E63_ALIGN_MIN;

    const bool pass = (best_off >= 0 && r_avg_ok &&
                       best_e63 >= e63_min && sep_ok);
```

**Fallback:** `pass`가 거짓이면 **P11** `else` 분기(버퍼 `memcpy` + `p0_chip_count_ = 64`).

---

## === DISC-P5 Offset Resolution ===

**best_off 타입:** `int` (지역 선언 `int best_off = -1;`).

**Step size:** `for (int off = 0; off < 64; ++off)` → **1 chip** 단위.

**Fine tuning (별도 fractional offset):** 본 함수 내 **확인 못함** (정수 `off`만 사용).

---

## === DISC-P6 Scan Range ===

**Offset loop (원문):**

```2344:2344:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
    for (int off = 0; off < 64; ++off) {
```

**최대 범위:** **off = 0 … 63** → **64 chip** 슬라이드.

**3블록 vs 2블록 (원문):**

```2416:2418:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
        // Stage 4b (PS-LTE): 3×64 FWHT 동일 row 일관성 (pre_reps_>1 시 PRE_SYM0 반복)
        // p0_buf128_*_[192]: off+191<192 일 때만 3블록 안전 → off==0 만 4b 전체.
        // 그 외 off 는 기존 2블록 XOR+fallback (슬라이드 OOB 방지).
```

```2471:2472:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
        if ((off + 128 + 63) < 192) {
```

---

## === DISC-P7 Alignment after Detection ===

**`best_off` 사용 (성공 시 시드·AGC 피크, 원문):**

```3068:3098:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
            for (int blk = 0; blk < 2; ++blk) {
                int32_t di = 0, dq = 0;
                walsh63_dot_(&p0_buf128_I_[best_off + (blk << 6)],
                             &p0_buf128_Q_[best_off + (blk << 6)],
                             di, dq);
                seed_dot_I += di;
                seed_dot_Q += dq;
            }
...
                for (int j = 0; j < 64; ++j) {
                    const int32_t ai = p0_buf128_I_[best_off + j];
                    const int32_t aq = p0_buf128_Q_[best_off + j];
```

**`psal_commit_align_` (carry 복사, 원문):**

```2141:2161:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
void HTS_V400_Dispatcher::psal_commit_align_() noexcept {
    psal_pending_ = false;
    const int commit_off = psal_off_;
    const int carry = 64 - commit_off;
    p0_carry_count_ = carry;
    if (carry > 0) {
        const int src = commit_off + 128;
        for (int j = 0; j < carry; ++j) {
            p0_carry_I_[j] = p0_buf128_I_[src + j];
            p0_carry_Q_[j] = p0_buf128_Q_[src + j];
        }
    }
    pre_phase_ = 1;
...
}
```

**호출 (원문):**

```3114:3116:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
        psal_off_ = best_off;
        psal_e63_ = best_e63;
        psal_commit_align_();
```

**Payload 시작 계산:** 본 스캔 블록만으로 **페이로드 절대 인덱스 산식은 확인 못함**. `pre_phase_ = 1` 및 Phase 1 주석은 `Feed_Chip` WAIT_SYNC 경로에 있음 (`HTS_V400_Dispatcher.cpp` 3252행대).

---

## === DISC-P8 Energy Normalization ===

**`p0_buf128_*`에 쓰이는 칩 경로 (WAIT_SYNC, `pre_phase_==0`, 원문):**

```3212:3218:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
    if (phase_ == RxPhase::WAIT_SYNC) {
        if (pre_phase_ == 0) {
            p0_buf128_I_[p0_chip_count_] = chip_I;
            p0_buf128_Q_[p0_chip_count_] = chip_Q;
            ++p0_chip_count_;
            if (p0_chip_count_ < 192)
                return;
```

직전 `Feed_Chip`에서 **`chip_I`/`chip_Q`는 DC 제거·`pre_agc_.Apply`·`apply_holo_lpi_inverse_rx_chip_` 이후** (`3197–3200`행대).

**정리 (코드 순서 한정):**

- **입력 버퍼 origin:** **AGC(및 홀로 역보정) 후** `int16_t` 칩.
- **Normalization:** FWHT 후 에너지에 **`>>16` / `>>14` 등 시프트 스케일**이 **`accum` 계산에 포함** (예: 2블록 XOR 경로 `e_sum >> 16`, 3블록 `e_scaled >> 16`, 8×8 경로 `>>14` 등 — **별도 float 정규화 없음**).
- **`gain_shift_`를 에너지 적분에 곱하는 코드는 `phase0_scan_` 본문에서 확인 못함** (`pre_agc_.Set_From_Peak`은 스캔 **성공 후** 호출).

---

## === DISC-P9 CFO vs Preamble ===

**`phase0_scan_` 내부에 CFO 추정 함수 호출:** **확인 못함**.

**스캔 성공 직후 (원문):**

```3078:3084:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
            est_I_ = seed_dot_I;
            est_Q_ = seed_Q_;
            est_count_ = 2;
            // B-2: est seed 확정 직후 mag/recip 즉시 계산
            //   이후 on_sym_ 심볼당 정규화 derotation 사용
            update_derot_shift_from_est_();
            // CFO 는 Walsh 도메인에서 처리 예정 — MCE 철거 (Stage 1)
```

**순서 (이 블록 한정):**

1. `walsh63_dot_`로 `seed_dot_I/Q` 합산  
2. `est_I_` / `est_Q_` 대입  
3. `update_derot_shift_from_est_()` 호출  
4. `pre_agc_.Set_From_Peak(peak_avg)` (`p0_buf128`의 **best_off 구간** 피크 평균)

---

## === DISC-P10 3-block scan ===

**`full3blk_dom` 조건 (원문):**

```2714:2716:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
            const bool full3blk_dom = ((off + 128 + 63) < 192);
            int best_dom_pick = (r0 == r1 && r1 == r2) ? r0 : r2;
            if (full3blk_dom) {
```

**T0/T1/T2:** 각각 `p0_buf128_*[off + 0..63]`, `[off+64..]`, (3블록 시) `[off+128..]`를 `T0_*` / `T1_*` / `T2_*`에 복사 후 **`fwht_64_complex_inplace_`** (`2431–2476`행대).

**Fix A3 관계 (주석 원문):**

```2710:2713:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
            // Fix A1 (V28): 3-way 일치 시 r0, 아니면 r2.
            // Fix A3 (V34): block-2 FWHT 에너지에서 coherent pair 순차 excision
            // (bin63 하드코딩 없음). pair1=(top1,top2) → pair2=(top3,top4) 검사,
            // 2-level jam이면 top5, 1-level이면 top3를 preamble 후보로 채택.
```

---

## === DISC-P11 Preamble Failure ===

**검출 실패 조건 (원문):**

```2863:2864:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
    const bool pass = (best_off >= 0 && r_avg_ok &&
                       best_e63 >= e63_min && sep_ok);
```

**재스캔:** **`phase0_scan_` 내부에서 `off` 루프 재시도는 없음** — 한 호출에서 0..63 전부 평가.

**실패 시 처리 (원문):**

```3117:3138:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
    } else {
#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
        {
            static int s_shift_count = 0;
            ++s_shift_count;
            if (s_shift_count <= 3) {
                std::printf(
                    "[DIAG-SHIFT] count=%d before_shift buf[128..143]_I:",
                    s_shift_count);
                for (int i = 0; i < 16; ++i) {
                    std::printf(" %d",
                                static_cast<int>(p0_buf128_I_[128 + i]));
                }
                std::printf("\n");
            }
        }
#endif
        std::memcpy(p0_buf128_I_, p0_buf128_I_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        std::memcpy(p0_buf128_Q_, p0_buf128_Q_ + 128,
                    static_cast<size_t>(64) * sizeof(int16_t));
        p0_chip_count_ = 64;
    }
```

**실패 카운터 (DIAG·`HTS_PHASE0_WALSH_BANK` 전용, 원문):**

```2887:2898:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
#if defined(HTS_DIAG_PRINTF) && defined(HTS_PHASE0_WALSH_BANK)
    // ─── DIAG-SCAN-STAT: scan pass/fail 누적 (로직 무변경) ───
    {
        static int s_scan_total = 0;
        static int s_scan_pass = 0;
        static int s_scan_fail = 0;
        ++s_scan_total;
        if (pass) {
            ++s_scan_pass;
        } else {
            ++s_scan_fail;
        }
```

**Timeout:** 본 함수 내 **`timeout` 명칭의 타이머는 확인 못함**.

---

## === DISC-P12 TX Preamble Generation (T6 SIM) ===

**TX 측:** `build_tx` → `setup` + `Build_Packet` (`HTS_T6_SIM_Test.cpp`).

**`setup` (원문):**

```150:157:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
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

**`kPreReps` / `kPreBoost` / `kAmp` (원문):**

```53:55:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
static constexpr int16_t  kAmp       = 1000;
static constexpr int      kPreReps   = 4;
static constexpr int      kPreBoost  = 1;
```

**`Build_Packet` 호출 (원문):**

```170:176:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
    setup(tx, ds);
    fill_info(ds, t, pkt.info);
    pkt.n = tx.Build_Packet(PayloadMode::DATA, pkt.info, 8, kAmp,
                            pkt.I, pkt.Q, kMaxC);
```

**채널 적용:** `feed_raw_ext`는 **`Build_Packet` 결과 칩열에 대해** `Feed_Chip(rxI[i],rxQ[i])`만 수행 — **채널 모델은 시나리오별로 `rxI`/`rxQ` 생성 측에 있음**(본 Discovery 범위 밖이면 **확인 못함**).

**RX 측 기대 오차:** **코드에서 “클린 채널 = 0 chip”을 명시한 구문은 확인 못함.** (사용자 다음 단계 지시서의 **측정 가정**: 클린 시나리오에서 `best_off` 분포·`expected_off` 대비 편차 조사.)

---

## 자진 고지

- **`HTS_PHASE0_WALSH_BANK` / `HTS_TARGET_AMI`:** `phase0_scan_`에 **별도 분기 블록이 존재**하나, 본 문서는 **PS-LTE 기본 경로 위주**로 인용함.
- **Phase 1 이후 페이로드 인덱스:** `psal_commit_align_` 이후 `Feed_Chip` Phase 1 로직은 **추가 인용 없음** — 필요 시 `WAIT_SYNC` `pre_phase_==1` 구간 전체 별도 인용 권장.
- **Pluto SDR T2/T6 이력:** 저장소 외부 메모 — **본 파일에 근거 없음**.

# Stage A Discovery Results

조사 범위: `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM` 중심 (읽기 전용).  
날짜 기준: 사용자 정보 2026-04-19.

---

## DISC-1: FEC Decoder Files

**File 1:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Polar_Codec.h`  
**Class:** `ProtectedEngine::HTS_Polar_Codec`

원문 시그니처:

```47:56:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Polar_Codec.h
    [[nodiscard]] static bool Decode_SC(const int16_t *llr, uint8_t *out,
                                        int *out_len) noexcept;
    // ── SCL-8 디코딩 ─────────────────────────────────────────
    /// @brief SCL-8 CRC-Aided 디코더
    /// @param llr     입력 LLR (int16_t[512])
    /// @param out     복원된 8바이트 데이터
    /// @param out_len 유효 바이트 수
    /// @return CRC 통과 경로 존재 시 true
    [[nodiscard]] static bool Decode_SCL(const int16_t *llr, uint8_t *out,
                                         int *out_len) noexcept;
```

- **Decode_SC `llr`:** `const int16_t*`, 주석상 `int16_t[512]`, “양수=0 쪽, 음수=1 쪽”
- **`out`:** `uint8_t*`, 복원 데이터
- **`out_len`:** `int*`, 유효 바이트 수

구현 파일: `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Polar_Codec.cpp` (`Decode_SC` / `Decode_SCL` 본문).

---

**File 2:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.hpp` — `ProtectedEngine::FEC_HARQ`  
Conv + REP + (Polar 켜짐 시) Polar 경로는 **단일 TU `HTS_FEC_HARQ.cpp`**의 `Decode_Core` / `Viterbi_Decode` 등 비공개 멤버로 묶여 있으며, 외부에서 직접 호출되는 디코드 진입점 예시:

```371:378:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.hpp
        [[nodiscard]] static bool Decode16(const RxState16& s,
            uint8_t* out, int* len, uint32_t il, WorkBuf& wb) noexcept;
        [[nodiscard]] static bool Decode64(const RxState64& s,
            uint8_t* out, int* len, uint32_t il, WorkBuf& wb) noexcept;

        // [적응형] Decode — bps 지정
        [[nodiscard]] static bool Decode64_A(const RxState64& s,
            uint8_t* out, int* len, uint32_t il, int bps,
            WorkBuf& wb) noexcept;
```

IR 경로:

```279:286:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.hpp
        [[nodiscard]] static bool Decode64_IR(
            const int16_t* sym_I, const int16_t* sym_Q,
            int nsym, int nc, int bps,
            uint32_t il_seed, int rv,
            IR_RxState& ir_state,
            uint8_t* out, int* olen,
            WorkBuf& wb,
            uint8_t walsh_shift = 0u) noexcept;
```

구현 파일: `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.cpp`.

---

## DISC-2: LLR Data Type

**Polar `Decode_SCL` 입력:** `const int16_t *llr` — 코드워드 **N=512**개 LLR (비트당 1 LLR).

**문서화된 부호 규칙 (Polar 헤더):**

```42:44:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Polar_Codec.h
    /// @brief SC 디코더: LLR 512개 → 80비트 정보 복원
    /// @param llr     입력 LLR (int16_t[512], 양수=0 쪽, 음수=1 쪽)
```

**FEC IR 경로에서 Polar 직전 `int16_t` 버퍼:** `Decode64_IR` 내에서 `polar_llr16`에 대입 후 `Decode_SCL` 호출 (원문은 DISC-5 인용 구간 참고). 클램프 대상은 **±32767** (`m_hi`/`m_lo`).

**심볼→비트 LLR (내부 `Bin_To_LLR`):** 출력 배열은 호출부 `int32_t* llr` (심볼당 `bps`개). Polar 경로에서는 `raw`를 `>> 10` 후 `tpe_clamp_llr`로 **±500000** 클램프.

원문 (계산 + 정규화):

```387:440:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.cpp
void FEC_HARQ::Bin_To_LLR(const int32_t *fI, const int32_t *fQ, int nc, int bps,
                          int32_t *llr) noexcept {
    const int nsym = 1 << bps;
    const int32_t ns_lt =
        static_cast<int32_t>(0u - static_cast<uint32_t>(nsym < nc));
    const int valid = (nsym & ns_lt) | (nc & ~ns_lt);
    int32_t raw[BPS64_MAX]{};
    for (int b = 0; b < bps; ++b) {
        const int sh_bit = bps - 1 - b;
#if defined(HTS_FEC_POLAR_ENABLE)
        // Max-Log-MAP: max(corr|bit=0) - max(corr|bit=1) — Sum 대비 잡음 빈 누적 완화
        int32_t pos_max = INT32_MIN;
        int32_t neg_max = INT32_MIN;
        for (int m = 0; m < valid; ++m) {
            const int32_t corr = fI[m] + fQ[m];
            ...
        }
        raw[b] = pos_max - neg_max;
#else
        ...
#endif
    }
    ...
    for (int b = 0; b < bps; ++b) {
#if defined(HTS_FEC_POLAR_ENABLE)
        llr[b] = raw[b] >> 10;
#else
        llr[b] = raw[b] >> 12;
#endif
    }
}
```

**`tpe_clamp_llr` 범위 (원문):**

```172:180:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.cpp
//  tpe_clamp_llr: LLR 단일 값 클램프 (Decode_Core·IR 경로에서 사용)
static inline int32_t tpe_clamp_llr(int32_t v) noexcept {
    const int32_t lim = 500000;
    const int32_t m_pos = -static_cast<int32_t>(v > lim);
    const int32_t m_neg = -static_cast<int32_t>(v < -lim);
    return (lim & m_pos) | (-lim & m_neg) | (v & ~(m_pos | m_neg));
}
```

**LLRs per symbol:** `bps` (64칩 심볼당 `bps`개 비트 LLR이 `TOTAL_CODED` 그리드에 매핑됨) — 루프 원문:

```1071:1078:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.cpp
        for (int b = 0; b < bps; ++b) {
            const int bi = sym * bps + b;
            const uint32_t in_range =
                0u - static_cast<uint32_t>(bi < TOTAL_CODED);
            wb.ru.all_llr[static_cast<std::size_t>(bi)] =
                tpe_clamp_llr(llr[static_cast<std::size_t>(b)]) &
                static_cast<int32_t>(in_range);
        }
```

---

## DISC-3: FWHT API

### (A) 디스패처 파일 범위 — 64점 in-place 복소 FWHT

**함수명:** `fwht_64_complex_inplace_`  
**in-place:** 예 (`T_I`/`T_Q` 동일 배열 갱신).  
**64-point 전용:** 루프 상한 `stride < 64` 고정.  
**파일:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp`

```102:120:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
/// @brief Fast Walsh-Hadamard Transform 64-point (complex I/Q)
/// @details radix-2 butterfly, in-place, 6 stages (Baseline Stage 2 + WBANK)
/// @note Hadamard natural ordering (row idx = sequency)
static inline void fwht_64_complex_inplace_(int32_t* __restrict T_I,
                                            int32_t* __restrict T_Q) noexcept {
    for (int stride = 1; stride < 64; stride <<= 1) {
        for (int i = 0; i < 64; i += (stride << 1)) {
            for (int j = 0; j < stride; ++j) {
                const int a = i + j;
                const int b = a + stride;
                const int32_t tI = T_I[a];
                const int32_t tQ = T_Q[a];
                T_I[a] = tI + T_I[b];
                T_Q[a] = tQ + T_Q[b];
                T_I[b] = tI - T_I[b];
                T_Q[b] = tQ - T_Q[b];
            }
        }
    }
}
```

**사용 예 (Wait_sync 페이로드 전 동기 구간, `T_I`/`T_Q` int32_t):**

```3270:3276:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
        alignas(4) int32_t T_I[64];
        alignas(4) int32_t T_Q[64];
        for (int j = 0; j < 64; ++j) {
            T_I[j] = static_cast<int32_t>(orig_I_[j]);
            T_Q[j] = static_cast<int32_t>(orig_Q_[j]);
        }
        fwht_64_complex_inplace_(T_I, T_Q);
```

### (B) FEC 파일 범위 — `FEC_HARQ::FWHT(int32_t* d, int n)` , n=16 또는 64 전개

**in-place:** `d` 버퍼를 나비 연산으로 갱신.  
**가변 길이:** `n == 16` / `n == 64` 는 전개 경로; 그 외 `n` 은 일반 루프 분기(동일 함수 내).  
**파일:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.cpp`

```282:294:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.cpp
void FEC_HARQ::FWHT(int32_t *d, int n) noexcept {
    if (d == nullptr || n <= 1) {
        return;
    }
    if (n == 16) {
        FWHT_Unroll16(d);
        return;
    }
    if (n == 64) {
        FWHT_Unroll64(d);
        return;
    }
```

**`Decode64_IR` 내부 호출 예:**

```1039:1040:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.cpp
        FWHT(fI.data(), nc);
        FWHT(fQ.data(), nc);
```

---

## DISC-4: AJC Fix A3

**확인 못함:** 별도 “AJC 클래스”의 Fix A3 — `HTS_V400_Dispatcher.cpp` 주석은 **“[REMOVED Step3] … AntiJam 엔진 제거”** 로 별도 `ajc_.Process` 호출은 제거된 상태.

**감싸는 함수 (Fix A3 블록이 들어 있는 멤버 함수):**

```2143:2145:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
void HTS_V400_Dispatcher::phase0_scan_() noexcept {
    int32_t best_e63 = 0, second_e63 = 0;
    int best_off = -1;
```

**Excision 보조(에너지 배열에서 최댓값 bin):** 별도 파일이 아니라 동일 `.cpp` 의 `find_argmax_64` (Fix A3 주석 바로 위에 정의).

```123:137:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
/// @brief 64-bin 에너지 배열 argmax — Fix A3 Walsh-row excision
static inline void find_argmax_64(const int64_t* __restrict e,
                                  int* __restrict out_bin,
                                  int64_t* __restrict out_e) noexcept {
    int best_bin = 0;
    int64_t best_e = e[0];
    for (int m = 1; m < 64; ++m) {
        if (e[m] > best_e) {
            best_e = e[m];
            best_bin = m;
        }
    }
    *out_bin = best_bin;
    *out_e = best_e;
}
```

**Fix A3 (V34) 동기·에너지 excision 구현 위치:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp` (주석 `[FIXA3]` / `e_b2` 배열).

**Top-2 / zero-out 대응 원문 (발췌):**

```2693:2758:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
            const bool full3blk_dom = ((off + 128 + 63) < 192);
            ...
                int64_t e_b2[64];
                for (int m = 0; m < 64; ++m) {
                    e_b2[m] = static_cast<int64_t>(T2_I[m]) * T2_I[m] +
                              static_cast<int64_t>(T2_Q[m]) * T2_Q[m];
                }
                int t1_bin = 0;
                int64_t t1_e = 0;
                find_argmax_64(e_b2, &t1_bin, &t1_e);
                ...
                if (t1_e > 0) {
                    const int64_t t1_saved = e_b2[t1_bin];
                    e_b2[t1_bin] = 0;
                    find_argmax_64(e_b2, &t2_bin, &t2_e);
                    ...
                        const int64_t t2_saved = e_b2[t2_bin];
                        e_b2[t2_bin] = 0;
                        ...
                        const int64_t t3_saved = e_b2[t3_bin];
                        e_b2[t3_bin] = 0;
                        ...
                            e_b2[t4_bin] = 0;
                        ...
                            e_b2[t3_bin] = t3_saved;
                            e_b2[t2_bin] = t2_saved;
                    ...
                    e_b2[t1_bin] = t1_saved;
                }
```

**호출 위치 (Dispatcher):** `Feed_Chip` → `WAIT_SYNC` 분기에서 `phase0_scan_()` 호출.

```3225:3226:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
            phase0_scan_();
            return;
```

**호출 맥락:** `phase0_scan_()` 내부 `off` 루프·`T2_I` FWHT 이후 블록. `T2_I` 채움·FWHT:

```2451:2455:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
            for (int i = 0; i < 64; ++i) {
                T2_I[i] = static_cast<int32_t>(p0_buf128_I_[off + 128 + i]);
                T2_Q[i] = static_cast<int32_t>(p0_buf128_Q_[off + 128 + i]);
            }
            fwht_64_complex_inplace_(T2_I, T2_Q);
```

---

## DISC-5: IR-HARQ Soft Combining

**파일:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.cpp`  
**헤더 상태 타입:** `HTS_FEC_HARQ.hpp` 의 `IR_RxState` (`llr_accum[TOTAL_CODED]`).

**핵심 함수 (이름만):** `FEC_HARQ::Decode64_IR`, `FEC_HARQ::IR_Init` (헤더 선언), Polar 분기 내 누적/폴드/디코드.

**Soft combining 핵심 원문 (688→512 슬롯 링 누적 + 스케일 + int16 Polar 입력):**

```1156:1202:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.cpp
    std::memset(static_cast<void *>(ir_state.llr_accum), 0,
                static_cast<std::size_t>(POLAR_N) * sizeof(int32_t));
    static_assert((static_cast<unsigned>(POLAR_N) &
                    (static_cast<unsigned>(POLAR_N) - 1u)) == 0u,
                  "POLAR_N must be power of 2 for AND mask");
    for (int i = 0; i < TOTAL_CODED; ++i) {
        ir_state.llr_accum[static_cast<std::size_t>(
            i & (POLAR_N - 1))] += polar_llr_coded[i];
    }

    int32_t max_abs = 1;
    for (int i = 0; i < POLAR_N; ++i) {
        int32_t a = ir_state.llr_accum[static_cast<std::size_t>(i)];
        ...
    }
    ...
    for (int i = 0; i < POLAR_N; ++i) {
        const int32_t v =
            ir_state.llr_accum[static_cast<std::size_t>(i)] >> shift;
        ...
        polar_llr16[static_cast<std::size_t>(i)] =
            static_cast<int16_t>(clamped);
    }
    ...
    bool dec_ok =
        HTS_Polar_Codec::Decode_SCL(polar_llr16, rx, &olen_p);
```

**`wb_` (WorkBuf) 선언 및 크기 주석:**

```217:229:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.hpp
        /// @warning sizeof(WorkBuf) ≈ 13KB — ARM에서 전역 또는 정적 배치 권장.
        /// alignas(32): 워드 블록·캐시 라인 정렬(필드 순서는 인코더/비터비 경로 고정)
        struct alignas(32) WorkBuf {
            int32_t  pm[2][64];
            uint8_t  surv[VIT_STEPS][64];       // Viterbi 경로 256→88
            uint8_t  tb[VIT_STEPS];             // traceback 256→88
            uint16_t perm[TOTAL_CODED];       // 순열 인덱스 1024→688
            int32_t  tmp_soft[TOTAL_CODED];     // 소프트 메트릭 1024→688
            union {
                uint8_t  rep[TOTAL_CODED];      // Encode 경로 전용
                int32_t  all_llr[TOTAL_CODED];  // Decode 경로 (REP in-place 후 앞 CONV_OUT 슬롯이 soft)
            } ru;
        };
```

**Dispatcher `wb_` 및 BUG-FIX-IR5:**

```422:424:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.hpp
        FEC_HARQ::WorkBuf wb_{};
```

```1421:1425:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
void HTS_V400_Dispatcher::try_decode_() noexcept {
    // [BUG-FIX-IR5] wb_ 초기화: Encode 잔류 데이터가 Decode 경로를 오염시키는
    // 것을 방지. 16칩(NSYM16=172)은 wb_ 사용 영역이 작아 영향 없으나,
    // 64칩(NSYM64=172, BPS=4)은 전체 TOTAL_CODED(688) 슬롯을 사용하므로 필수.
    std::memset(&wb_, 0, sizeof(wb_));
```

**Retry 상한 (`max_harq_`):**

```3842:3844:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
                    max_harq_ = FEC_HARQ::DATA_K; // 모든 모드에서 32
```

(`DATA_K` 정의: `HTS_FEC_HARQ.hpp` 에 `static constexpr int DATA_K = 800;` — 주석과 대조 시 별도 검증 필요. **실제 대입 원문은 위 한 줄만 인용**.)

---

## DISC-6: Dispatcher demodulate / receive flow

**진입점 (칩 단위):** `HTS_V400_Dispatcher::Feed_Chip(int16_t rx_I, int16_t rx_Q)` — `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp`

**요약 체인 (원문에 근거한 순서, 일부 단계는 주석상 NOP/제거):**

1. **`Feed_Chip`** — DC 제거, `pre_agc_.Apply`, `apply_holo_lpi_inverse_rx_chip_` 등 (원문 3152–3176 부근).
2. **`phase_ == WAIT_SYNC`** — 버퍼링 후 **`phase0_scan_()`** 호출 (원문 3225–3226).
3. **64칩 `buf_I_`/`buf_Q_` 완성 후** — `orig_I_`/`orig_Q_` 복사, AJC/클립 경로는 주석상 제거 (3262–3265).
4. **`on_sym_()`** — Walsh permuter 적용, IR이면 `ir_chip_I_`/`ir_chip_Q_`에 기록, 아니면 `FEC_HARQ::Feed*` 누적 (1230–1414).
5. **`pay_recv_ >= pay_total_` 일 때 `try_decode_()`** (1417–1419).

```1417:1419:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
    buf_idx_ = 0;
    if (pay_recv_ >= pay_total_)
        try_decode_();
```

**버퍼 흐름 (원문 표현):** `on_sym_` 주석 — IR 시 칩 도메인 `+=` 금지, 결합은 `Decode*_IR` 의 `ir_state_(LLR)`:

```1243:1245:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
        /* IR-HARQ: RV마다 송신 파형이 다르므로 칩 도메인 += 금지.
           결합은 FEC Decode*_IR 의 ir_state_(LLR)에서만 수행; 수신 칩은 매 심볼
           덮어쓰기. */
```

---

## DISC-7: Row energy calculation

**파일:** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp`  
**에너지 타입:** `int64_t` (`e_b2[m]`, `max_e0` 등).

**핵심 원문 (Stage 4b, T0 블록 예):**

```2419:2431:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.cpp
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
```

**Overflow / saturation:** 본 루프에서는 `int64_t` 곱으로 표기; **uint32 별도 포화 처리 문구는 이 인용 구간에 없음** → “이 구간 원문 기준 explicit uint32 saturation 없음”.

**다른 row-energy (8칩 FWHT row7 누적):** `energy_8x8_noncoh_row7_` — 반환 `int64_t`, 내부에서 `int32_t` FWHT 후 `int64_t` 제곱합 (원문 158–174).

---

## DISC-8: Test harness (11200 / 10927)

**Entry file (T6 SIM V3):** `D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp`

**합산 pass 카운터:** `main()` 내 `grand_pass`, `grand_total`.

```813:841:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
    int grand_pass = 0, grand_total = 0;
    long long grand_bits = 0;
    ...
        grand_pass += r.pass;
        grand_total += r.total;
        grand_bits += r.total_bit_errors;
    ...
    std::printf("║  정량 합계: %6d / %6d (%5.1f%%) — 전체 BER: %7.5f        ║\n",
                grand_pass, grand_total,
                (grand_total > 0) ? 100.0 * grand_pass / grand_total : 0.0,
                grand_ber);
```

**시나리오 라벨 (원문 주석 예):**

```379:382:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
//  S1: 완벽한 무결성 (클린 채널)
static void test_S1() {
    hdr("S1", "완벽한 무결성 (클린 채널)");
```

`test_S2`…`test_S10` 동일 파일에 정의.

**Output format sample (원문 10줄, `main` 종합 보고 `printf` 서식):**

```807:816:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
    std::printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    std::printf("║  종합 보고서 (Pass + CRC+payload BER)                         ║\n");
    std::printf("╠═════════╤══════════════════╤═══════╤═══════╤════════╤═══════╣\n");
    std::printf("║ 시나리오│ 조건             │ Pass  │ CRC+  │  BER   │ 판정  ║\n");
    std::printf("╠═════════╪══════════════════╪═══════╪═══════╪════════╪═══════╣\n");

    int grand_pass = 0, grand_total = 0;
    long long grand_bits = 0;
    int cat_pass = 0, cat_total = 0;
    for (int i = 0; i < g_n_results; ++i) {
```

```833:849:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_TEST\t6_sim\HTS_T6_SIM_Test.cpp
    std::printf("╠═════════╧══════════════════╧═══════╧═══════╧════════╧═══════╣\n");
    const double grand_ber = (grand_total > 0)
        ? static_cast<double>(grand_bits) /
          (static_cast<double>(grand_total) * 64.0)
        : 0.0;
    std::printf("║  정량 합계: %6d / %6d (%5.1f%%) — 전체 BER: %7.5f        ║\n",
                grand_pass, grand_total,
                (grand_total > 0) ? 100.0 * grand_pass / grand_total : 0.0,
                grand_ber);
    std::printf("║  범주 PASS: %2d / %2d                                           ║\n",
                cat_pass, cat_total);
    std::printf("║  총 bit errors: %lld / %lld                                   ║\n",
                static_cast<long long>(grand_bits),
                static_cast<long long>(grand_total) * 64LL);
    std::printf("║  총 소요:   %.1fs                                              ║\n",
                total_sec);
    std::printf("╚═══════════════════════════════════════════════════════════════╝\n");
```

**`11200` / `10927` 리터럴:** 본 `HTS_T6_SIM_Test.cpp` 안에서는 **검색되지 않음** (합계는 `g_results` 누적 행의 `total` 합).  
`10927/11200` 문구는 문서 `RESULT_PROMPT_47_PHASE1_v1.1.md` 등에 **다른 실행 파일명**과 함께 등장 — **해당 `.exe` 소스 경로는 본 워크스페이스 조사에서 확인 못함**.

---

## DISC-9: KCMVP — FEC dependency

**직접 연결:** `HTS_LIM\HTS_TEST\KCMVP_암호_4종_종합_테스트.cpp` 에서 `FEC` / `Polar` / `HARQ` 문자열 grep 결과 **0건** (본 조사 범위).

**FEC 소스 내 `KCMVP` 문자열:** `HTS_LIM\HTS_LIM\HTS_FEC_HARQ.cpp` / `HTS_FEC_HARQ.hpp` grep 결과 **0건**.

**Constant-time 요구가 Polar 디코더 주석에 명시되는지:** `HTS_Polar_Codec.cpp` 의 SC/SCL 루프에 “branchless TPE” 등 표현은 있으나,**“KCMVP” 명시 문구는 확인 못함**.

---

## DISC-10: SRAM budget

**원문 근거 (디스패처 헤더 주석 — 설계 주장, 빌드 산출물과 별도):**

```49:52:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_V400_Dispatcher.hpp
//  [최종 메모리 배치]
//   SRAM1+2 (128KB): harq_I_(58KB) + wb_(15KB) + orig_acc_(29KB) + etc
//   CCM     (64KB):  harq_Q_(58KB) + MSP 스택(4KB)
//   총 사용 ~178KB / 192KB (14.3KB = 7.4% 마진)
```

**`WorkBuf` 빌드 타임 상한:**

```442:443:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_FEC_HARQ.hpp
    static_assert(sizeof(FEC_HARQ::WorkBuf) <= 14000u,
        "WorkBuf exceeds 14KB — rep|all_llr union·in-place REP 재검토");
```

**`IMPL_BUF_SIZE` (다른 모듈 예 — Walsh permuter):**

```109:120:D:\HTS_ARM11_Firmware\HTS_LIM\HTS_LIM\HTS_Walsh_Row_Permuter.hpp
    // 빌드타임 보증: static_assert(sizeof(Impl) <= IMPL_BUF_SIZE)
    static constexpr size_t IMPL_BUF_SIZE = 64u;
    ...
    alignas(IMPL_BUF_ALIGN) uint8_t impl_buf_[IMPL_BUF_SIZE]{};
```

**확인 못함:** 사용자 예시에 나온 심볼명 `spread_tensor[12288]`, `bit_votes[8192]`, `matrix[4096]` 등 — **해당 이름의 전역 배열은 본 검색에서 발견되지 않음** (`HTS_3D_Tensor_FEC.cpp` 는 `std::vector<double> tensor` 등 **동적** 표현이 있음).

**Total static / Remaining / optional IR LLR scaling margin:** 전 펌웨어 `.map` 없이 **수치 합계 산출은 확인 못함** (주석 수치만 원문 인용).

---

## 자진 고지

- **확인 못함:** `10927/11200`을 산출하는 **V34_FIXA3 전용** 소스 파일 경로; 펌웨어 전역 static 합산·DMA 버퍼 총량의 **링커 기반 검증**; KCMVP 인증 문서와 FEC의 **정식 요구사항 매핑**.
- **추가 발견:** FWHT가 **디스패처(`fwht_64_complex_inplace_`)** 와 **FEC(`FEC_HARQ::FWHT`)** 에 **이중** 존재.
- **의심/주의:** `max_harq_ = FEC_HARQ::DATA_K` 주석(“32”)과 `DATA_K=800` 정의의 **문면 불일치** — 의미 해석 없이 원문만 제시.

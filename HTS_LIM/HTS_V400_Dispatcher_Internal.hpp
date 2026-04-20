#ifndef HTS_V400_DISPATCHER_INTERNAL_HPP
#define HTS_V400_DISPATCHER_INTERNAL_HPP

#include "HTS_V400_Dispatcher.hpp"
#include <climits>
#include <cstdint>
#include <cstddef>

#if !defined(HTS_TARGET_AMI) && !defined(HTS_TARGET_PSLTE)
#  define HTS_TARGET_PSLTE
#endif
#if defined(HTS_TARGET_AMI) && defined(HTS_TARGET_PSLTE)
#  error "HTS_TARGET_AMI 와 HTS_TARGET_PSLTE 는 동시 정의 불가"
#endif
#if defined(HTS_TARGET_AMI)
#  error "HTS_TARGET_AMI 32-chip 경로는 2026-04-18 T6 붕괴 (43/11200) 로 비활성. 원인 규명 전 빌드 금지."
#endif

namespace ProtectedEngine {

constexpr uint8_t k_W63_FWHT_ROW_NATURAL = 63u;

#if defined(HTS_PHASE0_WALSH_BANK)
extern uint8_t k_W63_FWHT_ROW;
#endif

struct V400HarqCcmChase {
    int32_t harq_Q[FEC_HARQ::NSYM64][FEC_HARQ::C64];
};
struct V400HarqCcmIr {
    int16_t chip_I[FEC_HARQ::NSYM64][FEC_HARQ::C64];
    int16_t chip_Q[FEC_HARQ::NSYM64][FEC_HARQ::C64];
    FEC_HARQ::IR_RxState ir_state;
};
union V400HarqCcm {
    V400HarqCcmChase chase;
    V400HarqCcmIr ir;
};

alignas(64) extern uint8_t g_v400_sym_scratch[FEC_HARQ::NSYM64];
alignas(64) extern HTS_CCM_SECTION V400HarqCcm g_harq_ccm_union;
alignas(64) extern uint64_t g_sic_bits[FEC_HARQ::NSYM64];

extern const int16_t k_walsh_dummy_iq_[64];

extern "C" {
extern volatile int g_hts_ir_diag_chip0;
extern volatile int g_hts_ir_diag_feed_idx;
}

namespace detail {
constexpr uint32_t popc32(uint32_t x) noexcept {
    x = x - ((x >> 1u) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2u) & 0x33333333u);
    return (((x + (x >> 4u)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24u;
}
constexpr uint32_t fast_abs(int32_t x) noexcept {
    const uint32_t m = 0u - static_cast<uint32_t>(x < 0);
    return (static_cast<uint32_t>(x) ^ m) + m;
}
/// @brief Walsh-Hadamard 행렬 H_64의 63번 행.
///        w63[j] = (-1)^popcount(j).  ROM 배치.
inline constexpr int8_t k_w63[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1,
    +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1,
    +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1
};

/// @brief Fast Walsh-Hadamard Transform 64-point (complex I/Q)
/// @details radix-2 butterfly, in-place, 6 stages (Baseline Stage 2 + WBANK)
/// @note Hadamard natural ordering (row idx = sequency)
inline void fwht_64_complex_inplace_(int32_t* __restrict T_I,
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

/// @brief 64-bin 에너지 배열 argmax — Fix A3 Walsh-row excision
inline void find_argmax_64(const int64_t* __restrict e,
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

/// @brief 8-point FWHT (복소 I/Q, in-place, 3 stage)
inline void fwht_8_complex_inplace_(int32_t* T_I,
                                            int32_t* T_Q) noexcept {
    for (int stride = 1; stride < 8; stride <<= 1) {
        for (int i = 0; i < 8; i += (stride << 1)) {
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

/// @brief 64-chip 블록의 8×8 non-coh 에너지 (각 8칩 FWHT row 7 누적)
/// @note k_w63 자기유사: 8칩 H8[7] 패턴 반복 → CFO 짧은 구간에서 row 7 누적
inline int64_t energy_8x8_noncoh_row7_(const int16_t* chip_I,
                                                const int16_t* chip_Q) noexcept {
    int64_t e_row7 = 0;
    for (int s = 0; s < 8; ++s) {
        alignas(32) int32_t t_I[8];
        alignas(32) int32_t t_Q[8];
        for (int i = 0; i < 8; ++i) {
            t_I[i] = static_cast<int32_t>(chip_I[s * 8 + i]);
            t_Q[i] = static_cast<int32_t>(chip_Q[s * 8 + i]);
        }
        fwht_8_complex_inplace_(t_I, t_Q);
        e_row7 += static_cast<int64_t>(t_I[7]) * t_I[7] +
                  static_cast<int64_t>(t_Q[7]) * t_Q[7];
    }
    return e_row7;
}
/// Walsh-63 bin dot (ADD/SUB만) — Phase 0 스캔용
inline void walsh63_dot_(const int16_t *chip_I, const int16_t *chip_Q,
                         int32_t &dot_I, int32_t &dot_Q) noexcept {
    int32_t dI = 0;
    int32_t dQ = 0;
    for (int j = 0; j < 64; ++j) {
        const int32_t sI = static_cast<int32_t>(chip_I[j]);
        const int32_t sQ = static_cast<int32_t>(chip_Q[j]);
        if (k_w63[static_cast<std::size_t>(j)] > 0) {
            dI += sI;
            dQ += sQ;
        } else {
            dI -= sI;
            dQ -= sQ;
        }
    }
    dot_I = dI;
    dot_Q = dQ;
}
inline void fwht_raw(int32_t *d, int n) noexcept {
    if (n == 64) {
        for (int i = 0; i < 64; i += 2) {
            int32_t u = d[i], v = d[i + 1];
            d[i] = u + v;
            d[i + 1] = u - v;
        }
        for (int i = 0; i < 64; i += 4) {
            int32_t u = d[i], v = d[i + 2];
            d[i] = u + v;
            d[i + 2] = u - v;
            u = d[i + 1];
            v = d[i + 3];
            d[i + 1] = u + v;
            d[i + 3] = u - v;
        }
        for (int i = 0; i < 64; i += 8) {
            for (int k = 0; k < 4; ++k) {
                int32_t u = d[i + k], v = d[i + 4 + k];
                d[i + k] = u + v;
                d[i + 4 + k] = u - v;
            }
        }
        for (int i = 0; i < 64; i += 16) {
            for (int k = 0; k < 8; ++k) {
                int32_t u = d[i + k], v = d[i + 8 + k];
                d[i + k] = u + v;
                d[i + 8 + k] = u - v;
            }
        }
        for (int i = 0; i < 64; i += 32) {
            for (int k = 0; k < 16; ++k) {
                int32_t u = d[i + k], v = d[i + 16 + k];
                d[i + k] = u + v;
                d[i + 16 + k] = u - v;
            }
        }
        for (int k = 0; k < 32; ++k) {
            int32_t u = d[k], v = d[k + 32];
            d[k] = u + v;
            d[k + 32] = u - v;
        }
        return;
    }
    if (n == 16) {
        for (int i = 0; i < 16; i += 2) {
            int32_t u = d[i], v = d[i + 1];
            d[i] = u + v;
            d[i + 1] = u - v;
        }
        for (int i = 0; i < 16; i += 4) {
            int32_t u = d[i], v = d[i + 2];
            d[i] = u + v;
            d[i + 2] = u - v;
            u = d[i + 1];
            v = d[i + 3];
            d[i + 1] = u + v;
            d[i + 3] = u - v;
        }
        for (int i = 0; i < 16; i += 8) {
            for (int k = 0; k < 4; ++k) {
                int32_t u = d[i + k], v = d[i + 4 + k];
                d[i + k] = u + v;
                d[i + 4 + k] = u - v;
            }
        }
        for (int k = 0; k < 8; ++k) {
            int32_t u = d[k], v = d[k + 8];
            d[k] = u + v;
            d[k + 8] = u - v;
        }
        return;
    }
    for (int len = 1; len < n; len <<= 1) {
        for (int i = 0; i < n; i += 2 * len) {
            for (int j = 0; j < len; ++j) {
                int32_t u = d[i + j], v = d[i + len + j];
                d[i + j] = u + v;
                d[i + len + j] = u - v;
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────
// [Step B] Walsh-row 시퀀스 매칭 프리앰블 검출 (v5, Python 검증)
// 원리: 프리앰블이 [W7, W23, W45, W63] 4개 Walsh row 시퀀스로 구성.
//   각 64-chip 블록 FWHT → 기대 row 의 complex 값 추출 → 블록 간 coherent sum
//   score = |Σ_b F_b[seq[b]]|²
// Python v25 실측 (변동 n=4000): 1D 89.1% → v5 91.2% (+2.1%p)
// 특히 Swept 22dB/Multi 17dB 완전 회복 (Phase C 61%→100%)
// ─────────────────────────────────────────────────────────────
#ifdef HTS_WALSH_V5_PREAMBLE

// 프리앰블 시퀀스 (Python 검증값)
inline constexpr uint8_t kV5_PreambleSeq[4] = {7u, 23u, 45u, 63u};

/// @brief v5 Walsh-row 시퀀스 매칭 score 계산 (단일 offset, 4×64 chip)
/// @note FWHT 는 기존 fwht_raw() 재사용 (ADD/SUB only, MUL-free)
inline int64_t walsh_v5_score_(const int16_t *buf_I,
                               const int16_t *buf_Q) noexcept {
    int32_t Z_I = 0;
    int32_t Z_Q = 0;
    for (int b = 0; b < 4; ++b) {
        const int16_t *blk_I = &buf_I[b * 64];
        const int16_t *blk_Q = &buf_Q[b * 64];
        alignas(16) int32_t t_I[64];
        alignas(16) int32_t t_Q[64];
        for (int i = 0; i < 64; ++i) {
            t_I[i] = static_cast<int32_t>(blk_I[i]);
            t_Q[i] = static_cast<int32_t>(blk_Q[i]);
        }
        fwht_raw(t_I, 64);
        fwht_raw(t_Q, 64);
        const uint8_t r = kV5_PreambleSeq[b];
        Z_I += t_I[r];
        Z_Q += t_Q[r];
    }
    const int64_t zi64 = static_cast<int64_t>(Z_I);
    const int64_t zq64 = static_cast<int64_t>(Z_Q);
    return zi64 * zi64 + zq64 * zq64;
}

/// @brief v5 2-블록 축소 버전 (p0_buf128_I_/Q_ 192 chip 윈도우 내 128 chip)
/// @note 실증용. 4블록 full 은 버퍼 256 chip 확장 후 사용.
inline int64_t walsh_v5_score_2blk_(const int16_t *buf_I,
                                    const int16_t *buf_Q) noexcept {
    constexpr uint8_t seq2[2] = {kV5_PreambleSeq[0], kV5_PreambleSeq[1]};
    int32_t Z_I = 0;
    int32_t Z_Q = 0;
    for (int b = 0; b < 2; ++b) {
        const int16_t *blk_I = &buf_I[b * 64];
        const int16_t *blk_Q = &buf_Q[b * 64];
        alignas(16) int32_t t_I[64];
        alignas(16) int32_t t_Q[64];
        for (int i = 0; i < 64; ++i) {
            t_I[i] = static_cast<int32_t>(blk_I[i]);
            t_Q[i] = static_cast<int32_t>(blk_Q[i]);
        }
        fwht_raw(t_I, 64);
        fwht_raw(t_Q, 64);
        const uint8_t r = seq2[b];
        Z_I += t_I[r];
        Z_Q += t_Q[r];
    }
    const int64_t zi64 = static_cast<int64_t>(Z_I);
    const int64_t zq64 = static_cast<int64_t>(Z_Q);
    return zi64 * zi64 + zq64 * zq64;
}

#endif  // HTS_WALSH_V5_PREAMBLE
inline void walsh_enc(uint8_t sym, int n, int16_t amp, int16_t *oI,
                      int16_t *oQ) noexcept {
    const int32_t ampi = static_cast<int32_t>(amp);
    for (int j = 0; j < n; ++j) {
        const uint32_t p =
            popc32(static_cast<uint32_t>(sym) & static_cast<uint32_t>(j)) & 1u;
        const int16_t ch =
            static_cast<int16_t>(ampi * (1 - 2 * static_cast<int32_t>(p)));
        oI[j] = ch;
        oQ[j] = ch;
    }
}
// ── [적응형 I/Q] I/Q 독립 모드 (평시 2배 처리량) ──
//  sym_I → I 채널, sym_Q → Q 채널에 독립 Walsh 인코딩
//  처리량 2배: 동일 칩 수로 2개 심볼 전송
inline void walsh_enc_split(uint8_t sym_I, uint8_t sym_Q, int n, int16_t amp,
                            int16_t *oI, int16_t *oQ) noexcept {
    const int32_t ampi = static_cast<int32_t>(amp);
    for (int j = 0; j < n; ++j) {
        const uint32_t pI =
            popc32(static_cast<uint32_t>(sym_I) & static_cast<uint32_t>(j)) &
            1u;
        const uint32_t pQ =
            popc32(static_cast<uint32_t>(sym_Q) & static_cast<uint32_t>(j)) &
            1u;
        oI[j] = static_cast<int16_t>(ampi * (1 - 2 * static_cast<int32_t>(pI)));
        oQ[j] = static_cast<int16_t>(ampi * (1 - 2 * static_cast<int32_t>(pQ)));
    }
}
/// 인접 교환 정렬 — N=64, 고정 2016 인접 스왑(브랜치리스 트립).
constexpr int kMagSort64N = 64;
static_assert(static_cast<std::size_t>(kMagSort64N) *
                      static_cast<std::size_t>(kMagSort64N - 1) / 2u ==
                  2016u,
              "mag sort trip count");
inline void sort_u32_ct_adjacent_64_dispatch(uint32_t *a) noexcept {
    if (a == nullptr) {
        return;
    }
    for (int pass = 0; pass < kMagSort64N - 1; ++pass) {
        const int imax = kMagSort64N - 1 - pass;
        for (int i = 0; i < imax; ++i) {
            const uint32_t x = a[i];
            const uint32_t y = a[i + 1];
            const uint32_t gt = static_cast<uint32_t>(x > y);
            const uint32_t m = 0u - gt;
            a[i] = (x & ~m) | (y & m);
            a[i + 1] = (y & ~m) | (x & m);
        }
    }
}
inline void fill_u32_pad_max_(uint32_t *work, const uint32_t *src,
                              int nc) noexcept {
    for (int i = 0; i < nc; ++i) {
        work[static_cast<std::size_t>(i)] = src[static_cast<std::size_t>(i)];
    }
    for (int i = nc; i < kMagSort64N; ++i) {
        work[static_cast<std::size_t>(i)] = 0xFFFFFFFFu;
    }
}
inline int16_t ssat16_dispatch_(int32_t v) noexcept {
#if defined(__GNUC__) && defined(__ARM_ARCH) && (__ARM_ARCH >= 6)
    return static_cast<int16_t>(__builtin_arm_ssat(v, 16));
#else
    const uint32_t ov_hi = static_cast<uint32_t>(v > 32767);
    const uint32_t ov_lo = static_cast<uint32_t>(v < -32768);
    const int32_t msk = -static_cast<int32_t>(ov_hi | ov_lo);
    const int32_t repl = (32767 & -static_cast<int32_t>(ov_hi)) |
                         (-32768 & -static_cast<int32_t>(ov_lo));
    return static_cast<int16_t>((v & ~msk) | (repl & msk));
#endif
}
// [REMOVED Step1] I/Q 클립 경로 제거 — Gaussian noise 에 무효 확정 (T6 96.5% 유지).
//  T6 진단 시 outlier 발생 비율 0.003%, abs_sum=0. 완전 제거.

constexpr uint32_t k_BH_NOISE_FLOOR = 50u; // baseline 하한 (무간섭 판별)
constexpr uint32_t k_BH_SATURATION =
    8000u; // baseline 상한 (ADC 포화 방어)
} // namespace detail

} // namespace ProtectedEngine

#endif // HTS_V400_DISPATCHER_INTERNAL_HPP

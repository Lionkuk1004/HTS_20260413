// =============================================================================
// HTS_CFO_V5a.cpp — INNOViD HTS Rx CFO V5a
// Phase 1-3: Derotate / Walsh63_Dot / Energy_Multiframe (file-static).
// Phase 1-4: L&R segment autocorr estimator (Luise & Reggiannini 1995).
// Lab 전용: `/DHTS_V5A_DISABLE_PTE` 시 coarse/fine PTE(parabolic) 오프셋을 0으로
// 고정(미정의 시 기존과 동일).
// Lab 전용: `/DHTS_V5A_DISABLE_LR` 시 Estimate 루프에서 L&R(LR_Estimate_impl) 보정 0.
// Lab 전용: `/DHTS_LR_DIAG` 시 `g_lr_diag` 에 L&R 단계 스냅샷(세그먼트·R·Z·lag별 Hz).
// =============================================================================
#include "HTS_CFO_V5a.hpp"
#include "HTS_Rx_CFO_SinCos_Table.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace hts {
namespace rx_cfo {
namespace {
#if defined(HTS_CFO_V5A_PTE_DIAG)
struct V5aPteDiagBucket {
    char tag[12];
    uint32_t estimate_calls;
    uint32_t coarse_enter;
    uint32_t fine_enter;
    uint32_t coarse_nonzero;
    uint32_t fine_nonzero;
    uint64_t coarse_abs_sum_q15;
    uint64_t fine_abs_sum_q15;
    uint64_t lr_abs_sum_hz;
    uint32_t pte_samples;
    int32_t pte_cfo_min_hz;
    int32_t pte_cfo_max_hz;
    int64_t pte_cfo_sum_hz;
    uint32_t coarse_hist[5];
    uint32_t fine_hist[5];
};

static V5aPteDiagBucket g_v5a_pte_diag[16]{};
static int g_v5a_pte_diag_count = 0;
static int g_v5a_pte_diag_cur = 0;

static inline uint32_t pte_hist_bin_from_abs_q15(int32_t abs_q15) noexcept {
    if (abs_q15 == 0) return 0u;
    if (abs_q15 <= 4096) return 1u;
    if (abs_q15 <= 8192) return 2u;
    if (abs_q15 <= 16384) return 3u;
    return 4u;
}

static int diag_find_or_add_context_(const char* tag) noexcept {
    if (tag == nullptr || tag[0] == '\0') {
        return 0;
    }
    for (int i = 0; i < g_v5a_pte_diag_count; ++i) {
        if (std::strncmp(g_v5a_pte_diag[i].tag, tag,
                         sizeof(g_v5a_pte_diag[i].tag) - 1) == 0) {
            return i;
        }
    }
    if (g_v5a_pte_diag_count >= static_cast<int>(sizeof(g_v5a_pte_diag) /
                                                 sizeof(g_v5a_pte_diag[0]))) {
        return 0;
    }
    V5aPteDiagBucket& b = g_v5a_pte_diag[g_v5a_pte_diag_count];
    std::memset(&b, 0, sizeof(b));
    std::strncpy(b.tag, tag, sizeof(b.tag) - 1);
    b.pte_cfo_min_hz = 2147483647;
    b.pte_cfo_max_hz = -2147483647 - 1;
    ++g_v5a_pte_diag_count;
    return g_v5a_pte_diag_count - 1;
}
#endif

// 레거시 preamble/autocorr CFO 경로(정수 구현)와 동일.
inline constexpr int64_t kPreambleMagApproxThreshold = 1000LL;
inline constexpr int64_t kAutocorrMag2Threshold = 1000000LL;

// Same sequence as HTS_V400_Dispatcher_Internal.hpp k_w63 (Walsh-Hadamard row 63).
static constexpr int8_t kWalsh63Row63[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1};

/// BPTE i32 → i16 saturation (constant-time). `sat_i32_to_i16` / `Apply_Per_Chip`.
static inline int16_t saturate_i32_to_i16_bpte(int32_t v) noexcept {
    const int32_t high_diff = v - 32767;
    const int32_t high_mask = high_diff >> 31;
    int32_t r = (high_mask & v) | (~high_mask & 32767);

    const int32_t low_diff = r - (-32768);
    const int32_t low_mask = low_diff >> 31;
    r = (low_mask & static_cast<int32_t>(-32768)) | (~low_mask & r);

    return static_cast<int16_t>(r);
}

static inline int16_t sat_i32_to_i16(int32_t v) noexcept {
    return saturate_i32_to_i16_bpte(v);
}

// (x*A + y*B) / 2^14 with signed rounding (int64 inner product).
static inline int32_t dot_q14_round(int32_t x, int32_t a_q14, int32_t y,
                                    int32_t b_q14) noexcept {
    const int64_t p = static_cast<int64_t>(x) * static_cast<int64_t>(a_q14) +
                      static_cast<int64_t>(y) * static_cast<int64_t>(b_q14);
    const int64_t sign_mask = p >> 63;
    const int64_t bias =
        ((static_cast<int64_t>(1) << 13) ^ sign_mask) - sign_mask;
    return static_cast<int32_t>((p + bias) >> 14);
}

// --- Q14 per-chip path (레거시 Apply/Advance 정수 수식) ---
static inline int32_t apply_int_root_q14(int64_t c2) noexcept {
    if (c2 <= 0) {
        return 0;
    }
    int32_t lo = 0;
    constexpr int32_t kQ14 = 16384;
    int32_t hi = kQ14;
    while (lo < hi) {
        const int32_t mid = (lo + hi + 1) >> 1;
        if (static_cast<int64_t>(mid) * mid <= c2) {
            lo = mid;
        } else {
            hi = mid - 1;
        }
    }
    return lo;
}

static inline void apply_renorm_q14_accum(int32_t& ca, int32_t& sa) noexcept {
    const int64_t fc = static_cast<int64_t>(ca);
    const int64_t fs = static_cast<int64_t>(sa);
    const int64_t mag2 = fc * fc + fs * fs;
    if (mag2 <= 0) {
        return;
    }
    int64_t lo = 0;
    int64_t hi = 3037000499LL;
    while (lo < hi) {
        const int64_t mid = (lo + hi + 1) >> 1;
        if (mid != 0 && mid <= mag2 / mid) {
            lo = mid;
        } else {
            hi = mid - 1;
        }
    }
    const int64_t sm = lo;
    if (sm == 0) {
        return;
    }
    constexpr int64_t k14 = 16384;
    ca = static_cast<int32_t>((fc * k14 + (sm >> 1)) / sm);
    sa = static_cast<int32_t>((fs * k14 + (sm >> 1)) / sm);
}

static inline uint32_t apply_phase_inc_q32_from_hz(int32_t cfo_hz) noexcept {
    const int64_t phase_inc_s64 =
        (-static_cast<int64_t>(cfo_hz) * 4294967296LL) /
        static_cast<int64_t>(kChipRateHz);
    return static_cast<uint32_t>(phase_inc_s64);
}

static void Derotate_impl(const int16_t* rI, const int16_t* rQ, int16_t* oI,
                          int16_t* oQ, int chips, int32_t cfo_hz) noexcept {
    const int64_t phase_inc_s64 =
        (-static_cast<int64_t>(cfo_hz) * 4294967296LL) /
        static_cast<int64_t>(kChipRateHz);
    const uint32_t phase_inc_q32 = static_cast<uint32_t>(phase_inc_s64);
    uint32_t phase_q32 = 0u;
    for (int k = 0; k < chips; ++k) {
        const int32_t cos_q14 = static_cast<int32_t>(Lookup_Cos(phase_q32));
        const int32_t sin_q14 = static_cast<int32_t>(Lookup_Sin(phase_q32));
        const int32_t x = static_cast<int32_t>(rI[k]);
        const int32_t y = static_cast<int32_t>(rQ[k]);
        const int32_t a = dot_q14_round(x, cos_q14, y, -sin_q14);
        const int32_t b = dot_q14_round(x, sin_q14, y, cos_q14);
        oI[k] = sat_i32_to_i16(a);
        oQ[k] = sat_i32_to_i16(b);
        phase_q32 += phase_inc_q32;
    }
}

static void Walsh63_Dot_impl(const int16_t* rI, const int16_t* rQ, int32_t& dI,
                             int32_t& dQ) noexcept {
    int32_t aI = 0;
    int32_t aQ = 0;
    for (int k = 0; k < kChipsPerSym; ++k) {
        const int32_t w = static_cast<int32_t>(kWalsh63Row63[k]);
        aI += static_cast<int32_t>(rI[k]) * w;
        aQ += static_cast<int32_t>(rQ[k]) * w;
    }
    dI = aI;
    dQ = aQ;
}

static int64_t Energy_Multiframe_impl(const int16_t* rI,
                                      const int16_t* rQ) noexcept {
    int32_t d0I = 0;
    int32_t d0Q = 0;
    int32_t d1I = 0;
    int32_t d1Q = 0;
    Walsh63_Dot_impl(&rI[0], &rQ[0], d0I, d0Q);
    Walsh63_Dot_impl(&rI[kChipsPerSym], &rQ[kChipsPerSym], d1I, d1Q);
    const int64_t e0 =
        static_cast<int64_t>(d0I) * d0I + static_cast<int64_t>(d0Q) * d0Q;
    const int64_t e1 =
        static_cast<int64_t>(d1I) * d1I + static_cast<int64_t>(d1Q) * d1Q;
    return e0 + e1;
}

static inline int32_t q15_mul_hz_round(int32_t offset_q15,
                                       int32_t step_hz) noexcept {
    const int64_t prod =
        static_cast<int64_t>(offset_q15) * static_cast<int64_t>(step_hz);
    const int64_t adj = (prod >= 0) ? (1LL << 14) : -(1LL << 14);
    return static_cast<int32_t>((prod + adj) >> 15);
}

static inline int32_t parabolic_offset_q15_impl(int64_t em1_s64, int64_t e0_s64,
                                                int64_t ep1_s64) noexcept {
    int64_t max_abs = em1_s64;
    if (max_abs < 0) {
        max_abs = -max_abs;
    }
    int64_t t = e0_s64;
    if (t < 0) {
        t = -t;
    }
    if (t > max_abs) {
        max_abs = t;
    }
    t = ep1_s64;
    if (t < 0) {
        t = -t;
    }
    if (t > max_abs) {
        max_abs = t;
    }

    int sh = 0;
    while (max_abs > 0x3FFFFFFFLL) {
        max_abs >>= 1;
        ++sh;
    }
    const int32_t em1 = static_cast<int32_t>(em1_s64 >> sh);
    const int32_t e0 = static_cast<int32_t>(e0_s64 >> sh);
    const int32_t ep1 = static_cast<int32_t>(ep1_s64 >> sh);

    const int32_t denom_signed = em1 - (e0 << 1) + ep1;
    const int32_t abs_denom_mask = static_cast<int32_t>(denom_signed >> 31);
    const int32_t abs_denom =
        static_cast<int32_t>((denom_signed ^ abs_denom_mask) - abs_denom_mask);

    const int64_t safe_diff = static_cast<int64_t>(abs_denom) - 1LL;
    const int32_t valid_mask = static_cast<int32_t>(~(safe_diff >> 63));

    const int32_t numer = em1 - ep1;
    const int32_t offset_q15_raw = static_cast<int32_t>(
        (static_cast<int64_t>(numer) << 14) /
        static_cast<int64_t>(abs_denom + (1 - (valid_mask & 1))));
    const int32_t offset_q15_signed =
        static_cast<int32_t>((offset_q15_raw ^ abs_denom_mask) - abs_denom_mask);
    int32_t offset_q15 = static_cast<int32_t>(offset_q15_signed & valid_mask);

    if (offset_q15 > 32768) {
        offset_q15 = 32768;
    }
    if (offset_q15 < -32768) {
        offset_q15 = -32768;
    }
    return offset_q15;
}

// --- Integer atan2·Q12 (레거시 동일 LUT): ARM L&R + Holo autocorr ---
static inline int32_t isc_atan_frac_q12(int32_t y, int32_t x) noexcept {
    if (x <= 0 || y < 0 || y > x) {
        return 0;
    }
    if (y == 0) {
        return 0;
    }
    static constexpr int16_t kLut[17] = {
        0,    256,  511,  763,  1018, 1266, 1508, 1741,
        1965, 2178, 2380, 2571, 2749, 2915, 3068, 3208, 3217};
    const int64_t ratio_q14 =
        (static_cast<int64_t>(y) << 14) / static_cast<int64_t>(x);
    const int64_t scaled = ratio_q14 * 16;
    int32_t idx = static_cast<int32_t>(scaled >> 14);
    if (idx >= 16) {
        return static_cast<int32_t>(kLut[16]);
    }
    const int32_t base = static_cast<int32_t>(kLut[idx]);
    const int32_t next = static_cast<int32_t>(kLut[idx + 1]);
    const int32_t frac = static_cast<int32_t>(scaled & ((1 << 14) - 1));
    return base + static_cast<int32_t>(
               (static_cast<int64_t>(next - base) * frac) >> 14);
}

// --- atan primary branch [0, π/4): PTE on legacy kLut (BPTE idx clamp, linear 동일 idx) ---
static inline int32_t isc_atan_frac_pte_q12(int32_t v, int32_t u) noexcept {
    static constexpr int16_t kLut[17] = {
        0,    256,  511,  763,  1018, 1266, 1508, 1741,
        1965, 2178, 2380, 2571, 2749, 2915, 3068, 3208, 3217};
    const int64_t ratio_q14 =
        (static_cast<int64_t>(v) << 14) / static_cast<int64_t>(u);
    const int64_t scaled = ratio_q14 * 16;
    int32_t idx = static_cast<int32_t>(scaled >> 14);
    const int32_t t_hi = (16 - idx) >> 31;
    idx = (idx & ~t_hi) | (16 & t_hi);
    const int32_t dz = idx ^ 16;
    const int32_t is_lut16 = ~((dz | -dz) >> 31);
    const int32_t t_lo = (idx - 1) >> 31;
    const int32_t lo_i = (idx - 1) & ~t_lo;
    const int32_t sum = idx + 1;
    const int32_t ov2 = (16 - sum) >> 31;
    const int32_t hi_i = (sum & ~ov2) | (16 & ov2);
    const int32_t y_lo = static_cast<int32_t>(kLut[lo_i]);
    const int32_t y_md = static_cast<int32_t>(kLut[idx]);
    const int32_t y_hi = static_cast<int32_t>(kLut[hi_i]);
    const int32_t frac = static_cast<int32_t>(scaled & ((1 << 14) - 1));
    const int32_t a2 = y_hi - y_lo;
    const int32_t b2 = y_hi - (y_md << 1) + y_lo;
    const int64_t t1 = (static_cast<int64_t>(a2) * static_cast<int64_t>(frac)) >> 14;
    const int64_t x_sq =
        (static_cast<int64_t>(frac) * static_cast<int64_t>(frac)) >> 14;
    const int64_t t2 = (static_cast<int64_t>(b2) * x_sq) >> 14;
    const int32_t pte =
        y_md + static_cast<int32_t>(t1 + t2);
    return (pte & ~is_lut16) |
           (static_cast<int32_t>(kLut[16]) & is_lut16);
}

// --- Full-plane atan2 Q12: BPTE masks only (no if/else/?:) ---
static inline int32_t isc_atan2_q12_dpte(int32_t y, int32_t x) noexcept {
    const int32_t nz = static_cast<int32_t>(
        ((static_cast<int64_t>(x) | static_cast<int64_t>(y)) |
         (-(static_cast<int64_t>(x) | static_cast<int64_t>(y)))) >>
        63);
    const int32_t x_mask = x >> 31;
    const int32_t y_mask = y >> 31;
    const int32_t ax = (x ^ x_mask) - x_mask;
    const int32_t ay = (y ^ y_mask) - y_mask;
    const int32_t swap_gt = (ax - ay) >> 31;
    const int32_t u = (ax & ~swap_gt) | (ay & swap_gt);
    const int32_t v = (ay & ~swap_gt) | (ax & swap_gt);
    const int32_t uz_m = (u | -u) >> 31;
    const int32_t u_safe = u + 1 + uz_m;
    const int32_t sgn_y = y >> 31;
    const int32_t ret_u0 = (6434 ^ sgn_y) - sgn_y;
    const int32_t ang_pte = isc_atan_frac_pte_q12(v, u_safe);
    const int32_t ang_core = (ang_pte & uz_m) | (ret_u0 & ~uz_m);
    const int32_t ang_sw = (ang_core & ~swap_gt) | ((6434 - ang_core) & swap_gt);
    const int32_t ang_x = (ang_sw & ~x_mask) | ((12868 - ang_sw) & x_mask);
    const int32_t ang_y = (ang_x & ~y_mask) | ((-ang_x) & y_mask);
    return ang_y & nz;
}

static inline int32_t atan2_dpte_q15(int64_t y, int64_t x) noexcept {
    while ((y > 0x7FFFFFFFLL) || (y < -0x80000000LL) ||
           (x > 0x7FFFFFFFLL) || (x < -0x80000000LL)) {
        y >>= 1;
        x >>= 1;
    }
    const int32_t phase_q12 =
        isc_atan2_q12_dpte(static_cast<int32_t>(y), static_cast<int32_t>(x));
    return static_cast<int32_t>((static_cast<int64_t>(phase_q12) * 32768LL) /
                                12868LL);
}

static inline int32_t isc_atan2_q12(int32_t y, int32_t x) noexcept {
    if (x == 0 && y == 0) {
        return 0;
    }
    const int32_t ax = (x < 0) ? -x : x;
    const int32_t ay = (y < 0) ? -y : y;
    const bool swap = ay > ax;
    const int32_t u = swap ? ay : ax;
    const int32_t v = swap ? ax : ay;
    if (u == 0) {
        return (y >= 0) ? 6434 : -6434;
    }
    int32_t ang = isc_atan_frac_q12(v, u);
    if (swap) {
        ang = 6434 - ang;
    }
    if (x < 0) {
        ang = 12868 - ang;
    }
    if (y < 0) {
        ang = -ang;
    }
    return ang;
}

// L&R: atan2 → phase_q15 (arg(Z) × 32768 / π)
#if defined(HTS_PLATFORM_ARM)

static inline int32_t Atan2_To_Q15(int64_t y, int64_t x) noexcept {
    while ((y > 0x7FFFFFFFLL) || (y < -0x80000000LL) ||
           (x > 0x7FFFFFFFLL) || (x < -0x80000000LL)) {
        y >>= 1;
        x >>= 1;
    }
    const int32_t phase_q12 =
        isc_atan2_q12(static_cast<int32_t>(y), static_cast<int32_t>(x));
    return static_cast<int32_t>((static_cast<int64_t>(phase_q12) * 32768LL) /
                                12868LL);
}

#else

static inline int32_t Atan2_To_Q15(int64_t y, int64_t x) noexcept {
    const double yd = static_cast<double>(y);
    const double xd = static_cast<double>(x);
    const double phase_rad = std::atan2(yd, xd);
    constexpr double kInvPi = 0.31830988618379067154;
    const double phase_q15_d = phase_rad * 32768.0 * kInvPi;
    if (phase_q15_d > 32767.0) {
        return 32767;
    }
    if (phase_q15_d < -32768.0) {
        return -32768;
    }
    return static_cast<int32_t>(std::llround(phase_q15_d));
}

#endif

/// 레거시 autocorr CFO: block atan2·Q12 후 /lag.
static int32_t Autocorr_Block_Atan2_Q12(int32_t ac_I, int32_t ac_Q) noexcept {
    int64_t y = static_cast<int64_t>(ac_Q);
    int64_t x = static_cast<int64_t>(ac_I);
    while ((y > 0x7FFFFFFFLL) || (y < -0x80000000LL) ||
           (x > 0x7FFFFFFFLL) || (x < -0x80000000LL)) {
        y >>= 1;
        x >>= 1;
    }
    return isc_atan2_q12(static_cast<int32_t>(y), static_cast<int32_t>(x));
}

// Luise & Reggiannini (1995): Δf = arg(Z) / (π·(M+1)·T_seg)
//                         = phase_q15 × 10^7 / 26214400  (Hz), M=4, T_seg=16 µs.
static int32_t LR_Estimate_impl(const int16_t* rI,
                                const int16_t* rQ) noexcept {
    int32_t seg_I[kLR_NumSeg];
    int32_t seg_Q[kLR_NumSeg];

    for (int s = 0; s < kLR_NumSeg; ++s) {
        int32_t aI = 0;
        int32_t aQ = 0;
        const int base = s * kLR_SegSize;
        for (int k = 0; k < kLR_SegSize; ++k) {
            const int walsh_idx = (base + k) & (kChipsPerSym - 1);
            const int8_t w = kWalsh63Row63[walsh_idx];
            aI += static_cast<int32_t>(rI[base + k]) * static_cast<int32_t>(w);
            aQ += static_cast<int32_t>(rQ[base + k]) * static_cast<int32_t>(w);
        }
        seg_I[s] = aI;
        seg_Q[s] = aQ;
    }

#if defined(HTS_LR_DIAG)
    for (int s = 0; s < kLR_NumSeg; ++s) {
        g_lr_diag.seg_I[s] = static_cast<int64_t>(seg_I[s]);
        g_lr_diag.seg_Q[s] = static_cast<int64_t>(seg_Q[s]);
    }
    int64_t R_re_lag[kLR_MaxLag]{};
    int64_t R_im_lag[kLR_MaxLag]{};
#endif

    int64_t Z_re = 0;
    int64_t Z_im = 0;

    for (int lag = 1; lag <= kLR_MaxLag; ++lag) {
        int64_t acc_re = 0;
        int64_t acc_im = 0;
        for (int n = 0; n + lag < kLR_NumSeg; ++n) {
            const int64_t aI = seg_I[n + lag];
            const int64_t aQ = seg_Q[n + lag];
            const int64_t bI = seg_I[n];
            const int64_t bQ = seg_Q[n];
            const int64_t cr = aI * bI + aQ * bQ;
            const int64_t ci = aQ * bI - aI * bQ;
            acc_re += cr;
            acc_im += ci;
            Z_re += cr;
            Z_im += ci;
        }
#if defined(HTS_LR_DIAG)
        R_re_lag[lag - 1] = acc_re;
        R_im_lag[lag - 1] = acc_im;
#endif
    }

    const int32_t phase_q15 = Atan2_To_Q15(Z_im, Z_re);
    const int32_t lr_hz = static_cast<int32_t>(
        (static_cast<int64_t>(phase_q15) * 10000000LL) / 26214400LL);

#if defined(HTS_LR_DIAG)
    for (int d = 0; d < kLR_MaxLag; ++d) {
        g_lr_diag.R_re[d] = R_re_lag[d];
        g_lr_diag.R_im[d] = R_im_lag[d];
        const int32_t ph_d = Atan2_To_Q15(R_im_lag[d], R_re_lag[d]);
        g_lr_diag.lag_cfo_hz[d] = static_cast<int32_t>(
            (static_cast<int64_t>(ph_d) * 10000000LL) / 26214400LL);
    }
    g_lr_diag.Z_re = Z_re;
    g_lr_diag.Z_im = Z_im;
    g_lr_diag.Z_phase_q15 = phase_q15;
    g_lr_diag.lr_cfo_hz = lr_hz;
#endif

    return lr_hz;
}

#if defined(HTS_USE_MNM_WALSH)
// M&M (1997) style weights Q15, M=16 chip lags (constexpr table).
#define HTS_MNM_W_ROW(d_) \
    static_cast<int32_t>( \
        (3LL * (256LL - static_cast<int64_t>(16 - (d_)) * \
                            static_cast<int64_t>(16 - (d_))) * \
         32768LL) / \
        4096LL)
static constexpr int32_t kMnM_W_Q15[16] = {
    HTS_MNM_W_ROW(1),  HTS_MNM_W_ROW(2),  HTS_MNM_W_ROW(3),
    HTS_MNM_W_ROW(4),  HTS_MNM_W_ROW(5),  HTS_MNM_W_ROW(6),
    HTS_MNM_W_ROW(7),  HTS_MNM_W_ROW(8),  HTS_MNM_W_ROW(9),
    HTS_MNM_W_ROW(10), HTS_MNM_W_ROW(11), HTS_MNM_W_ROW(12),
    HTS_MNM_W_ROW(13), HTS_MNM_W_ROW(14), HTS_MNM_W_ROW(15),
    HTS_MNM_W_ROW(16)};
#undef HTS_MNM_W_ROW

// Walsh-aware chip-domain M&M + atan2 DPTE (parallel to LR_Estimate_impl).
static int32_t MnM_Walsh_Estimate_dpte_impl(const int16_t* rI,
                                            const int16_t* rQ) noexcept {
    int32_t pure_I[kPreambleChips];
    int32_t pure_Q[kPreambleChips];
    for (int n = 0; n < kPreambleChips; ++n) {
        const int32_t sym0_m = static_cast<int32_t>(n - 64) >> 31;
        const int32_t w_raw =
            static_cast<int32_t>(kWalsh63Row63[static_cast<size_t>(n) & 63u]);
        const int32_t w_eff = (w_raw & sym0_m) | (1 & ~sym0_m);
        const int32_t sm_eff = (w_eff >> 31) & sym0_m;
        const int32_t ri = static_cast<int32_t>(rI[n]);
        const int32_t rq = static_cast<int32_t>(rQ[n]);
        pure_I[n] = (ri ^ sm_eff) - sm_eff;
        pure_Q[n] = (rq ^ sm_eff) - sm_eff;
    }
    int64_t R_re[16]{};
    int64_t R_im[16]{};
    for (int d = 1; d <= 16; ++d) {
        int64_t acc_re = 0;
        int64_t acc_im = 0;
        const int n_max = kPreambleChips - d;
        for (int n = 0; n < n_max; ++n) {
            const int64_t aI = static_cast<int64_t>(pure_I[n + d]);
            const int64_t aQ = static_cast<int64_t>(pure_Q[n + d]);
            const int64_t bI = static_cast<int64_t>(pure_I[n]);
            const int64_t bQ = static_cast<int64_t>(pure_Q[n]);
            acc_re += aI * bI + aQ * bQ;
            acc_im += aQ * bI - aI * bQ;
        }
        R_re[d - 1] = acc_re;
        R_im[d - 1] = acc_im;
    }
    int64_t Z_re = 0;
    int64_t Z_im = 0;
    for (int d = 0; d < 16; ++d) {
        const int32_t rr = static_cast<int32_t>(R_re[d] >> 16);
        const int32_t ri = static_cast<int32_t>(R_im[d] >> 16);
        Z_re += static_cast<int64_t>(rr) * static_cast<int64_t>(kMnM_W_Q15[d]);
        Z_im += static_cast<int64_t>(ri) * static_cast<int64_t>(kMnM_W_Q15[d]);
    }
    const int32_t phase_q15 = atan2_dpte_q15(Z_im, Z_re);
    return static_cast<int32_t>(
        (static_cast<int64_t>(phase_q15) * 10000000LL) / 26214400LL);
}
#endif

}  // namespace

#if defined(HTS_USE_PACD)
int32_t Atan2_Correlation_Dpte_Q15(int64_t y, int64_t x) noexcept {
    return atan2_dpte_q15(y, x);
}
#endif

#if defined(HTS_LR_DIAG)
LrDiagSnapshot g_lr_diag{};
#endif

CFO_V5a::CFO_V5a() noexcept
    : last_cfo_hz_(0),
      runtime_enabled_(false)
#if defined(HTS_V5A_TESTONLY)
      ,
      last_cb_bin_(-1),
      last_fb_bin_(-1)
#endif
{
    for (int i = 0; i < kPreambleChips; ++i) {
        work_I_[i] = 0;
        work_Q_[i] = 0;
    }
    for (int j = 0; j < 32; ++j) {
        fine_energies_[j] = 0;
    }
}

void CFO_V5a::Init() noexcept {
    Build_SinCos_Table();
    last_cfo_hz_ = 0;
    last_apply_gate_mag_ = 0;
    last_apply_gate_autocorr_ = false;
#if defined(HTS_V5A_TESTONLY)
    last_cb_bin_ = -1;
    last_fb_bin_ = -1;
#endif
    runtime_enabled_ = (HTS_CFO_V5A_ENABLE != 0);
    Set_Apply_Cfo(0);
}

bool CFO_V5a::IsApplyAllowed() const noexcept {
    const int64_t thr = last_apply_gate_autocorr_ ? kAutocorrMag2Threshold
                                                   : kPreambleMagApproxThreshold;
    const int64_t diff = last_apply_gate_mag_ - thr;
    const int32_t mask = static_cast<int32_t>(~(diff >> 63));
    return mask != 0;
}

bool CFO_V5a::IsApplyDriveActive() const noexcept {
    return apply_sin_per_q14_ != 0 || apply_cos_per_q14_ != 16384;
}

CFO_Result CFO_V5a::Estimate(const int16_t* rx_I,
                             const int16_t* rx_Q) noexcept {
    CFO_Result res{};
    res.cfo_hz = 0;
    res.peak_energy = 0;
    res.valid = false;
    last_cfo_hz_ = 0;

    if (rx_I == nullptr || rx_Q == nullptr) {
        last_apply_gate_mag_ = 0;
        last_apply_gate_autocorr_ = false;
#if defined(HTS_V5A_TESTONLY)
        last_cb_bin_ = -1;
        last_fb_bin_ = -1;
#endif
        return res;
    }

    {
        int32_t d0I = 0;
        int32_t d0Q = 0;
        int32_t d1I = 0;
        int32_t d1Q = 0;
        Walsh63_Dot_impl(rx_I, rx_Q, d0I, d0Q);
        Walsh63_Dot_impl(&rx_I[kChipsPerSym], &rx_Q[kChipsPerSym], d1I, d1Q);
        const int64_t cos_delta = static_cast<int64_t>(d0I) * d1I +
                                  static_cast<int64_t>(d0Q) * d1Q;
        const int64_t sin_delta = static_cast<int64_t>(d0Q) * d1I -
                                  static_cast<int64_t>(d0I) * d1Q;
        const int64_t ac =
            (cos_delta < 0) ? -cos_delta : cos_delta;
        const int64_t as =
            (sin_delta < 0) ? -sin_delta : sin_delta;
        const int64_t mag_approx =
            (ac > as) ? (ac + (as >> 1)) : (as + (ac >> 1));
        last_apply_gate_mag_ = mag_approx;
        last_apply_gate_autocorr_ = false;
    }

    int32_t cfo_estimate = 0;
    int64_t final_peak_e = 0;
#if defined(HTS_CFO_V5A_PTE_DIAG)
    V5aPteDiagBucket* diag = &g_v5a_pte_diag[g_v5a_pte_diag_cur];
    diag->estimate_calls++;
#endif

    for (int pass = 0; pass < kCfoIterPasses; ++pass) {
        int64_t coarse_energies[kCfoCoarseBanks]{};
        const int32_t coarse_center = cfo_estimate;
        const int32_t coarse_start = coarse_center - kCfoRangeHz;

        int64_t cb_e = -1;
        int32_t cb_cfo = coarse_start;
        int cb_best_idx = 0;

        for (int b = 0; b < kCfoCoarseBanks; ++b) {
            const int32_t cfo_total = coarse_start + b * kCfoCoarseStep;
            Derotate_impl(rx_I, rx_Q, work_I_, work_Q_, kPreambleChips,
                          cfo_total);
            const int64_t e = Energy_Multiframe_impl(work_I_, work_Q_);
            coarse_energies[b] = e;
            // strict e > cb_e (tie → 유지), BPTE — `if (e > cb_e)` 와 비트 동치
            const int64_t diff_ce = e - cb_e;
            const int64_t update_ce = ~((diff_ce - 1) >> 63);
            cb_e = (~update_ce & cb_e) | (update_ce & e);
            cb_cfo = static_cast<int32_t>(
                (~update_ce & static_cast<int64_t>(cb_cfo)) |
                (update_ce & static_cast<int64_t>(cfo_total)));
            cb_best_idx = static_cast<int>(
                (~update_ce & static_cast<int64_t>(cb_best_idx)) |
                (update_ce & static_cast<int64_t>(b)));
        }

        int32_t coarse_offset_q15 = 0;
        if (cb_best_idx > 0 && cb_best_idx < (kCfoCoarseBanks - 1)) {
#if defined(HTS_V5A_DISABLE_PTE)
            coarse_offset_q15 = 0;
#else
            coarse_offset_q15 =
                parabolic_offset_q15_impl(coarse_energies[cb_best_idx - 1],
                                          coarse_energies[cb_best_idx],
                                          coarse_energies[cb_best_idx + 1]);
#endif
#if defined(HTS_CFO_V5A_PTE_DIAG)
            diag->coarse_enter++;
#endif
        }
#if defined(HTS_CFO_V5A_PTE_DIAG)
        {
            const int32_t abs_coarse =
                (coarse_offset_q15 < 0) ? -coarse_offset_q15 : coarse_offset_q15;
            if (abs_coarse != 0) {
                diag->coarse_nonzero++;
            }
            diag->coarse_abs_sum_q15 += static_cast<uint64_t>(abs_coarse);
            diag->coarse_hist[pte_hist_bin_from_abs_q15(abs_coarse)]++;
        }
#endif
        const int32_t cb_refined =
            cb_cfo + q15_mul_hz_round(coarse_offset_q15, kCfoCoarseStep);

        int32_t best_cfo = cb_refined;
        int64_t best_e = -1;
        int fine_best_idx = 0;

        const int32_t fine_start =
            cb_refined - (kCfoFineBanks / 2) * kCfoFineStep;

        for (int f = 0; f < kCfoFineBanks; ++f) {
            const int32_t cfo = fine_start + f * kCfoFineStep;
            Derotate_impl(rx_I, rx_Q, work_I_, work_Q_, kPreambleChips, cfo);
            const int64_t e = Energy_Multiframe_impl(work_I_, work_Q_);
            if (f < 32) {
                fine_energies_[f] = e;
            }
            // strict e > best_e (tie → 유지), BPTE
            const int64_t diff_fe = e - best_e;
            const int64_t update_fe = ~((diff_fe - 1) >> 63);
            best_e = (~update_fe & best_e) | (update_fe & e);
            best_cfo = static_cast<int32_t>(
                (~update_fe & static_cast<int64_t>(best_cfo)) |
                (update_fe & static_cast<int64_t>(cfo)));
            fine_best_idx = static_cast<int>(
                (~update_fe & static_cast<int64_t>(fine_best_idx)) |
                (update_fe & static_cast<int64_t>(f)));
        }
        int32_t fine_offset_q15 = 0;
        if (fine_best_idx > 0 && fine_best_idx < (kCfoFineBanks - 1)) {
#if defined(HTS_V5A_DISABLE_PTE)
            fine_offset_q15 = 0;
#else
            fine_offset_q15 = parabolic_offset_q15_impl(
                fine_energies_[fine_best_idx - 1], fine_energies_[fine_best_idx],
                fine_energies_[fine_best_idx + 1]);
#endif
#if defined(HTS_CFO_V5A_PTE_DIAG)
            diag->fine_enter++;
#endif
        }
#if defined(HTS_CFO_V5A_PTE_DIAG)
        {
            const int32_t abs_fine =
                (fine_offset_q15 < 0) ? -fine_offset_q15 : fine_offset_q15;
            if (abs_fine != 0) {
                diag->fine_nonzero++;
            }
            diag->fine_abs_sum_q15 += static_cast<uint64_t>(abs_fine);
            diag->fine_hist[pte_hist_bin_from_abs_q15(abs_fine)]++;
        }
#endif
        const int32_t fine_refined =
            best_cfo + q15_mul_hz_round(fine_offset_q15, kCfoFineStep);

        Derotate_impl(rx_I, rx_Q, work_I_, work_Q_, kPreambleChips,
                      fine_refined);
#if defined(HTS_V5A_DISABLE_LR)
        const int32_t lr_cfo = 0;
#else
#if defined(HTS_USE_MNM_WALSH)
        const int32_t lr_cfo = MnM_Walsh_Estimate_dpte_impl(work_I_, work_Q_);
#else
        const int32_t lr_cfo = LR_Estimate_impl(work_I_, work_Q_);
#endif
#endif

        cfo_estimate = fine_refined + lr_cfo;
#if defined(HTS_LR_DIAG)
        g_lr_diag.cb_cfo = cb_cfo;
        g_lr_diag.fine_refined = fine_refined;
        g_lr_diag.cfo_estimate_hz = cfo_estimate;
#endif
#if defined(HTS_CFO_V5A_PTE_DIAG)
        {
            const int32_t lr_abs = (lr_cfo < 0) ? -lr_cfo : lr_cfo;
            diag->lr_abs_sum_hz += static_cast<uint64_t>(lr_abs);
            if (fine_refined < diag->pte_cfo_min_hz) diag->pte_cfo_min_hz = fine_refined;
            if (fine_refined > diag->pte_cfo_max_hz) diag->pte_cfo_max_hz = fine_refined;
            diag->pte_cfo_sum_hz += static_cast<int64_t>(fine_refined);
            diag->pte_samples++;
        }
#endif
        final_peak_e = best_e;
#if defined(HTS_V5A_TESTONLY)
        last_cb_bin_ = cb_best_idx;
        last_fb_bin_ = fine_best_idx;
#endif
    }

    last_cfo_hz_ = cfo_estimate;
    res.cfo_hz = cfo_estimate;
    res.peak_energy = final_peak_e;
    res.valid = true;
    return res;
}

void CFO_V5a::Estimate_From_Autocorr(int32_t ac_I, int32_t ac_Q,
                                     int32_t lag_chips) noexcept {
    if (lag_chips <= 0) {
        last_apply_gate_mag_ = 0;
        last_apply_gate_autocorr_ = true;
        last_cfo_hz_ = 0;
        Set_Apply_Cfo(0);
        return;
    }
    const int64_t mag2 = static_cast<int64_t>(ac_I) * ac_I +
                         static_cast<int64_t>(ac_Q) * ac_Q;
    last_apply_gate_mag_ = mag2;
    last_apply_gate_autocorr_ = true;
    if (mag2 < kAutocorrMag2Threshold) {
        last_cfo_hz_ = 0;
        Set_Apply_Cfo(0);
        return;
    }
    const int32_t phase_q12_block = Autocorr_Block_Atan2_Q12(ac_I, ac_Q);
    const int32_t phase_q12_chip = phase_q12_block / lag_chips;
    const int64_t num =
        static_cast<int64_t>(phase_q12_chip) *
        static_cast<int64_t>(kChipRateHz);
    constexpr int64_t kDen = 2LL * 12868LL;  // 2·(π rad in Q12 units)
    const int64_t q =
        (num >= 0) ? (num + kDen / 2) : (num - kDen / 2);
    const int32_t cfo_hz = static_cast<int32_t>(q / kDen);
    last_cfo_hz_ = cfo_hz;
    Set_Apply_Cfo(cfo_hz);
}

void CFO_V5a::ApplyDerotate(const int16_t* in_I, const int16_t* in_Q,
                            int16_t* out_I, int16_t* out_Q, int chips,
                            int32_t cfo_hz) noexcept {
    Derotate_impl(in_I, in_Q, out_I, out_Q, chips, cfo_hz);
}

void CFO_V5a::Set_Apply_Cfo(int32_t cfo_hz) noexcept {
    apply_cfo_hz_ = cfo_hz;
    if (cfo_hz == 0) {
        apply_sin_per_q14_ = 0;
        apply_cos_per_q14_ = 16384;
    } else {
        const uint32_t inc_q32 = apply_phase_inc_q32_from_hz(cfo_hz);
        apply_sin_per_q14_ = static_cast<int32_t>(Lookup_Sin(inc_q32));
        apply_cos_per_q14_ = static_cast<int32_t>(Lookup_Cos(inc_q32));
        const int64_t sin_sq =
            static_cast<int64_t>(apply_sin_per_q14_) * apply_sin_per_q14_;
        constexpr int64_t kOne = 16384;
        const int64_t q14_sq = kOne * kOne;
        const int64_t c2 = q14_sq - sin_sq;
        if (c2 <= 0) {
            apply_cos_per_q14_ = 0;
        } else {
            apply_cos_per_q14_ = apply_int_root_q14(c2);
        }
    }
    Reset_Apply_Phase();
}

void CFO_V5a::Set_Apply_SinCosPerChip_Q14(int32_t sin_per_chip_q14,
                                         int32_t cos_per_chip_q14) noexcept {
    apply_cfo_hz_ = 0;
    apply_sin_per_q14_ = sin_per_chip_q14;
    apply_cos_per_q14_ = cos_per_chip_q14;
    Reset_Apply_Phase();
}

void CFO_V5a::Reset_Apply_Phase() noexcept {
    apply_cos_acc_q14_ = 16384;
    apply_sin_acc_q14_ = 0;
    apply_chip_counter_ = 0;
}

void CFO_V5a::Advance_Phase_Only(int chips) noexcept {
    if (chips <= 0) {
        return;
    }
    for (int k = 0; k < chips; ++k) {
        const int32_t next_cos = static_cast<int32_t>(
            (static_cast<int64_t>(apply_cos_acc_q14_) *
                 static_cast<int64_t>(apply_cos_per_q14_) -
             static_cast<int64_t>(apply_sin_acc_q14_) *
                 static_cast<int64_t>(apply_sin_per_q14_)) >>
            14);
        const int32_t next_sin = static_cast<int32_t>(
            (static_cast<int64_t>(apply_cos_acc_q14_) *
                 static_cast<int64_t>(apply_sin_per_q14_) +
             static_cast<int64_t>(apply_sin_acc_q14_) *
                 static_cast<int64_t>(apply_cos_per_q14_)) >>
            14);
        apply_cos_acc_q14_ = next_cos;
        apply_sin_acc_q14_ = next_sin;
        ++apply_chip_counter_;
        if ((apply_chip_counter_ & 0x3F) == 0) {
            apply_renorm_q14_accum(apply_cos_acc_q14_, apply_sin_acc_q14_);
        }
    }
}

void CFO_V5a::Apply_Per_Chip(int16_t& chip_I, int16_t& chip_Q) noexcept {
    const int32_t ci = static_cast<int32_t>(chip_I);
    const int32_t cq = static_cast<int32_t>(chip_Q);
    const int32_t ri = (ci * apply_cos_acc_q14_ + cq * apply_sin_acc_q14_) >> 14;
    const int32_t rq = (cq * apply_cos_acc_q14_ - ci * apply_sin_acc_q14_) >> 14;
    chip_I = saturate_i32_to_i16_bpte(ri);
    chip_Q = saturate_i32_to_i16_bpte(rq);
    const int32_t next_cos = static_cast<int32_t>(
        (static_cast<int64_t>(apply_cos_acc_q14_) *
             static_cast<int64_t>(apply_cos_per_q14_) -
         static_cast<int64_t>(apply_sin_acc_q14_) *
             static_cast<int64_t>(apply_sin_per_q14_)) >>
        14);
    const int32_t next_sin = static_cast<int32_t>(
        (static_cast<int64_t>(apply_cos_acc_q14_) *
             static_cast<int64_t>(apply_sin_per_q14_) +
         static_cast<int64_t>(apply_sin_acc_q14_) *
             static_cast<int64_t>(apply_cos_per_q14_)) >>
        14);
    apply_cos_acc_q14_ = next_cos;
    apply_sin_acc_q14_ = next_sin;
    ++apply_chip_counter_;
    if ((apply_chip_counter_ & 0x3F) == 0) {
        apply_renorm_q14_accum(apply_cos_acc_q14_, apply_sin_acc_q14_);
    }
}

#if defined(HTS_ALLOW_HOST_BUILD)
namespace test_export {

void Derotate_Table(const int16_t* rI, const int16_t* rQ, int16_t* oI,
                    int16_t* oQ, int chips, int32_t cfo_hz) noexcept {
    Derotate_impl(rI, rQ, oI, oQ, chips, cfo_hz);
}

void Walsh63_Dot_Table(const int16_t* rI, const int16_t* rQ, int32_t& dI,
                       int32_t& dQ) noexcept {
    Walsh63_Dot_impl(rI, rQ, dI, dQ);
}

int64_t Energy_Multiframe_Table(const int16_t* rI,
                                const int16_t* rQ) noexcept {
    return Energy_Multiframe_impl(rI, rQ);
}

int32_t LR_Estimate(const int16_t* rI, const int16_t* rQ) noexcept {
    return LR_Estimate_impl(rI, rQ);
}

#if defined(HTS_USE_MNM_WALSH)
int32_t MnM_Walsh_Estimate_Dpte_Table(const int16_t* rI,
                                    const int16_t* rQ) noexcept {
    return MnM_Walsh_Estimate_dpte_impl(rI, rQ);
}

int32_t Atan2_Dpte_Q15_Table(int64_t y, int64_t x) noexcept {
    return atan2_dpte_q15(y, x);
}
#endif

int32_t Parabolic_Offset_Q15(int64_t em1, int64_t e0, int64_t ep1) noexcept {
    return parabolic_offset_q15_impl(em1, e0, ep1);
}

}  // namespace test_export
#endif

#if defined(HTS_CFO_V5A_PTE_DIAG)
void V5a_Pte_Diag_Reset() noexcept {
    std::memset(g_v5a_pte_diag, 0, sizeof(g_v5a_pte_diag));
    g_v5a_pte_diag_count = 1;
    g_v5a_pte_diag_cur = 0;
    std::strncpy(g_v5a_pte_diag[0].tag, "GLOBAL",
                 sizeof(g_v5a_pte_diag[0].tag) - 1);
    g_v5a_pte_diag[0].pte_cfo_min_hz = 2147483647;
    g_v5a_pte_diag[0].pte_cfo_max_hz = -2147483647 - 1;
}

void V5a_Pte_Diag_Set_Context(const char* tag) noexcept {
    g_v5a_pte_diag_cur = diag_find_or_add_context_(tag);
}

void V5a_Pte_Diag_Print_Summary() noexcept {
    std::printf("[V5A-PTE-DIAG] === summary begin ===\n");
    for (int i = 0; i < g_v5a_pte_diag_count; ++i) {
        const V5aPteDiagBucket& d = g_v5a_pte_diag[i];
        if (d.estimate_calls == 0) {
            continue;
        }
        const uint32_t denom = (d.estimate_calls > 0u) ? d.estimate_calls : 1u;
        const uint32_t coarse_calls = (d.coarse_hist[0] + d.coarse_hist[1] +
                                       d.coarse_hist[2] + d.coarse_hist[3] +
                                       d.coarse_hist[4]);
        const uint32_t fine_calls = (d.fine_hist[0] + d.fine_hist[1] +
                                     d.fine_hist[2] + d.fine_hist[3] +
                                     d.fine_hist[4]);
        const uint64_t coarse_den = (coarse_calls > 0u) ? coarse_calls : 1u;
        const uint64_t fine_den = (fine_calls > 0u) ? fine_calls : 1u;
        const uint64_t avg_abs_coarse = d.coarse_abs_sum_q15 / coarse_den;
        const uint64_t avg_abs_fine = d.fine_abs_sum_q15 / fine_den;
        const uint64_t avg_lr = d.lr_abs_sum_hz / static_cast<uint64_t>(denom);
        const uint32_t pte_den = (d.pte_samples > 0u) ? d.pte_samples : 1u;
        const int64_t avg_pte = d.pte_cfo_sum_hz / static_cast<int64_t>(pte_den);
        std::printf(
            "[V5A-PTE-DIAG][%s] est=%u coarse_enter=%u fine_enter=%u coarse_nz=%u fine_nz=%u avg|coarse_q15|=%llu avg|fine_q15|=%llu avg|lr_hz|=%llu pte_cfo[min,max,avg]=[%d,%d,%lld]\n",
            d.tag, static_cast<unsigned>(d.estimate_calls),
            static_cast<unsigned>(d.coarse_enter), static_cast<unsigned>(d.fine_enter),
            static_cast<unsigned>(d.coarse_nonzero), static_cast<unsigned>(d.fine_nonzero),
            static_cast<unsigned long long>(avg_abs_coarse),
            static_cast<unsigned long long>(avg_abs_fine),
            static_cast<unsigned long long>(avg_lr), static_cast<int>(d.pte_cfo_min_hz),
            static_cast<int>(d.pte_cfo_max_hz), static_cast<long long>(avg_pte));
        std::printf(
            "[V5A-PTE-DIAG][%s] coarse_hist=[%u,%u,%u,%u,%u] fine_hist=[%u,%u,%u,%u,%u]\n",
            d.tag, static_cast<unsigned>(d.coarse_hist[0]),
            static_cast<unsigned>(d.coarse_hist[1]),
            static_cast<unsigned>(d.coarse_hist[2]),
            static_cast<unsigned>(d.coarse_hist[3]),
            static_cast<unsigned>(d.coarse_hist[4]),
            static_cast<unsigned>(d.fine_hist[0]),
            static_cast<unsigned>(d.fine_hist[1]),
            static_cast<unsigned>(d.fine_hist[2]),
            static_cast<unsigned>(d.fine_hist[3]),
            static_cast<unsigned>(d.fine_hist[4]));
    }
    std::printf("[V5A-PTE-DIAG] === summary end ===\n");
}
#endif

}  // namespace rx_cfo
}  // namespace hts

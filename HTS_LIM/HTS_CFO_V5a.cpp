// =============================================================================
// HTS_CFO_V5a.cpp — INNOViD HTS Rx CFO V5a
// Phase 1-3: Derotate / Walsh63_Dot / Energy_Multiframe (file-static).
// Phase 1-4: L&R segment autocorr estimator (Luise & Reggiannini 1995).
// =============================================================================
#include "HTS_CFO_V5a.hpp"
#include "HTS_Rx_CFO_SinCos_Table.hpp"

#if !defined(HTS_PLATFORM_ARM)
#include <cmath>
#endif

namespace hts {
namespace rx_cfo {
namespace {

// Same sequence as HTS_V400_Dispatcher_Internal.hpp k_w63 (Walsh-Hadamard row 63).
static constexpr int8_t kWalsh63Row63[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1};

static inline int16_t sat_i32_to_i16(int32_t v) noexcept {
    if (v > 32767) {
        return 32767;
    }
    if (v < -32768) {
        return -32768;
    }
    return static_cast<int16_t>(v);
}

// (x*A + y*B) / 2^14 with signed rounding (int64 inner product).
static inline int32_t dot_q14_round(int32_t x, int32_t a_q14, int32_t y,
                                    int32_t b_q14) noexcept {
    const int64_t p = static_cast<int64_t>(x) * static_cast<int64_t>(a_q14) +
                      static_cast<int64_t>(y) * static_cast<int64_t>(b_q14);
    if (p >= 0) {
        return static_cast<int32_t>((p + (1LL << 13)) >> 14);
    }
    return static_cast<int32_t>((p - (1LL << 13)) >> 14);
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

// --- L&R: atan2 → phase_q15 (arg(Z) × 32768 / π) ---
// HTS_CFO_Compensator::atan2_q12 is private; ARM path duplicates the same
// Q12 mapping (atan2·4096, ±π ≈ ±12868) from HTS_CFO_Compensator.h without
// modifying that translation unit.

#if defined(HTS_PLATFORM_ARM)

static inline int32_t lr_atan_frac_q12(int32_t y, int32_t x) noexcept {
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

static inline int32_t lr_atan2_q12(int32_t y, int32_t x) noexcept {
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
    int32_t ang = lr_atan_frac_q12(v, u);
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

static inline int32_t Atan2_To_Q15(int64_t y, int64_t x) noexcept {
    while ((y > 0x7FFFFFFFLL) || (y < -0x80000000LL) ||
           (x > 0x7FFFFFFFLL) || (x < -0x80000000LL)) {
        y >>= 1;
        x >>= 1;
    }
    const int32_t phase_q12 =
        lr_atan2_q12(static_cast<int32_t>(y), static_cast<int32_t>(x));
    // Q12 = atan2·4096 (π rad ≈ 12868); Q15 = arg × 32768/π.
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

    int64_t Z_re = 0;
    int64_t Z_im = 0;

    for (int lag = 1; lag <= kLR_MaxLag; ++lag) {
        for (int n = 0; n + lag < kLR_NumSeg; ++n) {
            const int64_t aI = seg_I[n + lag];
            const int64_t aQ = seg_Q[n + lag];
            const int64_t bI = seg_I[n];
            const int64_t bQ = seg_Q[n];
            Z_re += aI * bI + aQ * bQ;
            Z_im += aQ * bI - aI * bQ;
        }
    }

    const int32_t phase_q15 = Atan2_To_Q15(Z_im, Z_re);
    const int64_t cfo_hz_s64 =
        (static_cast<int64_t>(phase_q15) * 10000000LL) / 26214400LL;
    return static_cast<int32_t>(cfo_hz_s64);
}

}  // namespace

CFO_V5a::CFO_V5a() noexcept
    : last_cfo_hz_(0),
      runtime_enabled_(false) {
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
    runtime_enabled_ = (HTS_CFO_V5A_ENABLE != 0);
}

CFO_Result CFO_V5a::Estimate(const int16_t* rx_I,
                             const int16_t* rx_Q) noexcept {
    (void)rx_I;
    (void)rx_Q;
    CFO_Result res{};
    res.cfo_hz = 0;
    res.peak_energy = 0;
    res.valid = false;
    last_cfo_hz_ = 0;
    return res;
}

void CFO_V5a::ApplyDerotate(const int16_t* in_I, const int16_t* in_Q,
                            int16_t* out_I, int16_t* out_Q, int chips,
                            int32_t cfo_hz) noexcept {
    Derotate_impl(in_I, in_Q, out_I, out_Q, chips, cfo_hz);
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

}  // namespace test_export
#endif

}  // namespace rx_cfo
}  // namespace hts

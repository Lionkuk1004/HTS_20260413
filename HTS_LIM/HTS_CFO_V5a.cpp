// =============================================================================
// HTS_CFO_V5a.cpp — INNOViD HTS Rx CFO V5a
// Phase 1-3: Derotate / Walsh63_Dot / Energy_Multiframe (file-static).
// =============================================================================
#include "HTS_CFO_V5a.hpp"
#include "HTS_Rx_CFO_SinCos_Table.hpp"

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

}  // namespace test_export
#endif

}  // namespace rx_cfo
}  // namespace hts

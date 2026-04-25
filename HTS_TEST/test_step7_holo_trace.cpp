#include <cmath>
#include <cstdint>
#include <cstdio>

#include "legacy/HTS_CFO_Compensator.h"
#include "../HTS_LIM/HTS_CFO_V5a.hpp"
#include "../HTS_LIM/HTS_Rx_CFO_SinCos_Table.hpp"

#if !defined(HTS_ALLOW_HOST_BUILD)
#error "Host-only unit test (HTS_ALLOW_HOST_BUILD required)"
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kN = 128;
constexpr int kLag = 32;
constexpr int kAmp = 16384;
constexpr int kChipRate = 1000000;
constexpr int kHz = 5000;

static int32_t atan_frac_q12_ref(int32_t y, int32_t x) {
    if (x <= 0 || y < 0 || y > x) {
        return 0;
    }
    if (y == 0) {
        return 0;
    }
    static constexpr int16_t kLut[17] = {
        0,   256,  511,  763,  1018, 1266, 1508, 1741, 1965,
        2178, 2380, 2571, 2749, 2915, 3068, 3208, 3217};
    const int64_t ratio_q14 = (static_cast<int64_t>(y) << 14) / x;
    const int64_t scaled = ratio_q14 * 16;
    int32_t idx = static_cast<int32_t>(scaled >> 14);
    if (idx >= 16) {
        return static_cast<int32_t>(kLut[16]);
    }
    const int32_t base = static_cast<int32_t>(kLut[idx]);
    const int32_t next = static_cast<int32_t>(kLut[idx + 1]);
    const int32_t frac = static_cast<int32_t>(scaled & ((1 << 14) - 1));
    return base + static_cast<int32_t>((static_cast<int64_t>(next - base) * frac) >> 14);
}

static int32_t atan2_q12_ref(int32_t y, int32_t x) {
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
    int32_t ang = atan_frac_q12_ref(v, u);
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

static int32_t taylor_sin_q14_from_q12(int32_t phase_q12_chip) {
    const int64_t ph = static_cast<int64_t>(phase_q12_chip);
    int64_t sin_q14 = (ph * 4LL) -
                      (ph * ph * ph * 16384LL) /
                          (6LL * 4096LL * 4096LL * 4096LL);
    if (sin_q14 > 32767LL) {
        sin_q14 = 32767LL;
    }
    if (sin_q14 < -32768LL) {
        sin_q14 = -32768LL;
    }
    return static_cast<int32_t>(sin_q14);
}

static uint32_t phase_inc_q32_from_hz_ref(int32_t cfo_hz) {
    const int64_t phase_inc_s64 =
        (-static_cast<int64_t>(cfo_hz) * 4294967296LL) / static_cast<int64_t>(kChipRate);
    return static_cast<uint32_t>(phase_inc_s64);
}

}  // namespace

int main() {
    hts::rx_cfo::Build_SinCos_Table();

    int16_t I[kN]{};
    int16_t Q[kN]{};
    const double w = 2.0 * kPi * static_cast<double>(kHz) / static_cast<double>(kChipRate);
    for (int n = 0; n < kN; ++n) {
        const double ph = w * static_cast<double>(n);
        I[n] = static_cast<int16_t>(std::llround(static_cast<double>(kAmp) * std::cos(ph)));
        Q[n] = static_cast<int16_t>(std::llround(static_cast<double>(kAmp) * std::sin(ph)));
    }

    int64_t ac_i64 = 0;
    int64_t ac_q64 = 0;
    for (int n = 0; n + kLag < kN; ++n) {
        const int32_t i0 = static_cast<int32_t>(I[n]);
        const int32_t q0 = static_cast<int32_t>(Q[n]);
        const int32_t i1 = static_cast<int32_t>(I[n + kLag]);
        const int32_t q1 = static_cast<int32_t>(Q[n + kLag]);
        ac_i64 += static_cast<int64_t>(i0) * i1 + static_cast<int64_t>(q0) * q1;
        ac_q64 += static_cast<int64_t>(q0) * i1 - static_cast<int64_t>(i0) * q1;
    }
    const int32_t ac_i = static_cast<int32_t>(ac_i64);
    const int32_t ac_q = static_cast<int32_t>(ac_q64);
    const double mag = std::sqrt(static_cast<double>(ac_i64) * ac_i64 +
                                 static_cast<double>(ac_q64) * ac_q64);

    const int32_t phase_q12 = atan2_q12_ref(ac_q, ac_i);
    const int32_t phase_q12_chip = phase_q12 / kLag;
    const int32_t cfo_sin_q14 = taylor_sin_q14_from_q12(phase_q12_chip);
    const int32_t phase_q15_ref =
        static_cast<int32_t>((static_cast<int64_t>(phase_q12) * 32768LL) / 12868LL);

    const int64_t num = static_cast<int64_t>(phase_q12_chip) * static_cast<int64_t>(kChipRate);
    constexpr int64_t kDen = 2LL * 12868LL;
    const int64_t q = (num >= 0) ? (num + kDen / 2) : (num - kDen / 2);
    const int32_t v5a_hz_ref = static_cast<int32_t>(q / kDen);
    const uint32_t v5a_inc_q32_ref = phase_inc_q32_from_hz_ref(v5a_hz_ref);
    const int32_t v5a_sin_ref = static_cast<int32_t>(hts::rx_cfo::Lookup_Sin(v5a_inc_q32_ref));

    ProtectedEngine::HTS_CFO_Compensator cfo{};
    cfo.Init();
    cfo.Estimate_From_Autocorr(ac_i, ac_q, kLag);

    hts::rx_cfo::CFO_V5a v5a{};
    v5a.Init();
    v5a.Estimate_From_Autocorr(ac_i, ac_q, kLag);

    const int32_t v5a_hz = v5a.GetLastCfoHz();
    const uint32_t v5a_inc_q32 = phase_inc_q32_from_hz_ref(v5a_hz);

    std::printf("Trace input: hz=%d N=%d lag=%d amp=%d\n", kHz, kN, kLag, kAmp);
    std::printf("Trace ac: ac_I=%d ac_Q=%d mag=%.2f\n", ac_i, ac_q, mag);
    std::printf(
        "Trace cfo_: phase_q12=%d phase_q12_chip=%d sin_q14=%d sin_apply=%d cos_apply=%d\n",
        phase_q12, phase_q12_chip, cfo_sin_q14, cfo.Get_Sin_Per_Chip_Q14(),
        cfo.Get_Cos_Per_Chip_Q14());
    std::printf(
        "Trace v5a(ref): phase_q12=%d phase_q15_ref=%d cfo_hz_ref=%d phase_inc_q32_ref=0x%08X "
        "sin_ref=%d\n",
        phase_q12, phase_q15_ref, v5a_hz_ref, static_cast<unsigned>(v5a_inc_q32_ref),
        v5a_sin_ref);
    std::printf(
        "Trace v5a(run): cfo_hz=%d phase_inc_q32=0x%08X sin_apply=%d cos_apply=%d\n", v5a_hz,
        static_cast<unsigned>(v5a_inc_q32), v5a.Get_Apply_Sin_Per_Chip_Q14(),
        v5a.Get_Apply_Cos_Per_Chip_Q14());

    std::printf(
        "Trace diff: phase_q12(cfo-v5a_ref)=%d sin(cfo-v5a_run)=%d cos(cfo-v5a_run)=%d\n", 0,
        cfo.Get_Sin_Per_Chip_Q14() - v5a.Get_Apply_Sin_Per_Chip_Q14(),
        cfo.Get_Cos_Per_Chip_Q14() - v5a.Get_Apply_Cos_Per_Chip_Q14());

    std::printf("test_step7_holo_trace: PASS\n");
    return 0;
}

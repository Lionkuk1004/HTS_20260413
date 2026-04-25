#include <cmath>
#include <cstdint>
#include <cstdio>

#include "legacy/HTS_CFO_Compensator.h"
#include "../HTS_LIM/HTS_CFO_V5a.hpp"
#include "../HTS_LIM/HTS_Preamble_Holographic.h"
#include "../HTS_LIM/HTS_Rx_CFO_SinCos_Table.hpp"

#if !defined(HTS_ALLOW_HOST_BUILD)
#error "Host-only unit test (HTS_ALLOW_HOST_BUILD required)"
#endif

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kFs = 1000000.0;
constexpr int kN = 64;
constexpr int kLag = 32;
constexpr int16_t kAmp = 1000;
static constexpr int8_t kWalsh63Row63[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1};
}

int main() {
    hts::rx_cfo::Build_SinCos_Table();
    using ProtectedEngine::Holographic::holographic_dot_segmented;

    int16_t base_i[kN]{};
    int16_t base_q[kN]{};
    for (int n = 0; n < kN; ++n) {
        base_i[n] = static_cast<int16_t>(kAmp * kWalsh63Row63[n]);
        base_q[n] = 0;
    }
    const int64_t e_base = holographic_dot_segmented(base_i, base_q);

    const int hz_list[] = {0, 1000, 5000, 10000};
    for (const int hz : hz_list) {
        int16_t cfo_i[kN]{};
        int16_t cfo_q[kN]{};
        for (int n = 0; n < kN; ++n) {
            const double ph =
                2.0 * kPi * static_cast<double>(hz) * static_cast<double>(n) / kFs;
            cfo_i[n] = static_cast<int16_t>(
                std::llround(static_cast<double>(base_i[n]) * std::cos(ph) -
                             static_cast<double>(base_q[n]) * std::sin(ph)));
            cfo_q[n] = static_cast<int16_t>(
                std::llround(static_cast<double>(base_i[n]) * std::sin(ph) +
                             static_cast<double>(base_q[n]) * std::cos(ph)));
        }

        int64_t ac_i64 = 0;
        int64_t ac_q64 = 0;
        for (int n = 0; n + kLag < kN; ++n) {
            const int32_t i0 = cfo_i[n];
            const int32_t q0 = cfo_q[n];
            const int32_t i1 = cfo_i[n + kLag];
            const int32_t q1 = cfo_q[n + kLag];
            ac_i64 += static_cast<int64_t>(i0) * i1 + static_cast<int64_t>(q0) * q1;
            ac_q64 += static_cast<int64_t>(q0) * i1 - static_cast<int64_t>(i0) * q1;
        }
        const int32_t ac_i = static_cast<int32_t>(ac_i64);
        const int32_t ac_q = static_cast<int32_t>(ac_q64);

        const int64_t e_no = holographic_dot_segmented(cfo_i, cfo_q);

        ProtectedEngine::HTS_CFO_Compensator cfo{};
        cfo.Init();
        cfo.Estimate_From_Autocorr(ac_i, ac_q, kLag);
        int16_t out_c_i[kN]{};
        int16_t out_c_q[kN]{};
        for (int n = 0; n < kN; ++n) {
            out_c_i[n] = cfo_i[n];
            out_c_q[n] = cfo_q[n];
            cfo.Apply(out_c_i[n], out_c_q[n]);
        }
        const int64_t e_c = holographic_dot_segmented(out_c_i, out_c_q);

        hts::rx_cfo::CFO_V5a v5a{};
        v5a.Init();
        v5a.Estimate_From_Autocorr(ac_i, ac_q, kLag);
        int16_t out_v_i[kN]{};
        int16_t out_v_q[kN]{};
        for (int n = 0; n < kN; ++n) {
            out_v_i[n] = cfo_i[n];
            out_v_q[n] = cfo_q[n];
            v5a.Apply_Per_Chip(out_v_i[n], out_v_q[n]);
        }
        const int64_t e_v = holographic_dot_segmented(out_v_i, out_v_q);

        const double r_no = (e_base > 0) ? static_cast<double>(e_no) / e_base : 0.0;
        const double r_c = (e_base > 0) ? static_cast<double>(e_c) / e_base : 0.0;
        const double r_v = (e_base > 0) ? static_cast<double>(e_v) / e_base : 0.0;
        std::printf(
            "HoloCfoSensitivity: hz=%6d ratio_no=%.6f ratio_cfo=%.6f ratio_v5a=%.6f "
            "sin(cfo,v5a)=(%d,%d)\n",
            hz, r_no, r_c, r_v, cfo.Get_Sin_Per_Chip_Q14(),
            v5a.Get_Apply_Sin_Per_Chip_Q14());
    }

    std::printf("test_holo_cfo_sensitivity: PASS\n");
    return 0;
}

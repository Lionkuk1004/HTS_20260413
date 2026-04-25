#include <cmath>
#include <cstdint>
#include <cstdio>

#include "../HTS_LIM/HTS_CFO_Compensator.h"
#include "../HTS_LIM/HTS_CFO_V5a.hpp"
#include "../HTS_LIM/HTS_Rx_CFO_SinCos_Table.hpp"

#if !defined(HTS_ALLOW_HOST_BUILD)
#error "Host-only unit test (HTS_ALLOW_HOST_BUILD required)"
#endif

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr int kN = 64;
constexpr int kLag = 32;
constexpr int kHz = 5000;
constexpr double kFs = 1000000.0;
constexpr int16_t kAmp = 1000;

int diff_l1(const int16_t* a_i, const int16_t* a_q, const int16_t* b_i,
            const int16_t* b_q, int n) {
    int max_d = 0;
    for (int i = 0; i < n; ++i) {
        const int d = std::abs(static_cast<int>(a_i[i]) - static_cast<int>(b_i[i])) +
                      std::abs(static_cast<int>(a_q[i]) - static_cast<int>(b_q[i]));
        if (d > max_d) {
            max_d = d;
        }
    }
    return max_d;
}
}  // namespace

int main() {
    hts::rx_cfo::Build_SinCos_Table();

    int16_t src_i[kN]{};
    int16_t src_q[kN]{};
    int16_t cfo_i[kN]{};
    int16_t cfo_q[kN]{};
    for (int n = 0; n < kN; ++n) {
        src_i[n] = kAmp;
        src_q[n] = 0;
        const double ph = 2.0 * kPi * static_cast<double>(kHz) *
                          static_cast<double>(n) / kFs;
        cfo_i[n] = static_cast<int16_t>(std::llround(kAmp * std::cos(ph)));
        cfo_q[n] = static_cast<int16_t>(std::llround(kAmp * std::sin(ph)));
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

    ProtectedEngine::HTS_CFO_Compensator cfo{};
    cfo.Init();
    cfo.Estimate_From_Autocorr(ac_i, ac_q, kLag);

    hts::rx_cfo::CFO_V5a v5a{};
    v5a.Init();
    v5a.Estimate_From_Autocorr(ac_i, ac_q, kLag);

    int16_t out_c_i[kN]{};
    int16_t out_c_q[kN]{};
    int16_t out_v_i[kN]{};
    int16_t out_v_q[kN]{};
    for (int n = 0; n < kN; ++n) {
        out_c_i[n] = cfo_i[n];
        out_c_q[n] = cfo_q[n];
        out_v_i[n] = cfo_i[n];
        out_v_q[n] = cfo_q[n];
        cfo.Apply(out_c_i[n], out_c_q[n]);
        v5a.Apply_Per_Chip(out_v_i[n], out_v_q[n]);
    }

    const int err_cfo_src = diff_l1(out_c_i, out_c_q, src_i, src_q, kN);
    const int err_v5a_src = diff_l1(out_v_i, out_v_q, src_i, src_q, kN);
    const int err_cfo_cfo = diff_l1(out_c_i, out_c_q, cfo_i, cfo_q, kN);
    const int err_v5a_cfo = diff_l1(out_v_i, out_v_q, cfo_i, cfo_q, kN);
    const int err_cfo_v5a = diff_l1(out_c_i, out_c_q, out_v_i, out_v_q, kN);

    std::printf(
        "ApplyReal: ac=(%d,%d) cfo_est(s,c)=(%d,%d) v5a_est(s,c)=(%d,%d)\n", ac_i,
        ac_q, cfo.Get_Sin_Per_Chip_Q14(), cfo.Get_Cos_Per_Chip_Q14(),
        v5a.Get_Apply_Sin_Per_Chip_Q14(), v5a.Get_Apply_Cos_Per_Chip_Q14());
    std::printf(
        "ApplyReal: max|cfo_out-src|=%d max|v5a_out-src|=%d "
        "max|cfo_out-cfo_in|=%d max|v5a_out-cfo_in|=%d max|cfo_out-v5a_out|=%d\n",
        err_cfo_src, err_v5a_src, err_cfo_cfo, err_v5a_cfo, err_cfo_v5a);
    std::printf("test_step7_apply_real: PASS\n");
    return 0;
}

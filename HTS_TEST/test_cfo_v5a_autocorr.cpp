// =============================================================================
// test_cfo_v5a_autocorr.cpp — CFO_V5a::Estimate_From_Autocorr (Holo lag path)
// Step 4 host UT (HTS_ALLOW_HOST_BUILD)
// =============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "../HTS_LIM/HTS_CFO_Compensator.h"
#include "../HTS_LIM/HTS_CFO_V5a.hpp"
#include "../HTS_LIM/HTS_Rx_CFO_SinCos_Table.hpp"

#if !defined(HTS_ALLOW_HOST_BUILD)
#error "Host-only unit test (HTS_ALLOW_HOST_BUILD required)"
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int32_t kLag = 32;

int g_fail = 0;

void test_zero_phase() noexcept {
    hts::rx_cfo::CFO_V5a v;
    v.Init();
    v.Estimate_From_Autocorr(1000, 0, kLag);
    if (v.GetLastCfoHz() != 0) {
        std::printf("FAIL Test1: last_hz=%d expected 0\n",
                    static_cast<int>(v.GetLastCfoHz()));
        ++g_fail;
        return;
    }
    if (v.Get_Apply_Sin_Per_Chip_Q14() != 0 ||
        v.Get_Apply_Cos_Per_Chip_Q14() != 16384) {
        std::printf("FAIL Test1: sin/cos not identity\n");
        ++g_fail;
        return;
    }
    std::printf("Test 1 PASS: ac on +I axis -> 0 Hz\n");
}

void autocorr_lag32(const int16_t* I, const int16_t* Q, int N, int32_t& ac_I,
                    int32_t& ac_Q) noexcept {
    int64_t sI = 0;
    int64_t sQ = 0;
    for (int n = 0; n + kLag < N; ++n) {
        const int32_t I0 = static_cast<int32_t>(I[n]);
        const int32_t Q0 = static_cast<int32_t>(Q[n]);
        const int32_t I1 = static_cast<int32_t>(I[n + kLag]);
        const int32_t Q1 = static_cast<int32_t>(Q[n + kLag]);
        sI += static_cast<int64_t>(I0) * I1 + static_cast<int64_t>(Q0) * Q1;
        sQ += static_cast<int64_t>(I0) * Q1 - static_cast<int64_t>(Q0) * I1;
    }
    ac_I = static_cast<int32_t>(sI);
    ac_Q = static_cast<int32_t>(sQ);
}

void test_positive_cfo_hz(int32_t target_hz) noexcept {
    constexpr int N = 160;
    constexpr int amp = 1200;
    constexpr double fs = 1000000.0;
    const double w = 2.0 * kPi * static_cast<double>(target_hz) / fs;
    int16_t I[N]{};
    int16_t Q[N]{};
    for (int n = 0; n < N; ++n) {
        const double ph = w * static_cast<double>(n);
        I[n] = static_cast<int16_t>(
            std::llround(static_cast<double>(amp) * std::cos(ph)));
        Q[n] = static_cast<int16_t>(
            std::llround(static_cast<double>(amp) * std::sin(ph)));
    }
    int32_t ac_I = 0;
    int32_t ac_Q = 0;
    autocorr_lag32(I, Q, N, ac_I, ac_Q);
    hts::rx_cfo::CFO_V5a v;
    v.Init();
    v.Estimate_From_Autocorr(ac_I, ac_Q, kLag);
    const int32_t est = v.GetLastCfoHz();
    const int32_t err = est - target_hz;
    if (err > 200 || err < -200) {
        std::printf(
            "FAIL Test2: target=%d est=%d acI=%d acQ=%d\n",
            static_cast<int>(target_hz), static_cast<int>(est),
            static_cast<int>(ac_I), static_cast<int>(ac_Q));
        ++g_fail;
        return;
    }
    std::printf("Test 2 PASS: +%d Hz -> est=%d (err=%d)\n",
                static_cast<int>(target_hz), static_cast<int>(est),
                static_cast<int>(err));
}

void test_negative_cfo_hz() noexcept {
    constexpr int32_t target_hz = -3000;
    constexpr int N = 160;
    constexpr int amp = 1200;
    constexpr double fs = 1000000.0;
    const double w = 2.0 * kPi * static_cast<double>(target_hz) / fs;
    int16_t I[N]{};
    int16_t Q[N]{};
    for (int n = 0; n < N; ++n) {
        const double ph = w * static_cast<double>(n);
        I[n] = static_cast<int16_t>(
            std::llround(static_cast<double>(amp) * std::cos(ph)));
        Q[n] = static_cast<int16_t>(
            std::llround(static_cast<double>(amp) * std::sin(ph)));
    }
    int32_t ac_I = 0;
    int32_t ac_Q = 0;
    autocorr_lag32(I, Q, N, ac_I, ac_Q);
    hts::rx_cfo::CFO_V5a v;
    v.Init();
    v.Estimate_From_Autocorr(ac_I, ac_Q, kLag);
    const int32_t est = v.GetLastCfoHz();
    const int32_t err = est - target_hz;
    if (err > 200 || err < -200) {
        std::printf("FAIL Test3: target=%d est=%d\n",
                    static_cast<int>(target_hz), static_cast<int>(est));
        ++g_fail;
        return;
    }
    std::printf("Test 3 PASS: %d Hz -> est=%d (err=%d)\n",
                static_cast<int>(target_hz), static_cast<int>(est),
                static_cast<int>(err));
}

void test_vs_compensator_effective_hz() noexcept {
    constexpr int32_t ac_I = 3000000;
    constexpr int32_t ac_Q = 1500000;
    ProtectedEngine::HTS_CFO_Compensator cfo;
    cfo.Init();
    cfo.Estimate_From_Autocorr(ac_I, ac_Q, kLag);
    if (!cfo.Is_Apply_Active()) {
        std::printf("FAIL Test4: cfo not active\n");
        ++g_fail;
        return;
    }
    hts::rx_cfo::CFO_V5a v;
    v.Init();
    v.Estimate_From_Autocorr(ac_I, ac_Q, kLag);
    const double hz_c = cfo.Get_Est_Hz(1000000.0);
    const int32_t hz_v = v.GetLastCfoHz();
    const double diff = std::fabs(hz_c - static_cast<double>(hz_v));
    // cfo_: Q12/lag + Taylor sin + int cos; V5a: atan2→Hz→Lookup (Step 4 Hz 통일).
    // Effective CFO(Hz) should agree; per-chip Q14 may differ by >100 LSB.
    if (diff > 200.0) {
        std::printf(
            "FAIL Test4: Get_Est_Hz=%.2f vs V5a last_hz=%d (|diff|=%.2f)\n",
            hz_c, static_cast<int>(hz_v), diff);
        ++g_fail;
        return;
    }
    const int32_t s_c = cfo.Get_Sin_Per_Chip_Q14();
    const int32_t c_c = cfo.Get_Cos_Per_Chip_Q14();
    const int32_t s_v = v.Get_Apply_Sin_Per_Chip_Q14();
    const int32_t c_v = v.Get_Apply_Cos_Per_Chip_Q14();
    const int32_t ds = (s_v > s_c) ? (s_v - s_c) : (s_c - s_v);
    const int32_t dc = (c_v > c_c) ? (c_v - c_c) : (c_c - c_v);
    std::printf(
        "Test 4 PASS: |Hz_cfo - Hz_v5a|=%.2f (|d_sin|=%d |d_cos|=%d; Q14 paths "
        "differ)\n",
        diff, static_cast<int>(ds), static_cast<int>(dc));
}

}  // namespace

int main() {
    hts::rx_cfo::Build_SinCos_Table();
    test_zero_phase();
    test_positive_cfo_hz(5000);
    test_negative_cfo_hz();
    test_vs_compensator_effective_hz();
    if (g_fail != 0) {
        std::printf("test_cfo_v5a_autocorr: FAIL (%d)\n", g_fail);
        return 1;
    }
    std::printf("test_cfo_v5a_autocorr: PASS\n");
    return 0;
}

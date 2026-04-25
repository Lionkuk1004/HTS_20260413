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
constexpr int32_t kLag = 32;
constexpr double kFs = 1000000.0;
constexpr int kChips = 64;

int g_fail = 0;

static constexpr int8_t kWalsh63Row63[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1};

void make_ac_from_hz(int32_t hz, int32_t& ac_i, int32_t& ac_q) {
    constexpr double kMag = 5000000.0;
    const double theta =
        2.0 * kPi * static_cast<double>(hz) * static_cast<double>(kLag) / kFs;
    ac_i = static_cast<int32_t>(std::llround(kMag * std::cos(theta)));
    ac_q = static_cast<int32_t>(std::llround(kMag * std::sin(theta)));
}

void test_perchip_signs_from_hz(int32_t hz) {
    int32_t ac_i = 0;
    int32_t ac_q = 0;
    make_ac_from_hz(hz, ac_i, ac_q);

    ProtectedEngine::HTS_CFO_Compensator cfo{};
    cfo.Init();
    cfo.Estimate_From_Autocorr(ac_i, ac_q, kLag);

    hts::rx_cfo::CFO_V5a v5a_est{};
    v5a_est.Init();
    v5a_est.Estimate_From_Autocorr(ac_i, ac_q, kLag);

    hts::rx_cfo::CFO_V5a v5a_set{};
    v5a_set.Init();
    v5a_set.Set_Apply_Cfo(hz);

    if (!cfo.Is_Apply_Active() || !v5a_est.IsApplyAllowed()) {
        std::printf("FAIL Test1 hz=%d: inactive gate (ac_i=%d ac_q=%d)\n", hz,
                    ac_i, ac_q);
        ++g_fail;
        return;
    }

    const int c_s = cfo.Get_Sin_Per_Chip_Q14();
    const int c_c = cfo.Get_Cos_Per_Chip_Q14();
    const int v_est_s = v5a_est.Get_Apply_Sin_Per_Chip_Q14();
    const int v_est_c = v5a_est.Get_Apply_Cos_Per_Chip_Q14();
    const int v_set_s = v5a_set.Get_Apply_Sin_Per_Chip_Q14();
    const int v_set_c = v5a_set.Get_Apply_Cos_Per_Chip_Q14();

    std::printf(
        "Test1 hz=%6d ac=(%d,%d) cfo(s,c)=(%d,%d) "
        "v5a_est(s,c)=(%d,%d) v5a_set(s,c)=(%d,%d)\n",
        hz, ac_i, ac_q, c_s, c_c, v_est_s, v_est_c, v_set_s, v_set_c);
}

void test_apply_diff_from_hz(int32_t hz) {
    int32_t ac_i = 0;
    int32_t ac_q = 0;
    make_ac_from_hz(hz, ac_i, ac_q);

    ProtectedEngine::HTS_CFO_Compensator cfo{};
    cfo.Init();
    cfo.Estimate_From_Autocorr(ac_i, ac_q, kLag);

    hts::rx_cfo::CFO_V5a v5a{};
    v5a.Init();
    v5a.Set_Apply_Cfo(hz);

    int max_diff = 0;
    int sum_diff = 0;
    int16_t ci = 1000;
    int16_t cq = -700;
    int16_t vi = 1000;
    int16_t vq = -700;
    for (int n = 0; n < 64; ++n) {
        cfo.Apply(ci, cq);
        v5a.Apply_Per_Chip(vi, vq);
        const int d = std::abs(static_cast<int>(ci) - static_cast<int>(vi)) +
                      std::abs(static_cast<int>(cq) - static_cast<int>(vq));
        if (d > max_diff) {
            max_diff = d;
        }
        sum_diff += d;
    }

    std::printf("Test2 hz=%6d apply64 max_diff=%d mean_diff=%.2f\n", hz,
                max_diff, static_cast<double>(sum_diff) / 64.0);
}

void test_estimate_sign_from_signal(int32_t injected_hz) {
    constexpr int N = 160;
    constexpr int amp = 1200;
    const double w = 2.0 * kPi * static_cast<double>(injected_hz) / kFs;
    int16_t I[N]{};
    int16_t Q[N]{};
    for (int n = 0; n < N; ++n) {
        const double ph = w * static_cast<double>(n);
        I[n] = static_cast<int16_t>(
            std::llround(static_cast<double>(amp) * std::cos(ph)));
        Q[n] = static_cast<int16_t>(
            std::llround(static_cast<double>(amp) * std::sin(ph)));
    }

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

    const int32_t ac_i = static_cast<int32_t>(sI);
    const int32_t ac_q = static_cast<int32_t>(sQ);

    ProtectedEngine::HTS_CFO_Compensator cfo{};
    cfo.Init();
    cfo.Estimate_From_Autocorr(ac_i, ac_q, kLag);

    hts::rx_cfo::CFO_V5a v5a{};
    v5a.Init();
    v5a.Estimate_From_Autocorr(ac_i, ac_q, kLag);

    const double hz_c = cfo.Get_Est_Hz(kFs);
    const int32_t hz_v = v5a.GetLastCfoHz();
    std::printf(
        "Test3 inj_hz=%6d ac=(%d,%d) est_hz cfo=%.2f v5a=%d\n", injected_hz,
        ac_i, ac_q, hz_c, hz_v);
}

void test_apply_equivalence() {
    int16_t src_i[kChips]{};
    int16_t src_q[kChips]{};
    int16_t rot_i[kChips]{};
    int16_t rot_q[kChips]{};
    constexpr int32_t kHz = 1000;

    for (int n = 0; n < kChips; ++n) {
        src_i[n] = 1000;
        src_q[n] = 0;
        const double ph = 2.0 * kPi * static_cast<double>(kHz) *
                          static_cast<double>(n) / kFs;
        rot_i[n] = static_cast<int16_t>(std::llround(1000.0 * std::cos(ph)));
        rot_q[n] = static_cast<int16_t>(std::llround(1000.0 * std::sin(ph)));
    }

    int64_t sI = 0;
    int64_t sQ = 0;
    for (int n = 0; n + kLag < kChips; ++n) {
        const int32_t I0 = static_cast<int32_t>(rot_i[n]);
        const int32_t Q0 = static_cast<int32_t>(rot_q[n]);
        const int32_t I1 = static_cast<int32_t>(rot_i[n + kLag]);
        const int32_t Q1 = static_cast<int32_t>(rot_q[n + kLag]);
        sI += static_cast<int64_t>(I0) * I1 + static_cast<int64_t>(Q0) * Q1;
        sQ += static_cast<int64_t>(I0) * Q1 - static_cast<int64_t>(Q0) * I1;
    }
    const int32_t ac_i = static_cast<int32_t>(sI);
    const int32_t ac_q = static_cast<int32_t>(sQ);

    ProtectedEngine::HTS_CFO_Compensator cfo{};
    cfo.Init();
    cfo.Estimate_From_Autocorr(ac_i, ac_q, kLag);
    hts::rx_cfo::CFO_V5a v5a{};
    v5a.Init();
    v5a.Estimate_From_Autocorr(ac_i, ac_q, kLag);

    int16_t c_i[kChips]{};
    int16_t c_q[kChips]{};
    int16_t v_i[kChips]{};
    int16_t v_q[kChips]{};
    for (int n = 0; n < kChips; ++n) {
        c_i[n] = rot_i[n];
        c_q[n] = rot_q[n];
        v_i[n] = rot_i[n];
        v_q[n] = rot_q[n];
        cfo.Apply(c_i[n], c_q[n]);
        v5a.Apply_Per_Chip(v_i[n], v_q[n]);
    }

    int max_err_cfo = 0;
    int max_err_v5a = 0;
    int max_diff_cv = 0;
    for (int n = 0; n < kChips; ++n) {
        const int ec = std::abs(static_cast<int>(c_i[n]) - static_cast<int>(src_i[n])) +
                       std::abs(static_cast<int>(c_q[n]) - static_cast<int>(src_q[n]));
        const int ev = std::abs(static_cast<int>(v_i[n]) - static_cast<int>(src_i[n])) +
                       std::abs(static_cast<int>(v_q[n]) - static_cast<int>(src_q[n]));
        const int ed = std::abs(static_cast<int>(c_i[n]) - static_cast<int>(v_i[n])) +
                       std::abs(static_cast<int>(c_q[n]) - static_cast<int>(v_q[n]));
        if (ec > max_err_cfo) max_err_cfo = ec;
        if (ev > max_err_v5a) max_err_v5a = ev;
        if (ed > max_diff_cv) max_diff_cv = ed;
    }

    std::printf(
        "Test4 ApplyEq: ac=(%d,%d) max_err_to_src cfo=%d v5a=%d cfo_vs_v5a=%d\n",
        ac_i, ac_q, max_err_cfo, max_err_v5a, max_diff_cv);
}

void test_walsh_vs_holo_path() {
    constexpr int32_t kHz = 5000;
    int16_t walsh_i[128]{};
    int16_t walsh_q[128]{};
    for (int n = 0; n < 128; ++n) {
        const int8_t w = kWalsh63Row63[n & 63];
        const double base_i = static_cast<double>(w) * 1000.0;
        const double ph = 2.0 * kPi * static_cast<double>(kHz) *
                          static_cast<double>(n) / kFs;
        walsh_i[n] = static_cast<int16_t>(
            std::llround(base_i * std::cos(ph)));
        walsh_q[n] = static_cast<int16_t>(
            std::llround(base_i * std::sin(ph)));
    }

    hts::rx_cfo::CFO_V5a v_walsh{};
    v_walsh.Init();
    const auto res = v_walsh.Estimate(walsh_i, walsh_q);
    v_walsh.Set_Apply_Cfo(res.cfo_hz);

    int64_t sI = 0;
    int64_t sQ = 0;
    for (int n = 0; n + kLag < 128; ++n) {
        const int32_t I0 = static_cast<int32_t>(walsh_i[n]);
        const int32_t Q0 = static_cast<int32_t>(walsh_q[n]);
        const int32_t I1 = static_cast<int32_t>(walsh_i[n + kLag]);
        const int32_t Q1 = static_cast<int32_t>(walsh_q[n + kLag]);
        sI += static_cast<int64_t>(I0) * I1 + static_cast<int64_t>(Q0) * Q1;
        sQ += static_cast<int64_t>(I0) * Q1 - static_cast<int64_t>(Q0) * I1;
    }
    hts::rx_cfo::CFO_V5a v_holo{};
    v_holo.Init();
    v_holo.Estimate_From_Autocorr(static_cast<int32_t>(sI), static_cast<int32_t>(sQ), kLag);

    std::printf(
        "Test5 Path: walsh_hz=%d sin=%d cos=%d | holo_hz=%d sin=%d cos=%d\n",
        static_cast<int>(res.cfo_hz), v_walsh.Get_Apply_Sin_Per_Chip_Q14(),
        v_walsh.Get_Apply_Cos_Per_Chip_Q14(), v_holo.GetLastCfoHz(),
        v_holo.Get_Apply_Sin_Per_Chip_Q14(), v_holo.Get_Apply_Cos_Per_Chip_Q14());
}

}  // namespace

int main() {
    hts::rx_cfo::Build_SinCos_Table();

    test_perchip_signs_from_hz(1000);
    test_perchip_signs_from_hz(-1000);
    test_perchip_signs_from_hz(5000);

    test_apply_diff_from_hz(1000);
    test_apply_diff_from_hz(-1000);
    test_apply_diff_from_hz(5000);

    test_estimate_sign_from_signal(1000);
    test_estimate_sign_from_signal(-1000);
    test_estimate_sign_from_signal(5000);
    test_apply_equivalence();
    test_walsh_vs_holo_path();

    if (g_fail != 0) {
        std::printf("test_step7_signs: FAIL (%d)\n", g_fail);
        return 1;
    }
    std::printf("test_step7_signs: PASS\n");
    return 0;
}

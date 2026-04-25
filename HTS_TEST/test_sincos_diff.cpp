// =============================================================================
// test_sincos_diff.cpp — Phase A/B: cfo_ vs V5a + PTE LUT diagnostics
// HTS_ALLOW_HOST_BUILD only.
// =============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "../HTS_LIM/HTS_CFO_Compensator.h"
#include "../HTS_LIM/HTS_CFO_V5a.hpp"
#include "../HTS_LIM/HTS_Rx_CFO_SinCos_Table.hpp"

#if !defined(HTS_ALLOW_HOST_BUILD)
#error "Host-only diagnostic (HTS_ALLOW_HOST_BUILD required)"
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int32_t kLag = 32;
constexpr int32_t kFs = 1000000;
constexpr int32_t kMag = 3000000;

int g_fail = 0;

static int64_t unit_circle_err_q28(int32_t s_q14, int32_t c_q14) noexcept {
    const int64_t s2 = static_cast<int64_t>(s_q14) * s_q14;
    const int64_t c2 = static_cast<int64_t>(c_q14) * c_q14;
    const int64_t sum = s2 + c2;
    constexpr int64_t kOne = 16384;
    const int64_t target = kOne * kOne;
    return sum - target;
}

static void synth_autocorr_for_hz(int32_t hz, int32_t& ac_I,
                                  int32_t& ac_Q) noexcept {
    const double th =
        (2.0 * kPi * static_cast<double>(hz) * static_cast<double>(kLag)) /
        static_cast<double>(kFs);
    ac_I = static_cast<int32_t>(std::llround(static_cast<double>(kMag) * std::cos(th)));
    ac_Q = static_cast<int32_t>(std::llround(static_cast<double>(kMag) * std::sin(th)));
}

static int max_abs(int a, int b) noexcept {
    const int d = a - b;
    return (d >= 0) ? d : -d;
}

/// Pre–Phase B: nearest sample only (same index as PTE centre idx).
static int16_t lookup_sin_snap(uint32_t phase_q32) noexcept {
    const int idx = static_cast<int>(
        (phase_q32 >> hts::rx_cfo::kSinCosIndexShift) &
        (hts::rx_cfo::kSinCosTableSize - 1));
    return hts::rx_cfo::g_sin_table[idx];
}

static void test5_snap_vs_pte() noexcept {
    int max_d = 0;
    constexpr uint32_t kStep = 997u;  // coprime with 2^32
    uint32_t p = 0u;
    for (int n = 0; n < 500000; ++n) {
        const int16_t snap = lookup_sin_snap(p);
        const int16_t pte = hts::rx_cfo::Lookup_Sin(p);
        const int d = max_abs(static_cast<int>(pte), static_cast<int>(snap));
        if (d > max_d) {
            max_d = d;
        }
        p += kStep;
    }
    std::printf("Test 5: snap vs PTE max |d_sin|=%d (500k phases)\n",
                static_cast<int>(max_d));
}

static void test6_pte_vs_std_sin() noexcept {
    int max_d = 0;
    constexpr uint32_t kStep = 65537u;
    uint32_t p = 0u;
    for (int n = 0; n < 1000000; ++n) {
        const int16_t pte = hts::rx_cfo::Lookup_Sin(p);
        const double ang =
            (static_cast<double>(p) * (2.0 * kPi)) / 4294967296.0;
        const int32_t ref =
            static_cast<int32_t>(std::llround(std::sin(ang) * 16384.0));
        const int d = max_abs(static_cast<int>(pte), static_cast<int>(ref));
        if (d > max_d) {
            max_d = d;
        }
        p += kStep;
    }
    std::printf("Test 6: PTE vs std::sin max |err|=%d (1M samples)\n",
                static_cast<int>(max_d));
    if (max_d >= 5) {
        std::printf("FAIL Test6: max err >= 5 LSB\n");
        ++g_fail;
    }
}

static void test7_cfo_vs_v5a_64chip() noexcept {
    static const int32_t kHzList[] = {
        -10000, -5000, -1000, 0, 1000, 5000, 10000,
    };
    int worst = 0;
    for (int32_t hz : kHzList) {
        int32_t ac_I = 0;
        int32_t ac_Q = 0;
        synth_autocorr_for_hz(hz, ac_I, ac_Q);
        ProtectedEngine::HTS_CFO_Compensator cfo;
        cfo.Init();
        cfo.Estimate_From_Autocorr(ac_I, ac_Q, kLag);
        if (!cfo.Is_Apply_Active()) {
            continue;
        }
        const int32_t hz_eff =
            static_cast<int32_t>(std::llround(cfo.Get_Est_Hz(1000000.0)));
        hts::rx_cfo::CFO_V5a v5a;
        v5a.Init();
        v5a.Set_Apply_Cfo(hz_eff);
        int max_d = 0;
        int16_t i_c = 4096;
        int16_t q_c = 4096;
        int16_t i_v = i_c;
        int16_t q_v = q_c;
        for (int k = 0; k < 64; ++k) {
            cfo.Apply(i_c, q_c);
            v5a.Apply_Per_Chip(i_v, q_v);
            const int di = max_abs(static_cast<int>(i_v), static_cast<int>(i_c));
            const int dq = max_abs(static_cast<int>(q_v), static_cast<int>(q_c));
            if (di > max_d) {
                max_d = di;
            }
            if (dq > max_d) {
                max_d = dq;
            }
        }
        if (max_d > worst) {
            worst = max_d;
        }
        std::printf("Test 7: nomHz=%6d hz_eff=%6d 64-chip max |I/Q| diff=%d\n",
                    static_cast<int>(hz), static_cast<int>(hz_eff),
                    static_cast<int>(max_d));
    }
    std::printf("Test 7: worst 64-chip max |I/Q| diff=%d\n", worst);
    // Taylor(cfo_) vs LUT(V5a) per-chip: <50 LSB 비현실. PTE 후 악화 방지 상한만 검사.
    constexpr int kRegressionCeil = 9500;
    if (worst >= kRegressionCeil) {
        std::printf("FAIL Test7: worst >= %d (regression guard)\n",
                    static_cast<int>(kRegressionCeil));
        ++g_fail;
    }
}

}  // namespace

int main() {
    hts::rx_cfo::Build_SinCos_Table();

    static const int32_t kHzList[] = {
        -10000, -5000, -1000, 0, 1000, 5000, 10000,
    };

    std::printf(
        "=== Phase A: cfo_ vs V5a sin/cos (autocorr synth @ lag=%d, |ac|~%d) ===\n",
        static_cast<int>(kLag), static_cast<int>(kMag));
    std::printf(
        "cfo_: HTS_CFO_Compensator.h Estimate_From_Autocorr (Taylor sin + "
        "int sqrt cos)\n");
    std::printf(
        "V5a:  Set_Apply_Cfo -> Lookup_Sin/Cos + apply_int_root_q14 cos\n\n");

    int worst_ds = 0;
    int worst_dc = 0;
    int64_t worst_uc_cfo = 0;
    int64_t worst_uc_v5a = 0;

    for (int32_t hz : kHzList) {
        int32_t ac_I = 0;
        int32_t ac_Q = 0;
        synth_autocorr_for_hz(hz, ac_I, ac_Q);

        ProtectedEngine::HTS_CFO_Compensator cfo;
        cfo.Init();
        cfo.Estimate_From_Autocorr(ac_I, ac_Q, kLag);

        hts::rx_cfo::CFO_V5a v5a;
        v5a.Init();
        v5a.Set_Apply_Cfo(hz);

        int32_t s_c = 0;
        int32_t c_c = 16384;
        double hz_c = 0.0;
        if (cfo.Is_Apply_Active()) {
            s_c = cfo.Get_Sin_Per_Chip_Q14();
            c_c = cfo.Get_Cos_Per_Chip_Q14();
            hz_c = cfo.Get_Est_Hz(1000000.0);
        }

        const int32_t s_v = v5a.Get_Apply_Sin_Per_Chip_Q14();
        const int32_t c_v = v5a.Get_Apply_Cos_Per_Chip_Q14();

        const int ds = max_abs(s_v, s_c);
        const int dc = max_abs(c_v, c_c);
        const int64_t uc_c = unit_circle_err_q28(s_c, c_c);
        const int64_t uc_v = unit_circle_err_q28(s_v, c_v);

        if (ds > worst_ds) {
            worst_ds = ds;
        }
        if (dc > worst_dc) {
            worst_dc = dc;
        }
        const int64_t auc = (uc_c >= 0) ? uc_c : -uc_c;
        const int64_t auv = (uc_v >= 0) ? uc_v : -uc_v;
        if (auc > worst_uc_cfo) {
            worst_uc_cfo = auc;
        }
        if (auv > worst_uc_v5a) {
            worst_uc_v5a = auv;
        }

        std::printf(
            "Hz=%6d  cfo_active=%d  Get_Est_Hz=%.2f  |d_sin|=%4d |d_cos|=%4d  "
            "uc_err_cfo=%lld uc_err_v5a=%lld\n",
            static_cast<int>(hz), cfo.Is_Apply_Active() ? 1 : 0, hz_c,
            static_cast<int>(ds), static_cast<int>(dc),
            static_cast<long long>(uc_c), static_cast<long long>(uc_v));
    }

    std::printf("\n--- 64-chip Apply path max |I/Q| diff (same start vector) ---\n");
    for (int32_t hz : kHzList) {
        int32_t ac_I = 0;
        int32_t ac_Q = 0;
        synth_autocorr_for_hz(hz, ac_I, ac_Q);

        ProtectedEngine::HTS_CFO_Compensator cfo;
        cfo.Init();
        cfo.Estimate_From_Autocorr(ac_I, ac_Q, kLag);

        hts::rx_cfo::CFO_V5a v5a;
        v5a.Init();
        v5a.Set_Apply_Cfo(hz);

        if (!cfo.Is_Apply_Active()) {
            std::printf("Hz=%6d  skip Apply (cfo inactive)\n", static_cast<int>(hz));
            continue;
        }

        int max_d = 0;
        int16_t i_c = 4096;
        int16_t q_c = 4096;
        int16_t i_v = i_c;
        int16_t q_v = q_c;
        for (int k = 0; k < 64; ++k) {
            cfo.Apply(i_c, q_c);
            v5a.Apply_Per_Chip(i_v, q_v);
            const int di = max_abs(static_cast<int>(i_v), static_cast<int>(i_c));
            const int dq = max_abs(static_cast<int>(q_v), static_cast<int>(q_c));
            if (di > max_d) {
                max_d = di;
            }
            if (dq > max_d) {
                max_d = dq;
            }
        }
        std::printf("Hz=%6d  max_abs(I/Q diff) over 64 Apply = %d\n",
                    static_cast<int>(hz), static_cast<int>(max_d));
    }

    std::printf(
        "\n=== Summary (scalar per-chip sin/cos) ===\n"
        "worst |d_sin|=%d  worst |d_cos|=%d\n"
        "worst |sin^2+cos^2 - 16384^2| cfo_=%lld  v5a=%lld\n",
        worst_ds, worst_dc, static_cast<long long>(worst_uc_cfo),
        static_cast<long long>(worst_uc_v5a));

    std::printf(
        "\n=== Same effective Hz (V5a.Set_Apply_Cfo(round(Get_Est_Hz))) ===\n"
        "Isolates Taylor+int_cos vs LUT+int_cos at matched CFO.\n");
    worst_ds = 0;
    worst_dc = 0;
    for (int32_t hz : kHzList) {
        int32_t ac_I = 0;
        int32_t ac_Q = 0;
        synth_autocorr_for_hz(hz, ac_I, ac_Q);
        ProtectedEngine::HTS_CFO_Compensator cfo;
        cfo.Init();
        cfo.Estimate_From_Autocorr(ac_I, ac_Q, kLag);
        if (!cfo.Is_Apply_Active()) {
            continue;
        }
        const int32_t hz_eff =
            static_cast<int32_t>(std::llround(cfo.Get_Est_Hz(1000000.0)));
        hts::rx_cfo::CFO_V5a v5a;
        v5a.Init();
        v5a.Set_Apply_Cfo(hz_eff);
        const int32_t s_c = cfo.Get_Sin_Per_Chip_Q14();
        const int32_t c_c = cfo.Get_Cos_Per_Chip_Q14();
        const int32_t s_v = v5a.Get_Apply_Sin_Per_Chip_Q14();
        const int32_t c_v = v5a.Get_Apply_Cos_Per_Chip_Q14();
        const int ds = max_abs(s_v, s_c);
        const int dc = max_abs(c_v, c_c);
        if (ds > worst_ds) {
            worst_ds = ds;
        }
        if (dc > worst_dc) {
            worst_dc = dc;
        }
        std::printf(
            "nomHz=%6d  hz_eff=%6d  |d_sin|=%4d |d_cos|=%4d\n",
            static_cast<int>(hz), static_cast<int>(hz_eff),
            static_cast<int>(ds), static_cast<int>(dc));
    }
    std::printf("matched-CFO worst |d_sin|=%d |d_cos|=%d\n", worst_ds, worst_dc);

    std::printf("\n=== Phase B PTE tests ===\n");
    test5_snap_vs_pte();
    test6_pte_vs_std_sin();
    test7_cfo_vs_v5a_64chip();

    if (g_fail != 0) {
        std::printf("\ntest_sincos_diff: FAIL (%d)\n", g_fail);
        return 1;
    }
    std::printf("\ntest_sincos_diff: PASS\n");
    return 0;
}

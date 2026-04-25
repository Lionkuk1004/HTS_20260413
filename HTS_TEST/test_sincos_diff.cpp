// =============================================================================
// test_sincos_diff.cpp — Phase A: cfo_ (Taylor+int cos) vs V5a (LUT+int cos)
// HTS_ALLOW_HOST_BUILD only. No production code changes.
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
constexpr int32_t kMag = 3000000;  // |ac| large enough for mag² ≥ 1e6 gate

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

    std::printf("\nPhase A done. See repo docs / commit message for cited lines.\n");
    return 0;
}

#include <cstdint>
#include <cstdio>

#include "../HTS_LIM/HTS_CFO_Compensator.h"
#include "../HTS_LIM/HTS_CFO_V5a.hpp"
#include "../HTS_LIM/HTS_Rx_CFO_SinCos_Table.hpp"

#if !defined(HTS_ALLOW_HOST_BUILD)
#error "Host-only diagnostic test (HTS_ALLOW_HOST_BUILD required)"
#endif

namespace {

struct AcCase {
    const char* name;
    int32_t ac_i;
    int32_t ac_q;
};

int abs_i32(int v) {
    return (v < 0) ? -v : v;
}

int g_fail = 0;

void run_case(const AcCase& tc) {
    constexpr int32_t kLag = 32;
    constexpr int16_t kSeedI = 1100;
    constexpr int16_t kSeedQ = -700;
    constexpr int kChips = 64;

    ProtectedEngine::HTS_CFO_Compensator cfo{};
    cfo.Init();
    cfo.Estimate_From_Autocorr(tc.ac_i, tc.ac_q, kLag);

    hts::rx_cfo::CFO_V5a v5a{};
    v5a.Init();
    v5a.Estimate_From_Autocorr(tc.ac_i, tc.ac_q, kLag);

    const bool cfo_active = cfo.Is_Apply_Active();
    const bool v5a_allowed = v5a.IsApplyAllowed();

    const int cfo_sin = cfo.Get_Sin_Per_Chip_Q14();
    const int cfo_cos = cfo.Get_Cos_Per_Chip_Q14();
    const int v5a_sin = v5a.Get_Apply_Sin_Per_Chip_Q14();
    const int v5a_cos = v5a.Get_Apply_Cos_Per_Chip_Q14();

    int max_apply_diff = 0;
    int sum_apply_diff = 0;

    int16_t ci = kSeedI;
    int16_t cq = kSeedQ;
    int16_t vi = kSeedI;
    int16_t vq = kSeedQ;
    for (int n = 0; n < kChips; ++n) {
        cfo.Apply(ci, cq);
        v5a.Apply_Per_Chip(vi, vq);
        const int d = abs_i32(static_cast<int>(ci) - static_cast<int>(vi)) +
                      abs_i32(static_cast<int>(cq) - static_cast<int>(vq));
        if (d > max_apply_diff) {
            max_apply_diff = d;
        }
        sum_apply_diff += d;
    }

    // Step 6 Holo path: cfo drives active + V5a copies cfo sin/cos and advances 192 chips.
    ProtectedEngine::HTS_CFO_Compensator cfo_s6{};
    cfo_s6.Init();
    hts::rx_cfo::CFO_V5a v5a_s6{};
    v5a_s6.Init();
    cfo_s6.Estimate_From_Autocorr(tc.ac_i, tc.ac_q, kLag);
    if (cfo_s6.Is_Apply_Active()) {
        v5a_s6.Estimate_From_Autocorr(tc.ac_i, tc.ac_q, kLag);
        v5a_s6.Set_Apply_SinCosPerChip_Q14(cfo_s6.Get_Sin_Per_Chip_Q14(),
                                            cfo_s6.Get_Cos_Per_Chip_Q14());
        v5a_s6.Advance_Phase_Only(192);
    } else {
        v5a_s6.Set_Apply_Cfo(0);
    }

    int apply_calls_s6 = 0;
    int16_t s6i = kSeedI;
    int16_t s6q = kSeedQ;
    for (int n = 0; n < kChips; ++n) {
        if (cfo_s6.Is_Apply_Active() || v5a_s6.IsApplyDriveActive()) {
            ++apply_calls_s6;
            v5a_s6.Apply_Per_Chip(s6i, s6q);
        }
    }

    // Step 7 attempted path: no cfo_.Estimate_From_Autocorr, V5a-only gate + advance.
    ProtectedEngine::HTS_CFO_Compensator cfo_s7{};
    cfo_s7.Init();
    hts::rx_cfo::CFO_V5a v5a_s7{};
    v5a_s7.Init();
    v5a_s7.Estimate_From_Autocorr(tc.ac_i, tc.ac_q, kLag);
    if (v5a_s7.IsApplyAllowed()) {
        v5a_s7.Advance_Phase_Only(192);
    } else {
        v5a_s7.Set_Apply_Cfo(0);
    }

    int apply_calls_s7 = 0;
    int16_t s7i = kSeedI;
    int16_t s7q = kSeedQ;
    int max_s6_vs_s7 = 0;
    int sum_s6_vs_s7 = 0;
    for (int n = 0; n < kChips; ++n) {
        if (cfo_s7.Is_Apply_Active() || v5a_s7.IsApplyDriveActive()) {
            ++apply_calls_s7;
            v5a_s7.Apply_Per_Chip(s7i, s7q);
        }
        const int d = abs_i32(static_cast<int>(s6i) - static_cast<int>(s7i)) +
                      abs_i32(static_cast<int>(s6q) - static_cast<int>(s7q));
        if (d > max_s6_vs_s7) {
            max_s6_vs_s7 = d;
        }
        sum_s6_vs_s7 += d;
    }

    std::printf(
        "[%s] ac=(%d,%d) cfo_active=%d v5a_allowed=%d "
        "sin/cos cfo=(%d,%d) v5a=(%d,%d) ds=%d dc=%d\n",
        tc.name, tc.ac_i, tc.ac_q, cfo_active ? 1 : 0, v5a_allowed ? 1 : 0,
        cfo_sin, cfo_cos, v5a_sin, v5a_cos, abs_i32(cfo_sin - v5a_sin),
        abs_i32(cfo_cos - v5a_cos));
    std::printf(
        "    apply64 cfo_vs_v5a: max=%d mean=%.2f | "
        "step6_calls=%d step7_calls=%d step6_vs_step7(max=%d mean=%.2f)\n",
        max_apply_diff, static_cast<double>(sum_apply_diff) / kChips,
        apply_calls_s6, apply_calls_s7, max_s6_vs_s7,
        static_cast<double>(sum_s6_vs_s7) / kChips);

    if (cfo_active != v5a_allowed) {
        std::printf("    FAIL: gate mismatch (cfo_active != v5a_allowed)\n");
        ++g_fail;
    }
}

}  // namespace

int main() {
    hts::rx_cfo::Build_SinCos_Table();

    const AcCase cases[] = {
        {"near-threshold-low", 700, 700},           // mag2 < 1e6 expected off
        {"near-threshold-high", 1000, 1000},        // mag2 >= 1e6 expected on
        {"holo-1", 3000000, 1500000},
        {"holo-2", 31846410, 50206566},
        {"holo-3", -23653299, 50253392},
        {"holo-4", 49526551, 51434324},
    };

    for (const auto& tc : cases) {
        run_case(tc);
    }

    if (g_fail != 0) {
        std::printf("test_step7_diag: FAIL (%d)\n", g_fail);
        return 1;
    }
    std::printf("test_step7_diag: PASS\n");
    return 0;
}

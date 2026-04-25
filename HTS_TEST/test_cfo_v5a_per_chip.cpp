// =============================================================================
// test_cfo_v5a_per_chip.cpp — CFO_V5a per-chip Q14 path vs batch / compensator
// Step 1 host UT (HTS_ALLOW_HOST_BUILD)
// =============================================================================
#include <algorithm>
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

int g_fail = 0;

void test_identity_zero() noexcept {
    constexpr int N = 48;
    int16_t inI[N]{};
    int16_t inQ[N]{};
    for (int i = 0; i < N; ++i) {
        inI[i] = static_cast<int16_t>((i * 19 - 2000) & 0x7FFF);
        inQ[i] = static_cast<int16_t>((i * 37 + 900) & 0x7FFF);
    }
    hts::rx_cfo::CFO_V5a v;
    v.Init();
    v.Set_Apply_Cfo(0);
    for (int k = 0; k < N; ++k) {
        int16_t aI = inI[k];
        int16_t aQ = inQ[k];
        v.Apply_Per_Chip(aI, aQ);
        if (aI != inI[k] || aQ != inQ[k]) {
            std::printf("FAIL: Test1 identity k=%d\n", k);
            ++g_fail;
            return;
        }
    }
    std::printf("Test 1 PASS: Set_Apply_Cfo(0) identity\n");
}

void test_1khz_determinism() noexcept {
    constexpr int N = 64;
    constexpr int32_t kHz = 1000;
    int16_t inI[N]{};
    int16_t inQ[N]{};
    for (int i = 0; i < N; ++i) {
        inI[i] = static_cast<int16_t>(1500 + (i * 113) % 1200);
        inQ[i] = static_cast<int16_t>(-900 + (i * 91) % 800);
    }
    hts::rx_cfo::Build_SinCos_Table();
    int16_t o1I[N]{};
    int16_t o1Q[N]{};
    int16_t o2I[N]{};
    int16_t o2Q[N]{};
    for (int pass = 0; pass < 2; ++pass) {
        hts::rx_cfo::CFO_V5a v;
        v.Init();
        v.Set_Apply_Cfo(kHz);
        for (int k = 0; k < N; ++k) {
            int16_t aI = inI[k];
            int16_t aQ = inQ[k];
            v.Apply_Per_Chip(aI, aQ);
            if (pass == 0) {
                o1I[k] = aI;
                o1Q[k] = aQ;
            } else {
                o2I[k] = aI;
                o2Q[k] = aQ;
            }
        }
    }
    int max_abs = 0;
    int any_change = 0;
    for (int k = 0; k < N; ++k) {
        max_abs = (std::max)(max_abs,
                              std::abs(static_cast<int>(o1I[k]) -
                                       static_cast<int>(o2I[k])));
        max_abs = (std::max)(max_abs,
                              std::abs(static_cast<int>(o1Q[k]) -
                                       static_cast<int>(o2Q[k])));
        if (o1I[k] != inI[k] || o1Q[k] != inQ[k]) {
            any_change = 1;
        }
    }
    if (max_abs != 0) {
        std::printf("FAIL: Test2 determinism max err=%d\n", max_abs);
        ++g_fail;
    } else if (any_change == 0) {
        std::printf("FAIL: Test2 no rotation at 1 kHz\n");
        ++g_fail;
    } else {
        std::printf("Test 2 PASS: 1 kHz determinism + non-identity\n");
    }
}

void test_advance_192_vs_dummy_chips() noexcept {
    constexpr int32_t kHz = 2500;
    const int16_t chipI = 2100;
    const int16_t chipQ = -1300;
    hts::rx_cfo::CFO_V5a vA;
    vA.Init();
    hts::rx_cfo::Build_SinCos_Table();
    vA.Set_Apply_Cfo(kHz);
    vA.Advance_Phase_Only(192);
    int16_t oI = chipI;
    int16_t oQ = chipQ;
    vA.Apply_Per_Chip(oI, oQ);

    hts::rx_cfo::CFO_V5a vB;
    vB.Init();
    vB.Set_Apply_Cfo(kHz);
    for (int k = 0; k < 192; ++k) {
        int16_t dI = 1;
        int16_t dQ = 0;
        vB.Apply_Per_Chip(dI, dQ);
    }
    int16_t o2I = chipI;
    int16_t o2Q = chipQ;
    vB.Apply_Per_Chip(o2I, o2Q);

    const int d1 = std::abs(static_cast<int>(oI) - static_cast<int>(o2I));
    const int d2 = std::abs(static_cast<int>(oQ) - static_cast<int>(o2Q));
    if (d1 != 0 || d2 != 0) {
        std::printf("FAIL: Test3 advance192 vs 192 dummy (%d,%d)\n", d1, d2);
        ++g_fail;
    } else {
        std::printf("Test 3 PASS: Advance_Phase_Only(192) vs 192 Apply\n");
    }
}

void test_vs_compensator_apply() noexcept {
    ProtectedEngine::HTS_CFO_Compensator cfo;
    cfo.Init();
    // mag² ≥ 1e6 활성화 (HTS_CFO_Compensator::Estimate_From_Autocorr)
    cfo.Estimate_From_Autocorr(3000000, 1500000, 32);
    if (!cfo.Is_Apply_Active()) {
        std::printf("FAIL: Test4 compensator not active\n");
        ++g_fail;
        return;
    }
    hts::rx_cfo::CFO_V5a v;
    v.Init();
    hts::rx_cfo::Build_SinCos_Table();
    v.Set_Apply_SinCosPerChip_Q14(cfo.Get_Sin_Per_Chip_Q14(),
                                  cfo.Get_Cos_Per_Chip_Q14());

    constexpr int N = 96;
    int max_abs = 0;
    for (int t = 0; t < N; ++t) {
        int16_t cI = static_cast<int16_t>(800 + (t * 97) % 600);
        int16_t cQ = static_cast<int16_t>(-400 + (t * 53) % 500);
        int16_t vI = cI;
        int16_t vQ = cQ;
        cfo.Apply(cI, cQ);
        v.Apply_Per_Chip(vI, vQ);
        max_abs = (std::max)(max_abs,
                              std::abs(static_cast<int>(cI) -
                                       static_cast<int>(vI)));
        max_abs = (std::max)(max_abs,
                              std::abs(static_cast<int>(cQ) -
                                       static_cast<int>(vQ)));
    }
    if (max_abs > 10) {
        std::printf("FAIL: Test4 vs cfo_.Apply max err=%d\n", max_abs);
        ++g_fail;
    } else {
        std::printf("Test 4 PASS: vs cfo_.Apply max err=%d\n", max_abs);
    }
}

}  // namespace

int main() {
    hts::rx_cfo::Build_SinCos_Table();
    test_identity_zero();
    test_1khz_determinism();
    test_advance_192_vs_dummy_chips();
    test_vs_compensator_apply();
    if (g_fail != 0) {
        std::printf("test_cfo_v5a_per_chip: %d failure(s)\n", g_fail);
        return 1;
    }
    std::printf("test_cfo_v5a_per_chip: PASS\n");
    return 0;
}

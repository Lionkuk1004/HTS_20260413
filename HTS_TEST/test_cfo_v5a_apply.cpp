// =============================================================================
// test_cfo_v5a_apply.cpp — CFO V5a ApplyDerotate batch (Phase 2-1 host UT)
// =============================================================================
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "HTS_CFO_V5a.hpp"
#include "HTS_Rx_CFO_SinCos_Table.hpp"

#if !defined(HTS_ALLOW_HOST_BUILD)
#error "Host-only unit test (HTS_ALLOW_HOST_BUILD required)"
#endif

namespace {

int g_fail = 0;

void rot_ideal(const int16_t* inI, const int16_t* inQ, int n, int32_t cfo_hz,
               int16_t* outI, int16_t* outQ) noexcept {
    static constexpr double kPi = 3.14159265358979323846;
    const double inc =
        2.0 * kPi * static_cast<double>(cfo_hz) /
        static_cast<double>(hts::rx_cfo::kChipRateHz);
    double ph = 0.0;
    for (int k = 0; k < n; ++k) {
        const double c = std::cos(ph);
        const double s = std::sin(ph);
        const double xi = static_cast<double>(inI[k]);
        const double yq = static_cast<double>(inQ[k]);
        const double r0 = xi * c - yq * s;
        const double i0 = xi * s + yq * c;
        const long long lr = std::llround(r0);
        const long long li = std::llround(i0);
        outI[k] = (lr > 32767)   ? static_cast<int16_t>(32767)
                  : (lr < -32768) ? static_cast<int16_t>(-32768)
                                  : static_cast<int16_t>(lr);
        outQ[k] = (li > 32767)   ? static_cast<int16_t>(32767)
                  : (li < -32768) ? static_cast<int16_t>(-32768)
                                  : static_cast<int16_t>(li);
        ph += inc;
    }
}

void test_identity_zero() noexcept {
    constexpr int N = 128;
    int16_t inI[N]{};
    int16_t inQ[N]{};
    int16_t outI[N]{};
    int16_t outQ[N]{};
    for (int i = 0; i < N; ++i) {
        inI[i] = static_cast<int16_t>((i * 19 - 2000) & 0x7FFF);
        inQ[i] = static_cast<int16_t>((i * 37 + 900) & 0x7FFF);
    }
    hts::rx_cfo::CFO_V5a cfo;
    cfo.Init();
    cfo.ApplyDerotate(inI, inQ, outI, outQ, N, 0);
    for (int k = 0; k < N; ++k) {
        if (outI[k] != inI[k] || outQ[k] != inQ[k]) {
            std::printf("FAIL: identity CFO=0 at k=%d\n", k);
            ++g_fail;
            return;
        }
    }
    std::printf("Test 1 PASS: CFO=0 identity\n");
}

void test_roundtrip_1khz() noexcept {
    constexpr int N = 64;
    constexpr int32_t kHz = 1000;
    int16_t inI[N]{};
    int16_t inQ[N]{};
    int16_t midI[N]{};
    int16_t midQ[N]{};
    int16_t outI[N]{};
    int16_t outQ[N]{};
    for (int i = 0; i < N; ++i) {
        inI[i] = static_cast<int16_t>(1500 + (i * 113) % 1200);
        inQ[i] = static_cast<int16_t>(-900 + (i * 91) % 800);
    }
    hts::rx_cfo::CFO_V5a cfo;
    cfo.Init();
    hts::rx_cfo::Build_SinCos_Table();
    rot_ideal(inI, inQ, N, kHz, midI, midQ);
    // +kHz 채널 회전 제거: Derotate(+kHz) (Phase 1-3 derotate 부호 규약과 동일)
    cfo.ApplyDerotate(midI, midQ, outI, outQ, N, kHz);
    int max_abs = 0;
    for (int k = 0; k < N; ++k) {
        max_abs = (std::max)(max_abs,
                              std::abs(static_cast<int>(outI[k]) -
                                       static_cast<int>(inI[k])));
        max_abs = (std::max)(max_abs,
                              std::abs(static_cast<int>(outQ[k]) -
                                       static_cast<int>(inQ[k])));
    }
    if (max_abs > 40) {
        std::printf("FAIL: round-trip 1 kHz max err=%d\n", max_abs);
        ++g_fail;
    } else {
        std::printf("Test 2 PASS: round-trip 1 kHz max err=%d\n", max_abs);
    }
}

void test_lengths() noexcept {
    constexpr int kMax = 256;
    int16_t inI[kMax]{};
    int16_t inQ[kMax]{};
    int16_t outI[kMax]{};
    int16_t outQ[kMax]{};
    const int lens[] = {64, 128, 256};
    hts::rx_cfo::Build_SinCos_Table();
    for (int li = 0; li < 3; ++li) {
        const int len = lens[li];
        hts::rx_cfo::CFO_V5a cfo;
        cfo.Init();
        for (int i = 0; i < len; ++i) {
            inI[i] = static_cast<int16_t>((i * 17 - 1000) & 0x7FFF);
            inQ[i] = static_cast<int16_t>((i * 31 + 500) & 0x7FFF);
        }
        cfo.ApplyDerotate(inI, inQ, outI, outQ, len, 0);
        for (int k = 0; k < len; ++k) {
            if (outI[k] != inI[k] || outQ[k] != inQ[k]) {
                std::printf("FAIL: length %d identity at k=%d\n", len, k);
                ++g_fail;
                return;
            }
        }
    }
    std::printf("Test 3 PASS: lengths 64/128/256 CFO=0\n");
}

void test_determinism() noexcept {
    constexpr int N = 96;
    int16_t inI[N]{};
    int16_t inQ[N]{};
    int16_t o1I[N]{};
    int16_t o1Q[N]{};
    int16_t o2I[N]{};
    int16_t o2Q[N]{};
    for (int i = 0; i < N; ++i) {
        inI[i] = static_cast<int16_t>(500 + i * 11);
        inQ[i] = static_cast<int16_t>(-300 + i * 7);
    }
    hts::rx_cfo::CFO_V5a cfo;
    cfo.Init();
    hts::rx_cfo::Build_SinCos_Table();
    cfo.ApplyDerotate(inI, inQ, o1I, o1Q, N, 2500);
    cfo.ApplyDerotate(inI, inQ, o2I, o2Q, N, 2500);
    for (int k = 0; k < N; ++k) {
        if (o1I[k] != o2I[k] || o1Q[k] != o2Q[k]) {
            std::printf("FAIL: determinism at k=%d\n", k);
            ++g_fail;
            return;
        }
    }
    std::printf("Test 4 PASS: determinism\n");
}

}  // namespace

int main() {
    hts::rx_cfo::Build_SinCos_Table();
    test_identity_zero();
    test_roundtrip_1khz();
    test_lengths();
    test_determinism();
    if (g_fail != 0) {
        std::printf("test_cfo_v5a_apply: %d failure(s)\n", g_fail);
        return 1;
    }
    std::printf("test_cfo_v5a_apply: PASS\n");
    return 0;
}

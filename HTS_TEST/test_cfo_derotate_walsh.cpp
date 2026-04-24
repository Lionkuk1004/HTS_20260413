// =============================================================================
// test_cfo_derotate_walsh.cpp — INNOViD HTS CFO V5a Phase 1-3 unit tests (host)
// =============================================================================
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>

#include "HTS_CFO_V5a.hpp"
#include "HTS_CFO_V5a_TestExport.hpp"
#include "HTS_Rx_CFO_SinCos_Table.hpp"

#if !defined(HTS_ALLOW_HOST_BUILD)
#error "Host-only unit test (HTS_ALLOW_HOST_BUILD required)"
#endif

namespace {

int g_fail = 0;

void fail(const char* msg) noexcept {
    std::printf("FAIL: %s\n", msg);
    ++g_fail;
}

static constexpr int8_t kW63[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1};

void test_derotate_identity() noexcept {
    constexpr int N = 128;
    int16_t inI[N]{};
    int16_t inQ[N]{};
    int16_t outI[N]{};
    int16_t outQ[N]{};
    for (int i = 0; i < N; ++i) {
        inI[i] = static_cast<int16_t>((i * 17 - 1000) & 0x7FFF);
        inQ[i] = static_cast<int16_t>((i * 31 + 500) & 0x7FFF);
    }
    hts::rx_cfo::Build_SinCos_Table();
    hts::rx_cfo::test_export::Derotate_Table(inI, inQ, outI, outQ, N, 0);
    for (int k = 0; k < N; ++k) {
        if (outI[k] != inI[k] || outQ[k] != inQ[k]) {
            fail("Derotate CFO=0 must be identity per chip");
            return;
        }
    }
}

void test_derotate_roundtrip_5khz() noexcept {
    constexpr int N = 256;
    constexpr int32_t kCfo = 5000;
    int16_t inI[N]{};
    int16_t inQ[N]{};
    int16_t midI[N]{};
    int16_t midQ[N]{};
    int16_t outI[N]{};
    int16_t outQ[N]{};
    for (int i = 0; i < N; ++i) {
        inI[i] = static_cast<int16_t>(3000 + ((i * 131) % 2000));
        inQ[i] = static_cast<int16_t>(-2000 + ((i * 97) % 1800));
    }
    hts::rx_cfo::Build_SinCos_Table();
    // Firmware-consistent synthetic +5 kHz phase ramp, then inverse (see phase_inc in
    // HTS_CFO_V5a.cpp: cfo=-kCfo applies exp(+jω), cfo=+kCfo applies exp(-jω)).
    hts::rx_cfo::test_export::Derotate_Table(inI, inQ, midI, midQ, N, -kCfo);
    hts::rx_cfo::test_export::Derotate_Table(midI, midQ, outI, outQ, N, +kCfo);
    int max_abs = 0;
    for (int k = 0; k < N; ++k) {
        max_abs = (std::max)(max_abs, std::abs(static_cast<int>(outI[k]) - static_cast<int>(inI[k])));
        max_abs = (std::max)(max_abs, std::abs(static_cast<int>(outQ[k]) - static_cast<int>(inQ[k])));
    }
    // Two passes × Q14 1024 LUT × int16: ~30–35 LSB typical (not additive with chip count).
    if (max_abs > 40) {
        std::printf("FAIL: CFO 5 kHz round-trip max abs err=%d (allowed 40 LSB)\n", max_abs);
        ++g_fail;
    }
}

void test_walsh63_peak() noexcept {
    int16_t rI[64]{};
    int16_t rQ[64]{};
    for (int j = 0; j < 64; ++j) {
        rI[j] = static_cast<int16_t>(2000 * static_cast<int32_t>(kW63[j]));
        rQ[j] = 0;
    }
    int32_t dI = 0;
    int32_t dQ = 0;
    hts::rx_cfo::Build_SinCos_Table();
    hts::rx_cfo::test_export::Walsh63_Dot_Table(rI, rQ, dI, dQ);
    if (dI != 128000) {
        std::printf("FAIL: Walsh63 dI=%d expected 128000\n", static_cast<int>(dI));
        ++g_fail;
    }
    if (dQ != 0) {
        std::printf("FAIL: Walsh63 dQ=%d expected 0\n", static_cast<int>(dQ));
        ++g_fail;
    }
}

void test_energy_multiframe() noexcept {
    constexpr int N = 128;
    int16_t preI[N]{};
    int16_t preQ[N]{};
    int16_t noiseI[N]{};
    int16_t noiseQ[N]{};
    for (int sym = 0; sym < 2; ++sym) {
        const int base = sym * 64;
        for (int j = 0; j < 64; ++j) {
            preI[base + j] =
                static_cast<int16_t>(2000 * static_cast<int32_t>(kW63[j]));
            preQ[base + j] = 0;
        }
    }
    std::mt19937 rng(12345u);
    std::uniform_int_distribution<int> dist(-120, 120);
    for (int k = 0; k < N; ++k) {
        noiseI[k] = static_cast<int16_t>(dist(rng));
        noiseQ[k] = static_cast<int16_t>(dist(rng));
    }
    hts::rx_cfo::Build_SinCos_Table();
    const int64_t e_pre =
        hts::rx_cfo::test_export::Energy_Multiframe_Table(preI, preQ);
    const int64_t e_noise =
        hts::rx_cfo::test_export::Energy_Multiframe_Table(noiseI, noiseQ);
    if (e_pre <= 0) {
        fail("Preamble energy must be > 0");
    }
    if (e_noise >= e_pre) {
        std::printf("FAIL: noise energy %lld >= preamble %lld\n",
                    static_cast<long long>(e_noise),
                    static_cast<long long>(e_pre));
        ++g_fail;
    }
}

}  // namespace

int main() {
    test_derotate_identity();
    test_derotate_roundtrip_5khz();
    test_walsh63_peak();
    test_energy_multiframe();
    if (g_fail != 0) {
        std::printf("test_cfo_derotate_walsh: %d failure(s)\n", g_fail);
        return 1;
    }
    std::printf("test_cfo_derotate_walsh: PASS\n");
    return 0;
}

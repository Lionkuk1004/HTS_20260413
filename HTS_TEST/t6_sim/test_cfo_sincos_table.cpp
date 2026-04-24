// =============================================================================
// test_cfo_sincos_table.cpp — INNOViD HTS Rx CFO Q14 sin/cos table unit test
// =============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "HTS_Rx_CFO_SinCos_Table.hpp"

using hts::rx_cfo::Build_SinCos_Table;
using hts::rx_cfo::g_cos_table;
using hts::rx_cfo::g_sin_table;
using hts::rx_cfo::kQ14One;
using hts::rx_cfo::kSinCosIndexShift;
using hts::rx_cfo::kSinCosTableSize;
using hts::rx_cfo::Lookup_Cos;
using hts::rx_cfo::Lookup_Sin;

namespace {

int g_fail = 0;

void fail(const char* msg) noexcept {
    std::printf("FAIL: %s\n", msg);
    ++g_fail;
}

static inline int32_t ref_q14_sin(double ang) noexcept {
    return static_cast<int32_t>(
        std::llround(std::sin(ang) * static_cast<double>(kQ14One)));
}
static inline int32_t ref_q14_cos(double ang) noexcept {
    return static_cast<int32_t>(
        std::llround(std::cos(ang) * static_cast<double>(kQ14One)));
}

void test_cardinal_angles() noexcept {
    Build_SinCos_Table();

    if (g_sin_table[0] != 0) {
        fail("sin 0° (idx 0) must be 0");
    }
    {
        const int32_t d = static_cast<int32_t>(g_cos_table[0]) - kQ14One;
        if (d < -1 || d > 1) {
            fail("cos 0° within 1 LSB of +1.0 Q14");
        }
    }

    const int i90 = kSinCosTableSize / 4;
    {
        const int32_t d = static_cast<int32_t>(g_sin_table[i90]) - kQ14One;
        if (d < -1 || d > 1) {
            fail("sin 90° within 1 LSB of +1.0 Q14");
        }
    }
    {
        const int32_t d = static_cast<int32_t>(g_cos_table[i90]) - 0;
        if (d < -1 || d > 1) {
            fail("cos 90° within 1 LSB of 0");
        }
    }

    const int i180 = kSinCosTableSize / 2;
    {
        const int32_t d = static_cast<int32_t>(g_sin_table[i180]) - 0;
        if (d < -1 || d > 1) {
            fail("sin 180° within 1 LSB of 0");
        }
    }
    {
        const int32_t d = static_cast<int32_t>(g_cos_table[i180]) + kQ14One;
        if (d < -1 || d > 1) {
            fail("cos 180° within 1 LSB of -1.0 Q14");
        }
    }

    const int i270 = (kSinCosTableSize * 3) / 4;
    {
        const int32_t d = static_cast<int32_t>(g_sin_table[i270]) + kQ14One;
        if (d < -1 || d > 1) {
            fail("sin 270° within 1 LSB of -1.0 Q14");
        }
    }
    {
        const int32_t d = static_cast<int32_t>(g_cos_table[i270]) - 0;
        if (d < -1 || d > 1) {
            fail("cos 270° within 1 LSB of 0");
        }
    }

}

void test_vs_std_all_indices() noexcept {
    static constexpr double kPi = 3.14159265358979323846;
    for (int i = 0; i < kSinCosTableSize; ++i) {
        const double ang =
            2.0 * kPi * static_cast<double>(i) / static_cast<double>(kSinCosTableSize);
        const int32_t rs = ref_q14_sin(ang);
        const int32_t rc = ref_q14_cos(ang);
        const int32_t ds = static_cast<int32_t>(g_sin_table[i]) - rs;
        const int32_t dc = static_cast<int32_t>(g_cos_table[i]) - rc;
        if (ds < -1 || ds > 1 || dc < -1 || dc > 1) {
            std::printf("FAIL idx=%d sin tab=%d ref=%d d=%d; cos tab=%d ref=%d d=%d\n",
                        i, static_cast<int>(g_sin_table[i]), static_cast<int>(rs),
                        static_cast<int>(ds), static_cast<int>(g_cos_table[i]),
                        static_cast<int>(rc), static_cast<int>(dc));
            ++g_fail;
            if (g_fail > 20) {
                return;
            }
        }
    }
}

void test_lookup_phase() noexcept {
    for (int i = 0; i < kSinCosTableSize; ++i) {
        const uint32_t phase =
            static_cast<uint32_t>(static_cast<uint64_t>(static_cast<unsigned>(i))
                                   << static_cast<unsigned>(kSinCosIndexShift));
        const int16_t ls = Lookup_Sin(phase);
        const int16_t lc = Lookup_Cos(phase);
        if (ls != g_sin_table[i] || lc != g_cos_table[i]) {
            fail("Lookup_Sin/Cos must match table at same index");
            return;
        }
    }
}

}  // namespace

int main() noexcept {
    test_cardinal_angles();
    test_vs_std_all_indices();
    test_lookup_phase();
    if (g_fail != 0) {
        std::printf("test_cfo_sincos_table: %d failure(s)\n", g_fail);
        return 1;
    }
    std::printf("test_cfo_sincos_table: PASS (1024 vs std, cardinal, lookup)\n");
    return 0;
}

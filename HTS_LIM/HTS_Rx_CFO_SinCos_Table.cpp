// =============================================================================
// HTS_Rx_CFO_SinCos_Table.cpp — INNOViD HTS Rx CFO sin/cos Q14 table
// Phase 1-2: host init via <cmath>; ARM firmware copies constexpr ROM (no float).
// =============================================================================
#include "HTS_Rx_CFO_SinCos_Table.hpp"

#if !defined(HTS_PLATFORM_ARM) || defined(HTS_ALLOW_HOST_BUILD)
#include <cmath>
#else
#include "HTS_Rx_CFO_SinCos_Table_ROM.hpp"
#endif

namespace hts {
namespace rx_cfo {

int16_t g_sin_table[kSinCosTableSize] = {};
int16_t g_cos_table[kSinCosTableSize] = {};

namespace {
bool g_initialized = false;

static inline int16_t clamp_q14_i32(int32_t v) noexcept {
    if (v > 32767) {
        return 32767;
    }
    if (v < -32768) {
        return -32768;
    }
    return static_cast<int16_t>(v);
}
}  // namespace

void Build_SinCos_Table() noexcept {
    if (g_initialized) {
        return;
    }
#if !defined(HTS_PLATFORM_ARM) || defined(HTS_ALLOW_HOST_BUILD)
    static constexpr double kPi = 3.14159265358979323846;
    for (int i = 0; i < kSinCosTableSize; ++i) {
        const double ang =
            2.0 * kPi * static_cast<double>(i) / static_cast<double>(kSinCosTableSize);
        const double sd = std::sin(ang) * static_cast<double>(kQ14One);
        const double cd = std::cos(ang) * static_cast<double>(kQ14One);
        const int32_t si = static_cast<int32_t>(std::llround(sd));
        const int32_t ci = static_cast<int32_t>(std::llround(cd));
        g_sin_table[i] = clamp_q14_i32(si);
        g_cos_table[i] = clamp_q14_i32(ci);
    }
#else
    for (int i = 0; i < kSinCosTableSize; ++i) {
        g_sin_table[i] = kSinRomQ14[i];
        g_cos_table[i] = kCosRomQ14[i];
    }
#endif
    g_initialized = true;
}

}  // namespace rx_cfo
}  // namespace hts

// =============================================================================
// HTS_Rx_CFO_SinCos_Table.hpp — INNOViD HTS Rx CFO sin/cos lookup (Q14)
// =============================================================================
#pragma once

#include <cstdint>

namespace hts {
namespace rx_cfo {

inline constexpr int kSinCosTableSize = 1024;
inline constexpr int kSinCosIndexShift = 22;  // uint32 Q32 -> 10-bit index
inline constexpr int32_t kQ14One = 16384;

extern int16_t g_sin_table[kSinCosTableSize];
extern int16_t g_cos_table[kSinCosTableSize];

void Build_SinCos_Table() noexcept;

static inline int16_t Lookup_Sin(uint32_t phase_q32) noexcept {
    const int idx =
        static_cast<int>((phase_q32 >> kSinCosIndexShift) & (kSinCosTableSize - 1));
    return g_sin_table[idx];
}

static inline int16_t Lookup_Cos(uint32_t phase_q32) noexcept {
    const int idx =
        static_cast<int>((phase_q32 >> kSinCosIndexShift) & (kSinCosTableSize - 1));
    return g_cos_table[idx];
}

}  // namespace rx_cfo
}  // namespace hts

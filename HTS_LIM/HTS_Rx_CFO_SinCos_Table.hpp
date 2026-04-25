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

/// PTE: quadratic through (idx,y_m) and (idx+1,y_r) with u=frac/2^22∈[0,1),
///     curvature from y_l at idx−1: P = y_m + (y_r−y_m)·u + (y_r−2y_m+y_l)·u·(u−1)/2.
///     u·(u−1) in int64: u·(u−2^22) / 2^44 (no float).
inline int16_t Lookup_Sincos_Parabolic_Table(const int16_t* table,
                                             uint32_t phase_q32) noexcept {
    constexpr uint32_t kFracMask = (1u << 22) - 1u;
    const int idx =
        static_cast<int>((phase_q32 >> kSinCosIndexShift) & (kSinCosTableSize - 1));
    const int i_l = (idx + kSinCosTableSize - 1) & (kSinCosTableSize - 1);
    const int i_r = (idx + 1) & (kSinCosTableSize - 1);
    const int32_t y_l = static_cast<int32_t>(table[i_l]);
    const int32_t y_m = static_cast<int32_t>(table[idx]);
    const int32_t y_r = static_cast<int32_t>(table[i_r]);
    const int64_t u = static_cast<int64_t>(phase_q32 & kFracMask);
    const int64_t dm = static_cast<int64_t>(y_r) - static_cast<int64_t>(y_m);
    const int64_t q = static_cast<int64_t>(y_r) - (static_cast<int64_t>(y_m) << 1) +
                      static_cast<int64_t>(y_l);
    const int64_t t1 = (dm * u) >> 22;
    const int64_t t2 = (q * u * (u - (1LL << 22))) >> 44;
    const int64_t p = static_cast<int64_t>(y_m) + t1 + t2;
    if (p > 32767) {
        return 32767;
    }
    if (p < -32768) {
        return -32768;
    }
    return static_cast<int16_t>(p);
}

static inline int16_t Lookup_Sin(uint32_t phase_q32) noexcept {
    return Lookup_Sincos_Parabolic_Table(g_sin_table, phase_q32);
}

static inline int16_t Lookup_Cos(uint32_t phase_q32) noexcept {
    return Lookup_Sincos_Parabolic_Table(g_cos_table, phase_q32);
}

}  // namespace rx_cfo
}  // namespace hts

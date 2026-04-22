// ============================================================================
// HTS_Preamble_Holographic.cpp
// ============================================================================
#include "HTS_Preamble_Holographic.h"

namespace ProtectedEngine {
namespace Holographic {

namespace {

// k_w63 (±1) — popcount parity of index (Dispatcher k_w63 와 동일, 링크 충돌 방지)
constexpr int8_t k_w63_local[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1,
    +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1,
    +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1
};

}  // namespace

// Zadoff-Chu ROM 테이블 (u=1, N=64), Q14
const ChuSample k_chu_table[64] = {
    {  16384,      0 },
    {  16305,  -1606 },
    {  15679,  -4756 },
    {  13623,  -9102 },
    {   9102, -13623 },
    {   1606, -16305 },
    {  -7723, -14449 },
    { -15137,  -6270 },
    { -15137,   6270 },
    {  -4756,  15679 },
    {  10394,  12665 },
    {  16069,  -3196 },
    {   3196, -16069 },
    { -14449,  -7723 },
    { -10394,  12665 },
    {  11585,  11585 },
    {  11585, -11585 },
    { -12665, -10394 },
    {  -7723,  14449 },
    {  16069,   3196 },
    {  -3196, -16069 },
    { -12665,  10394 },
    {  15679,   4756 },
    {  -6270, -15137 },
    {  -6270,  15137 },
    {  14449,  -7723 },
    { -16305,  -1606 },
    {  13623,   9102 },
    {  -9102, -13623 },
    {   4756,  15679 },
    {  -1606, -16305 },
    {      0,  16384 },
    {      0, -16384 },
    {   1606,  16305 },
    {  -4756, -15679 },
    {   9102,  13623 },
    { -13623,  -9102 },
    {  16305,   1606 },
    { -14449,   7723 },
    {   6270, -15137 },
    {   6270,  15137 },
    { -15679,  -4756 },
    {  12665, -10394 },
    {   3196,  16069 },
    { -16069,  -3196 },
    {   7723, -14449 },
    {  12665,  10394 },
    { -11585,  11585 },
    { -11585, -11585 },
    {  10394, -12665 },
    {  14449,   7723 },
    {  -3196,  16069 },
    { -16069,   3196 },
    { -10394, -12665 },
    {   4756, -15679 },
    {  15137,  -6270 },
    {  15137,   6270 },
    {   7723,  14449 },
    {  -1606,  16305 },
    {  -9102,  13623 },
    { -13623,   9102 },
    { -15679,   4756 },
    { -16305,   1606 },
    { -16384,      0 }
};

int64_t holographic_dot_segmented(const int16_t* chip_I,
                                    const int16_t* chip_Q) noexcept {
    int64_t total_energy = 0;

    for (int seg = 0; seg < 8; ++seg) {
        int32_t seg_I = 0;
        int32_t seg_Q = 0;

        for (int k = 0; k < 8; ++k) {
            const int i = (seg << 3) + k;
            const int32_t rI = static_cast<int32_t>(chip_I[i]);
            const int32_t rQ = static_cast<int32_t>(chip_Q[i]);

            const int32_t cos_v = static_cast<int32_t>(k_chu_table[i].cos_q14);
            const int32_t sin_v = static_cast<int32_t>(k_chu_table[i].sin_q14);

            const int32_t desc_I = (rI * cos_v + rQ * sin_v) >> 14;
            const int32_t desc_Q = (rQ * cos_v - rI * sin_v) >> 14;

            if (k_w63_local[i] > 0) {
                seg_I += desc_I;
                seg_Q += desc_Q;
            } else {
                seg_I -= desc_I;
                seg_Q -= desc_Q;
            }
        }

        const int64_t eI = static_cast<int64_t>(seg_I) * seg_I;
        const int64_t eQ = static_cast<int64_t>(seg_Q) * seg_Q;
        total_energy += (eI + eQ);
    }

    return total_energy;
}

int32_t peak_to_median_ratio_x10(const int64_t* energies, int n) noexcept {
    if (n != 64) {
        return 0;
    }

    int64_t buf[64];
    for (int i = 0; i < 64; ++i) {
        buf[i] = energies[i];
    }

    int64_t peak = 0;
    for (int i = 0; i < 64; ++i) {
        if (buf[i] > peak) {
            peak = buf[i];
        }
    }

    for (int k = 0; k <= 32; ++k) {
        int min_idx = k;
        int64_t min_val = buf[k];
        for (int i = k + 1; i < 64; ++i) {
            if (buf[i] < min_val) {
                min_val = buf[i];
                min_idx = i;
            }
        }
        if (min_idx != k) {
            const int64_t t = buf[k];
            buf[k] = buf[min_idx];
            buf[min_idx] = t;
        }
    }
    const int64_t med = buf[32];

    if (med <= 0) {
        return 9999;
    }

    const int64_t ratio_x10 = (peak * 10) / med;
    if (ratio_x10 > 9999) {
        return 9999;
    }
    if (ratio_x10 < 0) {
        return 0;
    }
    return static_cast<int32_t>(ratio_x10);
}

int32_t verify_chu_table_max_err_ppm() noexcept {
    constexpr int64_t Q28_ONE = 268435456LL;

    int64_t max_abs_err_q28 = 0;

    for (int n = 0; n < 64; ++n) {
        const int64_t c = static_cast<int64_t>(k_chu_table[n].cos_q14);
        const int64_t s = static_cast<int64_t>(k_chu_table[n].sin_q14);
        const int64_t mag_q28 = c * c + s * s;
        const int64_t err = mag_q28 - Q28_ONE;
        const int64_t abs_err = (err >= 0) ? err : -err;
        if (abs_err > max_abs_err_q28) {
            max_abs_err_q28 = abs_err;
        }
    }

    const int64_t ppm = (max_abs_err_q28 * 1000000LL) / Q28_ONE;
    return static_cast<int32_t>(ppm);
}

}  // namespace Holographic
}  // namespace ProtectedEngine

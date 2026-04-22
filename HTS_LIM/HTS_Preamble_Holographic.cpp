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
    // Phase 4 방안 A: Chu descramble 제거 — raw chip × k_w63(±1) 후 8-seg 합성
    int64_t total_energy = 0;

    for (int seg = 0; seg < 8; ++seg) {
        int32_t seg_I = 0;
        int32_t seg_Q = 0;

        for (int k = 0; k < 8; ++k) {
            const int i = (seg << 3) + k;
            const int32_t rI = static_cast<int32_t>(chip_I[i]);
            const int32_t rQ = static_cast<int32_t>(chip_Q[i]);

            if (k_w63_local[i] > 0) {
                seg_I += rI;
                seg_Q += rQ;
            } else {
                seg_I -= rI;
                seg_Q -= rQ;
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

namespace {

// sin(i·2π/256)·16384, i=0..255 (Q14)
constexpr int16_t k_sin_q14_256[256] = {
    0, 402, 804, 1205, 1606, 2006, 2404, 2801,
    3196, 3590, 3981, 4370, 4756, 5139, 5520, 5897,
    6270, 6639, 7005, 7366, 7723, 8076, 8423, 8765,
    9102, 9434, 9760, 10080, 10394, 10702, 11003, 11297,
    11585, 11866, 12140, 12406, 12665, 12916, 13160, 13395,
    13623, 13842, 14053, 14256, 14449, 14635, 14811, 14978,
    15137, 15286, 15426, 15557, 15679, 15791, 15893, 15986,
    16069, 16143, 16207, 16261, 16305, 16340, 16364, 16379,
    16384, 16379, 16364, 16340, 16305, 16261, 16207, 16143,
    16069, 15986, 15893, 15791, 15679, 15557, 15426, 15286,
    15137, 14978, 14811, 14635, 14449, 14256, 14053, 13842,
    13623, 13395, 13160, 12916, 12665, 12406, 12140, 11866,
    11585, 11297, 11003, 10702, 10394, 10080, 9760, 9434,
    9102, 8765, 8423, 8076, 7723, 7366, 7005, 6639,
    6270, 5897, 5520, 5139, 4756, 4370, 3981, 3590,
    3196, 2801, 2404, 2006, 1606, 1205, 804, 402,
    0, -402, -804, -1205, -1606, -2006, -2404, -2801,
    -3196, -3590, -3981, -4370, -4756, -5139, -5520, -5897,
    -6270, -6639, -7005, -7366, -7723, -8076, -8423, -8765,
    -9102, -9434, -9760, -10080, -10394, -10702, -11003, -11297,
    -11585, -11866, -12140, -12406, -12665, -12916, -13160, -13395,
    -13623, -13842, -14053, -14256, -14449, -14635, -14811, -14978,
    -15137, -15286, -15426, -15557, -15679, -15791, -15893, -15986,
    -16069, -16143, -16207, -16261, -16305, -16340, -16364, -16379,
    -16384, -16379, -16364, -16340, -16305, -16261, -16207, -16143,
    -16069, -15986, -15893, -15791, -15679, -15557, -15426, -15286,
    -15137, -14978, -14811, -14635, -14449, -14256, -14053, -13842,
    -13623, -13395, -13160, -12916, -12665, -12406, -12140, -11866,
    -11585, -11297, -11003, -10702, -10394, -10080, -9760, -9434,
    -9102, -8765, -8423, -8076, -7723, -7366, -7005, -6639,
    -6270, -5897, -5520, -5139, -4756, -4370, -3981, -3590,
    -3196, -2801, -2404, -2006, -1606, -1205, -804, -402
};

inline int16_t lut_sin_q14(uint8_t idx) noexcept {
    return k_sin_q14_256[idx];
}

inline int16_t lut_cos_q14(uint8_t idx) noexcept {
    return k_sin_q14_256[static_cast<uint8_t>(idx + 64u)];
}

}  // namespace

void derotate_buffer_q8(const int16_t* src_I,
                         const int16_t* src_Q,
                         int16_t* dst_I,
                         int16_t* dst_Q,
                         int n_chip,
                         int8_t phase_q8) noexcept {
    if (n_chip <= 0 || src_I == nullptr || src_Q == nullptr ||
        dst_I == nullptr || dst_Q == nullptr) {
        return;
    }

    const uint8_t step_idx =
        static_cast<uint8_t>(static_cast<int8_t>(-phase_q8));
    const int32_t dc = static_cast<int32_t>(lut_cos_q14(step_idx));
    const int32_t ds = static_cast<int32_t>(lut_sin_q14(step_idx));

    int32_t cos_acc = 16384;
    int32_t sin_acc = 0;

    for (int k = 0; k < n_chip; ++k) {
        const int32_t sI = static_cast<int32_t>(src_I[k]);
        const int32_t sQ = static_cast<int32_t>(src_Q[k]);

        const int64_t rI64 =
            static_cast<int64_t>(sI) * cos_acc - static_cast<int64_t>(sQ) * sin_acc;
        const int64_t rQ64 =
            static_cast<int64_t>(sI) * sin_acc + static_cast<int64_t>(sQ) * cos_acc;

        int32_t rI = static_cast<int32_t>(rI64 >> 14);
        int32_t rQ = static_cast<int32_t>(rQ64 >> 14);
        if (rI > 32767) {
            rI = 32767;
        } else if (rI < -32768) {
            rI = -32768;
        }
        if (rQ > 32767) {
            rQ = 32767;
        } else if (rQ < -32768) {
            rQ = -32768;
        }
        dst_I[k] = static_cast<int16_t>(rI);
        dst_Q[k] = static_cast<int16_t>(rQ);

        const int64_t new_cos64 =
            static_cast<int64_t>(cos_acc) * dc - static_cast<int64_t>(sin_acc) * ds;
        const int64_t new_sin64 =
            static_cast<int64_t>(cos_acc) * ds + static_cast<int64_t>(sin_acc) * dc;
        cos_acc = static_cast<int32_t>(new_cos64 >> 14);
        sin_acc = static_cast<int32_t>(new_sin64 >> 14);

        if ((k & 63) == 63) {
            const int64_t mag_sq = static_cast<int64_t>(cos_acc) * cos_acc +
                                   static_cast<int64_t>(sin_acc) * sin_acc;
            constexpr int64_t target = 268435456LL;
            if (mag_sq > target * 4LL) {
                cos_acc >>= 1;
                sin_acc >>= 1;
            }
        }
    }
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

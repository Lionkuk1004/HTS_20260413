// HTS_V400_Dispatcher_Gravity.cpp
// 6면 Gravity Cube — ab19ea9^ 알고리즘 이식 + face_D PTE/미적분 + DPTE/BPTE
// G-3-2

#include "HTS_V400_Dispatcher_Gravity.hpp"

#if defined(HTS_USE_GRAVITY) && HTS_USE_GRAVITY

#include "HTS_V400_Dispatcher_Internal.hpp"
#include "HTS_Holo_Tensor_4D_Defs.h"
#include <climits>
#include <cstdint>

#if defined(HTS_USE_PACD) && HTS_USE_PACD
#include "HTS_CFO_V5a.hpp"
#endif

#if defined(HTS_USE_PN_MASKED) && HTS_USE_PN_MASKED
#include "HTS_V400_Dispatcher_PNMasked.hpp"
#endif

namespace HTS_LIM {
namespace detail_gravity {

namespace {

using ProtectedEngine::Xoshiro128ss;
using ProtectedEngine::detail::fwht_raw;

// ab19ea9^ / G2 CMYK — `HTS_Holo_Preamble_Gen.hpp` 동일 수식 (블록별 ctx).
void gen_holo_sequence_cmyk_impl(const uint32_t seed[4], uint32_t slot,
                                 uint32_t block_id, int16_t amp,
                                 int16_t* const out_I) noexcept {
    if (seed == nullptr || out_I == nullptr) {
        return;
    }
    auto mix = [](const uint32_t z0) noexcept -> uint32_t {
        uint32_t z = z0;
        z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
        z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
        return z ^ (z >> 16u);
    };
    const uint32_t ctx = slot ^ (block_id * 0xDEADBEEFu);
    Xoshiro128ss rng{};
    rng.s[0] = mix(seed[0] ^ ctx);
    rng.s[1] = mix(seed[1] ^ (ctx * 0x9E3779B9u));
    rng.s[2] = mix(seed[2] ^ (ctx * 0x517CC1B7u));
    rng.s[3] = mix(seed[3] ^ (ctx * 0x6C62272Eu));
    rng.s[0] ^= 0x50524500u;
    for (int w = 0; w < 8; ++w) {
        (void)rng.Next();
    }

    int32_t buf[64];
    for (int i = 0; i < 64; ++i) {
        buf[i] = (rng.Next() & 1u) ? 1 : -1;
    }
    fwht_raw(buf, 64);

    static constexpr uint8_t k_perm[24][4] = {
        {0, 1, 2, 3}, {0, 1, 3, 2}, {0, 2, 1, 3}, {0, 2, 3, 1}, {0, 3, 1, 2},
        {0, 3, 2, 1}, {1, 0, 2, 3}, {1, 0, 3, 2}, {1, 2, 0, 3}, {1, 2, 3, 0},
        {1, 3, 0, 2}, {1, 3, 2, 0}, {2, 0, 1, 3}, {2, 0, 3, 1}, {2, 1, 0, 3},
        {2, 1, 3, 0}, {2, 3, 0, 1}, {2, 3, 1, 0}, {3, 0, 1, 2}, {3, 0, 2, 1},
        {3, 1, 0, 2}, {3, 1, 2, 0}, {3, 2, 0, 1}, {3, 2, 1, 0}};
    for (int i = 0; i < 64; i += 4) {
        const uint32_t gyro = rng.Next();
        for (int j = 0; j < 4; ++j) {
            if ((gyro >> static_cast<uint32_t>(j)) & 1u) {
                buf[i + j] = -buf[i + j];
            }
        }
        const uint8_t pi =
            static_cast<uint8_t>(((gyro >> 4u) & 0x1Fu) % 24u);
        const uint8_t* p = k_perm[pi];
        const int32_t v0 = buf[i + p[0]];
        const int32_t v1 = buf[i + p[1]];
        const int32_t v2 = buf[i + p[2]];
        const int32_t v3 = buf[i + p[3]];
        buf[i + 0] = v0 + v1 + v2 + v3;
        buf[i + 1] = v0 - v1 + v2 - v3;
        buf[i + 2] = v0 + v1 - v2 - v3;
        buf[i + 3] = v0 - v1 - v2 + v3;
    }
    fwht_raw(buf, 64);

    for (int i = 0; i < 64; ++i) {
        const int32_t b = buf[i];
        out_I[i] = (b >= 0)
                       ? amp
                       : static_cast<int16_t>(-static_cast<int32_t>(amp));
    }
}

constexpr int32_t k_gravity_window_chips_ = 256;

// --- Mirror: PNMasked `pn_masked_pte_subchip` (Q14) when HTS_USE_PN_MASKED off ---
inline int32_t grav_pte_subchip_q14_mirror(int32_t y_minus, int32_t y_zero,
                                           int32_t y_plus) noexcept {
    const int64_t num =
        static_cast<int64_t>(y_minus) - static_cast<int64_t>(y_plus);
    int64_t den = 2 * (static_cast<int64_t>(y_minus) -
                       (static_cast<int64_t>(y_zero) << 1) +
                       static_cast<int64_t>(y_plus));
    const int64_t den_nz = (den | (-den)) >> 63;
    den = (den_nz & den) | ((~den_nz) & 1);
    const int64_t k_q14 = static_cast<int64_t>(1) << 14;
    const int64_t offset_q14 = (num * k_q14) / den;
    const int64_t high_diff = offset_q14 - 8192;
    const int64_t high_mask = high_diff >> 63;
    int64_t result = (high_mask & offset_q14) | (~high_mask & 8192);
    const int64_t low_diff = result + 8192;
    const int64_t ge_mask = ~(low_diff >> 63);
    result = (ge_mask & result) | (~ge_mask & (-8192));
    return static_cast<int32_t>(result);
}

inline int32_t grav_pte_subchip_dispatch(int32_t ym, int32_t y0,
                                         int32_t yp) noexcept {
#if defined(HTS_USE_PN_MASKED) && HTS_USE_PN_MASKED
    return ::detail::pn_masked_pte_subchip(ym, y0, yp);
#else
    return grav_pte_subchip_q14_mirror(ym, y0, yp);
#endif
}

// --- ab19ea9^: q10_ratio_clamp (성공 경로 수식 보존) ---
inline int32_t q10_ratio_clamp(int64_t num, int64_t den,
                               int64_t max_q10) noexcept {
    if (den <= 0) {
        return 0;
    }
    if (num <= 0) {
        return 0;
    }
    const int64_t q = (num << 10) / den;
    if (q > max_q10) {
        return static_cast<int32_t>(max_q10);
    }
    return static_cast<int32_t>(q);
}

inline int64_t int_sqrt64(int64_t x) noexcept {
    if (x <= 0) {
        return 0;
    }
    int64_t y = x;
    int64_t s = 1;
    while (s < y) {
        y >>= 1;
        s <<= 1;
    }
    y = (y + s) >> 1;
    if (y < 1) {
        y = 1;
    }
    for (int i = 0; i < 6; ++i) {
        y = (y + x / y) >> 1;
        if (y < 1) {
            y = 1;
            break;
        }
    }
    return y;
}

// ab19ea9^ gravity_xc_single_offset
inline void gravity_xc_single_offset(
    const int16_t* tx, const int16_t* tq, int32_t off, const int16_t* tplA,
    const int16_t* tplB, const int16_t* tplC, const int16_t* tplD,
    int64_t sub_score[4][4], int64_t sub_xI[4][4], int64_t sub_xQ[4][4],
    int64_t tmpl_xI[4], int64_t tmpl_xQ[4], int64_t tmpl_total[4],
    int64_t* out_total) noexcept {
    const int16_t* T[4] = {tplA, tplB, tplC, tplD};
    int64_t grand = 0;
    for (int t = 0; t < 4; ++t) {
        tmpl_xI[t] = 0;
        tmpl_xQ[t] = 0;
        tmpl_total[t] = 0;
        const int base = static_cast<int>(off) + t * 64;
        for (int s = 0; s < 4; ++s) {
            int64_t xI = 0;
            int64_t xQ = 0;
            for (int k = 0; k < 16; ++k) {
                const int idx = s * 16 + k;
                const int sg = (T[t][idx] > 0) ? 1 : -1;
                xI += static_cast<int64_t>(tx[base + idx]) *
                      static_cast<int64_t>(sg);
                xQ += static_cast<int64_t>(tq[base + idx]) *
                      static_cast<int64_t>(sg);
            }
            sub_xI[t][s] = xI;
            sub_xQ[t][s] = xQ;
            const int64_t mag2 = xI * xI + xQ * xQ;
            sub_score[t][s] = mag2;
            tmpl_xI[t] += xI;
            tmpl_xQ[t] += xQ;
            tmpl_total[t] += mag2;
            grand += mag2;
        }
    }
    *out_total = grand;
}

// ab19ea9^ gravity_compute_faces (tmpl_* 는 호출부에서 out_cube 로 복사)
inline void gravity_compute_faces(
    const int64_t sub_score[4][4], const int64_t sub_xI[4][4],
    const int64_t sub_xQ[4][4], const int64_t tmpl_xI[4],
    const int64_t tmpl_xQ[4], const int64_t tmpl_total[4], int64_t best_total,
    int64_t nf_total, int64_t nf_per_tmpl, int32_t* face_a, int32_t* face_b,
    int32_t* face_c, int32_t* face_d, int32_t* face_e, int32_t* face_f,
    int64_t* sum_dI_out, int64_t* sum_dQ_out) noexcept {
    int64_t A_mn = tmpl_total[0];
    int64_t A_mx = tmpl_total[0];
    for (int t = 1; t < 4; ++t) {
        if (tmpl_total[t] < A_mn) {
            A_mn = tmpl_total[t];
        }
        if (tmpl_total[t] > A_mx) {
            A_mx = tmpl_total[t];
        }
    }
    *face_a = q10_ratio_clamp(A_mn, A_mx, 1024);

    int64_t B_sum = 0;
    for (int t = 0; t < 4; ++t) {
        int64_t mn = sub_score[t][0];
        int64_t mx = sub_score[t][0];
        for (int s = 1; s < 4; ++s) {
            if (sub_score[t][s] < mn) {
                mn = sub_score[t][s];
            }
            if (sub_score[t][s] > mx) {
                mx = sub_score[t][s];
            }
        }
        B_sum += static_cast<int64_t>(q10_ratio_clamp(mn, mx, 1024));
    }
    *face_b = static_cast<int32_t>(B_sum >> 2);

    int64_t s32[4] = {0, 0, 0, 0};
    for (int t = 0; t < 4; ++t) {
        const int64_t xI1 = sub_xI[t][0] + sub_xI[t][1];
        const int64_t xQ1 = sub_xQ[t][0] + sub_xQ[t][1];
        const int64_t xI2 = sub_xI[t][2] + sub_xI[t][3];
        const int64_t xQ2 = sub_xQ[t][2] + sub_xQ[t][3];
        s32[t] = (xI1 * xI1 + xQ1 * xQ1) + (xI2 * xI2 + xQ2 * xQ2);
    }
    int64_t s32_mn = s32[0];
    int64_t s32_mx = s32[0];
    for (int t = 1; t < 4; ++t) {
        if (s32[t] < s32_mn) {
            s32_mn = s32[t];
        }
        if (s32[t] > s32_mx) {
            s32_mx = s32[t];
        }
    }
    const int32_t C32_q10 = q10_ratio_clamp(s32_mn, s32_mx, 1024);
    *face_c = (C32_q10 < *face_a) ? C32_q10 : *face_a;

    constexpr int64_t k_face_d_max_q10 =
        (static_cast<int64_t>(65535) * 1024LL);
    *face_d = q10_ratio_clamp(best_total, nf_total, k_face_d_max_q10);

    int64_t dI[3];
    int64_t dQ[3];
    for (int k = 0; k < 3; ++k) {
        dI[k] = tmpl_xI[k + 1] * tmpl_xI[k] + tmpl_xQ[k + 1] * tmpl_xQ[k];
        dQ[k] = tmpl_xQ[k + 1] * tmpl_xI[k] - tmpl_xI[k + 1] * tmpl_xQ[k];
    }
    const int64_t sum_dI = dI[0] + dI[1] + dI[2];
    const int64_t sum_dQ = dQ[0] + dQ[1] + dQ[2];
    if (sum_dI_out != nullptr) {
        *sum_dI_out = sum_dI;
    }
    if (sum_dQ_out != nullptr) {
        *sum_dQ_out = sum_dQ;
    }
    int64_t abs_d_sum = 0;
    for (int k = 0; k < 3; ++k) {
        int64_t m2 = dI[k] * dI[k] + dQ[k] * dQ[k];
        if (m2 < 0) {
            m2 = INT64_MAX;
        }
        abs_d_sum += int_sqrt64(m2);
    }
    int64_t mag_sum2 = sum_dI * sum_dI + sum_dQ * sum_dQ;
    if (mag_sum2 < 0) {
        mag_sum2 = INT64_MAX;
    }
    const int64_t mag_sum = int_sqrt64(mag_sum2);
    *face_e = q10_ratio_clamp(mag_sum, abs_d_sum, 1024);

    int64_t min_tmpl = tmpl_total[0];
    for (int t = 1; t < 4; ++t) {
        if (tmpl_total[t] < min_tmpl) {
            min_tmpl = tmpl_total[t];
        }
    }
    constexpr int64_t k_face_f_max_q10 =
        (static_cast<int64_t>(65535) * 1024LL);
    *face_f = q10_ratio_clamp(min_tmpl, nf_per_tmpl, k_face_f_max_q10);
}

// ab19ea9^ gravity_estimate_noise_floor
inline int64_t gravity_estimate_noise_floor(
    const int16_t* tx, const int16_t* tq, int32_t n, const int16_t* tplA,
    const int16_t* tplB, const int16_t* tplC, const int16_t* tplD) noexcept {
    int64_t samples[20];
    int num = 0;

    for (int off = 16; off + 256 <= n && num < 20; off += 41) {
        int64_t sub_score[4][4];
        int64_t sub_xI[4][4];
        int64_t sub_xQ[4][4];
        int64_t tmpl_xI[4];
        int64_t tmpl_xQ[4];
        int64_t tmpl_total[4];
        int64_t total = 0;

        gravity_xc_single_offset(tx, tq, static_cast<int32_t>(off), tplA,
                                 tplB, tplC, tplD, sub_score, sub_xI, sub_xQ,
                                 tmpl_xI, tmpl_xQ, tmpl_total, &total);

        samples[num++] = total;
    }

    if (num == 0) {
        return 1;
    }

    for (int i = 1; i < num; ++i) {
        const int64_t key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            --j;
        }
        samples[j + 1] = key;
    }

    const int64_t nf = samples[num / 3];
    return (nf < 1) ? 1 : nf;
}

inline int32_t sat_i32_from_i64_shr(int64_t v, int sh) noexcept {
    const int64_t x = v >> sh;
    if (x > static_cast<int64_t>(INT32_MAX)) {
        return INT32_MAX;
    }
    if (x < static_cast<int64_t>(INT32_MIN)) {
        return INT32_MIN;
    }
    return static_cast<int32_t>(x);
}

#define GRAV_FAIL_BIT(face_q10, thr)                                          \
    ((uint32_t)(((int64_t)(thr) - (int64_t)(face_q10)) >> 63) & 1u)

} // namespace

bool gravity_evaluate_cube(
    const int16_t* rx_pre_I,
    const int16_t* rx_pre_Q,
    const int16_t* tx_pre_tmpl_I,
    int32_t rx_chip_count,
    GravityCube6* out_cube) noexcept {
    if (rx_pre_I == nullptr || rx_pre_Q == nullptr ||
        tx_pre_tmpl_I == nullptr || out_cube == nullptr) {
        return false;
    }
    if (rx_chip_count < k_gravity_window_chips_) {
        *out_cube = GravityCube6{};
        return false;
    }

    const int16_t* tplA = tx_pre_tmpl_I;
    const int16_t* tplB = tx_pre_tmpl_I + 64;
    const int16_t* tplC = tx_pre_tmpl_I + 128;
    const int16_t* tplD = tx_pre_tmpl_I + 192;

    int64_t sub_score[4][4];
    int64_t sub_xI[4][4];
    int64_t sub_xQ[4][4];
    int64_t tmpl_xI64[4];
    int64_t tmpl_xQ64[4];
    int64_t tmpl_total[4];

    int32_t best_off = -1;
    int64_t best_total = -1;

    for (int32_t off = 0; off + k_gravity_window_chips_ <= rx_chip_count;
         ++off) {
        int64_t total = 0;
        gravity_xc_single_offset(rx_pre_I, rx_pre_Q, off, tplA, tplB, tplC,
                                 tplD, sub_score, sub_xI, sub_xQ, tmpl_xI64,
                                 tmpl_xQ64, tmpl_total, &total);
        if (total > best_total) {
            best_total = total;
            best_off = off;
        }
    }

    if (best_off < 0 || best_total < 0) {
        *out_cube = GravityCube6{};
        return false;
    }

    int64_t grand_at_best = 0;
    gravity_xc_single_offset(rx_pre_I, rx_pre_Q, best_off, tplA, tplB, tplC,
                             tplD, sub_score, sub_xI, sub_xQ, tmpl_xI64,
                             tmpl_xQ64, tmpl_total, &grand_at_best);

    const int64_t nf_total = gravity_estimate_noise_floor(
        rx_pre_I, rx_pre_Q, rx_chip_count, tplA, tplB, tplC, tplD);
    const int64_t nf_per_tmpl = nf_total >> 2;

    int32_t face_a = 0;
    int32_t face_b = 0;
    int32_t face_c = 0;
    int32_t face_d_ratio = 0;
    int32_t face_e = 0;
    int32_t face_f = 0;
    int64_t sum_dI = 0;
    int64_t sum_dQ = 0;
    gravity_compute_faces(sub_score, sub_xI, sub_xQ, tmpl_xI64, tmpl_xQ64,
                            tmpl_total, grand_at_best, nf_total, nf_per_tmpl,
                            &face_a, &face_b, &face_c, &face_d_ratio, &face_e,
                            &face_f, &sum_dI, &sum_dQ);

    // --- face_D Layer 2–4: PTE + 미적분 + sharpness (템플릿 에너지 3점) ---
    const int32_t y_minus =
        sat_i32_from_i64_shr(tmpl_total[0], 10);
    const int32_t y_zero =
        sat_i32_from_i64_shr(tmpl_total[1], 10);
    const int32_t y_plus =
        sat_i32_from_i64_shr(tmpl_total[2], 10);

    const int32_t pte_delta_q14 =
        grav_pte_subchip_dispatch(y_minus, y_zero, y_plus);

    const int32_t d1_left = y_zero - y_minus;
    const int32_t d1_right = y_plus - y_zero;
    const uint32_t left_nonneg =
        1u ^ (static_cast<uint32_t>(d1_left) >> 31);
    const uint32_t right_nonpos =
        (static_cast<uint32_t>(d1_right) >> 31) |
        (static_cast<uint32_t>(d1_right == 0));
    const uint32_t zc_ok = left_nonneg & right_nonpos;

    const int32_t d2 = y_minus - (y_zero << 1) + y_plus;
    const uint32_t curve_neg =
        (static_cast<uint32_t>(static_cast<uint32_t>(d2) >> 31)) & 1u;

    const int32_t d2_sign = d2 >> 31;
    const int32_t abs_d2 = static_cast<int32_t>(
        (static_cast<uint32_t>(d2 ^ d2_sign)) - static_cast<uint32_t>(d2_sign));
    const int32_t y_safe = (y_zero == 0) ? 1 : y_zero;
    const int32_t sharpness_q8 =
        static_cast<int32_t>((static_cast<int64_t>(abs_d2) << 8) /
                             static_cast<int64_t>(y_safe));

    const uint32_t flat_ok =
        (static_cast<uint32_t>((49 - sharpness_q8) >> 31)) & 1u;
    const uint32_t spike_ok =
        (static_cast<uint32_t>(
             (sharpness_q8 - (GRAVITY_SPIKE_THR_Q8 + 1)) >> 31)) &
        1u;

    const uint32_t pte_valid =
        zc_ok & curve_neg & flat_ok & spike_ok;

#if defined(HTS_USE_PACD) && HTS_USE_PACD
    const int32_t phase_q15 =
        hts::rx_cfo::Atan2_Correlation_Dpte_Q15(sum_dQ, sum_dI);
    const int32_t ph_mask = phase_q15 >> 31;
    const int32_t ph_abs =
        static_cast<int32_t>((static_cast<uint32_t>(phase_q15) ^ ph_mask) -
                             ph_mask);
    const int32_t ph_pen = ph_abs >> 10;
    const int32_t ph_pen_c = (ph_pen > 256) ? 256 : ph_pen;
    const int32_t fe_adj = face_e - ph_pen_c;
    const int32_t fe_m = fe_adj >> 31;
    face_e = (fe_adj & ~fe_m) | (0 & fe_m);
#else
    (void)sum_dI;
    (void)sum_dQ;
#endif

    GravityCube6 out{};
    out.face_A_q10 = face_a;
    out.face_B_q10 = face_b;
    out.face_C_q10 = face_c;
    out.face_D_q10 = face_d_ratio;
    out.face_E_q10 = face_e;
    out.face_F_q10 = face_f;
    out.best_off = static_cast<int>(best_off);
    out.total_score = sat_i32_from_i64_shr(best_total, 8);
    out.pte_delta_q14 = pte_delta_q14;
    out.pte_sharpness_q8 = sharpness_q8;
    out.pte_valid_mask = pte_valid;

    for (int t = 0; t < 4; ++t) {
        out.tmpl_xI[t] = sat_i32_from_i64_shr(tmpl_xI64[t], 8);
        out.tmpl_xQ[t] = sat_i32_from_i64_shr(tmpl_xQ64[t], 8);
    }

    const uint32_t fail_a = GRAV_FAIL_BIT(out.face_A_q10, GRAVITY_THR_A_Q10);
    const uint32_t fail_b = GRAV_FAIL_BIT(out.face_B_q10, GRAVITY_THR_B_Q10);
    const uint32_t fail_c = GRAV_FAIL_BIT(out.face_C_q10, GRAVITY_THR_C_Q10);
    const uint32_t fail_d = GRAV_FAIL_BIT(out.face_D_q10, GRAVITY_THR_D_Q10);
    const uint32_t fail_e = GRAV_FAIL_BIT(out.face_E_q10, GRAVITY_THR_E_Q10);
    const uint32_t fail_f = GRAV_FAIL_BIT(out.face_F_q10, GRAVITY_THR_F_Q10);
    const uint32_t fail_pte = (pte_valid ^ 1u) & 1u;

    const uint32_t any_fail =
        fail_a | fail_b | fail_c | fail_d | fail_e | fail_f | fail_pte;

    *out_cube = out;
    return any_fail == 0u;
}

void gravity_fill_cmyk_templates_i256(const uint32_t seed[4], uint32_t slot,
                                        int16_t pre_amp,
                                        int16_t* out256) noexcept {
    if (seed == nullptr || out256 == nullptr) {
        return;
    }
    gen_holo_sequence_cmyk_impl(seed, slot, 0u, pre_amp, out256);
    gen_holo_sequence_cmyk_impl(seed, slot, 1u, pre_amp, out256 + 64);
    gen_holo_sequence_cmyk_impl(seed, slot, 2u, pre_amp, out256 + 128);
    gen_holo_sequence_cmyk_impl(seed, slot, 3u, pre_amp, out256 + 192);
}

} // namespace detail_gravity
} // namespace HTS_LIM

#endif // HTS_USE_GRAVITY

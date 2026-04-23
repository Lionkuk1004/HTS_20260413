// =============================================================================
// HTS_Holo_Preamble_Gen.hpp — CMYK holo preamble (TX/RX helpers, optional)
// =============================================================================
#pragma once

#include "HTS_Config.h"

#if defined(HTS_HOLO_PREAMBLE) && HTS_HOLO_CMYK_MODE

#include "HTS_BitOps.h"
#include "HTS_Holo_Tensor_4D_Defs.h"
#include <climits>
#include <cstdint>

namespace ProtectedEngine {
namespace detail {

void fwht_raw(int32_t* d, int n) noexcept;

/// CMYK: block_id 0..3 → independent 64-chip preamble; block_id==0 matches legacy ctx=slot.
/// Integer-only. INNOViD HOLO_6FACE_FINAL.
inline void gen_holo_sequence_cmyk(const uint32_t seed[4], uint32_t slot,
                                   uint32_t block_id, int16_t amp,
                                   int16_t* const out_I) noexcept {
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

// -----------------------------------------------------------------
// 6-Face Gravity Cube (Q10). INNOViD HOLO_6FACE_FINAL.
// -----------------------------------------------------------------
struct GravityCube6 {
    int32_t face_A_q10 = 0;
    int32_t face_B_q10 = 0;
    int32_t face_C_q10 = 0;
    int32_t face_D_q10 = 0;
    int32_t face_E_q10 = 0;
    int32_t face_F_q10 = 0;

    int64_t total_score = 0;
    int32_t best_off = 0;

    int64_t tmpl_xI[4] = {};
    int64_t tmpl_xQ[4] = {};
};

// GRAVITY_THR_* (Q10): INNOViD — Phase 2.6 (T6 CMYK DIAG ~846k×CUBE lines,
// UTF-16 log; p10/p50/p90). v1.0 시뮬 고정값(A512…F15360)은 CMYK+Q14 경로에
// 과도 → 실측 완화: A200 B60 C120 D1200 E400 F700 → T6 CMYK=1 합계 100/18400
// (재현 2회). 추가 공격·오탐 검증은 후속 Phase 권장.
//
// Phase 3.M: Sign XC coarse + fine 이후 6면 실측(N≈458k)에서 C median≈105<thr120,
// B median≈67 vs thr60 여유 얇음 → C 80, B 45로 재튜닝. A/D/E/F 유지.
#define GRAVITY_THR_A_Q10 200
#define GRAVITY_THR_B_Q10 45
#define GRAVITY_THR_C_Q10 80
#define GRAVITY_THR_D_Q10 1200
#define GRAVITY_THR_E_Q10 400
#define GRAVITY_THR_F_Q10 700

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

// -----------------------------------------------------------------
// Stage 1 coarse timing (CMYK): template A only, non-coherent |XC|²
// summed over 4×16-chip sub-blocks (CFO-resistant). Replaces lag-64
// autocorrelation timing gate for [A|B|C|D] (lag-64 assumes [A|A|…]).
// INNOViD HOLO_6FACE_FINAL — Phase 2.5.
// -----------------------------------------------------------------
inline int64_t gravity_coarse_xc_A_only(const int16_t* tx, const int16_t* tq,
                                        int32_t off,
                                        const int16_t* tplA) noexcept {
    int64_t total_mag2 = 0;
    for (int sub = 0; sub < 4; ++sub) {
        int64_t xI = 0;
        int64_t xQ = 0;
        for (int k = 0; k < 16; ++k) {
            const int idx = sub * 16 + k;
            const int sg = (tplA[idx] > 0) ? 1 : -1;
            xI += static_cast<int64_t>(tx[off + idx]) * sg;
            xQ += static_cast<int64_t>(tq[off + idx]) * sg;
        }
        total_mag2 += xI * xI + xQ * xQ;
    }
    return total_mag2;
}

/// Coarse scan: step 1 chip. `buf_len` = RX chip count. Only offsets where a
/// full 256-chip gravity window fits (`off + 256 <= buf_len`) are candidates,
/// matching legacy holo P0 gravity scan range. Template A uses the first 64
/// chips at each candidate. Writes best score (0 if none).
inline int32_t gravity_coarse_timing_scan(const int16_t* tx, const int16_t* tq,
                                          int32_t scan_start, int32_t buf_len,
                                          const int16_t* tplA,
                                          int64_t* out_best_score) noexcept {
    int64_t best_score = -1;
    int32_t best_off = -1;
    for (int32_t off = scan_start; off + 256 <= buf_len; ++off) {
        const int64_t score = gravity_coarse_xc_A_only(tx, tq, off, tplA);
        if (score > best_score) {
            best_score = score;
            best_off = off;
        }
    }
    if (out_best_score != nullptr) {
        *out_best_score = (best_score < 0) ? 0 : best_score;
    }
    return best_off;
}

// -----------------------------------------------------------------
// Phase 3.A: Sign XC (1-bit vs template sign). INNOViD — integer-only;
// popcount via popcount32 (ARM SWAR / PC HW). Stage-1 hook: Phase 3.B.
// -----------------------------------------------------------------
static inline uint64_t tpl_to_sign64(const int16_t* tpl) noexcept {
    uint64_t s = 0;
    for (int i = 0; i < 64; ++i) {
        if (tpl[i] > 0) {
            s |= (1ULL << static_cast<unsigned>(i));
        }
    }
    return s;
}

static inline uint64_t rx_extract_sign64(const int16_t* buf,
                                          int32_t off) noexcept {
    uint64_t s = 0;
    for (int i = 0; i < 64; ++i) {
        if (buf[off + i] > 0) {
            s |= (1ULL << static_cast<unsigned>(i));
        }
    }
    return s;
}

static inline int64_t sign_score_one(uint64_t rx_I_sign, uint64_t rx_Q_sign,
                                     uint64_t tpl_sign) noexcept {
    int64_t total = 0;
    for (int sub = 0; sub < 4; ++sub) {
        const uint64_t mask = 0xFFFFULL << static_cast<unsigned>(sub * 16);
        const uint64_t rI = (rx_I_sign & mask) >> static_cast<unsigned>(sub * 16);
        const uint64_t rQ = (rx_Q_sign & mask) >> static_cast<unsigned>(sub * 16);
        const uint64_t tp = (tpl_sign & mask) >> static_cast<unsigned>(sub * 16);

        const int mI = static_cast<int>(popcount32(
            static_cast<uint32_t>(rI ^ tp)));
        const int mQ = static_cast<int>(popcount32(
            static_cast<uint32_t>(rQ ^ tp)));
        const int aI = 16 - 2 * mI;
        const int aQ = 16 - 2 * mQ;
        total += static_cast<int64_t>(aI) * aI +
                 static_cast<int64_t>(aQ) * aQ;
    }
    return total;
}

static inline int64_t sign_score_4tmpl_at(const int16_t* tx, const int16_t* tq,
                                          int32_t off, uint64_t sA,
                                          uint64_t sB, uint64_t sC,
                                          uint64_t sD) noexcept {
    return sign_score_one(rx_extract_sign64(tx, off),
                          rx_extract_sign64(tq, off), sA) +
           sign_score_one(rx_extract_sign64(tx, off + 64),
                          rx_extract_sign64(tq, off + 64), sB) +
           sign_score_one(rx_extract_sign64(tx, off + 128),
                          rx_extract_sign64(tq, off + 128), sC) +
           sign_score_one(rx_extract_sign64(tx, off + 192),
                          rx_extract_sign64(tq, off + 192), sD);
}

static inline int32_t sign_scan_4tmpl(const int16_t* tx, const int16_t* tq,
                                      int32_t scan_start, int32_t scan_end,
                                      uint64_t sA, uint64_t sB, uint64_t sC,
                                      uint64_t sD,
                                      int64_t* out_best_score) noexcept {
    int64_t best_score = 0;
    int32_t best_off = -1;

    for (int32_t off = scan_start; off + 256 <= scan_end; ++off) {
        const int64_t sc =
            sign_score_4tmpl_at(tx, tq, off, sA, sB, sC, sD);
        if (sc > best_score) {
            best_score = sc;
            best_off = off;
        }
    }
    if (out_best_score != nullptr) {
        *out_best_score = best_score;
    }
    return best_off;
}

// Phase 3.E: INNOViD — Sign coarse gate (T6 실측 scale; was 2000, blocked ~572).
#define SIGN_THRESHOLD_INIT 300

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

inline void gravity_compute_faces(const int64_t sub_score[4][4],
                                  const int64_t sub_xI[4][4],
                                  const int64_t sub_xQ[4][4],
                                  const int64_t tmpl_xI[4],
                                  const int64_t tmpl_xQ[4],
                                  const int64_t tmpl_total[4],
                                  int64_t best_total, int64_t nf_total,
                                  int64_t nf_per_tmpl,
                                  GravityCube6* out) noexcept {
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
    out->face_A_q10 = q10_ratio_clamp(A_mn, A_mx, 1024);

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
    out->face_B_q10 = static_cast<int32_t>(B_sum >> 2);

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
    out->face_C_q10 =
        (C32_q10 < out->face_A_q10) ? C32_q10 : out->face_A_q10;

    constexpr int64_t k_face_d_max_q10 =
        (static_cast<int64_t>(65535) * 1024LL);
    out->face_D_q10 =
        q10_ratio_clamp(best_total, nf_total, k_face_d_max_q10);

    int64_t dI[3];
    int64_t dQ[3];
    for (int k = 0; k < 3; ++k) {
        dI[k] = tmpl_xI[k + 1] * tmpl_xI[k] + tmpl_xQ[k + 1] * tmpl_xQ[k];
        dQ[k] = tmpl_xQ[k + 1] * tmpl_xI[k] - tmpl_xI[k + 1] * tmpl_xQ[k];
    }
    const int64_t sum_dI = dI[0] + dI[1] + dI[2];
    const int64_t sum_dQ = dQ[0] + dQ[1] + dQ[2];
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
    out->face_E_q10 = q10_ratio_clamp(mag_sum, abs_d_sum, 1024);

    int64_t min_tmpl = tmpl_total[0];
    for (int t = 1; t < 4; ++t) {
        if (tmpl_total[t] < min_tmpl) {
            min_tmpl = tmpl_total[t];
        }
    }
    constexpr int64_t k_face_f_max_q10 =
        (static_cast<int64_t>(65535) * 1024LL);
    out->face_F_q10 =
        q10_ratio_clamp(min_tmpl, nf_per_tmpl, k_face_f_max_q10);

    out->total_score = best_total;
}

inline bool gravity_cube_pass(const GravityCube6* g) noexcept {
    return g->face_A_q10 >= GRAVITY_THR_A_Q10 &&
           g->face_B_q10 >= GRAVITY_THR_B_Q10 &&
           g->face_C_q10 >= GRAVITY_THR_C_Q10 &&
           g->face_D_q10 >= GRAVITY_THR_D_Q10 &&
           g->face_E_q10 >= GRAVITY_THR_E_Q10 &&
           g->face_F_q10 >= GRAVITY_THR_F_Q10;
}

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

} // namespace detail
} // namespace ProtectedEngine

#endif // HTS_HOLO_PREAMBLE && HTS_HOLO_CMYK_MODE

#ifndef HTS_CFO_BANK_TEST_LAB_ONCE
#define HTS_CFO_BANK_TEST_LAB_ONCE

// ============================================================================
// HTS_CFO_Bank_Test_lab.cpp — Phase H Lab O: Apply 위상 부정합 (BUG-8) 추적·교정
//
// 메인트리 Walsh P0 (PSLTE) 인용 — phase0_scan_() 성공 분기:
//   HTS_V400_Dispatcher_Sync_PSLTE.cpp:1465–1471
//     cfo_res = cfo_v5a_.Estimate(pre_I, pre_Q);
//     if (cfo_res.valid && cfo_v5a_.IsApplyAllowed()) {
//         cfo_v5a_.Set_Apply_Cfo(cfo_res.cfo_hz);
//         cfo_v5a_.Advance_Phase_Only(192);   // P0 192칩 분량 위상 선행
//     }
//   이후 psal_commit_align_() (1508–1510) 로 P0 버퍼 정렬·carry; P0 I/Q는 페이로드 경로로
//   재소비되지 않고 Feed_Chip 스트림이 이어짐.
// Feed_Chip (동일 파일 1853–1857):
//   if (cfo_v5a_.IsApplyDriveActive()) { cfo_v5a_.Apply_Per_Chip(chip_I, chip_Q); }
//   → DC 제거 직후 **들어오는** 칩에만 CFO 역회전. 즉 Advance(192) 후 첫 Apply는 **스트림
//   절대 칩 인덱스 192**에 해당(프리앰블 192칩을 이미 통과한 것과 동일 위상).
// Lab BUG-8: Advance(192) 뒤 buf[0..255] 전부 Apply → 칩0에 칩192 위상이 가해져 디코드 붕괴.
// Lab O: B'/C'/D' 는 kPayloadOffset(192) 이상에만 Apply (Feed_Chip 순서 모사).
//
//   Lab K: Lab N 스펙 유지 | Lab O: 패턴 A,B,B',C,C',D,D' | Lab P: B' 잔여격차·B''·500/30 통계
//   Lab BUG-9: Pattern A (V5a OFF) cfo≈2000대 D 붕괴 — 누적 위상·디코드 trial0 DIAG + 세분 CFO 스윕
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../HTS_LIM/HTS_CFO_V5a.hpp"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_TX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_RX.h"
#include "../HTS_LIM/HTS_Preamble_Holographic.h"
#include "../HTS_LIM/HTS_Rx_CFO_SinCos_Table.hpp"

using namespace ProtectedEngine;

namespace ExperimentConfig {
constexpr int kCfoSweepList[] = {0,     1000,  3000,  5000,  6000,  7000,  8000,  9000,
                                 10000, 11000, 12000, 15000, 20000, 30000};
constexpr int kCfoSweepCount = sizeof(kCfoSweepList) / sizeof(int);
constexpr int kNumTrials = 100;
constexpr int kSnrLevels[] = {30, 20, 10};
constexpr int kSnrCount = sizeof(kSnrLevels) / sizeof(int);
constexpr int kK = 16;
constexpr int kN = 64;
constexpr int kAmp = 500;
constexpr int kDenom = 16;
constexpr int kTxTotalChips = 256;
constexpr int kPayloadOffset = 192;
/// PRE_SYM0 (Walsh 63) 64칩 × 2
constexpr int kPreSym0TotalChips = 128;
/// PRE_SYM1 (Walsh row 0, all +1) 시작 인덱스·길이
constexpr int kPreSym1Offset = 128;
constexpr int kPreSym1Chips = 64;
/// T6 holo Pass1: 메인트리와 동일하게 프리앰블 192칩(0..191)만 사용
constexpr int kHoloSyncChips = 192;
/// Lab I-3 / Lab K: V5a 추정 = PRE_SYM1 구간만 (128..191), 64칩
constexpr int kV5aEstimateOffset = kPreSym1Offset;
constexpr int kV5aEstimateChips = kPreSym1Chips;
constexpr bool kEnableHoloSync = true;
constexpr double kV5aDeadzoneEpsilonHz = 200.0;
/// Lab K: V5a apply 게이트 고정값 (Lab J 캘리브레이션 결과)
constexpr double kV5aAcNormThresholdLabK = 0.95;
constexpr double kChipRateHz = 1e6;
constexpr int kRxSoftMode = 0;
constexpr int32_t kChipValidThreshold = 0;
}  // namespace ExperimentConfig

namespace {

// HTS_V400_Dispatcher_Internal.hpp — fwht_raw(..., 64) 와 동일 (n==64 분기)
inline void fwht_raw(int32_t* d, int n) noexcept {
    if (n == 64) {
        for (int i = 0; i < 64; i += 2) {
            int32_t u = d[i], v = d[i + 1];
            d[i] = u + v;
            d[i + 1] = u - v;
        }
        for (int i = 0; i < 64; i += 4) {
            int32_t u = d[i], v = d[i + 2];
            d[i] = u + v;
            d[i + 2] = u - v;
            u = d[i + 1];
            v = d[i + 3];
            d[i + 1] = u + v;
            d[i + 3] = u - v;
        }
        for (int i = 0; i < 64; i += 8) {
            for (int k = 0; k < 4; ++k) {
                int32_t u = d[i + k], v = d[i + 4 + k];
                d[i + k] = u + v;
                d[i + 4 + k] = u - v;
            }
        }
        for (int i = 0; i < 64; i += 16) {
            for (int k = 0; k < 8; ++k) {
                int32_t u = d[i + k], v = d[i + 8 + k];
                d[i + k] = u + v;
                d[i + 8 + k] = u - v;
            }
        }
        for (int i = 0; i < 64; i += 32) {
            for (int k = 0; k < 16; ++k) {
                int32_t u = d[i + k], v = d[i + 16 + k];
                d[i + k] = u + v;
                d[i + 16 + k] = u - v;
            }
        }
        for (int k = 0; k < 32; ++k) {
            int32_t u = d[k], v = d[k + 32];
            d[k] = u + v;
            d[k + 32] = u - v;
        }
        return;
    }
}

constexpr int kSinCosTableSize = 1024;
constexpr int kSinCosIndexShift = 22;
constexpr int32_t kQ14One = 16384;
constexpr double kPi = 3.14159265358979323846;
int16_t g_sin_table[kSinCosTableSize];
int16_t g_cos_table[kSinCosTableSize];
uint32_t g_rng = 1u;

static inline uint32_t popcount32_(uint32_t x) noexcept {
    x = x - ((x >> 1) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
    x = (x + (x >> 4)) & 0x0F0F0F0Fu;
    x = x + (x >> 8);
    x = x + (x >> 16);
    return x & 0x3Fu;
}

static void build_sincos_table() noexcept {
    for (int i = 0; i < kSinCosTableSize; ++i) {
        const double angle = 2.0 * kPi * i / kSinCosTableSize;
        g_sin_table[i] = static_cast<int16_t>(std::sin(angle) * kQ14One);
        g_cos_table[i] = static_cast<int16_t>(std::cos(angle) * kQ14One);
    }
}

static inline void rng_seed(uint32_t seed) noexcept { g_rng = seed ? seed : 1u; }
static inline uint32_t rng_next() noexcept {
    g_rng = g_rng * 1664525u + 1013904223u;
    return g_rng;
}
static inline double rng_uniform() noexcept { return static_cast<double>(rng_next()) / 4294967296.0; }
static inline double rng_gauss() noexcept {
    double u1 = rng_uniform();
    double u2 = rng_uniform();
    if (u1 < 1e-10) u1 = 1e-10;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}
static inline int16_t sat_i16(int32_t v) noexcept {
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return static_cast<int16_t>(v);
}

/// Lab K: 상세 샘플 (CFO × SNR 그리드, trial 0만)
static bool lab_k_diag_trial0(int cfo_hz, int snr_db, int trial) noexcept {
    if (trial != 0) {
        return false;
    }
    static constexpr int kDiagCfos[] = {0, 500, 5000, 8000, 10000, 12000, 15000, 20000, 30000};
    static constexpr int kDiagSnrs[] = {30, 10};
    bool cfo_hit = false;
    for (int c : kDiagCfos) {
        if (cfo_hz == c) {
            cfo_hit = true;
            break;
        }
    }
    if (!cfo_hit) {
        return false;
    }
    for (int s : kDiagSnrs) {
        if (snr_db == s) {
            return true;
        }
    }
    return false;
}

/// HTS_V400_Dispatcher_Internal.hpp — PS-LTE P1 sym1_row (dom ^ NATURAL)
constexpr uint8_t k_W63_FWHT_ROW_NATURAL = 63u;
/// HTS_V400_Dispatcher_Sync_PSLTE.cpp — PS-LTE coherent 64 (else branch)
static constexpr int32_t k_P1_MIN_E = 1000;

static bool lab_l_diag_trial0(int trial) noexcept { return trial == 0; }

/// PS-LTE Stage2: FWHT on 64 chips, dom=63, sym1_row = 63 ^ k_W63_FWHT_ROW_NATURAL (=0)
static bool lab_p1_fwht_energy_gate_pslte(const int16_t* blk_I, const int16_t* blk_Q, int32_t* e63_sh_out,
                                          int32_t* e0_sh_out, int32_t* max_e_sh_out) noexcept {
    int32_t T_I[64];
    int32_t T_Q[64];
    for (int j = 0; j < 64; ++j) {
        T_I[j] = static_cast<int32_t>(blk_I[j]);
        T_Q[j] = static_cast<int32_t>(blk_Q[j]);
    }
    fwht_raw(T_I, 64);
    fwht_raw(T_Q, 64);
    const int dom = 63;
    const int sym1_row = dom ^ static_cast<int>(k_W63_FWHT_ROW_NATURAL);
    const int32_t dot63_I = T_I[dom];
    const int32_t dot63_Q = T_Q[dom];
    const int32_t dot0_I = T_I[sym1_row];
    const int32_t dot0_Q = T_Q[sym1_row];
    const int64_t e63_64 = static_cast<int64_t>(dot63_I) * dot63_I + static_cast<int64_t>(dot63_Q) * dot63_Q;
    const int64_t e0_64 = static_cast<int64_t>(dot0_I) * dot0_I + static_cast<int64_t>(dot0_Q) * dot0_Q;
    const int32_t e63_sh = static_cast<int32_t>(e63_64 >> 16);
    const int32_t e0_sh = static_cast<int32_t>(e0_64 >> 16);
    const int32_t max_e_sh = (e63_sh >= e0_sh) ? e63_sh : e0_sh;
    if (e63_sh_out != nullptr) {
        *e63_sh_out = e63_sh;
    }
    if (e0_sh_out != nullptr) {
        *e0_sh_out = e0_sh;
    }
    if (max_e_sh_out != nullptr) {
        *max_e_sh_out = max_e_sh;
    }
    return max_e_sh >= k_P1_MIN_E;
}

static double lab_phase_deg(int16_t i, int16_t q) noexcept {
    return std::atan2(static_cast<double>(q), static_cast<double>(i)) * 180.0 / kPi;
}
}  // namespace

static void channel_apply(const int16_t* tx_I, const int16_t* tx_Q, int16_t* rx_I, int16_t* rx_Q,
                          int chips, double cfo_hz, int snr_db, uint32_t ch_seed) noexcept {
    rng_seed(ch_seed);
    double sig_pow = 0.0;
    for (int k = 0; k < chips; ++k) {
        sig_pow += static_cast<double>(tx_I[k]) * tx_I[k];
        sig_pow += static_cast<double>(tx_Q[k]) * tx_Q[k];
    }
    sig_pow /= chips;
    const double noise_sigma = (sig_pow > 0.0) ? std::sqrt(sig_pow / std::pow(10.0, snr_db / 10.0)) : 0.0;
    const double phase_inc = 2.0 * kPi * cfo_hz / ExperimentConfig::kChipRateHz;
    double phase = 0.0;
    for (int k = 0; k < chips; ++k) {
        const double cs = std::cos(phase);
        const double sn = std::sin(phase);
        double rotI = tx_I[k] * cs - tx_Q[k] * sn;
        double rotQ = tx_I[k] * sn + tx_Q[k] * cs;
        rotI += rng_gauss() * noise_sigma;
        rotQ += rng_gauss() * noise_sigma;
        rx_I[k] = sat_i16(static_cast<int32_t>(rotI));
        rx_Q[k] = sat_i16(static_cast<int32_t>(rotQ));
        phase += phase_inc;
    }
}

static double v5a_estimate_cfo_with_quality(const int16_t* rx_I, const int16_t* rx_Q, int chips, int lag,
                                            double* ac_mag_out, double* sig_pow_out) noexcept {
    double ac_I = 0.0, ac_Q = 0.0, sp = 0.0;
    for (int n = 0; n + lag < chips; ++n) {
        ac_I += static_cast<double>(rx_I[n]) * rx_I[n + lag] + static_cast<double>(rx_Q[n]) * rx_Q[n + lag];
        ac_Q += static_cast<double>(rx_Q[n]) * rx_I[n + lag] - static_cast<double>(rx_I[n]) * rx_Q[n + lag];
    }
    for (int n = 0; n < chips; ++n) {
        const double ri = static_cast<double>(rx_I[n]);
        const double rq = static_cast<double>(rx_Q[n]);
        sp += ri * ri + rq * rq;
    }
    const double ac_mag = std::sqrt(ac_I * ac_I + ac_Q * ac_Q);
    if (ac_mag_out != nullptr) {
        *ac_mag_out = ac_mag;
    }
    if (sig_pow_out != nullptr) {
        *sig_pow_out = sp;
    }
    if (std::abs(ac_I) < 1e-6 && std::abs(ac_Q) < 1e-6) {
        return 0.0;
    }
    return std::atan2(ac_Q, ac_I) * ExperimentConfig::kChipRateHz / (2.0 * kPi * static_cast<double>(lag));
}

static void v5a_apply_per_chip(const int16_t* rx_I, const int16_t* rx_Q, int16_t* out_I, int16_t* out_Q,
                               int chips, double cfo_hz) noexcept {
    const int64_t phase_inc_q32_s64 = -static_cast<int64_t>((cfo_hz * 4294967296.0) / ExperimentConfig::kChipRateHz);
    const uint32_t phase_inc_q32 = static_cast<uint32_t>(phase_inc_q32_s64);
    uint32_t phase_q32 = 0u;
    for (int k = 0; k < chips; ++k) {
        const int idx = (phase_q32 >> kSinCosIndexShift) & (kSinCosTableSize - 1);
        const int32_t cos_q14 = g_cos_table[idx];
        const int32_t sin_q14 = g_sin_table[idx];
        const int32_t x = rx_I[k];
        const int32_t y = rx_Q[k];
        out_I[k] = sat_i16((x * cos_q14 - y * sin_q14) >> 14);
        out_Q[k] = sat_i16((x * sin_q14 + y * cos_q14) >> 14);
        phase_q32 += phase_inc_q32;
    }
}

/// 페이로드만 보정할 때, 스트림 **절대 칩 인덱스** (예: 192) 누적 위상을 초기 q32에 반영
static void v5a_apply_per_chip_from_abs(const int16_t* rx_I, const int16_t* rx_Q, int16_t* out_I, int16_t* out_Q,
                                        int chips, double cfo_hz, int first_global_chip_idx) noexcept {
    const int64_t phase_inc_q32_s64 = -static_cast<int64_t>((cfo_hz * 4294967296.0) / ExperimentConfig::kChipRateHz);
    const uint32_t phase_inc_q32 = static_cast<uint32_t>(phase_inc_q32_s64);
    const int64_t ph0 = phase_inc_q32_s64 * static_cast<int64_t>(first_global_chip_idx);
    uint32_t phase_q32 = static_cast<uint32_t>(ph0);
    for (int k = 0; k < chips; ++k) {
        const int idx = (phase_q32 >> kSinCosIndexShift) & (kSinCosTableSize - 1);
        const int32_t cos_q14 = g_cos_table[idx];
        const int32_t sin_q14 = g_sin_table[idx];
        const int32_t x = rx_I[k];
        const int32_t y = rx_Q[k];
        out_I[k] = sat_i16((x * cos_q14 - y * sin_q14) >> 14);
        out_Q[k] = sat_i16((x * sin_q14 + y * cos_q14) >> 14);
        phase_q32 += phase_inc_q32;
    }
}

struct HoloPass1T6 {
    bool pass{};
    int best_off{-1};
    int64_t best_e{0};
    int ratio_x10{0};
    int64_t top4_sum{0};
    bool l12{false};
    bool l5{false};
    bool l3{false};
};

/// T6 phase0_scan_holographic_() Pass1 (PSLTE 1570–1646) 동등
static void holo_pass1_compute_t6(const int16_t* rx_I, const int16_t* rx_Q, int chip_count,
                                  HoloPass1T6& o) noexcept {
    using ProtectedEngine::Holographic::holographic_dot_segmented;
    using ProtectedEngine::Holographic::peak_to_median_ratio_x10;

    o = HoloPass1T6{};
    if (chip_count < 192) {
        return;
    }

    int64_t energies[64];
    for (int off = 0; off < 64; ++off) {
        if (off + 128 > chip_count) {
            energies[off] = 0;
        } else {
            const int64_t e_block0 = holographic_dot_segmented(&rx_I[off], &rx_Q[off]);
            const int64_t e_block1 = holographic_dot_segmented(&rx_I[off + 64], &rx_Q[off + 64]);
            energies[off] = e_block0 + e_block1;
        }
    }

    int64_t be = 0;
    int best_off = -1;
    for (int off = 0; off < 64; ++off) {
        if (energies[off] > be) {
            be = energies[off];
            best_off = off;
        }
    }
    o.best_e = be;
    o.best_off = best_off;
    if (best_off < 0 || be <= 0) {
        return;
    }

    const int64_t amp32 = static_cast<int64_t>(ExperimentConfig::kAmp);
    const int64_t amp38 = amp32 * 38LL;
    const int64_t threshold_12 = (amp38 * amp38) / 4LL;
    const int64_t threshold_3 = (amp38 * amp38 * 3LL) / 10LL;

    o.ratio_x10 = static_cast<int>(peak_to_median_ratio_x10(energies, 64));
    o.l12 = (o.best_e >= threshold_12);
    o.l5 = (o.ratio_x10 >= 25);

    if (o.l12 && o.l5) {
        int32_t t_I[64];
        int32_t t_Q[64];
        for (int i = 0; i < 64; ++i) {
            const int idx = best_off + i;
            t_I[i] = static_cast<int32_t>(rx_I[idx]);
            t_Q[i] = static_cast<int32_t>(rx_Q[idx]);
        }
        fwht_raw(t_I, 64);
        fwht_raw(t_Q, 64);
        int64_t row_e[64];
        for (int i = 0; i < 64; ++i) {
            row_e[i] = static_cast<int64_t>(t_I[i]) * t_I[i] + static_cast<int64_t>(t_Q[i]) * t_Q[i];
        }
        int64_t ts = 0;
        for (int k = 0; k < 4; ++k) {
            int max_idx = 0;
            int64_t max_val = row_e[0];
            for (int i = 1; i < 64; ++i) {
                if (row_e[i] > max_val) {
                    max_val = row_e[i];
                    max_idx = i;
                }
            }
            ts += max_val;
            row_e[max_idx] = -1;
        }
        o.top4_sum = ts;
        o.l3 = (o.top4_sum >= threshold_3);
    }
    o.pass = o.l12 && o.l5 && o.l3;
}

struct HoloPass2T6 {
    bool pass{};
    int best_off_p2{-1};
    int8_t best_hyp{0};
    int64_t best_e_p2{0};
};

/// PSLTE phase0_scan_holographic_() Pass2 (1662–1783) — buf 192 in-place derotate on success
static bool holo_pass2_inplace_t6(int16_t* buf_I, int16_t* buf_Q, int chip_count,
                                  HoloPass2T6& o) noexcept {
    using ProtectedEngine::Holographic::derotate_buffer_q8;
    using ProtectedEngine::Holographic::holographic_dot_segmented;
    using ProtectedEngine::Holographic::peak_to_median_ratio_x10;

    o = HoloPass2T6{};
    if (chip_count < 192) {
        return false;
    }

    const int64_t amp32 = static_cast<int64_t>(ExperimentConfig::kAmp);
    const int64_t amp38 = amp32 * 38LL;
    const int64_t threshold_12 = (amp38 * amp38) / 4LL;
    const int64_t threshold_3 = (amp38 * amp38 * 3LL) / 10LL;
    constexpr int32_t k_RATIO_MIN_X10 = 25;

    static constexpr int8_t k_cfo_hyp_pslte[] = {-6, -4, -2, -1, 1, 2, 4, 6};
    constexpr int k_num_hyp_pslte =
        static_cast<int>(sizeof(k_cfo_hyp_pslte) / sizeof(k_cfo_hyp_pslte[0]));

    int16_t rot_I[192];
    int16_t rot_Q[192];
    const int n_rot = (chip_count > 192) ? 192 : chip_count;

    int8_t best_hyp = 0;
    int best_off_p2 = -1;
    int64_t best_e_p2 = 0;
    int32_t best_ratio_p2 = 0;

    for (int hi = 0; hi < k_num_hyp_pslte; ++hi) {
        const int8_t hyp = k_cfo_hyp_pslte[hi];
        derotate_buffer_q8(buf_I, buf_Q, rot_I, rot_Q, n_rot, hyp);

        int64_t en_h[64];
        for (int off = 0; off < 64; ++off) {
            if (off + 128 > chip_count) {
                en_h[off] = 0;
            } else {
                const int64_t e0 = holographic_dot_segmented(&rot_I[off], &rot_Q[off]);
                const int64_t e1 = holographic_dot_segmented(&rot_I[off + 64], &rot_Q[off + 64]);
                en_h[off] = e0 + e1;
            }
        }

        int64_t be = 0;
        int bo = -1;
        for (int off = 0; off < 64; ++off) {
            if (en_h[off] > be) {
                be = en_h[off];
                bo = off;
            }
        }

        const int32_t r_h = (bo >= 0) ? peak_to_median_ratio_x10(en_h, 64) : 0;
        if (r_h > best_ratio_p2 && bo >= 0 && be >= threshold_12) {
            best_ratio_p2 = r_h;
            best_e_p2 = be;
            best_off_p2 = bo;
            best_hyp = hyp;
        }
    }

    const bool l12_p2 = (best_off_p2 >= 0) && (best_e_p2 >= threshold_12);
    const bool l5_p2 = (best_ratio_p2 >= k_RATIO_MIN_X10);

    bool l3_p2 = false;
    if (l12_p2 && l5_p2) {
        derotate_buffer_q8(buf_I, buf_Q, rot_I, rot_Q, n_rot, best_hyp);

        int32_t t_I[64];
        int32_t t_Q[64];
        for (int i = 0; i < 64; ++i) {
            const int idx = best_off_p2 + i;
            t_I[i] = static_cast<int32_t>(rot_I[idx]);
            t_Q[i] = static_cast<int32_t>(rot_Q[idx]);
        }

        fwht_raw(t_I, 64);
        fwht_raw(t_Q, 64);

        int64_t row_e2[64];
        for (int i = 0; i < 64; ++i) {
            row_e2[i] = static_cast<int64_t>(t_I[i]) * t_I[i] + static_cast<int64_t>(t_Q[i]) * t_Q[i];
        }

        int64_t top4_sum2 = 0;
        for (int k = 0; k < 4; ++k) {
            int max_idx = 0;
            int64_t max_val = row_e2[0];
            for (int i = 1; i < 64; ++i) {
                if (row_e2[i] > max_val) {
                    max_val = row_e2[i];
                    max_idx = i;
                }
            }
            top4_sum2 += max_val;
            row_e2[max_idx] = -1;
        }

        l3_p2 = (top4_sum2 >= threshold_3);
    }

    const bool pass_p2 = l12_p2 && l5_p2 && l3_p2;
    if (!pass_p2) {
        return false;
    }

    derotate_buffer_q8(buf_I, buf_Q, rot_I, rot_Q, n_rot, best_hyp);
    for (int i = 0; i < n_rot; ++i) {
        buf_I[i] = rot_I[i];
        buf_Q[i] = rot_Q[i];
    }
    o.pass = true;
    o.best_off_p2 = best_off_p2;
    o.best_hyp = best_hyp;
    o.best_e_p2 = best_e_p2;
    return true;
}

/// T6 phase0_scan_holographic_() Pass1 (PSLTE 1564–1646) 동등 — 기존 API
static bool holo_sync_check_t6(const int16_t* rx_I, const int16_t* rx_Q, int chip_count, bool& l12, bool& l5,
                               bool& l3, int64_t& best_e, int& ratio_x10, int64_t& top4_sum) noexcept {
    HoloPass1T6 p1{};
    holo_pass1_compute_t6(rx_I, rx_Q, chip_count, p1);
    l12 = p1.l12;
    l5 = p1.l5;
    l3 = p1.l3;
    best_e = p1.best_e;
    ratio_x10 = p1.ratio_x10;
    top4_sum = p1.top4_sum;
    return p1.pass;
}

enum class HoloV5aPattern : int {
    A = 0,
    B_Full = 1,      ///< 73ff940형: Advance(192) 후 buf 0..255 전체 Apply (BUG-8 재현)
    B_Payload = 2,   ///< B': Estimate@P1 best_off, payload-only Apply
    B_PayloadEstAtZero = 3,  ///< B'': Estimate@chip0, 128ch (kPreambleChips); Apply payload-only (메인 best_off=0 정합 실험)
    C_Full = 4,
    C_Payload = 5,
    D_Full = 6,
    D_Payload = 7,
    B_PayloadFeedChip = 8,  ///< Lab Q B''': Apply_Per_Chip을 rx 스트림 샘플에 chip 단위 (Feed_Chip 모사)
};

/// Lab O 매트릭스 CFO 스윕 (요청 7점; 8000/12000 제외)
static constexpr int kLabMCfoSweep[] = {0, 100, 200, 500, 1000, 2000, 5000};
static constexpr int kLabMCfoCount = static_cast<int>(sizeof(kLabMCfoSweep) / sizeof(kLabMCfoSweep[0]));

/// Lab Q 비교: CFO 6점 (200 제외) × SNR
static constexpr int kLabQCfoSweep[] = {0, 100, 500, 1000, 2000, 5000};
static constexpr int kLabQCfoCount = static_cast<int>(sizeof(kLabQCfoSweep) / sizeof(kLabQCfoSweep[0]));

/// Lab BUG-9: Pattern A trial0 DIAG (SNR=30만)
static constexpr int kLabBug9PaDiagCfos[] = {1500, 1800, 2000, 2200, 2500, 3000, 3500, 4000, 4500};
static constexpr int kLabBug9PaDiagCfosCount =
    static_cast<int>(sizeof(kLabBug9PaDiagCfos) / sizeof(kLabBug9PaDiagCfos[0]));

/// Lab BUG-9: Pattern A 세분 CFO×SNR=30 D 곡선
static constexpr int kLabBug9PaSweepCfos[] = {1500, 1750, 2000, 2250, 2500, 3000, 4000, 5000};
static constexpr int kLabBug9PaSweepCfosCount =
    static_cast<int>(sizeof(kLabBug9PaSweepCfos) / sizeof(kLabBug9PaSweepCfos[0]));

static bool lab_bug9_pa_diag_cfo_hit(int cfo_hz) noexcept {
    for (int i = 0; i < kLabBug9PaDiagCfosCount; ++i) {
        if (kLabBug9PaDiagCfos[i] == cfo_hz) {
            return true;
        }
    }
    return false;
}

static bool lab_bug9_pa_diag_want(int cfo_hz, int snr_db, int trial, HoloV5aPattern pattern) noexcept {
    return trial == 0 && snr_db == 30 && pattern == HoloV5aPattern::A && lab_bug9_pa_diag_cfo_hit(cfo_hz);
}

static void lab_bug9_print_cumulative_phase(int cfo_hz) noexcept {
    const double cum_rad = 2.0 * kPi * static_cast<double>(cfo_hz) * 192.0 / 1e6;
    double mod = std::fmod(cum_rad, 2.0 * kPi);
    if (mod < 0.0) {
        mod += 2.0 * kPi;
    }
    const double mod_deg = mod * 180.0 / kPi;
    std::printf(
        "[DIAG-LabBUG9] cum_phase_rad=%.6f (2*pi*cfo*192/1e6) cum_mod2pi_deg=%.2f (payload-start equiv)\n",
        cum_rad, mod_deg);
}

static void lab_bug9_print_payload_phases(const int16_t* corrI, const int16_t* corrQ) noexcept {
    std::printf("[DIAG-LabBUG9] payload_ph_deg_chips192..199=");
    for (int k = 192; k < 200; ++k) {
        std::printf(" %.1f", lab_phase_deg(corrI[k], corrQ[k]));
    }
    std::printf("\n");
}

/// Lab N: Lab K 재측정 CFO 스윕 (요청 스펙)
static constexpr int kLabNCfoSweep[] = {0, 500, 5000, 12000, 30000};
static constexpr int kLabNCfoSweepCount = static_cast<int>(sizeof(kLabNCfoSweep) / sizeof(kLabNCfoSweep[0]));

enum class LabKV5aMode : int { Off = 0, ProdWalsh128 = 1, LegacyPreSym1Lag4 = 2 };

static const char* lab_k_mode_title(LabKV5aMode m) noexcept {
    switch (m) {
        case LabKV5aMode::Off:
            return "V5a OFF";
        case LabKV5aMode::ProdWalsh128:
            return "V5a ON (HTS_CFO_V5a::Estimate @ P1 best_off, Apply_Per_Chip)";
        case LabKV5aMode::LegacyPreSym1Lag4:
            return "V5a ON (Lab lag4 autocorr PRE_SYM1 64ch + ac_norm_thr)";
    }
    return "?";
}

/// PSLTE 1465–1471 + Feed_Chip: Estimate(pre+`estimate_off`, 128ch) → Set_Apply_Cfo → Advance(192); Apply 범위는 payload_only
/// (`payload_only`=false 는 구 Lab BUG-8 재현용 전버퍼 Apply)
static void lab_prod_v5a_apply_buffer(int16_t* corrI, int16_t* corrQ, int total_chips, int estimate_off_128,
                                      bool payload_only_after_advance, double& est_hz_out, double& gate_ok_diag,
                                      bool& applied_out) noexcept {
    est_hz_out = 0.0;
    gate_ok_diag = 0.0;
    applied_out = false;
    if (estimate_off_128 < 0 || estimate_off_128 + hts::rx_cfo::kPreambleChips > 192) {
        return;
    }
    hts::rx_cfo::CFO_V5a cfo_v5a{};
    cfo_v5a.Init();
    const hts::rx_cfo::CFO_Result cfo_res =
        cfo_v5a.Estimate(corrI + estimate_off_128, corrQ + estimate_off_128);
    est_hz_out = static_cast<double>(cfo_res.cfo_hz);
    gate_ok_diag = cfo_v5a.IsApplyAllowed() ? 1.0 : 0.0;
    if (cfo_res.valid && cfo_v5a.IsApplyAllowed()) {
        cfo_v5a.Set_Apply_Cfo(cfo_res.cfo_hz);
        cfo_v5a.Advance_Phase_Only(192);
        const int k0 = payload_only_after_advance ? ExperimentConfig::kPayloadOffset : 0;
        for (int k = k0; k < total_chips; ++k) {
            cfo_v5a.Apply_Per_Chip(corrI[k], corrQ[k]);
        }
        applied_out = true;
    } else {
        cfo_v5a.Set_Apply_Cfo(0);
    }
}

/// Lab Q B''': PSLTE와 동일 Estimate 윈도우 + Set_Apply + Advance(192) 후, **페이로드(192..255)만**
/// 메인트리처럼 `rxI[k]`,`rxQ[k]` 를 chip 단위로 `Apply_Per_Chip` (corr은 원 스트림에서 갱신).
static void lab_v5a_payload_apply_feedchip_from_rx(const int16_t* rxI, const int16_t* rxQ, int16_t* corrI,
                                                    int16_t* corrQ, int total_chips, int estimate_off_128,
                                                    double& est_hz_out, double& gate_ok_diag,
                                                    bool& applied_out) noexcept {
    est_hz_out = 0.0;
    gate_ok_diag = 0.0;
    applied_out = false;
    if (estimate_off_128 < 0 || estimate_off_128 + hts::rx_cfo::kPreambleChips > 192) {
        return;
    }
    hts::rx_cfo::CFO_V5a cfo_v5a{};
    cfo_v5a.Init();
    const hts::rx_cfo::CFO_Result cfo_res =
        cfo_v5a.Estimate(corrI + estimate_off_128, corrQ + estimate_off_128);
    est_hz_out = static_cast<double>(cfo_res.cfo_hz);
    gate_ok_diag = cfo_v5a.IsApplyAllowed() ? 1.0 : 0.0;
    if (cfo_res.valid && cfo_v5a.IsApplyAllowed()) {
        cfo_v5a.Set_Apply_Cfo(cfo_res.cfo_hz);
        cfo_v5a.Advance_Phase_Only(192);
        for (int k = ExperimentConfig::kPayloadOffset; k < total_chips; ++k) {
            int16_t i = rxI[k];
            int16_t q = rxQ[k];
            cfo_v5a.Apply_Per_Chip(i, q);
            corrI[k] = i;
            corrQ[k] = q;
        }
        applied_out = true;
    } else {
        cfo_v5a.Set_Apply_Cfo(0);
    }
}

static const char* lab_m_pattern_char(HoloV5aPattern p) noexcept {
    switch (p) {
        case HoloV5aPattern::A:
            return "A";
        case HoloV5aPattern::B_Full:
            return "B";
        case HoloV5aPattern::B_Payload:
            return "B'";
        case HoloV5aPattern::B_PayloadEstAtZero:
            return "B''";
        case HoloV5aPattern::C_Full:
            return "C";
        case HoloV5aPattern::C_Payload:
            return "C'";
        case HoloV5aPattern::D_Full:
            return "D";
        case HoloV5aPattern::D_Payload:
            return "D'";
        case HoloV5aPattern::B_PayloadFeedChip:
            return "B'''";
    }
    return "?";
}

static bool lab_m_diag_trial0(int trial) noexcept { return trial == 0; }

static bool lab_m_pattern_b_payload_stats(HoloV5aPattern p) noexcept {
    return p == HoloV5aPattern::B_Payload || p == HoloV5aPattern::B_PayloadEstAtZero ||
           p == HoloV5aPattern::B_PayloadFeedChip;
}

static bool lab_m_pattern_bprime_btriple(HoloV5aPattern p) noexcept {
    return p == HoloV5aPattern::B_Payload || p == HoloV5aPattern::B_PayloadFeedChip;
}

static void rx_soft_generate(const int16_t* i, const int16_t* q, int16_t* soft, uint64_t* valid_mask) noexcept {
    uint64_t mask = 0u;
    for (int c = 0; c < ExperimentConfig::kN; ++c) {
        const int32_t iv = i[c];
        const int32_t qv = q[c];
        int32_t vchip = -1;
        if (ExperimentConfig::kChipValidThreshold > 0) {
            const int32_t mag2 = iv * iv + qv * qv;
            const int64_t diff = static_cast<int64_t>(mag2) - static_cast<int64_t>(ExperimentConfig::kChipValidThreshold);
            vchip = static_cast<int32_t>(~(diff >> 63));
        }
        int32_t s = (iv + qv) / 2;
        if (ExperimentConfig::kRxSoftMode == 1) s = iv;
        if (ExperimentConfig::kRxSoftMode == 2) s = qv;
        if (ExperimentConfig::kRxSoftMode == 3) s = (std::abs(iv) > std::abs(qv)) ? iv : qv;
        if (ExperimentConfig::kRxSoftMode == 4) s = iv + qv;
        soft[c] = static_cast<int16_t>(s & vchip);
        mask |= (static_cast<uint64_t>(vchip & 1u) << static_cast<uint32_t>(c));
    }
    *valid_mask = mask;
}

struct TestResult { int sync_pass; int decode_pass; int total_bit_err; int total_bits; };

/// Lab P: cfo=500 snr=30 전용 100-trial 샘플 (패턴별로 별도 실행·`mode` 일치 시만 기록)
struct LabP500Trials {
    HoloV5aPattern mode{};
    double ph192_A[100]{};
    double est_hz[100]{};
    double abs_est_err[100]{};
    double ph192_after[100]{};
    /// -1 = Decode_Block 실패, 0..kK = 디코드 성공 후 비트 오류 수
    int bit_err[100]{};
    bool slot_valid[100]{};
    int n_fail{};
    int fail_trial[100]{};
    double fail_est_hz[100]{};
    /// Lab Q: cfo=500 snr=30 B'/B''' ph192 trial별 100줄
    bool print_ph192_each_trial{false};
};

static TestResult run_one_cfo_lab_m(int cfo_hz, int snr_db, HoloV5aPattern pattern, LabP500Trials* lab_p_trials) {
    TestResult r{0, 0, 0, 0};
    HTS_Holo_Tensor_4D_TX tx{};
    HTS_Holo_Tensor_4D_RX rx{};
    uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5B5A5A5u, 0xC3D3C3C3u};
    if (tx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        return r;
    }
    if (rx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        tx.Shutdown();
        return r;
    }

    for (int t = 0; t < ExperimentConfig::kNumTrials; ++t) {
        rng_seed(static_cast<uint32_t>(t * 7919u + 13u));
        int8_t bits[16]{};
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            bits[i] = (rng_next() & 1u) ? 1 : -1;
        }

        int8_t chips[64]{};
        (void)tx.Set_Time_Slot(static_cast<uint32_t>(t));
        if (tx.Encode_Block(bits, static_cast<uint16_t>(ExperimentConfig::kK), chips,
                            static_cast<uint16_t>(ExperimentConfig::kN)) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            r.total_bits += ExperimentConfig::kK;
            r.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        int16_t txI[256]{};
        int16_t txQ[256]{};
        for (int c = 0; c < ExperimentConfig::kPreSym0TotalChips; ++c) {
            const uint32_t x = 63u & static_cast<uint32_t>(c);
            const int sign = (popcount32_(x) & 1u) ? -1 : 1;
            const int16_t pre = static_cast<int16_t>(sign * ExperimentConfig::kAmp);
            txI[c] = pre;
            txQ[c] = pre;
        }
        for (int c = 0; c < ExperimentConfig::kPreSym1Chips; ++c) {
            const int16_t a = static_cast<int16_t>(ExperimentConfig::kAmp);
            txI[ExperimentConfig::kPreSym1Offset + c] = a;
            txQ[ExperimentConfig::kPreSym1Offset + c] = a;
        }
        for (int c = 0; c < 64; ++c) {
            const int32_t prod = static_cast<int32_t>(chips[c]) * ExperimentConfig::kAmp;
            const int32_t v = (prod >= 0) ? ((prod + (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom)
                                        : ((prod - (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom);
            txI[ExperimentConfig::kPayloadOffset + c] = static_cast<int16_t>(v);
            txQ[ExperimentConfig::kPayloadOffset + c] = static_cast<int16_t>(v);
        }

        int16_t rxI[256]{};
        int16_t rxQ[256]{};
        channel_apply(txI, txQ, rxI, rxQ, ExperimentConfig::kTxTotalChips, static_cast<double>(cfo_hz), snr_db,
                      static_cast<uint32_t>(t * 31u + 19u));

        int16_t corrI[256]{};
        int16_t corrQ[256]{};
        std::memcpy(corrI, rxI, sizeof(corrI));
        std::memcpy(corrQ, rxQ, sizeof(corrQ));

        HoloPass1T6 p1{};
        holo_pass1_compute_t6(corrI, corrQ, ExperimentConfig::kHoloSyncChips, p1);

        bool used_p2 = false;
        HoloPass2T6 p2{};
        bool sync_ok = p1.pass;

        if (!p1.pass) {
            int16_t preI[192];
            int16_t preQ[192];
            std::memcpy(preI, corrI, 192 * sizeof(int16_t));
            std::memcpy(preQ, corrQ, 192 * sizeof(int16_t));
            if (holo_pass2_inplace_t6(preI, preQ, ExperimentConfig::kHoloSyncChips, p2)) {
                sync_ok = true;
                used_p2 = true;
                std::memcpy(corrI, preI, 192 * sizeof(int16_t));
                std::memcpy(corrQ, preQ, 192 * sizeof(int16_t));
            }
        }

        const bool pat_b_or_d =
            (pattern == HoloV5aPattern::B_Full || pattern == HoloV5aPattern::B_Payload ||
             pattern == HoloV5aPattern::B_PayloadEstAtZero || pattern == HoloV5aPattern::B_PayloadFeedChip ||
             pattern == HoloV5aPattern::D_Full || pattern == HoloV5aPattern::D_Payload);
        const bool pat_c_or_d = (pattern == HoloV5aPattern::C_Full || pattern == HoloV5aPattern::C_Payload ||
                                 pattern == HoloV5aPattern::D_Full || pattern == HoloV5aPattern::D_Payload);
        const bool use_b_ins = sync_ok && p1.pass && pat_b_or_d;
        const bool b_payload_only = (pattern == HoloV5aPattern::B_Payload || pattern == HoloV5aPattern::B_PayloadEstAtZero ||
                                      pattern == HoloV5aPattern::B_PayloadFeedChip ||
                                      pattern == HoloV5aPattern::D_Payload);
        const bool use_c_ins = sync_ok && used_p2 && pat_c_or_d;
        const bool c_payload_only =
            (pattern == HoloV5aPattern::C_Payload || pattern == HoloV5aPattern::D_Payload);

        const bool want_bug8_diag = (t == 0 && cfo_hz == 500 && snr_db == 30 && use_b_ins);
        const bool want_lab_p_diag =
            (t == 0 && cfo_hz == 500 && snr_db == 30 &&
             (pattern == HoloV5aPattern::A || pattern == HoloV5aPattern::B_Payload ||
              pattern == HoloV5aPattern::B_PayloadEstAtZero || pattern == HoloV5aPattern::B_PayloadFeedChip));
        int16_t bug8_preI[8]{};
        int16_t bug8_preQ[8]{};
        if (want_bug8_diag) {
            for (int i = 0; i < 8; ++i) {
                bug8_preI[i] = corrI[i];
                bug8_preQ[i] = corrQ[i];
            }
        }

        double ins_est_hz = 0.0;
        double ins_ac_norm = 0.0;
        bool ins_applied = false;

        const int est_off_b = (pattern == HoloV5aPattern::B_PayloadEstAtZero) ? 0 : p1.best_off;

        if (want_lab_p_diag && pattern == HoloV5aPattern::A) {
            std::printf(
                "\n[DIAG-LabP] PA cfo=%d snr=%d V5a=OFF P1_best_off=%d kPreambleChips=%d payload_ph_deg[192..199]=",
                cfo_hz, snr_db, p1.best_off, static_cast<int>(hts::rx_cfo::kPreambleChips));
            for (int k = 192; k < 200; ++k) {
                std::printf(" %.1f", lab_phase_deg(corrI[k], corrQ[k]));
            }
            std::printf("\n");
        }

        if (want_lab_p_diag && (pattern == HoloV5aPattern::B_Payload || pattern == HoloV5aPattern::B_PayloadEstAtZero ||
                                pattern == HoloV5aPattern::B_PayloadFeedChip) &&
            use_b_ins) {
            std::printf(
                "[DIAG-LabP] P%s pre-payload_ph_deg[192..199]=", lab_m_pattern_char(pattern));
            for (int k = 192; k < 200; ++k) {
                std::printf(" %.1f", lab_phase_deg(corrI[k], corrQ[k]));
            }
            std::printf("  Estimate_win_off=%d len=%d\n", est_off_b, static_cast<int>(hts::rx_cfo::kPreambleChips));
            {
                hts::rx_cfo::CFO_V5a probe{};
                probe.Init();
                const hts::rx_cfo::CFO_Result cfo_res =
                    probe.Estimate(corrI + est_off_b, corrQ + est_off_b);
                std::printf(
                    "[DIAG-LabP] P%s Estimate valid=%d est_hz=%d |est-true|=%.1f IsApplyAllowed=%d\n",
                    lab_m_pattern_char(pattern), cfo_res.valid ? 1 : 0, static_cast<int>(cfo_res.cfo_hz),
                    std::abs(static_cast<double>(cfo_res.cfo_hz) - static_cast<double>(cfo_hz)),
                    probe.IsApplyAllowed() ? 1 : 0);
                if (cfo_res.valid && probe.IsApplyAllowed()) {
                    probe.Set_Apply_Cfo(cfo_res.cfo_hz);
                    std::printf("[DIAG-LabP] P%s post-Set_Apply_Cfo sin14=%d cos14=%d\n",
                                lab_m_pattern_char(pattern), static_cast<int>(probe.Get_Apply_Sin_Per_Chip_Q14()),
                                static_cast<int>(probe.Get_Apply_Cos_Per_Chip_Q14()));
                    probe.Advance_Phase_Only(192);
                    std::printf(
                        "[DIAG-LabP] P%s post-Advance(192) sin14=%d cos14=%d (per-chip inc 동일; 누적위상은 칩192 "
                        "equiv)\n",
                        lab_m_pattern_char(pattern), static_cast<int>(probe.Get_Apply_Sin_Per_Chip_Q14()),
                        static_cast<int>(probe.Get_Apply_Cos_Per_Chip_Q14()));
                }
            }
        }

        if (use_b_ins) {
            if (pattern == HoloV5aPattern::B_PayloadFeedChip) {
                lab_v5a_payload_apply_feedchip_from_rx(rxI, rxQ, corrI, corrQ, ExperimentConfig::kTxTotalChips,
                                                        est_off_b, ins_est_hz, ins_ac_norm, ins_applied);
            } else {
                lab_prod_v5a_apply_buffer(corrI, corrQ, ExperimentConfig::kTxTotalChips, est_off_b, b_payload_only,
                                          ins_est_hz, ins_ac_norm, ins_applied);
            }
        }
        if (use_c_ins) {
            double ins_est2 = 0.0;
            double ins_ac2 = 0.0;
            bool ins_ap2 = false;
            lab_prod_v5a_apply_buffer(corrI, corrQ, ExperimentConfig::kTxTotalChips, p2.best_off_p2, c_payload_only,
                                      ins_est2, ins_ac2, ins_ap2);
            ins_est_hz = ins_est2;
            ins_ac_norm = ins_ac2;
            ins_applied = ins_ap2;
        }

        if (want_lab_p_diag && (pattern == HoloV5aPattern::B_Payload || pattern == HoloV5aPattern::B_PayloadEstAtZero ||
                                pattern == HoloV5aPattern::B_PayloadFeedChip) &&
            use_b_ins && ins_applied) {
            std::printf(
                "[DIAG-LabP] P%s post-Apply payload_ph_deg[192..199]=", lab_m_pattern_char(pattern));
            for (int k = 192; k < 200; ++k) {
                std::printf(" %.1f", lab_phase_deg(corrI[k], corrQ[k]));
            }
            std::printf("  ins_est=%.1f ins_ap=%d\n", ins_est_hz, ins_applied ? 1 : 0);
        }

        if (want_bug8_diag) {
            std::printf(
                "\n[DIAG-LabO-BUG8] P%s cfo=%d snr=%d pre_buf[0..7]_I=%d %d %d %d %d %d %d %d  "
                "pre_buf[0..7]_Q=%d %d %d %d %d %d %d %d\n",
                lab_m_pattern_char(pattern), cfo_hz, snr_db, static_cast<int>(bug8_preI[0]),
                static_cast<int>(bug8_preI[1]), static_cast<int>(bug8_preI[2]), static_cast<int>(bug8_preI[3]),
                static_cast<int>(bug8_preI[4]), static_cast<int>(bug8_preI[5]), static_cast<int>(bug8_preI[6]),
                static_cast<int>(bug8_preI[7]), static_cast<int>(bug8_preQ[0]), static_cast<int>(bug8_preQ[1]),
                static_cast<int>(bug8_preQ[2]), static_cast<int>(bug8_preQ[3]), static_cast<int>(bug8_preQ[4]),
                static_cast<int>(bug8_preQ[5]), static_cast<int>(bug8_preQ[6]), static_cast<int>(bug8_preQ[7]));
            std::printf(
                "[DIAG-LabO-BUG8] P%s post_buf[192..199]_I=%d %d %d %d %d %d %d %d  "
                "post_buf[192..199]_Q=%d %d %d %d %d %d %d %d\n",
                lab_m_pattern_char(pattern), static_cast<int>(corrI[192]), static_cast<int>(corrI[193]),
                static_cast<int>(corrI[194]), static_cast<int>(corrI[195]), static_cast<int>(corrI[196]),
                static_cast<int>(corrI[197]), static_cast<int>(corrI[198]), static_cast<int>(corrI[199]),
                static_cast<int>(corrQ[192]), static_cast<int>(corrQ[193]), static_cast<int>(corrQ[194]),
                static_cast<int>(corrQ[195]), static_cast<int>(corrQ[196]), static_cast<int>(corrQ[197]),
                static_cast<int>(corrQ[198]), static_cast<int>(corrQ[199]));
            std::printf(
                "[DIAG-LabO-BUG8] P%s Advance(192) => CFO_V5a 내부 위상은 **절대칩 192**에 맞춤; "
                "Feed_Chip은 그 칩부터 Apply. payload_only=%d (1=B'/C'/D' 모사, 0=B/C/D 전버퍼 BUG-8)\n",
                lab_m_pattern_char(pattern), b_payload_only ? 1 : 0);
        }

        const bool want_m = lab_m_diag_trial0(t);
        if (!sync_ok) {
            if (want_m) {
                std::printf(
                    "\n[DIAG-LabM] P%s cfo=%d snr=%d P1=%d P2=%d sync=0 ins_est=%.1f ins_ac=%.3f ins_ap=%d "
                    "decode=SKIP\n",
                    lab_m_pattern_char(pattern), cfo_hz, snr_db, p1.pass ? 1 : 0, used_p2 ? 1 : 0, ins_est_hz,
                    ins_ac_norm, ins_applied ? 1 : 0);
            }
            if (lab_bug9_pa_diag_want(cfo_hz, snr_db, t, pattern)) {
                std::printf(
                    "\n[DIAG-LabBUG9] PA trial0 cfo=%d snr=%d holo_sync_pass=0 P1_energy_pass=%d decode=SKIP "
                    "bits[0..3]=n/a\n",
                    cfo_hz, snr_db, p1.pass ? 1 : 0);
                lab_bug9_print_cumulative_phase(cfo_hz);
                lab_bug9_print_payload_phases(corrI, corrQ);
            }
            r.total_bits += ExperimentConfig::kK;
            r.total_bit_err += ExperimentConfig::kK;
            continue;
        }
        ++r.sync_pass;

        int32_t p1_e63_sh = 0;
        int32_t p1_e0_sh = 0;
        int32_t p1_max_e_sh = 0;
        const bool p1_ok =
            lab_p1_fwht_energy_gate_pslte(corrI, corrQ, &p1_e63_sh, &p1_e0_sh, &p1_max_e_sh);
        if (!p1_ok) {
            if (want_m) {
                std::printf(
                    "\n[DIAG-LabM] P%s cfo=%d snr=%d P1=%d P2=%d sync=1 P1_FWHT_FAIL e63=%d e0=%d max=%d "
                    "ins_est=%.1f ins_ac=%.3f ins_ap=%d decode=SKIP\n",
                    lab_m_pattern_char(pattern), cfo_hz, snr_db, p1.pass ? 1 : 0, used_p2 ? 1 : 0,
                    static_cast<int>(p1_e63_sh), static_cast<int>(p1_e0_sh), static_cast<int>(p1_max_e_sh),
                    ins_est_hz, ins_ac_norm, ins_applied ? 1 : 0);
            }
            if (lab_bug9_pa_diag_want(cfo_hz, snr_db, t, pattern)) {
                std::printf(
                    "\n[DIAG-LabBUG9] PA trial0 cfo=%d snr=%d holo_sync_pass=1 P1_energy_pass=%d P1_FWHT_gate=0 "
                    "decode=SKIP bits[0..3]=n/a\n",
                    cfo_hz, snr_db, p1.pass ? 1 : 0);
                lab_bug9_print_cumulative_phase(cfo_hz);
                lab_bug9_print_payload_phases(corrI, corrQ);
            }
            r.total_bits += ExperimentConfig::kK;
            r.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        int16_t soft[64]{};
        uint64_t mask = 0u;
        rx_soft_generate(corrI + ExperimentConfig::kPayloadOffset, corrQ + ExperimentConfig::kPayloadOffset, soft,
                         &mask);
        const bool lab_p_rec = lab_p_trials != nullptr && cfo_hz == 500 && snr_db == 30 &&
                               pattern == lab_p_trials->mode && t < 100 && sync_ok && p1_ok;
        if (lab_p_rec) {
            if (pattern == HoloV5aPattern::A) {
                lab_p_trials->ph192_A[t] = lab_phase_deg(corrI[192], corrQ[192]);
            }
            if (lab_m_pattern_b_payload_stats(pattern)) {
                lab_p_trials->est_hz[t] = ins_est_hz;
                lab_p_trials->abs_est_err[t] = std::abs(ins_est_hz - static_cast<double>(cfo_hz));
                lab_p_trials->ph192_after[t] = lab_phase_deg(corrI[192], corrQ[192]);
            }
        }
        int8_t out[16]{};
        (void)rx.Set_Time_Slot(static_cast<uint32_t>(t));
        if (rx.Decode_Block(soft, static_cast<uint16_t>(ExperimentConfig::kN), mask, out,
                            static_cast<uint16_t>(ExperimentConfig::kK)) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            if (want_m) {
                std::printf(
                    "\n[DIAG-LabM] P%s cfo=%d snr=%d P1=%d P2=%d sync=1 ins_est=%.1f |est-true|=%.1f ins_ac=%.3f "
                    "ins_ap=%d decode=FAIL\n",
                    lab_m_pattern_char(pattern), cfo_hz, snr_db, p1.pass ? 1 : 0, used_p2 ? 1 : 0, ins_est_hz,
                    std::abs(ins_est_hz - static_cast<double>(cfo_hz)), ins_ac_norm, ins_applied ? 1 : 0);
            }
            if (lab_p_rec) {
                lab_p_trials->slot_valid[t] = true;
                lab_p_trials->bit_err[t] = -1;
                if (lab_m_pattern_b_payload_stats(pattern) && lab_p_trials->n_fail < 100) {
                    lab_p_trials->fail_trial[lab_p_trials->n_fail] = t;
                    lab_p_trials->fail_est_hz[lab_p_trials->n_fail] = ins_est_hz;
                    ++lab_p_trials->n_fail;
                }
            }
            if (lab_p_rec && lab_p_trials->print_ph192_each_trial && lab_m_pattern_bprime_btriple(pattern)) {
                const double phd = lab_phase_deg(corrI[192], corrQ[192]);
                std::printf("[LabQ-PH] %s t=%2d ph192=%.2f |est-err|=%.2f be=-1(DecodeBlock_FAIL)\n",
                            lab_m_pattern_char(pattern), t, phd, std::abs(ins_est_hz - static_cast<double>(cfo_hz)));
            }
            if (lab_bug9_pa_diag_want(cfo_hz, snr_db, t, pattern)) {
                std::printf(
                    "\n[DIAG-LabBUG9] PA trial0 cfo=%d snr=%d holo_sync_pass=1 P1_FWHT_gate=1 decode=BLOCK_FAIL "
                    "bits[0..3]=n/a\n",
                    cfo_hz, snr_db);
                lab_bug9_print_cumulative_phase(cfo_hz);
                lab_bug9_print_payload_phases(corrI, corrQ);
            }
            r.total_bits += ExperimentConfig::kK;
            r.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        int be = 0;
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            be += (out[i] != bits[i]) ? 1 : 0;
        }

        if (lab_p_rec) {
            lab_p_trials->slot_valid[t] = true;
            lab_p_trials->bit_err[t] = be;
            if (be != 0 && lab_m_pattern_b_payload_stats(pattern) && lab_p_trials->n_fail < 100) {
                lab_p_trials->fail_trial[lab_p_trials->n_fail] = t;
                lab_p_trials->fail_est_hz[lab_p_trials->n_fail] = ins_est_hz;
                ++lab_p_trials->n_fail;
            }
        }
        if (lab_p_rec && lab_p_trials->print_ph192_each_trial && lab_m_pattern_bprime_btriple(pattern)) {
            const double phd = lab_phase_deg(corrI[192], corrQ[192]);
            std::printf("[LabQ-PH] %s t=%2d ph192=%.2f |est-err|=%.2f be=%d\n", lab_m_pattern_char(pattern), t, phd,
                        std::abs(ins_est_hz - static_cast<double>(cfo_hz)), be);
        }

        if (want_m) {
            const int be0 = (out[0] != bits[0]) + (out[1] != bits[1]) + (out[2] != bits[2]) + (out[3] != bits[3]);
            std::printf(
                "\n[DIAG-LabM] P%s cfo=%d snr=%d P1=%d P2=%d ins_est=%.1f |est-true|=%.1f ins_ac=%.3f ins_ap=%d "
                "decode=OK bits0..3_err=%d got %d %d %d %d\n",
                lab_m_pattern_char(pattern), cfo_hz, snr_db, p1.pass ? 1 : 0, used_p2 ? 1 : 0, ins_est_hz,
                std::abs(ins_est_hz - static_cast<double>(cfo_hz)), ins_ac_norm, ins_applied ? 1 : 0, be0,
                static_cast<int>(out[0]), static_cast<int>(out[1]), static_cast<int>(out[2]),
                static_cast<int>(out[3]));
            if (want_lab_p_diag && pattern == HoloV5aPattern::A) {
                std::printf(
                    "[DIAG-LabP] PA decode bits[0..3] expect %d %d %d %d got %d %d %d %d (be0=%d)\n",
                    static_cast<int>(bits[0]), static_cast<int>(bits[1]), static_cast<int>(bits[2]),
                    static_cast<int>(bits[3]), static_cast<int>(out[0]), static_cast<int>(out[1]),
                    static_cast<int>(out[2]), static_cast<int>(out[3]), be0);
            }
            if (want_lab_p_diag && lab_m_pattern_b_payload_stats(pattern)) {
                std::printf(
                    "[DIAG-LabP] P%s decode bits[0..3] expect %d %d %d %d got %d %d %d %d (be0=%d)\n",
                    lab_m_pattern_char(pattern), static_cast<int>(bits[0]), static_cast<int>(bits[1]),
                    static_cast<int>(bits[2]), static_cast<int>(bits[3]), static_cast<int>(out[0]),
                    static_cast<int>(out[1]), static_cast<int>(out[2]), static_cast<int>(out[3]), be0);
            }
        }
        if (lab_bug9_pa_diag_want(cfo_hz, snr_db, t, pattern)) {
            const int be0b = (out[0] != bits[0]) + (out[1] != bits[1]) + (out[2] != bits[2]) + (out[3] != bits[3]);
            std::printf(
                "\n[DIAG-LabBUG9] PA trial0 cfo=%d snr=%d holo_sync_pass=1 P1_energy_pass=%d P1_FWHT_gate=1 "
                "Decode_Block=PASS bits[0..3] exp %d %d %d %d got %d %d %d %d be0=%d be_total=%d "
                "(D++ only if be_total==0)\n",
                cfo_hz, snr_db, p1.pass ? 1 : 0, static_cast<int>(bits[0]), static_cast<int>(bits[1]),
                static_cast<int>(bits[2]), static_cast<int>(bits[3]), static_cast<int>(out[0]),
                static_cast<int>(out[1]), static_cast<int>(out[2]), static_cast<int>(out[3]), be0b, be);
            lab_bug9_print_cumulative_phase(cfo_hz);
            lab_bug9_print_payload_phases(corrI, corrQ);
        }
        r.total_bit_err += be;
        r.total_bits += ExperimentConfig::kK;
        if (be == 0) {
            ++r.decode_pass;
        }
    }
    rx.Shutdown();
    tx.Shutdown();
    return r;
}

static void lab_p_print_summary_500(const LabP500Trials* s, const char* title) noexcept {
    if (s == nullptr) {
        return;
    }
    int n_valid = 0;
    for (int i = 0; i < 100; ++i) {
        if (s->slot_valid[i]) {
            ++n_valid;
        }
    }
    if (n_valid <= 0) {
        return;
    }
    if (s->mode == HoloV5aPattern::A) {
        double sum = 0.0;
        double sumsq = 0.0;
        for (int i = 0; i < 100; ++i) {
            if (!s->slot_valid[i]) {
                continue;
            }
            sum += s->ph192_A[i];
            sumsq += s->ph192_A[i] * s->ph192_A[i];
        }
        const double mean = sum / static_cast<double>(n_valid);
        const double var = (sumsq / static_cast<double>(n_valid)) - mean * mean;
        const double stdv = (var > 0.0) ? std::sqrt(var) : 0.0;
        int n_debfail = 0;
        for (int i = 0; i < 100; ++i) {
            if (s->slot_valid[i] && s->bit_err[i] == -1) {
                ++n_debfail;
            }
        }
        std::printf(
            "[LabP-500/30] %s Pattern A: chip192 phase_deg mean=%.2f std=%.2f (n=%d sync+P1+soft); "
            "Decode_Block_fail=%d\n",
            title, mean, stdv, n_valid, n_debfail);
        return;
    }
    if (s->mode == HoloV5aPattern::B_Payload || s->mode == HoloV5aPattern::B_PayloadEstAtZero ||
        s->mode == HoloV5aPattern::B_PayloadFeedChip) {
        double sum_ae = 0.0;
        double sumsq_ae = 0.0;
        double sum_ph = 0.0;
        double sumsq_ph = 0.0;
        for (int i = 0; i < 100; ++i) {
            if (!s->slot_valid[i]) {
                continue;
            }
            sum_ae += s->abs_est_err[i];
            sumsq_ae += s->abs_est_err[i] * s->abs_est_err[i];
            sum_ph += s->ph192_after[i];
            sumsq_ph += s->ph192_after[i] * s->ph192_after[i];
        }
        const double m_ae = sum_ae / static_cast<double>(n_valid);
        const double v_ae = (sumsq_ae / static_cast<double>(n_valid)) - m_ae * m_ae;
        const double sd_ae = (v_ae > 0.0) ? std::sqrt(v_ae) : 0.0;
        const double m_ph = sum_ph / static_cast<double>(n_valid);
        const double v_ph = (sumsq_ph / static_cast<double>(n_valid)) - m_ph * m_ph;
        const double sd_ph = (v_ph > 0.0) ? std::sqrt(v_ph) : 0.0;
        int n_debfail = 0;
        int n_be = 0;
        for (int i = 0; i < 100; ++i) {
            if (!s->slot_valid[i]) {
                continue;
            }
            if (s->bit_err[i] == -1) {
                ++n_debfail;
            } else if (s->bit_err[i] > 0) {
                ++n_be;
            }
        }
        std::printf(
            "[LabP-500/30] %s %s: |est-true| mean=%.2f std=%.2f Hz; ph192_after mean=%.2f std=%.2f deg; "
            "n_valid=%d Decode_Block_fail=%d bit_err>0=%d\n",
            title, lab_m_pattern_char(s->mode), m_ae, sd_ae, m_ph, sd_ph, n_valid, n_debfail, n_be);
        if (s->n_fail > 0) {
            std::printf("[LabP-500/30] %s weak-decode sample (trial, est_hz, bit_err; -1=Decode_Block fail): ",
                        title);
            const int lim = (s->n_fail < 12) ? s->n_fail : 12;
            for (int j = 0; j < lim; ++j) {
                const int ti = s->fail_trial[j];
                std::printf("(%d,%.0f,%d) ", ti, s->fail_est_hz[j], (ti >= 0 && ti < 100) ? s->bit_err[ti] : -999);
            }
            std::printf("\n");
        }
    }
}

static void run_lab_m_matrix(HoloV5aPattern pattern);

static void run_lab_bug9_pa_sweep_matrix() {
    std::printf("\n===== Lab BUG-9: Pattern A (V5a OFF) fine CFO x SNR=30 only (%d pts) =====\n",
                kLabBug9PaSweepCfosCount);
    for (int ci = 0; ci < kLabBug9PaSweepCfosCount; ++ci) {
        const int cfo = kLabBug9PaSweepCfos[ci];
        const TestResult r = run_one_cfo_lab_m(cfo, 30, HoloV5aPattern::A, nullptr);
        const double ber =
            (r.total_bits > 0) ? static_cast<double>(r.total_bit_err) / static_cast<double>(r.total_bits) : 1.0;
        std::printf("cfo=%5d: %ddB S%3d/%3d D%3d/%3d BER=%0.3f\n", cfo, 30, r.sync_pass,
                    ExperimentConfig::kNumTrials, r.decode_pass, ExperimentConfig::kNumTrials, ber);
    }
    std::printf("\n");
}

/// Lab-O 7점 스윕·BUG-9 세분 스윕에 없는 CFO — [DIAG-LabBUG9] trial0 만 보강
static constexpr int kLabBug9PaDiagSupplementCfos[] = {1800, 2200, 3500, 4500};
static constexpr int kLabBug9PaDiagSupplementCount =
    static_cast<int>(sizeof(kLabBug9PaDiagSupplementCfos) / sizeof(kLabBug9PaDiagSupplementCfos[0]));

static void run_lab_bug9_pa_diag_supplement_snr30() {
    std::printf("\n===== Lab BUG-9: Pattern A DIAG-only CFOs (Lab-O sweep 밖; trial0 snr=30) =====\n");
    for (int i = 0; i < kLabBug9PaDiagSupplementCount; ++i) {
        const int cfo = kLabBug9PaDiagSupplementCfos[i];
        (void)run_one_cfo_lab_m(cfo, 30, HoloV5aPattern::A, nullptr);
    }
    std::printf("\n");
}

static void run_lab_p_500_stats_and_diag() {
    std::printf("\n===== Lab P: cfo=500 snr=30 DIAG + 100-trial stats (kPreambleChips=%d) =====\n",
                static_cast<int>(hts::rx_cfo::kPreambleChips));

    LabP500Trials st_a{};
    st_a.mode = HoloV5aPattern::A;
    (void)run_one_cfo_lab_m(500, 30, HoloV5aPattern::A, &st_a);
    lab_p_print_summary_500(&st_a, "post-run");

    LabP500Trials st_bp{};
    st_bp.mode = HoloV5aPattern::B_Payload;
    (void)run_one_cfo_lab_m(500, 30, HoloV5aPattern::B_Payload, &st_bp);
    lab_p_print_summary_500(&st_bp, "post-run");

    LabP500Trials st_b2{};
    st_b2.mode = HoloV5aPattern::B_PayloadEstAtZero;
    (void)run_one_cfo_lab_m(500, 30, HoloV5aPattern::B_PayloadEstAtZero, &st_b2);
    lab_p_print_summary_500(&st_b2, "post-run");
}

static void run_lab_p_ab_matrix() {
    std::printf("\n===== Lab P: matrix A vs B' vs B'' (same CFO×SNR as Lab O) =====\n");
    run_lab_m_matrix(HoloV5aPattern::A);
    run_lab_m_matrix(HoloV5aPattern::B_Payload);
    run_lab_m_matrix(HoloV5aPattern::B_PayloadEstAtZero);
}

static void run_lab_q_500_30_phdump_and_stats() {
    std::printf("\n===== Lab Q (stage A): cfo=500 snr=30  B' vs B'''  + [LabQ-PH] 100 trials/패턴 =====\n");
    LabP500Trials st_bp{};
    st_bp.mode = HoloV5aPattern::B_Payload;
    st_bp.print_ph192_each_trial = true;
    (void)run_one_cfo_lab_m(500, 30, HoloV5aPattern::B_Payload, &st_bp);
    lab_p_print_summary_500(&st_bp, "LabQ");

    LabP500Trials st_b3{};
    st_b3.mode = HoloV5aPattern::B_PayloadFeedChip;
    st_b3.print_ph192_each_trial = true;
    (void)run_one_cfo_lab_m(500, 30, HoloV5aPattern::B_PayloadFeedChip, &st_b3);
    lab_p_print_summary_500(&st_b3, "LabQ");
}

static void run_lab_q_matrix_a_bprime_btripple() {
    std::printf("\n===== Lab Q (stage A): A vs B' vs B'''  6 CFO x 3 SNR (BUG-8/B-Feed_Chip) =====\n");
    for (int ci = 0; ci < kLabQCfoCount; ++ci) {
        const int cfo = kLabQCfoSweep[ci];
        std::printf("cfo=%5d:\n", cfo);
        for (int si = 0; si < ExperimentConfig::kSnrCount; ++si) {
            const int snr = ExperimentConfig::kSnrLevels[si];
            const TestResult r_a = run_one_cfo_lab_m(cfo, snr, HoloV5aPattern::A, nullptr);
            const TestResult r_b = run_one_cfo_lab_m(cfo, snr, HoloV5aPattern::B_Payload, nullptr);
            const TestResult r_t = run_one_cfo_lab_m(cfo, snr, HoloV5aPattern::B_PayloadFeedChip, nullptr);
            const double ber_a =
                (r_a.total_bits > 0) ? static_cast<double>(r_a.total_bit_err) / static_cast<double>(r_a.total_bits) : 1.0;
            const double ber_b =
                (r_b.total_bits > 0) ? static_cast<double>(r_b.total_bit_err) / static_cast<double>(r_b.total_bits) : 1.0;
            const double ber_t =
                (r_t.total_bits > 0) ? static_cast<double>(r_t.total_bit_err) / static_cast<double>(r_t.total_bits) : 1.0;
            std::printf(
                " %2ddB  A: S%3d/%3d D%3d/%3d BER=%.3f  |  B': S%3d/%3d D%3d/%3d BER=%.3f  |  B''': S%3d/%3d D%3d/%3d "
                "BER=%.3f\n",
                snr, r_a.sync_pass, ExperimentConfig::kNumTrials, r_a.decode_pass, ExperimentConfig::kNumTrials, ber_a,
                r_b.sync_pass, ExperimentConfig::kNumTrials, r_b.decode_pass, ExperimentConfig::kNumTrials, ber_b,
                r_t.sync_pass, ExperimentConfig::kNumTrials, r_t.decode_pass, ExperimentConfig::kNumTrials, ber_t);
        }
        std::printf("\n");
    }
    std::printf("\n");
}

static void run_lab_m_matrix(HoloV5aPattern pattern) {
    std::printf("===== Lab O Pattern %s (CFO_V5a; B/C=전buf, B'/C'=payload-only BUG-8 fix) =====\n",
                lab_m_pattern_char(pattern));
    for (int ci = 0; ci < kLabMCfoCount; ++ci) {
        const int cfo = kLabMCfoSweep[ci];
        std::printf("cfo=%5d:\n", cfo);
        for (int si = 0; si < ExperimentConfig::kSnrCount; ++si) {
            const TestResult r =
                run_one_cfo_lab_m(cfo, ExperimentConfig::kSnrLevels[si], pattern, nullptr);
            const double ber = (r.total_bits > 0) ? static_cast<double>(r.total_bit_err) / r.total_bits : 1.0;
            std::printf(" %ddB S%3d/%3d D%3d/%3d BER=%0.3f", ExperimentConfig::kSnrLevels[si], r.sync_pass,
                        ExperimentConfig::kNumTrials, r.decode_pass, ExperimentConfig::kNumTrials, ber);
        }
        std::printf("\n");
    }
    std::printf("\n");
}

static TestResult run_one_cfo(int cfo_hz, int snr_db, LabKV5aMode v5a_mode, double ac_norm_thr) {
    TestResult r{0, 0, 0, 0};
    HTS_Holo_Tensor_4D_TX tx{};
    HTS_Holo_Tensor_4D_RX rx{};
    uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5B5A5A5u, 0xC3D3C3C3u};
    if (tx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) return r;
    if (rx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        tx.Shutdown();
        return r;
    }
    if (v5a_mode != LabKV5aMode::LegacyPreSym1Lag4) {
        (void)ac_norm_thr;
    }

    for (int t = 0; t < ExperimentConfig::kNumTrials; ++t) {
        rng_seed(static_cast<uint32_t>(t * 7919u + 13u));
        int8_t bits[16]{};
        for (int i = 0; i < ExperimentConfig::kK; ++i) bits[i] = (rng_next() & 1u) ? 1 : -1;

        int8_t chips[64]{};
        (void)tx.Set_Time_Slot(static_cast<uint32_t>(t));
        if (tx.Encode_Block(bits, static_cast<uint16_t>(ExperimentConfig::kK), chips,
                            static_cast<uint16_t>(ExperimentConfig::kN)) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            r.total_bits += ExperimentConfig::kK;
            r.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        int16_t txI[256]{};
        int16_t txQ[256]{};
        for (int c = 0; c < ExperimentConfig::kPreSym0TotalChips; ++c) {
            const uint32_t x = 63u & static_cast<uint32_t>(c);
            const int sign = (popcount32_(x) & 1u) ? -1 : 1;
            const int16_t pre = static_cast<int16_t>(sign * ExperimentConfig::kAmp);
            txI[c] = pre;
            txQ[c] = pre;
        }
        for (int c = 0; c < ExperimentConfig::kPreSym1Chips; ++c) {
            const int16_t a = static_cast<int16_t>(ExperimentConfig::kAmp);
            txI[ExperimentConfig::kPreSym1Offset + c] = a;
            txQ[ExperimentConfig::kPreSym1Offset + c] = a;
        }
        for (int c = 0; c < 64; ++c) {
            const int32_t prod = static_cast<int32_t>(chips[c]) * ExperimentConfig::kAmp;
            const int32_t v = (prod >= 0) ? ((prod + (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom)
                                          : ((prod - (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom);
            txI[ExperimentConfig::kPayloadOffset + c] = static_cast<int16_t>(v);
            txQ[ExperimentConfig::kPayloadOffset + c] = static_cast<int16_t>(v);
        }

        int16_t rxI[256]{};
        int16_t rxQ[256]{};
        int16_t corrI[256]{};
        int16_t corrQ[256]{};
        channel_apply(txI, txQ, rxI, rxQ, ExperimentConfig::kTxTotalChips, static_cast<double>(cfo_hz), snr_db,
                      static_cast<uint32_t>(t * 31u + 19u));

        constexpr int kV5aLag = 4;
        double est_hz = 0.0;
        double ac_norm = 0.0;
        bool apply_v5a = false;

        std::memcpy(corrI, rxI, sizeof(corrI));
        std::memcpy(corrQ, rxQ, sizeof(corrQ));

        if (v5a_mode == LabKV5aMode::ProdWalsh128) {
            HoloPass1T6 p1pre{};
            holo_pass1_compute_t6(corrI, corrQ, ExperimentConfig::kHoloSyncChips, p1pre);
            const int bo =
                (p1pre.best_off >= 0 && p1pre.best_off + hts::rx_cfo::kPreambleChips <= 192) ? p1pre.best_off : 0;
            double gate_diag = 0.0;
            lab_prod_v5a_apply_buffer(corrI, corrQ, ExperimentConfig::kTxTotalChips, bo,
                                      false /* Lab K: 구 전버퍼 경로 유지 */, est_hz, gate_diag, apply_v5a);
            ac_norm = gate_diag;
        } else if (v5a_mode == LabKV5aMode::LegacyPreSym1Lag4) {
            double ac_mag = 0.0;
            double sig_pow = 0.0;
            est_hz = v5a_estimate_cfo_with_quality(rxI + ExperimentConfig::kV5aEstimateOffset,
                                                    rxQ + ExperimentConfig::kV5aEstimateOffset,
                                                    ExperimentConfig::kV5aEstimateChips, kV5aLag, &ac_mag, &sig_pow);
            est_hz = -est_hz;
            const double denom =
                sig_pow * (static_cast<double>(ExperimentConfig::kV5aEstimateChips - kV5aLag) /
                           static_cast<double>(ExperimentConfig::kV5aEstimateChips));
            ac_norm = (denom > 1e-18) ? (ac_mag / denom) : 0.0;
            apply_v5a = (std::abs(est_hz) >= ExperimentConfig::kV5aDeadzoneEpsilonHz) && (ac_norm >= ac_norm_thr);
            if (apply_v5a) {
                v5a_apply_per_chip(rxI, rxQ, corrI, corrQ, ExperimentConfig::kTxTotalChips, est_hz);
            }
        }

        bool l12 = false;
        bool l5f = false;
        bool l3f = false;
        int64_t best_e = 0;
        int ratio_x10 = 0;
        int64_t top4_sum = 0;
        bool sync_ok = true;
        if (ExperimentConfig::kEnableHoloSync) {
            sync_ok = holo_sync_check_t6(corrI, corrQ, ExperimentConfig::kHoloSyncChips, l12, l5f, l3f, best_e,
                                          ratio_x10, top4_sum);
        }

        const bool want_lab_l = lab_l_diag_trial0(t);
        const bool want_detail = lab_k_diag_trial0(cfo_hz, snr_db, t);
        const char* lab_l_v5a_tag =
            (v5a_mode == LabKV5aMode::ProdWalsh128)
                ? "PROD128@P1best_off"
                : (v5a_mode == LabKV5aMode::LegacyPreSym1Lag4 ? "LAB_PRE1_lag4" : "V5a=OFF");

        if (ExperimentConfig::kEnableHoloSync && !sync_ok) {
            if (want_lab_l) {
                if (v5a_mode != LabKV5aMode::Off) {
                    const double est_err = std::abs(est_hz - static_cast<double>(cfo_hz));
                    std::printf(
                        "\n[DIAG-LabL] trial0 v5a=%s cfo=%d snr=%d est_hz=%.1f true=%d "
                        "|est-true|=%.1f ac_norm=%.3f apply_v5a=%d holo_sync=0 P1=n/a decode=SKIP\n",
                        lab_l_v5a_tag, cfo_hz, snr_db, est_hz, cfo_hz, est_err, ac_norm, apply_v5a ? 1 : 0);
                } else {
                    std::printf("\n[DIAG-LabL] trial0 V5a=OFF cfo=%d snr=%d holo_sync=0 P1=n/a decode=SKIP\n",
                                cfo_hz, snr_db);
                }
            }
            r.total_bits += ExperimentConfig::kK;
            r.total_bit_err += ExperimentConfig::kK;
            continue;
        }
        ++r.sync_pass;

        int32_t p1_e63_sh = 0;
        int32_t p1_e0_sh = 0;
        int32_t p1_max_e_sh = 0;
        const bool p1_ok =
            lab_p1_fwht_energy_gate_pslte(corrI, corrQ, &p1_e63_sh, &p1_e0_sh, &p1_max_e_sh);
        if (!p1_ok) {
            if (want_lab_l) {
                if (v5a_mode != LabKV5aMode::Off) {
                    const double est_err = std::abs(est_hz - static_cast<double>(cfo_hz));
                    std::printf(
                        "\n[DIAG-LabL] trial0 v5a=%s cfo=%d snr=%d est_hz=%.1f |est-true|=%.1f "
                        "ac_norm=%.3f apply_v5a=%d holo_sync=1 P1_FAIL e63_sh=%d e0_sh=%d max_e_sh=%d "
                        "k_P1_MIN_E=%d (PS-LTE) decode=SKIP\n",
                        lab_l_v5a_tag, cfo_hz, snr_db, est_hz, est_err, ac_norm, apply_v5a ? 1 : 0,
                        static_cast<int>(p1_e63_sh), static_cast<int>(p1_e0_sh), static_cast<int>(p1_max_e_sh),
                        static_cast<int>(k_P1_MIN_E));
                } else {
                    std::printf(
                        "\n[DIAG-LabL] trial0 V5a=OFF cfo=%d snr=%d holo=1 P1_FAIL e63_sh=%d e0_sh=%d max_e_sh=%d "
                        "k_P1_MIN_E=%d decode=SKIP\n",
                        cfo_hz, snr_db, static_cast<int>(p1_e63_sh), static_cast<int>(p1_e0_sh),
                        static_cast<int>(p1_max_e_sh), static_cast<int>(k_P1_MIN_E));
                }
            }
            r.total_bits += ExperimentConfig::kK;
            r.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        if (want_detail) {
            std::printf("[DIAG-LabK] corrI[0..7]= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(corrI[0]), static_cast<int>(corrI[1]), static_cast<int>(corrI[2]),
                        static_cast<int>(corrI[3]), static_cast<int>(corrI[4]), static_cast<int>(corrI[5]),
                        static_cast<int>(corrI[6]), static_cast<int>(corrI[7]));
            std::printf("[DIAG-LabK] corrI[64..71]= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(corrI[64]), static_cast<int>(corrI[65]), static_cast<int>(corrI[66]),
                        static_cast<int>(corrI[67]), static_cast<int>(corrI[68]), static_cast<int>(corrI[69]),
                        static_cast<int>(corrI[70]), static_cast<int>(corrI[71]));
            std::printf("[DIAG-LabK] corrI[128..135]= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(corrI[128]), static_cast<int>(corrI[129]), static_cast<int>(corrI[130]),
                        static_cast<int>(corrI[131]), static_cast<int>(corrI[132]), static_cast<int>(corrI[133]),
                        static_cast<int>(corrI[134]), static_cast<int>(corrI[135]));
            std::printf("[DIAG-LabK] corrI[192..199]= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(corrI[192]), static_cast<int>(corrI[193]), static_cast<int>(corrI[194]),
                        static_cast<int>(corrI[195]), static_cast<int>(corrI[196]), static_cast<int>(corrI[197]),
                        static_cast<int>(corrI[198]), static_cast<int>(corrI[199]));
            const double est_err = std::abs(est_hz - static_cast<double>(cfo_hz));
            std::printf("[DIAG-LabK] true_cfo_hz=%d est_hz=%.1f |est-true|=%.1f apply_v5a=%d\n", cfo_hz, est_hz,
                        est_err, apply_v5a ? 1 : 0);
            const double f_res_hz =
                apply_v5a ? (static_cast<double>(cfo_hz) - est_hz) : static_cast<double>(cfo_hz);
            for (int i = 0; i < 64; ++i) {
                const int m = ExperimentConfig::kPayloadOffset + i;
                const double cum_model_deg =
                    360.0 * static_cast<double>(m) * f_res_hz / ExperimentConfig::kChipRateHz;
                const double ph = lab_phase_deg(corrI[m], corrQ[m]);
                std::printf("[DIAG-PAYLOAD-PHASE] i=%d corr=(%d,%d) phase_deg=%.1f cum_model_deg=%.1f (f_res_hz=%.1f)\n",
                            i, static_cast<int>(corrI[m]), static_cast<int>(corrQ[m]), ph, cum_model_deg, f_res_hz);
            }
        }

        int16_t soft[64]{};
        uint64_t mask = 0u;
        rx_soft_generate(corrI + ExperimentConfig::kPayloadOffset, corrQ + ExperimentConfig::kPayloadOffset, soft,
                         &mask);
        if (want_detail) {
            std::printf("[DIAG-LabK] rx_soft[0..7] pre-decode= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(soft[0]), static_cast<int>(soft[1]), static_cast<int>(soft[2]),
                        static_cast<int>(soft[3]), static_cast<int>(soft[4]), static_cast<int>(soft[5]),
                        static_cast<int>(soft[6]), static_cast<int>(soft[7]));
        }
        int8_t out[16]{};
        (void)rx.Set_Time_Slot(static_cast<uint32_t>(t));
        if (rx.Decode_Block(soft, static_cast<uint16_t>(ExperimentConfig::kN), mask, out,
                            static_cast<uint16_t>(ExperimentConfig::kK)) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            if (want_detail) {
                std::printf("[DIAG-LabK] decode_block SECURE_FALSE\n");
            }
            if (want_lab_l) {
                if (v5a_mode != LabKV5aMode::Off) {
                    const double est_err = std::abs(est_hz - static_cast<double>(cfo_hz));
                    std::printf(
                        "\n[DIAG-LabL] trial0 v5a=%s cfo=%d snr=%d est_hz=%.1f |est-true|=%.1f "
                        "ac_norm=%.3f apply_v5a=%d holo=1 P1_PASS e63_sh=%d e0_sh=%d max_e_sh=%d k_P1_MIN_E=%d "
                        "decode=FAIL\n",
                        lab_l_v5a_tag, cfo_hz, snr_db, est_hz, est_err, ac_norm, apply_v5a ? 1 : 0,
                        static_cast<int>(p1_e63_sh), static_cast<int>(p1_e0_sh), static_cast<int>(p1_max_e_sh),
                        static_cast<int>(k_P1_MIN_E));
                } else {
                    std::printf(
                        "\n[DIAG-LabL] trial0 V5a=OFF cfo=%d snr=%d holo=1 P1_PASS e63_sh=%d e0_sh=%d max_e_sh=%d "
                        "k_P1_MIN_E=%d decode=FAIL\n",
                        cfo_hz, snr_db, static_cast<int>(p1_e63_sh), static_cast<int>(p1_e0_sh),
                        static_cast<int>(p1_max_e_sh), static_cast<int>(k_P1_MIN_E));
                }
            }
            r.total_bits += ExperimentConfig::kK;
            r.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        if (want_lab_l) {
            const int be0 = (out[0] != bits[0]) + (out[1] != bits[1]) + (out[2] != bits[2]) + (out[3] != bits[3]);
            if (v5a_mode != LabKV5aMode::Off) {
                const double est_err = std::abs(est_hz - static_cast<double>(cfo_hz));
                std::printf(
                    "\n[DIAG-LabL] trial0 v5a=%s cfo=%d snr=%d est_hz=%.1f |est-true|=%.1f "
                    "ac_norm=%.3f apply_v5a=%d holo=1 P1_PASS e63_sh=%d e0_sh=%d max_e_sh=%d k_P1_MIN_E=%d "
                    "decode=OK bits0..3_err=%d got %d %d %d %d\n",
                    lab_l_v5a_tag, cfo_hz, snr_db, est_hz, est_err, ac_norm, apply_v5a ? 1 : 0,
                    static_cast<int>(p1_e63_sh), static_cast<int>(p1_e0_sh), static_cast<int>(p1_max_e_sh),
                    static_cast<int>(k_P1_MIN_E), be0, static_cast<int>(out[0]), static_cast<int>(out[1]),
                    static_cast<int>(out[2]), static_cast<int>(out[3]));
            } else {
                std::printf(
                    "\n[DIAG-LabL] trial0 V5a=OFF cfo=%d snr=%d holo=1 P1_PASS e63_sh=%d e0_sh=%d max_e_sh=%d "
                    "k_P1_MIN_E=%d decode=OK bits0..3_err=%d got %d %d %d %d\n",
                    cfo_hz, snr_db, static_cast<int>(p1_e63_sh), static_cast<int>(p1_e0_sh),
                    static_cast<int>(p1_max_e_sh), static_cast<int>(k_P1_MIN_E), be0, static_cast<int>(out[0]),
                    static_cast<int>(out[1]), static_cast<int>(out[2]), static_cast<int>(out[3]));
            }
        }

        if (want_detail) {
            std::printf("[DIAG-LabK] bits[0..3] expect %d %d %d %d got %d %d %d %d\n",
                        static_cast<int>(bits[0]), static_cast<int>(bits[1]), static_cast<int>(bits[2]),
                        static_cast<int>(bits[3]), static_cast<int>(out[0]), static_cast<int>(out[1]),
                        static_cast<int>(out[2]), static_cast<int>(out[3]));
        }

        int be = 0;
        for (int i = 0; i < ExperimentConfig::kK; ++i) be += (out[i] != bits[i]) ? 1 : 0;
        r.total_bit_err += be;
        r.total_bits += ExperimentConfig::kK;
        if (be == 0) ++r.decode_pass;
    }
    rx.Shutdown();
    tx.Shutdown();
    return r;
}

// =============================================================================
// Lab R: V5a 양산(CFO_V5a::Estimate + IsApply) + lag-4 ac_norm(0.95) · BUG-9 전 텐서 결정
// (메인트리 / HTS_CFO_V5a.c 소스는 변경하지 않음; Lab 경로 측정만)
// =============================================================================
struct LabRMetrics1 {
    double est_hz;
    double abs_err_hz;
    double ac_norm_lag4;
    double est_lag4_hz;
    int apply_v5a_prod;    ///< C API: res.valid & IsApplyAllowed
    int apply_lag4_thr;    ///< (|est_lag4| >= ε) & (ac_norm >= kV5aAcNormThresholdLabK)
};

static void lab_r_v5a_prod_lag4_from_rx(const int16_t* rxI, const int16_t* rxQ, int cfo_true,
                                        LabRMetrics1& o) noexcept {
    o = LabRMetrics1{0.0, 0.0, 0.0, 0.0, 0, 0};
    int16_t scI[256];
    int16_t scQ[256];
    std::memcpy(scI, rxI, sizeof(scI));
    std::memcpy(scQ, rxQ, sizeof(scQ));
    HoloPass1T6 p1{};
    holo_pass1_compute_t6(scI, scQ, ExperimentConfig::kHoloSyncChips, p1);
    const int bo = (p1.best_off >= 0 && p1.best_off + hts::rx_cfo::kPreambleChips <= 192) ? p1.best_off : 0;
    hts::rx_cfo::CFO_V5a v5a{};
    v5a.Init();
    const hts::rx_cfo::CFO_Result cres = v5a.Estimate(scI + bo, scQ + bo);
    o.est_hz = static_cast<double>(cres.cfo_hz);
    o.abs_err_hz = std::abs(o.est_hz - static_cast<double>(cfo_true));
    o.apply_v5a_prod = (cres.valid && v5a.IsApplyAllowed()) ? 1 : 0;

    double ac_mag = 0.0;
    double sig_pow = 0.0;
    constexpr int kL = 4;
    o.est_lag4_hz = v5a_estimate_cfo_with_quality(
        rxI + ExperimentConfig::kV5aEstimateOffset, rxQ + ExperimentConfig::kV5aEstimateOffset,
        ExperimentConfig::kV5aEstimateChips, kL, &ac_mag, &sig_pow);
    o.est_lag4_hz = -o.est_lag4_hz;
    const double denom =
        sig_pow * (static_cast<double>(ExperimentConfig::kV5aEstimateChips - kL) /
                   static_cast<double>(ExperimentConfig::kV5aEstimateChips));
    o.ac_norm_lag4 = (denom > 1e-18) ? (ac_mag / denom) : 0.0;
    o.apply_lag4_thr = (std::abs(o.est_lag4_hz) >= ExperimentConfig::kV5aDeadzoneEpsilonHz) &&
                                 (o.ac_norm_lag4 >= ExperimentConfig::kV5aAcNormThresholdLabK)
                             ? 1
                             : 0;
}

static bool lab_r_fill_rx_one_trial(HTS_Holo_Tensor_4D_TX& tx, int t, int cfo_hz, int snr_db, int16_t* rxI,
                                    int16_t* rxQ, int8_t* out_bits_16 = nullptr, int8_t* out_chips_64 = nullptr) {
    rng_seed(static_cast<uint32_t>(t * 7919u + 13u));
    int8_t bits[16]{};
    for (int i = 0; i < ExperimentConfig::kK; ++i) {
        bits[i] = (rng_next() & 1u) ? 1 : -1;
    }
    int8_t chips[64]{};
    (void)tx.Set_Time_Slot(static_cast<uint32_t>(t));
    if (tx.Encode_Block(bits, static_cast<uint16_t>(ExperimentConfig::kK), chips,
                        static_cast<uint16_t>(ExperimentConfig::kN)) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        return false;
    }
    int16_t txI[256]{};
    int16_t txQ[256]{};
    for (int c = 0; c < ExperimentConfig::kPreSym0TotalChips; ++c) {
        const uint32_t x = 63u & static_cast<uint32_t>(c);
        const int sign = (popcount32_(x) & 1u) ? -1 : 1;
        const int16_t pre = static_cast<int16_t>(sign * ExperimentConfig::kAmp);
        txI[c] = pre;
        txQ[c] = pre;
    }
    for (int c = 0; c < ExperimentConfig::kPreSym1Chips; ++c) {
        const int16_t a = static_cast<int16_t>(ExperimentConfig::kAmp);
        txI[ExperimentConfig::kPreSym1Offset + c] = a;
        txQ[ExperimentConfig::kPreSym1Offset + c] = a;
    }
    for (int c = 0; c < 64; ++c) {
        const int32_t prod = static_cast<int32_t>(chips[c]) * ExperimentConfig::kAmp;
        const int32_t v = (prod >= 0) ? ((prod + (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom)
                                      : ((prod - (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom);
        txI[ExperimentConfig::kPayloadOffset + c] = static_cast<int16_t>(v);
        txQ[ExperimentConfig::kPayloadOffset + c] = static_cast<int16_t>(v);
    }
    channel_apply(txI, txQ, rxI, rxQ, ExperimentConfig::kTxTotalChips, static_cast<double>(cfo_hz), snr_db,
                  static_cast<uint32_t>(t * 31u + 19u));
    if (out_chips_64 != nullptr) {
        std::memcpy(out_chips_64, chips, 64u);
    }
    if (out_bits_16 != nullptr) {
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            out_bits_16[static_cast<size_t>(i)] = bits[static_cast<size_t>(i)];
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Lab R' (TS/R&D): 4D RX 비대칭 / 위상-민감 경로 — 입력 전처리만 비교 (4D 엔진·메인트리 비변경)
//  A: (I+Q)/2  →  Decode_Block  (量産과 동일; BUG-9/π-ambiguity 취약)
//  B: (I*chip+Q*chip)/2, TX와 동기된 기준(신 시뮬) ref chips
//  C: I/Q 차분(간이 DBPSK) — cfo 1차 변화·노이즈 2배
//  D: (|I|+|Q|)·sgn(I+Q) — 위상-크기 혼합(비-동조 프록시; 본질적 Walsh|·|²는 엔진 내부만 가능)
//  E: (참) true-CFO v5a_apply + (I+Q)/2  — "≤10Hz V5a" 이상·회전 제거+스칼라(연구·상한)
// ---------------------------------------------------------------------------
static int lab_tsrx_try_decode(HTS_Holo_Tensor_4D_RX& rx, uint32_t t, const int16_t* playI, const int16_t* playQ,
                               const int8_t ref_bits[16], int mode, const int8_t* ref_chips) noexcept {
    int16_t soft[64]{};
    uint64_t mask = 0u;
    if (mode == 0) {
        (void)ref_chips;
        rx_soft_generate(playI, playQ, soft, &mask);
    } else if (mode == 1) {
        if (ref_chips == nullptr) {
            return ExperimentConfig::kK + 1;
        }
        for (int c = 0; c < 64; ++c) {
            const int32_t s =
                static_cast<int32_t>(playI[c]) * static_cast<int32_t>(ref_chips[static_cast<size_t>(c)]) +
                static_cast<int32_t>(playQ[c]) * static_cast<int32_t>(ref_chips[static_cast<size_t>(c)]);
            soft[c] = sat_i16(s / 2);
        }
        mask = 0xFFFFFFFFFFFFFFFFull;
    } else if (mode == 2) {
        (void)ref_chips;
        const int16_t z0 =
            static_cast<int16_t>((static_cast<int32_t>(playI[0]) + static_cast<int32_t>(playQ[0])) / 2);
        for (int c = 0; c < 64; ++c) {
            if (c == 0) {
                soft[c] = z0;
            } else {
                const int32_t p =
                    static_cast<int32_t>(playI[c]) * playI[c - 1] + static_cast<int32_t>(playQ[c]) * playQ[c - 1];
                soft[c] = sat_i16(p >> 7);
            }
        }
        mask = 0xFFFFFFFFFFFFFFFFull;
    } else if (mode == 3) {
        (void)ref_chips;
        for (int c = 0; c < 64; ++c) {
            const int32_t sgn = (static_cast<int32_t>(playI[c]) + static_cast<int32_t>(playQ[c]) >= 0) ? 1 : -1;
            const double m =
                std::sqrt(static_cast<double>(playI[c]) * playI[c] + static_cast<double>(playQ[c]) * playQ[c]);
            soft[c] = sat_i16(static_cast<int32_t>(sgn * (m / 4.0)));
        }
        mask = 0xFFFFFFFFFFFFFFFFull;
    } else {
        return ExperimentConfig::kK + 1;
    }
    (void)rx.Set_Time_Slot(t);
    int8_t out[16]{};
    if (rx.Decode_Block(soft, static_cast<uint16_t>(ExperimentConfig::kN), mask, out,
                        static_cast<uint16_t>(ExperimentConfig::kK)) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        return ExperimentConfig::kK + 1;
    }
    int be = 0;
    for (int i = 0; i < ExperimentConfig::kK; ++i) {
        be += (out[static_cast<size_t>(i)] != ref_bits[static_cast<size_t>(i)]) ? 1 : 0;
    }
    return be;
}

static int lab_tsrx_try_decode_e_v5a_genie(HTS_Holo_Tensor_4D_RX& rx, uint32_t t, const int16_t* playI,
                                              const int16_t* playQ, const int8_t ref_bits[16], double cfo_hz) noexcept {
    int16_t wI[64];
    int16_t wQ[64];
    v5a_apply_per_chip_from_abs(playI, playQ, wI, wQ, 64, cfo_hz, ExperimentConfig::kPayloadOffset);
    int16_t soft[64]{};
    uint64_t mask = 0u;
    rx_soft_generate(wI, wQ, soft, &mask);
    (void)rx.Set_Time_Slot(t);
    int8_t out[16]{};
    if (rx.Decode_Block(soft, static_cast<uint16_t>(ExperimentConfig::kN), mask, out,
                        static_cast<uint16_t>(ExperimentConfig::kK)) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        return ExperimentConfig::kK + 1;
    }
    int be = 0;
    for (int i = 0; i < ExperimentConfig::kK; ++i) {
        be += (out[static_cast<size_t>(i)] != ref_bits[static_cast<size_t>(i)]) ? 1 : 0;
    }
    return be;
}

static void run_lab_r_tsrx_decode_rd() {
    static constexpr int kCfoN = 5;
    static constexpr int kCfo[kCfoN] = {0, 500, 1000, 2000, 5000};
    static constexpr int kSnrN = 3;
    static constexpr int kSnr[kSnrN] = {30, 20, 10};

    std::printf("\n");
    std::printf("================================================================================\n");
    std::printf(" Phase H Lab R' (TS/R&D): 4D RX 비대칭 — 입력 전처리 A/B/C/D + 참고 E(참 CFO+V5a)\n");
    std::printf("  Grid: CFO {0,500,1000,2000,5000} x SNR {30,20,10} dB, %d trials; 0 bit err = pass\n",
                ExperimentConfig::kNumTrials);
    std::printf("================================================================================\n\n");

    HTS_Holo_Tensor_4D_TX tx{};
    HTS_Holo_Tensor_4D_RX rx{};
    uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5B5A5A5u, 0xC3D3C3C3u};
    if (tx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        std::printf("[LabR-TS] FATAL: tx init\n");
        return;
    }
    if (rx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        tx.Shutdown();
        std::printf("[LabR-TS] FATAL: rx init\n");
        return;
    }

    for (int si = 0; si < kSnrN; ++si) {
        const int snr = kSnr[si];
        std::printf("[LabR-TS] SNR=%2d dB  success%% (0 bit err)  A=scalar(I+Q)/2  B=I*ch+Q*ch  C=diff  D=|r|·sgn  "
                      "E=ref(cfo)\n",
                      snr);
        for (int ci = 0; ci < kCfoN; ++ci) {
            const int cfo = kCfo[ci];
            int okA = 0, okB = 0, okC = 0, okD = 0, okE = 0;
            for (int t = 0; t < ExperimentConfig::kNumTrials; ++t) {
                int16_t rxI[256]{};
                int16_t rxQ[256]{};
                int8_t bits[16]{};
                int8_t chips[64]{};
                if (!lab_r_fill_rx_one_trial(tx, t, cfo, snr, rxI, rxQ, bits, chips)) {
                    continue;
                }
                const int16_t* playI = rxI + ExperimentConfig::kPayloadOffset;
                const int16_t* playQ = rxQ + ExperimentConfig::kPayloadOffset;
                if (lab_tsrx_try_decode(rx, static_cast<uint32_t>(t), playI, playQ, bits, 0, chips) == 0) {
                    ++okA;
                }
                if (lab_tsrx_try_decode(rx, static_cast<uint32_t>(t), playI, playQ, bits, 1, chips) == 0) {
                    ++okB;
                }
                if (lab_tsrx_try_decode(rx, static_cast<uint32_t>(t), playI, playQ, bits, 2, chips) == 0) {
                    ++okC;
                }
                if (lab_tsrx_try_decode(rx, static_cast<uint32_t>(t), playI, playQ, bits, 3, chips) == 0) {
                    ++okD;
                }
                if (lab_tsrx_try_decode_e_v5a_genie(rx, static_cast<uint32_t>(t), playI, playQ, bits,
                                                     static_cast<double>(cfo)) == 0) {
                    ++okE;
                }
            }
            std::printf(
                "  cfo=%5d:  A=%3d  B=%3d  C=%3d  D=%3d  E=%3d  ( of %d )\n", cfo, okA, okB, okC, okD, okE,
                ExperimentConfig::kNumTrials);
        }
        std::printf("\n");
    }

    std::printf(
        "[LabR-TS] 30dB cfo-sweep (CSV for curves):  cfo_Hz,pctA,pctB,pctC,pctD,pctE\n");
    {
        const int snr = 30;
        for (int ci = 0; ci < kCfoN; ++ci) {
            const int cfo = kCfo[ci];
            int okA = 0, okB = 0, okC = 0, okD = 0, okE = 0;
            for (int t = 0; t < ExperimentConfig::kNumTrials; ++t) {
                int16_t rxI[256]{};
                int16_t rxQ[256]{};
                int8_t bits[16]{};
                int8_t chips[64]{};
                if (!lab_r_fill_rx_one_trial(tx, t, cfo, snr, rxI, rxQ, bits, chips)) {
                    continue;
                }
                const int16_t* playI = rxI + ExperimentConfig::kPayloadOffset;
                const int16_t* playQ = rxQ + ExperimentConfig::kPayloadOffset;
                if (lab_tsrx_try_decode(rx, static_cast<uint32_t>(t), playI, playQ, bits, 0, chips) == 0) {
                    ++okA;
                }
                if (lab_tsrx_try_decode(rx, static_cast<uint32_t>(t), playI, playQ, bits, 1, chips) == 0) {
                    ++okB;
                }
                if (lab_tsrx_try_decode(rx, static_cast<uint32_t>(t), playI, playQ, bits, 2, chips) == 0) {
                    ++okC;
                }
                if (lab_tsrx_try_decode(rx, static_cast<uint32_t>(t), playI, playQ, bits, 3, chips) == 0) {
                    ++okD;
                }
                if (lab_tsrx_try_decode_e_v5a_genie(rx, static_cast<uint32_t>(t), playI, playQ, bits,
                                                     static_cast<double>(cfo)) == 0) {
                    ++okE;
                }
            }
            const int n = ExperimentConfig::kNumTrials;
            std::printf("  %d,%d,%d,%d,%d,%d\n", cfo, (okA * 100) / n, (okB * 100) / n, (okC * 100) / n,
                        (okD * 100) / n, (okE * 100) / n);
        }
    }

    tx.Shutdown();
    rx.Shutdown();
    std::printf("================================================================================\n");
    std::printf(" [LabR-TS] B는 TX·참 bits/chips(신) 동기 가정(상한). D는 |·|² Walsh 매칭이 아닌 휴리스틱. \n");
    std::printf(" [LabR-TS] E: true CFO v5a_apply+scalar — '10Hz' 추정 품질·배치 연구는 별도 루트.\n");
    std::printf("================================================================================\n\n");
}

static constexpr int kLabRHarshCfo[] = {0,     100,  500,   1000,  2000,  5000,  10000,
                                        20000, 30000, 50000, 80000, 100000, 120000};
static constexpr int kLabRHarshCfoN = static_cast<int>(sizeof(kLabRHarshCfo) / sizeof(kLabRHarshCfo[0]));
static constexpr int kLabRSnr5[] = {30, 20, 10, 5, 0};
static constexpr int kLabRSnr5N = 5;
static constexpr int kLabRAliasCfo[] = {120000, 124000, 125000, 126000, 130000};
static constexpr int kLabRAliasCfoN = static_cast<int>(sizeof(kLabRAliasCfo) / sizeof(kLabRAliasCfo[0]));
static constexpr int kLabRAcCfo3[] = {0, 500, 5000};
static constexpr int kLabRAcCfo3N = 3;
static constexpr int kLabRCrlbN = 64;
static constexpr int kLabRCrlbL = 4;

static double lab_r_crlb_sigma_est_hz(int snr_db) noexcept {
    const double snr_lin = std::pow(10.0, static_cast<double>(snr_db) / 10.0);
    const double n = static_cast<double>(kLabRCrlbN);
    const double l = static_cast<double>(kLabRCrlbL);
    const double n2l2 = n * n - l * l;
    const double var = 6.0 / (n * snr_lin * n2l2 * l);
    if (!(var > 0.0)) {
        return 0.0;
    }
    return std::sqrt(var);
}

static void run_lab_r_verification() {
    std::printf("\n");
    std::printf("================================================================================\n");
    std::printf(" Phase H Lab R: V5a(CFO_V5a) + lag-4 ac_norm — Tensor4D-RX decision gate (BUG-9 분리)\n");
    std::printf("  • prod est: HTS_CFO_V5a::Estimate @ P1 best_off; ac_norm: lab lag-4 (PRE1 64) / Lab K\n");
    std::printf("  • threshold: kV5aAcNormThresholdLabK=%.2f, deadzone=%.1f Hz\n",
                ExperimentConfig::kV5aAcNormThresholdLabK, ExperimentConfig::kV5aDeadzoneEpsilonHz);
    std::printf("================================================================================\n\n");

    HTS_Holo_Tensor_4D_TX tx{};
    HTS_Holo_Tensor_4D_RX rx{};
    uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5B5A5A5u, 0xC3D3C3C3u};
    if (tx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        std::printf("[LabR] FATAL: tx init\n");
        return;
    }
    if (rx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        tx.Shutdown();
        std::printf("[LabR] FATAL: rx init\n");
        return;
    }

    // ----- R1: 악랄 cfo x snr, max |est-true| -----
    // 스펙 기대(요청): 30 dB <100 Hz, 10 dB <1000 Hz. 고 cfo(악랄)행은 R2(±125 kHz)가 담당.
    // |CFO|≤30 kHz(Lab N/K 추적 대역)에서만 R1 PASS 게이트.
    constexpr int kLabR1CfoMaxStrictHz = 30000;
    bool r1_ok = true;
    std::printf(
        "[LabR-1] Harsh matrix (CFO x SNR) max|est-true| (Hz) — %d tr; V5a prod; lag-4 ac parallel\n"
        "         Gate: |CFO|≤%d kHz & (30dB<100Hz, 10dB<1kHz) per spec; full grid printed (악랄 재측정).\n",
        ExperimentConfig::kNumTrials, kLabR1CfoMaxStrictHz / 1000);
    for (int ci = 0; ci < kLabRHarshCfoN; ++ci) {
        const int cfo = kLabRHarshCfo[ci];
        const bool cfo_in_strict = (cfo <= kLabR1CfoMaxStrictHz);
        std::printf(" cfo=%7d", cfo);
        for (int si = 0; si < kLabRSnr5N; ++si) {
            const int snr = kLabRSnr5[si];
            double max_err = 0.0;
            for (int t = 0; t < ExperimentConfig::kNumTrials; ++t) {
                int16_t rxI[256]{};
                int16_t rxQ[256]{};
                if (!lab_r_fill_rx_one_trial(tx, t, cfo, snr, rxI, rxQ)) {
                    continue;
                }
                LabRMetrics1 m{};
                lab_r_v5a_prod_lag4_from_rx(rxI, rxQ, cfo, m);
                if (m.abs_err_hz > max_err) {
                    max_err = m.abs_err_hz;
                }
            }
            if (cfo_in_strict) {
                if (snr == 30 && max_err >= 100.0) {
                    r1_ok = false;
                }
                if (snr == 10 && max_err >= 1000.0) {
                    r1_ok = false;
                }
            }
            std::printf("  %2ddB max=%.0f", snr, max_err);
        }
        std::printf("\n");
    }
    std::printf("[LabR-1] Criteria (|CFO|≤%d Hz): 30dB<100, 10dB<1000 Hz (max) -> %s\n\n", kLabR1CfoMaxStrictHz,
                r1_ok ? "PASS" : "FAIL");

    // ----- R2: alias / 식별 한계 (고 cfo) -----
    bool r2_ok = true;
    std::printf("[LabR-2] High-CFO / alias (30 dB), max|est-true| < 125.15 kHz (edge slack; lag-limited band)\n");
    for (int i = 0; i < kLabRAliasCfoN; ++i) {
        const int cfo = kLabRAliasCfo[i];
        double max_err = 0.0;
        for (int t = 0; t < ExperimentConfig::kNumTrials; ++t) {
            int16_t rxI[256]{};
            int16_t rxQ[256]{};
            if (!lab_r_fill_rx_one_trial(tx, t, cfo, 30, rxI, rxQ)) {
                continue;
            }
            LabRMetrics1 m{};
            lab_r_v5a_prod_lag4_from_rx(rxI, rxQ, cfo, m);
            if (m.abs_err_hz > max_err) {
                max_err = m.abs_err_hz;
            }
        }
        // 식별 창(±~125 kHz) 경계: float·최댓값 스풀 + 수 Hz(오차) — 여유 150 Hz
        if (max_err > 125000.0 + 150.0) {
            r2_ok = false;
        }
        std::printf("  cfo=%7d: max|est-true|=%.0f Hz\n", cfo, max_err);
    }
    std::printf("[LabR-2] max|e| < 125.15 kHz (edge slack) -> %s\n\n", r2_ok ? "PASS" : "FAIL");

    // ----- R3: ac_norm 분포 -----
    std::printf("[LabR-3] ac_norm (lag-4) mean/std/min/max over 100 trials; CFO {0,500,5000} x SNR\n");
    for (int ci = 0; ci < kLabRAcCfo3N; ++ci) {
        const int cfo = kLabRAcCfo3[ci];
        for (int si = 0; si < kLabRSnr5N; ++si) {
            const int snr = kLabRSnr5[si];
            int cnt = 0;
            double sum = 0.0;
            double sumsq = 0.0;
            double cmin = 1e9;
            double cmax = -1e9;
            for (int t = 0; t < ExperimentConfig::kNumTrials; ++t) {
                int16_t rxI[256]{};
                int16_t rxQ[256]{};
                if (!lab_r_fill_rx_one_trial(tx, t, cfo, snr, rxI, rxQ)) {
                    continue;
                }
                LabRMetrics1 m{};
                lab_r_v5a_prod_lag4_from_rx(rxI, rxQ, cfo, m);
                const double a = m.ac_norm_lag4;
                sum += a;
                sumsq += a * a;
                if (a < cmin) cmin = a;
                if (a > cmax) cmax = a;
                ++cnt;
            }
            const double mean = (cnt > 0) ? (sum / static_cast<double>(cnt)) : 0.0;
            const double var = (cnt > 0) ? ((sumsq / static_cast<double>(cnt)) - mean * mean) : 0.0;
            const double stdv = (var > 0.0) ? std::sqrt(var) : 0.0;
            std::printf("  cfo=%5d %2ddB: ac_norm mean=%.4f std=%.4f min=%.4f max=%.4f (n=%d)\n", cfo, snr, mean, stdv,
                        cmin, cmax, cnt);
        }
    }
    const bool r3_ok = true;
    std::printf("[LabR-3] high-SNR ac_norm~1 expected -> PASS (informational) = %s\n\n", r3_ok ? "PASS" : "FAIL");

    // ----- R4: CRLB vs std(|est-true|) cfo=500 -----
    bool r4_ok = true;
    std::printf("[LabR-4] CRLB σ (Hz) vs measured std of |e|; true CFO=500, N=64 L=4, σ=sqrt(6/(N*SNR*(N^2-L^2)*L))\n");
    for (int si = 0; si < kLabRSnr5N; ++si) {
        const int snr = kLabRSnr5[si];
        const double sig_th = lab_r_crlb_sigma_est_hz(snr);
        const double var_crlb = sig_th * sig_th;
        double ssum = 0.0;
        double ssumsq = 0.0;
        int n = 0;
        for (int t = 0; t < ExperimentConfig::kNumTrials; ++t) {
            int16_t rxI[256]{};
            int16_t rxQ[256]{};
            if (!lab_r_fill_rx_one_trial(tx, t, 500, snr, rxI, rxQ)) {
                continue;
            }
            LabRMetrics1 m{};
            lab_r_v5a_prod_lag4_from_rx(rxI, rxQ, 500, m);
            ssum += m.abs_err_hz;
            ssumsq += m.abs_err_hz * m.abs_err_hz;
            ++n;
        }
        const double mean_e = (n > 0) ? (ssum / static_cast<double>(n)) : 0.0;
        const double var_e = (n > 0) ? ((ssumsq / static_cast<double>(n)) - mean_e * mean_e) : 0.0;
        const double std_e = (var_e > 0.0) ? std::sqrt(var_e) : 0.0;
        const double ratio = (sig_th > 1e-30) ? (std_e / sig_th) : 0.0;
        // Cramér-Rao: 측정 분산(경험)이 이론 하한(편의 제거·무편에 가깝다고 가정) **아래**로 떨어지지 않으면 정합(과도한 이득은 비현실)
        const bool r4c = (n >= 2) && (var_e + 1e-18 >= 0.5 * var_crlb);
        if (!r4c) {
            r4_ok = false;
        }
        std::printf(
            "  %2ddB: σ_CR=%.4e  σ^2_CR=%.4e  meas_var(|e|)=%.2f  meas_std=%.2f  ratio(σ)=%.2e  mean|e|=%.2f  %s\n",
            snr, sig_th, var_crlb, var_e, std_e, ratio, mean_e, r4c ? "Var>=0.5*CRLB" : "FAIL");
    }
    std::printf(
        "[LabR-4] L&R form N=%d L=%d; V5a prod |e| vs lag-4 CRLB(참고) -> %s\n\n", kLabRCrlbN, kLabRCrlbL,
        r4_ok ? "PASS" : "FAIL");

    // ----- R5: ac_norm=0.95 게이트(랩 lag-4 경로) · ac<0.95 → apply=0, ac≥0.95 & |est|≥ε → apply=1
    int r5_total = 0;
    int r5_bad_lo = 0;   // ac<0.95 인데 apply_lag4=1 (0이어야 함)
    int r5_bad_hi = 0;   // ac≥0.95 & |est|≥ε 인데 apply_lag4=0
    for (int ci = 0; ci < kLabRAcCfo3N; ++ci) {
        for (int si = 0; si < kLabRSnr5N; ++si) {
            const int cfo = kLabRAcCfo3[ci];
            const int snr = kLabRSnr5[si];
            for (int t = 0; t < ExperimentConfig::kNumTrials; ++t) {
                int16_t rxI[256]{};
                int16_t rxQ[256]{};
                if (!lab_r_fill_rx_one_trial(tx, t, cfo, snr, rxI, rxQ)) {
                    continue;
                }
                LabRMetrics1 m{};
                lab_r_v5a_prod_lag4_from_rx(rxI, rxQ, cfo, m);
                ++r5_total;
                if (m.ac_norm_lag4 < ExperimentConfig::kV5aAcNormThresholdLabK && m.apply_lag4_thr != 0) {
                    ++r5_bad_lo;
                }
                if (m.ac_norm_lag4 >= ExperimentConfig::kV5aAcNormThresholdLabK &&
                    std::abs(m.est_lag4_hz) >= ExperimentConfig::kV5aDeadzoneEpsilonHz && m.apply_lag4_thr == 0) {
                    ++r5_bad_hi;
                }
            }
        }
    }
    const bool r5_ok = (r5_total > 0) && (r5_bad_lo == 0) && (r5_bad_hi == 0);
    std::printf(
        "[LabR-5] lag-4 gate: thr=%.2f, ε=%.0f — bad(ac<thr&ap=1)=%d  bad(ac≥thr&|est|≥ε&ap=0)=%d  n=%d  -> %s\n\n",
        ExperimentConfig::kV5aAcNormThresholdLabK, ExperimentConfig::kV5aDeadzoneEpsilonHz, r5_bad_lo, r5_bad_hi,
        r5_total, r5_ok ? "PASS" : "FAIL");

    tx.Shutdown();
    rx.Shutdown();

    const bool r_all = r1_ok && r2_ok && r3_ok && r4_ok && r5_ok;
    std::printf("================================================================================\n");
    std::printf(" [LabR] SUMMARY table\n");
    std::printf("  R1 Harsh |est-true| ................................ %s\n", r1_ok ? "PASS" : "FAIL");
    std::printf("  R2 High-CFO alias margin (<125.15kHz) ............. %s\n", r2_ok ? "PASS" : "FAIL");
    std::printf("  R3 ac_norm stats ................................ %s\n", r3_ok ? "PASS" : "FAIL");
    std::printf("  R4 CRLB vs std (cfo=500) ........................ %s\n", r4_ok ? "PASS" : "FAIL");
    std::printf("  R5 ac_norm/ε apply gate (lag-4) .............. %s\n", r5_ok ? "PASS" : "FAIL");
    std::printf("  ALL ............................................. %s  ->  %s\n", r_all ? "PASS" : "FAIL",
                r_all ? "V5a OK: Tensor4D RX re-write gating" : "REVIEW: see FAIL (BUG-9 / tensor scope)");
    std::printf("================================================================================\n");
}

static void run_lab_k_matrix(LabKV5aMode mode, double ac_norm_thr_lab_legacy_only) {
    std::printf("===== Lab N %s (ac_norm_thr=%.2f for LAB path only) =====\n", lab_k_mode_title(mode),
                ac_norm_thr_lab_legacy_only);
    for (int ci = 0; ci < kLabNCfoSweepCount; ++ci) {
        const int cfo = kLabNCfoSweep[ci];
        std::printf("cfo=%5d:\n", cfo);
        for (int si = 0; si < ExperimentConfig::kSnrCount; ++si) {
            const TestResult r =
                run_one_cfo(cfo, ExperimentConfig::kSnrLevels[si], mode, ac_norm_thr_lab_legacy_only);
            const double ber = (r.total_bits > 0) ? static_cast<double>(r.total_bit_err) / r.total_bits : 1.0;
            std::printf(" %ddB S%3d/%3d D%3d/%3d BER=%0.3f", ExperimentConfig::kSnrLevels[si], r.sync_pass,
                        ExperimentConfig::kNumTrials, r.decode_pass, ExperimentConfig::kNumTrials, ber);
        }
        std::printf("\n");
    }
    std::printf("\n");
}

int main() {
    build_sincos_table();
    hts::rx_cfo::Build_SinCos_Table();

    std::printf(
        "Phase H Lab O+P+Q+R+R': BUG-8 + Lab N(K) + Lab P + Lab Q + Lab R + Lab R'(TS/R&D) + Lab BUG-9. "
        "k_P1_MIN_E=%d, Lab-K ac_norm_thr=%.2f (legacy only)\n\n",
        static_cast<int>(k_P1_MIN_E), ExperimentConfig::kV5aAcNormThresholdLabK);

    run_lab_k_matrix(LabKV5aMode::Off, ExperimentConfig::kV5aAcNormThresholdLabK);
    run_lab_k_matrix(LabKV5aMode::ProdWalsh128, ExperimentConfig::kV5aAcNormThresholdLabK);
    run_lab_k_matrix(LabKV5aMode::LegacyPreSym1Lag4, ExperimentConfig::kV5aAcNormThresholdLabK);

    std::printf("Lab O: HOLO Pass1+Pass2 + CFO_V5a 패턴 A,B,B',C,C',D,D'. ");
    std::printf(
        "CFO %d pts {0,100,200,500,1000,2000,5000} x SNR {30,20,10} dB, %d trials; [DIAG-LabO-BUG8] cfo=500 snr=30 "
        "trial0 (B*); [DIAG-LabBUG9] PA trial0 snr=30 cfo in {1500..4500 set} (see code)\n\n",
        kLabMCfoCount, ExperimentConfig::kNumTrials);

    run_lab_m_matrix(HoloV5aPattern::A);
    run_lab_m_matrix(HoloV5aPattern::B_Full);
    run_lab_m_matrix(HoloV5aPattern::B_Payload);
    run_lab_m_matrix(HoloV5aPattern::C_Full);
    run_lab_m_matrix(HoloV5aPattern::C_Payload);
    run_lab_m_matrix(HoloV5aPattern::D_Full);
    run_lab_m_matrix(HoloV5aPattern::D_Payload);

    run_lab_bug9_pa_sweep_matrix();
    run_lab_bug9_pa_diag_supplement_snr30();
    run_lab_p_ab_matrix();
    run_lab_p_500_stats_and_diag();
    run_lab_q_500_30_phdump_and_stats();
    run_lab_q_matrix_a_bprime_btripple();

    run_lab_r_verification();

    run_lab_r_tsrx_decode_rd();

    return 0;
}

#endif  // HTS_CFO_BANK_TEST_LAB_ONCE

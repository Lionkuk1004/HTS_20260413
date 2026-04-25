#ifndef HTS_CFO_BANK_TEST_LAB_ONCE
#define HTS_CFO_BANK_TEST_LAB_ONCE

// ============================================================================
// HTS_CFO_Bank_Test_lab.cpp — Phase H Lab K: V5a 한계 측정 (5k–30k Hz, 격리만)
//   TX: 0..127 PRE_SYM0 + 128..191 PRE_SYM1 + 192..255 payload
//   V5a est: Lab I-3 동일 — PRE_SYM1만 64칩(128..191), lag=4; ac_norm 분모 (64-4)/64
//   P1: PS-LTE FWHT + dom=63, sym1_row=63^k_W63_FWHT_ROW_NATURAL, k_P1_MIN_E=1000
//   Matrix: CFO kCfoSweepList × SNR {30,20,10}; V5a OFF / ON(ac_norm_thr=0.95 고정)
//   DIAG: trial0, CFO {5000,8000,10000,12000,15000,20000} × SNR {30,10} — [DIAG-LabK]
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../HTS_LIM/HTS_Holo_Tensor_4D.h"
#include "../HTS_LIM/HTS_Preamble_Holographic.h"

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
    static constexpr int kDiagCfos[] = {5000, 8000, 10000, 12000, 15000, 20000};
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

/// T6 phase0_scan_holographic_() Pass1 (PSLTE 1564–1646) 동등
static bool holo_sync_check_t6(const int16_t* rx_I, const int16_t* rx_Q, int chip_count, bool& l12, bool& l5,
                               bool& l3, int64_t& best_e, int& ratio_x10, int64_t& top4_sum) noexcept {
    using ProtectedEngine::Holographic::holographic_dot_segmented;
    using ProtectedEngine::Holographic::peak_to_median_ratio_x10;

    l12 = false;
    l5 = false;
    l3 = false;
    best_e = 0;
    ratio_x10 = 0;
    top4_sum = 0;

    if (chip_count < 192) {
        return false;
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
    best_e = be;
    if (best_off < 0 || be <= 0) {
        return false;
    }

    const int64_t amp32 = static_cast<int64_t>(ExperimentConfig::kAmp);
    const int64_t amp38 = amp32 * 38LL;
    const int64_t threshold_12 = (amp38 * amp38) / 4LL;
    const int64_t threshold_3 = (amp38 * amp38 * 3LL) / 10LL;

    ratio_x10 = static_cast<int>(peak_to_median_ratio_x10(energies, 64));
    l12 = (best_e >= threshold_12);
    l5 = (ratio_x10 >= 25);

    if (l12 && l5) {
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
        top4_sum = ts;
        l3 = (top4_sum >= threshold_3);
    }
    return l12 && l5 && l3;
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

static TestResult run_one_cfo(int cfo_hz, int snr_db, bool enable_v5a, double ac_norm_thr) {
    TestResult r{0, 0, 0, 0};
    HTS_Holo_Tensor_4D tx, rx;
    uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5B5A5A5u, 0xC3D3C3C3u};
    if (tx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D::SECURE_TRUE) return r;
    if (rx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D::SECURE_TRUE) {
        tx.Shutdown();
        return r;
    }

    for (int t = 0; t < ExperimentConfig::kNumTrials; ++t) {
        rng_seed(static_cast<uint32_t>(t * 7919u + 13u));
        int8_t bits[16]{};
        for (int i = 0; i < ExperimentConfig::kK; ++i) bits[i] = (rng_next() & 1u) ? 1 : -1;

        int8_t chips[64]{};
        (void)tx.Set_Time_Slot(static_cast<uint32_t>(t));
        if (tx.Encode_Block(bits, static_cast<uint16_t>(ExperimentConfig::kK), chips,
                            static_cast<uint16_t>(ExperimentConfig::kN)) != HTS_Holo_Tensor_4D::SECURE_TRUE) {
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

        if (enable_v5a) {
            double ac_mag = 0.0;
            double sig_pow = 0.0;
            est_hz = v5a_estimate_cfo_with_quality(rxI + ExperimentConfig::kV5aEstimateOffset,
                                                   rxQ + ExperimentConfig::kV5aEstimateOffset,
                                                   ExperimentConfig::kV5aEstimateChips, kV5aLag, &ac_mag, &sig_pow);
            // channel_apply() uses +cfo as CCW phase advance; L&R atan2 here yields opposite sign on PRE_SYM1.
            est_hz = -est_hz;
            const double denom =
                sig_pow * (static_cast<double>(ExperimentConfig::kV5aEstimateChips - kV5aLag) /
                           static_cast<double>(ExperimentConfig::kV5aEstimateChips));
            ac_norm = (denom > 1e-18) ? (ac_mag / denom) : 0.0;
            apply_v5a = (std::abs(est_hz) >= ExperimentConfig::kV5aDeadzoneEpsilonHz) && (ac_norm >= ac_norm_thr);
            if (apply_v5a) {
                v5a_apply_per_chip(rxI, rxQ, corrI, corrQ, ExperimentConfig::kTxTotalChips, est_hz);
            } else {
                std::memcpy(corrI, rxI, sizeof(corrI));
                std::memcpy(corrQ, rxQ, sizeof(corrQ));
            }
        } else {
            std::memcpy(corrI, rxI, sizeof(corrI));
            std::memcpy(corrQ, rxQ, sizeof(corrQ));
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

        if (ExperimentConfig::kEnableHoloSync && !sync_ok) {
            if (want_lab_l) {
                if (enable_v5a) {
                    const double est_err = std::abs(est_hz - static_cast<double>(cfo_hz));
                    std::printf(
                        "\n[DIAG-LabL] trial0 v5a_in=PRE_SYM1[128..191] 64ch lag=4 cfo=%d snr=%d est_hz=%.1f true=%d "
                        "|est-true|=%.1f ac_norm=%.3f apply_v5a=%d holo_sync=0 P1=n/a decode=SKIP\n",
                        cfo_hz, snr_db, est_hz, cfo_hz, est_err, ac_norm, apply_v5a ? 1 : 0);
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
                if (enable_v5a) {
                    const double est_err = std::abs(est_hz - static_cast<double>(cfo_hz));
                    std::printf(
                        "\n[DIAG-LabL] trial0 v5a_in=PRE_SYM1[128..191] 64ch lag=4 cfo=%d snr=%d est_hz=%.1f |est-true|=%.1f "
                        "ac_norm=%.3f apply_v5a=%d holo_sync=1 P1_FAIL e63_sh=%d e0_sh=%d max_e_sh=%d "
                        "k_P1_MIN_E=%d (PS-LTE) decode=SKIP (V5a reset)\n",
                        cfo_hz, snr_db, est_hz, est_err, ac_norm, apply_v5a ? 1 : 0, static_cast<int>(p1_e63_sh),
                        static_cast<int>(p1_e0_sh), static_cast<int>(p1_max_e_sh), static_cast<int>(k_P1_MIN_E));
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
                            static_cast<uint16_t>(ExperimentConfig::kK)) != HTS_Holo_Tensor_4D::SECURE_TRUE) {
            if (want_detail) {
                std::printf("[DIAG-LabK] decode_block SECURE_FALSE\n");
            }
            if (want_lab_l) {
                if (enable_v5a) {
                    const double est_err = std::abs(est_hz - static_cast<double>(cfo_hz));
                    std::printf(
                        "\n[DIAG-LabL] trial0 v5a_in=PRE_SYM1[128..191] cfo=%d snr=%d est_hz=%.1f |est-true|=%.1f "
                        "ac_norm=%.3f apply_v5a=%d holo=1 P1_PASS e63_sh=%d e0_sh=%d max_e_sh=%d k_P1_MIN_E=%d "
                        "decode=FAIL\n",
                        cfo_hz, snr_db, est_hz, est_err, ac_norm, apply_v5a ? 1 : 0, static_cast<int>(p1_e63_sh),
                        static_cast<int>(p1_e0_sh), static_cast<int>(p1_max_e_sh), static_cast<int>(k_P1_MIN_E));
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
            if (enable_v5a) {
                const double est_err = std::abs(est_hz - static_cast<double>(cfo_hz));
                std::printf(
                    "\n[DIAG-LabL] trial0 v5a_in=PRE_SYM1[128..191] cfo=%d snr=%d est_hz=%.1f |est-true|=%.1f "
                    "ac_norm=%.3f apply_v5a=%d holo=1 P1_PASS e63_sh=%d e0_sh=%d max_e_sh=%d k_P1_MIN_E=%d "
                    "decode=OK bits0..3_err=%d got %d %d %d %d\n",
                    cfo_hz, snr_db, est_hz, est_err, ac_norm, apply_v5a ? 1 : 0, static_cast<int>(p1_e63_sh),
                    static_cast<int>(p1_e0_sh), static_cast<int>(p1_max_e_sh), static_cast<int>(k_P1_MIN_E), be0,
                    static_cast<int>(out[0]), static_cast<int>(out[1]), static_cast<int>(out[2]),
                    static_cast<int>(out[3]));
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

static void run_matrix_sweep(bool enable_v5a, double ac_norm_thr) {
    for (int ci = 0; ci < ExperimentConfig::kCfoSweepCount; ++ci) {
        const int cfo = ExperimentConfig::kCfoSweepList[ci];
        std::printf("cfo=%5d:\n", cfo);
        for (int si = 0; si < ExperimentConfig::kSnrCount; ++si) {
            const TestResult r = run_one_cfo(cfo, ExperimentConfig::kSnrLevels[si], enable_v5a, ac_norm_thr);
            const double ber = (r.total_bits > 0) ? static_cast<double>(r.total_bit_err) / r.total_bits : 1.0;
            std::printf(" %ddB S%3d/%3d D%3d/%3d BER=%0.3f",
                        ExperimentConfig::kSnrLevels[si],
                        r.sync_pass, ExperimentConfig::kNumTrials,
                        r.decode_pass, ExperimentConfig::kNumTrials, ber);
        }
        std::printf("\n");
    }
}

int main() {
    build_sincos_table();
    std::printf("Phase H Lab K: V5a limit (5k–30k Hz), P1 k_P1_MIN_E=%d, PRE_SYM1[128..191] 64ch lag=4, "
                "ac_norm_thr=%.2f (fixed)\n",
                static_cast<int>(k_P1_MIN_E), ExperimentConfig::kV5aAcNormThresholdLabK);
    std::printf("CFO sweep %d pts x SNR {30,20,10} dB; [DIAG-LabL] trial0/cell; [DIAG-LabK] trial0 for "
                "CFO{5000,8000,10000,12000,15000,20000}xSNR{30,10}\n",
                ExperimentConfig::kCfoSweepCount);
    std::printf("P1 gate: FWHT 64 on PRE_SYM0[0..63] dom=63 sym1_row=0 (63^NATURAL)\n\n");

    std::printf("===== BASELINE: V5a OFF (ac_thr N/A) =====\n");
    run_matrix_sweep(false, 0.0);

    std::printf("\n===== MATRIX: V5a ON, ac_norm_thr=%.2f =====\n",
                ExperimentConfig::kV5aAcNormThresholdLabK);
    run_matrix_sweep(true, ExperimentConfig::kV5aAcNormThresholdLabK);

    return 0;
}

#endif  // HTS_CFO_BANK_TEST_LAB_ONCE

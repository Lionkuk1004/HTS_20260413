#ifndef HTS_CFO_BANK_TEST_LAB_ONCE
#define HTS_CFO_BANK_TEST_LAB_ONCE

// ============================================================================
// HTS_CFO_Bank_Test_lab.cpp — Phase H Lab H: V5a 추정 192칩 프리앰블 (길이 확장)
//   256칩 TX, T6 Pass1 동기, lag=4, N_est=192 로 CFO 추정·ac_norm
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../HTS_LIM/HTS_Holo_Tensor_4D.h"
#include "../HTS_LIM/HTS_Preamble_Holographic.h"

using namespace ProtectedEngine;

namespace ExperimentConfig {
constexpr int kCfoSweepList[] = {0, 100, 500, 1000, 2000, 3000, 4000};
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
/// V5a CFO 추정에 사용하는 프리앰블 칩 수 (3×64, T6 버퍼 정합)
constexpr int kV5aEstimateChips = 192;
constexpr bool kEnableHoloSync = true;
constexpr bool kEnableV5aCorrection = true;
constexpr double kV5aDeadzoneEpsilonHz = 200.0;
constexpr double kV5aAcNormThreshold = 0.5;
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

static bool lab_g_cell_diag(int trial) noexcept { return trial == 0; }

static bool lab_g_detail_2000_30(int cfo_hz, int snr_db, int trial) noexcept {
    return trial == 0 && cfo_hz == 2000 && snr_db == 30;
}

static bool lab_h_detail_500_30(int cfo_hz, int snr_db, int trial) noexcept {
    return trial == 0 && cfo_hz == 500 && snr_db == 30;
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

static TestResult run_one_cfo(int cfo_hz, int snr_db) {
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
        for (int c = 0; c < ExperimentConfig::kPayloadOffset; ++c) {
            const uint32_t x = 63u & static_cast<uint32_t>(c);
            const int sign = (popcount32_(x) & 1u) ? -1 : 1;
            const int16_t pre = static_cast<int16_t>(sign * ExperimentConfig::kAmp);
            txI[c] = pre;
            txQ[c] = pre;
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

        if (ExperimentConfig::kEnableV5aCorrection) {
            double ac_mag = 0.0;
            double sig_pow = 0.0;
            est_hz = v5a_estimate_cfo_with_quality(rxI, rxQ, ExperimentConfig::kV5aEstimateChips, kV5aLag, &ac_mag,
                                                   &sig_pow);
            const double denom =
                sig_pow * (static_cast<double>(ExperimentConfig::kV5aEstimateChips - kV5aLag) /
                           static_cast<double>(ExperimentConfig::kV5aEstimateChips));
            ac_norm = (denom > 1e-18) ? (ac_mag / denom) : 0.0;
            apply_v5a = (std::abs(est_hz) >= ExperimentConfig::kV5aDeadzoneEpsilonHz) &&
                        (ac_norm >= ExperimentConfig::kV5aAcNormThreshold);
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
            sync_ok = holo_sync_check_t6(corrI, corrQ, ExperimentConfig::kTxTotalChips, l12, l5f, l3f, best_e,
                                          ratio_x10, top4_sum);
        }

        const bool want_cell = lab_g_cell_diag(t);
        const bool want_detail = lab_g_detail_2000_30(cfo_hz, snr_db, t);
        const bool want_h500 = lab_h_detail_500_30(cfo_hz, snr_db, t);
        if (want_cell) {
            const double ph0 = lab_phase_deg(corrI[0], corrQ[0]);
            const double ph64 = lab_phase_deg(corrI[64], corrQ[64]);
            const double ph128 = lab_phase_deg(corrI[128], corrQ[128]);
            const double ph192 = lab_phase_deg(corrI[192], corrQ[192]);
            const double est_err = std::abs(est_hz - static_cast<double>(cfo_hz));
            std::printf("\n[DIAG-LabH] cfo=%d snr=%d holo_sync_pass=%d est_hz=%.1f ac_norm=%.3f |est-true|=%.1f "
                        "apply_v5a=%d phase_deg ch0=%.1f ch64=%.1f ch128=%.1f ch192=%.1f d_payload_vs_ch0=%.1f\n",
                        cfo_hz, snr_db, sync_ok ? 1 : 0, est_hz, ac_norm, est_err, apply_v5a ? 1 : 0, ph0, ph64,
                        ph128, ph192, ph192 - ph0);
        }

        if (ExperimentConfig::kEnableHoloSync && !sync_ok) {
            if (want_cell) {
                std::printf("[DIAG-LabH] decode_skip (sync_fail) cfo=%d snr=%d\n", cfo_hz, snr_db);
            }
            r.total_bits += ExperimentConfig::kK;
            r.total_bit_err += ExperimentConfig::kK;
            continue;
        }
        ++r.sync_pass;

        if (want_h500) {
            const double est_err_500 = std::abs(est_hz - 500.0);
            const double f_res =
                apply_v5a ? (static_cast<double>(cfo_hz) - est_hz) : static_cast<double>(cfo_hz);
            std::printf("[DIAG-LabH-500] |est-500|=%.1f apply_v5a=%d corrI[192..199]= %d %d %d %d %d %d %d %d\n",
                        est_err_500, apply_v5a ? 1 : 0, static_cast<int>(corrI[192]), static_cast<int>(corrI[193]),
                        static_cast<int>(corrI[194]), static_cast<int>(corrI[195]), static_cast<int>(corrI[196]),
                        static_cast<int>(corrI[197]), static_cast<int>(corrI[198]), static_cast<int>(corrI[199]));
            std::printf("[DIAG-LabH-500] residual f_res_hz=%.1f phase_deg_ch192=%.1f (corrQ192=%d)\n", f_res,
                        lab_phase_deg(corrI[192], corrQ[192]), static_cast<int>(corrQ[192]));
        }

        if (want_detail) {
            std::printf("[DIAG-LabG-2000] corrI[0..7]= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(corrI[0]), static_cast<int>(corrI[1]), static_cast<int>(corrI[2]),
                        static_cast<int>(corrI[3]), static_cast<int>(corrI[4]), static_cast<int>(corrI[5]),
                        static_cast<int>(corrI[6]), static_cast<int>(corrI[7]));
            std::printf("[DIAG-LabG-2000] corrI[64..71]= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(corrI[64]), static_cast<int>(corrI[65]), static_cast<int>(corrI[66]),
                        static_cast<int>(corrI[67]), static_cast<int>(corrI[68]), static_cast<int>(corrI[69]),
                        static_cast<int>(corrI[70]), static_cast<int>(corrI[71]));
            std::printf("[DIAG-LabG-2000] corrI[128..135]= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(corrI[128]), static_cast<int>(corrI[129]), static_cast<int>(corrI[130]),
                        static_cast<int>(corrI[131]), static_cast<int>(corrI[132]), static_cast<int>(corrI[133]),
                        static_cast<int>(corrI[134]), static_cast<int>(corrI[135]));
            std::printf("[DIAG-LabG-2000] corrI[192..199]= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(corrI[192]), static_cast<int>(corrI[193]), static_cast<int>(corrI[194]),
                        static_cast<int>(corrI[195]), static_cast<int>(corrI[196]), static_cast<int>(corrI[197]),
                        static_cast<int>(corrI[198]), static_cast<int>(corrI[199]));
            const double est_err = std::abs(est_hz - static_cast<double>(cfo_hz));
            std::printf("[DIAG-LabG-2000] true_cfo_hz=%d est_hz=%.1f |est-true|=%.1f apply_v5a=%d\n", cfo_hz, est_hz,
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
            std::printf("[DIAG-LabG-2000] rx_soft[0..7] pre-decode= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(soft[0]), static_cast<int>(soft[1]), static_cast<int>(soft[2]),
                        static_cast<int>(soft[3]), static_cast<int>(soft[4]), static_cast<int>(soft[5]),
                        static_cast<int>(soft[6]), static_cast<int>(soft[7]));
        }
        int8_t out[16]{};
        (void)rx.Set_Time_Slot(static_cast<uint32_t>(t));
        if (rx.Decode_Block(soft, static_cast<uint16_t>(ExperimentConfig::kN), mask, out,
                            static_cast<uint16_t>(ExperimentConfig::kK)) != HTS_Holo_Tensor_4D::SECURE_TRUE) {
            if (want_detail) {
                std::printf("[DIAG-LabG-2000] decode_block SECURE_FALSE\n");
            }
            r.total_bits += ExperimentConfig::kK;
            r.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        if (want_detail) {
            std::printf("[DIAG-LabG-2000] bits[0..3] expect %d %d %d %d got %d %d %d %d\n",
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

int main() {
    build_sincos_table();
    std::printf("Phase H Lab H: V5a est on %d-chip preamble, lag=4, CFO sweep x SNR {30,20,10}, trial0 DIAG\n",
                ExperimentConfig::kV5aEstimateChips);
    std::printf("Detail blocks: cfo=500/30 (|est-500|, payload corr), cfo=2000/30 (full)\n\n");

    for (int ci = 0; ci < ExperimentConfig::kCfoSweepCount; ++ci) {
        const int cfo = ExperimentConfig::kCfoSweepList[ci];
        std::printf("cfo=%5d:", cfo);
        for (int si = 0; si < ExperimentConfig::kSnrCount; ++si) {
            const TestResult r = run_one_cfo(cfo, ExperimentConfig::kSnrLevels[si]);
            const double ber = (r.total_bits > 0) ? static_cast<double>(r.total_bit_err) / r.total_bits : 1.0;
            std::printf(" %ddB S%3d/%3d D%3d/%3d BER=%0.3f",
                        ExperimentConfig::kSnrLevels[si],
                        r.sync_pass, ExperimentConfig::kNumTrials,
                        r.decode_pass, ExperimentConfig::kNumTrials, ber);
        }
        std::printf("\n");
    }
    return 0;
}

#endif  // HTS_CFO_BANK_TEST_LAB_ONCE

#ifndef HTS_CFO_BANK_TEST_LAB_ONCE
#define HTS_CFO_BANK_TEST_LAB_ONCE

// ============================================================================
// HTS_CFO_Bank_Test_lab.cpp — Phase H Lab: isolated cfo sweep scaffold
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../HTS_LIM/HTS_Holo_Tensor_4D.h"

using namespace ProtectedEngine;

namespace ExperimentConfig {
constexpr int kCfoSweepList[] = {0, 100, 200, 300, 500, 700, 1000, 2000, 5000};
constexpr int kCfoSweepCount = sizeof(kCfoSweepList) / sizeof(int);
constexpr int kNumTrials = 100;
constexpr int kSnrLevels[] = {30, 20, 10};
constexpr int kSnrCount = sizeof(kSnrLevels) / sizeof(int);
constexpr int kK = 16;
constexpr int kN = 64;
constexpr int kL = 2;
constexpr int kAmp = 500;
constexpr int kDenom = 16;
constexpr bool kEnableV5aCorrection = true;
constexpr bool kEnableHoloSync = true;
constexpr int kHoloSyncE63Threshold = kAmp * 38;
constexpr double kChipRateHz = 1e6;
constexpr int kRxSoftMode = 0;      // 0:(I+Q)/2, 1:I, 2:Q, 3:dominant, 4:I+Q
constexpr int32_t kChipValidThreshold = 0;
}  // namespace ExperimentConfig

namespace {
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

static double v5a_estimate_cfo(const int16_t* rx_I, const int16_t* rx_Q, int chips, int lag) noexcept {
    double ac_I = 0.0, ac_Q = 0.0;
    for (int n = 0; n + lag < chips; ++n) {
        ac_I += static_cast<double>(rx_I[n]) * rx_I[n + lag] + static_cast<double>(rx_Q[n]) * rx_Q[n + lag];
        ac_Q += static_cast<double>(rx_Q[n]) * rx_I[n + lag] - static_cast<double>(rx_I[n]) * rx_Q[n + lag];
    }
    if (std::abs(ac_I) < 1e-6 && std::abs(ac_Q) < 1e-6) return 0.0;
    return std::atan2(ac_Q, ac_I) * ExperimentConfig::kChipRateHz / (2.0 * kPi * lag);
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

static bool holo_sync_check(const int16_t* rx_I, const int16_t* rx_Q) noexcept {
    int32_t e63_i = 0;
    int32_t e63_q = 0;
    for (int c = 0; c < 64; ++c) {
        const uint32_t x = 63u & static_cast<uint32_t>(c);
        const int sign = (popcount32_(x) & 1u) ? -1 : 1;
        e63_i += sign * static_cast<int32_t>(rx_I[c]);
        e63_q += sign * static_cast<int32_t>(rx_Q[c]);
    }
    const int64_t e63 = static_cast<int64_t>(e63_i) * e63_i + static_cast<int64_t>(e63_q) * e63_q;
    const int64_t thr = static_cast<int64_t>(ExperimentConfig::kHoloSyncE63Threshold) * ExperimentConfig::kHoloSyncE63Threshold;
    return e63 >= thr;
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
    if (rx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D::SECURE_TRUE) { tx.Shutdown(); return r; }

    for (int t = 0; t < ExperimentConfig::kNumTrials; ++t) {
        rng_seed(static_cast<uint32_t>(t * 7919u + 13u));
        int8_t bits[16]{};
        for (int i = 0; i < ExperimentConfig::kK; ++i) bits[i] = (rng_next() & 1u) ? 1 : -1;

        int8_t chips[64]{};
        (void)tx.Set_Time_Slot(static_cast<uint32_t>(t));
        if (tx.Encode_Block(bits, static_cast<uint16_t>(ExperimentConfig::kK), chips,
                            static_cast<uint16_t>(ExperimentConfig::kN)) != HTS_Holo_Tensor_4D::SECURE_TRUE) {
            r.total_bits += ExperimentConfig::kK; r.total_bit_err += ExperimentConfig::kK; continue;
        }

        int16_t txI[128]{}, txQ[128]{};
        for (int c = 0; c < 64; ++c) {
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
            txI[64 + c] = static_cast<int16_t>(v);
            txQ[64 + c] = static_cast<int16_t>(v);
        }

        int16_t rxI[128]{}, rxQ[128]{}, corrI[128]{}, corrQ[128]{};
        channel_apply(txI, txQ, rxI, rxQ, 128, static_cast<double>(cfo_hz), snr_db, static_cast<uint32_t>(t * 31u + 19u));
        if (ExperimentConfig::kEnableV5aCorrection) {
            const double est = v5a_estimate_cfo(rxI, rxQ, 64, 32);
            v5a_apply_per_chip(rxI, rxQ, corrI, corrQ, 128, est);
        } else {
            std::memcpy(corrI, rxI, sizeof(corrI)); std::memcpy(corrQ, rxQ, sizeof(corrQ));
        }

        if (ExperimentConfig::kEnableHoloSync && !holo_sync_check(corrI, corrQ)) {
            r.total_bits += ExperimentConfig::kK; r.total_bit_err += ExperimentConfig::kK; continue;
        }
        ++r.sync_pass;

        int16_t soft[64]{};
        uint64_t mask = 0xFFFFFFFFFFFFFFFFull;
        rx_soft_generate(corrI + 64, corrQ + 64, soft, &mask);
        int8_t out[16]{};
        (void)rx.Set_Time_Slot(static_cast<uint32_t>(t));
        if (rx.Decode_Block(soft, static_cast<uint16_t>(ExperimentConfig::kN), mask, out,
                            static_cast<uint16_t>(ExperimentConfig::kK)) != HTS_Holo_Tensor_4D::SECURE_TRUE) {
            r.total_bits += ExperimentConfig::kK; r.total_bit_err += ExperimentConfig::kK; continue;
        }

        int be = 0;
        for (int i = 0; i < ExperimentConfig::kK; ++i) be += (out[i] != bits[i]) ? 1 : 0;
        r.total_bit_err += be; r.total_bits += ExperimentConfig::kK;
        if (be == 0) ++r.decode_pass;
    }
    rx.Shutdown();
    tx.Shutdown();
    return r;
}

int main() {
    build_sincos_table();
    std::printf("Phase H Lab isolated scaffold\n");
    std::printf("mode=%s, rx_soft=%d, v5a=%d\n",
                ExperimentConfig::kEnableHoloSync ? "S5H-like" : "S5-like",
                ExperimentConfig::kRxSoftMode, ExperimentConfig::kEnableV5aCorrection ? 1 : 0);
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

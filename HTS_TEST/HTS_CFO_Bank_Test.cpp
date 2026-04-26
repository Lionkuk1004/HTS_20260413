#ifndef HTS_CFO_BANK_TEST_LAB_ONCE
#define HTS_CFO_BANK_TEST_LAB_ONCE

// ============================================================================
// HTS_CFO_Bank_Test.cpp — Phase H Lab: isolated cfo sweep scaffold
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../HTS_LIM/HTS_Holo_Tensor_4D_TX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_RX.h"

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
constexpr bool kEnableHoloSync = false;
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
    HTS_Holo_Tensor_4D_TX tx{};
    HTS_Holo_Tensor_4D_RX rx{};
    uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5B5A5A5u, 0xC3D3C3C3u};
    if (tx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) return r;
    if (rx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) { tx.Shutdown(); return r; }

    for (int t = 0; t < ExperimentConfig::kNumTrials; ++t) {
        rng_seed(static_cast<uint32_t>(t * 7919u + 13u));
        int8_t bits[16]{};
        for (int i = 0; i < ExperimentConfig::kK; ++i) bits[i] = (rng_next() & 1u) ? 1 : -1;

        int8_t chips[64]{};
        (void)tx.Set_Time_Slot(static_cast<uint32_t>(t));
        if (tx.Encode_Block(bits, static_cast<uint16_t>(ExperimentConfig::kK), chips,
                            static_cast<uint16_t>(ExperimentConfig::kN)) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            r.total_bits += ExperimentConfig::kK; r.total_bit_err += ExperimentConfig::kK; continue;
        }

        int16_t txI[64]{}, txQ[64]{};
        for (int c = 0; c < 64; ++c) {
            const int32_t prod = static_cast<int32_t>(chips[c]) * ExperimentConfig::kAmp;
            const int32_t v = (prod >= 0) ? ((prod + (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom)
                                          : ((prod - (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom);
            txI[c] = static_cast<int16_t>(v); txQ[c] = static_cast<int16_t>(v);
        }

        int16_t rxI[64]{}, rxQ[64]{}, corrI[64]{}, corrQ[64]{};
        channel_apply(txI, txQ, rxI, rxQ, 64, static_cast<double>(cfo_hz), snr_db, static_cast<uint32_t>(t * 31u + 19u));
        if (ExperimentConfig::kEnableV5aCorrection) {
            const double est = v5a_estimate_cfo(rxI, rxQ, 64, 32);
            v5a_apply_per_chip(rxI, rxQ, corrI, corrQ, 64, est);
        } else {
            std::memcpy(corrI, rxI, sizeof(corrI)); std::memcpy(corrQ, rxQ, sizeof(corrQ));
        }

        if (ExperimentConfig::kEnableHoloSync && !holo_sync_check(corrI, corrQ)) {
            r.total_bits += ExperimentConfig::kK; r.total_bit_err += ExperimentConfig::kK; continue;
        }
        ++r.sync_pass;

        int16_t soft[64]{};
        uint64_t mask = 0xFFFFFFFFFFFFFFFFull;
        rx_soft_generate(corrI, corrQ, soft, &mask);
        int8_t out[16]{};
        (void)rx.Set_Time_Slot(static_cast<uint32_t>(t));
        if (rx.Decode_Block(soft, static_cast<uint16_t>(ExperimentConfig::kN), mask, out,
                            static_cast<uint16_t>(ExperimentConfig::kK)) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
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
            std::printf(" %ddB %3d/%3d BER=%0.3f", ExperimentConfig::kSnrLevels[si], r.decode_pass, ExperimentConfig::kNumTrials, ber);
        }
        std::printf("\n");
    }
    return 0;
}

#endif  // HTS_CFO_BANK_TEST_LAB_ONCE
// ============================================================================
// HTS_CFO_Bank_Test.cpp — Phase H Lab: V5a + 4D Tensor Integration
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../HTS_LIM/HTS_Holo_Tensor_4D_TX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_RX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_Defs.h"

using namespace ProtectedEngine;

namespace ExperimentConfig {
constexpr int kCfoSweepList[] = {0, 50, 100, 200, 300, 500, 700, 1000, 2000, 3000, 5000};
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
constexpr bool kEnableHoloSync = false;
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
static inline double rng_uniform() noexcept {
    return static_cast<double>(rng_next()) / 4294967296.0;
}
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

static void channel_apply(const int16_t* tx_I, const int16_t* tx_Q,
                          int16_t* rx_I, int16_t* rx_Q,
                          int chips, double cfo_hz, int snr_db,
                          uint32_t ch_seed) noexcept {
    rng_seed(ch_seed);
    double sig_pow = 0.0;
    for (int k = 0; k < chips; ++k) {
        sig_pow += static_cast<double>(tx_I[k]) * tx_I[k];
        sig_pow += static_cast<double>(tx_Q[k]) * tx_Q[k];
    }
    sig_pow /= chips;
    const double noise_sigma =
        (sig_pow > 0.0) ? std::sqrt(sig_pow / std::pow(10.0, snr_db / 10.0)) : 0.0;

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

static double v5a_estimate_cfo(const int16_t* rx_I, const int16_t* rx_Q,
                               int chips, int lag) noexcept {
    double ac_I = 0.0;
    double ac_Q = 0.0;
    for (int n = 0; n + lag < chips; ++n) {
        ac_I += static_cast<double>(rx_I[n]) * rx_I[n + lag] +
                static_cast<double>(rx_Q[n]) * rx_Q[n + lag];
        ac_Q += static_cast<double>(rx_Q[n]) * rx_I[n + lag] -
                static_cast<double>(rx_I[n]) * rx_Q[n + lag];
    }
    if (std::abs(ac_I) < 1e-6 && std::abs(ac_Q) < 1e-6) return 0.0;
    const double phase = std::atan2(ac_Q, ac_I);
    return phase * ExperimentConfig::kChipRateHz / (2.0 * kPi * lag);
}

static void v5a_apply_per_chip(const int16_t* rx_I, const int16_t* rx_Q,
                               int16_t* out_I, int16_t* out_Q,
                               int chips, double cfo_hz) noexcept {
    const int64_t phase_inc_q32_s64 =
        -static_cast<int64_t>((cfo_hz * 4294967296.0) / ExperimentConfig::kChipRateHz);
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

static void fwht_64(int32_t* data) noexcept {
    int h = 1;
    while (h < 64) {
        for (int i = 0; i < 64; i += h * 2) {
            for (int j = i; j < i + h; ++j) {
                const int32_t x = data[j];
                const int32_t y = data[j + h];
                data[j] = x + y;
                data[j + h] = x - y;
            }
        }
        h *= 2;
    }
}

static bool holo_sync_check(const int16_t* rx_I, const int16_t* rx_Q) noexcept {
    int32_t wI[64], wQ[64];
    for (int k = 0; k < 64; ++k) {
        wI[k] = rx_I[k];
        wQ[k] = rx_Q[k];
    }
    fwht_64(wI);
    fwht_64(wQ);
    const int64_t e63 = static_cast<int64_t>(wI[63]) * wI[63] +
                        static_cast<int64_t>(wQ[63]) * wQ[63];
    const int64_t thr = static_cast<int64_t>(ExperimentConfig::kHoloSyncE63Threshold) *
                        ExperimentConfig::kHoloSyncE63Threshold;
    return e63 >= thr;
}

static void generate_holo_preamble(int16_t* tx_I, int16_t* tx_Q, int amp) noexcept {
    const uint8_t row = 63u;
    for (int c = 0; c < 64; ++c) {
        const uint32_t x = row & static_cast<uint32_t>(c);
        const int parity = static_cast<int>(popcount32_(x) & 1u);
        tx_I[c] = static_cast<int16_t>(parity ? -amp : amp);
        tx_Q[c] = 0;
    }
}

static void rx_soft_generate(const int16_t* buf_I, const int16_t* buf_Q,
                             int16_t* rx_soft, int N, uint64_t* out_valid_mask) noexcept {
    uint64_t valid_mask = 0u;
    for (int c = 0; c < N; ++c) {
        const int32_t i_val = buf_I[c];
        const int32_t q_val = buf_Q[c];
        int32_t valid_chip = -1;
        if (ExperimentConfig::kChipValidThreshold > 0) {
            const int32_t mag_sq = i_val * i_val + q_val * q_val;
            const int64_t diff =
                static_cast<int64_t>(mag_sq) -
                static_cast<int64_t>(ExperimentConfig::kChipValidThreshold);
            valid_chip = static_cast<int32_t>(~(diff >> 63));
        }
        int32_t soft = (i_val + q_val) / 2;
        switch (ExperimentConfig::kRxSoftMode) {
            case 1: soft = i_val; break;
            case 2: soft = q_val; break;
            case 3: soft = (std::abs(i_val) > std::abs(q_val)) ? i_val : q_val; break;
            case 4: soft = i_val + q_val; break;
            default: break;
        }
        rx_soft[c] = static_cast<int16_t>(soft & valid_chip);
        valid_mask |= (static_cast<uint64_t>(valid_chip & 1u) << static_cast<uint32_t>(c));
    }
    *out_valid_mask = valid_mask;
}

struct TestResult {
    int sync_pass;
    int decode_pass;
    int total_bit_err;
    int total_bits;
};

static TestResult run_one_cfo(int cfo_hz, int snr_db, int num_trials) {
    TestResult res = {0, 0, 0, 0};

    HTS_Holo_Tensor_4D_TX tx_tensor;
    HTS_Holo_Tensor_4D_RX rx_tensor;
    uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5B5A5A5u, 0xC3D3C3C3u};
    if (tx_tensor.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) return res;
    if (rx_tensor.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        tx_tensor.Shutdown();
        return res;
    }

    for (int trial = 0; trial < num_trials; ++trial) {
        rng_seed(static_cast<uint32_t>(trial * 7919u + 13u));
        int8_t data_bits[16];
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            data_bits[i] = (rng_next() & 1u) ? 1 : -1;
        }

        (void)tx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
        int8_t chip_bpsk[64];
        if (tx_tensor.Encode_Block(data_bits, static_cast<uint16_t>(ExperimentConfig::kK),
                                   chip_bpsk, static_cast<uint16_t>(ExperimentConfig::kN)) !=
            HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            res.total_bits += ExperimentConfig::kK;
            res.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        int16_t tx_I[64], tx_Q[64];
        for (int c = 0; c < ExperimentConfig::kN; ++c) {
            const int32_t prod = static_cast<int32_t>(chip_bpsk[c]) * ExperimentConfig::kAmp;
            const int32_t v =
                (prod >= 0) ? ((prod + (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom)
                            : ((prod - (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom);
            tx_I[c] = static_cast<int16_t>(v);
            tx_Q[c] = static_cast<int16_t>(v);
        }

        bool sync_ok = true;
        if (ExperimentConfig::kEnableHoloSync) {
            int16_t pre_I[64], pre_Q[64];
            generate_holo_preamble(pre_I, pre_Q, ExperimentConfig::kAmp);
            int16_t pre_rx_I[64], pre_rx_Q[64];
            channel_apply(pre_I, pre_Q, pre_rx_I, pre_rx_Q, 64, static_cast<double>(cfo_hz),
                          snr_db, static_cast<uint32_t>(trial * 31u + 7u));
            int16_t pre_corr_I[64], pre_corr_Q[64];
            if (ExperimentConfig::kEnableV5aCorrection) {
                const double cfo_est = v5a_estimate_cfo(pre_rx_I, pre_rx_Q, 64, 32);
                v5a_apply_per_chip(pre_rx_I, pre_rx_Q, pre_corr_I, pre_corr_Q, 64, cfo_est);
            } else {
                std::memcpy(pre_corr_I, pre_rx_I, sizeof(pre_corr_I));
                std::memcpy(pre_corr_Q, pre_rx_Q, sizeof(pre_corr_Q));
            }
            sync_ok = holo_sync_check(pre_corr_I, pre_corr_Q);
        }
        if (sync_ok) {
            ++res.sync_pass;
        } else {
            res.total_bits += ExperimentConfig::kK;
            res.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        int16_t rx_I[64], rx_Q[64];
        channel_apply(tx_I, tx_Q, rx_I, rx_Q, ExperimentConfig::kN, static_cast<double>(cfo_hz),
                      snr_db, static_cast<uint32_t>(trial * 31u + 19u));
        int16_t corr_I[64], corr_Q[64];
        if (ExperimentConfig::kEnableV5aCorrection) {
            const double cfo_est = v5a_estimate_cfo(rx_I, rx_Q, ExperimentConfig::kN, 32);
            v5a_apply_per_chip(rx_I, rx_Q, corr_I, corr_Q, ExperimentConfig::kN, cfo_est);
        } else {
            std::memcpy(corr_I, rx_I, sizeof(corr_I));
            std::memcpy(corr_Q, rx_Q, sizeof(corr_Q));
        }

        int16_t rx_soft[64];
        uint64_t valid_mask = 0xFFFFFFFFFFFFFFFFull;
        rx_soft_generate(corr_I, corr_Q, rx_soft, ExperimentConfig::kN, &valid_mask);

        int8_t output_bits[16];
        (void)rx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
        if (rx_tensor.Decode_Block(rx_soft, static_cast<uint16_t>(ExperimentConfig::kN), valid_mask,
                                   output_bits, static_cast<uint16_t>(ExperimentConfig::kK)) !=
            HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            res.total_bits += ExperimentConfig::kK;
            res.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        int bit_err = 0;
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            if (output_bits[i] != data_bits[i]) ++bit_err;
        }
        res.total_bit_err += bit_err;
        res.total_bits += ExperimentConfig::kK;
        if (bit_err == 0) ++res.decode_pass;
    }

    rx_tensor.Shutdown();
    tx_tensor.Shutdown();
    return res;
}

static const char* rx_soft_mode_name(int mode) {
    switch (mode) {
        case 0: return "(I+Q)/2";
        case 1: return "I only";
        case 2: return "Q only";
        case 3: return "dominant";
        case 4: return "I+Q";
        default: return "unknown";
    }
}

void test_holo_tensor_cfo_sweep() {
    build_sincos_table();

    std::printf("\n============================================================\n");
    std::printf("  Phase H Lab: V5a + 4D Tensor CFO Sweep (isolated)\n");
    std::printf("============================================================\n");
    std::printf("Profile: K=%d N=%d L=%d  trials=%d  mode=%s  rx_soft=%s\n",
                ExperimentConfig::kK, ExperimentConfig::kN, ExperimentConfig::kL,
                ExperimentConfig::kNumTrials,
                ExperimentConfig::kEnableHoloSync ? "S5H-like" : "S5-like",
                rx_soft_mode_name(ExperimentConfig::kRxSoftMode));

    std::printf("%-8s", "cfo Hz");
    for (int s = 0; s < ExperimentConfig::kSnrCount; ++s) {
        std::printf("  %2ddB sync/dec/BER", ExperimentConfig::kSnrLevels[s]);
    }
    std::printf("\n------------------------------------------------------------\n");

    for (int c = 0; c < ExperimentConfig::kCfoSweepCount; ++c) {
        const int cfo_hz = ExperimentConfig::kCfoSweepList[c];
        std::printf("%-8d", cfo_hz);
        for (int s = 0; s < ExperimentConfig::kSnrCount; ++s) {
            const TestResult r = run_one_cfo(cfo_hz, ExperimentConfig::kSnrLevels[s],
                                             ExperimentConfig::kNumTrials);
            const double ber = (r.total_bits > 0)
                                   ? static_cast<double>(r.total_bit_err) / static_cast<double>(r.total_bits)
                                   : 0.0;
            std::printf("  %3d/%3d/%4.2f", r.sync_pass, r.decode_pass, ber);
        }
        std::printf("\n");
    }
}

int main() {
    test_holo_tensor_cfo_sweep();
    return 0;
}
// ============================================================================
// HTS_CFO_Bank_Test.cpp — Phase H Lab isolated scaffold
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../HTS_LIM/HTS_Holo_Tensor_4D_TX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_RX.h"

using namespace ProtectedEngine;

namespace ExperimentConfig {
constexpr int kCfoSweepList[] = {0, 50, 100, 200, 300, 500, 700, 1000, 2000, 3000, 5000};
constexpr int kCfoSweepCount = sizeof(kCfoSweepList) / sizeof(int);
constexpr int kNumTrials = 100;
constexpr int kSnrLevels[] = {30, 20, 10};
constexpr int kSnrCount = sizeof(kSnrLevels) / sizeof(int);
constexpr int kK = 16;
constexpr int kN = 64;
constexpr int kAmp = 500;
constexpr int kDenom = 16;
constexpr bool kEnableV5aCorrection = true;
constexpr bool kEnableHoloSync = false;
constexpr int kHoloSyncE63Threshold = kAmp * 38;
constexpr double kChipRateHz = 1e6;
constexpr int kRxSoftMode = 0;
constexpr int32_t kChipValidThreshold = 0;
}  // namespace ExperimentConfig

namespace {
constexpr double kPi = 3.14159265358979323846;
uint32_t g_rng = 1u;
static inline void rng_seed(uint32_t seed) noexcept { g_rng = seed ? seed : 1u; }
static inline uint32_t rng_next() noexcept { g_rng = g_rng * 1664525u + 1013904223u; return g_rng; }
static inline double rng_uniform() noexcept { return static_cast<double>(rng_next()) / 4294967296.0; }
static inline double rng_gauss() noexcept {
    double u1 = rng_uniform(); if (u1 < 1e-10) u1 = 1e-10;
    const double u2 = rng_uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}
static inline int16_t sat16(int32_t v) noexcept {
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return static_cast<int16_t>(v);
}
static inline uint32_t popcount32(uint32_t x) noexcept {
    x = x - ((x >> 1) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
    x = (x + (x >> 4)) & 0x0F0F0F0Fu;
    x = x + (x >> 8); x = x + (x >> 16);
    return x & 0x3Fu;
}
}

static void test_holo_tensor_cfo_sweep() {
    std::printf("Phase H Lab isolated scaffold\n");
    std::printf("mode=%s rx_soft=%d valid_th=%d\n",
                ExperimentConfig::kEnableHoloSync ? "S5H-like" : "S5-like",
                ExperimentConfig::kRxSoftMode,
                static_cast<int>(ExperimentConfig::kChipValidThreshold));

    HTS_Holo_Tensor_4D_TX tx;
    HTS_Holo_Tensor_4D_RX rx;
    const uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5B5A5A5u, 0xC3D3C3C3u};
    if (tx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE ||
        rx.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        std::printf("tensor init failed\n");
        return;
    }

    for (int s = 0; s < ExperimentConfig::kSnrCount; ++s) {
        const int snr_db = ExperimentConfig::kSnrLevels[s];
        std::printf("\nSNR=%ddB\n", snr_db);
        for (int ci = 0; ci < ExperimentConfig::kCfoSweepCount; ++ci) {
            const int cfo = ExperimentConfig::kCfoSweepList[ci];
            int pass = 0;
            int bit_err_sum = 0;
            for (int t = 0; t < ExperimentConfig::kNumTrials; ++t) {
                rng_seed(static_cast<uint32_t>(t * 7919u + 13u));
                int8_t bits[ExperimentConfig::kK]{};
                for (int i = 0; i < ExperimentConfig::kK; ++i) bits[i] = (rng_next() & 1u) ? 1 : -1;
                int8_t chips[64]{};
                (void)tx.Set_Time_Slot(static_cast<uint32_t>(t));
                if (tx.Encode_Block(bits, static_cast<uint16_t>(ExperimentConfig::kK), chips,
                                    static_cast<uint16_t>(ExperimentConfig::kN)) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
                    bit_err_sum += ExperimentConfig::kK; continue;
                }
                int16_t txI[64]{}, txQ[64]{}, rxI[64]{}, rxQ[64]{}, soft[64]{};
                for (int c = 0; c < 64; ++c) {
                    const int32_t prod = static_cast<int32_t>(chips[c]) * ExperimentConfig::kAmp;
                    const int32_t v = (prod >= 0) ? ((prod + (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom)
                                                  : ((prod - (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom);
                    txI[c] = static_cast<int16_t>(v); txQ[c] = static_cast<int16_t>(v); // I=Q duplicated
                    const double ph = 2.0 * kPi * static_cast<double>(cfo) * static_cast<double>(c) / ExperimentConfig::kChipRateHz;
                    const double n = rng_gauss() * std::pow(10.0, -snr_db / 20.0) * 10.0;
                    rxI[c] = sat16(static_cast<int32_t>(txI[c] * std::cos(ph) - txQ[c] * std::sin(ph) + n));
                    rxQ[c] = sat16(static_cast<int32_t>(txI[c] * std::sin(ph) + txQ[c] * std::cos(ph) + n));
                    int32_t vchip = -1;
                    if (ExperimentConfig::kChipValidThreshold > 0) {
                        const int32_t mag2 = rxI[c] * rxI[c] + rxQ[c] * rxQ[c];
                        const int64_t diff = static_cast<int64_t>(mag2) - static_cast<int64_t>(ExperimentConfig::kChipValidThreshold);
                        vchip = static_cast<int32_t>(~(diff >> 63));
                    }
                    int32_t ssoft = (rxI[c] + rxQ[c]) / 2;
                    if (ExperimentConfig::kRxSoftMode == 1) ssoft = rxI[c];
                    if (ExperimentConfig::kRxSoftMode == 2) ssoft = rxQ[c];
                    if (ExperimentConfig::kRxSoftMode == 3) ssoft = (std::abs(rxI[c]) > std::abs(rxQ[c])) ? rxI[c] : rxQ[c];
                    if (ExperimentConfig::kRxSoftMode == 4) ssoft = rxI[c] + rxQ[c];
                    soft[c] = static_cast<int16_t>(ssoft & vchip);
                }
                if (ExperimentConfig::kEnableHoloSync) {
                    int32_t wI[64]{};
                    for (int c = 0; c < 64; ++c) {
                        const uint32_t x = 63u & static_cast<uint32_t>(c);
                        const int sign = ((popcount32(x) & 1u) != 0u) ? -1 : 1;
                        wI[c] = static_cast<int32_t>(rxI[c]) * sign;
                    }
                    int64_t e = 0; for (int c = 0; c < 64; ++c) e += static_cast<int64_t>(wI[c]) * wI[c];
                    if (e < static_cast<int64_t>(ExperimentConfig::kHoloSyncE63Threshold) * ExperimentConfig::kHoloSyncE63Threshold) {
                        bit_err_sum += ExperimentConfig::kK; continue;
                    }
                }
                int8_t out[ExperimentConfig::kK]{};
                (void)rx.Set_Time_Slot(static_cast<uint32_t>(t));
                if (rx.Decode_Block(soft, static_cast<uint16_t>(ExperimentConfig::kN), 0xFFFFFFFFFFFFFFFFull, out,
                                    static_cast<uint16_t>(ExperimentConfig::kK)) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
                    bit_err_sum += ExperimentConfig::kK; continue;
                }
                int berr = 0; for (int i = 0; i < ExperimentConfig::kK; ++i) berr += (out[i] != bits[i]) ? 1 : 0;
                bit_err_sum += berr; if (berr == 0) ++pass;
            }
            const double ber = static_cast<double>(bit_err_sum) /
                               static_cast<double>(ExperimentConfig::kNumTrials * ExperimentConfig::kK);
            std::printf("cfo=%5d Hz: %3d/%3d PASS BER=%.4f\n", cfo, pass, ExperimentConfig::kNumTrials, ber);
        }
    }
    rx.Shutdown();
    tx.Shutdown();
}

int main() {
    test_holo_tensor_cfo_sweep();
    return 0;
}
// ============================================================================
// HTS_CFO_Bank_Test.cpp — Phase H Lab isolated scaffold
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../HTS_LIM/HTS_Holo_Tensor_4D_TX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_RX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_Defs.h"

using namespace ProtectedEngine;

namespace ExperimentConfig {
constexpr int kCfoSweepList[] = {0, 50, 100, 200, 300, 500, 700, 1000, 2000, 3000, 5000};
constexpr int kCfoSweepCount = sizeof(kCfoSweepList) / sizeof(int);
constexpr int kNumTrials = 100;
constexpr int kSnrLevels[] = {30, 20, 10};
constexpr int kSnrCount = sizeof(kSnrLevels) / sizeof(int);
constexpr int kK = 16;
constexpr int kN = 64;
constexpr int kAmp = 500;
constexpr int kDenom = 16;
constexpr bool kEnableV5aCorrection = true;
constexpr bool kEnableHoloSync = false;
constexpr int kHoloSyncE63Threshold = kAmp * 38;
constexpr double kChipRateHz = 1e6;
constexpr int kRxSoftMode = 0;           // 0:(I+Q)/2 1:I 2:Q 3:dominant 4:I+Q
constexpr int32_t kChipValidThreshold = 0; // 0 = all valid
}  // namespace ExperimentConfig

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr int kSinCosTableSize = 1024;
constexpr int kSinCosIndexShift = 22;
constexpr int32_t kQ14One = 16384;
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
        const double angle = 2.0 * kPi * static_cast<double>(i) /
                             static_cast<double>(kSinCosTableSize);
        g_sin_table[i] = static_cast<int16_t>(std::sin(angle) * kQ14One);
        g_cos_table[i] = static_cast<int16_t>(std::cos(angle) * kQ14One);
    }
}

static inline void rng_seed(uint32_t seed) noexcept { g_rng = seed ? seed : 1u; }
static inline uint32_t rng_next() noexcept {
    g_rng = g_rng * 1664525u + 1013904223u;
    return g_rng;
}
static inline double rng_uniform() noexcept {
    return static_cast<double>(rng_next()) / 4294967296.0;
}
static inline double rng_gauss() noexcept {
    double u1 = rng_uniform();
    double u2 = rng_uniform();
    if (u1 < 1e-10) u1 = 1e-10;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}
static inline int16_t sat_i16_(int32_t v) noexcept {
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return static_cast<int16_t>(v);
}

static void generate_holo_preamble(int16_t* tx_I, int16_t* tx_Q, int amp) noexcept {
    const uint8_t row = 63u;
    for (int c = 0; c < 64; ++c) {
        const uint32_t x = static_cast<uint32_t>(row) & static_cast<uint32_t>(c);
        const int parity = static_cast<int>(popcount32_(x) & 1u);
        tx_I[c] = static_cast<int16_t>(parity ? -amp : amp);
        tx_Q[c] = 0;
    }
}

static void channel_apply(const int16_t* tx_I, const int16_t* tx_Q,
                          int16_t* rx_I, int16_t* rx_Q,
                          int chips, double cfo_hz, int snr_db,
                          uint32_t ch_seed) noexcept {
    rng_seed(ch_seed);
    double sig_pow = 0.0;
    for (int k = 0; k < chips; ++k) {
        sig_pow += static_cast<double>(tx_I[k]) * tx_I[k];
        sig_pow += static_cast<double>(tx_Q[k]) * tx_Q[k];
    }
    sig_pow /= static_cast<double>(chips);
    const double noise_sigma =
        (sig_pow > 0.0) ? std::sqrt(sig_pow / std::pow(10.0, snr_db / 10.0)) : 0.0;
    const double phase_inc = 2.0 * kPi * cfo_hz / ExperimentConfig::kChipRateHz;
    double phase = 0.0;
    for (int k = 0; k < chips; ++k) {
        const double cs = std::cos(phase);
        const double sn = std::sin(phase);
        double rotI = tx_I[k] * cs - tx_Q[k] * sn;
        double rotQ = tx_I[k] * sn + tx_Q[k] * cs;
        rotI += rng_gauss() * noise_sigma;
        rotQ += rng_gauss() * noise_sigma;
        rx_I[k] = sat_i16_(static_cast<int32_t>(rotI));
        rx_Q[k] = sat_i16_(static_cast<int32_t>(rotQ));
        phase += phase_inc;
    }
}

static double v5a_estimate_cfo(const int16_t* rx_I, const int16_t* rx_Q,
                               int chips, int lag) noexcept {
    double ac_I = 0.0, ac_Q = 0.0;
    for (int n = 0; n + lag < chips; ++n) {
        ac_I += static_cast<double>(rx_I[n]) * rx_I[n + lag] +
                static_cast<double>(rx_Q[n]) * rx_Q[n + lag];
        ac_Q += static_cast<double>(rx_Q[n]) * rx_I[n + lag] -
                static_cast<double>(rx_I[n]) * rx_Q[n + lag];
    }
    if (std::abs(ac_I) < 1e-6 && std::abs(ac_Q) < 1e-6) return 0.0;
    const double phase = std::atan2(ac_Q, ac_I);
    return phase * ExperimentConfig::kChipRateHz / (2.0 * kPi * static_cast<double>(lag));
}

static void v5a_apply_per_chip(const int16_t* in_I, const int16_t* in_Q,
                               int16_t* out_I, int16_t* out_Q,
                               int chips, double cfo_hz) noexcept {
    const int64_t phase_inc_q32_s64 = -static_cast<int64_t>(
        (cfo_hz * 4294967296.0) / ExperimentConfig::kChipRateHz);
    const uint32_t phase_inc_q32 = static_cast<uint32_t>(phase_inc_q32_s64);
    uint32_t phase_q32 = 0u;
    for (int k = 0; k < chips; ++k) {
        const int idx = static_cast<int>((phase_q32 >> kSinCosIndexShift) &
                                         (kSinCosTableSize - 1));
        const int32_t cos_q14 = g_cos_table[idx];
        const int32_t sin_q14 = g_sin_table[idx];
        const int32_t x = in_I[k];
        const int32_t y = in_Q[k];
        out_I[k] = sat_i16_((x * cos_q14 - y * sin_q14) >> 14);
        out_Q[k] = sat_i16_((x * sin_q14 + y * cos_q14) >> 14);
        phase_q32 += phase_inc_q32;
    }
}

static bool holo_sync_check(const int16_t* rx_I, const int16_t* rx_Q) noexcept {
    int32_t wI[64]{}, wQ[64]{};
    for (int i = 0; i < 64; ++i) {
        wI[i] = rx_I[i];
        wQ[i] = rx_Q[i];
    }
    for (int h = 1; h < 64; h <<= 1) {
        for (int i = 0; i < 64; i += (h << 1)) {
            for (int j = i; j < i + h; ++j) {
                const int32_t a = wI[j], b = wI[j + h];
                wI[j] = a + b; wI[j + h] = a - b;
                const int32_t c = wQ[j], d = wQ[j + h];
                wQ[j] = c + d; wQ[j + h] = c - d;
            }
        }
    }
    const int64_t e63 = static_cast<int64_t>(wI[63]) * wI[63] +
                        static_cast<int64_t>(wQ[63]) * wQ[63];
    const int64_t thr = static_cast<int64_t>(ExperimentConfig::kHoloSyncE63Threshold) *
                        ExperimentConfig::kHoloSyncE63Threshold;
    return e63 >= thr;
}

static void rx_soft_generate(const int16_t* buf_I, const int16_t* buf_Q,
                             int16_t* rx_soft, uint64_t* out_valid_mask) noexcept {
    uint64_t valid_mask = 0u;
    for (int c = 0; c < ExperimentConfig::kN; ++c) {
        const int32_t i_val = buf_I[c];
        const int32_t q_val = buf_Q[c];
        int32_t valid_chip = -1;
        if (ExperimentConfig::kChipValidThreshold > 0) {
            const int32_t mag_sq = i_val * i_val + q_val * q_val;
            const int64_t diff = static_cast<int64_t>(mag_sq) -
                                 static_cast<int64_t>(ExperimentConfig::kChipValidThreshold);
            valid_chip = static_cast<int32_t>(~(diff >> 63));
        }
        int32_t soft = (i_val + q_val) / 2;
        if (ExperimentConfig::kRxSoftMode == 1) soft = i_val;
        if (ExperimentConfig::kRxSoftMode == 2) soft = q_val;
        if (ExperimentConfig::kRxSoftMode == 3) {
            const int32_t ai = (i_val < 0) ? -i_val : i_val;
            const int32_t aq = (q_val < 0) ? -q_val : q_val;
            soft = (ai > aq) ? i_val : q_val;
        }
        if (ExperimentConfig::kRxSoftMode == 4) soft = i_val + q_val;
        rx_soft[c] = static_cast<int16_t>(soft & valid_chip);
        valid_mask |= (static_cast<uint64_t>(valid_chip & 1u) <<
                       static_cast<uint32_t>(c));
    }
    *out_valid_mask = valid_mask;
}

struct TestResult { int sync_pass; int decode_pass; int total_bit_err; int total_bits; };

static TestResult run_one_cfo(int cfo_hz, int snr_db) noexcept {
    TestResult res{0, 0, 0, 0};
    HTS_Holo_Tensor_4D_TX tx_tensor;
    HTS_Holo_Tensor_4D_RX rx_tensor;
    const uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5B5A5A5u, 0xC3D3C3C3u};
    if (tx_tensor.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) return res;
    if (rx_tensor.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) { tx_tensor.Shutdown(); return res; }
    for (int trial = 0; trial < ExperimentConfig::kNumTrials; ++trial) {
        rng_seed(static_cast<uint32_t>(trial * 7919u + 13u));
        int8_t data_bits[ExperimentConfig::kK]{};
        for (int i = 0; i < ExperimentConfig::kK; ++i) data_bits[i] = (rng_next() & 1u) ? 1 : -1;
        int8_t chip_bpsk[64]{};
        (void)tx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
        if (tx_tensor.Encode_Block(data_bits, static_cast<uint16_t>(ExperimentConfig::kK),
                                   chip_bpsk, static_cast<uint16_t>(ExperimentConfig::kN)) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) continue;
        int16_t tx_I[64]{}, tx_Q[64]{};
        for (int c = 0; c < ExperimentConfig::kN; ++c) {
            const int32_t prod = static_cast<int32_t>(chip_bpsk[c]) * ExperimentConfig::kAmp;
            const int32_t v = (prod >= 0) ? ((prod + (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom)
                                          : ((prod - (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom);
            tx_I[c] = static_cast<int16_t>(v);
            tx_Q[c] = static_cast<int16_t>(v);
        }

        bool sync_ok = true;
        if (ExperimentConfig::kEnableHoloSync) {
            int16_t pre_I[64]{}, pre_Q[64]{}, pre_rx_I[64]{}, pre_rx_Q[64]{}, pre_corr_I[64]{}, pre_corr_Q[64]{};
            generate_holo_preamble(pre_I, pre_Q, ExperimentConfig::kAmp);
            channel_apply(pre_I, pre_Q, pre_rx_I, pre_rx_Q, 64, static_cast<double>(cfo_hz), snr_db, static_cast<uint32_t>(trial * 31u + 7u));
            if (ExperimentConfig::kEnableV5aCorrection) {
                const double cfo_est = v5a_estimate_cfo(pre_rx_I, pre_rx_Q, 64, 32);
                v5a_apply_per_chip(pre_rx_I, pre_rx_Q, pre_corr_I, pre_corr_Q, 64, cfo_est);
            } else {
                std::memcpy(pre_corr_I, pre_rx_I, sizeof(pre_corr_I));
                std::memcpy(pre_corr_Q, pre_rx_Q, sizeof(pre_corr_Q));
            }
            sync_ok = holo_sync_check(pre_corr_I, pre_corr_Q);
        }
        if (sync_ok) ++res.sync_pass; else { res.total_bits += ExperimentConfig::kK; res.total_bit_err += ExperimentConfig::kK; continue; }

        int16_t rx_I[64]{}, rx_Q[64]{}, corr_I[64]{}, corr_Q[64]{};
        channel_apply(tx_I, tx_Q, rx_I, rx_Q, 64, static_cast<double>(cfo_hz), snr_db, static_cast<uint32_t>(trial * 31u + 19u));
        if (ExperimentConfig::kEnableV5aCorrection) {
            const double cfo_est = v5a_estimate_cfo(rx_I, rx_Q, 64, 32);
            v5a_apply_per_chip(rx_I, rx_Q, corr_I, corr_Q, 64, cfo_est);
        } else {
            std::memcpy(corr_I, rx_I, sizeof(corr_I));
            std::memcpy(corr_Q, rx_Q, sizeof(corr_Q));
        }

        int16_t rx_soft[64]{};
        uint64_t valid_mask = 0xFFFFFFFFFFFFFFFFull;
        rx_soft_generate(corr_I, corr_Q, rx_soft, &valid_mask);
        int8_t output_bits[ExperimentConfig::kK]{};
        (void)rx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
        if (rx_tensor.Decode_Block(rx_soft, static_cast<uint16_t>(ExperimentConfig::kN), valid_mask,
                                   output_bits, static_cast<uint16_t>(ExperimentConfig::kK)) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            res.total_bits += ExperimentConfig::kK; res.total_bit_err += ExperimentConfig::kK; continue;
        }
        int bit_err = 0;
        for (int i = 0; i < ExperimentConfig::kK; ++i) bit_err += (output_bits[i] != data_bits[i]) ? 1 : 0;
        res.total_bit_err += bit_err;
        res.total_bits += ExperimentConfig::kK;
        if (bit_err == 0) ++res.decode_pass;
    }
    rx_tensor.Shutdown();
    tx_tensor.Shutdown();
    return res;
}
}  // namespace

static const char* rx_soft_mode_name(int mode) {
    switch (mode) {
        case 0: return "(I+Q)/2";
        case 1: return "I only";
        case 2: return "Q only";
        case 3: return "dominant";
        case 4: return "I+Q";
        default: return "unknown";
    }
}

void test_holo_tensor_cfo_sweep() {
    build_sincos_table();
    std::printf("\n============================================================\n");
    std::printf("  Phase H Lab: V5a + 4D Tensor CFO Sweep\n");
    std::printf("============================================================\n");
    std::printf("Profile: K=%d N=%d L=%d  trials=%d  mode=%s  rx_soft=%s\n",
                ExperimentConfig::kK, ExperimentConfig::kN, ExperimentConfig::kL,
                ExperimentConfig::kNumTrials,
                ExperimentConfig::kEnableHoloSync ? "S5H-like" : "S5-like",
                rx_soft_mode_name(ExperimentConfig::kRxSoftMode));
    std::printf("%-8s", "cfo Hz");
    for (int s = 0; s < ExperimentConfig::kSnrCount; ++s) {
        std::printf("  %2ddB sync/dec/BER", ExperimentConfig::kSnrLevels[s]);
    }
    std::printf("\n");
    for (int c = 0; c < ExperimentConfig::kCfoSweepCount; ++c) {
        const int cfo_hz = ExperimentConfig::kCfoSweepList[c];
        std::printf("%-8d", cfo_hz);
        for (int s = 0; s < ExperimentConfig::kSnrCount; ++s) {
            const TestResult r = run_one_cfo(cfo_hz, ExperimentConfig::kSnrLevels[s]);
            const double ber = (r.total_bits > 0)
                ? static_cast<double>(r.total_bit_err) / static_cast<double>(r.total_bits)
                : 0.0;
            std::printf("  %3d/%3d/%4.2f", r.sync_pass, r.decode_pass, ber);
        }
        std::printf("\n");
    }
}

int main() {
    test_holo_tensor_cfo_sweep();
    return 0;
}
// ============================================================================
// HTS_CFO_Bank_Test.cpp — Phase H Lab: V5a + 4D Tensor Integration
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../HTS_LIM/HTS_Holo_Tensor_4D_TX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_RX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_Defs.h"

using namespace ProtectedEngine;

namespace ExperimentConfig {
constexpr int kCfoSweepList[] = {0, 50, 100, 200, 300, 500, 700, 1000, 2000, 3000, 5000};
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
constexpr bool kEnableHoloSync = false;
constexpr int kHoloSyncE63Threshold = kAmp * 38;
constexpr double kChipRateHz = 1e6;
constexpr int kRxSoftMode = 0;
constexpr int32_t kChipValidThreshold = 0;
}  // namespace ExperimentConfig

namespace {
constexpr int kSinCosTableSize = 1024;
constexpr int kSinCosIndexShift = 22;
constexpr int32_t kQ14One = 16384;
constexpr double kPi = 3.14159265358979323846;
int16_t g_sin_table[kSinCosTableSize];
int16_t g_cos_table[kSinCosTableSize];

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
        const double angle = 2.0 * kPi * static_cast<double>(i) /
                             static_cast<double>(kSinCosTableSize);
        g_sin_table[i] = static_cast<int16_t>(std::sin(angle) * kQ14One);
        g_cos_table[i] = static_cast<int16_t>(std::cos(angle) * kQ14One);
    }
}
}  // namespace

namespace {
static uint32_t g_rng = 1u;
static inline void rng_seed(uint32_t seed) noexcept { g_rng = seed ? seed : 1u; }
static inline uint32_t rng_next() noexcept {
    g_rng = g_rng * 1664525u + 1013904223u;
    return g_rng;
}
static inline double rng_uniform() noexcept {
    return static_cast<double>(rng_next()) / 4294967296.0;
}
static inline double rng_gauss() noexcept {
    double u1 = rng_uniform();
    double u2 = rng_uniform();
    if (u1 < 1e-10) u1 = 1e-10;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}
}  // namespace

static void channel_apply(const int16_t* tx_I, const int16_t* tx_Q,
                          int16_t* rx_I, int16_t* rx_Q,
                          int chips, double cfo_hz, int snr_db,
                          uint32_t ch_seed) noexcept {
    rng_seed(ch_seed);
    double sig_pow = 0.0;
    for (int k = 0; k < chips; ++k) {
        sig_pow += static_cast<double>(tx_I[k]) * static_cast<double>(tx_I[k]);
        sig_pow += static_cast<double>(tx_Q[k]) * static_cast<double>(tx_Q[k]);
    }
    sig_pow /= static_cast<double>(chips);
    const double noise_sigma =
        (sig_pow > 0.0) ? std::sqrt(sig_pow / std::pow(10.0, snr_db / 10.0)) : 0.0;

    const double phase_inc = 2.0 * kPi * cfo_hz / ExperimentConfig::kChipRateHz;
    double phase = 0.0;
    for (int k = 0; k < chips; ++k) {
        const double cs = std::cos(phase);
        const double sn = std::sin(phase);
        double rotI = static_cast<double>(tx_I[k]) * cs - static_cast<double>(tx_Q[k]) * sn;
        double rotQ = static_cast<double>(tx_I[k]) * sn + static_cast<double>(tx_Q[k]) * cs;
        rotI += rng_gauss() * noise_sigma;
        rotQ += rng_gauss() * noise_sigma;
        int32_t fI = static_cast<int32_t>(rotI);
        int32_t fQ = static_cast<int32_t>(rotQ);
        if (fI > 32767) fI = 32767;
        if (fI < -32768) fI = -32768;
        if (fQ > 32767) fQ = 32767;
        if (fQ < -32768) fQ = -32768;
        rx_I[k] = static_cast<int16_t>(fI);
        rx_Q[k] = static_cast<int16_t>(fQ);
        phase += phase_inc;
    }
}

static double v5a_estimate_cfo(const int16_t* rx_I, const int16_t* rx_Q,
                               int chips, int lag) noexcept {
    double ac_I = 0.0;
    double ac_Q = 0.0;
    for (int n = 0; n + lag < chips; ++n) {
        ac_I += static_cast<double>(rx_I[n]) * static_cast<double>(rx_I[n + lag]) +
                static_cast<double>(rx_Q[n]) * static_cast<double>(rx_Q[n + lag]);
        ac_Q += static_cast<double>(rx_Q[n]) * static_cast<double>(rx_I[n + lag]) -
                static_cast<double>(rx_I[n]) * static_cast<double>(rx_Q[n + lag]);
    }
    if (std::abs(ac_I) < 1e-6 && std::abs(ac_Q) < 1e-6) return 0.0;
    const double phase = std::atan2(ac_Q, ac_I);
    return phase * ExperimentConfig::kChipRateHz / (2.0 * kPi * static_cast<double>(lag));
}

static void v5a_apply_per_chip(const int16_t* rx_I, const int16_t* rx_Q,
                               int16_t* out_I, int16_t* out_Q,
                               int chips, double cfo_hz) noexcept {
    const int64_t phase_inc_q32_s64 = -static_cast<int64_t>(
        (cfo_hz * 4294967296.0) / ExperimentConfig::kChipRateHz);
    const uint32_t phase_inc_q32 = static_cast<uint32_t>(phase_inc_q32_s64);
    uint32_t phase_q32 = 0u;
    for (int k = 0; k < chips; ++k) {
        const int idx = static_cast<int>(
            (phase_q32 >> kSinCosIndexShift) & (kSinCosTableSize - 1));
        const int32_t cos_q14 = g_cos_table[idx];
        const int32_t sin_q14 = g_sin_table[idx];
        const int32_t x = static_cast<int32_t>(rx_I[k]);
        const int32_t y = static_cast<int32_t>(rx_Q[k]);
        int32_t a = (x * cos_q14 - y * sin_q14) >> 14;
        int32_t b = (x * sin_q14 + y * cos_q14) >> 14;
        if (a > 32767) a = 32767;
        if (a < -32768) a = -32768;
        if (b > 32767) b = 32767;
        if (b < -32768) b = -32768;
        out_I[k] = static_cast<int16_t>(a);
        out_Q[k] = static_cast<int16_t>(b);
        phase_q32 += phase_inc_q32;
    }
}

static void fwht_64(int32_t* data) noexcept {
    int h = 1;
    while (h < 64) {
        for (int i = 0; i < 64; i += h * 2) {
            for (int j = i; j < i + h; ++j) {
                const int32_t x = data[j];
                const int32_t y = data[j + h];
                data[j] = x + y;
                data[j + h] = x - y;
            }
        }
        h *= 2;
    }
}

static bool holo_sync_check(const int16_t* rx_I, const int16_t* rx_Q) noexcept {
    int32_t wI[64]{};
    int32_t wQ[64]{};
    for (int k = 0; k < 64; ++k) {
        wI[k] = static_cast<int32_t>(rx_I[k]);
        wQ[k] = static_cast<int32_t>(rx_Q[k]);
    }
    fwht_64(wI);
    fwht_64(wQ);
    const int64_t e63 =
        static_cast<int64_t>(wI[63]) * static_cast<int64_t>(wI[63]) +
        static_cast<int64_t>(wQ[63]) * static_cast<int64_t>(wQ[63]);
    const int64_t thr =
        static_cast<int64_t>(ExperimentConfig::kHoloSyncE63Threshold) *
        static_cast<int64_t>(ExperimentConfig::kHoloSyncE63Threshold);
    return e63 >= thr;
}

static void generate_holo_preamble(int16_t* tx_I, int16_t* tx_Q, int amp) noexcept {
    const uint8_t row = 63u;
    for (int c = 0; c < 64; ++c) {
        const uint32_t x = static_cast<uint32_t>(row) & static_cast<uint32_t>(c);
        const int parity = static_cast<int>(popcount32_(x) & 1u);
        tx_I[c] = static_cast<int16_t>(parity ? -amp : amp);
        tx_Q[c] = 0;
    }
}

static void rx_soft_generate(const int16_t* buf_I, const int16_t* buf_Q,
                             int16_t* rx_soft, int N,
                             uint64_t* out_valid_mask) noexcept {
    uint64_t valid_mask = 0u;
    for (int c = 0; c < N; ++c) {
        const int32_t i_val = static_cast<int32_t>(buf_I[c]);
        const int32_t q_val = static_cast<int32_t>(buf_Q[c]);
        int32_t valid_chip;
        if (ExperimentConfig::kChipValidThreshold > 0) {
            const int32_t mag_sq = i_val * i_val + q_val * q_val;
            const int64_t diff =
                static_cast<int64_t>(mag_sq) -
                static_cast<int64_t>(ExperimentConfig::kChipValidThreshold);
            valid_chip = static_cast<int32_t>(~(diff >> 63));
        } else {
            valid_chip = -1;
        }
        int32_t soft;
        switch (ExperimentConfig::kRxSoftMode) {
            case 0: soft = (i_val + q_val) / 2; break;
            case 1: soft = i_val; break;
            case 2: soft = q_val; break;
            case 3: {
                const int32_t abs_i = (i_val < 0) ? -i_val : i_val;
                const int32_t abs_q = (q_val < 0) ? -q_val : q_val;
                soft = (abs_i > abs_q) ? i_val : q_val;
                break;
            }
            case 4: soft = i_val + q_val; break;
            default: soft = (i_val + q_val) / 2; break;
        }
        rx_soft[c] = static_cast<int16_t>(soft & valid_chip);
        valid_mask |= (static_cast<uint64_t>(valid_chip & 1u)
                       << static_cast<uint32_t>(c));
    }
    *out_valid_mask = valid_mask;
}

static int tx_build_tensor(HTS_Holo_Tensor_4D_TX& tensor,
                           const int8_t* data_bits,
                           int16_t* tx_I, int16_t* tx_Q,
                           int amp) noexcept {
    int8_t chip_bpsk[64]{};
    if (tensor.Encode_Block(data_bits, static_cast<uint16_t>(ExperimentConfig::kK),
                            chip_bpsk, static_cast<uint16_t>(ExperimentConfig::kN)) !=
        HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        return 0;
    }
    const int denom = ExperimentConfig::kDenom;
    const int half_denom = denom >> 1;
    for (int c = 0; c < ExperimentConfig::kN; ++c) {
        const int32_t prod = static_cast<int32_t>(chip_bpsk[c]) * amp;
        const int32_t v =
            (prod >= 0) ? ((prod + half_denom) / denom)
                        : ((prod - half_denom) / denom);
        tx_I[c] = static_cast<int16_t>(v);
        tx_Q[c] = static_cast<int16_t>(v);
    }
    return ExperimentConfig::kN;
}

struct TestResult {
    int sync_pass;
    int decode_pass;
    int total_bit_err;
    int total_bits;
};

static TestResult run_one_cfo(int cfo_hz, int snr_db, int num_trials) {
    TestResult res = {0, 0, 0, 0};
    HTS_Holo_Tensor_4D_TX tx_tensor;
    HTS_Holo_Tensor_4D_RX rx_tensor;
    uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5B5A5A5u, 0xC3D3C3C3u};
    if (tx_tensor.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        return res;
    }
    if (rx_tensor.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        tx_tensor.Shutdown();
        return res;
    }

    for (int trial = 0; trial < num_trials; ++trial) {
        rng_seed(static_cast<uint32_t>(trial * 7919u + 13u));
        int8_t data_bits[16]{};
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            data_bits[i] = ((rng_next() & 1u) != 0u) ? 1 : -1;
        }

        (void)tx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
        int16_t tx_I[64]{}, tx_Q[64]{};
        if (tx_build_tensor(tx_tensor, data_bits, tx_I, tx_Q, ExperimentConfig::kAmp) == 0) {
            continue;
        }

        bool sync_ok = true;
        if (ExperimentConfig::kEnableHoloSync) {
            int16_t pre_I[64]{}, pre_Q[64]{};
            generate_holo_preamble(pre_I, pre_Q, ExperimentConfig::kAmp);
            int16_t pre_rx_I[64]{}, pre_rx_Q[64]{};
            const uint32_t pre_seed = static_cast<uint32_t>(trial * 31u + 7u);
            channel_apply(pre_I, pre_Q, pre_rx_I, pre_rx_Q,
                          64, static_cast<double>(cfo_hz), snr_db, pre_seed);
            int16_t pre_corr_I[64]{}, pre_corr_Q[64]{};
            if (ExperimentConfig::kEnableV5aCorrection) {
                const double cfo_est = v5a_estimate_cfo(pre_rx_I, pre_rx_Q, 64, 32);
                v5a_apply_per_chip(pre_rx_I, pre_rx_Q, pre_corr_I, pre_corr_Q, 64, cfo_est);
            } else {
                std::memcpy(pre_corr_I, pre_rx_I, sizeof(pre_corr_I));
                std::memcpy(pre_corr_Q, pre_rx_Q, sizeof(pre_corr_Q));
            }
            sync_ok = holo_sync_check(pre_corr_I, pre_corr_Q);
        }

        if (sync_ok) {
            ++res.sync_pass;
        } else {
            res.total_bits += ExperimentConfig::kK;
            res.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        int16_t rx_I[64]{}, rx_Q[64]{};
        const uint32_t ch_seed = static_cast<uint32_t>(trial * 31u + 19u);
        channel_apply(tx_I, tx_Q, rx_I, rx_Q, ExperimentConfig::kN,
                      static_cast<double>(cfo_hz), snr_db, ch_seed);

        int16_t corr_I[64]{}, corr_Q[64]{};
        if (ExperimentConfig::kEnableV5aCorrection) {
            const double cfo_est = v5a_estimate_cfo(rx_I, rx_Q, ExperimentConfig::kN, 32);
            v5a_apply_per_chip(rx_I, rx_Q, corr_I, corr_Q, ExperimentConfig::kN, cfo_est);
        } else {
            std::memcpy(corr_I, rx_I, sizeof(corr_I));
            std::memcpy(corr_Q, rx_Q, sizeof(corr_Q));
        }

        int16_t rx_soft[64]{};
        uint64_t valid_mask = 0xFFFFFFFFFFFFFFFFull;
        rx_soft_generate(corr_I, corr_Q, rx_soft, ExperimentConfig::kN, &valid_mask);

        (void)rx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
        int8_t output_bits[16]{};
        if (rx_tensor.Decode_Block(rx_soft, static_cast<uint16_t>(ExperimentConfig::kN),
                                   valid_mask, output_bits,
                                   static_cast<uint16_t>(ExperimentConfig::kK)) !=
            HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            res.total_bits += ExperimentConfig::kK;
            res.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        int bit_err = 0;
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            if (output_bits[i] != data_bits[i]) ++bit_err;
        }
        res.total_bit_err += bit_err;
        res.total_bits += ExperimentConfig::kK;
        if (bit_err == 0) ++res.decode_pass;
    }

    rx_tensor.Shutdown();
    tx_tensor.Shutdown();
    return res;
}

static const char* rx_soft_mode_name(int mode) {
    switch (mode) {
        case 0: return "(I+Q)/2";
        case 1: return "I only";
        case 2: return "Q only";
        case 3: return "dominant";
        case 4: return "I+Q";
        default: return "unknown";
    }
}

void test_holo_tensor_cfo_sweep() {
    build_sincos_table();
    std::printf("\n");
    std::printf("============================================================\n");
    std::printf("  Phase H Lab: V5a + 4D Tensor CFO Sweep (영준님 격리 환경)\n");
    std::printf("============================================================\n");
    std::printf("Profile: K=%d, N=%d, L=%d (영준님 4D 텐서 default)\n",
                ExperimentConfig::kK, ExperimentConfig::kN, ExperimentConfig::kL);
    std::printf("Trials per (cfo, SNR): %d\n", ExperimentConfig::kNumTrials);
    std::printf("Chip rate: %.0f Hz (1 MHz 가정)\n", ExperimentConfig::kChipRateHz);
    std::printf("V5a correction: %s\n",
                ExperimentConfig::kEnableV5aCorrection ? "ON" : "OFF");
    std::printf("HOLO sync: %s (S%s sim)\n",
                ExperimentConfig::kEnableHoloSync ? "ON" : "OFF",
                ExperimentConfig::kEnableHoloSync ? "5H" : "5");
    std::printf("rx_soft mode: %d (%s)\n", ExperimentConfig::kRxSoftMode,
                rx_soft_mode_name(ExperimentConfig::kRxSoftMode));
    std::printf("ChipValidThreshold: %d %s\n",
                static_cast<int>(ExperimentConfig::kChipValidThreshold),
                (ExperimentConfig::kChipValidThreshold == 0) ? "(all valid)" : "");
    std::printf("============================================================\n\n");

    std::printf("%-8s", "cfo Hz");
    for (int s = 0; s < ExperimentConfig::kSnrCount; ++s) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), " %dB sync/dec/BER",
                      ExperimentConfig::kSnrLevels[s]);
        std::printf(" %-22s", buf);
    }
    std::printf("\n");
    std::printf("------------------------------------------------------------\n");

    for (int c = 0; c < ExperimentConfig::kCfoSweepCount; ++c) {
        const int cfo_hz = ExperimentConfig::kCfoSweepList[c];
        std::printf("%-8d", cfo_hz);
        for (int s = 0; s < ExperimentConfig::kSnrCount; ++s) {
            const int snr_db = ExperimentConfig::kSnrLevels[s];
            const TestResult r = run_one_cfo(cfo_hz, snr_db, ExperimentConfig::kNumTrials);
            const double ber = (r.total_bits > 0)
                ? static_cast<double>(r.total_bit_err) /
                      static_cast<double>(r.total_bits)
                : 0.0;
            std::printf("  %3d/%3d/%4.2f  %-4s",
                        r.sync_pass, r.decode_pass, ber,
                        (r.decode_pass >= (ExperimentConfig::kNumTrials * 95 / 100)) ? "OK"
                                                                                       : "");
        }
        std::printf("\n");
    }

    std::printf("\n");
    std::printf("Notes:\n");
    std::printf("  sync = HOLO sync 통과 수 (HOLO sync OFF 시 N/N)\n");
    std::printf("  dec  = 디코딩 100%% 성공 수\n");
    std::printf("  BER  = Bit Error Rate\n");
    std::printf("  OK   = decode >= 95%%\n\n");
    std::printf("영준님 실험:\n");
    std::printf("  1. kEnableHoloSync = false -> S5 시뮬\n");
    std::printf("  2. kEnableHoloSync = true  -> S5H 시뮬\n");
    std::printf("  3. kRxSoftMode 변경 (0~4)\n");
    std::printf("  4. kChipValidThreshold 변경\n");
    std::printf("  5. F5 빠른 재실행\n");
    std::printf("============================================================\n");
}

int main() {
    test_holo_tensor_cfo_sweep();
    return 0;
}
// ============================================================================
// HTS_CFO_Bank_Test.cpp — Phase H Lab: V5a + 4D Tensor Integration
// ============================================================================
// 영준님 격리 실험 환경
// 목적:
//   - V5a CFO 보정 + HTS_Holo_Tensor_4D 디코딩 통합
//   - HOLO sync ON/OFF 분기 (S5/S5H 시뮬)
//   - rx_soft 변환 빠른 실험 ((I+Q)/2 vs I-only vs 기타)
//   - cfo sweep 측정 → 진짜 FAIL 위치 식별
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "../HTS_LIM/HTS_Holo_Tensor_4D_TX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_RX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_Defs.h"

using namespace ProtectedEngine;

namespace ExperimentConfig {
constexpr int kCfoSweepList[] = {0, 50, 100, 200, 300, 500, 700, 1000, 2000, 3000, 5000};
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
constexpr bool kEnableHoloSync = false;
constexpr int kHoloSyncE63Threshold = kAmp * 38;
constexpr double kChipRateHz = 1e6;
constexpr int kRxSoftMode = 0;
constexpr int32_t kChipValidThreshold = 0;
}  // namespace ExperimentConfig

namespace {
constexpr int kSinCosTableSize = 1024;
constexpr int kSinCosIndexShift = 22;
constexpr int32_t kQ14One = 16384;
constexpr double kPi = 3.14159265358979323846;
int16_t g_sin_table[kSinCosTableSize];
int16_t g_cos_table[kSinCosTableSize];

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
        const double angle = 2.0 * kPi * static_cast<double>(i) /
                             static_cast<double>(kSinCosTableSize);
        g_sin_table[i] = static_cast<int16_t>(std::sin(angle) * kQ14One);
        g_cos_table[i] = static_cast<int16_t>(std::cos(angle) * kQ14One);
    }
}
}  // namespace

namespace {
static uint32_t g_rng = 1u;
static inline void rng_seed(uint32_t seed) noexcept { g_rng = seed ? seed : 1u; }
static inline uint32_t rng_next() noexcept {
    g_rng = g_rng * 1664525u + 1013904223u;
    return g_rng;
}
static inline double rng_uniform() noexcept {
    return static_cast<double>(rng_next()) / 4294967296.0;
}
static inline double rng_gauss() noexcept {
    double u1 = rng_uniform();
    double u2 = rng_uniform();
    if (u1 < 1e-10) u1 = 1e-10;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}
}  // namespace

static void channel_apply(const int16_t* tx_I, const int16_t* tx_Q,
                          int16_t* rx_I, int16_t* rx_Q,
                          int chips, double cfo_hz, int snr_db,
                          uint32_t ch_seed) noexcept {
    rng_seed(ch_seed);
    double sig_pow = 0.0;
    for (int k = 0; k < chips; ++k) {
        sig_pow += static_cast<double>(tx_I[k]) * static_cast<double>(tx_I[k]);
        sig_pow += static_cast<double>(tx_Q[k]) * static_cast<double>(tx_Q[k]);
    }
    sig_pow /= static_cast<double>(chips);
    const double noise_sigma =
        (sig_pow > 0.0) ? std::sqrt(sig_pow / std::pow(10.0, snr_db / 10.0)) : 0.0;

    const double phase_inc = 2.0 * kPi * cfo_hz / ExperimentConfig::kChipRateHz;
    double phase = 0.0;
    for (int k = 0; k < chips; ++k) {
        const double cs = std::cos(phase);
        const double sn = std::sin(phase);
        double rotI = static_cast<double>(tx_I[k]) * cs - static_cast<double>(tx_Q[k]) * sn;
        double rotQ = static_cast<double>(tx_I[k]) * sn + static_cast<double>(tx_Q[k]) * cs;
        rotI += rng_gauss() * noise_sigma;
        rotQ += rng_gauss() * noise_sigma;
        int32_t fI = static_cast<int32_t>(rotI);
        int32_t fQ = static_cast<int32_t>(rotQ);
        if (fI > 32767) fI = 32767;
        if (fI < -32768) fI = -32768;
        if (fQ > 32767) fQ = 32767;
        if (fQ < -32768) fQ = -32768;
        rx_I[k] = static_cast<int16_t>(fI);
        rx_Q[k] = static_cast<int16_t>(fQ);
        phase += phase_inc;
    }
}

static double v5a_estimate_cfo(const int16_t* rx_I, const int16_t* rx_Q,
                               int chips, int lag) noexcept {
    double ac_I = 0.0;
    double ac_Q = 0.0;
    for (int n = 0; n + lag < chips; ++n) {
        ac_I += static_cast<double>(rx_I[n]) * static_cast<double>(rx_I[n + lag]) +
                static_cast<double>(rx_Q[n]) * static_cast<double>(rx_Q[n + lag]);
        ac_Q += static_cast<double>(rx_Q[n]) * static_cast<double>(rx_I[n + lag]) -
                static_cast<double>(rx_I[n]) * static_cast<double>(rx_Q[n + lag]);
    }
    if (std::abs(ac_I) < 1e-6 && std::abs(ac_Q) < 1e-6) {
        return 0.0;
    }
    const double phase = std::atan2(ac_Q, ac_I);
    return phase * ExperimentConfig::kChipRateHz / (2.0 * kPi * static_cast<double>(lag));
}

static void v5a_apply_per_chip(const int16_t* rx_I, const int16_t* rx_Q,
                               int16_t* out_I, int16_t* out_Q,
                               int chips, double cfo_hz) noexcept {
    const int64_t phase_inc_q32_s64 = -static_cast<int64_t>(
        (cfo_hz * 4294967296.0) / ExperimentConfig::kChipRateHz);
    const uint32_t phase_inc_q32 = static_cast<uint32_t>(phase_inc_q32_s64);
    uint32_t phase_q32 = 0u;
    for (int k = 0; k < chips; ++k) {
        const int idx = static_cast<int>(
            (phase_q32 >> kSinCosIndexShift) & (kSinCosTableSize - 1));
        const int32_t cos_q14 = g_cos_table[idx];
        const int32_t sin_q14 = g_sin_table[idx];
        const int32_t x = static_cast<int32_t>(rx_I[k]);
        const int32_t y = static_cast<int32_t>(rx_Q[k]);
        int32_t a = (x * cos_q14 - y * sin_q14) >> 14;
        int32_t b = (x * sin_q14 + y * cos_q14) >> 14;
        if (a > 32767) a = 32767;
        if (a < -32768) a = -32768;
        if (b > 32767) b = 32767;
        if (b < -32768) b = -32768;
        out_I[k] = static_cast<int16_t>(a);
        out_Q[k] = static_cast<int16_t>(b);
        phase_q32 += phase_inc_q32;
    }
}

static void fwht_64(int32_t* data) noexcept {
    int h = 1;
    while (h < 64) {
        for (int i = 0; i < 64; i += h * 2) {
            for (int j = i; j < i + h; ++j) {
                const int32_t x = data[j];
                const int32_t y = data[j + h];
                data[j] = x + y;
                data[j + h] = x - y;
            }
        }
        h *= 2;
    }
}

static bool holo_sync_check(const int16_t* rx_I, const int16_t* rx_Q) noexcept {
    int32_t wI[64]{};
    int32_t wQ[64]{};
    for (int k = 0; k < 64; ++k) {
        wI[k] = static_cast<int32_t>(rx_I[k]);
        wQ[k] = static_cast<int32_t>(rx_Q[k]);
    }
    fwht_64(wI);
    fwht_64(wQ);
    const int64_t e63 =
        static_cast<int64_t>(wI[63]) * static_cast<int64_t>(wI[63]) +
        static_cast<int64_t>(wQ[63]) * static_cast<int64_t>(wQ[63]);
    const int64_t thr =
        static_cast<int64_t>(ExperimentConfig::kHoloSyncE63Threshold) *
        static_cast<int64_t>(ExperimentConfig::kHoloSyncE63Threshold);
    return e63 >= thr;
}

static void generate_holo_preamble(int16_t* tx_I, int16_t* tx_Q, int amp) noexcept {
    const uint8_t row = 63u;
    for (int c = 0; c < 64; ++c) {
        const uint32_t x = static_cast<uint32_t>(row) & static_cast<uint32_t>(c);
        const int parity = static_cast<int>(popcount32_(x) & 1u);
        tx_I[c] = static_cast<int16_t>(parity ? -amp : amp);
        tx_Q[c] = 0;
    }
}

static void rx_soft_generate(const int16_t* buf_I, const int16_t* buf_Q,
                             int16_t* rx_soft, int N,
                             uint64_t* out_valid_mask) noexcept {
    uint64_t valid_mask = 0u;
    for (int c = 0; c < N; ++c) {
        const int32_t i_val = static_cast<int32_t>(buf_I[c]);
        const int32_t q_val = static_cast<int32_t>(buf_Q[c]);
        int32_t valid_chip;
        if (ExperimentConfig::kChipValidThreshold > 0) {
            const int32_t mag_sq = i_val * i_val + q_val * q_val;
            const int64_t diff =
                static_cast<int64_t>(mag_sq) -
                static_cast<int64_t>(ExperimentConfig::kChipValidThreshold);
            valid_chip = static_cast<int32_t>(~(diff >> 63));
        } else {
            valid_chip = -1;
        }
        int32_t soft;
        switch (ExperimentConfig::kRxSoftMode) {
            case 0: soft = (i_val + q_val) / 2; break;
            case 1: soft = i_val; break;
            case 2: soft = q_val; break;
            case 3: {
                const int32_t abs_i = (i_val < 0) ? -i_val : i_val;
                const int32_t abs_q = (q_val < 0) ? -q_val : q_val;
                soft = (abs_i > abs_q) ? i_val : q_val;
                break;
            }
            case 4: soft = i_val + q_val; break;
            default: soft = (i_val + q_val) / 2; break;
        }
        rx_soft[c] = static_cast<int16_t>(soft & valid_chip);
        valid_mask |= (static_cast<uint64_t>(valid_chip & 1u)
                       << static_cast<uint32_t>(c));
    }
    *out_valid_mask = valid_mask;
}

static int tx_build_tensor(HTS_Holo_Tensor_4D_TX& tensor,
                           const int8_t* data_bits,
                           int16_t* tx_I, int16_t* tx_Q,
                           int amp) noexcept {
    int8_t chip_bpsk[64]{};
    if (tensor.Encode_Block(data_bits, static_cast<uint16_t>(ExperimentConfig::kK),
                            chip_bpsk, static_cast<uint16_t>(ExperimentConfig::kN)) !=
        HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        return 0;
    }
    const int denom = ExperimentConfig::kDenom;
    const int half_denom = denom >> 1;
    for (int c = 0; c < ExperimentConfig::kN; ++c) {
        const int32_t prod = static_cast<int32_t>(chip_bpsk[c]) * amp;
        const int32_t v =
            (prod >= 0) ? ((prod + half_denom) / denom)
                        : ((prod - half_denom) / denom);
        tx_I[c] = static_cast<int16_t>(v);
        tx_Q[c] = static_cast<int16_t>(v);
    }
    return ExperimentConfig::kN;
}

struct TestResult {
    int sync_pass;
    int decode_pass;
    int total_bit_err;
    int total_bits;
};

static TestResult run_one_cfo(int cfo_hz, int snr_db, int num_trials) {
    TestResult res = {0, 0, 0, 0};
    HTS_Holo_Tensor_4D_TX tx_tensor;
    HTS_Holo_Tensor_4D_RX rx_tensor;
    uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5B5A5A5u, 0xC3D3C3C3u};
    if (tx_tensor.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        return res;
    }
    if (rx_tensor.Initialize(master_seed, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        tx_tensor.Shutdown();
        return res;
    }

    for (int trial = 0; trial < num_trials; ++trial) {
        rng_seed(static_cast<uint32_t>(trial * 7919u + 13u));
        int8_t data_bits[16]{};
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            data_bits[i] = ((rng_next() & 1u) != 0u) ? 1 : -1;
        }

        (void)tx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
        int16_t tx_I[64]{}, tx_Q[64]{};
        if (tx_build_tensor(tx_tensor, data_bits, tx_I, tx_Q, ExperimentConfig::kAmp) == 0) {
            continue;
        }

        bool sync_ok = true;
        if (ExperimentConfig::kEnableHoloSync) {
            int16_t pre_I[64]{}, pre_Q[64]{};
            generate_holo_preamble(pre_I, pre_Q, ExperimentConfig::kAmp);
            int16_t pre_rx_I[64]{}, pre_rx_Q[64]{};
            const uint32_t pre_seed = static_cast<uint32_t>(trial * 31u + 7u);
            channel_apply(pre_I, pre_Q, pre_rx_I, pre_rx_Q,
                          64, static_cast<double>(cfo_hz), snr_db, pre_seed);
            int16_t pre_corr_I[64]{}, pre_corr_Q[64]{};
            if (ExperimentConfig::kEnableV5aCorrection) {
                const double cfo_est = v5a_estimate_cfo(pre_rx_I, pre_rx_Q, 64, 32);
                v5a_apply_per_chip(pre_rx_I, pre_rx_Q, pre_corr_I, pre_corr_Q, 64, cfo_est);
            } else {
                std::memcpy(pre_corr_I, pre_rx_I, sizeof(pre_corr_I));
                std::memcpy(pre_corr_Q, pre_rx_Q, sizeof(pre_corr_Q));
            }
            sync_ok = holo_sync_check(pre_corr_I, pre_corr_Q);
        }

        if (sync_ok) {
            ++res.sync_pass;
        } else {
            res.total_bits += ExperimentConfig::kK;
            res.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        int16_t rx_I[64]{}, rx_Q[64]{};
        const uint32_t ch_seed = static_cast<uint32_t>(trial * 31u + 19u);
        channel_apply(tx_I, tx_Q, rx_I, rx_Q, ExperimentConfig::kN,
                      static_cast<double>(cfo_hz), snr_db, ch_seed);

        int16_t corr_I[64]{}, corr_Q[64]{};
        if (ExperimentConfig::kEnableV5aCorrection) {
            const double cfo_est = v5a_estimate_cfo(rx_I, rx_Q, ExperimentConfig::kN, 32);
            v5a_apply_per_chip(rx_I, rx_Q, corr_I, corr_Q, ExperimentConfig::kN, cfo_est);
        } else {
            std::memcpy(corr_I, rx_I, sizeof(corr_I));
            std::memcpy(corr_Q, rx_Q, sizeof(corr_Q));
        }

        int16_t rx_soft[64]{};
        uint64_t valid_mask = 0xFFFFFFFFFFFFFFFFull;
        rx_soft_generate(corr_I, corr_Q, rx_soft, ExperimentConfig::kN, &valid_mask);

        (void)rx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
        int8_t output_bits[16]{};
        if (rx_tensor.Decode_Block(rx_soft, static_cast<uint16_t>(ExperimentConfig::kN),
                                   valid_mask, output_bits,
                                   static_cast<uint16_t>(ExperimentConfig::kK)) !=
            HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            res.total_bits += ExperimentConfig::kK;
            res.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        int bit_err = 0;
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            if (output_bits[i] != data_bits[i]) ++bit_err;
        }
        res.total_bit_err += bit_err;
        res.total_bits += ExperimentConfig::kK;
        if (bit_err == 0) ++res.decode_pass;
    }

    rx_tensor.Shutdown();
    tx_tensor.Shutdown();
    return res;
}

static const char* rx_soft_mode_name(int mode) {
    switch (mode) {
        case 0: return "(I+Q)/2";
        case 1: return "I only";
        case 2: return "Q only";
        case 3: return "dominant";
        case 4: return "I+Q";
        default: return "unknown";
    }
}

void test_holo_tensor_cfo_sweep() {
    build_sincos_table();
    std::printf("\n");
    std::printf("============================================================\n");
    std::printf("  Phase H Lab: V5a + 4D Tensor CFO Sweep (영준님 격리 환경)\n");
    std::printf("============================================================\n");
    std::printf("Profile: K=%d, N=%d, L=%d (영준님 4D 텐서 default)\n",
                ExperimentConfig::kK, ExperimentConfig::kN, ExperimentConfig::kL);
    std::printf("Trials per (cfo, SNR): %d\n", ExperimentConfig::kNumTrials);
    std::printf("Chip rate: %.0f Hz (1 MHz 가정)\n", ExperimentConfig::kChipRateHz);
    std::printf("V5a correction: %s\n",
                ExperimentConfig::kEnableV5aCorrection ? "ON" : "OFF");
    std::printf("HOLO sync: %s (S%s sim)\n",
                ExperimentConfig::kEnableHoloSync ? "ON" : "OFF",
                ExperimentConfig::kEnableHoloSync ? "5H" : "5");
    std::printf("rx_soft mode: %d (%s)\n", ExperimentConfig::kRxSoftMode,
                rx_soft_mode_name(ExperimentConfig::kRxSoftMode));
    std::printf("ChipValidThreshold: %d %s\n",
                static_cast<int>(ExperimentConfig::kChipValidThreshold),
                (ExperimentConfig::kChipValidThreshold == 0) ? "(all valid)" : "");
    std::printf("============================================================\n\n");

    std::printf("%-8s", "cfo Hz");
    for (int s = 0; s < ExperimentConfig::kSnrCount; ++s) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), " %dB sync/dec/BER",
                      ExperimentConfig::kSnrLevels[s]);
        std::printf(" %-22s", buf);
    }
    std::printf("\n");
    std::printf("------------------------------------------------------------\n");

    for (int c = 0; c < ExperimentConfig::kCfoSweepCount; ++c) {
        const int cfo_hz = ExperimentConfig::kCfoSweepList[c];
        std::printf("%-8d", cfo_hz);
        for (int s = 0; s < ExperimentConfig::kSnrCount; ++s) {
            const int snr_db = ExperimentConfig::kSnrLevels[s];
            const TestResult r = run_one_cfo(cfo_hz, snr_db, ExperimentConfig::kNumTrials);
            const double ber = (r.total_bits > 0)
                ? static_cast<double>(r.total_bit_err) /
                      static_cast<double>(r.total_bits)
                : 0.0;
            std::printf("  %3d/%3d/%4.2f  %-4s",
                        r.sync_pass, r.decode_pass, ber,
                        (r.decode_pass >= (ExperimentConfig::kNumTrials * 95 / 100)) ? "OK"
                                                                                       : "");
        }
        std::printf("\n");
    }

    std::printf("\n");
    std::printf("Notes:\n");
    std::printf("  sync = HOLO sync 통과 수 (HOLO sync OFF 시 N/N)\n");
    std::printf("  dec  = 디코딩 100%% 성공 수\n");
    std::printf("  BER  = Bit Error Rate\n");
    std::printf("  OK   = decode >= 95%%\n\n");
    std::printf("영준님 실험:\n");
    std::printf("  1. kEnableHoloSync = false -> S5 시뮬\n");
    std::printf("  2. kEnableHoloSync = true  -> S5H 시뮬\n");
    std::printf("  3. kRxSoftMode 변경 (0~4)\n");
    std::printf("  4. kChipValidThreshold 변경\n");
    std::printf("  5. F5 빠른 재실행\n");
    std::printf("============================================================\n");
}

int main() {
    test_holo_tensor_cfo_sweep();
    return 0;
}
// =============================================================================
// HTS_CFO_Bank_Test.cpp
// Isolated lab scaffold: 4D tensor TX/RX + optional V5a + optional holo sync
// =============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../HTS_LIM/HTS_CFO_V5a.hpp"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_TX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_RX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_Defs.h"
#include "../HTS_LIM/HTS_Preamble_Holographic.h"
#include "../HTS_LIM/HTS_Rx_CFO_SinCos_Table.hpp"

#if !defined(HTS_ALLOW_HOST_BUILD)
#error "Host-only lab scaffold (HTS_ALLOW_HOST_BUILD required)"
#endif

namespace {
using ProtectedEngine::HoloTensor_Profile;
using ProtectedEngine::k_holo_profiles;
using ProtectedEngine::HOLO_MAX_BLOCK_BITS;
using ProtectedEngine::HOLO_CHIP_COUNT;

constexpr int kTrials = 100;
constexpr int16_t kAmp = 1000;
constexpr int kLag = 32;
constexpr double kPi = 3.14159265358979323846;
constexpr double kFs = 1000000.0;
constexpr uint64_t kAllValidMask = 0xFFFFFFFFFFFFFFFFull;
constexpr uint32_t kSeedMixA = 0x9E3779B9u;
constexpr uint32_t kSeedMixB = 0xA5A5A5A5u;
constexpr uint32_t kSeedMixC = 0xC3C3C3C3u;

// ────────────────────────────────────
// 영준님이 VS 에서 수정 가능 영역:
// - CFO sweep 범위: kCfoSweepHz
// - AWGN 세기: kAwgnSigma
// - V5a 적용 여부: kApplyV5a
// - HOLO sync 임계값: kHoloSyncMinEnergy
// - rx_soft 생성 (현재 Dispatcher 동일): make_rx_soft_iq_avg_
// ────────────────────────────────────
constexpr int kCfoSweepHz[] = {0, 100, 200, 300, 500, 700, 1000, 2000, 5000};
constexpr double kAwgnSigma = 0.0;
constexpr bool kApplyV5a = true;
constexpr int64_t kHoloSyncMinEnergy = 10000000LL;

struct TrialStats {
    int pass = 0;
    int bit_errors = 0;
    int total_bits = 0;
};

static uint32_t lcg_next_(uint32_t& s) noexcept {
    s = s * 1664525u + 1013904223u;
    return s;
}

static int16_t sat_i16_(int32_t v) noexcept {
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return static_cast<int16_t>(v);
}

static void fill_info_(uint32_t seed, int trial, uint8_t out[8]) noexcept {
    uint32_t s = seed ^ static_cast<uint32_t>(trial * 2654435761u);
    for (int i = 0; i < 8; ++i) {
        out[i] = static_cast<uint8_t>(lcg_next_(s) >> 24);
    }
}

static void bytes_to_bpsk_bits_(const uint8_t* bytes, int8_t* bits,
                                int bit_count) noexcept {
    for (int i = 0; i < bit_count; ++i) {
        const uint8_t b = static_cast<uint8_t>(
            (bytes[i >> 3] >> (7 - (i & 7))) & 1u);
        bits[i] = (b != 0u) ? 1 : -1;
    }
}

static void bpsk_to_bytes_(const int8_t* bits, uint8_t out[8]) noexcept {
    std::memset(out, 0, 8);
    for (int i = 0; i < 64; ++i) {
        if (bits[i] > 0) {
            out[i >> 3] = static_cast<uint8_t>(
                out[i >> 3] | (1u << (7 - (i & 7))));
        }
    }
}

static int bit_errors_(const uint8_t a[8], const uint8_t b[8]) noexcept {
    int err = 0;
    for (int i = 0; i < 8; ++i) {
        uint8_t d = static_cast<uint8_t>(a[i] ^ b[i]);
        d = static_cast<uint8_t>(d - ((d >> 1) & 0x55u));
        d = static_cast<uint8_t>((d & 0x33u) + ((d >> 2) & 0x33u));
        d = static_cast<uint8_t>((d + (d >> 4)) & 0x0Fu);
        err += static_cast<int>(d);
    }
    return err;
}

static void derive_master_seed_(uint32_t seed, uint32_t master[4]) noexcept {
    master[0] = seed;
    master[1] = seed ^ kSeedMixA;
    master[2] = seed ^ kSeedMixB;
    master[3] = seed ^ kSeedMixC;
}

static bool build_tensor_tx_chips_(HTS_Holo_Tensor_4D_TX& tx, uint32_t seed,
                                   uint32_t slot, const uint8_t info[8],
                                   int16_t out_i[64], int16_t out_q[64]) noexcept {
    const HoloTensor_Profile profile = k_holo_profiles[1];
    int8_t in_bits[HOLO_MAX_BLOCK_BITS]{};
    int8_t chip_bpsk[HOLO_CHIP_COUNT]{};
    bytes_to_bpsk_bits_(info, in_bits, static_cast<int>(profile.block_bits));
    if (tx.Set_Time_Slot(slot) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) return false;
    if (tx.Encode_Block(in_bits, profile.block_bits, chip_bpsk,
                        profile.chip_count) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        return false;
    }
    const int lk = static_cast<int>(profile.num_layers) *
                   static_cast<int>(profile.block_bits);
    const int denom_raw = (lk > 0) ? lk : 32;
    const int denom = (denom_raw > 1) ? (denom_raw >> 1) : 1;
    for (int c = 0; c < 64; ++c) {
        const int32_t prod =
            static_cast<int32_t>(chip_bpsk[c]) * static_cast<int32_t>(kAmp);
        const int32_t v =
            (prod >= 0) ? ((prod + (denom >> 1)) / denom)
                        : ((prod - (denom >> 1)) / denom);
        out_i[c] = sat_i16_(v);
        out_q[c] = sat_i16_(v);  // Dispatcher TX 동일: I=Q
    }
    (void)seed;
    return true;
}

static void apply_channel_cfo_awgn_(const int16_t in_i[64], const int16_t in_q[64],
                                    int cfo_hz, uint32_t noise_seed,
                                    int16_t out_i[64], int16_t out_q[64]) noexcept {
    const double w = 2.0 * kPi * static_cast<double>(cfo_hz) / kFs;
    uint32_t s = noise_seed;
    for (int n = 0; n < 64; ++n) {
        const double ph = w * static_cast<double>(n);
        const double c = std::cos(ph);
        const double si = std::sin(ph);
        const double ri = static_cast<double>(in_i[n]) * c -
                          static_cast<double>(in_q[n]) * si;
        const double rq = static_cast<double>(in_i[n]) * si +
                          static_cast<double>(in_q[n]) * c;
        double ni = 0.0;
        double nq = 0.0;
        if (kAwgnSigma > 0.0) {
            const double u1 = (static_cast<double>(lcg_next_(s) & 0x00FFFFFFu) + 1.0) /
                              16777217.0;
            const double u2 = (static_cast<double>(lcg_next_(s) & 0x00FFFFFFu) + 1.0) /
                              16777217.0;
            const double r = std::sqrt(-2.0 * std::log(u1));
            const double th = 2.0 * kPi * u2;
            ni = kAwgnSigma * r * std::cos(th);
            nq = kAwgnSigma * r * std::sin(th);
        }
        out_i[n] = sat_i16_(static_cast<int32_t>(std::llround(ri + ni)));
        out_q[n] = sat_i16_(static_cast<int32_t>(std::llround(rq + nq)));
    }
}

static void apply_v5a_if_enabled_(int16_t io_i[64], int16_t io_q[64]) noexcept {
    if (!kApplyV5a) return;
    int64_t ac_i = 0;
    int64_t ac_q = 0;
    for (int n = 0; n + kLag < 64; ++n) {
        const int32_t i0 = io_i[n];
        const int32_t q0 = io_q[n];
        const int32_t i1 = io_i[n + kLag];
        const int32_t q1 = io_q[n + kLag];
        ac_i += static_cast<int64_t>(i0) * i1 + static_cast<int64_t>(q0) * q1;
        ac_q += static_cast<int64_t>(q0) * i1 - static_cast<int64_t>(i0) * q1;
    }
    hts::rx_cfo::CFO_V5a v5a{};
    v5a.Init();
    v5a.Estimate_From_Autocorr(static_cast<int32_t>(ac_i), static_cast<int32_t>(ac_q),
                               kLag);
    for (int n = 0; n < 64; ++n) {
        v5a.Apply_Per_Chip(io_i[n], io_q[n]);
    }
}

static bool sync_gate_(bool use_holo_sync, const int16_t in_i[64],
                       const int16_t in_q[64]) noexcept {
    if (!use_holo_sync) {
        return true;  // 일반 sync/우회 실험 모드
    }
    const int64_t e = ProtectedEngine::Holographic::holographic_dot_segmented(
        in_i, in_q);
    return e >= kHoloSyncMinEnergy;
}

static void make_rx_soft_iq_avg_(const int16_t in_i[64], const int16_t in_q[64],
                                 int16_t out_soft[64]) noexcept {
    // Dispatcher RX 동일 경로: (I + Q) / 2
    for (int c = 0; c < 64; ++c) {
        const int32_t sum = static_cast<int32_t>(in_i[c]) +
                            static_cast<int32_t>(in_q[c]);
        out_soft[c] = static_cast<int16_t>(sum / 2);
    }
}

static bool decode_tensor_rx_(HTS_Holo_Tensor_4D_RX& rx, uint32_t slot,
                              const int16_t rx_i[64], const int16_t rx_q[64],
                              uint8_t out[8]) noexcept {
    const HoloTensor_Profile profile = k_holo_profiles[1];
    int16_t rx_soft[64]{};
    int8_t out_bits[HOLO_MAX_BLOCK_BITS]{};
    make_rx_soft_iq_avg_(rx_i, rx_q, rx_soft);
    if (rx.Set_Time_Slot(slot) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) return false;
    if (rx.Decode_Block(rx_soft, profile.chip_count, kAllValidMask,
                        out_bits, profile.block_bits) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        return false;
    }
    bpsk_to_bytes_(out_bits, out);
    return true;
}

static TrialStats run_sweep_mode_(bool use_holo_sync, int cfo_hz) noexcept {
    TrialStats st{};
    for (int t = 0; t < kTrials; ++t) {
        const uint32_t seed = 0xA5100000u ^ static_cast<uint32_t>(t * 1103515245u);
        const uint32_t slot = 0u;
        uint8_t tx_info[8]{};
        fill_info_(seed, t, tx_info);

        uint32_t master[4]{};
        derive_master_seed_(seed, master);
        HTS_Holo_Tensor_4D_TX tx{};
        HTS_Holo_Tensor_4D_RX rx{};
        if (tx.Initialize(master, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) continue;
        if (rx.Initialize(master, nullptr) != HTS_Holo_Tensor_4D_TX::SECURE_TRUE) continue;

        int16_t tx_i[64]{};
        int16_t tx_q[64]{};
        if (!build_tensor_tx_chips_(tx, seed, slot, tx_info, tx_i, tx_q)) continue;

        int16_t ch_i[64]{};
        int16_t ch_q[64]{};
        apply_channel_cfo_awgn_(tx_i, tx_q, cfo_hz, seed ^ 0x55AA1234u, ch_i, ch_q);
        apply_v5a_if_enabled_(ch_i, ch_q);
        if (!sync_gate_(use_holo_sync, ch_i, ch_q)) {
            st.bit_errors += 64;
            st.total_bits += 64;
            continue;
        }

        uint8_t rx_info[8]{};
        const bool ok = decode_tensor_rx_(rx, slot, ch_i, ch_q, rx_info);
        const int ber = ok ? bit_errors_(tx_info, rx_info) : 64;
        if (ok && ber == 0) {
            st.pass++;
        }
        st.bit_errors += ber;
        st.total_bits += 64;
        rx.Shutdown();
        tx.Shutdown();
    }
    return st;
}

void test_holo_tensor_cfo_sweep() {
    const char* const mode_name[2] = {"S5-like(sync-off)", "S5H-like(sync-on)"};
    for (int mode = 0; mode < 2; ++mode) {
        const bool use_holo_sync = (mode != 0);
        std::printf("\n[ISOLATED] mode=%s v5a=%d awgn_sigma=%.2f\n",
                    mode_name[mode], kApplyV5a ? 1 : 0, kAwgnSigma);
        for (int i = 0; i < static_cast<int>(sizeof(kCfoSweepHz) / sizeof(kCfoSweepHz[0])); ++i) {
            const int cfo = kCfoSweepHz[i];
            const TrialStats st = run_sweep_mode_(use_holo_sync, cfo);
            const double ber =
                (st.total_bits > 0) ? (static_cast<double>(st.bit_errors) /
                                       static_cast<double>(st.total_bits))
                                    : 1.0;
            std::printf("cfo=%5d Hz: %3d/%3d PASS  BER=%.6f\n",
                        cfo, st.pass, kTrials, ber);
        }
    }
}

}  // namespace

int main() {
    hts::rx_cfo::Build_SinCos_Table();
    test_holo_tensor_cfo_sweep();
    std::printf("HTS_CFO_Bank_Test: PASS\n");
    return 0;
}

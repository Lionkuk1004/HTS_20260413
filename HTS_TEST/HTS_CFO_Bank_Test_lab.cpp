#ifndef HTS_CFO_BANK_TEST_LAB_ONCE
#define HTS_CFO_BANK_TEST_LAB_ONCE

// ============================================================================
// HTS_CFO_Bank_Test_lab.cpp — Phase H Lab C: Python vs C++ 정합 (BUG-6)
//   - kEnableHoloSync=false, V5a 없음, 프리앰블 없음, 64-chip 페이로드만, (I+Q)/2 소프트, mask 전부 유효
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../HTS_LIM/HTS_Holo_Tensor_4D.h"

using namespace ProtectedEngine;

namespace ExperimentConfig {
/// Lab C: Python 시뮬 CFO 스윕과 동일 목록
constexpr int kCfoSweepList[] = {0, 2000, 3000, 4000, 4500, 5000};
constexpr int kCfoSweepCount = sizeof(kCfoSweepList) / sizeof(int);
constexpr int kNumTrials = 100;
constexpr int kSnrLevels[] = {30, 20, 10};
constexpr int kSnrCount = sizeof(kSnrLevels) / sizeof(int);
constexpr int kK = 16;
constexpr int kN = 64;
constexpr int kAmp = 500;
constexpr int kDenom = 16;
constexpr double kChipRateHz = 1e6;
}  // namespace ExperimentConfig

namespace {
constexpr double kPi = 3.14159265358979323846;
uint32_t g_rng = 1u;

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

/// Python 정합: (I+Q)/2, valid_mask 전 비트 1 (BPTE/마스크 게이트 없음)
static void rx_soft_lab_c(const int16_t* i, const int16_t* q, int16_t* soft, uint64_t* valid_mask) noexcept {
    for (int c = 0; c < ExperimentConfig::kN; ++c) {
        const int32_t iv = i[c];
        const int32_t qv = q[c];
        soft[c] = static_cast<int16_t>((iv + qv) / 2);
    }
    *valid_mask = 0xFFFFFFFFFFFFFFFFull;
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

        int16_t txI[64]{};
        int16_t txQ[64]{};
        for (int c = 0; c < 64; ++c) {
            const int32_t prod = static_cast<int32_t>(chips[c]) * ExperimentConfig::kAmp;
            const int32_t v = (prod >= 0) ? ((prod + (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom)
                                          : ((prod - (ExperimentConfig::kDenom >> 1)) / ExperimentConfig::kDenom);
            txI[c] = static_cast<int16_t>(v);
            txQ[c] = static_cast<int16_t>(v);
        }

        const bool want_diag =
            (cfo_hz == 4000 && t == 0 && snr_db == 30);
        if (want_diag) {
            std::printf("[DIAG-LabC] cfo=%d snr=%d trial=%d (payload 64ch, no holo/V5a)\n", cfo_hz, snr_db, t);
            std::printf("[DIAG-LabC] chips[0..7]= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(chips[0]), static_cast<int>(chips[1]), static_cast<int>(chips[2]),
                        static_cast<int>(chips[3]), static_cast<int>(chips[4]), static_cast<int>(chips[5]),
                        static_cast<int>(chips[6]), static_cast<int>(chips[7]));
            std::printf("[DIAG-LabC] txI[0..7]= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(txI[0]), static_cast<int>(txI[1]), static_cast<int>(txI[2]),
                        static_cast<int>(txI[3]), static_cast<int>(txI[4]), static_cast<int>(txI[5]),
                        static_cast<int>(txI[6]), static_cast<int>(txI[7]));
            std::printf("[DIAG-LabC] txQ[0..7]= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(txQ[0]), static_cast<int>(txQ[1]), static_cast<int>(txQ[2]),
                        static_cast<int>(txQ[3]), static_cast<int>(txQ[4]), static_cast<int>(txQ[5]),
                        static_cast<int>(txQ[6]), static_cast<int>(txQ[7]));
        }

        int16_t rxI[64]{};
        int16_t rxQ[64]{};
        channel_apply(txI, txQ, rxI, rxQ, 64, static_cast<double>(cfo_hz), snr_db,
                      static_cast<uint32_t>(t * 31u + 19u));

        if (want_diag) {
            std::printf("[DIAG-LabC] rxI[0..7]= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(rxI[0]), static_cast<int>(rxI[1]), static_cast<int>(rxI[2]),
                        static_cast<int>(rxI[3]), static_cast<int>(rxI[4]), static_cast<int>(rxI[5]),
                        static_cast<int>(rxI[6]), static_cast<int>(rxI[7]));
            std::printf("[DIAG-LabC] rxQ[0..7]= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(rxQ[0]), static_cast<int>(rxQ[1]), static_cast<int>(rxQ[2]),
                        static_cast<int>(rxQ[3]), static_cast<int>(rxQ[4]), static_cast<int>(rxQ[5]),
                        static_cast<int>(rxQ[6]), static_cast<int>(rxQ[7]));
        }

        int16_t soft[64]{};
        uint64_t mask = 0u;
        rx_soft_lab_c(rxI, rxQ, soft, &mask);

        if (want_diag) {
            std::printf("[DIAG-LabC] rx_soft[0..7]= %d %d %d %d %d %d %d %d\n",
                        static_cast<int>(soft[0]), static_cast<int>(soft[1]), static_cast<int>(soft[2]),
                        static_cast<int>(soft[3]), static_cast<int>(soft[4]), static_cast<int>(soft[5]),
                        static_cast<int>(soft[6]), static_cast<int>(soft[7]));
            std::printf("[DIAG-LabC] valid_mask low64=0x%016llx\n",
                        static_cast<unsigned long long>(mask));
        }

        ++r.sync_pass;

        int8_t out[16]{};
        (void)rx.Set_Time_Slot(static_cast<uint32_t>(t));
        if (rx.Decode_Block(soft, static_cast<uint16_t>(ExperimentConfig::kN), mask, out,
                            static_cast<uint16_t>(ExperimentConfig::kK)) != HTS_Holo_Tensor_4D::SECURE_TRUE) {
            if (want_diag) {
                std::printf("[DIAG-LabC] decode_block SECURE_FALSE\n");
            }
            r.total_bits += ExperimentConfig::kK;
            r.total_bit_err += ExperimentConfig::kK;
            continue;
        }

        if (want_diag) {
            std::printf("[DIAG-LabC] bits[0..3] expect %d %d %d %d got %d %d %d %d\n",
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
    std::printf("Phase H Lab C: Python alignment (64-chip payload, no holo/V5a, soft=(I+Q)/2)\n");
    std::printf("Python ref @30dB (reported): cfo 4000 D~100/100, 4500 D~97/100, 5000 D~76/100\n\n");
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

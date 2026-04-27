// ============================================================================
// HTS_CFO_Bank_Test_v4.cpp — Lab: 양산 CFO_V5a + 128-chip preamble + pipeline DIAG
// ★ 양산 HTS_CFO_V5a (Estimate / Set_Apply_Cfo / Advance_Phase_Only /
//   Apply_Per_Chip) — Lab 전용; HTS_LIM/CFO_V5a.cpp 본문 미수정.
// ★ HTS_PIPELINE_DIAG: Stage1~6 printf (매크로 가드).
// ★ HTS_BATCH_1..5: 50 Hz step 정밀 sweep 분할 (cl /DHTS_BATCH_n).
//    경계 Hz(1000,2000,…)는 인접 배치에 중복 포함 → 통합 시 한 줄만 사용.
// ============================================================================
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "HTS_CFO_V5a.hpp"
#include "HTS_Holo_Tensor_4D_Defs.h"
#include "HTS_Holo_Tensor_4D_RX.h"
#include "HTS_Holo_Tensor_4D_TX.h"

#include <windows.h>

using namespace ProtectedEngine;

namespace ExperimentConfig {
// v4: S5H-style — 128-chip Walsh preamble + 64-chip 4D payload = 192 chips / packet
constexpr int kK = 16;
constexpr int kN = 64;
constexpr int kL = 2;
constexpr int kAmp = 500;
constexpr int kDenom = 16;
constexpr double kChipRateHz = 1e6;
constexpr bool kEnableHoloSync = true;
constexpr int kHoloSyncE63Threshold = kAmp * 38;
constexpr int kPreambleChips = 128;
constexpr int kPayloadChips = 64;
constexpr int kTotalChips = kPreambleChips + kPayloadChips;  // 192

// Sweep (30 dB 단일 SNR, dec/100 표)
#if defined(HTS_BATCH_1)
constexpr int kCfoSweepList[] = {
    0,    50,   100,  150,  200,  250,  300,  350,  400,  450,  500,
    550,  600,  650,  700,  750,  800,  850,  900,  950,  1000};
#elif defined(HTS_BATCH_2)
constexpr int kCfoSweepList[] = {
    1000, 1050, 1100, 1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500,
    1550, 1600, 1650, 1700, 1750, 1800, 1850, 1900, 1950, 2000};
#elif defined(HTS_BATCH_3)
constexpr int kCfoSweepList[] = {
    2000, 2050, 2100, 2150, 2200, 2250, 2300, 2350, 2400, 2450, 2500,
    2550, 2600, 2650, 2700, 2750, 2800, 2850, 2900, 2950, 3000};
#elif defined(HTS_BATCH_4)
constexpr int kCfoSweepList[] = {
    3000, 3050, 3100, 3150, 3200, 3250, 3300, 3350, 3400, 3450, 3500,
    3550, 3600, 3650, 3700, 3750, 3800, 3850, 3900, 3950, 4000};
#elif defined(HTS_BATCH_5)
constexpr int kCfoSweepList[] = {
    4000, 4050, 4100, 4150, 4200, 4250, 4300, 4350, 4400, 4450, 4500,
    4550, 4600, 4650, 4700, 4750, 4800, 4850, 4900, 4950, 5000};
#elif defined(HTS_BATCH_PTE_AB)
// Cliff 주변 PTE ON/OFF A/B (9점, trial 100 / SNR 30 dB)
constexpr int kCfoSweepList[] = {600,  750,  900,  2200, 2300, 2400,
                                 3700, 3850, 4000};
#elif defined(HTS_BATCH_LR_AB)
// LR 무력화 4-way 비교 (9점, trial 100 / SNR 30 dB)
constexpr int kCfoSweepList[] = {600,  750,  900,  2300, 2400, 3850,
                                 4000, 4500, 5000};
#else
// Default: 기존 27 점 (빠른 smoke)
constexpr int kCfoSweepList[] = {
    0,    50,   100,  150,  200,  250,  300,  350,  400,  450,  500,
    550,  600,  650,  700,  750,  800,  850,  900,  950,  1000,
    1500, 2000, 2500, 3000, 4000, 5000};
#endif
constexpr int kCfoSweepCount =
    static_cast<int>(sizeof(kCfoSweepList) / sizeof(kCfoSweepList[0]));
constexpr int kNumTrialsSweep = 100;

#if defined(HTS_PIPELINE_DIAG)
constexpr int kPipelineCfo[] = {0, 200, 300, 500, 1000};
constexpr int kPipelineCfoCount =
    static_cast<int>(sizeof(kPipelineCfo) / sizeof(kPipelineCfo[0]));
constexpr int kPipelineTrials = 3;
constexpr int kPipelineSnrDb = 30;
#endif
}  // namespace ExperimentConfig

namespace {
constexpr double kPi = 3.14159265358979323846;
uint32_t g_rng = 1u;
inline void rng_seed(uint32_t seed) noexcept {
    g_rng = seed ? seed : 1u;
}
inline uint32_t rng_next() noexcept {
    g_rng = g_rng * 1664525u + 1013904223u;
    return g_rng;
}
inline double rng_uniform() noexcept {
    return static_cast<double>(rng_next()) / 4294967296.0;
}
inline double rng_gauss() noexcept {
    double u1 = rng_uniform();
    double u2 = rng_uniform();
    if (u1 < 1e-10)
        u1 = 1e-10;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}
}  // namespace

static void channel_apply(const int16_t* tx_I, const int16_t* tx_Q,
                          int16_t* rx_I, int16_t* rx_Q, int chips,
                          double cfo_hz, int snr_db,
                          uint32_t ch_seed) noexcept {
    rng_seed(ch_seed);
    double sig_pow = 0.0;
    for (int k = 0; k < chips; ++k) {
        sig_pow += static_cast<double>(tx_I[k]) * tx_I[k];
        sig_pow += static_cast<double>(tx_Q[k]) * tx_Q[k];
    }
    sig_pow /= static_cast<double>(chips > 0 ? chips : 1);
    const double noise_sigma =
        (sig_pow > 0.0) ? std::sqrt(sig_pow / std::pow(10.0, snr_db / 10.0))
                        : 0.0;
    const double phase_inc =
        2.0 * kPi * cfo_hz / ExperimentConfig::kChipRateHz;
    double phase = 0.0;
    for (int k = 0; k < chips; ++k) {
        const double cs = std::cos(phase);
        const double sn = std::sin(phase);
        double rotI = tx_I[k] * cs - tx_Q[k] * sn;
        double rotQ = tx_I[k] * sn + tx_Q[k] * cs;
        rotI += rng_gauss() * noise_sigma;
        rotQ += rng_gauss() * noise_sigma;
        int32_t fI = static_cast<int32_t>(rotI);
        int32_t fQ = static_cast<int32_t>(rotQ);
        if (fI > 32767)
            fI = 32767;
        if (fI < -32768)
            fI = -32768;
        if (fQ > 32767)
            fQ = 32767;
        if (fQ < -32768)
            fQ = -32768;
        rx_I[k] = static_cast<int16_t>(fI);
        rx_Q[k] = static_cast<int16_t>(fQ);
        phase += phase_inc;
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
    const int64_t thr =
        static_cast<int64_t>(ExperimentConfig::kHoloSyncE63Threshold) *
        ExperimentConfig::kHoloSyncE63Threshold;
    return e63 >= thr;
}

/// PRE_SYM0: Walsh row 63, I=Q duplicated (양산 Lab 패턴).
/// PRE_SYM1: Walsh row 0 (all +1), I=Q duplicated.
static void build_walsh128_preamble(int16_t* pre_I, int16_t* pre_Q,
                                    int amp) noexcept {
    for (int c = 0; c < 64; ++c) {
        const uint32_t x = 63u & static_cast<uint32_t>(c);
        const int parity = static_cast<int>(std::popcount(x) & 1u);
        const int16_t v = static_cast<int16_t>(parity ? -amp : amp);
        pre_I[c] = v;
        pre_Q[c] = v;
    }
    for (int c = 0; c < 64; ++c) {
        pre_I[64 + c] = static_cast<int16_t>(amp);
        pre_Q[64 + c] = static_cast<int16_t>(amp);
    }
}

static int tx_build_tensor(HTS_Holo_Tensor_4D_TX& tensor,
                           const int8_t* data_bits, int16_t* tx_I,
                           int16_t* tx_Q, int amp) noexcept {
    int8_t chip_bpsk[64];
    if (tensor.Encode_Block(
            data_bits, static_cast<uint16_t>(ExperimentConfig::kK), chip_bpsk,
            static_cast<uint16_t>(ExperimentConfig::kN)) !=
        HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        return 0;
    }
    const int denom = ExperimentConfig::kDenom;
    const int half_denom = denom >> 1;
    for (int c = 0; c < ExperimentConfig::kN; ++c) {
        const int32_t prod = static_cast<int32_t>(chip_bpsk[c]) * amp;
        const int32_t v = (prod >= 0) ? ((prod + half_denom) / denom)
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

struct TrialOutcome {
    bool encode_ok;
    bool sync_ok;
    int bit_err;  ///< 0..kK if decode attempted; kK if decode API failed
};

/// 한 트라이얼: 양산 V5a + Two_Candidates. `diag_print` 이면 Stage printf.
static TrialOutcome run_single_trial_v4(HTS_Holo_Tensor_4D_TX& tx_tensor,
                                        HTS_Holo_Tensor_4D_RX& rx_tensor,
                                        int cfo_hz, int snr_db, int trial,
                                        bool diag_print) {
    rng_seed(static_cast<uint32_t>(trial * 7919u + 13u));
    int8_t data_bits[16];
    for (int i = 0; i < ExperimentConfig::kK; ++i) {
        data_bits[i] = (rng_next() & 1u) ? 1 : -1;
    }

    int16_t pre_I[128], pre_Q[128];
    build_walsh128_preamble(pre_I, pre_Q, ExperimentConfig::kAmp);

    (void)tx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
    int16_t pay_I[64], pay_Q[64];
    if (tx_build_tensor(tx_tensor, data_bits, pay_I, pay_Q,
                        ExperimentConfig::kAmp) == 0) {
        return TrialOutcome{false, false, ExperimentConfig::kK};
    }

    int16_t full_I[192], full_Q[192];
    std::memcpy(full_I, pre_I, sizeof(int16_t) * 128u);
    std::memcpy(full_I + 128, pay_I, sizeof(int16_t) * 64u);
    std::memcpy(full_Q, pre_Q, sizeof(int16_t) * 128u);
    std::memcpy(full_Q + 128, pay_Q, sizeof(int16_t) * 64u);

#if defined(HTS_PIPELINE_DIAG)
    if (diag_print) {
        std::printf("\n[TRIAL] cfo=%d snr=%d trial=%d\n", cfo_hz, snr_db,
                    trial);
        std::printf(
            "[STAGE1-TX] full_I[0..3]=%d,%d,%d,%d  full_I[64..67]=%d,%d,%d,%d  "
            "full_I[128..131]=%d,%d,%d,%d\n",
            static_cast<int>(full_I[0]), static_cast<int>(full_I[1]),
            static_cast<int>(full_I[2]), static_cast<int>(full_I[3]),
            static_cast<int>(full_I[64]), static_cast<int>(full_I[65]),
            static_cast<int>(full_I[66]), static_cast<int>(full_I[67]),
            static_cast<int>(full_I[128]), static_cast<int>(full_I[129]),
            static_cast<int>(full_I[130]), static_cast<int>(full_I[131]));
    }
#else
    (void)diag_print;
#endif

    int16_t rx_full_I[192], rx_full_Q[192];
    const uint32_t ch_seed = static_cast<uint32_t>(trial * 31u + 19u);
    channel_apply(full_I, full_Q, rx_full_I, rx_full_Q,
                  ExperimentConfig::kTotalChips, static_cast<double>(cfo_hz),
                  snr_db, ch_seed);

#if defined(HTS_PIPELINE_DIAG)
    if (diag_print) {
        std::printf(
            "[STAGE2-CH] rx_full_I[0..3]=%d,%d,%d,%d  "
            "rx_full_I[128..131]=%d,%d,%d,%d\n",
            static_cast<int>(rx_full_I[0]), static_cast<int>(rx_full_I[1]),
            static_cast<int>(rx_full_I[2]), static_cast<int>(rx_full_I[3]),
            static_cast<int>(rx_full_I[128]), static_cast<int>(rx_full_I[129]),
            static_cast<int>(rx_full_I[130]),
            static_cast<int>(rx_full_I[131]));
    }
#endif

    bool sync_ok = true;
    if (ExperimentConfig::kEnableHoloSync) {
        sync_ok = holo_sync_check(rx_full_I, rx_full_Q);
    }
    if (!sync_ok) {
#if defined(HTS_PIPELINE_DIAG)
        if (diag_print) {
            std::printf("[STAGE2b-SYNC] FAIL (skip est/apply/decode)\n");
        }
#endif
        return TrialOutcome{true, false, ExperimentConfig::kK};
    }

    hts::rx_cfo::CFO_V5a v5a;
    v5a.Init();
#if (HTS_CFO_V5A_ENABLE != 0)
    v5a.SetEnabled(true);
#endif

    const hts::rx_cfo::CFO_Result cfo_res =
        v5a.Estimate(rx_full_I, rx_full_Q);

#if defined(HTS_PIPELINE_DIAG)
    if (diag_print) {
        std::printf(
            "[STAGE3-EST] est=%d valid=%d allowed=%d sin14=%d cos14=%d",
            static_cast<int>(cfo_res.cfo_hz), cfo_res.valid ? 1 : 0,
            v5a.IsApplyAllowed() ? 1 : 0,
            static_cast<int>(v5a.Get_Apply_Sin_Per_Chip_Q14()),
            static_cast<int>(v5a.Get_Apply_Cos_Per_Chip_Q14()));
#if defined(HTS_V5A_BUG9_DIAG)
        std::printf(" gate_mag=%lld\n",
                    static_cast<long long>(v5a.Get_Last_Apply_Gate_Mag()));
#else
        std::printf("\n");
#endif
    }
#endif

    int16_t corr_I[64], corr_Q[64];
    if (cfo_res.valid && v5a.IsApplyAllowed()) {
        v5a.Set_Apply_Cfo(cfo_res.cfo_hz);
        v5a.Advance_Phase_Only(ExperimentConfig::kPreambleChips);
        for (int c = 0; c < ExperimentConfig::kPayloadChips; ++c) {
            int16_t in_I = rx_full_I[128 + c];
            int16_t in_Q = rx_full_Q[128 + c];
            v5a.Apply_Per_Chip(in_I, in_Q);
            corr_I[c] = in_I;
            corr_Q[c] = in_Q;
        }
    } else {
        std::memcpy(corr_I, rx_full_I + 128,
                    sizeof(int16_t) * static_cast<size_t>(
                        ExperimentConfig::kPayloadChips));
        std::memcpy(corr_Q, rx_full_Q + 128,
                    sizeof(int16_t) * static_cast<size_t>(
                        ExperimentConfig::kPayloadChips));
    }

#if defined(HTS_PIPELINE_DIAG)
    if (diag_print) {
        std::printf(
            "[STAGE4-APPLY] corr_I[0..3]=%d,%d,%d,%d  corr_I[60..63]=%d,%d,%d,%d\n",
            static_cast<int>(corr_I[0]), static_cast<int>(corr_I[1]),
            static_cast<int>(corr_I[2]), static_cast<int>(corr_I[3]),
            static_cast<int>(corr_I[60]), static_cast<int>(corr_I[61]),
            static_cast<int>(corr_I[62]), static_cast<int>(corr_I[63]));
    }
#endif

    (void)rx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
    int8_t bits_cand0[16];
    int8_t bits_cand1[16];
    const uint64_t valid_mask = 0xFFFFFFFFFFFFFFFFull;
    if (rx_tensor.Decode_Block_Two_Candidates(
            corr_I, corr_Q,
            static_cast<uint16_t>(ExperimentConfig::kN), valid_mask,
            bits_cand0, bits_cand1,
            static_cast<uint16_t>(ExperimentConfig::kK)) !=
        HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
#if defined(HTS_PIPELINE_DIAG)
        if (diag_print) {
            std::printf("[STAGE5-DEC] Decode_Block_Two_Candidates FAILED\n");
        }
#endif
        return TrialOutcome{true, true, ExperimentConfig::kK};
    }

    int bit_err_0 = 0;
    int bit_err_1 = 0;
    for (int i = 0; i < ExperimentConfig::kK; ++i) {
        if (bits_cand0[i] != data_bits[i])
            ++bit_err_0;
        if (bits_cand1[i] != data_bits[i])
            ++bit_err_1;
    }
    const int bit_err = (bit_err_0 <= bit_err_1) ? bit_err_0 : bit_err_1;

#if defined(HTS_PIPELINE_DIAG)
    if (diag_print) {
        std::printf(
            "[STAGE5-DEC] cand0_bits[0..7]=%d,%d,%d,%d,%d,%d,%d,%d  "
            "cand1_bits[0..7]=%d,%d,%d,%d,%d,%d,%d,%d\n",
            static_cast<int>(bits_cand0[0]), static_cast<int>(bits_cand0[1]),
            static_cast<int>(bits_cand0[2]), static_cast<int>(bits_cand0[3]),
            static_cast<int>(bits_cand0[4]), static_cast<int>(bits_cand0[5]),
            static_cast<int>(bits_cand0[6]), static_cast<int>(bits_cand0[7]),
            static_cast<int>(bits_cand1[0]), static_cast<int>(bits_cand1[1]),
            static_cast<int>(bits_cand1[2]), static_cast<int>(bits_cand1[3]),
            static_cast<int>(bits_cand1[4]), static_cast<int>(bits_cand1[5]),
            static_cast<int>(bits_cand1[6]), static_cast<int>(bits_cand1[7]));
        std::printf(
            "[STAGE6-OK] bit_err=%d (cand0=%d, cand1=%d) PASS=%d\n", bit_err,
            bit_err_0, bit_err_1, (bit_err == 0) ? 1 : 0);
    }
#else
    (void)bit_err_0;
    (void)bit_err_1;
#endif

    return TrialOutcome{true, true, bit_err};
}

static TestResult run_one_cfo_v4(HTS_Holo_Tensor_4D_TX& tx_tensor,
                                 HTS_Holo_Tensor_4D_RX& rx_tensor, int cfo_hz,
                                 int snr_db, int num_trials) {
    TestResult res = {0, 0, 0, 0};
    for (int trial = 0; trial < num_trials; ++trial) {
        const TrialOutcome o = run_single_trial_v4(
            tx_tensor, rx_tensor, cfo_hz, snr_db, trial, false);
        if (!o.encode_ok)
            continue;
        if (o.sync_ok)
            ++res.sync_pass;
        res.total_bits += ExperimentConfig::kK;
        res.total_bit_err += o.bit_err;
        if (o.bit_err == 0)
            ++res.decode_pass;
    }
    return res;
}

#if defined(HTS_PIPELINE_DIAG)
static void run_pipeline_diag_block(HTS_Holo_Tensor_4D_TX& tx_tensor,
                                    HTS_Holo_Tensor_4D_RX& rx_tensor) {
    std::printf(
        "\n======== HTS_PIPELINE_DIAG: 5 CFO x 3 trials @ %d dB ========\n",
        ExperimentConfig::kPipelineSnrDb);
    for (int ci = 0; ci < ExperimentConfig::kPipelineCfoCount; ++ci) {
        const int cfo_hz = ExperimentConfig::kPipelineCfo[ci];
        for (int t = 0; t < ExperimentConfig::kPipelineTrials; ++t) {
            (void)run_single_trial_v4(tx_tensor, rx_tensor, cfo_hz,
                                      ExperimentConfig::kPipelineSnrDb, t,
                                      true);
        }
    }
    std::printf(
        "======== end HTS_PIPELINE_DIAG ========\n\n");
}
#endif

static void test_holo_tensor_cfo_sweep_v4() {
    std::printf("\n");
    std::printf(
        "============================================================\n");
    std::printf("  HTS_CFO_Bank_Test v4: production CFO_V5a + 128-chip preamble\n");
#if defined(HTS_BATCH_1)
    std::printf("  Sweep batch: 1  (0..1000 Hz @50Hz, %d pts)\n",
                ExperimentConfig::kCfoSweepCount);
#elif defined(HTS_BATCH_2)
    std::printf("  Sweep batch: 2  (1000..2000 Hz @50Hz, %d pts)\n",
                ExperimentConfig::kCfoSweepCount);
#elif defined(HTS_BATCH_3)
    std::printf("  Sweep batch: 3  (2000..3000 Hz @50Hz, %d pts)\n",
                ExperimentConfig::kCfoSweepCount);
#elif defined(HTS_BATCH_4)
    std::printf("  Sweep batch: 4  (3000..4000 Hz @50Hz, %d pts)\n",
                ExperimentConfig::kCfoSweepCount);
#elif defined(HTS_BATCH_5)
    std::printf("  Sweep batch: 5  (4000..5000 Hz @50Hz, %d pts)\n",
                ExperimentConfig::kCfoSweepCount);
#elif defined(HTS_BATCH_PTE_AB)
    std::printf("  Sweep batch: PTE_AB (9 cliff-near CFO pts)\n");
#elif defined(HTS_BATCH_LR_AB)
    std::printf("  Sweep batch: LR_AB (9 pts, LR/PTE lab compare)\n");
#else
    std::printf("  Sweep batch: default (27 pts)\n");
#endif
    std::printf("  HTS_HOLO_RX_PHASE_B_DEROTATE (this TU): %s\n",
#if defined(HTS_HOLO_RX_PHASE_B_DEROTATE)
                "ON"
#else
                "OFF"
#endif
    );
    std::printf("  SNR=30 dB only | trials=%d | sweep points=%d\n",
                ExperimentConfig::kNumTrialsSweep,
                ExperimentConfig::kCfoSweepCount);
    std::printf(
        "============================================================\n\n");

    HTS_Holo_Tensor_4D_TX tx_tensor;
    HTS_Holo_Tensor_4D_RX rx_tensor;
    uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5B5A5A5u,
                               0xC3D3C3C3u};
    if (tx_tensor.Initialize(master_seed, nullptr) !=
        HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        std::printf("TX Init failed\n");
        return;
    }
    if (rx_tensor.Initialize(master_seed, nullptr) !=
        HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        tx_tensor.Shutdown();
        std::printf("RX Init failed\n");
        return;
    }

#if defined(HTS_PIPELINE_DIAG)
    run_pipeline_diag_block(tx_tensor, rx_tensor);
#endif

    constexpr int kSnrSweep = 30;
    std::printf("%-10s  %s\n", "cfo Hz", "sync/dec/100  BER@30dB");
    std::printf(
        "------------------------------------------------------------\n");
    for (int c = 0; c < ExperimentConfig::kCfoSweepCount; ++c) {
        const int cfo_hz = ExperimentConfig::kCfoSweepList[c];
        const TestResult r = run_one_cfo_v4(tx_tensor, rx_tensor, cfo_hz,
                                            kSnrSweep,
                                            ExperimentConfig::kNumTrialsSweep);
        const double ber =
            (r.total_bits > 0)
                ? static_cast<double>(r.total_bit_err) / r.total_bits
                : 0.0;
        std::printf("  %-8d  %3d/%3d/100  %6.4f\n", cfo_hz, r.sync_pass,
                    r.decode_pass, ber);
    }
    std::printf(
        "------------------------------------------------------------\n");
    std::printf("sync/dec = HOLO sync pass / decode perfect trials (of 100)\n");

    rx_tensor.Shutdown();
    tx_tensor.Shutdown();
}

int main() {
    test_holo_tensor_cfo_sweep_v4();
    return 0;
}

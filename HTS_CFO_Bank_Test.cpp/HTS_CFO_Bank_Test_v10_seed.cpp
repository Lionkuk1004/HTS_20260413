// ============================================================================
// HTS_CFO_Bank_Test_v10_seed.cpp — Step 2D Lab: TX/RX master_seed[0] mismatch
// v4 동일 경로: preamble + payload, cfo=0, timing=0, ψ=0, SNR=30 dB.
// TX master_seed 고정, RX 만 master_seed[0] 변형 (word[1..3] 동일).
// Initialize() 는 재호출 시 시드를 갱신하지 않으므로 시나리오마다
// TX/RX Shutdown 후 재 Initialize. 양산 코드 미변경 (Lab 전용).
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

using namespace ProtectedEngine;

namespace ExperimentConfig {
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
constexpr int kTotalChips = kPreambleChips + kPayloadChips;
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

struct TrialOutcome {
    bool encode_ok;
    bool sync_ok;
    int bit_err;
};

static TrialOutcome run_single_trial_v10(HTS_Holo_Tensor_4D_TX& tx_tensor,
                                         HTS_Holo_Tensor_4D_RX& rx_tensor,
                                         int snr_db, int trial) {
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

    int16_t rx_full_I[192], rx_full_Q[192];
    const uint32_t ch_seed = static_cast<uint32_t>(trial * 31u + 19u);
    channel_apply(full_I, full_Q, rx_full_I, rx_full_Q,
                  ExperimentConfig::kTotalChips, 0.0, snr_db, ch_seed);

    bool sync_ok = true;
    if (ExperimentConfig::kEnableHoloSync) {
        sync_ok = holo_sync_check(rx_full_I, rx_full_Q);
    }
    if (!sync_ok) {
        return TrialOutcome{true, false, ExperimentConfig::kK};
    }

    hts::rx_cfo::CFO_V5a v5a;
    v5a.Init();
#if (HTS_CFO_V5A_ENABLE != 0)
    v5a.SetEnabled(true);
#endif

    const hts::rx_cfo::CFO_Result cfo_res =
        v5a.Estimate(rx_full_I, rx_full_Q);

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

    return TrialOutcome{true, true, bit_err};
}

struct Agg {
    int sync_pass{ 0 };
    int decode_pass{ 0 };
};

struct SeedTest {
    uint32_t rx_w0;
    const char* name;
};

static Agg run_scenario(HTS_Holo_Tensor_4D_TX& tx_tensor,
                        HTS_Holo_Tensor_4D_RX& rx_tensor, int snr_db,
                        int num_trials) noexcept {
    Agg a{};
    for (int trial = 0; trial < num_trials; ++trial) {
        const TrialOutcome o =
            run_single_trial_v10(tx_tensor, rx_tensor, snr_db, trial);
        if (!o.encode_ok)
            continue;
        if (o.sync_ok)
            ++a.sync_pass;
        if (o.bit_err == 0)
            ++a.decode_pass;
    }
    return a;
}

static const char* row_tag(int dec) noexcept {
    if (dec >= 80)
        return "OK";
    if (dec <= 10)
        return "cliff";
    return "mid";
}

}  // namespace

int main() {
    constexpr int kTrials = 100;
    constexpr int kSnrDb = 30;
    constexpr uint32_t kTx0 = 0xDEADBEEFu;
    constexpr uint32_t kW1 = 0x9E2779B9u;
    constexpr uint32_t kW2 = 0xA5A5A5A5u;
    constexpr uint32_t kW3 = 0xC3D3C3C3u;
    const uint32_t tx_master[4] = {kTx0, kW1, kW2, kW3};

    static const SeedTest kTests[] = {
        {kTx0, "match (baseline)"},
        {kTx0 ^ 1u, "1 bit (LSB)"},
        {0xDEADBEEDu, "literal 0xDEADBEED"},
        {0xDEADBEFFu, "literal 0xDEADBEFF"},
        {kTx0 ^ (1u << 3), "1 bit (bit 3)"},
        {kTx0 ^ (1u << 7), "1 bit (bit 7)"},
        {kTx0 ^ (1u << 15), "1 bit (bit 15)"},
        {kTx0 ^ (1u << 23), "1 bit (bit 23)"},
        {kTx0 ^ (1u << 31), "1 bit (bit 31)"},
        {kTx0 ^ 0xFFu, "XOR 0xFF (low 8)"},
        {kTx0 ^ 0xFFFFu, "XOR 0xFFFF (low 16)"},
        {kTx0 ^ 0xFFFFFFFFu, "XOR 0xFFFFFFFF (invert w0)"},
        {0x12345678u, "completely different w0"},
    };
    const int nTests = static_cast<int>(sizeof(kTests) / sizeof(kTests[0]));

    std::printf(
        "=== Step 2D SEED mismatch sweep (cfo=0, timing=0, psi=0, SNR=%d dB) "
        "===\n",
        kSnrDb);
    std::printf(
        "TX master_seed[0..3] = %08X %08X %08X %08X (fixed); RX differs only "
        "in [0] unless noted.\n",
        tx_master[0], tx_master[1], tx_master[2], tx_master[3]);
    std::printf(
        "SEED drives Holo4D_Generate_Partitioned_Params + "
        "Holo4D_Generate_Phase_Mask (TX/RX .cpp).\n");
    std::printf("dec/sync = decode perfect / HOLO sync pass (of %d)\n\n",
                kTrials);
    std::printf("%-32s | RX[0]    | dec/100 | sync/100 | tag\n", "name");
    std::printf("--------------------------------+----------+---------+----------+------\n");

    HTS_Holo_Tensor_4D_TX tx_tensor;
    HTS_Holo_Tensor_4D_RX rx_tensor;

    for (int i = 0; i < nTests; ++i) {
        uint32_t rx_master[4] = {kTests[i].rx_w0, kW1, kW2, kW3};

        rx_tensor.Shutdown();
        tx_tensor.Shutdown();

        if (tx_tensor.Initialize(tx_master, nullptr) !=
            HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            std::printf("TX Init failed (%s)\n", kTests[i].name);
            return 1;
        }
        if (rx_tensor.Initialize(rx_master, nullptr) !=
            HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            std::printf("RX Init failed (%s)\n", kTests[i].name);
            tx_tensor.Shutdown();
            return 1;
        }

        const Agg a =
            run_scenario(tx_tensor, rx_tensor, kSnrDb, kTrials);
        std::printf("%-32s | %08X | %3d/100 | %3d/100 | %s\n", kTests[i].name,
                    kTests[i].rx_w0, a.decode_pass, a.sync_pass,
                    row_tag(a.decode_pass));
    }

    std::printf("--------------------------------+----------+---------+----------+------\n");

    rx_tensor.Shutdown();
    tx_tensor.Shutdown();
    return 0;
}

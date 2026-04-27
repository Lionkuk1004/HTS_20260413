// ============================================================================
// HTS_CFO_Bank_Test_v11_phase_iq.cpp — Step B-A Lab: ψ sweep with I≠Q TX
// v9 와 동일 RX 경로; preamble 은 I/Q 서로 다른 Walsh row, payload 는
// Encode_Block I 에 대해 Q = -I (항상 I≠Q 를 많이 만족). ψ 24점 sweep.
// 양산 코드 미변경 (Lab 전용).
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
#if defined(HTS_USE_4D_DIVERSITY)
#include "HTS_Secure_Memory.h"
#endif

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

/// Sylvester-style 64-chip Walsh: chip c, row r → (-1)^{popcount(c & r)}
static int16_t walsh64_row_chip(uint32_t chip_idx, uint32_t row_sel,
                                int amp) noexcept {
    const uint32_t c = chip_idx & 63u;
    const uint32_t masked = c & (row_sel & 63u);
    const int parity = static_cast<int>(std::popcount(masked) & 1u);
    return static_cast<int16_t>(parity ? -amp : amp);
}

/// Preamble: I/Q 다른 row (0..63: I=row63, Q=row21; 64..127: I=+amp, Q=row62)
static void build_walsh128_preamble_i_ne_q(int16_t* pre_I, int16_t* pre_Q,
                                           int amp) noexcept {
    constexpr uint32_t kRowI0 = 63u;
    constexpr uint32_t kRowQ0 = 21u;
    constexpr uint32_t kRowQ1 = 62u;
    for (int c = 0; c < 64; ++c) {
        const uint32_t u = static_cast<uint32_t>(c);
        pre_I[c] = walsh64_row_chip(u, kRowI0, amp);
        pre_Q[c] = walsh64_row_chip(u, kRowQ0, amp);
    }
    for (int c = 0; c < 64; ++c) {
        const uint32_t u = static_cast<uint32_t>(64 + c);
        pre_I[64 + c] = static_cast<int16_t>(amp);
        pre_Q[64 + c] = walsh64_row_chip(u, kRowQ1, amp);
    }
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

static void apply_carrier_phase(const int16_t* in_I, const int16_t* in_Q,
                                int16_t* out_I, int16_t* out_Q, int n_chips,
                                double psi_rad) noexcept {
    if (n_chips <= 0)
        return;
    int32_t cos_q15 =
        static_cast<int32_t>(std::cos(psi_rad) * 32768.0 + 0.5);
    int32_t sin_q15 =
        static_cast<int32_t>(std::sin(psi_rad) * 32768.0 + 0.5);
    if (cos_q15 > 32767)
        cos_q15 = 32767;
    if (cos_q15 < -32768)
        cos_q15 = -32768;
    if (sin_q15 > 32767)
        sin_q15 = 32767;
    if (sin_q15 < -32768)
        sin_q15 = -32768;
    for (int n = 0; n < n_chips; ++n) {
        const int32_t i_in = static_cast<int32_t>(in_I[n]);
        const int32_t q_in = static_cast<int32_t>(in_Q[n]);
        const int64_t i_out_q15 =
            static_cast<int64_t>(i_in) * cos_q15 -
            static_cast<int64_t>(q_in) * sin_q15;
        const int64_t q_out_q15 =
            static_cast<int64_t>(i_in) * sin_q15 +
            static_cast<int64_t>(q_in) * cos_q15;
        int32_t oi = static_cast<int32_t>(i_out_q15 >> 15);
        int32_t oq = static_cast<int32_t>(q_out_q15 >> 15);
        if (oi > 32767)
            oi = 32767;
        if (oi < -32768)
            oi = -32768;
        if (oq > 32767)
            oq = 32767;
        if (oq < -32768)
            oq = -32768;
        out_I[n] = static_cast<int16_t>(oi);
        out_Q[n] = static_cast<int16_t>(oq);
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

#if defined(HTS_USE_4D_DIVERSITY)
static uint32_t lab_rx_decode_4d_diversity_slots(
    HTS_Holo_Tensor_4D_RX& rx,
    uint32_t rx_slot,
    const int16_t* corr_I,
    const int16_t* corr_Q,
    uint16_t N_chip,
    uint16_t K_bits,
    int8_t* out_sym0,
    int8_t* out_sym1) noexcept
{
    int8_t cand0_I[HOLO_MAX_BLOCK_BITS];
    int8_t cand1_I[HOLO_MAX_BLOCK_BITS];
    int32_t metric_I[HOLO_MAX_BLOCK_BITS];
    int8_t cand0_Q[HOLO_MAX_BLOCK_BITS];
    int8_t cand1_Q[HOLO_MAX_BLOCK_BITS];
    int32_t metric_Q[HOLO_MAX_BLOCK_BITS];
    (void)rx.Set_Time_Slot(rx_slot);
    const uint32_t ok_I = rx.Decode_Block_Two_Candidates_With_Metric(
        corr_I, corr_Q, N_chip, 0xFFFFFFFFFFFFFFFFull, cand0_I, cand1_I, metric_I,
        K_bits);
    (void)rx.Set_Time_Slot(rx_slot ^ 1u);
    const uint32_t ok_Q = rx.Decode_Block_Two_Candidates_With_Metric(
        corr_I, corr_Q, N_chip, 0xFFFFFFFFFFFFFFFFull, cand0_Q, cand1_Q, metric_Q,
        K_bits);
    (void)rx.Set_Time_Slot(rx_slot);
    if (ok_I != HTS_Holo_Tensor_4D_RX::SECURE_TRUE ||
        ok_Q != HTS_Holo_Tensor_4D_RX::SECURE_TRUE) {
        SecureMemory::secureWipe(static_cast<void*>(metric_I), sizeof(metric_I));
        SecureMemory::secureWipe(static_cast<void*>(metric_Q), sizeof(metric_Q));
        SecureMemory::secureWipe(static_cast<void*>(cand0_I), sizeof(cand0_I));
        SecureMemory::secureWipe(static_cast<void*>(cand1_I), sizeof(cand1_I));
        SecureMemory::secureWipe(static_cast<void*>(cand0_Q), sizeof(cand0_Q));
        SecureMemory::secureWipe(static_cast<void*>(cand1_Q), sizeof(cand1_Q));
        return HTS_Holo_Tensor_4D_RX::SECURE_FALSE;
    }
    for (uint16_t k = 0u; k < K_bits; ++k) {
        const int64_t llr =
            static_cast<int64_t>(metric_I[static_cast<size_t>(k)]) +
            static_cast<int64_t>(metric_Q[static_cast<size_t>(k)]);
        const uint64_t sign_u =
            static_cast<uint64_t>(static_cast<uint64_t>(llr) >> 63u);
        const int8_t cand0 =
            static_cast<int8_t>(1 - 2 * static_cast<int32_t>(sign_u));
        out_sym0[static_cast<size_t>(k)] = cand0;
        out_sym1[static_cast<size_t>(k)] = static_cast<int8_t>(-cand0);
    }
    SecureMemory::secureWipe(static_cast<void*>(metric_I), sizeof(metric_I));
    SecureMemory::secureWipe(static_cast<void*>(metric_Q), sizeof(metric_Q));
    SecureMemory::secureWipe(static_cast<void*>(cand0_I), sizeof(cand0_I));
    SecureMemory::secureWipe(static_cast<void*>(cand1_I), sizeof(cand1_I));
    SecureMemory::secureWipe(static_cast<void*>(cand0_Q), sizeof(cand0_Q));
    SecureMemory::secureWipe(static_cast<void*>(cand1_Q), sizeof(cand1_Q));
    return HTS_Holo_Tensor_4D_RX::SECURE_TRUE;
}
#endif

static int tx_build_tensor(HTS_Holo_Tensor_4D_TX& tensor, uint32_t tx_slot,
                           const int8_t* data_bits, int16_t* tx_I,
                           int16_t* tx_Q, int amp) noexcept {
#if defined(HTS_USE_4D_DIVERSITY)
    int8_t chip_bpsk_I[HOLO_CHIP_COUNT];
    int8_t chip_bpsk_Q[HOLO_CHIP_COUNT];
    (void)tensor.Set_Time_Slot(tx_slot);
    if (tensor.Encode_Block(
            data_bits, static_cast<uint16_t>(ExperimentConfig::kK), chip_bpsk_I,
            static_cast<uint16_t>(ExperimentConfig::kN)) !=
        HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        SecureMemory::secureWipe(static_cast<void*>(chip_bpsk_I),
                                 sizeof(chip_bpsk_I));
        return 0;
    }
    (void)tensor.Set_Time_Slot(tx_slot ^ 1u);
    if (tensor.Encode_Block(
            data_bits, static_cast<uint16_t>(ExperimentConfig::kK), chip_bpsk_Q,
            static_cast<uint16_t>(ExperimentConfig::kN)) !=
        HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        SecureMemory::secureWipe(static_cast<void*>(chip_bpsk_I),
                                 sizeof(chip_bpsk_I));
        SecureMemory::secureWipe(static_cast<void*>(chip_bpsk_Q),
                                 sizeof(chip_bpsk_Q));
        (void)tensor.Set_Time_Slot(tx_slot);
        return 0;
    }
    (void)tensor.Set_Time_Slot(tx_slot);
    const int denom = ExperimentConfig::kDenom;
    const int half_denom = denom >> 1;
    for (int c = 0; c < ExperimentConfig::kN; ++c) {
        const int32_t prod_I =
            static_cast<int32_t>(chip_bpsk_I[c]) * static_cast<int32_t>(amp);
        const int32_t v_I =
            (prod_I >= 0) ? ((prod_I + half_denom) / denom)
                          : ((prod_I - half_denom) / denom);
        const int32_t prod_Q =
            static_cast<int32_t>(chip_bpsk_Q[c]) * static_cast<int32_t>(amp);
        const int32_t v_Q =
            (prod_Q >= 0) ? ((prod_Q + half_denom) / denom)
                          : ((prod_Q - half_denom) / denom);
        tx_I[c] = static_cast<int16_t>(v_I);
        tx_Q[c] = static_cast<int16_t>(-v_Q);
    }
    SecureMemory::secureWipe(static_cast<void*>(chip_bpsk_I), sizeof(chip_bpsk_I));
    SecureMemory::secureWipe(static_cast<void*>(chip_bpsk_Q), sizeof(chip_bpsk_Q));
    return ExperimentConfig::kN;
#else
    (void)tx_slot;
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
        tx_Q[c] = static_cast<int16_t>(-v);
    }
    return ExperimentConfig::kN;
#endif
}

struct TrialOutcome {
    bool encode_ok;
    bool sync_ok;
    int bit_err;
};

static TrialOutcome run_single_trial_v11(HTS_Holo_Tensor_4D_TX& tx_tensor,
                                         HTS_Holo_Tensor_4D_RX& rx_tensor,
                                         int snr_db, int trial,
                                         double psi_rad) {
    rng_seed(static_cast<uint32_t>(trial * 7919u + 13u));
    int8_t data_bits[16];
    for (int i = 0; i < ExperimentConfig::kK; ++i) {
        data_bits[i] = (rng_next() & 1u) ? 1 : -1;
    }

    int16_t pre_I[128], pre_Q[128];
    build_walsh128_preamble_i_ne_q(pre_I, pre_Q, ExperimentConfig::kAmp);

    (void)tx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
    int16_t pay_I[64], pay_Q[64];
    if (tx_build_tensor(tx_tensor, static_cast<uint32_t>(trial), data_bits, pay_I,
                        pay_Q, ExperimentConfig::kAmp) == 0) {
        return TrialOutcome{false, false, ExperimentConfig::kK};
    }

    int16_t full_I[192], full_Q[192];
    std::memcpy(full_I, pre_I, sizeof(int16_t) * 128u);
    std::memcpy(full_I + 128, pay_I, sizeof(int16_t) * 64u);
    std::memcpy(full_Q, pre_Q, sizeof(int16_t) * 128u);
    std::memcpy(full_Q + 128, pay_Q, sizeof(int16_t) * 64u);

    int16_t phased_I[192], phased_Q[192];
    apply_carrier_phase(full_I, full_Q, phased_I, phased_Q,
                        ExperimentConfig::kTotalChips, psi_rad);

    int16_t rx_full_I[192], rx_full_Q[192];
    const uint32_t ch_seed = static_cast<uint32_t>(trial * 31u + 19u);
    channel_apply(phased_I, phased_Q, rx_full_I, rx_full_Q,
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
#if defined(HTS_USE_4D_DIVERSITY)
    if (lab_rx_decode_4d_diversity_slots(
            rx_tensor, static_cast<uint32_t>(trial), corr_I, corr_Q,
            static_cast<uint16_t>(ExperimentConfig::kN),
            static_cast<uint16_t>(ExperimentConfig::kK), bits_cand0,
            bits_cand1) != HTS_Holo_Tensor_4D_RX::SECURE_TRUE) {
        return TrialOutcome{true, true, ExperimentConfig::kK};
    }
#else
    if (rx_tensor.Decode_Block_Two_Candidates(
            corr_I, corr_Q,
            static_cast<uint16_t>(ExperimentConfig::kN), valid_mask,
            bits_cand0, bits_cand1,
            static_cast<uint16_t>(ExperimentConfig::kK)) !=
        HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        return TrialOutcome{true, true, ExperimentConfig::kK};
    }
#endif

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

static Agg run_phase_psi(HTS_Holo_Tensor_4D_TX& tx_tensor,
                         HTS_Holo_Tensor_4D_RX& rx_tensor, double psi_rad,
                         int snr_db, int num_trials) noexcept {
    Agg a{};
    for (int trial = 0; trial < num_trials; ++trial) {
        const TrialOutcome o =
            run_single_trial_v11(tx_tensor, rx_tensor, snr_db, trial,
                                 psi_rad);
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

/// Step 2C Lab v9 (I=Q) decode/100 동일 순서 (고정 참조)
static constexpr int kLabV4Dec100[] = {
    100, 100, 100, 100, 100, 100, 21, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 17, 100, 100, 100, 100, 100};
static constexpr int kLabV4Sync100[] = {
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

}  // namespace

int main() {
    constexpr int kTrials = 100;
    constexpr int kSnrDb = 30;
    static const double kPhasesDeg[] = {
        0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0,
        120.0, 135.0, 150.0, 165.0, 180.0, 195.0, 210.0, 225.0,
        240.0, 255.0, 270.0, 285.0, 300.0, 315.0, 330.0, 345.0};
    const int nP =
        static_cast<int>(sizeof(kPhasesDeg) / sizeof(kPhasesDeg[0]));

    std::printf("=== Step B-A Phase sweep, TX I!=Q (Lab v11) ===\n");
    std::printf("Preamble: I=W64 row63, Q=row21 (ch0-63); I=+amp, Q=row62 "
                "(ch64-127).\n");
    std::printf("Payload: Q = -I after Encode_Block scaling.\n");
    std::printf("cfo=0, timing=0, SNR=%d dB, trials=%d/pt\n\n", kSnrDb,
                kTrials);

    HTS_Holo_Tensor_4D_TX tx_tensor;
    HTS_Holo_Tensor_4D_RX rx_tensor;
    uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5A5A5A5u,
                               0xC3D3C3C3u};
    if (tx_tensor.Initialize(master_seed, nullptr) !=
        HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        std::printf("TX Init failed\n");
        return 1;
    }
    if (rx_tensor.Initialize(master_seed, nullptr) !=
        HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        tx_tensor.Shutdown();
        std::printf("RX Init failed\n");
        return 1;
    }

    int16_t demo_I[192], demo_Q[192];
    build_walsh128_preamble_i_ne_q(demo_I, demo_Q, ExperimentConfig::kAmp);
    int8_t demo_bits[16];
    for (int b = 0; b < ExperimentConfig::kK; ++b) {
        demo_bits[b] = (b & 1) ? -1 : 1;
    }
    (void)tx_tensor.Set_Time_Slot(0u);
    int16_t pay_I[64], pay_Q[64];
    const int pay_ok =
        tx_build_tensor(tx_tensor, 0u, demo_bits, pay_I, pay_Q,
                        ExperimentConfig::kAmp);
    std::memcpy(demo_I + 128, pay_I, sizeof(int16_t) * 64u);
    std::memcpy(demo_Q + 128, pay_Q, sizeof(int16_t) * 64u);

    std::printf("TX signal verification (first 10 preamble chips):\n");
    int eq_pre = 0;
    for (int n = 0; n < 10; ++n) {
        if (demo_I[n] == demo_Q[n])
            ++eq_pre;
        std::printf("  chip %d: I=%6d, Q=%6d\n", n,
                    static_cast<int>(demo_I[n]),
                    static_cast<int>(demo_Q[n]));
    }
    std::printf(
        "(preamble 0-9: I==Q count = %d / 10; Walsh c=0 always same sign "
        "for any row)\n",
        eq_pre);
    if (pay_ok != 0) {
        std::printf("TX signal verification (payload chips 128-137, I!=Q "
                    "by construction):\n");
        int eq_pay = 0;
        for (int n = 0; n < 10; ++n) {
            const int idx = 128 + n;
            if (demo_I[idx] == demo_Q[idx])
                ++eq_pay;
            std::printf("  chip %d: I=%6d, Q=%6d\n", idx,
                        static_cast<int>(demo_I[idx]),
                        static_cast<int>(demo_Q[idx]));
        }
        std::printf("(payload window: I==Q count = %d / 10)\n\n", eq_pay);
    } else {
        std::printf("(encode verify skipped)\n\n");
    }

    std::printf(
        "%-7s | %-9s | %-9s | v4 dec | ddec | v4 sync | dsync | %s\n",
        "psi deg", "dec/100", "sync/100", "tag");
    std::printf("--------+-----------+-----------+--------+------+---------+-------+------\n");

    for (int i = 0; i < nP; ++i) {
        const double psi_deg = kPhasesDeg[i];
        const double psi_rad = psi_deg * kPi / 180.0;
        const Agg a = run_phase_psi(tx_tensor, rx_tensor, psi_rad, kSnrDb,
                                    kTrials);
        const int v4d = kLabV4Dec100[i];
        const int v4s = kLabV4Sync100[i];
        std::printf(
            "%5.1f | %3d/100 | %3d/100 | %6d | %+4d | %7d | %+5d | %s\n",
            psi_deg, a.decode_pass, a.sync_pass, v4d,
            a.decode_pass - v4d, v4s, a.sync_pass - v4s,
            row_tag(a.decode_pass));
    }
    std::printf("--------+-----------+-----------+--------+------+---------+-------+------\n");

    rx_tensor.Shutdown();
    tx_tensor.Shutdown();
    return 0;
}

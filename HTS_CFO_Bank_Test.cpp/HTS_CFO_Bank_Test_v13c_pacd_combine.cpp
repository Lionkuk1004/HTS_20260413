// ============================================================================
// HTS_CFO_Bank_Test_v13c_pacd_combine.cpp — Lab v13c: PaCD + Legacy combining
// BPTE(metric 합) 기반 분기 없는 비트 선택 + per-trial uniform SNR [-15,+35] dB.
// CFO=0, timing=0, MP 없음. 양산 코드 미변경 (Lab 전용).
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
constexpr int kNumTrials = 200;
constexpr int kNumPsi = 24;
constexpr double kSnrMin = -15.0;
constexpr double kSnrMax = 35.0;
constexpr int kNumBins = 10;

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
                          double cfo_hz, double snr_db,
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

static int tx_build_tensor(HTS_Holo_Tensor_4D_TX& tensor, uint32_t tx_slot,
                           const int8_t* data_bits, int16_t* tx_I,
                           int16_t* tx_Q, int amp) noexcept {
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
        tx_Q[c] = static_cast<int16_t>(v);
    }
    return ExperimentConfig::kN;
}

static int bit_err_vs_ref(const int8_t* cand, const int8_t* ref,
                          int k) noexcept {
    int e = 0;
    for (int i = 0; i < k; ++i) {
        if (cand[i] != ref[i])
            ++e;
    }
    return e;
}

static int pick_best_cand_err(const int8_t* c0, const int8_t* c1,
                              const int8_t* ref, int k) noexcept {
    const int e0 = bit_err_vs_ref(c0, ref, k);
    const int e1 = bit_err_vs_ref(c1, ref, k);
    return (e0 <= e1) ? e0 : e1;
}

static int64_t metric_abs_sum(const int32_t* m, int k) noexcept {
    int64_t s = 0;
    for (int i = 0; i < k; ++i) {
        const int32_t v = m[i];
        s += (v >= 0) ? static_cast<int64_t>(v) : -static_cast<int64_t>(v);
    }
    return s;
}

struct TrialDec {
    bool encode_ok;
    bool sync_ok;
    bool legacy_ok;
    bool pacd_ok;
    bool combined_ok;
};

static TrialDec run_single_trial_v13c(HTS_Holo_Tensor_4D_TX& tx_tensor,
                                      HTS_Holo_Tensor_4D_RX& rx_tensor,
                                      const int8_t* data_bits, int trial,
                                      int psi_idx, double psi_rad,
                                      double snr_db) noexcept {
    constexpr int kPre = ExperimentConfig::kPreambleChips;
    constexpr int kPay = ExperimentConfig::kPayloadChips;
    constexpr int kTot = ExperimentConfig::kTotalChips;
    static_assert(kPre > 0, "Preamble length must be valid");
    static_assert(kTot > kPre, "Total length must exceed preamble");

    int16_t pre_I[128], pre_Q[128];
    build_walsh128_preamble(pre_I, pre_Q, ExperimentConfig::kAmp);

    (void)tx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
    int16_t pay_I[64], pay_Q[64];
    if (tx_build_tensor(tx_tensor, static_cast<uint32_t>(trial), data_bits,
                        pay_I, pay_Q, ExperimentConfig::kAmp) == 0) {
        return TrialDec{false, false, false, false, false};
    }

    int16_t full_I[kTot], full_Q[kTot];
    std::memcpy(full_I, pre_I, sizeof(int16_t) * static_cast<size_t>(kPre));
    std::memcpy(full_I + kPre, pay_I, sizeof(int16_t) * static_cast<size_t>(kPay));
    std::memcpy(full_Q, pre_Q, sizeof(int16_t) * static_cast<size_t>(kPre));
    std::memcpy(full_Q + kPre, pay_Q, sizeof(int16_t) * static_cast<size_t>(kPay));

    int16_t phased_I[kTot], phased_Q[kTot];
    apply_carrier_phase(full_I, full_Q, phased_I, phased_Q, kTot, psi_rad);

    int16_t rx_full_I[kTot], rx_full_Q[kTot];
    const uint32_t ch_seed =
        static_cast<uint32_t>(trial * 31u + 19u + psi_idx * 65537u);
    channel_apply(phased_I, phased_Q, rx_full_I, rx_full_Q, kTot, 0.0, snr_db,
                  ch_seed);

    bool sync_ok = true;
    if (ExperimentConfig::kEnableHoloSync) {
        sync_ok = holo_sync_check(rx_full_I, rx_full_Q);
    }
    if (!sync_ok) {
        return TrialDec{true, false, false, false, false};
    }

    hts::rx_cfo::CFO_V5a v5a;
    v5a.Init();
#if (HTS_CFO_V5A_ENABLE != 0)
    v5a.SetEnabled(true);
#endif
    const hts::rx_cfo::CFO_Result cfo_res =
        v5a.Estimate(rx_full_I, rx_full_Q);

    int16_t buf_I[kTot], buf_Q[kTot];
    std::memcpy(buf_I, rx_full_I, sizeof(int16_t) * static_cast<size_t>(kPre));
    std::memcpy(buf_Q, rx_full_Q, sizeof(int16_t) * static_cast<size_t>(kPre));

    if (cfo_res.valid && v5a.IsApplyAllowed()) {
        v5a.Set_Apply_Cfo(cfo_res.cfo_hz);
        v5a.Advance_Phase_Only(kPre);
        for (int c = 0; c < kPay; ++c) {
            int16_t in_I = rx_full_I[kPre + c];
            int16_t in_Q = rx_full_Q[kPre + c];
            v5a.Apply_Per_Chip(in_I, in_Q);
            buf_I[kPre + c] = in_I;
            buf_Q[kPre + c] = in_Q;
        }
    } else {
        std::memcpy(buf_I + kPre, rx_full_I + kPre,
                    sizeof(int16_t) * static_cast<size_t>(kPay));
        std::memcpy(buf_Q + kPre, rx_full_Q + kPre,
                    sizeof(int16_t) * static_cast<size_t>(kPay));
    }

    int64_t z_re = 0;
    int64_t z_im = 0;
    for (int c = 0; c < kPre; ++c) {
        z_re += static_cast<int64_t>(buf_I[c]) * static_cast<int64_t>(pre_I[c]) +
                static_cast<int64_t>(buf_Q[c]) * static_cast<int64_t>(pre_Q[c]);
        z_im += static_cast<int64_t>(buf_Q[c]) * static_cast<int64_t>(pre_I[c]) -
                static_cast<int64_t>(buf_I[c]) * static_cast<int64_t>(pre_Q[c]);
    }
    const double psi_hat =
        std::atan2(static_cast<double>(z_im), static_cast<double>(z_re));
    const double cos_psi = std::cos(psi_hat);
    const double sin_psi = std::sin(psi_hat);

    int16_t buf_orig_I[kTot];
    int16_t buf_orig_Q[kTot];
    std::memcpy(buf_orig_I, buf_I, sizeof(buf_orig_I));
    std::memcpy(buf_orig_Q, buf_Q, sizeof(buf_orig_Q));

    for (int c = kPre; c < kTot; ++c) {
        const double orig_I = static_cast<double>(buf_I[c]);
        const double orig_Q = static_cast<double>(buf_Q[c]);
        long long di = std::llround(orig_I * cos_psi + orig_Q * sin_psi);
        long long dq = std::llround(orig_Q * cos_psi - orig_I * sin_psi);
        if (di > 32767)
            di = 32767;
        if (di < -32768)
            di = -32768;
        if (dq > 32767)
            dq = 32767;
        if (dq < -32768)
            dq = -32768;
        buf_I[c] = static_cast<int16_t>(di);
        buf_Q[c] = static_cast<int16_t>(dq);
    }

    (void)rx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
    const uint64_t valid_mask = 0xFFFFFFFFFFFFFFFFull;
    const uint16_t Kbits = static_cast<uint16_t>(ExperimentConfig::kK);
    const uint16_t Nchip = static_cast<uint16_t>(ExperimentConfig::kN);

    int8_t leg_c0[HOLO_MAX_BLOCK_BITS];
    int8_t leg_c1[HOLO_MAX_BLOCK_BITS];
    int32_t metric_legacy[HOLO_MAX_BLOCK_BITS];
    int8_t pacd_c0[HOLO_MAX_BLOCK_BITS];
    int8_t pacd_c1[HOLO_MAX_BLOCK_BITS];
    int32_t metric_pacd[HOLO_MAX_BLOCK_BITS];

    const uint32_t ok_leg = rx_tensor.Decode_Block_Two_Candidates_With_Metric(
        buf_orig_I + kPre, buf_orig_Q + kPre, Nchip, valid_mask, leg_c0,
        leg_c1, metric_legacy, Kbits);
    const uint32_t ok_pacd = rx_tensor.Decode_Block_Two_Candidates_With_Metric(
        buf_I + kPre, buf_Q + kPre, Nchip, valid_mask, pacd_c0, pacd_c1,
        metric_pacd, Kbits);

    const int best_leg_err =
        (ok_leg == HTS_Holo_Tensor_4D_RX::SECURE_TRUE)
            ? pick_best_cand_err(leg_c0, leg_c1, data_bits, ExperimentConfig::kK)
            : ExperimentConfig::kK;
    const int best_pacd_err =
        (ok_pacd == HTS_Holo_Tensor_4D_RX::SECURE_TRUE)
            ? pick_best_cand_err(pacd_c0, pacd_c1, data_bits,
                                 ExperimentConfig::kK)
            : ExperimentConfig::kK;

    const bool legacy_ok =
        (ok_leg == HTS_Holo_Tensor_4D_RX::SECURE_TRUE) && (best_leg_err == 0);
    const bool pacd_ok =
        (ok_pacd == HTS_Holo_Tensor_4D_RX::SECURE_TRUE) && (best_pacd_err == 0);

    int8_t bits_legacy_pick[HOLO_MAX_BLOCK_BITS];
    int8_t bits_pacd_pick[HOLO_MAX_BLOCK_BITS];
    if (ok_leg == HTS_Holo_Tensor_4D_RX::SECURE_TRUE) {
        if (bit_err_vs_ref(leg_c0, data_bits, ExperimentConfig::kK) <=
            bit_err_vs_ref(leg_c1, data_bits, ExperimentConfig::kK)) {
            std::memcpy(bits_legacy_pick, leg_c0,
                        sizeof(int8_t) * static_cast<size_t>(ExperimentConfig::kK));
        } else {
            std::memcpy(bits_legacy_pick, leg_c1,
                        sizeof(int8_t) * static_cast<size_t>(ExperimentConfig::kK));
        }
    } else {
        std::memset(bits_legacy_pick, 0,
                    sizeof(int8_t) * static_cast<size_t>(ExperimentConfig::kK));
    }
    if (ok_pacd == HTS_Holo_Tensor_4D_RX::SECURE_TRUE) {
        if (bit_err_vs_ref(pacd_c0, data_bits, ExperimentConfig::kK) <=
            bit_err_vs_ref(pacd_c1, data_bits, ExperimentConfig::kK)) {
            std::memcpy(bits_pacd_pick, pacd_c0,
                        sizeof(int8_t) * static_cast<size_t>(ExperimentConfig::kK));
        } else {
            std::memcpy(bits_pacd_pick, pacd_c1,
                        sizeof(int8_t) * static_cast<size_t>(ExperimentConfig::kK));
        }
    } else {
        std::memset(bits_pacd_pick, 0,
                    sizeof(int8_t) * static_cast<size_t>(ExperimentConfig::kK));
    }

    bool combined_ok = false;
    if (ok_leg == HTS_Holo_Tensor_4D_RX::SECURE_TRUE &&
        ok_pacd == HTS_Holo_Tensor_4D_RX::SECURE_TRUE) {
        const int64_t sum_legacy =
            metric_abs_sum(metric_legacy, ExperimentConfig::kK);
        const int64_t sum_pacd =
            metric_abs_sum(metric_pacd, ExperimentConfig::kK);
        const int64_t diff = sum_pacd - sum_legacy;
        const int32_t mask_legacy =
            static_cast<int32_t>(diff >> 63);
        const int32_t mask_pacd = ~mask_legacy;

        int8_t bits_combined[HOLO_MAX_BLOCK_BITS];
        for (int k = 0; k < ExperimentConfig::kK; ++k) {
            bits_combined[k] = static_cast<int8_t>(
                (bits_pacd_pick[k] & static_cast<int8_t>(mask_pacd)) |
                (bits_legacy_pick[k] & static_cast<int8_t>(mask_legacy)));
        }
        combined_ok =
            (bit_err_vs_ref(bits_combined, data_bits, ExperimentConfig::kK) == 0);
    }

    std::memset(buf_orig_I, 0, sizeof(buf_orig_I));
    std::memset(buf_orig_Q, 0, sizeof(buf_orig_Q));
    std::memset(metric_legacy, 0, sizeof(int32_t) * HOLO_MAX_BLOCK_BITS);
    std::memset(metric_pacd, 0, sizeof(int32_t) * HOLO_MAX_BLOCK_BITS);
    std::memset(leg_c0, 0, sizeof(leg_c0));
    std::memset(leg_c1, 0, sizeof(leg_c1));
    std::memset(pacd_c0, 0, sizeof(pacd_c0));
    std::memset(pacd_c1, 0, sizeof(pacd_c1));

    return TrialDec{true, true, legacy_ok, pacd_ok, combined_ok};
}

struct BinStat {
    uint64_t count{ 0 };
    double snr_sum{ 0.0 };
    uint64_t legacy_ok{ 0 };
    uint64_t pacd_ok{ 0 };
    uint64_t combined_ok{ 0 };
};

static int snr_bin_index(double snr_db) noexcept {
    int b = static_cast<int>(
        std::floor((snr_db - kSnrMin) / 5.0));
    if (b < 0)
        b = 0;
    if (b >= kNumBins)
        b = kNumBins - 1;
    return b;
}

static double bin_snr_low(int bin) noexcept {
    return kSnrMin + 5.0 * static_cast<double>(bin);
}

static void print_markdown_header() noexcept {
    std::printf("## Lab v13c PaCD + Combining + Random SNR\n\n");
    std::printf("### 빌드: OK (Lab TU)\n\n");
    std::printf("### Bin 별 통계\n\n");
    std::printf("| Bin | SNR range | trials | actual_avg | legacy%% | pacd%% | "
                "combined%% | gain |\n");
    std::printf("|-----|-----------|--------|------------|---------|-------|"
                "-----------|------|\n");
}

static double interp_threshold_snr_path(const BinStat* bins, int nbin,
                                        double target_pct,
                                        int path) noexcept {
    for (int b = 0; b < nbin; ++b) {
        if (bins[b].count == 0u)
            continue;
        uint64_t ok = 0u;
        if (path == 0)
            ok = bins[b].legacy_ok;
        else if (path == 1)
            ok = bins[b].pacd_ok;
        else
            ok = bins[b].combined_ok;
        const double p =
            100.0 * static_cast<double>(ok) /
            static_cast<double>(bins[b].count);
        if (p >= target_pct - 1e-9) {
            return bin_snr_low(b) + 2.5;
        }
    }
    return kSnrMax + 2.5;
}

}  // namespace

int main() {
    static const double kPhasesDeg[] = {
        0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0,
        120.0, 135.0, 150.0, 165.0, 180.0, 195.0, 210.0, 225.0,
        240.0, 255.0, 270.0, 285.0, 300.0, 315.0, 330.0, 345.0};

    HTS_Holo_Tensor_4D_TX tx_tensor;
    HTS_Holo_Tensor_4D_RX rx_tensor;
    uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5B5A5A5u,
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

    BinStat bins[kNumBins]{};

    for (int trial = 0; trial < kNumTrials; ++trial) {
        rng_seed(static_cast<uint32_t>(trial * 7919u + 13u));
        const double snr_db =
            kSnrMin + (kSnrMax - kSnrMin) * rng_uniform();
        const int bin = snr_bin_index(snr_db);

        int8_t data_bits[ExperimentConfig::kK];
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            data_bits[i] = (rng_next() & 1u) ? 1 : -1;
        }

        for (int psi_idx = 0; psi_idx < kNumPsi; ++psi_idx) {
            const double psi_deg = kPhasesDeg[static_cast<size_t>(psi_idx)];
            const double psi_rad = psi_deg * kPi / 180.0;
            const TrialDec d = run_single_trial_v13c(
                tx_tensor, rx_tensor, data_bits, trial, psi_idx, psi_rad,
                snr_db);
            if (!d.encode_ok)
                continue;
            bins[bin].count += 1u;
            bins[bin].snr_sum += snr_db;
            if (d.sync_ok) {
                if (d.legacy_ok)
                    bins[bin].legacy_ok += 1u;
                if (d.pacd_ok)
                    bins[bin].pacd_ok += 1u;
                if (d.combined_ok)
                    bins[bin].combined_ok += 1u;
            }
        }
    }

    print_markdown_header();

    for (int b = 0; b < kNumBins; ++b) {
        const double lo = bin_snr_low(b);
        const double hi = lo + 5.0;
        if (bins[b].count == 0u) {
            std::printf("| %d | %.0f~%.0f | 0 | n/a | n/a | n/a | n/a | n/a "
                        "|\n",
                        b, lo, hi);
            continue;
        }
        const double avg_snr = bins[b].snr_sum / static_cast<double>(bins[b].count);
        const double leg_pct =
            100.0 * static_cast<double>(bins[b].legacy_ok) /
            static_cast<double>(bins[b].count);
        const double pacd_pct =
            100.0 * static_cast<double>(bins[b].pacd_ok) /
            static_cast<double>(bins[b].count);
        const double comb_pct =
            100.0 * static_cast<double>(bins[b].combined_ok) /
            static_cast<double>(bins[b].count);
        const double gain = comb_pct - (leg_pct > pacd_pct ? leg_pct : pacd_pct);
        std::printf("| %d | %.0f~%.0f | %llu | %+6.2f | %5.1f | %5.1f | %5.1f | "
                    "%+5.1f |\n",
                    b, lo, hi,
                    static_cast<unsigned long long>(bins[b].count), avg_snr,
                    leg_pct, pacd_pct, comb_pct, gain);
    }

    std::printf("\n### 핵심 비교 (bin 중심 SNR에서 첫 목표%% 도달)\n\n");
    const double t90_leg = interp_threshold_snr_path(bins, kNumBins, 90.0, 0);
    const double t90_pacd = interp_threshold_snr_path(bins, kNumBins, 90.0, 1);
    const double t90_comb = interp_threshold_snr_path(bins, kNumBins, 90.0, 2);
    const double t95_leg = interp_threshold_snr_path(bins, kNumBins, 95.0, 0);
    const double t95_pacd = interp_threshold_snr_path(bins, kNumBins, 95.0, 1);
    const double t95_comb = interp_threshold_snr_path(bins, kNumBins, 95.0, 2);
    const double t99_leg = interp_threshold_snr_path(bins, kNumBins, 99.0, 0);
    const double t99_pacd = interp_threshold_snr_path(bins, kNumBins, 99.0, 1);
    const double t99_comb = interp_threshold_snr_path(bins, kNumBins, 99.0, 2);

    std::printf("- 임계 SNR 추정 (90%% 블록 정확, bin 중심): legacy≈%.1f dB, "
                "PaCD≈%.1f dB, Combined≈%.1f dB\n",
                t90_leg, t90_pacd, t90_comb);
    std::printf("- 임계 SNR 추정 (95%%): legacy≈%.1f dB, PaCD≈%.1f dB, "
                "Combined≈%.1f dB\n",
                t95_leg, t95_pacd, t95_comb);
    std::printf("- 임계 SNR 추정 (99%%): legacy≈%.1f dB, PaCD≈%.1f dB, "
                "Combined≈%.1f dB\n",
                t99_leg, t99_pacd, t99_comb);
    {
        const double max_single =
            (t90_leg > t90_pacd ? t90_leg : t90_pacd);
        std::printf(
            "- @90%%: legacy 임계 − Combined 임계 ≈ %.1f dB (양수면 Combined가 "
            "더 낮은 SNR에서 동일 목표)\n",
            t90_leg - t90_comb);
        std::printf(
            "- @90%%: max(legacy,PaCD) 단독 임계 − Combined 임계 ≈ %.1f dB\n",
            max_single - t90_comb);
    }

    std::printf("\n### 통과 기준 표 (위 추정값 요약)\n\n");
    std::printf("| 임계 SNR | legacy | PaCD | Combined |\n");
    std::printf("|----------|--------|------|----------|\n");
    std::printf("| 90%% | ~%.1f | ~%.1f | ~%.1f |\n", t90_leg, t90_pacd, t90_comb);
    std::printf("| 95%% | ~%.1f | ~%.1f | ~%.1f |\n", t95_leg, t95_pacd, t95_comb);
    std::printf("| 99%% | ~%.1f | ~%.1f | ~%.1f |\n", t99_leg, t99_pacd, t99_comb);

    std::printf("\n### ψ 별 효과 (선택)\n\n");
    std::printf("전체 스윕은 24 psi 혼합; cliff(예: psi=90)는 별도 TU로 분리 "
                "측정 권장.\n");

    rx_tensor.Shutdown();
    tx_tensor.Shutdown();
    return 0;
}

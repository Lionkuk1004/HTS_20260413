// ============================================================================
// HTS_CFO_Bank_Test_v13_pacd.cpp — Lab v13: Pilot-aided Coherent Detection PoC
// v9 와 동일 TX(I=Q)·프리앰블·채널; RX 에서 프리앰블 상관으로 ψ_hat 추정 후
// 페이로드만 exp(-j·ψ_hat) 역회전 후 V5a·Two_Candidates. HTS_USE_4D_DIVERSITY 없음.
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

/// (I+jQ) × exp(jψ); Q15 cos/sin, int64 중간값, >>15 후 포화.
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

/// PaCD Step 1: z = Σ rx·conj(pre) over preamble chips (int64 누적).
static void pacd_preamble_cross_corr(const int16_t* rx_I, const int16_t* rx_Q,
                                     const int16_t* pre_I, const int16_t* pre_Q,
                                     int n_pre, int64_t* z_re,
                                     int64_t* z_im) noexcept {
    int64_t acc_re = 0;
    int64_t acc_im = 0;
    for (int c = 0; c < n_pre; ++c) {
        const int64_t rxI = static_cast<int64_t>(rx_I[c]);
        const int64_t rxQ = static_cast<int64_t>(rx_Q[c]);
        const int64_t pI = static_cast<int64_t>(pre_I[c]);
        const int64_t pQ = static_cast<int64_t>(pre_Q[c]);
        acc_re += rxI * pI + rxQ * pQ;
        acc_im += rxQ * pI - rxI * pQ;
    }
    *z_re = acc_re;
    *z_im = acc_im;
}

/// PaCD Step 2: (I+jQ) * exp(-j·psi_hat) per chip (payload 구간만).
static void pacd_derotate_payload(const int16_t* in_I, const int16_t* in_Q,
                                  int n_chip, double psi_hat, int16_t* out_I,
                                  int16_t* out_Q) noexcept {
    const double cs = std::cos(psi_hat);
    const double sn = std::sin(psi_hat);
    for (int c = 0; c < n_chip; ++c) {
        const double cI = static_cast<double>(in_I[c]);
        const double cQ = static_cast<double>(in_Q[c]);
        const double cor_I = cI * cs + cQ * sn;
        const double cor_Q = cQ * cs - cI * sn;
        long long ri = std::llround(cor_I);
        long long rq = std::llround(cor_Q);
        if (ri > 32767) {
            ri = 32767;
        }
        if (ri < -32768) {
            ri = -32768;
        }
        if (rq > 32767) {
            rq = 32767;
        }
        if (rq < -32768) {
            rq = -32768;
        }
        out_I[c] = static_cast<int16_t>(ri);
        out_Q[c] = static_cast<int16_t>(rq);
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

struct TrialOutcome {
    bool encode_ok;
    bool sync_ok;
    int bit_err;
};

static TrialOutcome run_single_trial_v13(HTS_Holo_Tensor_4D_TX& tx_tensor,
                                         HTS_Holo_Tensor_4D_RX& rx_tensor,
                                         int snr_db, int trial,
                                         double psi_rad) {
    rng_seed(static_cast<uint32_t>(trial * 7919u + 13u));
    int8_t data_bits[16];
    for (int i = 0; i < ExperimentConfig::kK; ++i) {
        data_bits[i] = (rng_next() & 1u) ? 1 : -1;
    }

    int16_t pre_I[128], pre_Q[128];
    build_walsh128_preamble(pre_I, pre_Q, ExperimentConfig::kAmp);

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

    int64_t z_re = 0;
    int64_t z_im = 0;
    pacd_preamble_cross_corr(rx_full_I, rx_full_Q, pre_I, pre_Q,
                             ExperimentConfig::kPreambleChips, &z_re, &z_im);
    const double psi_hat =
        std::atan2(static_cast<double>(z_im), static_cast<double>(z_re));

    int16_t d_corr_I[64];
    int16_t d_corr_Q[64];
    pacd_derotate_payload(corr_I, corr_Q, ExperimentConfig::kPayloadChips,
                          psi_hat, d_corr_I, d_corr_Q);

    (void)rx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
    int8_t bits_cand0[16];
    int8_t bits_cand1[16];
    const uint64_t valid_mask = 0xFFFFFFFFFFFFFFFFull;
    if (rx_tensor.Decode_Block_Two_Candidates(
            d_corr_I, d_corr_Q,
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

static Agg run_phase_psi(HTS_Holo_Tensor_4D_TX& tx_tensor,
                         HTS_Holo_Tensor_4D_RX& rx_tensor, double psi_rad,
                         int snr_db, int num_trials) noexcept {
    Agg a{};
    for (int trial = 0; trial < num_trials; ++trial) {
        const TrialOutcome o =
            run_single_trial_v13(tx_tensor, rx_tensor, snr_db, trial, psi_rad);
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

/// Lab v4 참조 dec/100 (v9 sweep와 동일 순서)
static constexpr int kLabV4Dec100[] = {
    100, 100, 100, 100, 100, 100, 21, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 17, 100, 100, 100, 100, 100};

static void pacd_print_psi_hat_samples(HTS_Holo_Tensor_4D_TX& tx_tensor,
                                       int snr_db) {
    static const double kSampleDeg[] = {0.0, 90.0, 180.0, 270.0};
    std::printf("PaCD ψ_hat / PTE sample (trial=0 RNG, preamble 128 chip):\n");
    for (unsigned si = 0; si < 4; ++si) {
        const double psi_deg = kSampleDeg[si];
        const double psi_rad = psi_deg * kPi / 180.0;
        rng_seed(13u);
        int8_t data_bits[16];
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            data_bits[i] = (rng_next() & 1u) ? 1 : -1;
        }
        int16_t pre_I[128], pre_Q[128];
        build_walsh128_preamble(pre_I, pre_Q, ExperimentConfig::kAmp);
        (void)tx_tensor.Set_Time_Slot(0u);
        int16_t pay_I[64], pay_Q[64];
        if (tx_build_tensor(tx_tensor, 0u, data_bits, pay_I, pay_Q,
                            ExperimentConfig::kAmp) == 0) {
            std::printf("  psi_true=%3.0f: TX encode fail\n", psi_deg);
            continue;
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
        channel_apply(phased_I, phased_Q, rx_full_I, rx_full_Q,
                      ExperimentConfig::kTotalChips, 0.0, snr_db, 19u);
        int64_t z_re = 0;
        int64_t z_im = 0;
        pacd_preamble_cross_corr(rx_full_I, rx_full_Q, pre_I, pre_Q,
                                 ExperimentConfig::kPreambleChips, &z_re, &z_im);
        double e_pre = 0.0;
        for (int c = 0; c < ExperimentConfig::kPreambleChips; ++c) {
            const double ri = static_cast<double>(rx_full_I[c]);
            const double rq = static_cast<double>(rx_full_Q[c]);
            e_pre += ri * ri + rq * rq;
        }
        const double z_mag =
            std::hypot(static_cast<double>(z_re), static_cast<double>(z_im));
        const double pte_conf = z_mag / (std::sqrt(e_pre) + 1e-12);
        const double psi_hat_deg =
            std::atan2(static_cast<double>(z_im), static_cast<double>(z_re)) *
            180.0 / kPi;
        std::printf(
            "  psi_true=%3.0f: z_re=%lld z_im=%lld psi_hat=%8.2f deg  PTE~%.5f\n",
            psi_deg, static_cast<long long>(z_re),
            static_cast<long long>(z_im), psi_hat_deg, pte_conf);
    }
    std::printf("\n");
}

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

    std::printf("=== Lab v13 PaCD Phase 1 (cfo=0, timing=0, SNR=%d dB) ===\n",
                kSnrDb);
    std::printf("v9 TX(I=Q); RX: preamble z -> psi_hat; payload derotate; "
                "V5a+Two_Candidates. trials=%d/pt\n\n",
                kTrials);

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

    pacd_print_psi_hat_samples(tx_tensor, kSnrDb);

    std::printf(
        "%-7s | %-9s | %-9s | v4 dec | ddec | %s\n",
        "psi deg", "dec/100", "sync/100", "tag");
    std::printf("--------+-----------+-----------+--------+------+------\n");

    for (int i = 0; i < nP; ++i) {
        const double psi_deg = kPhasesDeg[i];
        const double psi_rad = psi_deg * kPi / 180.0;
        const Agg a = run_phase_psi(tx_tensor, rx_tensor, psi_rad, kSnrDb,
                                    kTrials);
        const int v4d = kLabV4Dec100[i];
        std::printf("%5.1f | %3d/100 | %3d/100 | %6d | %+4d | %s\n", psi_deg,
                    a.decode_pass, a.sync_pass, v4d,
                    a.decode_pass - v4d, row_tag(a.decode_pass));
    }
    std::printf("--------+-----------+-----------+--------+------+------\n");

    rx_tensor.Shutdown();
    tx_tensor.Shutdown();
    return 0;
}

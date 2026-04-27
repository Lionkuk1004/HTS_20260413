// ============================================================================
// HTS_CFO_Bank_Test_v13e_llr_blind.cpp — Lab v13e: V13d 동일 악랄 sweep +
// (1) Two-Candidates 선택: 정답 없이 metric soft 합으로 blind
// (2) Combined: 경로별 hard max 대신 signed metric LLR 합 + 분기 없는 부호
// RX API는 cand0/cand1에 동일 metric[]를 주므로 blind 규칙은 Σ m·c0 vs Σ m·c1
// (이 Walsh 디코더에선 보통 cand0과 일치). Lab 전용, 양산 코드 미변경.
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
constexpr int kNumTrials = 5000;
constexpr double kSnrMin = -20.0;
constexpr double kSnrMax = 35.0;
constexpr int kNumBins = 11;
constexpr double kCfoMax = 1000.0;
constexpr double kTimingMax = 0.5;
constexpr int kMpTauMax = 15;
constexpr double kMpAmpMax = 0.7;

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

static int16_t saturate_i16_from_double(double x) noexcept {
    long long v = std::llround(x);
    if (v > 32767)
        v = 32767;
    if (v < -32768)
        v = -32768;
    return static_cast<int16_t>(v);
}

static void apply_multipath(const int16_t* in_I, const int16_t* in_Q, int n_chips,
                            int mp_tau_chip, double mp_amp, double mp_phase_deg,
                            int16_t* out_I, int16_t* out_Q) noexcept {
    if (n_chips <= 0)
        return;
    if (mp_tau_chip <= 0 || mp_amp < 1e-15) {
        std::memcpy(out_I, in_I, sizeof(int16_t) * static_cast<size_t>(n_chips));
        std::memcpy(out_Q, in_Q, sizeof(int16_t) * static_cast<size_t>(n_chips));
        return;
    }
    const double ph = mp_phase_deg * kPi / 180.0;
    const double cp = std::cos(ph);
    const double sp = std::sin(ph);
    for (int c = 0; c < n_chips; ++c) {
        const int d = c - mp_tau_chip;
        double eI = 0.0;
        double eQ = 0.0;
        if (d >= 0) {
            const double di = static_cast<double>(in_I[d]);
            const double dq = static_cast<double>(in_Q[d]);
            eI = di * cp - dq * sp;
            eQ = dq * cp + di * sp;
        }
        const double oI = static_cast<double>(in_I[c]) + mp_amp * eI;
        const double oQ = static_cast<double>(in_Q[c]) + mp_amp * eQ;
        out_I[c] = saturate_i16_from_double(oI);
        out_Q[c] = saturate_i16_from_double(oQ);
    }
}

static void apply_timing_fractional(const int16_t* in_I, const int16_t* in_Q,
                                    int n_chips, double timing_chip,
                                    int16_t* out_I, int16_t* out_Q) noexcept {
    if (n_chips <= 0)
        return;
    if (std::fabs(timing_chip) < 1e-12) {
        std::memcpy(out_I, in_I, sizeof(int16_t) * static_cast<size_t>(n_chips));
        std::memcpy(out_Q, in_Q, sizeof(int16_t) * static_cast<size_t>(n_chips));
        return;
    }
    const int last = n_chips - 1;
    for (int c = 0; c < n_chips; ++c) {
        const double x = static_cast<double>(c) - timing_chip;
        int x0 = static_cast<int>(std::floor(x));
        const double frac = x - static_cast<double>(x0);
        double i0 = 0.0;
        double q0 = 0.0;
        double i1 = 0.0;
        double q1 = 0.0;
        if (x0 < 0) {
            i0 = i1 = static_cast<double>(in_I[0]);
            q0 = q1 = static_cast<double>(in_Q[0]);
            x0 = 0;
        } else if (x0 >= last) {
            i0 = i1 = static_cast<double>(in_I[last]);
            q0 = q1 = static_cast<double>(in_Q[last]);
            x0 = last;
        } else {
            i0 = static_cast<double>(in_I[x0]);
            q0 = static_cast<double>(in_Q[x0]);
            i1 = static_cast<double>(in_I[x0 + 1]);
            q1 = static_cast<double>(in_Q[x0 + 1]);
        }
        out_I[c] = saturate_i16_from_double(i0 + frac * (i1 - i0));
        out_Q[c] = saturate_i16_from_double(q0 + frac * (q1 - q0));
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

/// 정답 미사용: soft 합 Σ m·c0 vs Σ m·c1 (부호 반대 후보만 구별 가능).
static void blind_pick_candidate(const int8_t* c0, const int8_t* c1,
                                 const int32_t* metric, int k,
                                 int8_t* out_bits) noexcept {
    int64_t sum0 = 0;
    int64_t sum1 = 0;
    for (int i = 0; i < k; ++i) {
        const int64_t m = static_cast<int64_t>(metric[i]);
        sum0 += m * static_cast<int64_t>(c0[i]);
        sum1 += m * static_cast<int64_t>(c1[i]);
    }
    if (sum0 >= sum1) {
        std::memcpy(out_bits, c0, sizeof(int8_t) * static_cast<size_t>(k));
    } else {
        std::memcpy(out_bits, c1, sizeof(int8_t) * static_cast<size_t>(k));
    }
}

struct TrialDec {
    bool encode_ok;
    bool sync_ok;
    bool legacy_ok;
    bool pacd_ok;
    bool dd_ok;
    bool combined_ok;
};

static TrialDec run_brutal_trial(HTS_Holo_Tensor_4D_TX& tx_tensor,
                                 HTS_Holo_Tensor_4D_RX& rx_tensor,
                                 const int8_t* data_bits, int trial,
                                 double snr_db, double cfo_hz, double timing_chip,
                                 int mp_tau, double mp_amp, double mp_phase_deg,
                                 double psi_true_rad) noexcept {
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
        return TrialDec{false, false, false, false, false, false};
    }

    int16_t full_I[kTot], full_Q[kTot];
    std::memcpy(full_I, pre_I, sizeof(int16_t) * static_cast<size_t>(kPre));
    std::memcpy(full_I + kPre, pay_I, sizeof(int16_t) * static_cast<size_t>(kPay));
    std::memcpy(full_Q, pre_Q, sizeof(int16_t) * static_cast<size_t>(kPre));
    std::memcpy(full_Q + kPre, pay_Q, sizeof(int16_t) * static_cast<size_t>(kPay));

    int16_t phased_I[kTot], phased_Q[kTot];
    apply_carrier_phase(full_I, full_Q, phased_I, phased_Q, kTot, psi_true_rad);

    int16_t timed_I[kTot], timed_Q[kTot];
    apply_timing_fractional(phased_I, phased_Q, kTot, timing_chip, timed_I,
                            timed_Q);

    int16_t mp_I[kTot], mp_Q[kTot];
    apply_multipath(timed_I, timed_Q, kTot, mp_tau, mp_amp, mp_phase_deg, mp_I,
                    mp_Q);

    int16_t rx_full_I[kTot], rx_full_Q[kTot];
    const uint32_t ch_seed = static_cast<uint32_t>(trial * 31u + 17u);
    channel_apply(mp_I, mp_Q, rx_full_I, rx_full_Q, kTot, cfo_hz, snr_db,
                  ch_seed);

    bool sync_ok = true;
    if (ExperimentConfig::kEnableHoloSync) {
        sync_ok = holo_sync_check(rx_full_I, rx_full_Q);
    }
    if (!sync_ok) {
        return TrialDec{true, false, false, false, false, false};
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

    int16_t buf_orig_I[kTot];
    int16_t buf_orig_Q[kTot];
    std::memcpy(buf_orig_I, buf_I, sizeof(buf_orig_I));
    std::memcpy(buf_orig_Q, buf_Q, sizeof(buf_orig_Q));

    int64_t z1_re = 0;
    int64_t z1_im = 0;
    for (int c = 0; c < kPre; ++c) {
        z1_re += static_cast<int64_t>(buf_I[c]) * static_cast<int64_t>(pre_I[c]) +
                 static_cast<int64_t>(buf_Q[c]) * static_cast<int64_t>(pre_Q[c]);
        z1_im += static_cast<int64_t>(buf_Q[c]) * static_cast<int64_t>(pre_I[c]) -
                 static_cast<int64_t>(buf_I[c]) * static_cast<int64_t>(pre_Q[c]);
    }
    const double psi_hat_1 =
        std::atan2(static_cast<double>(z1_im), static_cast<double>(z1_re));
    const double cos1 = std::cos(psi_hat_1);
    const double sin1 = std::sin(psi_hat_1);

    for (int c = kPre; c < kTot; ++c) {
        const double oI = static_cast<double>(buf_I[c]);
        const double oQ = static_cast<double>(buf_Q[c]);
        long long di = std::llround(oI * cos1 + oQ * sin1);
        long long dq = std::llround(oQ * cos1 - oI * sin1);
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

    int8_t bits_leg_blind[HOLO_MAX_BLOCK_BITS];
    int8_t bits_pacd_blind[HOLO_MAX_BLOCK_BITS];

    const uint32_t ok_leg = rx_tensor.Decode_Block_Two_Candidates_With_Metric(
        buf_orig_I + kPre, buf_orig_Q + kPre, Nchip, valid_mask, leg_c0, leg_c1,
        metric_legacy, Kbits);
    const uint32_t ok_pacd = rx_tensor.Decode_Block_Two_Candidates_With_Metric(
        buf_I + kPre, buf_Q + kPre, Nchip, valid_mask, pacd_c0, pacd_c1,
        metric_pacd, Kbits);

    bool legacy_ok = false;
    if (ok_leg == HTS_Holo_Tensor_4D_RX::SECURE_TRUE) {
        blind_pick_candidate(leg_c0, leg_c1, metric_legacy,
                             ExperimentConfig::kK, bits_leg_blind);
        legacy_ok = (bit_err_vs_ref(bits_leg_blind, data_bits,
                                    ExperimentConfig::kK) == 0);
    }

    bool pacd_ok = false;
    if (ok_pacd == HTS_Holo_Tensor_4D_RX::SECURE_TRUE) {
        blind_pick_candidate(pacd_c0, pacd_c1, metric_pacd,
                             ExperimentConfig::kK, bits_pacd_blind);
        pacd_ok = (bit_err_vs_ref(bits_pacd_blind, data_bits,
                                  ExperimentConfig::kK) == 0);
    }

    int8_t dd_c0[HOLO_MAX_BLOCK_BITS];
    int8_t dd_c1[HOLO_MAX_BLOCK_BITS];
    int32_t metric_dd[HOLO_MAX_BLOCK_BITS];
    uint32_t ok_dd = HTS_Holo_Tensor_4D_RX::SECURE_FALSE;
    int16_t buf_dd_I[kTot];
    int16_t buf_dd_Q[kTot];

    if (ok_pacd == HTS_Holo_Tensor_4D_RX::SECURE_TRUE) {
        int16_t recon_pay_I[64], recon_pay_Q[64];
        if (tx_build_tensor(tx_tensor, static_cast<uint32_t>(trial),
                            bits_pacd_blind, recon_pay_I, recon_pay_Q,
                            ExperimentConfig::kAmp) != 0) {
            int16_t sig_I[kTot], sig_Q[kTot];
            std::memcpy(sig_I, pre_I, sizeof(int16_t) * static_cast<size_t>(kPre));
            std::memcpy(sig_Q, pre_Q, sizeof(int16_t) * static_cast<size_t>(kPre));
            std::memcpy(sig_I + kPre, recon_pay_I,
                        sizeof(int16_t) * static_cast<size_t>(kPay));
            std::memcpy(sig_Q + kPre, recon_pay_Q,
                        sizeof(int16_t) * static_cast<size_t>(kPay));

            int64_t z2_re = 0;
            int64_t z2_im = 0;
            for (int c = 0; c < kTot; ++c) {
                z2_re += static_cast<int64_t>(buf_orig_I[c]) *
                             static_cast<int64_t>(sig_I[c]) +
                         static_cast<int64_t>(buf_orig_Q[c]) *
                             static_cast<int64_t>(sig_Q[c]);
                z2_im += static_cast<int64_t>(buf_orig_Q[c]) *
                             static_cast<int64_t>(sig_I[c]) -
                         static_cast<int64_t>(buf_orig_I[c]) *
                             static_cast<int64_t>(sig_Q[c]);
            }
            const double psi_hat_2 = std::atan2(
                static_cast<double>(z2_im), static_cast<double>(z2_re));
            const double cos2 = std::cos(psi_hat_2);
            const double sin2 = std::sin(psi_hat_2);

            std::memcpy(buf_dd_I, buf_orig_I, sizeof(buf_dd_I));
            std::memcpy(buf_dd_Q, buf_orig_Q, sizeof(buf_dd_Q));
            for (int c = kPre; c < kTot; ++c) {
                const double oI = static_cast<double>(buf_orig_I[c]);
                const double oQ = static_cast<double>(buf_orig_Q[c]);
                long long di = std::llround(oI * cos2 + oQ * sin2);
                long long dq = std::llround(oQ * cos2 - oI * sin2);
                if (di > 32767)
                    di = 32767;
                if (di < -32768)
                    di = -32768;
                if (dq > 32767)
                    dq = 32767;
                if (dq < -32768)
                    dq = -32768;
                buf_dd_I[c] = static_cast<int16_t>(di);
                buf_dd_Q[c] = static_cast<int16_t>(dq);
            }

            ok_dd = rx_tensor.Decode_Block_Two_Candidates_With_Metric(
                buf_dd_I + kPre, buf_dd_Q + kPre, Nchip, valid_mask, dd_c0,
                dd_c1, metric_dd, Kbits);
        }
    }

    int8_t bits_dd_blind[HOLO_MAX_BLOCK_BITS];
    bool dd_ok = false;
    if (ok_dd == HTS_Holo_Tensor_4D_RX::SECURE_TRUE) {
        blind_pick_candidate(dd_c0, dd_c1, metric_dd, ExperimentConfig::kK,
                             bits_dd_blind);
        dd_ok = (bit_err_vs_ref(bits_dd_blind, data_bits, ExperimentConfig::kK) ==
                 0);
    }

    bool combined_ok = false;
    if (ok_leg == HTS_Holo_Tensor_4D_RX::SECURE_TRUE &&
        ok_pacd == HTS_Holo_Tensor_4D_RX::SECURE_TRUE &&
        ok_dd == HTS_Holo_Tensor_4D_RX::SECURE_TRUE) {
        int8_t bits_combined[HOLO_MAX_BLOCK_BITS];
        for (int k = 0; k < ExperimentConfig::kK; ++k) {
            const int64_t llr_sum =
                static_cast<int64_t>(metric_legacy[k]) +
                static_cast<int64_t>(metric_pacd[k]) +
                static_cast<int64_t>(metric_dd[k]);
            const int32_t mask_neg =
                static_cast<int32_t>(llr_sum >> 63);
            const int32_t v =
                (mask_neg & static_cast<int32_t>(-1)) |
                (~mask_neg & static_cast<int32_t>(1));
            bits_combined[k] = static_cast<int8_t>(v);
        }
        combined_ok =
            (bit_err_vs_ref(bits_combined, data_bits, ExperimentConfig::kK) == 0);
    }

    std::memset(buf_orig_I, 0, sizeof(buf_orig_I));
    std::memset(buf_orig_Q, 0, sizeof(buf_orig_Q));
    std::memset(metric_legacy, 0, sizeof(int32_t) * HOLO_MAX_BLOCK_BITS);
    std::memset(metric_pacd, 0, sizeof(int32_t) * HOLO_MAX_BLOCK_BITS);
    std::memset(metric_dd, 0, sizeof(int32_t) * HOLO_MAX_BLOCK_BITS);
    std::memset(leg_c0, 0, sizeof(leg_c0));
    std::memset(leg_c1, 0, sizeof(leg_c1));
    std::memset(pacd_c0, 0, sizeof(pacd_c0));
    std::memset(pacd_c1, 0, sizeof(pacd_c1));
    std::memset(dd_c0, 0, sizeof(dd_c0));
    std::memset(dd_c1, 0, sizeof(dd_c1));

    return TrialDec{true,  true,  legacy_ok, pacd_ok, dd_ok,
                    combined_ok};
}

struct BinStat {
    uint64_t count{ 0 };
    double snr_sum{ 0.0 };
    uint64_t legacy_ok{ 0 };
    uint64_t pacd_ok{ 0 };
    uint64_t dd_ok{ 0 };
    uint64_t combined_ok{ 0 };
};

static int snr_bin_index(double snr_db) noexcept {
    int b = static_cast<int>(std::floor((snr_db - kSnrMin) / 5.0));
    if (b < 0)
        b = 0;
    if (b >= kNumBins)
        b = kNumBins - 1;
    return b;
}

static double bin_snr_low(int bin) noexcept {
    return kSnrMin + 5.0 * static_cast<double>(bin);
}

static bool interp_threshold_snr_path(const BinStat* bins, int nbin,
                                      double target_pct, int path,
                                      double* out_center_snr) noexcept {
    for (int b = 0; b < nbin; ++b) {
        if (bins[b].count == 0u)
            continue;
        uint64_t ok = 0u;
        if (path == 0)
            ok = bins[b].legacy_ok;
        else if (path == 1)
            ok = bins[b].pacd_ok;
        else if (path == 2)
            ok = bins[b].dd_ok;
        else
            ok = bins[b].combined_ok;
        const double p =
            100.0 * static_cast<double>(ok) /
            static_cast<double>(bins[b].count);
        if (p >= target_pct - 1e-9) {
            *out_center_snr = bin_snr_low(b) + 2.5;
            return true;
        }
    }
    return false;
}

}  // namespace

int main() {
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
        rng_seed(static_cast<uint32_t>(0x9E3779B9u * static_cast<uint32_t>(trial) +
                                       1u));

        const double snr_db =
            kSnrMin + (kSnrMax - kSnrMin) * rng_uniform();
        const double cfo_hz =
            (rng_uniform() * 2.0 - 1.0) * kCfoMax;
        const double timing_chip =
            (rng_uniform() * 2.0 - 1.0) * kTimingMax;
        const bool mp_active = (rng_uniform() > 0.5);
        const int mp_tau =
            mp_active
                ? (1 + static_cast<int>(rng_uniform() * static_cast<double>(kMpTauMax)))
                : 0;
        const double mp_amp =
            mp_active ? (rng_uniform() * kMpAmpMax) : 0.0;
        const double mp_phase_deg = rng_uniform() * 360.0;
        const double psi_deg = rng_uniform() * 360.0;
        const double psi_true_rad = psi_deg * kPi / 180.0;

        int8_t data_bits[ExperimentConfig::kK];
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            data_bits[i] = (rng_next() & 1u) ? 1 : -1;
        }

        const TrialDec d = run_brutal_trial(
            tx_tensor, rx_tensor, data_bits, trial, snr_db, cfo_hz,
            timing_chip, mp_tau, mp_amp, mp_phase_deg, psi_true_rad);

        const int bin = snr_bin_index(snr_db);
        if (!d.encode_ok)
            continue;
        bins[bin].count += 1u;
        bins[bin].snr_sum += snr_db;
        if (d.sync_ok) {
            if (d.legacy_ok)
                bins[bin].legacy_ok += 1u;
            if (d.pacd_ok)
                bins[bin].pacd_ok += 1u;
            if (d.dd_ok)
                bins[bin].dd_ok += 1u;
            if (d.combined_ok)
                bins[bin].combined_ok += 1u;
        }
    }

    std::printf("## Lab v13e LLR Sum + Blind 결과\n\n");
    std::printf("### Bin 별 통계 (V13d 와 같은 형식)\n\n");
    std::printf("| Bin | SNR range | trials | actual_avg | legacy%% | pacd%% | "
                "dd%% | combined%% |\n");
    std::printf("|-----|-----------|--------|------------|---------|-------|---"
                "|-----------|\n");

    uint64_t subnoise_count = 0u;
    uint64_t subnoise_combined = 0u;
    for (int b = 0; b < kNumBins; ++b) {
        const double lo = bin_snr_low(b);
        const double hi = lo + 5.0;
        if (bins[b].count == 0u) {
            std::printf("| %d | %.0f~%.0f | 0 | n/a | n/a | n/a | n/a |\n", b,
                        lo, hi);
            continue;
        }
        if (hi <= 0.0) {
            subnoise_count += bins[b].count;
            subnoise_combined += bins[b].combined_ok;
        }
        const double avg_snr = bins[b].snr_sum / static_cast<double>(bins[b].count);
        const double leg_pct =
            100.0 * static_cast<double>(bins[b].legacy_ok) /
            static_cast<double>(bins[b].count);
        const double pacd_pct =
            100.0 * static_cast<double>(bins[b].pacd_ok) /
            static_cast<double>(bins[b].count);
        const double dd_pct =
            100.0 * static_cast<double>(bins[b].dd_ok) /
            static_cast<double>(bins[b].count);
        const double comb_pct =
            100.0 * static_cast<double>(bins[b].combined_ok) /
            static_cast<double>(bins[b].count);
        std::printf("| %d | %.0f~%.0f | %llu | %+6.2f | %5.1f | %5.1f | %5.1f | "
                    "%5.1f |\n",
                    b, lo, hi,
                    static_cast<unsigned long long>(bins[b].count), avg_snr,
                    leg_pct, pacd_pct, dd_pct, comb_pct);
    }

    constexpr int kBinCompare = 10;
    double e10_leg = 0.0;
    double e10_pacd = 0.0;
    double e10_dd = 0.0;
    double e10_comb = 0.0;
    if (bins[kBinCompare].count > 0u) {
        const double inv =
            100.0 / static_cast<double>(bins[kBinCompare].count);
        e10_leg = static_cast<double>(bins[kBinCompare].legacy_ok) * inv;
        e10_pacd = static_cast<double>(bins[kBinCompare].pacd_ok) * inv;
        e10_dd = static_cast<double>(bins[kBinCompare].dd_ok) * inv;
        e10_comb = static_cast<double>(bins[kBinCompare].combined_ok) * inv;
    }
    std::printf("\n### V13d vs V13e 비교 (bin 10 = 30~35 dB, V13d 숫자는 고정 "
                "참조 `v13d_brutal_result.txt`)\n\n");
    std::printf("| 디코더 | V13d (oracle+hard) | V13e (blind+LLR) | 차이(p.p.) |\n");
    std::printf("|--------|-------------------|------------------|------------|\n");
    std::printf("| Legacy | 72.0%% | %5.1f%% | %+5.1f |\n", e10_leg,
                e10_leg - 72.0);
    std::printf("| PaCD | 69.3%% | %5.1f%% | %+5.1f |\n", e10_pacd,
                e10_pacd - 69.3);
    std::printf("| DD | 73.3%% | %5.1f%% | %+5.1f |\n", e10_dd, e10_dd - 73.3);
    std::printf("| Combined | 61.1%% | %5.1f%% | %+5.1f |\n", e10_comb,
                e10_comb - 61.1);

    const double subnoise_comb_pct =
        (subnoise_count > 0u)
            ? (100.0 * static_cast<double>(subnoise_combined) /
               static_cast<double>(subnoise_count))
            : 0.0;

    std::printf("\n### 90%% 임계 SNR — V13e (blind + LLR sum)\n\n");
    double t90_l = 0.0, t90_p = 0.0, t90_d = 0.0, t90_c = 0.0;
    const bool hit_l =
        interp_threshold_snr_path(bins, kNumBins, 90.0, 0, &t90_l);
    const bool hit_p =
        interp_threshold_snr_path(bins, kNumBins, 90.0, 1, &t90_p);
    const bool hit_d =
        interp_threshold_snr_path(bins, kNumBins, 90.0, 2, &t90_d);
    const bool hit_c =
        interp_threshold_snr_path(bins, kNumBins, 90.0, 3, &t90_c);
    std::printf("| 디코더 | 임계 SNR | gain vs Legacy |\n");
    std::printf("|--------|---------|----------------|\n");
    if (hit_l)
        std::printf("| Legacy | ~%.1f dB | - |\n", t90_l);
    else
        std::printf("| Legacy | >35 dB (미도달) | - |\n");
    if (hit_p)
        std::printf("| PaCD | ~%.1f dB | ~%.1f dB |\n", t90_p,
                    hit_l ? (t90_l - t90_p) : 0.0);
    else
        std::printf("| PaCD | >35 dB (미도달) | n/a |\n");
    if (hit_d)
        std::printf("| DD | ~%.1f dB | ~%.1f dB |\n", t90_d,
                    hit_l ? (t90_l - t90_d) : 0.0);
    else
        std::printf("| DD | >35 dB (미도달) | n/a |\n");
    if (hit_c)
        std::printf("| Combined | ~%.1f dB | ~%.1f dB |\n", t90_c,
                    hit_l ? (t90_l - t90_c) : 0.0);
    else
        std::printf("| Combined | >35 dB (미도달) | n/a |\n");

    std::printf("\n### 영준님 통찰 검증 (본 실행 요약)\n\n");
    if (hit_p && hit_d) {
        std::printf("- 미적분 정밀화 (DD vs PaCD @90%%): PaCD 임계 − DD 임계 ≈ %.1f "
                    "dB (양수면 DD 유리)\n",
                    t90_p - t90_d);
    } else {
        std::printf("- 미적분 정밀화 (DD vs PaCD @90%%): 한쪽 이상 미도달 → "
                    "악랄 sweep에서 90%% 미달\n");
    }
    if (hit_p && hit_c) {
        std::printf("- Combined 추가 이득 @90%%: PaCD 임계 − Combined 임계 ≈ %.1f "
                    "dB\n",
                    t90_p - t90_c);
    } else {
        std::printf("- Combined @90%%: PaCD 또는 Combined 임계 미도달\n");
    }
    std::printf("- SNR≤0 dB 구간(표 상한 0) combined 성공 비율: ~%.2f %% (trial "
                "가중)\n",
                subnoise_comb_pct);
    std::printf("- 악랄 sweep: trial=%d, CFO±%.0f Hz, timing±%.2f chip, MP "
                "50%%×tau≤%d, psi U[0,360)\n",
                kNumTrials, kCfoMax, kTimingMax, kMpTauMax);

    rx_tensor.Shutdown();
    tx_tensor.Shutdown();
    return 0;
}

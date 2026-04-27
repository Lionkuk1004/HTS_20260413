// ============================================================================
// HTS_CFO_Bank_Test_v6_lrdiag.cpp — Lab: HTS_LR_DIAG per-trial dump + stats
// Preamble + channel 동일 v5_estonly; cfo_true=1000 Hz, SNR=30, trials=100.
// ============================================================================
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "HTS_CFO_V5a.hpp"

#ifndef HTS_LR_DIAG
#error Build with /DHTS_LR_DIAG (Lab only).
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kPreambleChips = 128;
constexpr int kAmp = 500;
constexpr int kTrials = 100;
constexpr int kSnrDb = 30;
constexpr int32_t kCfoTrueHz = 1000;
constexpr double kChipRateHz = 1e6;
constexpr double kQ15ToHz =
    10000000.0 / 26214400.0;  // same as (phase_q15 * 1e7) / 26214400

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
        2.0 * kPi * cfo_hz / kChipRateHz;
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

struct Acc {
    double sum{ 0.0 };
    double sumsq{ 0.0 };
    int n{ 0 };
    void push(double x) noexcept {
        sum += x;
        sumsq += x * x;
        ++n;
    }
    double mean() const noexcept {
        return n > 0 ? sum / static_cast<double>(n) : 0.0;
    }
    double stddev_pop() const noexcept {
        if (n <= 0)
            return 0.0;
        const double m = mean();
        const double v = sumsq / static_cast<double>(n) - m * m;
        return v > 0.0 ? std::sqrt(v) : 0.0;
    }
};

static void print_hist_int(const char* label, const int32_t* vals, int n) {
    int32_t uniq[128];
    int cnt[128];
    int u = 0;
    for (int i = 0; i < n; ++i) {
        const int32_t v = vals[i];
        int k = 0;
        for (; k < u; ++k) {
            if (uniq[k] == v) {
                cnt[k]++;
                break;
            }
        }
        if (k == u && u < 128) {
            uniq[u] = v;
            cnt[u] = 1;
            ++u;
        }
    }
    std::printf("%s: ", label);
    for (int k = 0; k < u; ++k) {
        std::printf("%d:%d%s", static_cast<int>(uniq[k]), cnt[k],
                    (k + 1 < u) ? ", " : "");
    }
    std::printf("\n");
}

}  // namespace

int main() {
    using hts::rx_cfo::CFO_Result;
    using hts::rx_cfo::CFO_V5a;
    using hts::rx_cfo::g_lr_diag;
    using hts::rx_cfo::kLR_MaxLag;

    int16_t tx_I[kPreambleChips];
    int16_t tx_Q[kPreambleChips];
    build_walsh128_preamble(tx_I, tx_Q, kAmp);

    CFO_V5a v5a;
    v5a.Init();
#if (HTS_CFO_V5A_ENABLE != 0)
    v5a.SetEnabled(true);
#endif

    std::printf("=== LR DIAG (cfo_true=%d Hz, SNR=%d dB, trials=%d) ===\n\n",
                static_cast<int>(kCfoTrueHz), kSnrDb, kTrials);
    std::printf(
        "trial | cb_cfo | fine_ref | lag1_cfo | lag2_cfo | lag3_cfo | "
        "lag4_cfo | Z_ph_q15 | lr_cfo | est\n");
    std::printf(
        "------+--------+----------+----------+----------+----------+-----"
        "-----+----------+--------+----\n");

    int32_t hist_cb[kTrials];
    int32_t hist_fine[kTrials];
    Acc acc_lag[4];
    Acc acc_z_ph;
    Acc acc_z_hz;
    Acc acc_lr;
    Acc acc_est;

    for (int t = 0; t < kTrials; ++t) {
        int16_t rx_I[kPreambleChips];
        int16_t rx_Q[kPreambleChips];
        const uint32_t ch_seed =
            static_cast<uint32_t>(static_cast<unsigned>(t) * 31u + 19u);
        channel_apply(tx_I, tx_Q, rx_I, rx_Q, kPreambleChips,
                      static_cast<double>(kCfoTrueHz), kSnrDb, ch_seed);

        const CFO_Result res = v5a.Estimate(rx_I, rx_Q);
        const int32_t est = res.cfo_hz;

        hist_cb[t] = g_lr_diag.cb_cfo;
        hist_fine[t] = g_lr_diag.fine_refined;

        for (int d = 0; d < kLR_MaxLag; ++d) {
            acc_lag[d].push(static_cast<double>(g_lr_diag.lag_cfo_hz[d]));
        }
        acc_z_ph.push(static_cast<double>(g_lr_diag.Z_phase_q15));
        acc_z_hz.push(static_cast<double>(g_lr_diag.Z_phase_q15) * kQ15ToHz);
        acc_lr.push(static_cast<double>(g_lr_diag.lr_cfo_hz));
        acc_est.push(static_cast<double>(est));

        std::printf(
            "%5d | %6d | %8d | %8d | %8d | %8d | %8d | %9d | %6d | %5d\n",
            t, static_cast<int>(g_lr_diag.cb_cfo),
            static_cast<int>(g_lr_diag.fine_refined),
            static_cast<int>(g_lr_diag.lag_cfo_hz[0]),
            static_cast<int>(g_lr_diag.lag_cfo_hz[1]),
            static_cast<int>(g_lr_diag.lag_cfo_hz[2]),
            static_cast<int>(g_lr_diag.lag_cfo_hz[3]),
            static_cast<int>(g_lr_diag.Z_phase_q15),
            static_cast<int>(g_lr_diag.lr_cfo_hz), static_cast<int>(est));
    }

    std::printf("\n=== Statistics (%d trials) ===\n", kTrials);
    print_hist_int("cb_cfo (Hz:count)", hist_cb, kTrials);
    print_hist_int("fine_refined (Hz:count)", hist_fine, kTrials);
    for (int d = 0; d < kLR_MaxLag; ++d) {
        std::printf("lag%d_cfo std: %.2f Hz (mean %.2f Hz)\n", d + 1,
                    acc_lag[d].stddev_pop(), acc_lag[d].mean());
    }
    std::printf("Z_phase_q15 std: %.2f (mean %.2f)\n",
                acc_z_ph.stddev_pop(), acc_z_ph.mean());
    std::printf("Z_equiv_hz std: %.2f Hz (mean %.2f Hz)\n",
                acc_z_hz.stddev_pop(), acc_z_hz.mean());
    std::printf("lr_cfo std:      %.2f Hz (mean %.2f Hz)\n",
                acc_lr.stddev_pop(), acc_lr.mean());
    std::printf("est std:         %.2f Hz (mean %.2f Hz)\n",
                acc_est.stddev_pop(), acc_est.mean());

    const int N = kPreambleChips;
    const double snr_lin = std::pow(10.0, static_cast<double>(kSnrDb) / 10.0);
    const double crlb_sigma =
        std::sqrt(6.0 /
                  (std::pow(2.0 * kPi, 2.0) * snr_lin * static_cast<double>(N) *
                   (static_cast<double>(N) * static_cast<double>(N) - 1.0))) *
        kChipRateHz;
    std::printf("\nCRLB sigma (user formula, N=%d, SNR=%d dB, fs=1e6): %.3f Hz\n",
                N, kSnrDb, crlb_sigma);
    std::printf("lr_cfo std / CRLB: %.2f\n", acc_lr.stddev_pop() / crlb_sigma);

#if defined(HTS_LR_DIAG_SNR_SWEEP)
    std::printf("\n=== SNR sweep (cfo_true=%d Hz, trials=%d) ===\n",
                static_cast<int>(kCfoTrueHz), kTrials);
    static const int kSweepSnr[] = { 0, 10, 20, 30, 40 };
    const int n_snr = static_cast<int>(sizeof(kSweepSnr) / sizeof(kSweepSnr[0]));
    std::printf("SNR_dB | lr_std_Hz | est_std_Hz | lr_std/CRLB\n");
    std::printf("-------+-----------+------------+------------\n");
    for (int si = 0; si < n_snr; ++si) {
        const int snr = kSweepSnr[si];
        Acc a_lr{};
        Acc a_est{};
        const double snr_l = std::pow(10.0, static_cast<double>(snr) / 10.0);
        const double crlb_s =
            std::sqrt(6.0 / (std::pow(2.0 * kPi, 2.0) * snr_l *
                             static_cast<double>(N) *
                             (static_cast<double>(N) * static_cast<double>(N) -
                              1.0))) *
            kChipRateHz;
        for (int t = 0; t < kTrials; ++t) {
            int16_t rx_I[kPreambleChips];
            int16_t rx_Q[kPreambleChips];
            const uint32_t ch_seed =
                static_cast<uint32_t>(static_cast<unsigned>(t) * 31u + 19u);
            channel_apply(tx_I, tx_Q, rx_I, rx_Q, kPreambleChips,
                          static_cast<double>(kCfoTrueHz), snr, ch_seed);
            const CFO_Result res = v5a.Estimate(rx_I, rx_Q);
            a_lr.push(static_cast<double>(g_lr_diag.lr_cfo_hz));
            a_est.push(static_cast<double>(res.cfo_hz));
        }
        const double lr_s = a_lr.stddev_pop();
        std::printf(" %5d | %9.2f | %10.2f | %11.2f\n", snr, lr_s,
                    a_est.stddev_pop(), lr_s / crlb_s);
    }
#endif

    return 0;
}

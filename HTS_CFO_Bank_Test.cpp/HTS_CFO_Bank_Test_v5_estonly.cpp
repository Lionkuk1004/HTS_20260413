// ============================================================================
// HTS_CFO_Bank_Test_v5_estonly.cpp — Lab: V5a Estimate only (no 4D decode)
// 동일: Walsh128 preamble, channel_apply(SNR, AWGN), CFO_V5a::Estimate.
// ============================================================================
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "HTS_CFO_V5a.hpp"

#ifndef HTS_V5A_TESTONLY
#error HTS_V5A_TESTONLY required — build with /DHTS_V5A_TESTONLY (Lab only).
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kPreambleChips = 128;
constexpr int kAmp = 500;
constexpr int kTrials = 100;
constexpr int kSnrDb = 30;
constexpr double kChipRateHz = 1e6;

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

struct EstStats {
    int32_t cfo_true{ 0 };
    int32_t est_min{ 2147483647 };
    int32_t est_max{ -2147483647 - 1 };
    int64_t est_sum{ 0 };
    int64_t err_abs_sum{ 0 };
    int64_t err_sq_sum{ 0 };
    int cb_idx_hist[hts::rx_cfo::kCfoCoarseBanks]{};
    int fb_idx_hist[hts::rx_cfo::kCfoFineBanks]{};
    int n_trials{ 0 };
};

static void append_hist_str(char* buf, size_t buf_sz, const int* hist,
                            int n_bins) noexcept {
    buf[0] = '\0';
    for (int i = 0; i < n_bins; ++i) {
        if (hist[i] == 0)
            continue;
        char piece[32];
        std::snprintf(piece, sizeof(piece), "%d:%d,", i, hist[i]);
        std::strncat(buf, piece, buf_sz - std::strlen(buf) - 1);
    }
    const size_t L = std::strlen(buf);
    if (L > 0 && buf[L - 1] == ',')
        buf[L - 1] = '\0';
}

}  // namespace

int main() {
    using hts::rx_cfo::CFO_V5a;
    using hts::rx_cfo::CFO_Result;
    using hts::rx_cfo::kCfoCoarseBanks;
    using hts::rx_cfo::kCfoFineBanks;

    static const int32_t test_cfos[] = {
        0,    200,  500,  600,  700,  750,  800,  900,  1500, 2000, 2200,
        2250, 2300, 2350, 2400, 3000, 3500, 3700, 3800, 3850, 3900, 4000,
        4500, 5000, 6000, 8000, 10000};
    const int N_CFO =
        static_cast<int>(sizeof(test_cfos) / sizeof(test_cfos[0]));

    std::printf("=== V5a Estimate standalone (Lab v4 preamble + channel) ===\n");
    std::printf("SNR: %d dB, trials: %d, kAmp: %d, CFO points: %d\n",
                kSnrDb, kTrials, kAmp, N_CFO);
    std::printf("\n");
    std::printf(
        "cfo_true | est_avg | err_avg | err_min | err_max | err_std | "
        "cb_hist (bin:cnt) | fb_hist (bin:cnt)\n");
    std::printf(
        "---------+---------+---------+---------+---------+---------+-----"
        "----------------+-----------------------\n");

    CFO_V5a v5a;
    v5a.Init();
#if (HTS_CFO_V5A_ENABLE != 0)
    v5a.SetEnabled(true);
#endif

    int16_t tx_I[kPreambleChips];
    int16_t tx_Q[kPreambleChips];
    build_walsh128_preamble(tx_I, tx_Q, kAmp);

    for (int idx = 0; idx < N_CFO; ++idx) {
        const int32_t cfo = test_cfos[idx];
        EstStats st{};
        st.cfo_true = cfo;

        for (int t = 0; t < kTrials; ++t) {
            int16_t rx_I[kPreambleChips];
            int16_t rx_Q[kPreambleChips];
            const uint32_t ch_seed =
                static_cast<uint32_t>(static_cast<unsigned>(t) * 31u + 19u);
            channel_apply(tx_I, tx_Q, rx_I, rx_Q, kPreambleChips,
                          static_cast<double>(cfo), kSnrDb, ch_seed);

            const CFO_Result res = v5a.Estimate(rx_I, rx_Q);
            const int32_t est = res.cfo_hz;
            const int32_t err = est - cfo;

            st.est_sum += static_cast<int64_t>(est);
            st.err_abs_sum += static_cast<int64_t>(err < 0 ? -err : err);
            st.err_sq_sum += static_cast<int64_t>(err) * err;
            if (est < st.est_min)
                st.est_min = est;
            if (est > st.est_max)
                st.est_max = est;

            const int cb = v5a.Get_Last_Coarse_Bin();
            const int fb = v5a.Get_Last_Fine_Bin();
            if (cb >= 0 && cb < kCfoCoarseBanks)
                st.cb_idx_hist[cb]++;
            if (fb >= 0 && fb < kCfoFineBanks)
                st.fb_idx_hist[fb]++;
            ++st.n_trials;
        }

        const int n = st.n_trials;
        const int32_t est_avg =
            static_cast<int32_t>((st.est_sum + (n / 2)) / n);
        const int32_t err_avg = est_avg - cfo;
        const int32_t err_min = st.est_min - cfo;
        const int32_t err_max = st.est_max - cfo;
        const double mean_err =
            static_cast<double>(st.est_sum) / static_cast<double>(n) -
            static_cast<double>(cfo);
        const double mean_sq_err =
            static_cast<double>(st.err_sq_sum) / static_cast<double>(n);
        double var = mean_sq_err - mean_err * mean_err;
        if (var < 0.0)
            var = 0.0;
        const double err_std = std::sqrt(var);

        char cb_str[256]{};
        char fb_str[256]{};
        append_hist_str(cb_str, sizeof(cb_str), st.cb_idx_hist,
                        kCfoCoarseBanks);
        append_hist_str(fb_str, sizeof(fb_str), st.fb_idx_hist, kCfoFineBanks);

        std::printf("%8d | %7d | %+7d | %+7d | %+7d | %7.1f | %-19s | %s\n",
                    static_cast<int>(cfo), static_cast<int>(est_avg),
                    static_cast<int>(err_avg), static_cast<int>(err_min),
                    static_cast<int>(err_max), err_std, cb_str, fb_str);
    }

    return 0;
}

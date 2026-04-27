// ============================================================================
// HTS_CFO_Bank_Test_v7_mnm.cpp — Lab v7: 27 CFO × trials, LR vs Walsh M&M DPTE
// 동일 preamble + channel_apply as v5/v6; 직접 test_export::LR_Estimate vs
// MnM_Walsh_Estimate_Dpte_Table (동일 RX 버퍼, coarse/fine bank 없음).
// 빌드: /DHTS_ALLOW_HOST_BUILD /DHTS_USE_MNM_WALSH /DHTS_CFO_V5A_ENABLE=1
// ============================================================================
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "HTS_CFO_V5a.hpp"

#ifndef HTS_ALLOW_HOST_BUILD
#error Lab v7 requires /DHTS_ALLOW_HOST_BUILD
#endif
#ifndef HTS_USE_MNM_WALSH
#error Lab v7 requires /DHTS_USE_MNM_WALSH (MnM_Walsh_Estimate_Dpte_Table)
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kPreambleChips = 128;
constexpr int kAmp = 500;
constexpr int kTrials = 100;
constexpr int kSnrDb = 30;
constexpr double kChipRateHz = 1e6;

constexpr int32_t kCfoSweepList[] = {
    0,    50,   100,  150,  200,  250,  300,  350,  400,  450,  500,
    550,  600,  650,  700,  750,  800,  850,  900,  950,  1000,
    1500, 2000, 2500, 3000, 4000, 5000};
constexpr int kCfoSweepCount =
    static_cast<int>(sizeof(kCfoSweepList) / sizeof(kCfoSweepList[0]));

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

}  // namespace

int main() {
    using hts::rx_cfo::test_export::LR_Estimate;
    using hts::rx_cfo::test_export::MnM_Walsh_Estimate_Dpte_Table;

    int16_t tx_I[kPreambleChips];
    int16_t tx_Q[kPreambleChips];
    build_walsh128_preamble(tx_I, tx_Q, kAmp);

    const int N = kPreambleChips;
    const double snr_lin = std::pow(10.0, static_cast<double>(kSnrDb) / 10.0);
    const double crlb_sigma =
        std::sqrt(6.0 /
                  (std::pow(2.0 * kPi, 2.0) * snr_lin * static_cast<double>(N) *
                   (static_cast<double>(N) * static_cast<double>(N) - 1.0))) *
        kChipRateHz;

    std::printf(
        "=== Lab v7: LR vs Walsh M&M DPTE (%d CFO x %d trials, SNR=%d dB) ===\n",
        kCfoSweepCount, kTrials, kSnrDb);
    std::printf("CRLB sigma (N=%d): %.3f Hz\n\n", N, crlb_sigma);
    std::printf(
        " cfo_true | mean_err_lr | mean_err_mnm | std_lr | std_mnm | "
        "std_lr/CRLB | std_mnm/CRLB | improve(LR/MnM)\n");
    std::printf(
        "----------+-------------+--------------+--------+---------+-----"
        "--------+--------------+----------------\n");

    for (int ci = 0; ci < kCfoSweepCount; ++ci) {
        const int32_t cfo_true = kCfoSweepList[ci];
        Acc acc_lr_err;
        Acc acc_mnm_err;
        for (int t = 0; t < kTrials; ++t) {
            int16_t rx_I[kPreambleChips];
            int16_t rx_Q[kPreambleChips];
            const uint32_t ch_seed =
                static_cast<uint32_t>(static_cast<unsigned>(t) * 31u + 19u) ^
                static_cast<uint32_t>(cfo_true * 65537);
            channel_apply(tx_I, tx_Q, rx_I, rx_Q, kPreambleChips,
                          static_cast<double>(cfo_true), kSnrDb, ch_seed);

            const int32_t est_lr = LR_Estimate(rx_I, rx_Q);
            const int32_t est_mnm = MnM_Walsh_Estimate_Dpte_Table(rx_I, rx_Q);
            acc_lr_err.push(static_cast<double>(est_lr - cfo_true));
            acc_mnm_err.push(static_cast<double>(est_mnm - cfo_true));
        }
        const double std_lr = acc_lr_err.stddev_pop();
        const double std_mnm = acc_mnm_err.stddev_pop();
        const double improve =
            (std_mnm > 1e-6) ? (std_lr / std_mnm) : 0.0;
        std::printf(
            "%9d | %11.2f | %12.2f | %6.2f | %7.2f | %11.2f | %12.2f | %14.2f\n",
            static_cast<int>(cfo_true), acc_lr_err.mean(), acc_mnm_err.mean(),
            std_lr, std_mnm, std_lr / crlb_sigma, std_mnm / crlb_sigma,
            improve);
    }

    std::printf("\nDone.\n");
    return 0;
}

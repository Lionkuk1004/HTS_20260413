// ============================================================================
// HTS_CFO_Bank_Test_step4_v9_surrogate_full_est.cpp — Step 4 보조 측정
//
// 저장소 내 `LVL1_NORMAL` 문자열을 생성하는 **컴파일된 V9 시뮬 엔트리는 미발견**.
// 동일 이름의 **LR 측정 기록**은 `HTS_CFO_Bank_Test.cpp/stage5_results.csv`
// (PASS%, Avg_Err_Hz, Max_Err_Hz) 에 남아 있음 — MnM 열은 없음.
//
// 본 프로그램은 **양산과 동일한 CFO_V5a::Estimate()** (coarse/fine
// bank + PTE + LR 또는 HTS_USE_MNM_WALSH 시 M&M) 만 Lab에서 돌려,
// **이름표만** LVL1~5 / AMI 와 맞춘 **SNR·CFO 분포 서로게이트** 행을 출력한다.
// → 영준님 보유 V9 원본과 숫자 1:1 대응을 주장하지 않는다.
//
// 빌드 LR:  /DHTS_ALLOW_HOST_BUILD /DHTS_CFO_V5A_ENABLE=1
// 빌드 MnM: 위 + /DHTS_USE_MNM_WALSH
// ============================================================================
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "HTS_CFO_V5a.hpp"

#ifndef HTS_ALLOW_HOST_BUILD
#error Step4 surrogate requires /DHTS_ALLOW_HOST_BUILD
#endif

#ifndef HTS_STEP4_TRIALS
#define HTS_STEP4_TRIALS 200
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kPreambleChips = 128;
constexpr int kAmp = 500;
constexpr double kChipRateHz = 1e6;

static const int32_t kTestCfos[] = {
    0,    200,  500,  600,  700,  750,  800,  900,  1500, 2000, 2200,
    2250, 2300, 2350, 2400, 3000, 3500, 3700, 3800, 3850, 3900, 4000,
    4500, 5000, 6000, 8000, 10000};
static constexpr int kNCfo =
    static_cast<int>(sizeof(kTestCfos) / sizeof(kTestCfos[0]));

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

struct PoolAcc {
    double sum_err{ 0.0 };
    double sum_err2{ 0.0 };
    double sum_abs{ 0.0 };
    double max_abs{ 0.0 };
    int pass200{ 0 };
    int n{ 0 };
    void push(double err) noexcept {
        sum_err += err;
        sum_err2 += err * err;
        const double a = err < 0.0 ? -err : err;
        sum_abs += a;
        if (a > max_abs)
            max_abs = a;
        if (a <= 200.0)
            ++pass200;
        ++n;
    }
    double mean_err() const noexcept {
        return n > 0 ? sum_err / static_cast<double>(n) : 0.0;
    }
    double avg_abs_hz() const noexcept {
        return n > 0 ? sum_abs / static_cast<double>(n) : 0.0;
    }
    double err_std_hz() const noexcept {
        if (n <= 0)
            return 0.0;
        const double m = sum_err / static_cast<double>(n);
        const double v =
            sum_err2 / static_cast<double>(n) - m * m;
        return v > 0.0 ? std::sqrt(v) : 0.0;
    }
    double pass200_pct() const noexcept {
        return n > 0 ? 100.0 * static_cast<double>(pass200) /
                           static_cast<double>(n)
                     : 0.0;
    }
};

static void run_level(const char* level_name, int snr_db, bool ami_sigma3k,
                      const int16_t* tx_I, const int16_t* tx_Q,
                      hts::rx_cfo::CFO_V5a& v5a) noexcept {
    PoolAcc pool{};
    const int trials = HTS_STEP4_TRIALS;
    if (!ami_sigma3k) {
        for (int ci = 0; ci < kNCfo; ++ci) {
            const int32_t cfo_true = kTestCfos[ci];
            for (int t = 0; t < trials; ++t) {
                int16_t rx_I[kPreambleChips];
                int16_t rx_Q[kPreambleChips];
                const uint32_t ch_seed =
                    static_cast<uint32_t>(static_cast<unsigned>(t) * 31u +
                                           19u) ^
                    static_cast<uint32_t>(cfo_true * 65537 + ci * 17);
                channel_apply(tx_I, tx_Q, rx_I, rx_Q, kPreambleChips,
                              static_cast<double>(cfo_true), snr_db, ch_seed);
                const hts::rx_cfo::CFO_Result res = v5a.Estimate(rx_I, rx_Q);
                const double err =
                    static_cast<double>(res.cfo_hz - cfo_true);
                pool.push(err);
            }
        }
    } else {
        rng_seed(0xA11CEu);
        for (int t = 0; t < trials; ++t) {
            const double cfo_true = rng_gauss() * 3000.0;
            int16_t rx_I[kPreambleChips];
            int16_t rx_Q[kPreambleChips];
            const uint32_t ch_seed =
                static_cast<uint32_t>(static_cast<unsigned>(t) * 31u + 19u) ^
                rng_next();
            channel_apply(tx_I, tx_Q, rx_I, rx_Q, kPreambleChips, cfo_true,
                          snr_db, ch_seed);
            const hts::rx_cfo::CFO_Result res = v5a.Estimate(rx_I, rx_Q);
            const double err = static_cast<double>(res.cfo_hz) - cfo_true;
            pool.push(err);
        }
    }
    const double pk = pool.max_abs;
    const double p2 = pool.pass200_pct();
    std::printf(
        "%-22s snr=%2d dB cfg_trials=%d pool_n=%6d | mean_hz=%+8.2f "
        "mean_abs_hz=%8.2f std_hz=%8.2f max_abs_hz=%9.1f pass200_pct=%5.1f\n",
        level_name, snr_db, trials, pool.n, pool.mean_err(),
        pool.avg_abs_hz(), pool.err_std_hz(), pk, p2);
}

}  // namespace

int main() {
    using hts::rx_cfo::CFO_V5a;

#if defined(HTS_USE_MNM_WALSH)
    std::printf("STEP4_BUILD_TAG=MNM_HTSCFOV5A_ESTIMATE\n");
#else
    std::printf("STEP4_BUILD_TAG=LR_HTSCFOV5A_ESTIMATE\n");
#endif
    std::printf("STEP4_TRIALS=%d CFO_points=%d (surrogate, not archived V9)\n",
                HTS_STEP4_TRIALS, kNCfo);

    CFO_V5a v5a;
    v5a.Init();
#if (HTS_CFO_V5A_ENABLE != 0)
    v5a.SetEnabled(true);
#endif

    int16_t tx_I[kPreambleChips];
    int16_t tx_Q[kPreambleChips];
    build_walsh128_preamble(tx_I, tx_Q, kAmp);

    std::printf(
        "--- Rows: SNR-only surrogate labels (see source comment) ---\n");
    run_level("LVL1_NORMAL(sur)", 30, false, tx_I, tx_Q, v5a);
    run_level("LVL2_URBAN(sur)", 24, false, tx_I, tx_Q, v5a);
    run_level("LVL3_INDUSTRIAL(sur)", 18, false, tx_I, tx_Q, v5a);
    run_level("LVL4_MILITARY(sur)", 12, false, tx_I, tx_Q, v5a);
    run_level("LVL5_EXTREME(sur)", 6, false, tx_I, tx_Q, v5a);
    std::printf("--- AMI surrogate: AWGN 30 dB + CFO ~ N(0,3000^2) Hz ---\n");
    run_level("AMI_GAUSS_3KHZ(sur)", 30, true, tx_I, tx_Q, v5a);

    return 0;
}

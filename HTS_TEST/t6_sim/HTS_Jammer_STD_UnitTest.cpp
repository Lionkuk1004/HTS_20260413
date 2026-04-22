// HTS_Jammer_STD_UnitTest.cpp — Phase 1 단위 시험 (SPEC / Math_Defense v1.1)
// Phase 3 (PROMPT 49 v3.0): /DHTS_ENABLE_PHASE3_MIL + 단일 TU 링크 블록 하단 참고
#include "HTS_Jammer_STD.hpp"

#if defined(HTS_ENABLE_PHASE3_MIL)
#include "HTS_Phase3_MIL.hpp"
#endif

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <vector>

// 단일 TU(Phase3)에서 HTS_Jammer_STD.cpp 와 무명 네임스페이스 병합 충돌 방지
namespace Jammer_UT {

constexpr double kPi = 3.14159265358979323846;
constexpr double kChipRateHz = 200000.0;

constexpr double TOL_SIGMA = 0.01;   // v1.1 §14.2: 1%
constexpr double TOL_POWER = 0.01;
constexpr double TOL_PARSEVAL = 1e-6;
constexpr double TOL_DUTY = 0.01;
constexpr double J2_MIN_SEP_DB = 40.0;

int g_fail = 0;

void expect_true(const char* name, bool ok) {
    if (!ok) {
        std::printf("FAIL: %s\n", name);
        ++g_fail;
    } else {
        std::printf("OK:   %s\n", name);
    }
}

void log_trial_seed(std::uint32_t trial_idx, std::uint32_t base_seed,
                    const char* tag) {
    const std::uint32_t trial_seed = derive_seed(base_seed, trial_idx, tag);
    std::printf("[TRIAL-SEED] trial=%u base=0x%08X trial_seed=0x%08X tag=%s\n",
                static_cast<unsigned>(trial_idx), base_seed, trial_seed, tag);
}

void log_saturation(const char* jammer, const SatStats& st) {
    std::printf("[SATURATION] jammer=%s I_sat=%d Q_sat=%d samples=%d rate=%.6f%%\n",
                jammer, static_cast<int>(st.sat_i_count),
                static_cast<int>(st.sat_q_count), static_cast<int>(st.total_samples),
                st.sat_rate_pct());
}

void fft_radix2(std::vector<std::complex<double>>& a, bool inverse) {
    const std::size_t n = a.size();
    std::size_t j = 0;
    for (std::size_t i = 1; i < n; ++i) {
        std::size_t bit = n >> 1u;
        for (; j & bit; bit >>= 1u) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            std::swap(a[i], a[j]);
        }
    }
    for (std::size_t len = 2; len <= n; len <<= 1u) {
        const double ang = 2.0 * kPi / static_cast<double>(len) *
                            (inverse ? 1.0 : -1.0);
        const std::complex<double> wlen(std::cos(ang), std::sin(ang));
        for (std::size_t i = 0; i < n; i += len) {
            std::complex<double> w(1.0, 0.0);
            for (std::size_t k = 0; k < len / 2; ++k) {
                const std::size_t u = i + k;
                const std::size_t v = i + k + len / 2;
                const std::complex<double> x = a[u];
                const std::complex<double> y = a[v] * w;
                a[u] = x + y;
                a[v] = x - y;
                w *= wlen;
            }
        }
    }
    if (inverse) {
        for (std::size_t i = 0; i < n; ++i) {
            a[i] /= static_cast<double>(n);
        }
    }
}

double parseval_rel_err(const std::vector<std::int16_t>& ri,
                        const std::vector<std::int16_t>& rq) {
    const int N = static_cast<int>(ri.size());
    if (N <= 0) {
        return 1.0;
    }
    std::vector<std::complex<double>> X(static_cast<std::size_t>(N));
    for (int n = 0; n < N; ++n) {
        X[static_cast<std::size_t>(n)] =
            std::complex<double>(static_cast<double>(ri[static_cast<std::size_t>(n)]),
                                 static_cast<double>(rq[static_cast<std::size_t>(n)]));
    }
    double sum_time = 0.0;
    for (int n = 0; n < N; ++n) {
        const double I = static_cast<double>(ri[static_cast<std::size_t>(n)]);
        const double Q = static_cast<double>(rq[static_cast<std::size_t>(n)]);
        sum_time += I * I + Q * Q;
    }
    fft_radix2(X, false);
    double sum_freq = 0.0;
    for (int k = 0; k < N; ++k) {
        sum_freq += std::norm(X[static_cast<std::size_t>(k)]);
    }
    const double P_time = sum_time / static_cast<double>(N);
    const double P_freq = sum_freq / (static_cast<double>(N) * static_cast<double>(N));
    return std::abs(P_time - P_freq) / (P_time + 1e-30);
}

double expected_pulse_on_fraction(int n_chips, double D, double T_period) {
    int on = 0;
    for (int n = 0; n < n_chips; ++n) {
        double t_mod = std::fmod(static_cast<double>(n), T_period);
        if (t_mod < 0.0) {
            t_mod += T_period;
        }
        if (t_mod < D * T_period) {
            ++on;
        }
    }
    return static_cast<double>(on) / static_cast<double>(n_chips);
}

void check_parseval_prefix(const char* jammer_tag, const std::vector<std::int16_t>& ri,
                           const std::vector<std::int16_t>& rq, int n_fft) {
    std::vector<std::int16_t> si(static_cast<std::size_t>(n_fft));
    std::vector<std::int16_t> sq(static_cast<std::size_t>(n_fft));
    for (int i = 0; i < n_fft; ++i) {
        si[static_cast<std::size_t>(i)] = ri[static_cast<std::size_t>(i)];
        sq[static_cast<std::size_t>(i)] = rq[static_cast<std::size_t>(i)];
    }
    const double rel = parseval_rel_err(si, sq);
    std::printf("[PARSEVAL] %s: rel_err=%.3e (tol=%.1e)\n", jammer_tag, rel,
                TOL_PARSEVAL);
    expect_true("Parseval v1.1", rel < TOL_PARSEVAL);
}

void test_j1() {
    constexpr int N = 100000;
    constexpr int N_fft = 65536;
    constexpr std::uint32_t kBase = 0xA101A101u;
    log_trial_seed(0u, kBase, "J1");
    std::mt19937 rng(derive_seed(kBase, 0u, "J1"));
    SatStats stats{};
    std::vector<std::int16_t> rI(static_cast<std::size_t>(N), 0);
    std::vector<std::int16_t> rQ(static_cast<std::size_t>(N), 0);
    const double ps = 10000.0;
    const double snr = 10.0;
    ch_j1_awgn(rI.data(), rQ.data(), N, snr, ps, rng, &stats);
    const double p = measure_signal_power(rI.data(), rQ.data(), N);
    const double sigma2 = ps / std::pow(10.0, snr / 10.0);
    const double err = std::abs(p - sigma2) / sigma2;
    std::printf("J1 measured power=%.9f sigma2_target=%.9f err=%.6f%%\n", p, sigma2,
                100.0 * err);
    expect_true("J1 AWGN power err < 1%", err < TOL_SIGMA);
    log_saturation("J1", stats);
    check_parseval_prefix("J1", rI, rQ, N_fft);
}

void test_j4() {
    constexpr int N = 100000;
    constexpr int N_fft = 65536;
    constexpr std::uint32_t kBase = 0xB404B404u;
    log_trial_seed(0u, kBase, "J4");
    std::mt19937 rng(derive_seed(kBase, 0u, "J4"));
    SatStats stats{};
    std::vector<std::int16_t> rI(static_cast<std::size_t>(N), 0);
    std::vector<std::int16_t> rQ(static_cast<std::size_t>(N), 0);
    const double ps = 10000.0;
    const double jsr = 10.0;
    ch_j4_barrage(rI.data(), rQ.data(), N, jsr, ps, rng, &stats);
    const double p = measure_signal_power(rI.data(), rQ.data(), N);
    const double target = ps * std::pow(10.0, jsr / 10.0);
    const double err = std::abs(p - target) / target;
    std::printf("J4 measured power=%.9f P_j=%.9f err=%.6f%%\n", p, target, 100.0 * err);
    expect_true("J4 barrage err < 1%", err < TOL_SIGMA);
    log_saturation("J4", stats);
    check_parseval_prefix("J4", rI, rQ, N_fft);
}

void test_j2_fft() {
    constexpr int N = 2048;
    constexpr int N_fft = 2048;
    constexpr std::uint32_t kBase = 0xC202C202u;
    log_trial_seed(0u, kBase, "J2");
    std::mt19937 rng(derive_seed(kBase, 0u, "J2"));
    SatStats stats{};
    std::vector<std::int16_t> rI(static_cast<std::size_t>(N), 0);
    std::vector<std::int16_t> rQ(static_cast<std::size_t>(N), 0);
    const double ps = 1e6;
    const double jsr = 0.0;
    const double f_off = 50000.0;
    ch_j2_cw(rI.data(), rQ.data(), N, jsr, ps, f_off, rng, &stats);
    const double p_meas = measure_signal_power(rI.data(), rQ.data(), N);
    const double pj = ps * std::pow(10.0, jsr / 10.0);
    std::vector<std::complex<double>> X(static_cast<std::size_t>(N));
    for (int n = 0; n < N; ++n) {
        X[static_cast<std::size_t>(n)] =
            std::complex<double>(static_cast<double>(rI[static_cast<std::size_t>(n)]),
                                 static_cast<double>(rQ[static_cast<std::size_t>(n)]));
    }
    fft_radix2(X, false);
    int peak_k = 0;
    double peak_mag = 0.0;
    for (int k = 0; k < N; ++k) {
        const double m = std::abs(X[static_cast<std::size_t>(k)]);
        if (m > peak_mag) {
            peak_mag = m;
            peak_k = k;
        }
    }
    double adj_sum = 0.0;
    int adj_count = 0;
    for (int k = peak_k - 2; k <= peak_k + 2; ++k) {
        if (k == peak_k) {
            continue;
        }
        if (k < 0 || k >= N) {
            continue;
        }
        adj_sum += std::abs(X[static_cast<std::size_t>(k)]);
        ++adj_count;
    }
    const double adj_mean = adj_sum / static_cast<double>(std::max(1, adj_count));
    const double peak_db = 20.0 * std::log10(peak_mag + 1e-30);
    const double adj_db = 20.0 * std::log10(adj_mean + 1e-30);
    const double sep_adj = peak_db - adj_db;
    std::printf(
        "J2 peak_bin=%d peak=%.2f dB adj_mean=%.2f dB sep_adj=%.2f dB meanPow=%.6f\n",
        peak_k, peak_db, adj_db, sep_adj, p_meas);
    expect_true("J2 peak vs adjacent mean >= 40 dB", sep_adj >= J2_MIN_SEP_DB);
    expect_true("J2 mean power err < 1%", std::abs(p_meas - pj) / pj < TOL_POWER);
    log_saturation("J2", stats);
    check_parseval_prefix("J2", rI, rQ, N_fft);
}

void test_j3_pulse() {
    constexpr int N = 100000;
    constexpr int N_fft = 65536;
    constexpr std::uint32_t kBase = 0xD303D303u;
    log_trial_seed(0u, kBase, "J3");
    std::mt19937 rng(derive_seed(kBase, 0u, "J3"));
    SatStats stats{};
    std::vector<std::int16_t> rI(static_cast<std::size_t>(N), 0);
    std::vector<std::int16_t> rQ(static_cast<std::size_t>(N), 0);
    const double Tp = 100.0;
    const double duty = 0.25;
    const double ps = 10000.0;
    const double jsr_peak = 0.0;
    const double pj = ps * std::pow(10.0, jsr_peak / 10.0);
    ch_j3_pulse(rI.data(), rQ.data(), N, jsr_peak, ps, duty, Tp, 10000.0, 0, rng,
                &stats);
    const double duty_exp = expected_pulse_on_fraction(N, duty, Tp);
    int on = 0;
    for (int n = 0; n < N; ++n) {
        if (rI[static_cast<std::size_t>(n)] != 0 || rQ[static_cast<std::size_t>(n)] != 0) {
            ++on;
        }
    }
    const double duty_m = static_cast<double>(on) / static_cast<double>(N);
    std::printf("J3 duty measured=%.6f expected(discrete)=%.6f target_D=%.4f T=%.3f\n",
                duty_m, duty_exp, duty, Tp);
    expect_true("J3 duty vs discrete envelope", std::abs(duty_m - duty_exp) < 1e-6);
    expect_true("J3 duty vs D tol", std::abs(duty_m - duty) < TOL_DUTY);

    double p_on_sum = 0.0;
    int p_on_cnt = 0;
    for (int n = 0; n < N; ++n) {
        double t_mod = std::fmod(static_cast<double>(n), Tp);
        if (t_mod < 0.0) {
            t_mod += Tp;
        }
        if (t_mod < duty * Tp) {
            const double ii = static_cast<double>(rI[static_cast<std::size_t>(n)]);
            const double qq = static_cast<double>(rQ[static_cast<std::size_t>(n)]);
            p_on_sum += ii * ii + qq * qq;
            ++p_on_cnt;
        }
    }
    const double p_on = (p_on_cnt > 0) ? (p_on_sum / static_cast<double>(p_on_cnt)) : 0.0;
    const double p_on_err = (pj > 0.0) ? std::abs(p_on - pj) / pj : 0.0;
    std::printf("[J3-PULSE] ON_power=%.9f P_j=%.9f err=%.6f%% (ON chips=%d)\n", p_on, pj,
                100.0 * p_on_err, p_on_cnt);
    expect_true("J3 ON interval power err < 1%", p_on_err < TOL_POWER);

    // v1.1 §7.2: fractional period + duty (이산 기대값과 비교)
    std::vector<std::int16_t> r2I(static_cast<std::size_t>(N), 0);
    std::vector<std::int16_t> r2Q(static_cast<std::size_t>(N), 0);
    std::mt19937 rng2(derive_seed(kBase, 1u, "J3b"));
    SatStats stats2{};
    const double Tp2 = 12.5;
    const double duty2 = 0.5;
    const double exp2 = expected_pulse_on_fraction(N, duty2, Tp2);
    ch_j3_pulse(r2I.data(), r2Q.data(), N, jsr_peak, ps, duty2, Tp2, 10000.0, 0, rng2,
                &stats2);
    int on2 = 0;
    for (int n = 0; n < N; ++n) {
        if (r2I[static_cast<std::size_t>(n)] != 0 || r2Q[static_cast<std::size_t>(n)] != 0) {
            ++on2;
        }
    }
    const double duty_m2 = static_cast<double>(on2) / static_cast<double>(N);
    std::printf("J3b duty measured=%.6f expected(discrete)=%.6f D=%.2f T=%.4f\n", duty_m2,
                exp2, duty2, Tp2);
    expect_true("J3b duty matches discrete envelope",
                std::abs(duty_m2 - exp2) < 1e-5);

    log_saturation("J3b", stats2);
    log_saturation("J3", stats);
    check_parseval_prefix("J3", rI, rQ, N_fft);
}

void test_j5_power() {
    constexpr int N = 50000;
    constexpr int N_fft = 8192;
    constexpr std::uint32_t kBase = 0xE505E505u;
    log_trial_seed(0u, kBase, "J5");
    std::mt19937 rng(derive_seed(kBase, 0u, "J5"));
    SatStats stats{};
    std::vector<std::int16_t> rI(static_cast<std::size_t>(N), 0);
    std::vector<std::int16_t> rQ(static_cast<std::size_t>(N), 0);
    const double ps = 1e6;
    const double jsr = 3.0;
    const int tones = 4;
    const double bw = 200000.0;
    ch_j5_multitone(rI.data(), rQ.data(), N, jsr, ps, tones, bw, rng, &stats);
    const double p = measure_signal_power(rI.data(), rQ.data(), N);
    const double pj = ps * std::pow(10.0, jsr / 10.0);
    const double err_tot = std::abs(p - pj) / pj;
    std::printf("J5 mean power=%.9f P_j=%.9f err=%.6f%%\n", p, pj, 100.0 * err_tot);
    expect_true("J5 total power err < 1%", err_tot < TOL_POWER);

    // v1.1 §9.2: 정확 f_k 에서 복소 envelope 상관 → 톤당 평균 전력 ≈ P_j/N
    const double df = bw / static_cast<double>(tones);
    const double f_center = 0.0;
    for (int t = 0; t < tones; ++t) {
        const double fk =
            f_center + (static_cast<double>(t) - 0.5 * static_cast<double>(tones - 1)) * df;
        std::complex<double> acc{0.0, 0.0};
        for (int n = 0; n < N_fft; ++n) {
            const std::complex<double> z(
                static_cast<double>(rI[static_cast<std::size_t>(n)]),
                static_cast<double>(rQ[static_cast<std::size_t>(n)]));
            const double ph = -2.0 * kPi * fk * static_cast<double>(n) / kChipRateHz;
            acc += z * std::complex<double>(std::cos(ph), std::sin(ph));
        }
        const double p_k = std::norm(acc) /
                           (static_cast<double>(N_fft) * static_cast<double>(N_fft));
        const double p_exp = pj / static_cast<double>(tones);
        const double terr = std::abs(p_k - p_exp) / p_exp;
        std::printf("[J5-MT] tone=%d fk=%.1f P_k=%.6e P_exp=%.6e terr=%.4f%%\n", t, fk, p_k,
                    p_exp, 100.0 * terr);
        expect_true("J5 per-tone power err < 1%", terr < TOL_POWER);
    }

    log_saturation("J5", stats);
    check_parseval_prefix("J5", rI, rQ, N_fft);
}

void test_j6_swept() {
    constexpr int N = 200000;
    constexpr int N_fft = 65536;
    constexpr std::uint32_t kBase = 0xF606F606u;
    log_trial_seed(0u, kBase, "J6");
    std::mt19937 rng(derive_seed(kBase, 0u, "J6"));
    SatStats stats{};
    std::vector<std::int16_t> rI(static_cast<std::size_t>(N), 0);
    std::vector<std::int16_t> rQ(static_cast<std::size_t>(N), 0);
    const double pj = 1e8;
    const double f_start = -50000.0;
    const double f_end = 50000.0;
    const double rate = 2e8;
    ch_j6_swept(rI.data(), rQ.data(), N, 0.0, pj, f_start, f_end, rate, rng, &stats);
    const double p = measure_signal_power(rI.data(), rQ.data(), N);
    const double errp = std::abs(p - pj) / pj;
    std::printf("J6 mean power=%.9f P_j=%.9f err=%.6f%%\n", p, pj, 100.0 * errp);
    expect_true("J6 mean power err < 1%", errp < TOL_POWER);

    const double df_band = f_end - f_start;
    const double t_sweep = df_band / rate;
    int wraps = 0;
    double prev_f = f_start + std::fmod(0.0, df_band);
    for (int n = 1; n < N; ++n) {
        const double t_sec = static_cast<double>(n) / kChipRateHz;
        const double f_inst = f_start + std::fmod(rate * t_sec, df_band);
        if (f_inst + 1000.0 < prev_f) {
            ++wraps;
        }
        prev_f = f_inst;
    }
    std::printf("[J6-SWEPT] T_sweep=%.9e s wraps_obs=%d\n", t_sweep, wraps);
    expect_true("J6 sawtooth wrap observed", wraps > 0);

    double max_rel_err = 0.0;
    int smooth_samples = 0;
    std::vector<double> unw(static_cast<std::size_t>(N));
    unw[0] = std::atan2(static_cast<double>(rQ[0]), static_cast<double>(rI[0]));
    for (int n = 1; n < N; ++n) {
        double a = std::atan2(static_cast<double>(rQ[static_cast<std::size_t>(n)]),
                              static_cast<double>(rI[static_cast<std::size_t>(n)]));
        double d = a - unw[static_cast<std::size_t>(n - 1)];
        while (d > kPi) {
            d -= 2.0 * kPi;
        }
        while (d < -kPi) {
            d += 2.0 * kPi;
        }
        unw[static_cast<std::size_t>(n)] = unw[static_cast<std::size_t>(n - 1)] + d;
    }
    for (int n = 8000; n < N - 8000; n += 997) {
        const double f0 =
            f_start + std::fmod(rate * static_cast<double>(n) / kChipRateHz, df_band);
        const double f1 = f_start +
                          std::fmod(rate * static_cast<double>(n + 1) / kChipRateHz, df_band);
        if (f1 < f0 - 0.25 * df_band) {
            continue;
        }
        const double t_mid = (static_cast<double>(n) + 0.5) / kChipRateHz;
        const double f_mid = f_start + std::fmod(rate * t_mid, df_band);
        const double dth_exp = 2.0 * kPi * f_mid / kChipRateHz;
        const double dth_meas =
            unw[static_cast<std::size_t>(n + 1)] - unw[static_cast<std::size_t>(n)];
        const double denom = std::abs(dth_exp) + 1e-12;
        const double rel = std::abs(dth_meas - dth_exp) / denom;
        max_rel_err = std::max(max_rel_err, rel);
        ++smooth_samples;
    }
    std::printf("[J6-SWEPT] phase_step max|rel err| vs 2pi*f/f_chip=%.6f%% (n=%d)\n",
                100.0 * max_rel_err, smooth_samples);
    expect_true("J6 enough smooth phase-step samples", smooth_samples >= 10);
    expect_true("J6 phase-step vs f_inst rel err < 1%", max_rel_err < TOL_POWER);

    log_saturation("J6", stats);
    check_parseval_prefix("J6", rI, rQ, N_fft);
}

void test_derive_seed() {
    const std::uint32_t a = derive_seed(0x111u, 3u, "J4");
    const std::uint32_t b = derive_seed(0x111u, 4u, "J4");
    expect_true("derive_seed trial sensitivity", a != b);
}

void run_phase1_jammer_unit_tests() {
    std::printf("=== HTS_Jammer_STD Phase1 unit tests (v1.1) ===\n");
    test_derive_seed();
    test_j1();
    test_j4();
    test_j2_fft();
    test_j3_pulse();
    test_j5_power();
    test_j6_swept();
    std::printf("=== Phase1 done failures=%d ===\n", g_fail);
}

} // namespace Jammer_UT

int main() {
    Jammer_UT::run_phase1_jammer_unit_tests();
#if defined(HTS_ENABLE_PHASE3_MIL)
    Phase3_MIL::run_phase3_limit_finding();
#endif
    return (Jammer_UT::g_fail > 0) ? 1 : 0;
}

// ── Phase 3 단일 TU 링크 (기본 빌드에서는 컴파일 제외: vcxproj 2-file Phase1 유지) ──
#if defined(HTS_ENABLE_PHASE3_MIL) && defined(HTS_LINK_JAMMER_STD_IN_UNITTEST)
#include "HTS_Jammer_STD.cpp"
#include "../../HTS_LIM/HTS_Secure_Memory.cpp"
#include "../../HTS_LIM/HTS_Polar_Codec.cpp"
#include "../../HTS_LIM/HTS_FEC_HARQ.cpp"
#include "../../HTS_LIM/HTS_Holo_LPI.cpp"
#include "../../HTS_LIM/HTS_Walsh_Row_Permuter.cpp"
#include "HTS_Session_Derive_Stub.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Core.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Sync_PSLTE.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Payload.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_TX.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher_Decode.cpp"
#include "HTS_BER_PER_Measure.cpp"
#endif


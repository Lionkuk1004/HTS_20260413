// HTS_BER_PER_Measure.cpp — Phase 2 BER/PER + Clopper-Pearson (Beta exact, boost 미사용)
#include "HTS_BER_PER_Measure.hpp"
#include "HTS_V400_Dispatcher.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#if defined(HTS_PHASE3_NOISE_DIAG)
static int g_measure_diag_counter = 0;
#endif

using ProtectedEngine::DecodedPacket;
using ProtectedEngine::FEC_HARQ;
using ProtectedEngine::HTS_V400_Dispatcher;
using ProtectedEngine::PayloadMode;

namespace {

constexpr int16_t kAmp = 1000;
constexpr int kPreReps = 4;
constexpr int kPreBoost = 1;
constexpr int kMaxC = 2048 + (FEC_HARQ::NSYM64 + kPreReps + 4) * 64;
constexpr int kGuard = 256;

static DecodedPacket g_last{};
static void on_pkt(const DecodedPacket& p) { g_last = p; }

static void setup(HTS_V400_Dispatcher& d, std::uint32_t seed) noexcept {
    d.Set_Seed(seed);
    d.Set_IR_Mode(true);
    d.Set_Preamble_Boost(kPreBoost);
    d.Set_Preamble_Reps(kPreReps);
    d.Set_Packet_Callback(on_pkt);
    d.Update_Adaptive_BPS(1000);
    d.Set_Lab_IQ_Mode_Jam_Harness();
}

static void fill_info(std::uint32_t seed, int t, std::uint8_t* info) noexcept {
    std::uint32_t s = seed + static_cast<std::uint32_t>(t) * 0x6C62272Eu + 1u;
    for (int b = 0; b < 8; ++b) {
        s += 0x9E3779B9u;
        std::uint32_t z = s;
        z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
        z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
        z = z ^ (z >> 16u);
        info[b] = static_cast<std::uint8_t>(z & 0xFFu);
    }
}

static int feed_metrics(std::uint32_t ds, const std::int16_t* rxI, const std::int16_t* rxQ,
                        int n, const std::uint8_t* expected, int* bit_errors,
                        bool* crc_ok) noexcept {
    g_last = DecodedPacket{};
    HTS_V400_Dispatcher rx;
    setup(rx, ds);
    for (int i = 0; i < kGuard; ++i) {
        rx.Feed_Chip(0, 0);
    }
    for (int i = 0; i < n; ++i) {
        rx.Feed_Chip(rxI[i], rxQ[i]);
    }
    for (int i = 0; i < kGuard; ++i) {
        rx.Feed_Chip(0, 0);
    }
    *crc_ok = (g_last.success_mask == DecodedPacket::DECODE_MASK_OK) &&
              (g_last.data_len == 8);
    if (!*crc_ok) {
        *bit_errors = 64;
        return 64;
    }
    int bit_err = 0;
    for (int b = 0; b < 8; ++b) {
        std::uint8_t d = static_cast<std::uint8_t>(
            static_cast<std::uint8_t>(g_last.data[b]) ^ expected[b]);
        if (d != 0u) {
            d = static_cast<std::uint8_t>(d - ((d >> 1) & 0x55u));
            d = static_cast<std::uint8_t>((d & 0x33u) + ((d >> 2) & 0x33u));
            d = static_cast<std::uint8_t>((d + (d >> 4)) & 0x0Fu);
            bit_err += static_cast<int>(d);
        }
    }
    *bit_errors = bit_err;
    return bit_err;
}

// ── Incomplete Beta I_x(a,b) = B(x;a,b)/B(a,b) — NR 스타일 Lentz CF (boost 미사용) ──
static double lbeta_fn(double a, double b) noexcept {
    return std::lgamma(a) + std::lgamma(b) - std::lgamma(a + b);
}

static double betacf(double a, double b, double x) noexcept {
    constexpr int max_iter = 200;
    constexpr double eps = 3.0e-14;
    constexpr double fpmin = 1e-30;
    double qab = a + b;
    double qap = a + 1.0;
    double qam = a - 1.0;
    double c = 1.0;
    double d = 1.0 - qab * x / qap;
    if (std::fabs(d) < fpmin) {
        d = fpmin;
    }
    d = 1.0 / d;
    double h = d;
    for (int m = 1, m2 = 2; m <= max_iter; ++m, m2 += 2) {
        double aa = static_cast<double>(m) * (b - static_cast<double>(m)) * x /
                     ((qam + static_cast<double>(m2)) * (a + static_cast<double>(m2)));
        d = 1.0 + aa * d;
        if (std::fabs(d) < fpmin) {
            d = fpmin;
        }
        c = 1.0 + aa / c;
        if (std::fabs(c) < fpmin) {
            c = fpmin;
        }
        d = 1.0 / d;
        h *= d * c;
        aa = -(a + static_cast<double>(m)) * (qab + static_cast<double>(m)) * x /
             ((a + static_cast<double>(m2)) * (qap + static_cast<double>(m2)));
        d = 1.0 + aa * d;
        if (std::fabs(d) < fpmin) {
            d = fpmin;
        }
        c = 1.0 + aa / c;
        if (std::fabs(c) < fpmin) {
            c = fpmin;
        }
        d = 1.0 / d;
        const double del = d * c;
        h *= del;
        if (std::fabs(del - 1.0) < eps) {
            break;
        }
    }
    return h;
}

static double ibeta(double a, double b, double x) noexcept {
    if (x <= 0.0) {
        return 0.0;
    }
    if (x >= 1.0) {
        return 1.0;
    }
    const double bt =
        std::exp(a * std::log(x) + b * std::log(1.0 - x) - lbeta_fn(a, b));
    if (x < (a + 1.0) / (a + b + 2.0)) {
        return bt * betacf(a, b, x) / a;
    }
    return 1.0 - bt * betacf(b, a, 1.0 - x) / b;
}

static double beta_quantile(double p, double a, double b) noexcept {
    if (p <= 0.0) {
        return 0.0;
    }
    if (p >= 1.0) {
        return 1.0;
    }
    double lo = 0.0;
    double hi = 1.0;
    for (int it = 0; it < 80; ++it) {
        const double mid = 0.5 * (lo + hi);
        const double v = ibeta(a, b, mid);
        if (v < p) {
            lo = mid;
        } else {
            hi = mid;
        }
        if (hi - lo < 1e-14) {
            break;
        }
    }
    return 0.5 * (lo + hi);
}

static void apply_jammer(const HTS_Phase2::ChannelParams& p, std::int16_t* rI,
                         std::int16_t* rQ, int n, std::mt19937& rng, SatStats* st) noexcept {
    switch (p.jammer) {
    case HTS_Phase2::JammerId::J1:
        ch_j1_awgn(rI, rQ, n, p.snr_db, p.signal_power, rng, st);
        break;
    case HTS_Phase2::JammerId::J2:
        ch_j2_cw(rI, rQ, n, p.jsr_db, p.signal_power, p.f_offset_hz, rng, st);
        break;
    case HTS_Phase2::JammerId::J3:
        ch_j3_pulse(rI, rQ, n, p.jsr_peak_db, p.signal_power, p.duty_cycle, p.period_chips,
                    p.f_off_pulse_hz, p.pulse_mode, rng, st);
        break;
    case HTS_Phase2::JammerId::J4:
        ch_j4_barrage(rI, rQ, n, p.jsr_db, p.signal_power, rng, st);
        break;
    case HTS_Phase2::JammerId::J5:
        ch_j5_multitone(rI, rQ, n, p.jsr_db, p.signal_power, p.tone_count, p.bw_system_hz, rng,
                        st);
        break;
    case HTS_Phase2::JammerId::J6:
        ch_j6_swept(rI, rQ, n, p.jsr_db, p.signal_power, p.f_start_hz, p.f_end_hz,
                    p.rate_hz_per_sec, rng, st);
        break;
    default:
        break;
    }
}

} // namespace

namespace HTS_Phase2 {

std::pair<double, double> clopper_pearson_ci(std::int64_t n, std::int64_t x,
                                              double alpha) noexcept {
    if (n <= 0 || x < 0 || x > n) {
        return {0.0, 1.0};
    }
    const double a2 = 0.5 * alpha;
    if (x == 0) {
        const double upper = 1.0 - std::pow(a2, 1.0 / static_cast<double>(n));
        return {0.0, upper};
    }
    if (x == n) {
        const double lower = std::pow(a2, 1.0 / static_cast<double>(n));
        return {lower, 1.0};
    }
    const double a_lo = static_cast<double>(x);
    const double b_lo = static_cast<double>(n - x + 1);
    const double a_hi = static_cast<double>(x + 1);
    const double b_hi = static_cast<double>(n - x);
    const double lower = beta_quantile(a2, a_lo, b_lo);
    const double upper = beta_quantile(1.0 - a2, a_hi, b_hi);
    return {lower, upper};
}

double theoretical_ber_bpsk_awgn(double snr_db, double gp_db) noexcept {
    const double eb_n0_db = snr_db + gp_db;
    const double eb_n0 = std::pow(10.0, eb_n0_db / 10.0);
    const double z = std::sqrt(std::max(0.0, eb_n0));
    return 0.5 * std::erfc(z);
}

bool ber_matches_theory(double simulated_ber, double theoretical_ber, double tol_db) noexcept {
    if (theoretical_ber <= 0.0 || simulated_ber < 0.0) {
        return false;
    }
    const double s = std::max(simulated_ber, 1e-30);
    const double t = std::max(theoretical_ber, 1e-30);
    const double ratio_db = 10.0 * (std::log10(s) - std::log10(t));
    // Uncoded BPSK+AWGN 상한: 시뮬이 이론보다 좋으면(FEC/확산) ±1 dB 양방향 대신 상한만 검사.
    if (ratio_db <= 0.0) {
        return true;
    }
    return ratio_db <= tol_db;
}

BERPERResult measure_ber_per(const ChannelParams& params, std::int32_t trial_count,
                             std::uint32_t base_seed, double target_ber, double target_per) {
    BERPERResult r{};
    r.base_seed = base_seed;
    if (trial_count < 1) {
        trial_count = 1;
    }
    std::int64_t tot_bits = 0;
    std::int64_t tot_err = 0;
    std::int64_t tot_pkt = 0;
    std::int64_t pkt_err = 0;
    std::int32_t timeouts = 0;

    for (int t = 0; t < trial_count; ++t) {
        const std::uint32_t ds =
            derive_seed(base_seed, static_cast<std::uint32_t>(t), "ber_per");
        std::mt19937 rng_j(
            derive_seed(base_seed, static_cast<std::uint32_t>(t), "jammer_ch"));

        std::int16_t I[kMaxC]{};
        std::int16_t Q[kMaxC]{};
        std::uint8_t info[8]{};
        fill_info(ds, t, info);

        HTS_V400_Dispatcher tx;
        setup(tx, ds);
        const int n = tx.Build_Packet(PayloadMode::DATA, info, 8, kAmp, I, Q, kMaxC);
        ChannelParams jam = params;
        // SPEC §6.1: P_s 는 (I²+Q²)/N RMS. 기본 10000 과 TX 칩(kAmp 등) 불일치 시 잡음 과소 → J1 만 실측 P_s 로 정합.
        if (n > 0 && jam.jammer == JammerId::J1) {
            jam.signal_power = measure_signal_power(I, Q, n);
        }
        SatStats jam_st{};
        apply_jammer(jam, I, Q, n, rng_j, &jam_st);
        r.rx_sat.sat_i_count += jam_st.sat_i_count;
        r.rx_sat.sat_q_count += jam_st.sat_q_count;
        r.rx_sat.total_samples += jam_st.total_samples;

        int be = 0;
        bool crc_ok = false;
        const int err = feed_metrics(ds, I, Q, n, info, &be, &crc_ok);
        (void)err;
#if defined(HTS_PHASE3_NOISE_DIAG)
        if (g_measure_diag_counter < 30) {
            std::printf("[DIAG-3-MEASURE] cnt=%d snr=%.1f "
                        "tx=[%02X %02X %02X %02X %02X %02X %02X %02X] "
                        "rx=[%02X %02X %02X %02X %02X %02X %02X %02X] "
                        "bit_err=%d decode_ok=%d\n",
                        g_measure_diag_counter, static_cast<double>(jam.snr_db), info[0], info[1],
                        info[2], info[3], info[4], info[5], info[6], info[7], g_last.data[0],
                        g_last.data[1], g_last.data[2], g_last.data[3], g_last.data[4],
                        g_last.data[5], g_last.data[6], g_last.data[7], be, crc_ok ? 1 : 0);
            ++g_measure_diag_counter;
        }
#endif
        if (!crc_ok) {
            ++timeouts;
        }
        tot_bits += 64;
        tot_err += be;
        ++tot_pkt;
        if (be != 0 || !crc_ok) {
            ++pkt_err;
        }
    }

    r.total_bits = tot_bits;
    r.bit_errors = tot_err;
    r.total_packets = tot_pkt;
    r.packet_errors = pkt_err;
    r.timeout_trials = timeouts;

    r.ber = (tot_bits > 0) ? static_cast<double>(tot_err) / static_cast<double>(tot_bits) : 0.0;
    r.per = (tot_pkt > 0) ? static_cast<double>(pkt_err) / static_cast<double>(tot_pkt) : 0.0;

    const auto ber_ci = clopper_pearson_ci(tot_bits, tot_err, 0.05);
    r.ber_ci_lower = ber_ci.first;
    r.ber_ci_upper = ber_ci.second;
    const auto per_ci = clopper_pearson_ci(tot_pkt, pkt_err, 0.05);
    r.per_ci_lower = per_ci.first;
    r.per_ci_upper = per_ci.second;

    r.pass_ber = (r.ber_ci_upper < target_ber);
    r.pass_per = (r.per_ci_upper < target_per);
    return r;
}

BERPERResult measure_ber_per_with_per_trial_bit_errors(
    const ChannelParams& params, std::int32_t trial_count, std::uint32_t base_seed,
    double target_ber, double target_per, std::vector<std::uint32_t>& out_bit_errors_per_trial) {
    out_bit_errors_per_trial.clear();
    BERPERResult r{};
    r.base_seed = base_seed;
    if (trial_count < 1) {
        trial_count = 1;
    }
    out_bit_errors_per_trial.reserve(static_cast<std::size_t>(trial_count));
    std::int64_t tot_bits = 0;
    std::int64_t tot_err = 0;
    std::int64_t tot_pkt = 0;
    std::int64_t pkt_err = 0;
    std::int32_t timeouts = 0;

    for (int t = 0; t < trial_count; ++t) {
        const std::uint32_t ds =
            derive_seed(base_seed, static_cast<std::uint32_t>(t), "ber_per");
        std::mt19937 rng_j(
            derive_seed(base_seed, static_cast<std::uint32_t>(t), "jammer_ch"));

        std::int16_t I[kMaxC]{};
        std::int16_t Q[kMaxC]{};
        std::uint8_t info[8]{};
        fill_info(ds, t, info);

        HTS_V400_Dispatcher tx;
        setup(tx, ds);
        const int n = tx.Build_Packet(PayloadMode::DATA, info, 8, kAmp, I, Q, kMaxC);
        ChannelParams jam = params;
        if (n > 0 && jam.jammer == JammerId::J1) {
            jam.signal_power = measure_signal_power(I, Q, n);
        }
        SatStats jam_st{};
        apply_jammer(jam, I, Q, n, rng_j, &jam_st);
        r.rx_sat.sat_i_count += jam_st.sat_i_count;
        r.rx_sat.sat_q_count += jam_st.sat_q_count;
        r.rx_sat.total_samples += jam_st.total_samples;

        int be = 0;
        bool crc_ok = false;
        const int err = feed_metrics(ds, I, Q, n, info, &be, &crc_ok);
        (void)err;
#if defined(HTS_PHASE3_NOISE_DIAG)
        if (g_measure_diag_counter < 30) {
            std::printf("[DIAG-3-MEASURE] cnt=%d snr=%.1f "
                        "tx=[%02X %02X %02X %02X %02X %02X %02X %02X] "
                        "rx=[%02X %02X %02X %02X %02X %02X %02X %02X] "
                        "bit_err=%d decode_ok=%d\n",
                        g_measure_diag_counter, static_cast<double>(jam.snr_db), info[0], info[1],
                        info[2], info[3], info[4], info[5], info[6], info[7], g_last.data[0],
                        g_last.data[1], g_last.data[2], g_last.data[3], g_last.data[4],
                        g_last.data[5], g_last.data[6], g_last.data[7], be, crc_ok ? 1 : 0);
            ++g_measure_diag_counter;
        }
#endif
        out_bit_errors_per_trial.push_back(static_cast<std::uint32_t>(be));
        if (!crc_ok) {
            ++timeouts;
        }
        tot_bits += 64;
        tot_err += be;
        ++tot_pkt;
        if (be != 0 || !crc_ok) {
            ++pkt_err;
        }
    }

    r.total_bits = tot_bits;
    r.bit_errors = tot_err;
    r.total_packets = tot_pkt;
    r.packet_errors = pkt_err;
    r.timeout_trials = timeouts;

    r.ber = (tot_bits > 0) ? static_cast<double>(tot_err) / static_cast<double>(tot_bits) : 0.0;
    r.per = (tot_pkt > 0) ? static_cast<double>(pkt_err) / static_cast<double>(tot_pkt) : 0.0;

    const auto ber_ci = clopper_pearson_ci(tot_bits, tot_err, 0.05);
    r.ber_ci_lower = ber_ci.first;
    r.ber_ci_upper = ber_ci.second;
    const auto per_ci = clopper_pearson_ci(tot_pkt, pkt_err, 0.05);
    r.per_ci_lower = per_ci.first;
    r.per_ci_upper = per_ci.second;

    r.pass_ber = (r.ber_ci_upper < target_ber);
    r.pass_per = (r.per_ci_upper < target_per);
    return r;
}

} // namespace HTS_Phase2

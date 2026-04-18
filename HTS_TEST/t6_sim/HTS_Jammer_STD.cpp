// HTS_Jammer_STD.cpp — Phase 1 표준 재밍 (ch_j1 … ch_j6)
#include "HTS_Jammer_STD.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kChipRateHz = 200000.0;

inline void add_iq(std::int16_t* rI, std::int16_t* rQ, int idx, double di,
                   double dq, SatStats* sat) noexcept {
    const double xi = static_cast<double>(rI[idx]) + di;
    const double xq = static_cast<double>(rQ[idx]) + dq;
    const bool clip_i = (xi > 32767.0) || (xi < -32768.0);
    const bool clip_q = (xq > 32767.0) || (xq < -32768.0);
    if (sat != nullptr) {
        if (clip_i) {
            ++sat->sat_i_count;
        }
        if (clip_q) {
            ++sat->sat_q_count;
        }
        ++sat->total_samples;
    }
    rI[idx] = clip_int16(xi);
    rQ[idx] = clip_int16(xq);
}

} // namespace

double measure_signal_power(const std::int16_t* I, const std::int16_t* Q,
                            int N) noexcept {
    if (N <= 0) {
        return 0.0;
    }
    std::int64_t acc = 0;
    for (int n = 0; n < N; ++n) {
        acc += static_cast<std::int64_t>(I[n]) * I[n];
        acc += static_cast<std::int64_t>(Q[n]) * Q[n];
    }
    return static_cast<double>(acc) / static_cast<double>(N);
}

std::int16_t clip_int16(double x) noexcept {
    if (x > 32767.0) {
        return 32767;
    }
    if (x < -32768.0) {
        return -32768;
    }
    return static_cast<std::int16_t>(std::llround(x));
}

std::uint32_t derive_seed(std::uint32_t base_seed, std::uint32_t trial_idx,
                          const char* channel_name) noexcept {
    std::uint32_t h = 2166136261u;
    h = (h ^ base_seed) * 16777619u;
    h = (h ^ trial_idx) * 16777619u;
    if (channel_name != nullptr) {
        for (const char* p = channel_name; *p != '\0'; ++p) {
            h = (h ^ static_cast<std::uint32_t>(
                     static_cast<unsigned char>(*p))) *
                16777619u;
        }
    }
    return h;
}

void ch_j1_awgn(std::int16_t* rI, std::int16_t* rQ, int n_chips, double snr_db,
                double signal_power, std::mt19937& rng,
                SatStats* sat) noexcept {
    if (n_chips <= 0 || signal_power < 0.0) {
        return;
    }
    // σ²_total = P_s / 10^(SNR/10); n_I,n_Q ~ N(0, σ²/2) ⇒ Var_I+Var_Q = σ²
    const double sigma2 = signal_power / std::pow(10.0, snr_db / 10.0);
    const double sigma_branch = std::sqrt(0.5 * sigma2);
    std::normal_distribution<double> nd(0.0, sigma_branch);
    for (int n = 0; n < n_chips; ++n) {
        add_iq(rI, rQ, n, nd(rng), nd(rng), sat);
    }
}

void ch_j2_cw(std::int16_t* rI, std::int16_t* rQ, int n_chips, double jsr_db,
              double signal_power, double f_offset_hz, std::mt19937& rng,
              SatStats* sat) noexcept {
    if (n_chips <= 0 || signal_power < 0.0) {
        return;
    }
    const double pj = signal_power * std::pow(10.0, jsr_db / 10.0);
    const double A = std::sqrt(std::max(0.0, pj));
    std::uniform_real_distribution<double> ph_u(0.0, 2.0 * kPi);
    const double phi = ph_u(rng);
    const double dphi = 2.0 * kPi * f_offset_hz / kChipRateHz;
    for (int n = 0; n < n_chips; ++n) {
        const double ph = phi + dphi * static_cast<double>(n);
        add_iq(rI, rQ, n, A * std::cos(ph), A * std::sin(ph), sat);
    }
}

void ch_j3_pulse(std::int16_t* rI, std::int16_t* rQ, int n_chips,
                 double jsr_peak_db, double signal_power, double duty_cycle,
                 double period_chips, double f_offset_hz, int mode,
                 std::mt19937& rng, SatStats* sat) noexcept {
    if (n_chips <= 0 || signal_power < 0.0) {
        return;
    }
    // v1.1 §7.2: 실수 T_period, fmod — 정수 반올림·최소 1칩 ON 강제 금지
    const double T_period = period_chips;
    if (T_period <= 0.0) {
        return;
    }
    const double D = std::min(1.0, std::max(0.0, duty_cycle));

    const double pj_on = signal_power * std::pow(10.0, jsr_peak_db / 10.0);
    const double A_on = std::sqrt(std::max(0.0, pj_on));
    const double dphi = 2.0 * kPi * f_offset_hz / kChipRateHz;
    std::uniform_real_distribution<double> ph_u(0.0, 2.0 * kPi);
    const double phi0 = ph_u(rng);
    const double sigma_b = A_on / std::sqrt(2.0);
    std::normal_distribution<double> nd_awgn(0.0, sigma_b);

    for (int n = 0; n < n_chips; ++n) {
        double t_mod = std::fmod(static_cast<double>(n), T_period);
        if (t_mod < 0.0) {
            t_mod += T_period;
        }
        const bool on = (t_mod < D * T_period);
        if (!on) {
            if (sat != nullptr) {
                ++sat->total_samples;
            }
            continue;
        }
        if (mode == 0) {
            const double ph = phi0 + dphi * static_cast<double>(n);
            add_iq(rI, rQ, n, A_on * std::cos(ph), A_on * std::sin(ph), sat);
        } else {
            add_iq(rI, rQ, n, nd_awgn(rng), nd_awgn(rng), sat);
        }
    }
}

void ch_j4_barrage(std::int16_t* rI, std::int16_t* rQ, int n_chips,
                   double jsr_db, double signal_power, std::mt19937& rng,
                   SatStats* sat) noexcept {
    if (n_chips <= 0 || signal_power < 0.0) {
        return;
    }
    const double pj = signal_power * std::pow(10.0, jsr_db / 10.0);
    const double sigma_branch = std::sqrt(0.5 * pj);
    std::normal_distribution<double> nd(0.0, sigma_branch);
    for (int n = 0; n < n_chips; ++n) {
        add_iq(rI, rQ, n, nd(rng), nd(rng), sat);
    }
}

void ch_j5_multitone(std::int16_t* rI, std::int16_t* rQ, int n_chips,
                     double jsr_db, double signal_power, int tone_count,
                     double bw_system_hz, std::mt19937& rng,
                     SatStats* sat) noexcept {
    if (n_chips <= 0 || signal_power < 0.0 || tone_count < 2) {
        return;
    }
    const double pj = signal_power * std::pow(10.0, jsr_db / 10.0);
    const double A = std::sqrt(std::max(0.0, pj));
    const int N = tone_count;
    const double inv_sqrt_n = 1.0 / std::sqrt(static_cast<double>(N));
    const double df = bw_system_hz / static_cast<double>(N);
    // v1.1 §9.3: f_center = 0 (복소 baseband 중심, carrier 제거 후) — API 인자 불필요
    const double f_center = 0.0;
    std::vector<double> phi_k(static_cast<std::size_t>(N));
    std::uniform_real_distribution<double> ph_u(0.0, 2.0 * kPi);
    for (int k = 0; k < N; ++k) {
        phi_k[static_cast<std::size_t>(k)] = ph_u(rng);
    }
    for (int n = 0; n < n_chips; ++n) {
        double sumI = 0.0;
        double sumQ = 0.0;
        for (int k = 0; k < N; ++k) {
            const double fk =
                f_center + (static_cast<double>(k) - 0.5 * static_cast<double>(N - 1)) * df;
            const double ph =
                phi_k[static_cast<std::size_t>(k)] +
                2.0 * kPi * fk * static_cast<double>(n) / kChipRateHz;
            sumI += std::cos(ph);
            sumQ += std::sin(ph);
        }
        const double s = A * inv_sqrt_n;
        add_iq(rI, rQ, n, s * sumI, s * sumQ, sat);
    }
}

void ch_j6_swept(std::int16_t* rI, std::int16_t* rQ, int n_chips, double jsr_db,
                 double signal_power, double f_start_hz, double f_end_hz,
                 double rate_hz_per_sec, std::mt19937& rng,
                 SatStats* sat) noexcept {
    if (n_chips <= 0 || signal_power < 0.0) {
        return;
    }
    const double pj = signal_power * std::pow(10.0, jsr_db / 10.0);
    const double A = std::sqrt(std::max(0.0, pj));
    std::uniform_real_distribution<double> ph_u(0.0, 2.0 * kPi);
    const double phi0 = ph_u(rng);
    const double band = f_end_hz - f_start_hz;
    if (band <= 0.0) {
        return;
    }
    // v1.1 §10.5: sawtooth f_inst, 위상은 (2π)·Σ(f_inst/f_chip)+φ_0 로 연속 누적
    double phase_cycles = 0.0;
    for (int n = 0; n < n_chips; ++n) {
        const double t_sec = static_cast<double>(n) / kChipRateHz;
        const double u = std::fmod(rate_hz_per_sec * t_sec, band);
        const double f_inst = f_start_hz + u;
        phase_cycles += f_inst / kChipRateHz;
        const double p = 2.0 * kPi * phase_cycles + phi0;
        add_iq(rI, rQ, n, A * std::cos(p), A * std::sin(p), sat);
    }
}

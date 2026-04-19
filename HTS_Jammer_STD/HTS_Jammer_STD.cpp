// =============================================================================
/// @file  HTS_Jammer_STD.cpp
/// @brief SPEC_002 v1.0 수학 정의서 엄격 구현
/// @target PC 시뮬 전용
// =============================================================================
#include "HTS_Jammer_STD.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace HTS_Jammer_STD {
// ── 공통 유틸 ─────────────────────────────────────────────────
namespace {
constexpr int16_t I16_MAX = 32767;
constexpr int16_t I16_MIN = -32768;
inline int16_t sat16(int32_t v) noexcept {
    if (v > I16_MAX)
        return I16_MAX;
    if (v < I16_MIN)
        return I16_MIN;
    return static_cast<int16_t>(v);
}
inline int16_t sat16_d(double v) noexcept {
    if (v >= 32767.0)
        return I16_MAX;
    if (v <= -32768.0)
        return I16_MIN;
    return static_cast<int16_t>(std::lround(v));
}
inline double db_to_linear(double db) noexcept {
    return std::pow(10.0, db / 10.0);
}
} // anonymous namespace
// ── §3 Signal power (RMS 평균) ────────────────────────────────
double Measure_Signal_Power(const int16_t *I, const int16_t *Q,
                            int N) noexcept {
    if (I == nullptr || Q == nullptr || N <= 0)
        return 0.0;
    // SPEC_002 §3.1: P_s = (1/N) · Σ (I²+Q²)
    double sum = 0.0;
    for (int n = 0; n < N; ++n) {
        const double i = static_cast<double>(I[n]);
        const double q = static_cast<double>(Q[n]);
        sum += i * i + q * q;
    }
    return sum / static_cast<double>(N);
}
// ── §4 AWGN 잡음 ─────────────────────────────────────────────
void Add_AWGN(int16_t *I, int16_t *Q, int N, double snr_db, double P_s,
              std::mt19937 &rng) noexcept {
    if (I == nullptr || Q == nullptr || N <= 0 || P_s <= 0.0)
        return;
    // SPEC_002 §4.1~4.2
    //   σ²_total = P_s / 10^(SNR/10)
    //   I ~ N(0, σ²/2), Q ~ N(0, σ²/2) 독립
    const double P_n = P_s / db_to_linear(snr_db);
    const double sigma = std::sqrt(P_n / 2.0);
    std::normal_distribution<double> nd(0.0, sigma);
    for (int n = 0; n < N; ++n) {
        const double ni = nd(rng);
        const double nq = nd(rng);
        I[n] = sat16_d(static_cast<double>(I[n]) + ni);
        Q[n] = sat16_d(static_cast<double>(Q[n]) + nq);
    }
}
// ── §6 CW Tone ───────────────────────────────────────────────
void Add_CW(int16_t *I, int16_t *Q, int N, double jsr_db, double P_s,
            double f_offset_Hz, double f_chip_Hz, std::mt19937 &rng) noexcept {
    if (I == nullptr || Q == nullptr || N <= 0 || P_s <= 0.0)
        return;
    // SPEC_002 §5.3: A = sqrt(P_j), P_j = P_s · 10^(JSR/10)
    const double P_j = P_s * db_to_linear(jsr_db);
    const double A = std::sqrt(P_j);
    // SPEC_002 §6.2: φ ~ U[0, 2π) 독립
    std::uniform_real_distribution<double> phi_dist(0.0, 2.0 * M_PI);
    const double phi = phi_dist(rng);
    const double omega = 2.0 * M_PI * f_offset_Hz / f_chip_Hz;
    for (int n = 0; n < N; ++n) {
        const double ph = omega * static_cast<double>(n) + phi;
        const double ji = A * std::cos(ph);
        const double jq = A * std::sin(ph);
        I[n] = sat16_d(static_cast<double>(I[n]) + ji);
        Q[n] = sat16_d(static_cast<double>(Q[n]) + jq);
    }
}
// ── §7 Pulse (rectangular, Peak JSR) ──────────────────────────
void Add_Pulse(int16_t *I, int16_t *Q, int N, double peak_jsr_db, double P_s,
               double duty, int T_period_chips, double f_offset_Hz,
               double f_chip_Hz, std::mt19937 &rng) noexcept {
    if (I == nullptr || Q == nullptr || N <= 0 || P_s <= 0.0)
        return;
    if (T_period_chips <= 0 || duty <= 0.0 || duty > 1.0)
        return;
    // §7.3 Peak JSR (MIL-STD 관례): A_ON = sqrt(P_j)
    const double P_j_peak = P_s * db_to_linear(peak_jsr_db);
    const double A = std::sqrt(P_j_peak);
    // Pulse 내부는 CW 톤 (§7.1: j = envelope · j_CW)
    std::uniform_real_distribution<double> phi_dist(0.0, 2.0 * M_PI);
    const double phi = phi_dist(rng);
    const double omega = 2.0 * M_PI * f_offset_Hz / f_chip_Hz;
    const int on_chips = static_cast<int>(std::floor(duty * T_period_chips));
    for (int n = 0; n < N; ++n) {
        const int n_in_period = n % T_period_chips;
        // §7.4: rectangular, sharp edge (smooth 금지)
        if (n_in_period < on_chips) {
            const double ph = omega * static_cast<double>(n) + phi;
            const double ji = A * std::cos(ph);
            const double jq = A * std::sin(ph);
            I[n] = sat16_d(static_cast<double>(I[n]) + ji);
            Q[n] = sat16_d(static_cast<double>(Q[n]) + jq);
        }
    }
}
// ── §8 Barrage (Complex Gaussian) ─────────────────────────────
void Add_Barrage(int16_t *I, int16_t *Q, int N, double jsr_db, double P_s,
                 std::mt19937 &rng) noexcept {
    if (I == nullptr || Q == nullptr || N <= 0 || P_s <= 0.0)
        return;
    // §8.1: σ_j² = P_j, I ~ N(0, σ_j/√2), Q ~ N(0, σ_j/√2)
    const double P_j = P_s * db_to_linear(jsr_db);
    const double sigma = std::sqrt(P_j / 2.0);
    std::normal_distribution<double> nd(0.0, sigma);
    for (int n = 0; n < N; ++n) {
        const double ji = nd(rng);
        const double jq = nd(rng);
        I[n] = sat16_d(static_cast<double>(I[n]) + ji);
        Q[n] = sat16_d(static_cast<double>(Q[n]) + jq);
    }
}
// ── §9 Multi-tone ─────────────────────────────────────────────
void Add_MultiTone(int16_t *I, int16_t *Q, int N, double jsr_db, double P_s,
                   int N_tones, double BW_Hz, double f_chip_Hz,
                   std::mt19937 &rng) noexcept {
    if (I == nullptr || Q == nullptr || N <= 0 || P_s <= 0.0 || N_tones <= 0)
        return;
    // §9.1: A = sqrt(P_j), 1/√N 전력 보존 (§9.2)
    const double P_j = P_s * db_to_linear(jsr_db);
    const double A = std::sqrt(P_j);
    const double A_per_tone = A / std::sqrt(static_cast<double>(N_tones));
    // §9.3: f_k = (k - (N-1)/2) · Δf, Δf = BW/N (f_center = 0)
    const double df = BW_Hz / static_cast<double>(N_tones);
    // §9.4: φ_k iid uniform
    std::uniform_real_distribution<double> phi_dist(0.0, 2.0 * M_PI);
    std::vector<double> f_k(N_tones), phi_k(N_tones);
    for (int k = 0; k < N_tones; ++k) {
        f_k[k] = (static_cast<double>(k) - 0.5 * (N_tones - 1)) * df;
        phi_k[k] = phi_dist(rng);
    }
    for (int n = 0; n < N; ++n) {
        double ji = 0.0, jq = 0.0;
        const double t_n = static_cast<double>(n) / f_chip_Hz;
        for (int k = 0; k < N_tones; ++k) {
            const double ph = 2.0 * M_PI * f_k[k] * t_n + phi_k[k];
            ji += std::cos(ph);
            jq += std::sin(ph);
        }
        ji *= A_per_tone;
        jq *= A_per_tone;
        I[n] = sat16_d(static_cast<double>(I[n]) + ji);
        Q[n] = sat16_d(static_cast<double>(Q[n]) + jq);
    }
}
// ── §10 Swept (linear chirp, sawtooth) ────────────────────────
void Add_Swept(int16_t *I, int16_t *Q, int N, double jsr_db, double P_s,
               double f_start_Hz, double f_end_Hz, double rate_Hz_per_s,
               double f_chip_Hz, std::mt19937 &rng) noexcept {
    if (I == nullptr || Q == nullptr || N <= 0 || P_s <= 0.0)
        return;
    if (f_end_Hz <= f_start_Hz || rate_Hz_per_s <= 0.0)
        return;
    const double P_j = P_s * db_to_linear(jsr_db);
    const double A = std::sqrt(P_j);
    std::uniform_real_distribution<double> phi_dist(0.0, 2.0 * M_PI);
    const double phi_0 = phi_dist(rng);
    const double BW = f_end_Hz - f_start_Hz;
    // §10.3: f_current = f_start + ((rate·t) mod BW) — sawtooth
    // phase 적분: ∫2π·f(t)dt
    // sawtooth 구간 내부: f(t) = f_start + rate·τ (τ = t mod (BW/rate))
    // phase(t) = 2π·[f_start·τ + 0.5·rate·τ²] + sawtooth 누적
    double phase_acc = 0.0;
    double prev_tau = 0.0;
    const double period_s = BW / rate_Hz_per_s;
    for (int n = 0; n < N; ++n) {
        const double t = static_cast<double>(n) / f_chip_Hz;
        const double tau = std::fmod(t, period_s);
        if (tau < prev_tau) {
            // wrap: sawtooth 복귀, 누적 유지 (phase 연속성은 §10.3 sawtooth
            // 정의상 허용 — 순간 jump) 표준: smooth turnaround 금지. 즉 phase
            // 점프 허용. 실측 구현은 매 샘플 f(τ) 기반 재계산이 안정적.
        }
        prev_tau = tau;
        const double ph =
            2.0 * M_PI * (f_start_Hz * tau + 0.5 * rate_Hz_per_s * tau * tau) +
            phi_0;
        const double ji = A * std::cos(ph);
        const double jq = A * std::sin(ph);
        I[n] = sat16_d(static_cast<double>(I[n]) + ji);
        Q[n] = sat16_d(static_cast<double>(Q[n]) + jq);
        (void)phase_acc;
    }
}
// ── Partial Barrage ───────────────────────────────────────────
void Add_Partial_Barrage(int16_t *I, int16_t *Q, int nsym, int chips_per_sym,
                         double ratio_percent, double jsr_db, double P_s,
                         std::mt19937 &rng) noexcept {
    if (I == nullptr || Q == nullptr || nsym <= 0 || chips_per_sym <= 0 ||
        P_s <= 0.0)
        return;
    if (ratio_percent <= 0.0 || ratio_percent > 100.0)
        return;
    // 재밍 심볼 수 (round)
    const int n_jam = static_cast<int>(
        std::lround(static_cast<double>(nsym) * ratio_percent / 100.0));
    if (n_jam <= 0)
        return;
    // Fisher-Yates 부분 셔플 — 매 trial 독립 선택
    std::vector<int> sym_idx(nsym);
    for (int s = 0; s < nsym; ++s)
        sym_idx[s] = s;
    for (int s = 0; s < n_jam; ++s) {
        std::uniform_int_distribution<int> ud(s, nsym - 1);
        const int pick = ud(rng);
        std::swap(sym_idx[s], sym_idx[pick]);
    }
    // 선택된 n_jam 심볼에만 Barrage 주입
    const double P_j = P_s * db_to_linear(jsr_db);
    const double sigma = std::sqrt(P_j / 2.0);
    std::normal_distribution<double> nd(0.0, sigma);
    for (int k = 0; k < n_jam; ++k) {
        const int s = sym_idx[k];
        const int base = s * chips_per_sym;
        for (int c = 0; c < chips_per_sym; ++c) {
            const int n = base + c;
            const double ji = nd(rng);
            const double jq = nd(rng);
            I[n] = sat16_d(static_cast<double>(I[n]) + ji);
            Q[n] = sat16_d(static_cast<double>(Q[n]) + jq);
        }
    }
}
// ── §13 Saturation ────────────────────────────────────────────
uint32_t Saturate_Clip(int16_t *I, int16_t *Q, int N) noexcept {
    // 본 구현은 이미 sat16_d 로 각 add 마다 clip 적용.
    // 이 함수는 별도 외부 누적 입력 경로에서 사용. 현재 add 계열은 in-place
    // clip. 여기서는 count 만 반환하도록 호출부에서 관리. 단독 호출 시 전체
    // clip 체크.
    if (I == nullptr || Q == nullptr || N <= 0)
        return 0u;
    uint32_t cnt = 0u;
    for (int n = 0; n < N; ++n) {
        if (I[n] == I16_MAX || I[n] == I16_MIN || Q[n] == I16_MAX ||
            Q[n] == I16_MIN) {
            ++cnt;
        }
    }
    return cnt;
}
// ── 이름/단위 테이블 ──────────────────────────────────────────
const char *Channel_Name(ChannelType ch) noexcept {
    switch (ch) {
    case ChannelType::Clean:
        return "Clean";
    case ChannelType::AWGN:
        return "AWGN";
    case ChannelType::Barrage:
        return "Barrage";
    case ChannelType::CW:
        return "CW";
    case ChannelType::Pulse:
        return "Pulse";
    case ChannelType::MultiTone:
        return "Multi";
    case ChannelType::Swept:
        return "Swept";
    case ChannelType::Partial_Barrage:
        return "Partial_Barrage";
    }
    return "?";
}
const char *Channel_Unit(ChannelType ch) noexcept {
    switch (ch) {
    case ChannelType::Clean:
        return "none";
    case ChannelType::AWGN:
        return "dB_SNR";
    case ChannelType::Pulse:
        return "dB_PeakJSR";
    case ChannelType::Partial_Barrage:
        return "percent";
    default:
        return "dB_JSR";
    }
}
} // namespace HTS_Jammer_STD

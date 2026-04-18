// =====================================================================
// CFO_TEST.cpp — HTS B-CDMA CFO 알고리즘 정확도 검증 (FM 준수)
// =====================================================================
// 원칙:
//   1. threshold amp × 38 고정 (변경 금지)
//   2. 알고리즘이 threshold 를 넘는지만 평가
//   3. 실환경 위협 시뮬 (multipath, jammer, impulse)
//   4. False Positive 엄격 기준 (10000 trials)
//   5. 알고리즘 자체 정확도 (chip error, phase error) 검증
//
// 목적:
//   - 각 Apply 방식의 실제 정확도 측정 (threshold 무관)
//   - 실환경 위협 하에서 견고성 평가
//   - T6 점수 맞추기 아니라 진짜 성능 판단
// =====================================================================
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// =====================================================================
// 상수 (HTS 코드와 동일)
// =====================================================================
static constexpr int32_t kQ14One = 16384;
static constexpr int32_t kQ14Sq = kQ14One * kQ14One;
// HTS threshold 고정값 — 건드리지 않음
static constexpr int32_t K_THRESHOLD_FACTOR = 38; // amp × 38
// Walsh row 63 (HTS_V400_Dispatcher.cpp 동일)
static constexpr int8_t k_w63[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1};
// =====================================================================
// walsh63_dot_ (HTS 동일)
// =====================================================================
static void walsh63_dot(const int16_t *chip_I, const int16_t *chip_Q,
                        int32_t &dot_I, int32_t &dot_Q) noexcept {
    int32_t dI = 0, dQ = 0;
    for (int j = 0; j < 64; ++j) {
        const int32_t sI = static_cast<int32_t>(chip_I[j]);
        const int32_t sQ = static_cast<int32_t>(chip_Q[j]);
        if (k_w63[j] > 0) {
            dI += sI;
            dQ += sQ;
        } else {
            dI -= sI;
            dQ -= sQ;
        }
    }
    dot_I = dI;
    dot_Q = dQ;
}
// =====================================================================
// integer_sqrt (이진 탐색, 나눗셈 0회)
// =====================================================================
static int32_t integer_sqrt_q14(int64_t x) noexcept {
    if (x <= 0)
        return 0;
    const int64_t q14_sq = static_cast<int64_t>(kQ14One) * kQ14One;
    if (x >= q14_sq)
        return kQ14One;
    int32_t lo = 0, hi = kQ14One;
    while (lo < hi) {
        const int32_t mid = (lo + hi + 1) >> 1;
        const int64_t mid_sq = static_cast<int64_t>(mid) * mid;
        if (mid_sq <= x)
            lo = mid;
        else
            hi = mid - 1;
    }
    return lo;
}
static int32_t integer_sqrt_mag(int64_t x) noexcept {
    if (x <= 0)
        return 0;
    constexpr int32_t MAX_MAG = 23171;
    int32_t lo = 0, hi = MAX_MAG;
    while (lo < hi) {
        const int32_t mid = (lo + hi + 1) >> 1;
        const int64_t mid_sq = static_cast<int64_t>(mid) * mid;
        if (mid_sq <= x)
            lo = mid;
        else
            hi = mid - 1;
    }
    return lo;
}
// =====================================================================
// CFO Compensator (3가지 변형)
// =====================================================================
enum class CfoMode {
    BUGGY,     // 현재 (소각도 + 포화) 베이스라인
    V3_RENORM, // 복소 회전 + 128 chip 재정규화
    CMPLX_ONLY // 복소 회전만 (재정규화 없음)
};
struct CfoCompensator {
    int32_t cos_acc;
    int32_t sin_acc;
    int32_t cos_per_chip;
    int32_t sin_per_chip;
    int32_t chip_counter;
    bool active;
    CfoMode mode;
    void init(CfoMode m) {
        cos_acc = kQ14One;
        sin_acc = 0;
        cos_per_chip = kQ14One;
        sin_per_chip = 0;
        chip_counter = 0;
        active = false;
        mode = m;
    }
    void estimate(int32_t d0I, int32_t d0Q, int32_t d1I, int32_t d1Q,
                  int block_chips) {
        const int64_t cos_delta = (int64_t)d0I * d1I + (int64_t)d0Q * d1Q;
        const int64_t sin_delta = (int64_t)d0Q * d1I - (int64_t)d0I * d1Q;
        const int64_t ac = (cos_delta < 0) ? -cos_delta : cos_delta;
        const int64_t as = (sin_delta < 0) ? -sin_delta : sin_delta;
        const int64_t mag_approx =
            (ac > as) ? (ac + (as >> 1)) : (as + (ac >> 1));
        if (mag_approx < 1000) {
            active = false;
            return;
        }
        const int64_t k14 = kQ14One;
        const int32_t sin_block = (int32_t)((sin_delta * k14) / mag_approx);
        if (block_chips == 64)
            sin_per_chip = sin_block >> 6;
        else if (block_chips == 16)
            sin_per_chip = sin_block >> 4;
        else
            sin_per_chip = sin_block / block_chips;
        if (mode == CfoMode::BUGGY) {
            cos_per_chip = kQ14One;
        } else {
            const int64_t sin_sq = (int64_t)sin_per_chip * sin_per_chip;
            const int64_t q14_sq = k14 * k14;
            cos_per_chip = integer_sqrt_q14(q14_sq - sin_sq);
        }
        cos_acc = kQ14One;
        sin_acc = 0;
        chip_counter = 0;
        active = true;
    }
    void apply(int16_t &chipI, int16_t &chipQ) {
        if (!active)
            return;
        const int32_t ci = chipI, cq = chipQ;
        int32_t ri = (ci * cos_acc + cq * sin_acc) >> 14;
        int32_t rq = (cq * cos_acc - ci * sin_acc) >> 14;
        if (ri > 32767)
            ri = 32767;
        if (ri < -32768)
            ri = -32768;
        if (rq > 32767)
            rq = 32767;
        if (rq < -32768)
            rq = -32768;
        chipI = (int16_t)ri;
        chipQ = (int16_t)rq;
        if (mode == CfoMode::BUGGY) {
            sin_acc += sin_per_chip;
            if (sin_acc > kQ14One)
                sin_acc = kQ14One;
            if (sin_acc < -kQ14One)
                sin_acc = -kQ14One;
        } else {
            const int32_t next_cos =
                (int32_t)(((int64_t)cos_acc * cos_per_chip -
                           (int64_t)sin_acc * sin_per_chip) >>
                          14);
            const int32_t next_sin =
                (int32_t)(((int64_t)cos_acc * sin_per_chip +
                           (int64_t)sin_acc * cos_per_chip) >>
                          14);
            cos_acc = next_cos;
            sin_acc = next_sin;
            if (mode == CfoMode::V3_RENORM) {
                ++chip_counter;
                if ((chip_counter & 0x7F) == 0) {
                    const int64_t mag_sq =
                        (int64_t)cos_acc * cos_acc + (int64_t)sin_acc * sin_acc;
                    const int32_t cur_mag = integer_sqrt_mag(mag_sq);
                    if (cur_mag > 0) {
                        const int32_t scale = kQ14Sq / cur_mag;
                        cos_acc = (int32_t)(((int64_t)cos_acc * scale) >> 14);
                        sin_acc = (int32_t)(((int64_t)sin_acc * scale) >> 14);
                    }
                }
            }
        }
    }
};
// =====================================================================
// phase0_scan — HTS 동일, threshold amp×38 고정 (절대 변경 금지)
// =====================================================================
struct ScanResult {
    bool pass;
    bool off_ok, r_avg_ok, e63_ok, sep_ok;
    int32_t best_off;
    int32_t best_e63, second_e63, e63_min;
};
static ScanResult phase0_scan(const int16_t *buf_I, const int16_t *buf_Q,
                              int16_t tx_amp) {
    ScanResult r{};
    int32_t best_e63 = 0, second_e63 = 0;
    int32_t best_off = -1;
    int64_t sum_all = 0;
    for (int off = 0; off < 64; ++off) {
        int64_t e_nc = 0;
        for (int blk = 0; blk < 2; ++blk) {
            const int base = off + (blk << 6);
            int32_t dI = 0, dQ = 0;
            walsh63_dot(&buf_I[base], &buf_Q[base], dI, dQ);
            e_nc += (int64_t)dI * dI + (int64_t)dQ * dQ;
        }
        const int32_t accum = (int32_t)(e_nc >> 16);
        sum_all += accum;
        if (accum > best_e63) {
            second_e63 = best_e63;
            best_e63 = accum;
            best_off = off;
        } else if (accum > second_e63)
            second_e63 = accum;
    }
    const int64_t sum_others = sum_all - best_e63;
    const int64_t best_x63 = ((int64_t)best_e63 << 6) - best_e63;
    const int64_t so_x5 = (sum_others << 2) + sum_others;
    const bool r_avg_ok = (sum_others <= 0) || (best_x63 >= so_x5);
    const bool r_avg_high =
        (sum_others <= 0) || (best_x63 >= (sum_others << 3));
    const int64_t best_x4 = (int64_t)best_e63 << 2;
    const int64_t sec_x5 = ((int64_t)second_e63 << 2) + second_e63;
    const bool sep_ok =
        (!r_avg_high) || (second_e63 == 0) || (best_x4 > sec_x5);
    const int32_t avg_others =
        (sum_others > 0) ? (int32_t)((sum_others * 1040LL) >> 16) : 0;
    const int32_t adaptive_min =
        (avg_others > 0) ? (int32_t)(((int64_t)avg_others << 2) + avg_others)
                         : 5000;
    // FIXED: amp × 38 (절대 변경 금지)
    const int32_t amp32 = tx_amp;
    const int32_t k_E63_ALIGN_MIN = (amp32 << 5) + (amp32 << 2) + (amp32 << 1);
    const int32_t e63_min =
        (adaptive_min > k_E63_ALIGN_MIN) ? adaptive_min : k_E63_ALIGN_MIN;
    r.best_e63 = best_e63;
    r.second_e63 = second_e63;
    r.best_off = best_off;
    r.e63_min = e63_min;
    r.off_ok = (best_off >= 0);
    r.r_avg_ok = r_avg_ok;
    r.e63_ok = (best_e63 >= e63_min);
    r.sep_ok = sep_ok;
    r.pass = r.off_ok && r.r_avg_ok && r.e63_ok && r.sep_ok;
    return r;
}
// =====================================================================
// 채널 모델 — 실환경 위협 포함
// =====================================================================
struct ChannelParams {
    double cfo_hz = 0;
    double snr_db = 20;
    double chip_rate = 1e6;
    bool multipath_enabled = false;
    double path_delays[3] = {0.0, 0.5, 2.0};
    double path_gains[3] = {1.0, 0.3, 0.15};
    bool jammer_enabled = false;
    int jammer_type = 0; // 0=CW, 1=pulse, 2=barrage
    double jammer_power_db = 0;
    bool impulse_enabled = false;
    double impulse_prob = 0.01;
    double impulse_magnitude = 5.0;
    bool preamble = true;
};
static void generate_rx_with_channel(const ChannelParams &ch, int16_t tx_amp,
                                     int preamble_offset, uint32_t seed,
                                     int16_t *buf_I, int16_t *buf_Q) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> gauss(0.0, 1.0);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    double rI[192] = {}, rQ[192] = {};
    // 1. AWGN
    const double sig_pwr = (double)tx_amp * tx_amp;
    const double noise_pwr = sig_pwr / std::pow(10.0, ch.snr_db / 10.0);
    const double noise_std = std::sqrt(noise_pwr / 2.0);
    for (int k = 0; k < 192; ++k) {
        rI[k] = gauss(rng) * noise_std;
        rQ[k] = gauss(rng) * noise_std;
    }
    // 2. 프리앰블 + multipath
    if (ch.preamble) {
        const double chip_dt = 1.0 / ch.chip_rate;
        const double chip_phase = 2.0 * M_PI * ch.cfo_hz * chip_dt;
        const int n_paths = ch.multipath_enabled ? 3 : 1;
        for (int p = 0; p < n_paths; ++p) {
            const double delay = ch.multipath_enabled ? ch.path_delays[p] : 0.0;
            const double gain = ch.multipath_enabled ? ch.path_gains[p] : 1.0;
            const int di = (int)std::floor(delay);
            const double df = delay - di;
            for (int blk = 0; blk < 2; ++blk) {
                const int base = preamble_offset + blk * 64;
                for (int k = 0; k < 64; ++k) {
                    const int idx = base + k + di;
                    if (idx >= 192 || idx < 0)
                        continue;
                    const double phi = chip_phase * (blk * 64 + k);
                    const double sigI =
                        k_w63[k] * std::cos(phi) * tx_amp * gain;
                    const double sigQ =
                        k_w63[k] * std::sin(phi) * tx_amp * gain;
                    rI[idx] += sigI * (1 - df);
                    rQ[idx] += sigQ * (1 - df);
                    if (idx + 1 < 192) {
                        rI[idx + 1] += sigI * df;
                        rQ[idx + 1] += sigQ * df;
                    }
                }
            }
        }
    }
    // 3. Jammer
    if (ch.jammer_enabled) {
        const double j_amp = tx_amp * std::pow(10.0, ch.jammer_power_db / 20.0);
        const double chip_dt = 1.0 / ch.chip_rate;
        if (ch.jammer_type == 0) {
            const double jf = 3000.0;
            for (int k = 0; k < 192; ++k) {
                rI[k] += j_amp * std::cos(2 * M_PI * jf * k * chip_dt);
                rQ[k] += j_amp * std::sin(2 * M_PI * jf * k * chip_dt);
            }
        } else if (ch.jammer_type == 1) {
            for (int k = 0; k < 192; ++k) {
                if (uni(rng) < 0.05) {
                    rI[k] += j_amp * gauss(rng) * 3;
                    rQ[k] += j_amp * gauss(rng) * 3;
                }
            }
        } else {
            for (int k = 0; k < 192; ++k) {
                rI[k] += j_amp * gauss(rng);
                rQ[k] += j_amp * gauss(rng);
            }
        }
    }
    // 4. Impulse
    if (ch.impulse_enabled) {
        for (int k = 0; k < 192; ++k) {
            if (uni(rng) < ch.impulse_prob) {
                rI[k] += gauss(rng) * tx_amp * ch.impulse_magnitude;
                rQ[k] += gauss(rng) * tx_amp * ch.impulse_magnitude;
            }
        }
    }
    // 5. int16 quantize
    for (int k = 0; k < 192; ++k) {
        const int32_t vi = (int32_t)std::round(rI[k]);
        const int32_t vq = (int32_t)std::round(rQ[k]);
        buf_I[k] = (int16_t)std::max(-32768, std::min(32767, vi));
        buf_Q[k] = (int16_t)std::max(-32768, std::min(32767, vq));
    }
}
// =====================================================================
// RX + 정확도
// =====================================================================
struct RxAccuracy {
    bool scan_passed;
    int32_t best_e63;
    int32_t e63_min;
    double e63_margin;
    double mag_drift_pct;
    double phase_error_deg_at_128;
};
static RxAccuracy rx_process(const int16_t *buf_I, const int16_t *buf_Q,
                             int16_t tx_amp, CfoMode mode, double true_cfo_hz) {
    RxAccuracy acc{};
    // 1. 프리앰블 탐색 (Apply 전)
    ScanResult sc1 = phase0_scan(buf_I, buf_Q, tx_amp);
    if (sc1.best_off < 0) {
        acc.scan_passed = false;
        return acc;
    }
    // 2. Estimate
    int32_t d0I, d0Q, d1I, d1Q;
    walsh63_dot(&buf_I[sc1.best_off], &buf_Q[sc1.best_off], d0I, d0Q);
    walsh63_dot(&buf_I[sc1.best_off + 64], &buf_Q[sc1.best_off + 64], d1I, d1Q);
    CfoCompensator cfo;
    cfo.init(mode);
    cfo.estimate(d0I, d0Q, d1I, d1Q, 64);
    // 3. Apply to full buffer
    int16_t bI[192], bQ[192];
    std::memcpy(bI, buf_I, sizeof(bI));
    std::memcpy(bQ, buf_Q, sizeof(bQ));
    for (int k = 0; k < 128; ++k)
        cfo.apply(bI[k], bQ[k]);
    // 4. Scan 후 (Apply 적용 후 성능)
    ScanResult sc2 = phase0_scan(bI, bQ, tx_amp);
    acc.scan_passed = sc2.pass;
    acc.best_e63 = sc2.best_e63;
    acc.e63_min = sc2.e63_min;
    acc.e63_margin = sc2.e63_min > 0 ? (double)sc2.best_e63 / sc2.e63_min : 0;
    // 5. 단위원 drift
    const double cos_f = (double)cfo.cos_acc / kQ14One;
    const double sin_f = (double)cfo.sin_acc / kQ14One;
    const double mag = std::sqrt(cos_f * cos_f + sin_f * sin_f);
    acc.mag_drift_pct = (mag - 1.0) * 100;
    // 6. phase error @ chip 128
    const double chip_dt = 1e-6;
    const double true_phase_128 =
        std::fmod(2 * M_PI * true_cfo_hz * chip_dt * 128, 2 * M_PI);
    double true_mod = true_phase_128;
    if (true_mod > M_PI)
        true_mod -= 2 * M_PI;
    const double est_phase = std::atan2(sin_f, cos_f);
    double diff = est_phase - true_mod;
    while (diff > M_PI)
        diff -= 2 * M_PI;
    while (diff < -M_PI)
        diff += 2 * M_PI;
    acc.phase_error_deg_at_128 = std::abs(diff) * 180 / M_PI;
    return acc;
}
// =====================================================================
// 집계
// =====================================================================
struct Stats {
    int n;
    int pass;
    double mean_margin;
    double mean_phase_err;
    double mean_mag_drift;
};
static Stats run_scenario(const ChannelParams &ch, int16_t tx_amp, CfoMode mode,
                          int n_trials) {
    Stats s{};
    s.n = n_trials;
    double sm = 0, sp = 0, sd = 0;
    int cnt_acc = 0;
    for (int t = 0; t < n_trials; ++t) {
        int16_t bI[192], bQ[192];
        generate_rx_with_channel(ch, tx_amp, 32, (uint32_t)(t * 7919u + 42u),
                                 bI, bQ);
        RxAccuracy acc = rx_process(bI, bQ, tx_amp, mode, ch.cfo_hz);
        if (acc.scan_passed)
            ++s.pass;
        if (acc.e63_min > 0) {
            sm += acc.e63_margin;
            sp += acc.phase_error_deg_at_128;
            sd += acc.mag_drift_pct;
            ++cnt_acc;
        }
    }
    if (cnt_acc > 0) {
        s.mean_margin = sm / cnt_acc;
        s.mean_phase_err = sp / cnt_acc;
        s.mean_mag_drift = sd / cnt_acc;
    }
    return s;
}
static const char *mode_name(CfoMode m) {
    switch (m) {
    case CfoMode::BUGGY:
        return "BUGGY";
    case CfoMode::V3_RENORM:
        return "V3   ";
    case CfoMode::CMPLX_ONLY:
        return "CMPLX";
    }
    return "?";
}
// =====================================================================
// 메인
// =====================================================================
int main() {
    std::printf(
        "============================================================\n");
    std::printf("HTS CFO 알고리즘 정확도 검증 (FM 준수)\n");
    std::printf("  원칙 1: threshold amp×38 고정 (변경 금지)\n");
    std::printf("  원칙 2: 알고리즘이 threshold 를 넘어야 함\n");
    std::printf("  원칙 3: FP 엄격 기준 (10000 trials)\n");
    std::printf("  원칙 4: 실환경 위협 포함 (multipath, jammer, impulse)\n");
    std::printf(
        "============================================================\n\n");
    const int16_t TX_AMP = 1000;
    const int N_TRIALS = 100;
    const double SNR = 20.0;
    // =================================================================
    // Part 1: 알고리즘 정확도 (threshold 무관)
    // =================================================================
    std::printf("[Part 1] 알고리즘 자체 정확도 (Clean AWGN, SNR=20)\n");
    std::printf("  threshold 무관 지표: phase error, mag drift, pass rate\n\n");
    std::printf("  %-8s | %-6s | %-12s %-12s %-10s %-10s\n", "CFO(Hz)", "mode",
                "phase_err", "mag_drift", "margin", "pass");
    std::printf("  %s\n", std::string(80, '-').c_str());
    for (double cfo : {0.0, 1000.0, 2000.0, 5000.0, 10000.0, 20000.0}) {
        ChannelParams ch{};
        ch.cfo_hz = cfo;
        ch.snr_db = SNR;
        for (CfoMode m : {CfoMode::BUGGY, CfoMode::V3_RENORM}) {
            Stats s = run_scenario(ch, TX_AMP, m, N_TRIALS);
            std::printf("  %-8.0f | %-6s | %10.2f° %+10.3f%% %9.2fx %3d/%d\n",
                        cfo, mode_name(m), s.mean_phase_err, s.mean_mag_drift,
                        s.mean_margin, s.pass, s.n);
        }
        std::printf("\n");
    }
    // =================================================================
    // Part 2: False Positive 엄격 검증
    // =================================================================
    std::printf(
        "============================================================\n");
    std::printf("[Part 2] False Positive 엄격 검증 (threshold 고정)\n");
    std::printf(
        "============================================================\n\n");
    std::printf("  10000 trials × 여러 환경 (실환경 위협 포함)\n");
    std::printf("  FP > 0 이면 threshold 검증 실패\n\n");
    std::printf("  %-25s | %-6s %-6s %-6s %-6s %-6s\n", "환경", "SNR=5",
                "SNR=10", "SNR=15", "SNR=20", "SNR=25");
    std::printf("  %s\n", std::string(80, '-').c_str());
    const int FP_TRIALS = 10000;
    struct FpScen {
        const char *name;
        bool mp, jm, imp;
        int jt;
        double jd;
    };
    FpScen scens[] = {
        {"AWGN only", false, false, false, 0, 0},
        {"Multipath", true, false, false, 0, 0},
        {"CW jammer -10dB", false, true, false, 0, -10},
        {"Pulse jammer 0dB", false, true, false, 1, 0},
        {"Barrage jammer -5dB", false, true, false, 2, -5},
        {"Impulse noise", false, false, true, 0, 0},
        {"MP + CW -10dB", true, true, false, 0, -10},
    };
    for (const auto &sc : scens) {
        std::printf("  %-25s | ", sc.name);
        for (double snr : {5.0, 10.0, 15.0, 20.0, 25.0}) {
            ChannelParams ch{};
            ch.preamble = false; // 프리앰블 없음!
            ch.snr_db = snr;
            ch.multipath_enabled = sc.mp;
            ch.jammer_enabled = sc.jm;
            ch.jammer_type = sc.jt;
            ch.jammer_power_db = sc.jd;
            ch.impulse_enabled = sc.imp;
            int fp = 0;
            for (int t = 0; t < FP_TRIALS; ++t) {
                int16_t bI[192], bQ[192];
                generate_rx_with_channel(ch, TX_AMP, 32,
                                         (uint32_t)(t * 7919u + 999u), bI, bQ);
                ScanResult s = phase0_scan(bI, bQ, TX_AMP);
                if (s.pass)
                    ++fp;
            }
            std::printf("%4d   ", fp);
        }
        std::printf("\n");
    }
    // =================================================================
    // Part 3: 실환경 위협 하 성능 (프리앰블 있음, threshold 고정)
    // =================================================================
    std::printf(
        "\n============================================================\n");
    std::printf(
        "[Part 3] 실환경 위협 하 알고리즘 성능 (threshold amp×38 고정)\n");
    std::printf(
        "============================================================\n\n");
    struct Threat {
        const char *name;
        bool mp, jm, imp;
        int jt;
        double jd;
        double snr;
    };
    Threat threats[] = {
        {"Clean SNR=20", false, false, false, 0, 0, 20},
        {"Clean SNR=15", false, false, false, 0, 0, 15},
        {"Clean SNR=10", false, false, false, 0, 0, 10},
        {"Multipath SNR=20", true, false, false, 0, 0, 20},
        {"CW jammer -10dB", false, true, false, 0, -10, 20},
        {"Pulse jammer", false, true, false, 1, 0, 20},
        {"Impulse noise", false, false, true, 0, 0, 20},
        {"MP + CW worst", true, true, false, 0, -10, 15},
    };
    std::printf("  %-20s | %-10s | %-8s %-8s | %-8s %-8s\n", "Threat",
                "CFO(Hz)", "BUGGY", "margin", "V3", "margin");
    std::printf("  %s\n", std::string(80, '-').c_str());
    for (const auto &th : threats) {
        for (double cfo : {0.0, 1000.0, 5000.0, 10000.0}) {
            ChannelParams ch{};
            ch.cfo_hz = cfo;
            ch.snr_db = th.snr;
            ch.multipath_enabled = th.mp;
            ch.jammer_enabled = th.jm;
            ch.jammer_type = th.jt;
            ch.jammer_power_db = th.jd;
            ch.impulse_enabled = th.imp;
            Stats sB = run_scenario(ch, TX_AMP, CfoMode::BUGGY, N_TRIALS);
            Stats sV = run_scenario(ch, TX_AMP, CfoMode::V3_RENORM, N_TRIALS);
            std::printf(
                "  %-20s | %-10.0f | %3d/%d   %5.2fx | %3d/%d   %5.2fx\n",
                th.name, cfo, sB.pass, sB.n, sB.mean_margin, sV.pass, sV.n,
                sV.mean_margin);
        }
        std::printf("\n");
    }
    // =================================================================
    // Part 4: 장기 chip 누적 정확도 (drift)
    // =================================================================
    std::printf(
        "============================================================\n");
    std::printf("[Part 4] 장기 chip 누적 drift (알고리즘 본질 비교)\n");
    std::printf(
        "============================================================\n\n");
    for (double cfo : {1000.0, 5000.0}) {
        std::printf("  [CFO %.0f Hz]\n", cfo);
        std::printf("  %-10s | %-20s | %-20s\n", "chip", "BUGGY phase/mag",
                    "V3 phase/mag");
        for (int n : {64, 200, 500, 1000, 2000, 5000, 10000}) {
            const double chip_dt = 1e-6;
            const double chip_phase = 2 * M_PI * cfo * chip_dt;
            const int32_t sin_per = (int32_t)(std::sin(chip_phase) * kQ14One);
            // BUGGY
            CfoCompensator b;
            b.init(CfoMode::BUGGY);
            b.sin_per_chip = sin_per;
            b.cos_per_chip = kQ14One;
            b.cos_acc = kQ14One;
            b.sin_acc = 0;
            b.active = true;
            // V3
            CfoCompensator v;
            v.init(CfoMode::V3_RENORM);
            v.sin_per_chip = sin_per;
            int64_t sq = (int64_t)sin_per * sin_per;
            v.cos_per_chip = integer_sqrt_q14(kQ14Sq - sq);
            v.cos_acc = kQ14One;
            v.sin_acc = 0;
            v.chip_counter = 0;
            v.active = true;
            int16_t dI = 100, dQ = 0;
            for (int k = 0; k < n; ++k) {
                int16_t bI2 = 100, bQ2 = 0;
                b.apply(bI2, bQ2);
                int16_t vI = 100, vQ = 0;
                v.apply(vI, vQ);
            }
            auto compute = [&](int32_t cos_a, int32_t sin_a) {
                const double cf = (double)cos_a / kQ14One;
                const double sf = (double)sin_a / kQ14One;
                const double mag = std::sqrt(cf * cf + sf * sf);
                const double est = std::atan2(sf, cf);
                const double tru = std::fmod(chip_phase * n, 2 * M_PI);
                double tm = tru;
                if (tm > M_PI)
                    tm -= 2 * M_PI;
                double d = est - tm;
                while (d > M_PI)
                    d -= 2 * M_PI;
                while (d < -M_PI)
                    d += 2 * M_PI;
                return std::make_pair(d * 180 / M_PI, (mag - 1) * 100);
            };
            auto bstat = compute(b.cos_acc, b.sin_acc);
            auto vstat = compute(v.cos_acc, v.sin_acc);
            std::printf("  %-10d | %+7.2f° / %+6.2f%%  | %+7.2f° / %+6.3f%%\n",
                        n, bstat.first, bstat.second, vstat.first,
                        vstat.second);
        }
        std::printf("\n");
    }
    std::printf(
        "============================================================\n");
    std::printf("[결론]\n");
    std::printf(
        "============================================================\n");
    std::printf("  threshold amp×38 고정 하에서 각 알고리즘의 순수 성능.\n");
    std::printf("  알고리즘이 threshold 를 못 넘으면 알고리즘 부족.\n");
    std::printf("  threshold 낮추는 것 = 안전 마진 붕괴 = 절대 금지.\n");
    std::printf("\n");
    std::printf("  판단 기준:\n");
    std::printf("  - 같은 threshold 하에서 pass rate 더 높은 것이 우수\n");
    std::printf("  - margin 이 더 큰 것이 안전 마진 확보\n");
    std::printf("  - phase error 작은 것이 정확\n");
    std::printf("  - mag drift 0 에 가까운 것이 안정\n");
    return 0;
}

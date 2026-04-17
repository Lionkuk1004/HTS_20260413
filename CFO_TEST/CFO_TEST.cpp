// =====================================================================
// CFO_TEST.cpp — HTS B-CDMA CFO Compensator 진단 프로그램
// =====================================================================
// 목적: S5 1000Hz 회귀의 실제 실패 지점 식별
// 대상: HTS_CFO_Compensator + phase0_scan_ 로직 완전 재현
// 빌드: VS x64 Release / C++17
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
// =====================================================================
// 상수
// =====================================================================
static constexpr int32_t kQ14One = 16384;
static constexpr int32_t kQ14Sq = kQ14One * kQ14One; // 2^28 = 268,435,456
// HTS_V400_Dispatcher.cpp 의 k_w63 (Walsh row 63) 복사
static constexpr int8_t k_w63[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1};
// =====================================================================
// 유틸: walsh63_dot_ (HTS_V400_Dispatcher.cpp 와 동일)
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
// integer_sqrt_q14 — 이진 탐색, 나눗셈 0회
// =====================================================================
static int32_t integer_sqrt_q14(int64_t x) noexcept {
    if (x <= 0)
        return 0;
    const int64_t q14_sq = static_cast<int64_t>(kQ14One) * kQ14One;
    if (x >= q14_sq)
        return kQ14One;
    int32_t lo = 0;
    int32_t hi = kQ14One;
    while (lo < hi) {
        const int32_t mid = (lo + hi + 1) >> 1;
        const int64_t mid_sq = static_cast<int64_t>(mid) * mid;
        if (mid_sq <= x) {
            lo = mid;
        } else {
            hi = mid - 1;
        }
    }
    return lo;
}
// mag 가 kQ14One 을 초과 가능한 경우용 (복소 회전 drift)
static int32_t integer_sqrt_mag(int64_t x) noexcept {
    if (x <= 0)
        return 0;
    // 범위: 0 ~ 2 × kQ14Sq (두 개의 Q14² 합)
    // 결과: 0 ~ sqrt(2) × kQ14One ≈ 23170
    constexpr int32_t MAX_MAG = 23171;
    int32_t lo = 0;
    int32_t hi = MAX_MAG;
    while (lo < hi) {
        const int32_t mid = (lo + hi + 1) >> 1;
        const int64_t mid_sq = static_cast<int64_t>(mid) * mid;
        if (mid_sq <= x) {
            lo = mid;
        } else {
            hi = mid - 1;
        }
    }
    return lo;
}
// =====================================================================
// CFO Compensator — 세 가지 버전
// =====================================================================
enum class CfoMode {
    BUGGY,       // 현재 베이스라인: sin_acc += + 포화
    V3_RENORM,   // v3: 복소 회전 + 128 chip 재정규화
    COMPLEX_ONLY // v2: 복소 회전만 (재정규화 없음)
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
        // HTS_CFO_Compensator.h 의 Estimate_From_Preamble 로직
        const int64_t cos_delta =
            static_cast<int64_t>(d0I) * d1I + static_cast<int64_t>(d0Q) * d1Q;
        const int64_t sin_delta =
            static_cast<int64_t>(d0Q) * d1I - static_cast<int64_t>(d0I) * d1Q;
        const int64_t ac = (cos_delta < 0) ? -cos_delta : cos_delta;
        const int64_t as = (sin_delta < 0) ? -sin_delta : sin_delta;
        const int64_t mag_approx =
            (ac > as) ? (ac + (as >> 1)) : (as + (ac >> 1));
        if (mag_approx < 1000) {
            active = false;
            return;
        }
        const int64_t k14 = static_cast<int64_t>(kQ14One);
        const int32_t sin_block =
            static_cast<int32_t>((sin_delta * k14) / mag_approx);
        if (block_chips == 64) {
            sin_per_chip = sin_block >> 6;
        } else if (block_chips == 16) {
            sin_per_chip = sin_block >> 4;
        } else {
            sin_per_chip = sin_block / block_chips;
        }
        if (mode == CfoMode::BUGGY) {
            cos_per_chip = kQ14One; // 소각도 근사
        } else {
            // 정확 계산: sin² + cos² = Q14²
            const int64_t sin_sq =
                static_cast<int64_t>(sin_per_chip) * sin_per_chip;
            const int64_t q14_sq = k14 * k14;
            const int64_t cos_sq = q14_sq - sin_sq;
            cos_per_chip = integer_sqrt_q14(cos_sq);
        }
        cos_acc = kQ14One;
        sin_acc = 0;
        chip_counter = 0;
        active = true;
    }
    void apply(int16_t &chipI, int16_t &chipQ) {
        if (!active)
            return;
        const int32_t ci = static_cast<int32_t>(chipI);
        const int32_t cq = static_cast<int32_t>(chipQ);
        // chip 역회전 (공통)
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
        chipI = static_cast<int16_t>(ri);
        chipQ = static_cast<int16_t>(rq);
        // 누적 위상 갱신 (모드별)
        if (mode == CfoMode::BUGGY) {
            // 기존: 선형 누적 + 포화
            sin_acc += sin_per_chip;
            if (sin_acc > kQ14One)
                sin_acc = kQ14One;
            if (sin_acc < -kQ14One)
                sin_acc = -kQ14One;
            // cos_acc 변경 없음
        } else {
            // 복소 페이저 곱셈
            const int32_t next_cos = static_cast<int32_t>(
                (static_cast<int64_t>(cos_acc) * cos_per_chip -
                 static_cast<int64_t>(sin_acc) * sin_per_chip) >>
                14);
            const int32_t next_sin = static_cast<int32_t>(
                (static_cast<int64_t>(cos_acc) * sin_per_chip +
                 static_cast<int64_t>(sin_acc) * cos_per_chip) >>
                14);
            cos_acc = next_cos;
            sin_acc = next_sin;
            // v3: 128 chip 마다 재정규화
            if (mode == CfoMode::V3_RENORM) {
                ++chip_counter;
                if ((chip_counter & 0x7F) == 0) {
                    const int64_t mag_sq =
                        static_cast<int64_t>(cos_acc) * cos_acc +
                        static_cast<int64_t>(sin_acc) * sin_acc;
                    const int32_t cur_mag = integer_sqrt_mag(mag_sq);
                    if (cur_mag > 0) {
                        const int32_t scale = kQ14Sq / cur_mag; // int32 UDIV
                        cos_acc = static_cast<int32_t>(
                            (static_cast<int64_t>(cos_acc) * scale) >> 14);
                        sin_acc = static_cast<int32_t>(
                            (static_cast<int64_t>(sin_acc) * scale) >> 14);
                    }
                }
            }
        }
    }
};
// =====================================================================
// phase0_scan 결과 구조체
// =====================================================================
struct ScanResult {
    bool pass;
    bool off_ok;
    bool r_avg_ok;
    bool r_avg_high;
    bool e63_ok;
    bool sep_ok;
    int32_t best_off;
    int32_t best_e63;
    int32_t second_e63;
    int32_t e63_min;
    int32_t avg_others;
    int64_t sum_others;
    int32_t r_avg; // best / avg_others (정수)
    int32_t r_sep; // best / second
};
// phase0_scan_ 로직 완전 재현
static ScanResult phase0_scan(const int16_t *buf_I, const int16_t *buf_Q,
                              int16_t tx_amp, int32_t k_factor = 38) {
    ScanResult r{};
    int32_t best_e63 = 0, second_e63 = 0;
    int32_t best_off = -1;
    int64_t sum_all = 0;
    for (int off = 0; off < 64; ++off) {
        int64_t e_nc = 0;
        for (int blk = 0; blk < 2; ++blk) {
            const int base = off + (blk << 6);
            int32_t dot_I = 0, dot_Q = 0;
            walsh63_dot(&buf_I[base], &buf_Q[base], dot_I, dot_Q);
            e_nc += static_cast<int64_t>(dot_I) * dot_I +
                    static_cast<int64_t>(dot_Q) * dot_Q;
        }
        const int32_t accum = static_cast<int32_t>(e_nc >> 16);
        sum_all += static_cast<int64_t>(accum);
        if (accum > best_e63) {
            second_e63 = best_e63;
            best_e63 = accum;
            best_off = off;
        } else if (accum > second_e63) {
            second_e63 = accum;
        }
    }
    const int64_t sum_others = sum_all - static_cast<int64_t>(best_e63);
    // r_avg >= 5
    const int64_t best_x63 =
        (static_cast<int64_t>(best_e63) << 6) - static_cast<int64_t>(best_e63);
    const int64_t so_x5 = (sum_others << 2) + sum_others;
    const bool r_avg_ok = (sum_others <= 0) || (best_x63 >= so_x5);
    // r_avg >= 8
    const bool r_avg_high =
        (sum_others <= 0) || (best_x63 >= (sum_others << 3));
    // sep_ok
    const int64_t best_x4 = static_cast<int64_t>(best_e63) << 2;
    const int64_t sec_x5 = (static_cast<int64_t>(second_e63) << 2) +
                           static_cast<int64_t>(second_e63);
    const bool sep_ok =
        (!r_avg_high) || (second_e63 == 0) || (best_x4 > sec_x5);
    // e63_min
    const int32_t avg_others =
        (sum_others > 0) ? static_cast<int32_t>((sum_others * 1040LL) >> 16)
                         : 0;
    const int32_t adaptive_min =
        (avg_others > 0)
            ? static_cast<int32_t>((static_cast<int64_t>(avg_others) << 2) +
                                   avg_others)
            : 5000;
    const int32_t amp32 = static_cast<int32_t>(tx_amp);
    // k_factor × amp (38 기본)
    const int32_t k_E63_ALIGN_MIN = amp32 * k_factor;
    const int32_t e63_min =
        (adaptive_min > k_E63_ALIGN_MIN) ? adaptive_min : k_E63_ALIGN_MIN;
    r.best_e63 = best_e63;
    r.second_e63 = second_e63;
    r.best_off = best_off;
    r.e63_min = e63_min;
    r.avg_others = avg_others;
    r.sum_others = sum_others;
    r.off_ok = (best_off >= 0);
    r.r_avg_ok = r_avg_ok;
    r.r_avg_high = r_avg_high;
    r.e63_ok = (best_e63 >= e63_min);
    r.sep_ok = sep_ok;
    r.pass = r.off_ok && r.r_avg_ok && r.e63_ok && r.sep_ok;
    r.r_avg = (avg_others > 0) ? (best_e63 / avg_others) : 999;
    r.r_sep = (second_e63 > 0) ? (best_e63 / second_e63) : 999;
    return r;
}
// =====================================================================
// TX: 프리앰블 + CFO 시뮬레이션
// =====================================================================
struct TxParams {
    double cfo_hz;       // CFO
    double chip_rate;    // 1 Mcps 기본
    int16_t amp;         // TX amplitude
    double snr_db;       // SNR
    int preamble_offset; // 128 chip 버퍼 내 위치 (보통 32)
    uint32_t seed;
};
static void generate_rx_buffer(const TxParams &tx, int16_t *buf_I,
                               int16_t *buf_Q) {
    std::mt19937 rng(tx.seed);
    std::normal_distribution<double> gauss(0.0, 1.0);
    // 128 chip 버퍼 = preamble_offset 전후 noise, 프리앰블 구간 (64×2=128 chip)
    // 단, phase0_scan 은 0~127 offset 스캔하므로 buf 는 128 chip + 여유 필요
    // 원 코드는 p0_buf128_I_[192] 사용 → 안전하게 192 chip 가정
    const double sig_pwr = static_cast<double>(tx.amp) * tx.amp;
    const double noise_pwr = sig_pwr / std::pow(10.0, tx.snr_db / 10.0);
    const double noise_std = std::sqrt(noise_pwr / 2.0);
    const double chip_dt = 1.0 / tx.chip_rate;
    const double chip_phase = 2.0 * 3.141592653589793 * tx.cfo_hz * chip_dt;
    // 전체 버퍼를 noise 로 채움
    for (int k = 0; k < 192; ++k) {
        const double noise_I = gauss(rng) * noise_std;
        const double noise_Q = gauss(rng) * noise_std;
        buf_I[k] = static_cast<int16_t>(std::round(noise_I));
        buf_Q[k] = static_cast<int16_t>(std::round(noise_Q));
    }
    // 프리앰블 2 블록 (Walsh row 63, ±amp)
    for (int blk = 0; blk < 2; ++blk) {
        const int base = tx.preamble_offset + blk * 64;
        for (int k = 0; k < 64; ++k) {
            const int idx = base + k;
            if (idx >= 192)
                continue;
            // chip 간 연속 위상
            const double phi = chip_phase * (blk * 64 + k);
            const double sig_I = k_w63[k] * std::cos(phi) * tx.amp;
            const double sig_Q = k_w63[k] * std::sin(phi) * tx.amp;
            const int32_t mixed_I = static_cast<int32_t>(buf_I[idx]) +
                                    static_cast<int32_t>(std::round(sig_I));
            const int32_t mixed_Q = static_cast<int32_t>(buf_Q[idx]) +
                                    static_cast<int32_t>(std::round(sig_Q));
            buf_I[idx] = static_cast<int16_t>(
                std::max(-32768, std::min(32767, mixed_I)));
            buf_Q[idx] = static_cast<int16_t>(
                std::max(-32768, std::min(32767, mixed_Q)));
        }
    }
}
// =====================================================================
// RX: Estimate → Apply → Scan
// =====================================================================
struct RxResult {
    ScanResult scan_before; // Apply 전 (Estimate 만 한 상태)
    ScanResult scan_after;  // Apply 후
    int32_t sin_per_chip;
    int32_t cos_per_chip;
    bool est_active;
    int32_t d0I, d0Q, d1I, d1Q;
};
static RxResult rx_process(const int16_t *buf_I_orig, const int16_t *buf_Q_orig,
                           int16_t tx_amp, CfoMode mode,
                           int32_t k_factor = 38) {
    RxResult r{};
    // 1) 첫 번째 스캔 (Apply 없이 best_off 찾기)
    r.scan_before = phase0_scan(buf_I_orig, buf_Q_orig, tx_amp, k_factor);
    // 2) best_off 기준으로 d0, d1 추출
    if (r.scan_before.best_off < 0) {
        r.est_active = false;
        return r;
    }
    int32_t d0I = 0, d0Q = 0, d1I = 0, d1Q = 0;
    walsh63_dot(&buf_I_orig[r.scan_before.best_off],
                &buf_Q_orig[r.scan_before.best_off], d0I, d0Q);
    walsh63_dot(&buf_I_orig[r.scan_before.best_off + 64],
                &buf_Q_orig[r.scan_before.best_off + 64], d1I, d1Q);
    r.d0I = d0I;
    r.d0Q = d0Q;
    r.d1I = d1I;
    r.d1Q = d1Q;
    // 3) CFO Estimate
    CfoCompensator cfo;
    cfo.init(mode);
    cfo.estimate(d0I, d0Q, d1I, d1Q, 64);
    r.sin_per_chip = cfo.sin_per_chip;
    r.cos_per_chip = cfo.cos_per_chip;
    r.est_active = cfo.active;
    if (!cfo.active) {
        r.scan_after = r.scan_before;
        return r;
    }
    // 4) Apply (128 chip 버퍼 전체에)
    int16_t buf_I[192], buf_Q[192];
    std::memcpy(buf_I, buf_I_orig, sizeof(buf_I));
    std::memcpy(buf_Q, buf_Q_orig, sizeof(buf_Q));
    for (int k = 0; k < 128; ++k) {
        cfo.apply(buf_I[k], buf_Q[k]);
    }
    // 5) 두 번째 스캔 (Apply 후)
    r.scan_after = phase0_scan(buf_I, buf_Q, tx_amp, k_factor);
    return r;
}
// =====================================================================
// Monte Carlo 실행 + 요약
// =====================================================================
struct Summary {
    int n_trials;
    int pass_count;
    int fail_off_ok;
    int fail_r_avg_ok;
    int fail_e63_ok;
    int fail_sep_ok;
    double mean_best_e63;
    double mean_second_e63;
    double mean_e63_min;
    double mean_r_sep;
};
static Summary run_monte_carlo(double cfo_hz, int16_t tx_amp, double snr_db,
                               int n_trials, CfoMode mode,
                               int32_t k_factor = 38, bool verbose = false) {
    Summary s{};
    s.n_trials = n_trials;
    double sum_best = 0, sum_second = 0, sum_emin = 0, sum_rsep = 0;
    for (int t = 0; t < n_trials; ++t) {
        TxParams tx;
        tx.cfo_hz = cfo_hz;
        tx.chip_rate = 1e6;
        tx.amp = tx_amp;
        tx.snr_db = snr_db;
        tx.preamble_offset = 32;
        tx.seed = static_cast<uint32_t>(t * 7919u + 42u);
        int16_t buf_I[192], buf_Q[192];
        generate_rx_buffer(tx, buf_I, buf_Q);
        RxResult r = rx_process(buf_I, buf_Q, tx_amp, mode, k_factor);
        const ScanResult &sc = r.scan_after;
        if (sc.pass) {
            ++s.pass_count;
        } else {
            if (!sc.off_ok)
                ++s.fail_off_ok;
            if (!sc.r_avg_ok)
                ++s.fail_r_avg_ok;
            if (!sc.e63_ok)
                ++s.fail_e63_ok;
            if (!sc.sep_ok)
                ++s.fail_sep_ok;
        }
        sum_best += sc.best_e63;
        sum_second += sc.second_e63;
        sum_emin += sc.e63_min;
        sum_rsep += sc.r_sep;
        if (verbose && t < 5) {
            std::printf(
                "  trial %2d: pass=%d | off_ok=%d r_avg=%d e63=%d sep=%d | "
                "best=%d second=%d emin=%d r_sep=%d | "
                "sin_per=%d cos_per=%d\n",
                t, sc.pass ? 1 : 0, sc.off_ok ? 1 : 0, sc.r_avg_ok ? 1 : 0,
                sc.e63_ok ? 1 : 0, sc.sep_ok ? 1 : 0, sc.best_e63,
                sc.second_e63, sc.e63_min, sc.r_sep, r.sin_per_chip,
                r.cos_per_chip);
        }
    }
    s.mean_best_e63 = sum_best / n_trials;
    s.mean_second_e63 = sum_second / n_trials;
    s.mean_e63_min = sum_emin / n_trials;
    s.mean_r_sep = sum_rsep / n_trials;
    return s;
}
// =====================================================================
// 출력 유틸
// =====================================================================
static const char *mode_name(CfoMode m) {
    switch (m) {
    case CfoMode::BUGGY:
        return "BUGGY  (baseline)";
    case CfoMode::V3_RENORM:
        return "V3     (complex+renorm)";
    case CfoMode::COMPLEX_ONLY:
        return "CMPLX  (complex only)";
    }
    return "?";
}
static void print_summary_row(const Summary &s, CfoMode mode) {
    std::printf("  %-22s: pass %2d/%d | fail: off=%d r_avg=%d e63=%d sep=%d | "
                "mean_best=%8.0f mean_sec=%8.0f mean_emin=%6.0f r_sep~%.0f\n",
                mode_name(mode), s.pass_count, s.n_trials, s.fail_off_ok,
                s.fail_r_avg_ok, s.fail_e63_ok, s.fail_sep_ok, s.mean_best_e63,
                s.mean_second_e63, s.mean_e63_min, s.mean_r_sep);
}
// =====================================================================
// 메인 — 시나리오별 테스트
// =====================================================================
int main(int argc, char *argv[]) {
    std::printf(
        "============================================================\n");
    std::printf("HTS CFO Compensator 진단 프로그램\n");
    std::printf(
        "============================================================\n\n");
    const int N_TRIALS = 30;
    const int16_t TX_AMP = 1000;
    const double SNR = 20.0;
    // -----------------------------------------------------------------
    // Part 1: 1000 Hz 집중 분석 (verbose)
    // -----------------------------------------------------------------
    std::printf("[Part 1] CFO 1000 Hz 집중 분석 (trial 5개 상세)\n");
    std::printf("  tx_amp=%d, snr=%.0f dB, trials=%d\n", TX_AMP, SNR, N_TRIALS);
    std::printf("  k_factor=38 (기존 threshold)\n\n");
    for (CfoMode m :
         {CfoMode::BUGGY, CfoMode::V3_RENORM, CfoMode::COMPLEX_ONLY}) {
        std::printf("[mode = %s]\n", mode_name(m));
        Summary s = run_monte_carlo(1000.0, TX_AMP, SNR, N_TRIALS, m, 38, true);
        print_summary_row(s, m);
        std::printf("\n");
    }
    // -----------------------------------------------------------------
    // Part 2: CFO Sweep (여러 CFO에서 BUGGY vs V3 비교)
    // -----------------------------------------------------------------
    std::printf(
        "============================================================\n");
    std::printf("[Part 2] CFO Sweep — BUGGY vs V3 비교\n");
    std::printf(
        "============================================================\n\n");
    const double cfo_list[] = {0,    100,   500,   1000,  2000,
                               5000, 10000, 25000, 50000, 100000};
    std::printf("  %-10s | %-22s | %-22s\n", "CFO(Hz)",
                mode_name(CfoMode::BUGGY), mode_name(CfoMode::V3_RENORM));
    std::printf("  %s\n", std::string(80, '-').c_str());
    for (double cfo : cfo_list) {
        std::printf("  %-10.0f | ", cfo);
        Summary sB =
            run_monte_carlo(cfo, TX_AMP, SNR, N_TRIALS, CfoMode::BUGGY, 38);
        std::printf("pass %2d/%d (sep=%d)      | ", sB.pass_count, sB.n_trials,
                    sB.fail_sep_ok);
        Summary sV =
            run_monte_carlo(cfo, TX_AMP, SNR, N_TRIALS, CfoMode::V3_RENORM, 38);
        std::printf("pass %2d/%d (sep=%d)\n", sV.pass_count, sV.n_trials,
                    sV.fail_sep_ok);
    }
    // -----------------------------------------------------------------
    // Part 3: threshold k_factor 변화 — BUGGY vs V3
    // -----------------------------------------------------------------
    std::printf(
        "\n============================================================\n");
    std::printf("[Part 3] k_factor 변화 — BUGGY vs V3 비교\n");
    std::printf(
        "============================================================\n\n");
    std::printf("  BUGGY 모드 (현재 베이스라인):\n");
    std::printf("  %-10s | %s\n", "CFO(Hz)",
                "k=38  k=34  k=32  k=30  k=28  k=26  k=22  k=18");
    std::printf("  %s\n", std::string(80, '-').c_str());
    for (double cfo :
         {0.0, 1000.0, 2000.0, 5000.0, 10000.0, 15000.0, 25000.0}) {
        std::printf("  %-10.0f | ", cfo);
        for (int k_factor : {38, 34, 32, 30, 28, 26, 22, 18}) {
            Summary s = run_monte_carlo(cfo, TX_AMP, SNR, N_TRIALS,
                                        CfoMode::BUGGY, k_factor);
            std::printf("%3d/%d ", s.pass_count, s.n_trials);
        }
        std::printf("\n");
    }
    std::printf("\n  V3 모드 (복소 회전 + 재정규화):\n");
    std::printf("  %-10s | %s\n", "CFO(Hz)",
                "k=38  k=34  k=32  k=30  k=28  k=26  k=22  k=18");
    std::printf("  %s\n", std::string(80, '-').c_str());
    for (double cfo :
         {0.0, 1000.0, 2000.0, 5000.0, 10000.0, 15000.0, 25000.0}) {
        std::printf("  %-10.0f | ", cfo);
        for (int k_factor : {38, 34, 32, 30, 28, 26, 22, 18}) {
            Summary s = run_monte_carlo(cfo, TX_AMP, SNR, N_TRIALS,
                                        CfoMode::V3_RENORM, k_factor);
            std::printf("%3d/%d ", s.pass_count, s.n_trials);
        }
        std::printf("\n");
    }
    // false positive 체크 (프리앰블 없이)
    std::printf("\n  False Positive 체크 (프리앰블 없음, noise 만):\n");
    std::printf("  %-10s | %s\n", "SNR(dB)",
                "k=38  k=34  k=32  k=30  k=28  k=26  k=22  k=18");
    std::printf("  %s\n", std::string(80, '-').c_str());
    for (double snr : {10.0, 15.0, 20.0, 25.0}) {
        std::printf("  %-10.0f | ", snr);
        for (int k_factor : {38, 34, 32, 30, 28, 26, 22, 18}) {
            int fp = 0;
            for (int t = 0; t < 50; ++t) {
                TxParams tx;
                tx.cfo_hz = 0;
                tx.chip_rate = 1e6;
                tx.amp = TX_AMP;
                tx.snr_db = snr;
                tx.preamble_offset = 0; // 프리앰블 없음
                tx.seed = static_cast<uint32_t>(t * 7919u + 999u);
                int16_t buf_I[192], buf_Q[192];
                // 프리앰블 없는 buffer (noise only)
                std::mt19937 rng(tx.seed);
                std::normal_distribution<double> gauss(0.0, 1.0);
                const double sig_pwr = static_cast<double>(tx.amp) * tx.amp;
                const double noise_pwr = sig_pwr / std::pow(10.0, snr / 10.0);
                const double noise_std = std::sqrt(noise_pwr / 2.0);
                for (int k = 0; k < 192; ++k) {
                    buf_I[k] = static_cast<int16_t>(
                        std::round(gauss(rng) * noise_std));
                    buf_Q[k] = static_cast<int16_t>(
                        std::round(gauss(rng) * noise_std));
                }
                ScanResult sc = phase0_scan(buf_I, buf_Q, TX_AMP, k_factor);
                if (sc.pass)
                    ++fp;
            }
            std::printf("%3d/%d ", fp, 50);
        }
        std::printf("\n");
    }
    // -----------------------------------------------------------------
    // Part 4: 1000 Hz 의 정확한 실패 원인 분류
    // -----------------------------------------------------------------
    std::printf(
        "\n============================================================\n");
    std::printf("[Part 4] 1000 Hz V3 모드 실패 원인 분류 (30 trials)\n");
    std::printf(
        "============================================================\n\n");
    Summary s1000 =
        run_monte_carlo(1000.0, TX_AMP, SNR, 30, CfoMode::V3_RENORM, 38, false);
    std::printf("  총 %d trials 중 pass %d, fail %d\n", s1000.n_trials,
                s1000.pass_count, s1000.n_trials - s1000.pass_count);
    std::printf("  실패 게이트:\n");
    std::printf("    off_ok=0:   %d trials\n", s1000.fail_off_ok);
    std::printf("    r_avg_ok=0: %d trials\n", s1000.fail_r_avg_ok);
    std::printf("    e63_ok=0:   %d trials\n", s1000.fail_e63_ok);
    std::printf("    sep_ok=0:   %d trials  ← 시뮬 주 원인?\n",
                s1000.fail_sep_ok);
    std::printf("  평균값:\n");
    std::printf("    best_e63:   %.0f\n", s1000.mean_best_e63);
    std::printf("    second_e63: %.0f\n", s1000.mean_second_e63);
    std::printf("    e63_min:    %.0f\n", s1000.mean_e63_min);
    std::printf("    r_sep:      %.0f (1.25 미만이면 sep_ok 탈락)\n",
                s1000.mean_r_sep);
    // -----------------------------------------------------------------
    // Part 5: 실운용 타겟 CFO (재난망/AMI/군사용)
    // -----------------------------------------------------------------
    std::printf(
        "\n============================================================\n");
    std::printf("[Part 5] 실운용 타겟 CFO 성능\n");
    std::printf(
        "============================================================\n\n");
    struct TargetCase {
        const char *name;
        double cfo;
    };
    const TargetCase targets[] = {
        {"KT 재난망 정적", 1000},    {"AMI 스마트미터", 10000},
        {"NIS 시설간", 5000},        {"IoT 산업용", 24000},
        {"WiFi/BLE 표준", 48000},    {"군용 전술 (저속)", 20000},
        {"군용 전술 (고속)", 50000}, {"항공/함정", 100000},
    };
    std::printf("  %-25s %-10s | %-22s | %-22s\n", "타겟", "CFO(Hz)",
                mode_name(CfoMode::BUGGY), mode_name(CfoMode::V3_RENORM));
    std::printf("  %s\n", std::string(90, '-').c_str());
    for (const auto &tc : targets) {
        std::printf("  %-25s %-10.0f | ", tc.name, tc.cfo);
        Summary sB =
            run_monte_carlo(tc.cfo, TX_AMP, SNR, N_TRIALS, CfoMode::BUGGY, 38);
        Summary sV = run_monte_carlo(tc.cfo, TX_AMP, SNR, N_TRIALS,
                                     CfoMode::V3_RENORM, 38);
        std::printf("pass %2d/%d               | pass %2d/%d\n", sB.pass_count,
                    sB.n_trials, sV.pass_count, sV.n_trials);
    }
    std::printf("\n완료.\n");
    return 0;
}

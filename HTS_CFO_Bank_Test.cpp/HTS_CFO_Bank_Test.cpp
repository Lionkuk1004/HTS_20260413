// ============================================================================
// HTS_CFO_Bank_Test.cpp — Stage 5: ±12 kHz Parameter Verification
// ============================================================================
// 목표:
//   AMI (900 MHz) + PS-LTE (Band 28, 718~783 MHz) 공통 커버
//   저가 TCXO (±5 ppm) × 2 = ±10 ppm 최대
//   + Doppler (KTX 300 km/h 최대) + 온도 마진
//   → ±12 kHz 공통 Bank 설계
//
// 구조 변경:
//   V5a 이전 (±30 kHz): Coarse 21 × 3kHz + Fine 11 × 300Hz = 64 derotate
//   V5a Stage 5 (±12 kHz): Coarse 9 × 3kHz + Fine 11 × 300Hz = 40 derotate
//
// 검증 항목:
//   1. Bank 범위 내 CFO 에서 정밀도 유지
//   2. V9 5 Level 에서 정밀도 비교 (±30 vs ±12)
//   3. 속도 개선 측정
//   4. AMI/PS-LTE 실전 CFO 분포 시뮬레이션
//
// 이식 전 최종 검증
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <windows.h>
// ============================================================================
// HTS CFO Bank Range — 양산 파라미터
// ============================================================================
#ifndef HTS_CFO_RANGE_HZ
#define HTS_CFO_RANGE_HZ 12000 // AMI + PS-LTE 공통 (저가 TCXO ±5 ppm)
#endif
#ifndef HTS_CFO_COARSE_STEP_HZ
#define HTS_CFO_COARSE_STEP_HZ 3000
#endif
#ifndef HTS_CFO_FINE_STEP_HZ
#define HTS_CFO_FINE_STEP_HZ 300
#endif
#ifndef HTS_CFO_FINE_BANKS
#define HTS_CFO_FINE_BANKS 11 // ±1500 Hz around coarse winner
#endif
namespace {
constexpr int kChipsPerSym = 64;
constexpr int kPreambleChips = 128;
constexpr int kChipRateHz = 1000000;
// Bank 파라미터 (매크로로부터 계산)
constexpr int kCfoMin = -HTS_CFO_RANGE_HZ;
constexpr int kCfoMax = +HTS_CFO_RANGE_HZ;
constexpr int kCoarseStep = HTS_CFO_COARSE_STEP_HZ;
constexpr int kCoarseBankCount = (kCfoMax - kCfoMin) / kCoarseStep + 1;
constexpr int kFineStep = HTS_CFO_FINE_STEP_HZ;
constexpr int kFineBankCount = HTS_CFO_FINE_BANKS;
// 테스트 CFO 범위 (Bank 범위 내)
constexpr int kTestCfoMin = -HTS_CFO_RANGE_HZ;
constexpr int kTestCfoMax = +HTS_CFO_RANGE_HZ;
constexpr int kTestCfoStep = 1000;
constexpr int kTestCfoCount = (kTestCfoMax - kTestCfoMin) / kTestCfoStep + 1;
constexpr int kTrialsPerPoint = 200;
constexpr int8_t kW63[kChipsPerSym] = {
    1,  -1, -1, 1,  -1, 1,  1,  -1, -1, 1,  1,  -1, 1,  -1, -1, 1,
    -1, 1,  1,  -1, 1,  -1, -1, 1,  1,  -1, -1, 1,  -1, 1,  1,  -1,
    -1, 1,  1,  -1, 1,  -1, -1, 1,  1,  -1, -1, 1,  -1, 1,  1,  -1,
    1,  -1, -1, 1,  -1, 1,  1,  -1, -1, 1,  1,  -1, 1,  -1, -1, 1};
constexpr int16_t kTxAmp = 2000;
constexpr int32_t kQ14One = 16384;
constexpr double kPi = 3.14159265358979323846;
// Sin/Cos Table (Stage 2)
constexpr int kSinCosTableSize = 1024;
constexpr int kSinCosIndexShift = 22;
int16_t g_sin_table[kSinCosTableSize];
int16_t g_cos_table[kSinCosTableSize];
// L&R 파라미터
constexpr int kLR_SegSize = 16;
constexpr int kLR_NumSeg = 8;
constexpr int kLR_MaxLag = 4;
// V9 Difficulty Levels
enum DifficultyLevel {
    LVL1_NORMAL,
    LVL2_URBAN,
    LVL3_INDUSTRIAL,
    LVL4_MILITARY,
    LVL5_EXTREME
};
struct RandomEnv {
    int snr_db;
    double init_phase;
    double cfo_drift_hz_per_chip;
    int n_path;
    int mp_delay[4];
    double mp_gain[4];
    double mp_phase[4];
    int n_cw;
    double cw_freq[3];
    double cw_phase[3];
    double cw_amp[3];
    bool sweep_enable;
    double sweep_start_hz;
    double sweep_rate_hz_per_chip;
    double sweep_amp;
    double impulse_prob;
    double impulse_amp;
    double agc_scale;
    double dc_I, dc_Q;
};
} // anonymous namespace
// ============================================================================
// Timer
// ============================================================================
static LARGE_INTEGER g_freq_timer;
static inline void timer_init() noexcept {
    QueryPerformanceFrequency(&g_freq_timer);
}
static inline int64_t timer_now() noexcept {
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return now.QuadPart;
}
static inline double timer_ns(int64_t diff) noexcept {
    return static_cast<double>(diff) * 1e9 /
           static_cast<double>(g_freq_timer.QuadPart);
}
// ============================================================================
// 유틸
// ============================================================================
static inline void q14_sincos_double(double a, int32_t &s,
                                     int32_t &c) noexcept {
    s = static_cast<int32_t>(std::sin(a) * kQ14One);
    c = static_cast<int32_t>(std::cos(a) * kQ14One);
}
static uint32_t g_rng = 1u;
static inline void rng_seed(uint32_t seed) noexcept {
    g_rng = seed ? seed : 1u;
}
static inline uint32_t rng_next() noexcept {
    g_rng = g_rng * 1664525u + 1013904223u;
    return g_rng;
}
static inline double rng_uniform() noexcept {
    return static_cast<double>(rng_next()) / 4294967296.0;
}
static inline double rng_uniform_signed() noexcept {
    return rng_uniform() * 2.0 - 1.0;
}
static inline int rng_range_int(int lo, int hi) noexcept {
    return lo + static_cast<int>(rng_uniform() * (hi - lo + 1));
}
static inline double rng_range(double lo, double hi) noexcept {
    return lo + rng_uniform() * (hi - lo);
}
static inline double rng_gauss() noexcept {
    double u1 = rng_uniform();
    double u2 = rng_uniform();
    if (u1 < 1e-10)
        u1 = 1e-10;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}
// ============================================================================
// Sin/Cos Table 생성
// ============================================================================
static void build_sincos_table() noexcept {
    for (int i = 0; i < kSinCosTableSize; ++i) {
        const double angle = 2.0 * kPi * i / kSinCosTableSize;
        g_sin_table[i] = static_cast<int16_t>(std::sin(angle) * kQ14One);
        g_cos_table[i] = static_cast<int16_t>(std::cos(angle) * kQ14One);
    }
}
// ============================================================================
// Derotate (Table + Phase Precompute)
// ============================================================================
static void derotate(const int16_t *rI, const int16_t *rQ, int16_t *oI,
                     int16_t *oQ, int chips, int cfo_hz) noexcept {
    const int64_t phase_inc_q32_s64 =
        -(static_cast<int64_t>(cfo_hz) * 4294967296LL) / kChipRateHz;
    const uint32_t phase_inc_q32 = static_cast<uint32_t>(phase_inc_q32_s64);
    uint32_t phase_q32 = 0;
    for (int k = 0; k < chips; ++k) {
        const int idx =
            (phase_q32 >> kSinCosIndexShift) & (kSinCosTableSize - 1);
        const int32_t cos_q14 = g_cos_table[idx];
        const int32_t sin_q14 = g_sin_table[idx];
        const int32_t x = rI[k];
        const int32_t y = rQ[k];
        int32_t a = (x * cos_q14 - y * sin_q14) >> 14;
        int32_t b = (x * sin_q14 + y * cos_q14) >> 14;
        if (a > 32767)
            a = 32767;
        if (a < -32768)
            a = -32768;
        if (b > 32767)
            b = 32767;
        if (b < -32768)
            b = -32768;
        oI[k] = static_cast<int16_t>(a);
        oQ[k] = static_cast<int16_t>(b);
        phase_q32 += phase_inc_q32;
    }
}
// ============================================================================
// Walsh / Energy / L&R
// ============================================================================
static inline void walsh63_dot(const int16_t *rI, const int16_t *rQ,
                               int32_t &dI, int32_t &dQ) noexcept {
    int32_t aI = 0, aQ = 0;
    for (int k = 0; k < kChipsPerSym; ++k) {
        const int8_t w = kW63[k];
        aI += static_cast<int32_t>(rI[k]) * w;
        aQ += static_cast<int32_t>(rQ[k]) * w;
    }
    dI = aI;
    dQ = aQ;
}
static inline int64_t energy_multiframe(const int16_t *rI,
                                        const int16_t *rQ) noexcept {
    int32_t d0I, d0Q, d1I, d1Q;
    walsh63_dot(&rI[0], &rQ[0], d0I, d0Q);
    walsh63_dot(&rI[kChipsPerSym], &rQ[kChipsPerSym], d1I, d1Q);
    return (int64_t)d0I * d0I + (int64_t)d0Q * d0Q + (int64_t)d1I * d1I +
           (int64_t)d1Q * d1Q;
}
struct LR_Result {
    double cfo_hz;
};
static LR_Result lr_estimator(const int16_t *rI, const int16_t *rQ) noexcept {
    LR_Result r;
    int32_t seg_I[kLR_NumSeg];
    int32_t seg_Q[kLR_NumSeg];
    for (int s = 0; s < kLR_NumSeg; ++s) {
        int32_t aI = 0, aQ = 0;
        const int base = s * kLR_SegSize;
        for (int k = 0; k < kLR_SegSize; ++k) {
            const int walsh_idx = (base + k) % kChipsPerSym;
            const int8_t w = kW63[walsh_idx];
            aI += (int32_t)rI[base + k] * w;
            aQ += (int32_t)rQ[base + k] * w;
        }
        seg_I[s] = aI;
        seg_Q[s] = aQ;
    }
    double accu_re = 0.0;
    double accu_im = 0.0;
    for (int lag = 1; lag <= kLR_MaxLag; ++lag) {
        int64_t R_re = 0;
        int64_t R_im = 0;
        for (int n = 0; n + lag < kLR_NumSeg; ++n) {
            const int64_t aI = seg_I[n + lag];
            const int64_t aQ = seg_Q[n + lag];
            const int64_t bI = seg_I[n];
            const int64_t bQ = seg_Q[n];
            R_re += aI * bI + aQ * bQ;
            R_im += aQ * bI - aI * bQ;
        }
        accu_re += (double)R_re;
        accu_im += (double)R_im;
    }
    const double phase = std::atan2(accu_im, accu_re);
    const double M_mean = (kLR_MaxLag + 1) / 2.0;
    const double T_seg_sec = kLR_SegSize / (double)kChipRateHz;
    r.cfo_hz = phase / (2.0 * kPi * M_mean * T_seg_sec);
    return r;
}
// ============================================================================
// V5a Detect
// ============================================================================
struct DetectResult {
    bool pass;
    int32_t cfo_hz;
};
static DetectResult detect_v5a(const int16_t *rx_I,
                               const int16_t *rx_Q) noexcept {
    DetectResult res = {};
    int16_t tI[kPreambleChips], tQ[kPreambleChips];
    int32_t cfo_estimate = 0;
    for (int pass = 0; pass < 2; ++pass) {
        // Stage 1: Coarse
        int32_t cb_cfo = 0;
        int64_t cb_e = 0;
        for (int b = 0; b < kCoarseBankCount; ++b) {
            const int cfo_total = cfo_estimate + (kCfoMin + b * kCoarseStep);
            derotate(rx_I, rx_Q, tI, tQ, kPreambleChips, cfo_total);
            const int64_t e = energy_multiframe(tI, tQ);
            if (e > cb_e) {
                cb_e = e;
                cb_cfo = cfo_total;
            }
        }
        // Stage 2: Fine
        int32_t best_cfo = cb_cfo;
        int64_t best_e = cb_e;
        for (int f = 0; f < kFineBankCount; ++f) {
            const int cfo =
                cb_cfo - (kFineBankCount / 2) * kFineStep + f * kFineStep;
            derotate(rx_I, rx_Q, tI, tQ, kPreambleChips, cfo);
            const int64_t e = energy_multiframe(tI, tQ);
            if (e > best_e) {
                best_e = e;
                best_cfo = cfo;
            }
        }
        // Stage 3: L&R
        derotate(rx_I, rx_Q, tI, tQ, kPreambleChips, best_cfo);
        LR_Result lr = lr_estimator(tI, tQ);
        cfo_estimate = best_cfo + (int32_t)lr.cfo_hz;
        if (pass == 0) {
            const int64_t thr = (int64_t)kTxAmp * 38;
            const int64_t thr_sq = thr * thr;
            res.pass = (best_e >= thr_sq);
        }
    }
    res.cfo_hz = cfo_estimate;
    return res;
}
// ============================================================================
// 채널 (V9 축약)
// ============================================================================
static void generate_preamble(int16_t *tx_I, int16_t *tx_Q,
                              int16_t amp) noexcept {
    for (int k = 0; k < kChipsPerSym; ++k) {
        tx_I[k] = static_cast<int16_t>(kW63[k] * amp);
        tx_Q[k] = 0;
    }
    for (int k = 0; k < kChipsPerSym; ++k) {
        tx_I[kChipsPerSym + k] = static_cast<int16_t>(kW63[k] * amp);
        tx_Q[kChipsPerSym + k] = 0;
    }
}
static RandomEnv generate_env(DifficultyLevel lvl, uint32_t env_seed) noexcept {
    rng_seed(env_seed);
    RandomEnv e = {};
    e.init_phase = rng_uniform() * 2.0 * kPi;
    e.agc_scale = rng_range(0.9, 1.1);
    switch (lvl) {
    case LVL1_NORMAL:
        e.snr_db = rng_range_int(10, 25);
        e.n_path = rng_range_int(0, 1);
        e.n_cw = 0;
        e.sweep_enable = false;
        e.impulse_prob = 0;
        e.dc_I = rng_uniform_signed() * 0.02 * kTxAmp;
        e.dc_Q = rng_uniform_signed() * 0.02 * kTxAmp;
        break;
    case LVL2_URBAN:
        e.snr_db = rng_range_int(5, 20);
        e.cfo_drift_hz_per_chip = rng_range(0, 0.3);
        e.n_path = rng_range_int(1, 3);
        e.n_cw = rng_range_int(0, 1);
        e.impulse_prob = rng_range(0, 0.005);
        e.agc_scale = rng_range(0.85, 1.15);
        e.dc_I = rng_uniform_signed() * 0.04 * kTxAmp;
        e.dc_Q = rng_uniform_signed() * 0.04 * kTxAmp;
        break;
    case LVL3_INDUSTRIAL:
        e.snr_db = rng_range_int(0, 15);
        e.cfo_drift_hz_per_chip = rng_range(0, 0.5);
        e.n_path = rng_range_int(1, 3);
        e.n_cw = rng_range_int(1, 2);
        e.impulse_prob = rng_range(0.005, 0.02);
        e.agc_scale = rng_range(0.8, 1.2);
        e.dc_I = rng_uniform_signed() * 0.05 * kTxAmp;
        e.dc_Q = rng_uniform_signed() * 0.05 * kTxAmp;
        break;
    case LVL4_MILITARY:
        e.snr_db = rng_range_int(-5, 10);
        e.cfo_drift_hz_per_chip = rng_range(0.2, 0.8);
        e.n_path = rng_range_int(2, 4);
        e.n_cw = rng_range_int(1, 3);
        e.sweep_enable = (rng_uniform() > 0.5);
        e.impulse_prob = rng_range(0.01, 0.03);
        e.agc_scale = rng_range(0.75, 1.25);
        e.dc_I = rng_uniform_signed() * 0.06 * kTxAmp;
        e.dc_Q = rng_uniform_signed() * 0.06 * kTxAmp;
        break;
    case LVL5_EXTREME:
        e.snr_db = rng_range_int(-20, 5);
        e.cfo_drift_hz_per_chip = rng_range(0.5, 1.5);
        e.n_path = rng_range_int(2, 4);
        e.n_cw = 3;
        e.sweep_enable = (rng_uniform() > 0.3);
        e.impulse_prob = rng_range(0.02, 0.05);
        e.agc_scale = rng_range(0.7, 1.3);
        e.dc_I = rng_uniform_signed() * 0.07 * kTxAmp;
        e.dc_Q = rng_uniform_signed() * 0.07 * kTxAmp;
        break;
    default:
        break;
    }
    for (int i = 0; i < e.n_path; ++i) {
        e.mp_delay[i] = rng_range_int(1, 8);
        e.mp_gain[i] = rng_range(0.2, 0.8);
        e.mp_phase[i] = rng_uniform() * 2.0 * kPi;
    }
    const double jsr_min = (lvl == LVL4_MILITARY)  ? 5.0
                           : (lvl == LVL5_EXTREME) ? 10.0
                                                   : 0.0;
    const double jsr_max = (lvl == LVL4_MILITARY)  ? 15.0
                           : (lvl == LVL5_EXTREME) ? 30.0
                                                   : 5.0;
    for (int i = 0; i < e.n_cw; ++i) {
        // CW 재밍도 Bank 범위 근처로 제한 (실제 환경 모사)
        e.cw_freq[i] = rng_uniform_signed() * 8000.0; // ±8 kHz 내
        e.cw_phase[i] = rng_uniform() * 2.0 * kPi;
        const double jsr = rng_range(jsr_min, jsr_max);
        e.cw_amp[i] = kTxAmp * std::pow(10.0, jsr / 20.0);
    }
    if (e.sweep_enable) {
        e.sweep_start_hz = rng_uniform_signed() * 5000.0;
        e.sweep_rate_hz_per_chip = rng_range(-300, 300);
        const double sj_jsr = rng_range(jsr_min, jsr_max);
        e.sweep_amp = kTxAmp * std::pow(10.0, sj_jsr / 20.0);
    }
    e.impulse_amp = kTxAmp * rng_range(5.0, 15.0);
    return e;
}
static void apply_random_channel(const int16_t *tx_I, const int16_t *tx_Q,
                                 int16_t *rx_I, int16_t *rx_Q, int chips,
                                 int cfo_hz, const RandomEnv &env,
                                 uint32_t ch_seed) noexcept {
    rng_seed(ch_seed);
    double phase = env.init_phase;
    const double base_phase_inc = 2.0 * kPi * cfo_hz / kChipRateHz;
    const double noise_sigma = kTxAmp * std::pow(10.0, -env.snr_db / 20.0);
    double cw_phase_state[3];
    for (int i = 0; i < 3; ++i)
        cw_phase_state[i] = env.cw_phase[i];
    double sweep_phase = 0.0;
    double sweep_freq = env.sweep_start_hz;
    for (int k = 0; k < chips; ++k) {
        const double drift = base_phase_inc + 2.0 * kPi *
                                                  env.cfo_drift_hz_per_chip *
                                                  k / kChipRateHz;
        int32_t sin_q14, cos_q14;
        q14_sincos_double(phase, sin_q14, cos_q14);
        double rotI = (double)(tx_I[k] * cos_q14 - tx_Q[k] * sin_q14) / kQ14One;
        double rotQ = (double)(tx_I[k] * sin_q14 + tx_Q[k] * cos_q14) / kQ14One;
        rotI *= env.agc_scale;
        rotQ *= env.agc_scale;
        for (int p = 0; p < env.n_path; ++p) {
            if (k >= env.mp_delay[p]) {
                const double mp_ph =
                    phase - base_phase_inc * env.mp_delay[p] + env.mp_phase[p];
                int32_t ms, mc;
                q14_sincos_double(mp_ph, ms, mc);
                const int32_t mI = tx_I[k - env.mp_delay[p]];
                const int32_t mQ = tx_Q[k - env.mp_delay[p]];
                rotI += env.mp_gain[p] * (double)(mI * mc - mQ * ms) / kQ14One;
                rotQ += env.mp_gain[p] * (double)(mI * ms + mQ * mc) / kQ14One;
            }
        }
        for (int i = 0; i < env.n_cw; ++i) {
            rotI += env.cw_amp[i] * std::cos(cw_phase_state[i]);
            rotQ += env.cw_amp[i] * std::sin(cw_phase_state[i]);
            cw_phase_state[i] += 2.0 * kPi * env.cw_freq[i] / kChipRateHz;
        }
        if (env.sweep_enable) {
            rotI += env.sweep_amp * std::cos(sweep_phase);
            rotQ += env.sweep_amp * std::sin(sweep_phase);
            sweep_phase += 2.0 * kPi * sweep_freq / kChipRateHz;
            sweep_freq += env.sweep_rate_hz_per_chip;
        }
        rotI += rng_gauss() * noise_sigma;
        rotQ += rng_gauss() * noise_sigma;
        if (rng_uniform() < env.impulse_prob) {
            rotI += rng_gauss() * env.impulse_amp;
            rotQ += rng_gauss() * env.impulse_amp;
        }
        rotI += env.dc_I;
        rotQ += env.dc_Q;
        int32_t fI = (int32_t)rotI;
        int32_t fQ = (int32_t)rotQ;
        if (fI > 32767)
            fI = 32767;
        if (fI < -32768)
            fI = -32768;
        if (fQ > 32767)
            fQ = 32767;
        if (fQ < -32768)
            fQ = -32768;
        rx_I[k] = (int16_t)fI;
        rx_Q[k] = (int16_t)fQ;
        phase += drift;
    }
}
// ============================================================================
// AMI / PS-LTE 실전 CFO 분포 모사
// ============================================================================
// 저가 TCXO ±5 ppm × 2 = ±10 ppm
// AMI 900 MHz: ±9 kHz
// PS-LTE 718~783 MHz: ±7.8 kHz
// + 온도 마진 30%
// → Gaussian 분포 (σ = 3 kHz, 99% 내)
static int sample_realistic_cfo(uint32_t seed) noexcept {
    rng_seed(seed);
    const double sigma = 3000.0; // ppm drift 평균
    double cfo = rng_gauss() * sigma;
    // Clamp to ±10 kHz (realistic max)
    if (cfo > 10000.0)
        cfo = 10000.0;
    if (cfo < -10000.0)
        cfo = -10000.0;
    return (int)cfo;
}
// ============================================================================
// 메인
// ============================================================================
int main() {
    timer_init();
    build_sincos_table();
    std::printf(
        "=============================================================\n");
    std::printf("  HTS V5a — Stage 5: ±%d kHz Bank Verification\n",
                HTS_CFO_RANGE_HZ / 1000);
    std::printf(
        "=============================================================\n\n");
    std::printf("타겟 응용:\n");
    std::printf("  AMI 스마트미터링 (900 MHz 대): ±9 kHz max\n");
    std::printf("  PS-LTE 재난망 (718~783 MHz):   ±7.8 kHz max\n");
    std::printf("  저가 TCXO ±5 ppm × 2 = ±10 ppm\n");
    std::printf("  + 온도/Doppler 마진 → ±12 kHz 공통 Bank\n\n");
    std::printf("Bank 구성:\n");
    std::printf("  CFO range:    ±%d Hz\n", HTS_CFO_RANGE_HZ);
    std::printf("  Coarse banks: %d × %d Hz = ±%d Hz\n", kCoarseBankCount,
                kCoarseStep, (kCoarseBankCount - 1) * kCoarseStep / 2);
    std::printf("  Fine banks:   %d × %d Hz\n", kFineBankCount, kFineStep);
    std::printf("  Total derotate: %d × 2 pass = %d calls\n",
                kCoarseBankCount + kFineBankCount,
                (kCoarseBankCount + kFineBankCount) * 2);
    std::printf("\n");
    std::printf("  (이전 ±30 kHz: 21+11 = 32 × 2 = 64 calls)\n");
    std::printf("  (현재 ±12 kHz: %d+%d = %d × 2 = %d calls)\n",
                kCoarseBankCount, kFineBankCount,
                kCoarseBankCount + kFineBankCount,
                (kCoarseBankCount + kFineBankCount) * 2);
    std::printf("  (감소율: %.1f%%)\n",
                100.0 *
                    (1.0 - (double)(kCoarseBankCount + kFineBankCount) / 32.0));
    std::printf("\n");
    int16_t tx_I[kPreambleChips], tx_Q[kPreambleChips];
    generate_preamble(tx_I, tx_Q, kTxAmp);
    // === 1. Bank 범위 내 CFO Sweep ===
    std::printf("=== 1. Bank 범위 내 CFO 정밀도 ===\n");
    std::printf("CFO sweep: %+d ~ %+d Hz, step %d\n", kTestCfoMin, kTestCfoMax,
                kTestCfoStep);
    std::printf("Trials per point: %d\n\n", kTrialsPerPoint);
    DifficultyLevel levels[] = {LVL1_NORMAL, LVL2_URBAN, LVL3_INDUSTRIAL,
                                LVL4_MILITARY, LVL5_EXTREME};
    const char *level_names[] = {"LVL1_NORMAL    ", "LVL2_URBAN     ",
                                 "LVL3_INDUSTRIAL", "LVL4_MILITARY  ",
                                 "LVL5_EXTREME   "};
    std::printf("Level           | PASS%%  | Avg Err  | Max Err  | Comment\n");
    std::printf("----------------+--------+----------+----------+----------\n");
    FILE *fcsv = nullptr;
    fopen_s(&fcsv, "stage5_results.csv", "w");
    if (fcsv) {
        std::fprintf(fcsv, "Level,PASS_pct,Avg_Err_Hz,Max_Err_Hz\n");
    }
    double lvl_errs[5];
    double lvl_pass[5];
    for (int l = 0; l < 5; ++l) {
        int64_t err_sum = 0;
        int64_t err_max = 0;
        int pass_count = 0;
        int total_count = 0;
        for (int c = 0; c < kTestCfoCount; ++c) {
            const int cfo_hz = kTestCfoMin + c * kTestCfoStep;
            for (int t = 0; t < kTrialsPerPoint; ++t) {
                const uint32_t env_seed = static_cast<uint32_t>(
                    (l + 1) * 1000003u + (c + 1) * 9901u + (t + 1) * 97u);
                const uint32_t ch_seed = env_seed ^ 0xDEADBEEFu;
                RandomEnv env = generate_env(levels[l], env_seed);
                int16_t rI[kPreambleChips], rQ[kPreambleChips];
                apply_random_channel(tx_I, tx_Q, rI, rQ, kPreambleChips, cfo_hz,
                                     env, ch_seed);
                DetectResult r = detect_v5a(rI, rQ);
                ++total_count;
                if (r.pass) {
                    ++pass_count;
                    const int32_t e = r.cfo_hz - cfo_hz;
                    const int64_t abs_e = (e < 0) ? -e : e;
                    err_sum += abs_e;
                    if (abs_e > err_max)
                        err_max = abs_e;
                }
            }
        }
        const double pass_pct = 100.0 * pass_count / total_count;
        const double avg_err =
            (pass_count > 0) ? (double)err_sum / pass_count : 0.0;
        const char *comment = "";
        if (l == 0)
            comment = "AMI/PS-LTE 정상";
        else if (l == 1)
            comment = "도시 환경";
        else if (l == 2)
            comment = "산업 EMI";
        else if (l == 3)
            comment = "재밍 대응";
        else
            comment = "극한 (한계)";
        std::printf("%s | %5.1f%% | %7.0f Hz | %7.0f Hz | %s\n", level_names[l],
                    pass_pct, avg_err, (double)err_max, comment);
        if (fcsv) {
            std::fprintf(fcsv, "%s,%.1f,%.0f,%.0f\n", level_names[l], pass_pct,
                         avg_err, (double)err_max);
        }
        lvl_errs[l] = avg_err;
        lvl_pass[l] = pass_pct;
    }
    if (fcsv)
        std::fclose(fcsv);
    std::printf("\n");
    // === 2. 속도 측정 ===
    std::printf("=== 2. 속도 측정 (Full V5a) ===\n");
    int16_t rx_I[kPreambleChips], rx_Q[kPreambleChips];
    RandomEnv env = generate_env(LVL2_URBAN, 42);
    apply_random_channel(tx_I, tx_Q, rx_I, rx_Q, kPreambleChips, 5000, env,
                         123);
    // Warm-up
    for (int i = 0; i < 1000; ++i) {
        DetectResult r = detect_v5a(rx_I, rx_Q);
        (void)r;
    }
    constexpr int kSpeedTrials = 5000;
    const int64_t t0 = timer_now();
    for (int t = 0; t < kSpeedTrials; ++t) {
        DetectResult r = detect_v5a(rx_I, rx_Q);
        (void)r;
    }
    const int64_t t1 = timer_now();
    const double full_us = timer_ns(t1 - t0) / kSpeedTrials / 1000.0;
    std::printf("  V5a ±12 kHz: %.2f us/call\n", full_us);
    std::printf("  V5a ±30 kHz: 16.13 us/call (이전 기준)\n");
    std::printf("  개선:        %.1f%%\n", 100.0 * (1.0 - full_us / 16.13));
    std::printf("\n");
    // === 3. AMI/PS-LTE 실전 CFO 분포 ===
    std::printf("=== 3. AMI/PS-LTE 실전 CFO 분포 (Gaussian σ=3kHz) ===\n");
    int ami_pass = 0;
    int ami_total = 0;
    int64_t ami_err = 0;
    for (int t = 0; t < 10000; ++t) {
        const uint32_t cfo_seed = static_cast<uint32_t>(t * 37u + 1);
        const uint32_t env_seed = static_cast<uint32_t>(t * 53u + 17);
        const uint32_t ch_seed = env_seed ^ 0xABCD1234u;
        const int cfo_hz = sample_realistic_cfo(cfo_seed);
        // LVL2 (도시 환경) 가장 흔함
        RandomEnv e = generate_env(LVL2_URBAN, env_seed);
        int16_t rI[kPreambleChips], rQ[kPreambleChips];
        apply_random_channel(tx_I, tx_Q, rI, rQ, kPreambleChips, cfo_hz, e,
                             ch_seed);
        DetectResult r = detect_v5a(rI, rQ);
        ++ami_total;
        if (r.pass) {
            ++ami_pass;
            const int32_t err = r.cfo_hz - cfo_hz;
            ami_err += (err < 0) ? -err : err;
        }
    }
    const double ami_pass_pct = 100.0 * ami_pass / ami_total;
    const double ami_avg_err =
        (ami_pass > 0) ? (double)ami_err / ami_pass : 0.0;
    std::printf("  Trials:  %d (LVL2_URBAN, realistic CFO)\n", ami_total);
    std::printf("  PASS:    %.1f%%\n", ami_pass_pct);
    std::printf("  Avg Err: %.0f Hz\n", ami_avg_err);
    std::printf("\n");
    // === 4. 최종 판정 ===
    std::printf("=== 4. Stage 5 판정 ===\n");
    const bool pass_lvl1 = (lvl_pass[0] >= 99.0);
    const bool pass_lvl2 = (lvl_pass[1] >= 95.0);
    const bool pass_lvl3 = (lvl_pass[2] >= 90.0);
    const bool fast_enough = (full_us < 12.0);
    if (pass_lvl1 && pass_lvl2 && pass_lvl3 && fast_enough) {
        std::printf("  ✅ ±12 kHz Bank 양산 적합\n");
        std::printf("     - LVL1 PASS: %.1f%%\n", lvl_pass[0]);
        std::printf("     - LVL2 PASS: %.1f%%\n", lvl_pass[1]);
        std::printf("     - LVL3 PASS: %.1f%%\n", lvl_pass[2]);
        std::printf("     - Full V5a: %.2f us (목표 <12 us)\n", full_us);
        std::printf("     - AMI/PS-LTE 실전: PASS %.1f%%, Err %.0f Hz\n",
                    ami_pass_pct, ami_avg_err);
        std::printf("\n");
        std::printf("  → 양산 이식 준비 완료\n");
    } else {
        std::printf("  주의: 조건 미달\n");
        if (!pass_lvl1)
            std::printf("     LVL1 PASS 낮음: %.1f%%\n", lvl_pass[0]);
        if (!pass_lvl2)
            std::printf("     LVL2 PASS 낮음: %.1f%%\n", lvl_pass[1]);
        if (!pass_lvl3)
            std::printf("     LVL3 PASS 낮음: %.1f%%\n", lvl_pass[2]);
        if (!fast_enough)
            std::printf("     속도 느림: %.2f us\n", full_us);
    }
    std::printf("\n");
    std::printf("참고:\n");
    std::printf("  [1] AMI ISM 900 MHz: ±9 kHz @ TCXO ±5 ppm\n");
    std::printf("  [2] PS-LTE Band 28 (718~783 MHz): ±7.8 kHz\n");
    std::printf("  [3] 저가 TCXO BOM 비중: ~12%%\n");
    return 0;
}

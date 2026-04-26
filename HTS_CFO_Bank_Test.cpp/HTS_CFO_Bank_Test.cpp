// ============================================================================
// HTS_CFO_Bank_Test.cpp — Phase H Lab: V5a + 4D Tensor Integration
// ============================================================================
// 영준님 격리 실험 환경
// 목적:
//   - V5a CFO 보정 + HTS_Holo_Tensor_4D 디코딩 통합
//   - HOLO sync ON/OFF 분기 (S5/S5H 시뮬)
//   - rx_soft 변환 빠른 실험 ((I+Q)/2 vs I-only vs 기타)
//   - cfo sweep 측정 → 진짜 FAIL 위치 식별
//
// 영준님 의도:
//   Cursor 거치지 않고 VS 에서 직접 빠른 iteration
//   영준님 코드 직접 인용 (TX I=Q duplicated, RX (I+Q)/2)
//   메인트리 영향 0 (격리 환경)
//
// 영준님 메모리 #7, #8, #9 준수:
//   #7 BPTE: 분기 없는 패턴
//   #8 측정 기반: 추측 금지
//   #9 DPTE 양산 도구셋
// ============================================================================
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
// ──────────────────────────────────────────────────────────
// 영준님 4D 텐서 헤더 (메인트리 그대로 사용) — <windows.h> 전에 include 하여
// SAL/Windows 매크로(_Order 등)가 <atomic>·표준 머신 헤더를 깨는 것을 방지
// ──────────────────────────────────────────────────────────
#include "../HTS_LIM/HTS_Holo_Tensor_4D_TX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_RX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_Defs.h"
#include <windows.h>
using namespace ProtectedEngine;
// ============================================================================
// ★★★ 영준님 수정 가능 영역 (실험 파라미터) ★★★
// ============================================================================
namespace ExperimentConfig {
// CFO sweep 범위 (Hz)
constexpr int kCfoSweepList[] = {0,   50,   100,  200,  300, 500,
                                 700, 1000, 2000, 3000, 5000};
constexpr int kCfoSweepCount = sizeof(kCfoSweepList) / sizeof(int);
// 트라이얼 수
constexpr int kNumTrials = 100;
// SNR 레벨
constexpr int kSnrLevels[] = {30, 20, 10};
constexpr int kSnrCount = sizeof(kSnrLevels) / sizeof(int);
// 4D 텐서 profile (k_holo_profiles[1] DATA default 와 일치)
constexpr int kK = 16;     // block_bits
constexpr int kN = 64;     // chip_count
constexpr int kL = 2;      // num_layers
constexpr int kAmp = 500;  // amplitude
constexpr int kDenom = 16; // (L*K)/2 = 32/2 (영준님 TX 코드)
// V5a 보정 ON/OFF
constexpr bool kEnableV5aCorrection = true;
// HOLO sync 모드 (S5 vs S5H 시뮬)
//   false: S5 (일반 동기, 동기 자동 통과)
//   true:  S5H (HOLO 동기 시뮬, threshold 검증)
constexpr bool kEnableHoloSync = false; // ★ 영준님 변경 가능
// HOLO sync threshold (영준님 메모리 #6 [B]: e63_min=amp×38)
constexpr int kHoloSyncE63Threshold = kAmp * 38;
// Channel chip rate
constexpr double kChipRateHz = 1e6;
// rx_soft 변환 모드
//   0: (I+Q)/2 (현재 메인트리, 학계 MRC)
//   1: I 만
//   2: Q 만 (테스트)
//   3: |I| > |Q| 시 I, 아니면 Q (dominant)
//   4: I + Q (단순 합)
constexpr int kRxSoftMode = 0; // ★ 영준님 변경 가능
// BPTE chip validity threshold (0 = 모두 valid)
constexpr int32_t kChipValidThreshold = 0; // ★ 영준님 변경 가능
} // namespace ExperimentConfig
// ============================================================================
// Sin/Cos LUT (Q14)
// ============================================================================
namespace {
constexpr int kSinCosTableSize = 1024;
constexpr int kSinCosIndexShift = 22;
constexpr int32_t kQ14One = 16384;
constexpr double kPi = 3.14159265358979323846;
int16_t g_sin_table[kSinCosTableSize];
int16_t g_cos_table[kSinCosTableSize];
static void build_sincos_table() noexcept {
    for (int i = 0; i < kSinCosTableSize; ++i) {
        const double angle = 2.0 * kPi * i / kSinCosTableSize;
        g_sin_table[i] = static_cast<int16_t>(std::sin(angle) * kQ14One);
        g_cos_table[i] = static_cast<int16_t>(std::cos(angle) * kQ14One);
    }
}
} // namespace
// ============================================================================
// 난수 생성기
// ============================================================================
namespace {
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
static inline double rng_gauss() noexcept {
    double u1 = rng_uniform();
    double u2 = rng_uniform();
    if (u1 < 1e-10)
        u1 = 1e-10;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}
} // namespace
// ============================================================================
// Channel: CFO 회전 + AWGN
// ============================================================================
static void channel_apply(const int16_t *tx_I, const int16_t *tx_Q,
                          int16_t *rx_I, int16_t *rx_Q, int chips,
                          double cfo_hz, int snr_db,
                          uint32_t ch_seed) noexcept {
    rng_seed(ch_seed);
    // 신호 전력 계산
    double sig_pow = 0.0;
    for (int k = 0; k < chips; ++k) {
        sig_pow += static_cast<double>(tx_I[k]) * tx_I[k];
        sig_pow += static_cast<double>(tx_Q[k]) * tx_Q[k];
    }
    sig_pow /= chips;
    const double noise_sigma =
        (sig_pow > 0.0) ? std::sqrt(sig_pow / std::pow(10.0, snr_db / 10.0))
                        : 0.0;
    const double phase_inc = 2.0 * kPi * cfo_hz / ExperimentConfig::kChipRateHz;
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
// ============================================================================
// V5a 단순 CFO 추정 + 보정 (autocorr lag-32)
// ============================================================================
//   영준님 V5a 의 단순 모델
//   autocorr ac = Σ I[n]·I[n+32] + Q[n]·Q[n+32]
//                  Σ Q[n]·I[n+32] - I[n]·Q[n+32]
//   atan2 → cfo_hz
static double v5a_estimate_cfo(const int16_t *rx_I, const int16_t *rx_Q,
                               int chips, int lag) noexcept {
    double ac_I = 0.0;
    double ac_Q = 0.0;
    for (int n = 0; n + lag < chips; ++n) {
        ac_I += static_cast<double>(rx_I[n]) * rx_I[n + lag] +
                static_cast<double>(rx_Q[n]) * rx_Q[n + lag];
        ac_Q += static_cast<double>(rx_Q[n]) * rx_I[n + lag] -
                static_cast<double>(rx_I[n]) * rx_Q[n + lag];
    }
    if (std::abs(ac_I) < 1e-6 && std::abs(ac_Q) < 1e-6) {
        return 0.0;
    }
    const double phase = std::atan2(ac_Q, ac_I);
    const double cfo_hz =
        phase * ExperimentConfig::kChipRateHz / (2.0 * kPi * lag);
    return cfo_hz;
}
// V5a per-chip CFO 보정 (LUT 기반, 양산 V5a 와 유사)
static void v5a_apply_per_chip(const int16_t *rx_I, const int16_t *rx_Q,
                               int16_t *out_I, int16_t *out_Q, int chips,
                               double cfo_hz) noexcept {
    const int64_t phase_inc_q32_s64 =
        -(static_cast<int64_t>(cfo_hz * 4294967296.0) /
          static_cast<int64_t>(ExperimentConfig::kChipRateHz));
    const uint32_t phase_inc_q32 = static_cast<uint32_t>(phase_inc_q32_s64);
    uint32_t phase_q32 = 0;
    for (int k = 0; k < chips; ++k) {
        const int idx =
            (phase_q32 >> kSinCosIndexShift) & (kSinCosTableSize - 1);
        const int32_t cos_q14 = g_cos_table[idx];
        const int32_t sin_q14 = g_sin_table[idx];
        const int32_t x = rx_I[k];
        const int32_t y = rx_Q[k];
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
        out_I[k] = static_cast<int16_t>(a);
        out_Q[k] = static_cast<int16_t>(b);
        phase_q32 += phase_inc_q32;
    }
}
// ============================================================================
// HOLO 동기 시뮬 (영준님 메모리 #6 [B])
//   - PRE_SYM0 = 0x3F (Walsh row 63)
//   - threshold: e63_min = amp × 38
//   - 단순 FWHT 기반 sync 임계 검증
// ============================================================================
static void fwht_64(int32_t *data) noexcept {
    int h = 1;
    while (h < 64) {
        for (int i = 0; i < 64; i += h * 2) {
            for (int j = i; j < i + h; ++j) {
                const int32_t x = data[j];
                const int32_t y = data[j + h];
                data[j] = x + y;
                data[j + h] = x - y;
            }
        }
        h *= 2;
    }
}
static bool holo_sync_check(const int16_t *rx_I, const int16_t *rx_Q) noexcept {
    int32_t wI[64], wQ[64];
    for (int k = 0; k < 64; ++k) {
        wI[k] = rx_I[k];
        wQ[k] = rx_Q[k];
    }
    fwht_64(wI);
    fwht_64(wQ);
    // bin 63 의 에너지 (PRE_SYM0=0x3F)
    const int64_t e63 = static_cast<int64_t>(wI[63]) * wI[63] +
                        static_cast<int64_t>(wQ[63]) * wQ[63];
    const int64_t thr =
        static_cast<int64_t>(ExperimentConfig::kHoloSyncE63Threshold) *
        ExperimentConfig::kHoloSyncE63Threshold;
    return e63 >= thr;
}
// HOLO preamble 생성 (TX 측, 영준님 코드의 walsh_enc 패턴)
static void generate_holo_preamble(int16_t *tx_I, int16_t *tx_Q,
                                   int amp) noexcept {
    // PRE_SYM0 = 0x3F = Walsh row 63
    const uint8_t row = 63;
    for (int c = 0; c < 64; ++c) {
        uint32_t x = row & static_cast<uint32_t>(c);
        const int parity = static_cast<int>(std::popcount(x) & 1u);
        // chip = amp × (-1)^parity
        tx_I[c] = static_cast<int16_t>(parity ? -amp : amp);
        tx_Q[c] = 0;
    }
}
// ============================================================================
// rx_soft 생성 (★ 영준님 수정 가능)
// ============================================================================
static void rx_soft_generate(const int16_t *buf_I, const int16_t *buf_Q,
                             int16_t *rx_soft, int N,
                             uint64_t *out_valid_mask) noexcept {
    uint64_t valid_mask = 0u;
    for (int c = 0; c < N; ++c) {
        const int32_t i_val = static_cast<int32_t>(buf_I[c]);
        const int32_t q_val = static_cast<int32_t>(buf_Q[c]);
        // BPTE chip validity (영준님 메모리 #7)
        int32_t valid_chip;
        if (ExperimentConfig::kChipValidThreshold > 0) {
            const int32_t mag_sq = i_val * i_val + q_val * q_val;
            const int64_t diff =
                static_cast<int64_t>(mag_sq) -
                static_cast<int64_t>(ExperimentConfig::kChipValidThreshold);
            valid_chip = static_cast<int32_t>(~(diff >> 63));
        } else {
            valid_chip = -1; // all 1s (모두 valid)
        }
        // rx_soft 변환 (모드 별)
        int32_t soft;
        switch (ExperimentConfig::kRxSoftMode) {
        case 0: // (I+Q)/2 — 메인트리 현재
            soft = (i_val + q_val) / 2;
            break;
        case 1: // I 만
            soft = i_val;
            break;
        case 2: // Q 만
            soft = q_val;
            break;
        case 3: { // dominant (|I| > |Q| 시 I)
            const int32_t abs_i = (i_val < 0) ? -i_val : i_val;
            const int32_t abs_q = (q_val < 0) ? -q_val : q_val;
            soft = (abs_i > abs_q) ? i_val : q_val;
            break;
        }
        case 4: // I + Q
            soft = i_val + q_val;
            break;
        default:
            soft = (i_val + q_val) / 2;
            break;
        }
        // BPTE invalidation
        rx_soft[c] = static_cast<int16_t>(soft & valid_chip);
        // valid_mask 갱신
        valid_mask |= (static_cast<uint64_t>(valid_chip & 1u)
                       << static_cast<uint32_t>(c));
    }
    *out_valid_mask = valid_mask;
}
// ============================================================================
// TX (영준님 Build_Packet 의 4D 텐서 분기 직접 포팅)
// ============================================================================
static int tx_build_tensor(HTS_Holo_Tensor_4D_TX &tensor, const int8_t *data_bits,
                           int16_t *tx_I, int16_t *tx_Q, int amp) noexcept {
    int8_t chip_bpsk[64];
    if (tensor.Encode_Block(
            data_bits, static_cast<uint16_t>(ExperimentConfig::kK), chip_bpsk,
            static_cast<uint16_t>(ExperimentConfig::kN)) !=
        HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        return 0;
    }
    // 영준님 코드:
    //   prod = chip_bpsk[c] × amp
    //   v = (prod >= 0) ? ((prod + denom/2) / denom)
    //                   : ((prod - denom/2) / denom)
    //   oI[pos] = v;
    //   oQ[pos] = v;  ← I = Q duplicated
    const int denom = ExperimentConfig::kDenom;
    const int half_denom = denom >> 1;
    for (int c = 0; c < ExperimentConfig::kN; ++c) {
        const int32_t prod = static_cast<int32_t>(chip_bpsk[c]) * amp;
        const int32_t v = (prod >= 0) ? ((prod + half_denom) / denom)
                                      : ((prod - half_denom) / denom);
        tx_I[c] = static_cast<int16_t>(v);
        tx_Q[c] = static_cast<int16_t>(v);
    }
    return ExperimentConfig::kN;
}
// ============================================================================
// 시나리오 결과
// ============================================================================
struct TestResult {
    int sync_pass;
    int decode_pass;
    int total_bit_err;
    int total_bits;
};
// ============================================================================
// CFO Sweep 측정 (한 SNR)
// ============================================================================
static TestResult run_one_cfo(int cfo_hz, int snr_db, int num_trials) {
    TestResult res = {0, 0, 0, 0};
    HTS_Holo_Tensor_4D_TX tx_tensor;
    HTS_Holo_Tensor_4D_RX rx_tensor;
    uint32_t master_seed[4] = {0x00100000u, 0x9E2779B9u, 0xA5B5A5A5u,
                               0xC3D3C3C3u};
    if (tx_tensor.Initialize(master_seed, nullptr) !=
        HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        return res;
    }
    if (rx_tensor.Initialize(master_seed, nullptr) !=
        HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
        tx_tensor.Shutdown();
        return res;
    }
    for (int trial = 0; trial < num_trials; ++trial) {
        // 데이터 생성 (랜덤)
        rng_seed(static_cast<uint32_t>(trial * 7919u + 13u));
        int8_t data_bits[16];
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            data_bits[i] = (rng_next() & 1u) ? 1 : -1;
        }
        // TX: 4D 텐서 + I=Q 변조
        tx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
        int16_t tx_I[64], tx_Q[64];
        if (tx_build_tensor(tx_tensor, data_bits, tx_I, tx_Q,
                            ExperimentConfig::kAmp) == 0) {
            continue;
        }
        // HOLO sync 모드: preamble 추가 송신 (시뮬은 동기 단계만)
        bool sync_ok = true;
        if (ExperimentConfig::kEnableHoloSync) {
            int16_t pre_I[64], pre_Q[64];
            generate_holo_preamble(pre_I, pre_Q, ExperimentConfig::kAmp);
            // 채널 적용
            int16_t pre_rx_I[64], pre_rx_Q[64];
            const uint32_t pre_seed = static_cast<uint32_t>(trial * 31u + 7u);
            channel_apply(pre_I, pre_Q, pre_rx_I, pre_rx_Q, 64,
                          static_cast<double>(cfo_hz), snr_db, pre_seed);
            // V5a 보정 (preamble 도)
            int16_t pre_corr_I[64], pre_corr_Q[64];
            if (ExperimentConfig::kEnableV5aCorrection) {
                const double cfo_est =
                    v5a_estimate_cfo(pre_rx_I, pre_rx_Q, 64, 32);
                v5a_apply_per_chip(pre_rx_I, pre_rx_Q, pre_corr_I, pre_corr_Q,
                                   64, cfo_est);
            } else {
                std::memcpy(pre_corr_I, pre_rx_I, sizeof(pre_corr_I));
                std::memcpy(pre_corr_Q, pre_rx_Q, sizeof(pre_corr_Q));
            }
            // HOLO sync 임계 검증
            sync_ok = holo_sync_check(pre_corr_I, pre_corr_Q);
        }
        if (sync_ok) {
            ++res.sync_pass;
        } else {
            // sync 실패 = decode 시도 안 함 (S5H FAIL 시뮬)
            res.total_bits += ExperimentConfig::kK;
            res.total_bit_err += ExperimentConfig::kK; // worst case
            continue;
        }
        // 채널: payload chip
        int16_t rx_I[64], rx_Q[64];
        const uint32_t ch_seed = static_cast<uint32_t>(trial * 31u + 19u);
        channel_apply(tx_I, tx_Q, rx_I, rx_Q, ExperimentConfig::kN,
                      static_cast<double>(cfo_hz), snr_db, ch_seed);
        // V5a CFO 보정
        int16_t corr_I[64], corr_Q[64];
        if (ExperimentConfig::kEnableV5aCorrection) {
            const double cfo_est =
                v5a_estimate_cfo(rx_I, rx_Q, ExperimentConfig::kN, 32);
            v5a_apply_per_chip(rx_I, rx_Q, corr_I, corr_Q, ExperimentConfig::kN,
                               cfo_est);
        } else {
            std::memcpy(corr_I, rx_I, sizeof(corr_I));
            std::memcpy(corr_Q, rx_Q, sizeof(corr_Q));
        }
        // RX rx_soft 생성 (★ 영준님 수정 가능 영역)
        int16_t rx_soft[64];
        uint64_t valid_mask = 0xFFFFFFFFFFFFFFFFull;
        rx_soft_generate(corr_I, corr_Q, rx_soft, ExperimentConfig::kN,
                         &valid_mask);
        // 4D 텐서 디코딩
        rx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
        int8_t output_bits[16];
        if (rx_tensor.Decode_Block(
                rx_soft, static_cast<uint16_t>(ExperimentConfig::kN),
                valid_mask, output_bits,
                static_cast<uint16_t>(ExperimentConfig::kK)) !=
            HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            res.total_bits += ExperimentConfig::kK;
            res.total_bit_err += ExperimentConfig::kK;
            continue;
        }
        // 검증
        int bit_err = 0;
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            if (output_bits[i] != data_bits[i])
                ++bit_err;
        }
        res.total_bit_err += bit_err;
        res.total_bits += ExperimentConfig::kK;
        if (bit_err == 0)
            ++res.decode_pass;
    }
    rx_tensor.Shutdown();
    tx_tensor.Shutdown();
    return res;
}
// ============================================================================
// rx_soft 모드 이름
// ============================================================================
static const char *rx_soft_mode_name(int mode) {
    switch (mode) {
    case 0:
        return "(I+Q)/2";
    case 1:
        return "I only";
    case 2:
        return "Q only";
    case 3:
        return "dominant";
    case 4:
        return "I+Q";
    default:
        return "unknown";
    }
}
// ============================================================================
// 메인 (test_holo_tensor_cfo_sweep)
// ============================================================================
void test_holo_tensor_cfo_sweep() {
    build_sincos_table();
    std::printf("\n");
    std::printf(
        "============================================================\n");
    std::printf(
        "  Phase H Lab: V5a + 4D Tensor CFO Sweep (영준님 격리 환경)\n");
    std::printf(
        "============================================================\n");
    std::printf("Profile: K=%d, N=%d, L=%d (영준님 4D 텐서 default)\n",
                ExperimentConfig::kK, ExperimentConfig::kN,
                ExperimentConfig::kL);
    std::printf("Trials per (cfo, SNR): %d\n", ExperimentConfig::kNumTrials);
    std::printf("Chip rate: %.0f Hz (1 MHz 가정)\n",
                ExperimentConfig::kChipRateHz);
    std::printf("V5a correction: %s\n",
                ExperimentConfig::kEnableV5aCorrection ? "ON" : "OFF");
    std::printf("HOLO sync: %s (S%s sim)\n",
                ExperimentConfig::kEnableHoloSync ? "ON" : "OFF",
                ExperimentConfig::kEnableHoloSync ? "5H" : "5");
    std::printf("rx_soft mode: %d (%s)\n", ExperimentConfig::kRxSoftMode,
                rx_soft_mode_name(ExperimentConfig::kRxSoftMode));
    std::printf("ChipValidThreshold: %d %s\n",
                static_cast<int>(ExperimentConfig::kChipValidThreshold),
                ExperimentConfig::kChipValidThreshold == 0 ? "(all valid)"
                                                           : "");
    std::printf(
        "============================================================\n\n");
    // 헤더
    std::printf("%-8s", "cfo Hz");
    for (int s = 0; s < ExperimentConfig::kSnrCount; ++s) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), " %dB sync/dec/BER",
                      ExperimentConfig::kSnrLevels[s]);
        std::printf(" %-22s", buf);
    }
    std::printf("\n");
    std::printf(
        "------------------------------------------------------------\n");
    for (int c = 0; c < ExperimentConfig::kCfoSweepCount; ++c) {
        const int cfo_hz = ExperimentConfig::kCfoSweepList[c];
        std::printf("%-8d", cfo_hz);
        for (int s = 0; s < ExperimentConfig::kSnrCount; ++s) {
            const int snr_db = ExperimentConfig::kSnrLevels[s];
            const TestResult r =
                run_one_cfo(cfo_hz, snr_db, ExperimentConfig::kNumTrials);
            const double ber =
                (r.total_bits > 0)
                    ? static_cast<double>(r.total_bit_err) / r.total_bits
                    : 0.0;
            std::printf(
                "  %3d/%3d/%4.2f  %-4s", r.sync_pass, r.decode_pass, ber,
                (r.decode_pass >= ExperimentConfig::kNumTrials * 95 / 100)
                    ? "OK"
                    : "");
        }
        std::printf("\n");
    }
    std::printf("\n");
    std::printf("Notes:\n");
    std::printf("  sync = HOLO sync 통과 수 (HOLO sync OFF 시 N/N)\n");
    std::printf("  dec  = 디코딩 100%% 성공 수\n");
    std::printf("  BER  = Bit Error Rate\n");
    std::printf("  OK   = decode >= 95%%\n");
    std::printf("\n");
    std::printf("영준님 실험:\n");
    std::printf("  1. kEnableHoloSync = false → S5 시뮬\n");
    std::printf("  2. kEnableHoloSync = true  → S5H 시뮬\n");
    std::printf("  3. kRxSoftMode 변경 (0~4)\n");
    std::printf("  4. kChipValidThreshold 변경\n");
    std::printf("  5. F5 빠른 재실행\n");
    std::printf(
        "============================================================\n");
}
// ============================================================================
// main 진입점
// ============================================================================
int main() {
    test_holo_tensor_cfo_sweep();
    return 0;
}

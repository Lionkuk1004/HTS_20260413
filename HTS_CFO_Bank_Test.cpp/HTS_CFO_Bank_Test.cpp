// ============================================================================
// HTS_CFO_Bank_Test.cpp — Phase H Lab v3
// ★★★ Decode_Block_Two_Candidates 연결 (양산 RX derotate + BPSK 양 후보)
// ★★ 50Hz/100Hz 풀 정밀 sweep (v2 유지, 301점)
// ============================================================================
// 목적:
//   - V5a CFO 보정 + HTS_Holo_Tensor_4D 디코딩 통합
//   - HOLO sync ON/OFF 분기 (S5/S5H 시뮬)
//   - cfo sweep 측정 → cliff/회복 식별
//
// 변경점 (2026-04-26 v3):
//   - 기존: rx_tensor.Decode_Block(rx_soft, ...)  ← 단순 1D
//   - 변경: rx_tensor.Decode_Block_Two_Candidates(corr_I, corr_Q, ...)
//           ← 양산 derotate + BPSK cand0/cand1 양 후보
//   - Lab: cand0/cand1 중 data_bits 대비 bit_err 작은 쪽 선택
//     (양산은 CRC 또는 known reference; 여기서는 측정용 ground truth)
//
// 컴파일 옵션 (RX TU / Lab vcxproj):
//   /DHTS_HOLO_RX_PHASE_B_DEROTATE  ← 이 TU printf 라벨용(선택). 4D RX는
//       HTS_Holo_Tensor_4D_RX.cpp 에서 잔여 dφ derotate 항상 적용.
//
// kCfoSweepList (v2): 0~5000Hz @50Hz, 5100~25000Hz @100Hz, 예상 ~38분
//
// 영준님 메모리 #7, #8, #9 준수: BPTE 패턴, 측정 기반, DPTE 양산 도구셋
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
#include "../HTS_LIM/HTS_Holo_Tensor_4D_Defs.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_RX.h"
#include "../HTS_LIM/HTS_Holo_Tensor_4D_TX.h"
#include <windows.h>
using namespace ProtectedEngine;
// ============================================================================
// ★★★ 영준님 수정 가능 영역 (실험 파라미터) ★★★
// ============================================================================
namespace ExperimentConfig {
// ★★ 풀 정밀 CFO sweep — 50Hz/100Hz 단위 (총 301 점)
constexpr int kCfoSweepList[] = {
    // 저~중 CFO 정밀 (0~5000Hz, 50Hz step) — 101 점
    0,
    50,
    100,
    150,
    200,
    250,
    300,
    350,
    400,
    450,
    500,
    550,
    600,
    650,
    700,
    750,
    800,
    850,
    900,
    950,
    1000,
    1050,
    1100,
    1150,
    1200,
    1250,
    1300,
    1350,
    1400,
    1450,
    1500,
    1550,
    1600,
    1650,
    1700,
    1750,
    1800,
    1850,
    1900,
    1950,
    2000,
    2050,
    2100,
    2150,
    2200,
    2250,
    2300,
    2350,
    2400,
    2450,
    2500,
    2550,
    2600,
    2650,
    2700,
    2750,
    2800,
    2850,
    2900,
    2950,
    3000,
    3050,
    3100,
    3150,
    3200,
    3250,
    3300,
    3350,
    3400,
    3450,
    3500,
    3550,
    3600,
    3650,
    3700,
    3750,
    3800,
    3850,
    3900,
    3950,
    4000,
    4050,
    4100,
    4150,
    4200,
    4250,
    4300,
    4350,
    4400,
    4450,
    4500,
    4550,
    4600,
    4650,
    4700,
    4750,
    4800,
    4850,
    4900,
    4950,
    5000,
    // 고 CFO 정밀 (5100~25000Hz, 100Hz step) — 200 점
    5100,
    5200,
    5300,
    5400,
    5500,
    5600,
    5700,
    5800,
    5900,
    6000,
    6100,
    6200,
    6300,
    6400,
    6500,
    6600,
    6700,
    6800,
    6900,
    7000,
    7100,
    7200,
    7300,
    7400,
    7500,
    7600,
    7700,
    7800,
    7900,
    8000,
    8100,
    8200,
    8300,
    8400,
    8500,
    8600,
    8700,
    8800,
    8900,
    9000,
    9100,
    9200,
    9300,
    9400,
    9500,
    9600,
    9700,
    9800,
    9900,
    10000,
    10100,
    10200,
    10300,
    10400,
    10500,
    10600,
    10700,
    10800,
    10900,
    11000,
    11100,
    11200,
    11300,
    11400,
    11500,
    11600,
    11700,
    11800,
    11900,
    12000,
    12100,
    12200,
    12300,
    12400,
    12500,
    12600,
    12700,
    12800,
    12900,
    13000,
    13100,
    13200,
    13300,
    13400,
    13500,
    13600,
    13700,
    13800,
    13900,
    14000,
    14100,
    14200,
    14300,
    14400,
    14500,
    14600,
    14700,
    14800,
    14900,
    15000,
    15100,
    15200,
    15300,
    15400,
    15500,
    15600,
    15700,
    15800,
    15900,
    16000,
    16100,
    16200,
    16300,
    16400,
    16500,
    16600,
    16700,
    16800,
    16900,
    17000,
    17100,
    17200,
    17300,
    17400,
    17500,
    17600,
    17700,
    17800,
    17900,
    18000,
    18100,
    18200,
    18300,
    18400,
    18500,
    18600,
    18700,
    18800,
    18900,
    19000,
    19100,
    19200,
    19300,
    19400,
    19500,
    19600,
    19700,
    19800,
    19900,
    20000,
    20100,
    20200,
    20300,
    20400,
    20500,
    20600,
    20700,
    20800,
    20900,
    21000,
    21100,
    21200,
    21300,
    21400,
    21500,
    21600,
    21700,
    21800,
    21900,
    22000,
    22100,
    22200,
    22300,
    22400,
    22500,
    22600,
    22700,
    22800,
    22900,
    23000,
    23100,
    23200,
    23300,
    23400,
    23500,
    23600,
    23700,
    23800,
    23900,
    24000,
    24100,
    24200,
    24300,
    24400,
    24500,
    24600,
    24700,
    24800,
    24900,
    25000,
};
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
// TX (영준님 Build_Packet 의 4D 텐서 분기 직접 포팅)
// ============================================================================
static int tx_build_tensor(HTS_Holo_Tensor_4D_TX &tensor,
                           const int8_t *data_bits, int16_t *tx_I,
                           int16_t *tx_Q, int amp) noexcept {
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
// CFO Sweep 측정 (한 SNR) — Decode_Block_Two_Candidates + Lab 후보 선택
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
        // ★ 양산 RX: I/Q → (선택 derotate) → Walsh; φ / φ+π 후보 동시 반환
        rx_tensor.Set_Time_Slot(static_cast<uint32_t>(trial));
        int8_t bits_cand0[16];
        int8_t bits_cand1[16];
        const uint64_t valid_mask = 0xFFFFFFFFFFFFFFFFull;
        if (rx_tensor.Decode_Block_Two_Candidates(
                corr_I, corr_Q,
                static_cast<uint16_t>(ExperimentConfig::kN), valid_mask,
                bits_cand0, bits_cand1,
                static_cast<uint16_t>(ExperimentConfig::kK)) !=
            HTS_Holo_Tensor_4D_TX::SECURE_TRUE) {
            res.total_bits += ExperimentConfig::kK;
            res.total_bit_err += ExperimentConfig::kK;
            continue;
        }
        // Lab: CRC 없음 → ground truth 대비 bit_err 작은 후보 (양산과 구분)
        int bit_err_0 = 0;
        int bit_err_1 = 0;
        for (int i = 0; i < ExperimentConfig::kK; ++i) {
            if (bits_cand0[i] != data_bits[i])
                ++bit_err_0;
            if (bits_cand1[i] != data_bits[i])
                ++bit_err_1;
        }
        const int bit_err =
            (bit_err_0 <= bit_err_1) ? bit_err_0 : bit_err_1;
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
// 메인 (test_holo_tensor_cfo_sweep)
// ============================================================================
void test_holo_tensor_cfo_sweep() {
    build_sincos_table();
    std::printf("\n");
    std::printf(
        "============================================================\n");
    std::printf(
        "  Phase H Lab v3: V5a + 4D Tensor (Two_Candidates 연결)\n");
    std::printf("  ★ 양산 RX Decode_Block_Two_Candidates 호출\n");
    std::printf("  ★ HTS_HOLO_RX_PHASE_B_DEROTATE (이 TU): %s\n",
#if defined(HTS_HOLO_RX_PHASE_B_DEROTATE)
                "ON (양산 derotate 활성)"
#else
                "OFF (derotate 비활성)"
#endif
    );
    std::printf(
        "     실제 derotate는 HTS_LIM 빌드 시 RX.cpp에 동일 매크로 필요\n");
    std::printf(
        "============================================================\n");
    std::printf("Profile: K=%d, N=%d, L=%d\n",
                ExperimentConfig::kK, ExperimentConfig::kN,
                ExperimentConfig::kL);
    std::printf("Trials per (cfo, SNR): %d\n", ExperimentConfig::kNumTrials);
    std::printf("CFO sweep points: %d (0~5000Hz @50Hz, 5100~25000Hz @100Hz)\n",
                ExperimentConfig::kCfoSweepCount);
    std::printf("Chip rate: %.0f Hz (1 MHz 가정)\n",
                ExperimentConfig::kChipRateHz);
    std::printf("V5a correction: %s\n",
                ExperimentConfig::kEnableV5aCorrection ? "ON" : "OFF");
    std::printf("HOLO sync: %s (S%s sim)\n",
                ExperimentConfig::kEnableHoloSync ? "ON" : "OFF",
                ExperimentConfig::kEnableHoloSync ? "5H" : "5");
    std::printf("예상 시간: ~38 분 (3 SNR x 100 trial x 301 cfo)\n");
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
        // 매 50 점마다 fflush (긴 실행 중 부분 결과 보존)
        if ((c + 1) % 50 == 0) {
            std::fflush(stdout);
        }
    }
    std::printf("\n");
    std::printf("Notes:\n");
    std::printf("  sync = HOLO sync 통과 수 (HOLO sync OFF 시 N/N)\n");
    std::printf("  dec  = 디코딩 100%% 성공 수 (Two_Candidates + min bit_err)\n");
    std::printf("  BER  = Bit Error Rate\n");
    std::printf("  OK   = decode >= 95%%\n");
    std::printf("\n");
    std::printf("  ★ Decode_Block_Two_Candidates (양산 RX 경로)\n");
    std::printf("  ★ cand0/cand1 → ground truth 대비 bit_err 작은 쪽 (Lab 전용)\n");
    std::printf("  ★ HTS_HOLO_RX_PHASE_B_DEROTATE 로 derotate ON/OFF\n");
    std::printf("\n");
    std::printf("실험 매트릭스:\n");
    std::printf("  실험 1: 매크로 OFF + V5a ON  (Two_C only, baseline)\n");
    std::printf("  실험 2: 매크로 ON  + V5a ON  (양산 풀 경로)\n");
    std::printf("  실험 3: 매크로 ON  + V5a OFF (derotate 단독 효과)\n");
    std::printf("\n");
    std::printf("  kEnableHoloSync: false=S5, true=S5H (preamble+threshold)\n");
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

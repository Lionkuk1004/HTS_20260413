// =============================================================================
// HTS_T6_SIM_Test.cpp — V3: 무결 채널 시뮬레이션 (꼼수 제로)
// =============================================================================
//
// [금지 사항]
//   1. Feed_Chip 전 channel_phase_correct 등 사전 보상 일절 금지
//   2. 양수 SNR 안전 영역 기만 금지 → 음수 SNR (-30~-10 dB) 필수
//   3. success_mask만 검사 금지 → memcmp(expected, received, 8) 강제
//   4. 코어 루프 내 printf 금지 → 배열 누적 후 일괄 출력
//   5. 고정값 튜닝으로 통과시키기 금지
//
// [검증 원칙]
//   수신기는 임의 위상/타이밍/CFO 상태에서 블라인드 획득해야 한다.
//   FAIL이 나오면 그것이 현재 엔진의 실제 한계이다.
//
// 빌드: cd HTS_TEST\t6_sim
//   cl /nologo /O2 /std:c++17 /EHsc /MD /W3 /I"..\..\HTS_LIM" ^
//     /DHTS_ALLOW_HOST_BUILD /DHTS_FEC_SIMULATE_M4_RAM_LAYOUT ^
//     /DHTS_FEC_POLAR_DISABLE /DHTS_DIAG_PRINTF ^
//     /FeHTS_T6_SIM_Test.exe HTS_T6_SIM_Test.cpp /link /nologo
// =============================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
#error "[HTS_FATAL] PC 전용"
#endif

#include "HTS_V400_Dispatcher.hpp"
#include "HTS_Walsh_Row_Converter.hpp"
#if defined(HTS_HARQ_DIAG)
#include "HTS_HARQ_Diag.hpp"
#endif
#if defined(HTS_AMP_DIAG)
#include "HTS_Amp_Diag.hpp"
#endif
#if defined(HTS_SYNC_DIAG)
#include "HTS_Sync_Diag.hpp"
#endif
#if defined(HTS_WALSH_ROW_DIAG)
#include "HTS_Walsh_Row_Diag.hpp"
#endif
#if defined(HTS_LLR_DIAG)
#include "HTS_LLR_Diag.hpp"
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
#include "HTS_Row_Consistency_Diag.hpp"
#endif
#if defined(HTS_CW_DETECT_DIAG) || defined(HTS_CW_DETECT_DIAG_V2)
#include "HTS_CW_Detect_Diag.hpp"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>

using ProtectedEngine::DecodedPacket;
using ProtectedEngine::FEC_HARQ;
using ProtectedEngine::HTS_V400_Dispatcher;
using ProtectedEngine::PayloadMode;

// ═══════════════════════════════════════════════════════════════
//  상수
// ═══════════════════════════════════════════════════════════════
namespace {

static constexpr int16_t  kAmp       = 1000;
static constexpr int      kPreReps   = 4;
static constexpr int      kPreBoost  = 1;
static constexpr int      kMaxC      = 2048 + (FEC_HARQ::NSYM64 + kPreReps + 4) * 64;
static constexpr int      kGuard     = 256;
static constexpr double   kPi        = 3.14159265358979323846;
static constexpr double   kChipRate  = 200000.0;
static constexpr int      kTrials    = 20;  // 통계적 의미를 위해 20회

// ═══════════════════════════════════════════════════════════════
//  결과 수집 (루프 내 printf 금지 → 배열 누적)
// ═══════════════════════════════════════════════════════════════
struct TrialLog {
    bool pass;
};

struct TrialMetrics {
    bool pass;              // memcmp 완전 일치 (기존 feed_raw 동등)
    bool crc_passed;        // success_mask == DECODE_MASK_OK
    bool length_correct;    // data_len == 8
    int  bit_errors;        // 64 bit 중 틀린 수 (0~64)
    int  byte_errors;       // 8 byte 중 틀린 수 (0~8)
};

struct ScenarioResult {
    char       name[64];
    char       param[32];
    int        pass;             // bit-exact pass 수 (기존)
    int        crc_only_pass;    // CRC+길이 OK, payload bit error 있는 trial 수
    int        total;
    long long  total_bit_errors; // 시나리오별 bit error 합
};

static constexpr int kMaxScenario = 128;
static ScenarioResult g_results[kMaxScenario];
static int g_n_results = 0;

static void record(const char* name, const char* param, int pass, int total) {
    if (g_n_results >= kMaxScenario) return;
    auto& r = g_results[g_n_results++];
    std::strncpy(r.name, name, 63);  r.name[63] = '\0';
    std::strncpy(r.param, param, 31); r.param[31] = '\0';
    r.pass = pass;
    r.crc_only_pass = 0;
    r.total = total;
    r.total_bit_errors = 0;
}

static void record_ext(const char* name, const char* param,
                       int pass, int crc_only, int total,
                       long long bit_errors) {
    if (g_n_results >= kMaxScenario) return;
    auto& r = g_results[g_n_results++];
    std::strncpy(r.name, name, 63);  r.name[63] = '\0';
    std::strncpy(r.param, param, 31); r.param[31] = '\0';
    r.pass = pass;
    r.crc_only_pass = crc_only;
    r.total = total;
    r.total_bit_errors = bit_errors;
}

// ═══════════════════════════════════════════════════════════════
//  콜백
// ═══════════════════════════════════════════════════════════════
static DecodedPacket g_last{};
static void on_pkt(const DecodedPacket& p) { g_last = p; }

// ═══════════════════════════════════════════════════════════════
//  유틸 (채널 모델용 — TX/채널 측에서만 사용, RX 전처리 아님)
// ═══════════════════════════════════════════════════════════════
static int16_t sat16(double v) noexcept {
    if (v >  32767.0) return  32767;
    if (v < -32768.0) return -32768;
    return static_cast<int16_t>(std::lround(v));
}

static void fill_info(uint32_t seed, int t, uint8_t* info) noexcept {
    // seed는 이미 mk_seed()로 혼합됨 — 추가 XOR 없이 직접 사용
    // SplitMix32로 8바이트 독립 생성
    uint32_t s = seed + static_cast<uint32_t>(t) * 0x6C62272Eu + 1u;
    for (int b = 0; b < 8; ++b) {
        s += 0x9E3779B9u;
        uint32_t z = s;
        z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
        z = (z ^ (z >> 16u)) * 0x45D9F3Bu;
        z = z ^ (z >> 16u);
        info[b] = static_cast<uint8_t>(z & 0xFFu);
    }
}

static uint32_t mk_seed(uint32_t base, int t) noexcept {
    return base ^ static_cast<uint32_t>(t * 0x9E3779B9u);
}

// ═══════════════════════════════════════════════════════════════
//  디스패처 공통 설정
// ═══════════════════════════════════════════════════════════════
static void setup(HTS_V400_Dispatcher& d, uint32_t seed) noexcept {
    d.Set_Seed(seed);
    d.Set_IR_Mode(true);
    d.Set_Preamble_Boost(kPreBoost);
    d.Set_Preamble_Reps(kPreReps);
    d.Set_Packet_Callback(on_pkt);
    d.Update_Adaptive_BPS(1000);
    d.Set_Lab_IQ_Mode_Jam_Harness();
}

// ═══════════════════════════════════════════════════════════════
//  TX 빌드
// ═══════════════════════════════════════════════════════════════
struct TxPkt {
    int16_t I[kMaxC];
    int16_t Q[kMaxC];
    uint8_t info[8];
    int     n;
};

static TxPkt build_tx(uint32_t ds, int t) noexcept {
    TxPkt pkt{};
    HTS_V400_Dispatcher tx;
    setup(tx, ds);
    fill_info(ds, t, pkt.info);
    pkt.n = tx.Build_Packet(PayloadMode::DATA, pkt.info, 8, kAmp,
                            pkt.I, pkt.Q, kMaxC);
    return pkt;
}

// ═══════════════════════════════════════════════════════════════
//  RX 검증 + BER (사전 보정 없이 직접 Feed)
//  ★ channel_phase_correct 호출 없음 ★
// ═══════════════════════════════════════════════════════════════
static TrialMetrics feed_raw_ext(uint32_t ds, const int16_t* rxI,
                                 const int16_t* rxQ, int n,
                                 const uint8_t* expected,
                                 int pre_guard = kGuard) noexcept {
    TrialMetrics m{};
    m.pass = false;
    m.crc_passed = false;
    m.length_correct = false;
    m.bit_errors = 64;
    m.byte_errors = 8;

    g_last = DecodedPacket{};
    HTS_V400_Dispatcher rx;
    setup(rx, ds);

    for (int i = 0; i < pre_guard; ++i) rx.Feed_Chip(0, 0);
    for (int i = 0; i < n; ++i)         rx.Feed_Chip(rxI[i], rxQ[i]);
    for (int i = 0; i < kGuard; ++i)    rx.Feed_Chip(0, 0);

    m.crc_passed = (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
    m.length_correct = (g_last.data_len == 8);

    if (m.crc_passed && m.length_correct) {
        int bit_err = 0;
        int byte_err = 0;
        for (int b = 0; b < 8; ++b) {
            uint8_t d = static_cast<uint8_t>(
                static_cast<uint8_t>(g_last.data[b]) ^ expected[b]);
            if (d != 0u) {
                ++byte_err;
                d = static_cast<uint8_t>(d - ((d >> 1) & 0x55u));
                d = static_cast<uint8_t>((d & 0x33u) + ((d >> 2) & 0x33u));
                d = static_cast<uint8_t>((d + (d >> 4)) & 0x0Fu);
                bit_err += static_cast<int>(d);
            }
        }
        m.bit_errors = bit_err;
        m.byte_errors = byte_err;
        m.pass = (bit_err == 0);
    }
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::mark_packet_boundary_v2();
#elif defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::mark_packet_boundary();
#endif
    return m;
}

static bool feed_raw(uint32_t ds, const int16_t* rxI, const int16_t* rxQ,
                     int n, const uint8_t* expected, int pre_guard = kGuard) noexcept {
    return feed_raw_ext(ds, rxI, rxQ, n, expected, pre_guard).pass;
}

static TrialMetrics feed_raw_lpi_ext(uint32_t ds, const int16_t* rxI,
                                     const int16_t* rxQ, int n,
                                     const uint8_t* expected,
                                     const uint32_t lpi_seed[4]) noexcept {
    TrialMetrics m{};
    m.pass = false;
    m.crc_passed = false;
    m.length_correct = false;
    m.bit_errors = 64;
    m.byte_errors = 8;

    g_last = DecodedPacket{};
    HTS_V400_Dispatcher rx;
    setup(rx, ds);
    rx.Enable_Holo_LPI(lpi_seed);

    for (int i = 0; i < kGuard; ++i) rx.Feed_Chip(0, 0);
    for (int i = 0; i < n; ++i)      rx.Feed_Chip(rxI[i], rxQ[i]);
    for (int i = 0; i < kGuard; ++i) rx.Feed_Chip(0, 0);
    rx.Disable_Holo_LPI();

    m.crc_passed = (g_last.success_mask == DecodedPacket::DECODE_MASK_OK);
    m.length_correct = (g_last.data_len == 8);

    if (m.crc_passed && m.length_correct) {
        int bit_err = 0;
        int byte_err = 0;
        for (int b = 0; b < 8; ++b) {
            uint8_t d = static_cast<uint8_t>(
                static_cast<uint8_t>(g_last.data[b]) ^ expected[b]);
            if (d != 0u) {
                ++byte_err;
                d = static_cast<uint8_t>(d - ((d >> 1) & 0x55u));
                d = static_cast<uint8_t>((d & 0x33u) + ((d >> 2) & 0x33u));
                d = static_cast<uint8_t>((d + (d >> 4)) & 0x0Fu);
                bit_err += static_cast<int>(d);
            }
        }
        m.bit_errors = bit_err;
        m.byte_errors = byte_err;
        m.pass = (bit_err == 0);
    }
    return m;
}

static bool feed_raw_lpi(uint32_t ds, const int16_t* rxI, const int16_t* rxQ,
                         int n, const uint8_t* expected,
                         const uint32_t lpi_seed[4]) noexcept {
    return feed_raw_lpi_ext(ds, rxI, rxQ, n, expected, lpi_seed).pass;
}

// ═══════════════════════════════════════════════════════════════
//  채널 모델 (TX 출력에 적용 — RX 보정 아님)
// ═══════════════════════════════════════════════════════════════

/// 위상 회전 (degree)
static void ch_rotate(int16_t* I, int16_t* Q, int n, int deg) noexcept {
    const double rad = deg * kPi / 180.0;
    const double c = std::cos(rad), s = std::sin(rad);
    for (int i = 0; i < n; ++i) {
        const double di = I[i], dq = Q[i];
        I[i] = sat16(di * c - dq * s);
        Q[i] = sat16(di * s + dq * c);
    }
}

/// AWGN (음수 SNR 포함)
static void ch_awgn(int16_t* I, int16_t* Q, int n,
                    double snr_db, std::mt19937& rng) noexcept {
    double ps = 0;
    for (int i = 0; i < n; ++i)
        ps += static_cast<double>(I[i]) * I[i]
            + static_cast<double>(Q[i]) * Q[i];
    ps /= (n > 0) ? static_cast<double>(n) : 1.0;
    const double sigma = std::sqrt(ps / (2.0 * std::pow(10.0, snr_db / 10.0)));
    std::normal_distribution<double> nd(0.0, sigma);
    for (int i = 0; i < n; ++i) {
        I[i] = sat16(I[i] + nd(rng));
        Q[i] = sat16(Q[i] + nd(rng));
    }
}

/// CFO (Hz 단위, 칩레이트 기준)
static void ch_cfo(int16_t* I, int16_t* Q, int n, double cfo_hz) noexcept {
    for (int i = 0; i < n; ++i) {
        const double ph = 2.0 * kPi * cfo_hz * i / kChipRate;
        const double c = std::cos(ph), s = std::sin(ph);
        const double di = I[i], dq = Q[i];
        I[i] = sat16(di * c - dq * s);
        Q[i] = sat16(di * s + dq * c);
    }
}

/// 다중 경로 (3-tap: 직접파 + 반사 2개)
static void ch_multipath_3tap(int16_t* I, int16_t* Q, int n,
                               int d1, double a1,
                               int d2, double a2) noexcept {
    // 역순 (in-place, 큰 지연부터)
    int dmax = (d1 > d2) ? d1 : d2;
    if (dmax >= n) return;
    for (int i = n - 1; i >= dmax; --i) {
        double vi = I[i], vq = Q[i];
        if (i >= d1) { vi += a1 * I[i-d1]; vq += a1 * Q[i-d1]; }
        if (i >= d2) { vi += a2 * I[i-d2]; vq += a2 * Q[i-d2]; }
        I[i] = sat16(vi);
        Q[i] = sat16(vq);
    }
}

/// 바라지 재밍 (JSR 기준: 재머 전력 / 신호 전력)
static void ch_barrage(int16_t* I, int16_t* Q, int n,
                       double jsr_db, std::mt19937& rng) noexcept {
    const double sigma = static_cast<double>(kAmp)
                       * std::sqrt(std::pow(10.0, jsr_db / 10.0));
    std::normal_distribution<double> nd(0.0, sigma);
    for (int i = 0; i < n; ++i) {
        I[i] = sat16(I[i] + nd(rng));
        Q[i] = sat16(Q[i] + nd(rng));
    }
}

/// CW 재밍
static void ch_cw(int16_t* I, int16_t* Q, int n,
                  double jsr_db, double period_chips) noexcept {
    const double a = static_cast<double>(kAmp)
                   * std::sqrt(std::pow(10.0, jsr_db / 10.0));
    for (int i = 0; i < n; ++i) {
        const double ph = 2.0 * kPi * i / period_chips;
        I[i] = sat16(I[i] + a * std::cos(ph));
        Q[i] = sat16(Q[i] + a * std::sin(ph));
    }
}

// ═══════════════════════════════════════════════════════════════
//  출력 헬퍼
// ═══════════════════════════════════════════════════════════════
static void hdr(const char* id, const char* title) {
    std::printf("\n  ┌───────────────────────────────────────────────┐\n");
    std::printf("  │ %-3s %-43s │\n", id, title);
    std::printf("  └───────────────────────────────────────────────┘\n");
}

static void row(const char* label, int pass, int total) {
    double pct = (total > 0) ? 100.0 * pass / total : 0;
    char bar[21] = {};
    int fill = (total > 0) ? (pass * 20 / total) : 0;
    for (int i = 0; i < 20; ++i) bar[i] = (i < fill) ? '#' : '.';
    const char* tag = (pass == total) ? "PASS" :
                      (pass == 0)     ? "FAIL" : "PART";
    std::printf("    %-28s %3d/%3d %5.1f%% [%s] %s\n",
                label, pass, total, pct, bar, tag);
}

// ═══════════════════════════════════════════════════════════════
//  S1: 완벽한 무결성 (클린 채널)
// ═══════════════════════════════════════════════════════════════
static void test_S1() {
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::reset_sync_stats();
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::reset_row_stats();
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::reset_llr_stats();
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::reset_stats();
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::reset_stats();
    ProtectedEngine::CWDetectDiag::set_expect_non_cw(true);
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::reset_stats_v2();
    ProtectedEngine::CWDetectDiag::set_scenario_label(
        ProtectedEngine::CWDetectDiag::CWDetectScenarioLabel::Clean);
#endif
    hdr("S1", "완벽한 무결성 (클린 채널)");
    int ok = 0, crc_only = 0, build_fail = 0;
    long long total_bits = 0;
    for (int t = 0; t < kTrials; ++t) {
        const uint32_t ds = mk_seed(0x100000u, t);
        auto tx = build_tx(ds, t);
        if (tx.n <= 0) {
            ++build_fail;
            total_bits += 64;
            continue;
        }

        const TrialMetrics m = feed_raw_ext(ds, tx.I, tx.Q, tx.n, tx.info);
        if (m.pass) {
            ++ok;
        } else if (m.crc_passed && m.length_correct) {
            ++crc_only;
        }
        total_bits += m.bit_errors;
    }
    char diag[64];
    std::snprintf(diag, sizeof(diag), "B%d/OK%d/CRC+%d/BE%lld",
                  build_fail, ok, crc_only, static_cast<long long>(total_bits));
    row("Clean channel", ok, kTrials);
    record_ext("S1", diag, ok, crc_only, kTrials, total_bits);
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::print_sync_stats("S1");
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::print_row_stats("S1");
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::print_llr_stats("S1");
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::print_stats("S1");
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::print_stats("S1");
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::print_stats_2d("S1");
#endif
}

// ═══════════════════════════════════════════════════════════════
//  S2: 위상 전수 조사 (사전 보정 금지)
// ═══════════════════════════════════════════════════════════════
static void test_S2() {
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::reset_sync_stats();
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::reset_row_stats();
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::reset_llr_stats();
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::reset_stats();
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::reset_stats();
    ProtectedEngine::CWDetectDiag::set_expect_non_cw(true);
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::reset_stats_v2();
    ProtectedEngine::CWDetectDiag::set_scenario_label(
        ProtectedEngine::CWDetectDiag::CWDetectScenarioLabel::Clean);
#endif
    hdr("S2", "위상 전수 조사 (블라인드 획득)");
    const int degs[] = {0, 45, 90, 135, 180, 225, 270, 315};
    for (int deg : degs) {
        int ok = 0, crc_only = 0;
        long long total_bits = 0;
        for (int t = 0; t < kTrials; ++t) {
            const uint32_t ds = mk_seed(0x200000u, t);
            auto tx = build_tx(ds, t);
            if (tx.n <= 0) {
                total_bits += 64;
                continue;
            }
            int16_t rI[kMaxC], rQ[kMaxC];
            std::memcpy(rI, tx.I, sizeof(int16_t) * static_cast<size_t>(tx.n));
            std::memcpy(rQ, tx.Q, sizeof(int16_t) * static_cast<size_t>(tx.n));
            ch_rotate(rI, rQ, tx.n, deg);
            const TrialMetrics m = feed_raw_ext(ds, rI, rQ, tx.n, tx.info);
            if (m.pass) {
                ++ok;
            } else if (m.crc_passed && m.length_correct) {
                ++crc_only;
            }
            total_bits += m.bit_errors;
        }
        char label[32], param[16];
        std::snprintf(label, sizeof(label), "Phase %3d deg", deg);
        std::snprintf(param, sizeof(param), "%ddeg", deg);
        row(label, ok, kTrials);
        record_ext("S2", param, ok, crc_only, kTrials, total_bits);
    }
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::print_sync_stats("S2");
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::print_row_stats("S2");
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::print_llr_stats("S2");
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::print_stats("S2");
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::print_stats_2d("S2");
#endif
}

// ═══════════════════════════════════════════════════════════════
//  S3: 심해 SNR 워터폴 (-30 ~ -10 dB)
// ═══════════════════════════════════════════════════════════════
static void test_S3() {
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::reset_sync_stats();
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::reset_row_stats();
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::reset_llr_stats();
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::reset_stats();
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::reset_stats();
    ProtectedEngine::CWDetectDiag::set_expect_non_cw(true);
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::reset_stats_v2();
    ProtectedEngine::CWDetectDiag::set_scenario_label(
        ProtectedEngine::CWDetectDiag::CWDetectScenarioLabel::LowSnr);
#endif
    hdr("S3", "심해 SNR 워터폴 (-30 ~ +10 dB)");
    std::mt19937 rng(0x30000000u);
    const double snrs[] = {-30, -25, -20, -15, -10, -5, 0, 5, 10};
    for (double snr : snrs) {
        int ok = 0, crc_only = 0;
        long long total_bits = 0;
        for (int t = 0; t < kTrials; ++t) {
            const uint32_t ds = mk_seed(0x300000u, t);
            auto tx = build_tx(ds, t);
            if (tx.n <= 0) {
                total_bits += 64;
                continue;
            }
            int16_t rI[kMaxC], rQ[kMaxC];
            std::memcpy(rI, tx.I, sizeof(int16_t) * static_cast<size_t>(tx.n));
            std::memcpy(rQ, tx.Q, sizeof(int16_t) * static_cast<size_t>(tx.n));
            ch_awgn(rI, rQ, tx.n, snr, rng);
            const TrialMetrics m = feed_raw_ext(ds, rI, rQ, tx.n, tx.info);
            if (m.pass) {
                ++ok;
            } else if (m.crc_passed && m.length_correct) {
                ++crc_only;
            }
            total_bits += m.bit_errors;
        }
        char label[32], param[16];
        std::snprintf(label, sizeof(label), "SNR %+6.0f dB", snr);
        std::snprintf(param, sizeof(param), "%.0fdB", snr);
        row(label, ok, kTrials);
        record_ext("S3", param, ok, crc_only, kTrials, total_bits);
    }
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::print_sync_stats("S3");
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::print_row_stats("S3");
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::print_llr_stats("S3");
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::print_stats("S3");
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::print_stats("S3");
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::print_stats_2d("S3");
#endif
}

// ═══════════════════════════════════════════════════════════════
//  S4: 비동기 타이밍 오프셋 (양수/음수)
// ═══════════════════════════════════════════════════════════════
static void test_S4() {
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::reset_sync_stats();
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::reset_row_stats();
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::reset_llr_stats();
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::reset_stats();
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::reset_stats();
    ProtectedEngine::CWDetectDiag::set_expect_non_cw(true);
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::reset_stats_v2();
    ProtectedEngine::CWDetectDiag::set_scenario_label(
        ProtectedEngine::CWDetectDiag::CWDetectScenarioLabel::Clean);
#endif
    hdr("S4", "비동기 타이밍 오프셋");
    const int offsets[] = {0, 1, 5, 17, 31, 63, 127};
    for (int off : offsets) {
        int ok = 0, crc_only = 0;
        long long total_bits = 0;
        for (int t = 0; t < kTrials; ++t) {
            const uint32_t ds = mk_seed(0x400000u, t);
            auto tx = build_tx(ds, t);
            if (tx.n <= 0) {
                total_bits += 64;
                continue;
            }
            const TrialMetrics m =
                feed_raw_ext(ds, tx.I, tx.Q, tx.n, tx.info, kGuard + off);
            if (m.pass) {
                ++ok;
            } else if (m.crc_passed && m.length_correct) {
                ++crc_only;
            }
            total_bits += m.bit_errors;
        }
        char label[32], param[16];
        std::snprintf(label, sizeof(label), "Offset +%d chips", off);
        std::snprintf(param, sizeof(param), "+%d", off);
        row(label, ok, kTrials);
        record_ext("S4", param, ok, crc_only, kTrials, total_bits);
    }
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::print_sync_stats("S4");
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::print_row_stats("S4");
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::print_llr_stats("S4");
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::print_stats("S4");
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::print_stats_2d("S4");
#endif
}

// ═══════════════════════════════════════════════════════════════
//  S5: 극한 CFO 추종 (Hz 기반, 사전 보정 금지)
// ═══════════════════════════════════════════════════════════════
static void test_S5() {
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::reset_sync_stats();
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::reset_row_stats();
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::reset_llr_stats();
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::reset_stats();
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::reset_stats();
    ProtectedEngine::CWDetectDiag::set_expect_non_cw(true);
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::reset_stats_v2();
    ProtectedEngine::CWDetectDiag::set_scenario_label(
        ProtectedEngine::CWDetectDiag::CWDetectScenarioLabel::Clean);
#endif
    hdr("S5", "극한 CFO 추종 (블라인드)");
    const double cfos[] = {0, 50, 100, 200, 500, 1000, 2000, 5000};
    for (double cfo : cfos) {
        int ok = 0, crc_only = 0;
        long long total_bits = 0;
        for (int t = 0; t < kTrials; ++t) {
            const uint32_t ds = mk_seed(0x500000u, t);
            auto tx = build_tx(ds, t);
            if (tx.n <= 0) {
                total_bits += 64;
                continue;
            }
            int16_t rI[kMaxC], rQ[kMaxC];
            std::memcpy(rI, tx.I, sizeof(int16_t) * static_cast<size_t>(tx.n));
            std::memcpy(rQ, tx.Q, sizeof(int16_t) * static_cast<size_t>(tx.n));
            ch_cfo(rI, rQ, tx.n, cfo);
            const TrialMetrics m = feed_raw_ext(ds, rI, rQ, tx.n, tx.info);
            if (m.pass) {
                ++ok;
            } else if (m.crc_passed && m.length_correct) {
                ++crc_only;
            }
            total_bits += m.bit_errors;
        }
        char label[32], param[16];
        std::snprintf(label, sizeof(label), "CFO %+6.0f Hz", cfo);
        std::snprintf(param, sizeof(param), "%.0fHz", cfo);
        row(label, ok, kTrials);
        record_ext("S5", param, ok, crc_only, kTrials, total_bits);
    }
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::print_sync_stats("S5");
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::print_row_stats("S5");
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::print_llr_stats("S5");
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::print_stats("S5");
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::print_stats_2d("S5");
#endif
}

// ═══════════════════════════════════════════════════════════════
//  S6: 다중 경로 (3-tap)
// ═══════════════════════════════════════════════════════════════
static void test_S6() {
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::reset_sync_stats();
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::reset_row_stats();
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::reset_llr_stats();
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::reset_stats();
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::reset_stats();
    ProtectedEngine::CWDetectDiag::set_expect_non_cw(true);
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::reset_stats_v2();
    ProtectedEngine::CWDetectDiag::set_scenario_label(
        ProtectedEngine::CWDetectDiag::CWDetectScenarioLabel::Clean);
#endif
    hdr("S6", "다중 경로 (3-tap ISI)");
    struct MpCase { int d1; double a1; int d2; double a2; const char* label; };
    const MpCase cases[] = {
        { 1, 0.3,  3, 0.1,  "D=1/3 A=0.3/0.1"  },
        { 3, 0.5,  8, 0.2,  "D=3/8 A=0.5/0.2"  },
        { 5, 0.4, 15, 0.15, "D=5/15 A=0.4/0.15"},
        {10, 0.3, 32, 0.1,  "D=10/32 A=0.3/0.1"},
    };
    for (auto& mp : cases) {
        int ok = 0, crc_only = 0;
        long long total_bits = 0;
        for (int t = 0; t < kTrials; ++t) {
            const uint32_t ds = mk_seed(0x600000u, t);
            auto tx = build_tx(ds, t);
            if (tx.n <= 0) {
                total_bits += 64;
                continue;
            }
            int16_t rI[kMaxC], rQ[kMaxC];
            std::memcpy(rI, tx.I, sizeof(int16_t) * static_cast<size_t>(tx.n));
            std::memcpy(rQ, tx.Q, sizeof(int16_t) * static_cast<size_t>(tx.n));
            ch_multipath_3tap(rI, rQ, tx.n, mp.d1, mp.a1, mp.d2, mp.a2);
            const TrialMetrics m = feed_raw_ext(ds, rI, rQ, tx.n, tx.info);
            if (m.pass) {
                ++ok;
            } else if (m.crc_passed && m.length_correct) {
                ++crc_only;
            }
            total_bits += m.bit_errors;
        }
        row(mp.label, ok, kTrials);
        record_ext("S6", mp.label, ok, crc_only, kTrials, total_bits);
    }
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::print_sync_stats("S6");
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::print_row_stats("S6");
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::print_llr_stats("S6");
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::print_stats("S6");
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::print_stats_2d("S6");
#endif
}

// ═══════════════════════════════════════════════════════════════
//  S7: 바라지 재밍 (JSR +10 ~ +30 dB)
// ═══════════════════════════════════════════════════════════════
static void test_S7() {
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::reset_sync_stats();
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::reset_row_stats();
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::reset_llr_stats();
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::reset_stats();
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::reset_stats();
    ProtectedEngine::CWDetectDiag::set_expect_non_cw(true);
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::reset_stats_v2();
    ProtectedEngine::CWDetectDiag::set_scenario_label(
        ProtectedEngine::CWDetectDiag::CWDetectScenarioLabel::Barrage);
#endif
    hdr("S7", "바라지 재밍 (JSR +10 ~ +30 dB)");
    std::mt19937 rng(0x70000000u);
    const double jsrs[] = {0, 5, 10, 15, 20, 25, 30};
    for (double jsr : jsrs) {
        int ok = 0, crc_only = 0;
        long long total_bits = 0;
        for (int t = 0; t < kTrials; ++t) {
            const uint32_t ds = mk_seed(0x700000u, t);
            auto tx = build_tx(ds, t);
            if (tx.n <= 0) {
                total_bits += 64;
                continue;
            }
            int16_t rI[kMaxC], rQ[kMaxC];
            std::memcpy(rI, tx.I, sizeof(int16_t) * static_cast<size_t>(tx.n));
            std::memcpy(rQ, tx.Q, sizeof(int16_t) * static_cast<size_t>(tx.n));
            ch_barrage(rI, rQ, tx.n, jsr, rng);
            const TrialMetrics m = feed_raw_ext(ds, rI, rQ, tx.n, tx.info);
            if (m.pass) {
                ++ok;
            } else if (m.crc_passed && m.length_correct) {
                ++crc_only;
            }
            total_bits += m.bit_errors;
        }
        char label[32], param[16];
        std::snprintf(label, sizeof(label), "JSR %+5.0f dB", jsr);
        std::snprintf(param, sizeof(param), "%.0fdB", jsr);
        row(label, ok, kTrials);
        record_ext("S7", param, ok, crc_only, kTrials, total_bits);
    }
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::print_sync_stats("S7");
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::print_row_stats("S7");
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::print_llr_stats("S7");
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::print_stats("S7");
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::print_stats("S7");
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::print_stats_2d("S7");
#endif
}

// ═══════════════════════════════════════════════════════════════
//  S8: CW 톤 재밍
// ═══════════════════════════════════════════════════════════════
static void test_S8() {
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::reset_sync_stats();
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::reset_row_stats();
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::reset_llr_stats();
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::reset_stats();
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::reset_stats();
    ProtectedEngine::CWDetectDiag::set_expect_non_cw(false);
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::reset_stats_v2();
    ProtectedEngine::CWDetectDiag::set_scenario_label(
        ProtectedEngine::CWDetectDiag::CWDetectScenarioLabel::Cw);
#endif
    hdr("S8", "CW 톤 재밍");
    struct CwCase { double jsr; double period; const char* label; };
    const CwCase cases[] = {
        {10, 8.0,  "JSR+10 P=8"  },
        {15, 8.0,  "JSR+15 P=8"  },
        {20, 8.0,  "JSR+20 P=8"  },
        {15, 12.0, "JSR+15 P=12" },
        {20, 16.0, "JSR+20 P=16" },
    };
    for (auto& cw : cases) {
        int ok = 0, crc_only = 0;
        long long total_bits = 0;
        for (int t = 0; t < kTrials; ++t) {
            const uint32_t ds = mk_seed(0x800000u, t);
            auto tx = build_tx(ds, t);
            if (tx.n <= 0) {
                total_bits += 64;
                continue;
            }
            int16_t rI[kMaxC], rQ[kMaxC];
            std::memcpy(rI, tx.I, sizeof(int16_t) * static_cast<size_t>(tx.n));
            std::memcpy(rQ, tx.Q, sizeof(int16_t) * static_cast<size_t>(tx.n));
            ch_cw(rI, rQ, tx.n, cw.jsr, cw.period);
            const TrialMetrics m = feed_raw_ext(ds, rI, rQ, tx.n, tx.info);
            if (m.pass) {
                ++ok;
            } else if (m.crc_passed && m.length_correct) {
                ++crc_only;
            }
            total_bits += m.bit_errors;
        }
        row(cw.label, ok, kTrials);
        record_ext("S8", cw.label, ok, crc_only, kTrials, total_bits);
    }
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::print_sync_stats("S8");
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::print_row_stats("S8");
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::print_llr_stats("S8");
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::print_stats("S8");
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::print_stats("S8");
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::print_stats_2d("S8");
#endif
}

// ═══════════════════════════════════════════════════════════════
//  S9: 복합 스트레스 (타이밍 + 위상 + AWGN + CFO 동시)
// ═══════════════════════════════════════════════════════════════
static void test_S9() {
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::reset_sync_stats();
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::reset_row_stats();
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::reset_llr_stats();
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::reset_stats();
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::reset_stats();
    ProtectedEngine::CWDetectDiag::set_expect_non_cw(false);
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::reset_stats_v2();
    ProtectedEngine::CWDetectDiag::set_scenario_label(
        ProtectedEngine::CWDetectDiag::CWDetectScenarioLabel::Mixed);
#endif
    hdr("S9", "복합 스트레스 (135° + -15dB + CFO1500 + MP)");
    std::mt19937 rng(0x90000000u);
    int ok = 0, crc_only = 0;
    long long total_bits = 0;
    for (int t = 0; t < kTrials; ++t) {
        const uint32_t ds = mk_seed(0x900000u, t);
        auto tx = build_tx(ds, t);
        if (tx.n <= 0) {
            total_bits += 64;
            continue;
        }
        int16_t rI[kMaxC], rQ[kMaxC];
        std::memcpy(rI, tx.I, sizeof(int16_t) * static_cast<size_t>(tx.n));
        std::memcpy(rQ, tx.Q, sizeof(int16_t) * static_cast<size_t>(tx.n));

        ch_rotate(rI, rQ, tx.n, 135);
        ch_cfo(rI, rQ, tx.n, 1500.0);
        ch_multipath_3tap(rI, rQ, tx.n, 3, 0.2, 10, 0.1);
        ch_awgn(rI, rQ, tx.n, -15.0, rng);

        const int timing_off = static_cast<int>(rng() % 31u);
        const TrialMetrics m =
            feed_raw_ext(ds, rI, rQ, tx.n, tx.info, kGuard + timing_off);
        if (m.pass) {
            ++ok;
        } else if (m.crc_passed && m.length_correct) {
            ++crc_only;
        }
        total_bits += m.bit_errors;
    }
    row("Composite stress", ok, kTrials);
    record_ext("S9", "Full", ok, crc_only, kTrials, total_bits);
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::print_sync_stats("S9");
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::print_row_stats("S9");
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::print_llr_stats("S9");
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::print_stats("S9");
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::print_stats("S9");
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::print_stats_2d("S9");
#endif
}

// ═══════════════════════════════════════════════════════════════
//  S10: 내구도 + Holo LPI (10000회 → 메모리 릭 검증)
// ═══════════════════════════════════════════════════════════════
static void test_S10() {
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::reset_sync_stats();
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::reset_row_stats();
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::reset_llr_stats();
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::reset_stats();
#endif
#if defined(HTS_CW_DETECT_DIAG)
    ProtectedEngine::CWDetectDiag::reset_stats();
    ProtectedEngine::CWDetectDiag::set_expect_non_cw(true);
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::reset_stats_v2();
    ProtectedEngine::CWDetectDiag::set_scenario_label(
        ProtectedEngine::CWDetectDiag::CWDetectScenarioLabel::Clean);
#endif
    hdr("S10", "내구도 10000회 + LPI 정합");

    // Part A: 클린 내구도 10000회
    static constexpr int N_ENDURE = 10000;
    int ok_clean = 0, crc_only = 0;
    long long total_bits = 0;
    auto t0 = std::chrono::steady_clock::now();
    for (int t = 0; t < N_ENDURE; ++t) {
        const uint32_t ds = mk_seed(0xA00000u, t);
        auto tx = build_tx(ds, t);
        if (tx.n <= 0) {
            total_bits += 64;
            continue;
        }
        const TrialMetrics m = feed_raw_ext(ds, tx.I, tx.Q, tx.n, tx.info);
        if (m.pass) {
            ++ok_clean;
        } else if (m.crc_passed && m.length_correct) {
            ++crc_only;
        }
        total_bits += m.bit_errors;
    }
    auto t1 = std::chrono::steady_clock::now();
    double sec = std::chrono::duration<double>(t1 - t0).count();
    {
        char label[48];
        std::snprintf(label, sizeof(label), "Endurance %d (%.1fs)", N_ENDURE, sec);
        row(label, ok_clean, N_ENDURE);
        record_ext("S10a", "Endurance", ok_clean, crc_only, N_ENDURE, total_bits);
    }

    // Part B: Holo LPI ON/OFF 100회
    static constexpr int N_LPI = 100;
    int ok_off = 0, ok_on = 0;
    for (int t = 0; t < N_LPI; ++t) {
        const uint32_t ds = mk_seed(0xB00000u, t);
        const uint32_t lpi_seed[4] = {
            ds ^ 0xA5A5A5A5u, (ds << 1) ^ 0x3C3C3C3Cu,
            (ds >> 1) ^ 0x96969696u, ds ^ 0xDEADBEEFu
        };

        // OFF
        {
            auto tx = build_tx(ds, t);
            if (tx.n > 0 && feed_raw(ds, tx.I, tx.Q, tx.n, tx.info))
                ++ok_off;
        }

        // ON (TX LPI → RX LPI)
        {
            HTS_V400_Dispatcher tx_d;
            setup(tx_d, ds);
            tx_d.Enable_Holo_LPI(lpi_seed);
            uint8_t info[8]{};
            fill_info(ds, t, info);
            int16_t sI[kMaxC], sQ[kMaxC];
            int n = tx_d.Build_Packet(PayloadMode::DATA, info, 8, kAmp,
                                       sI, sQ, kMaxC);
            tx_d.Disable_Holo_LPI();
            if (n > 0 && feed_raw_lpi(ds, sI, sQ, n, info, lpi_seed))
                ++ok_on;
        }
    }
    row("LPI OFF", ok_off, N_LPI);
    row("LPI ON",  ok_on,  N_LPI);
    record("S10b", "LPI-OFF", ok_off, N_LPI);
    record("S10c", "LPI-ON",  ok_on,  N_LPI);
#if defined(HTS_SYNC_DIAG)
    ProtectedEngine::SyncDiag::print_sync_stats("S10");
#endif
#if defined(HTS_WALSH_ROW_DIAG)
    ProtectedEngine::WalshRowDiag::print_row_stats("S10");
#endif
#if defined(HTS_LLR_DIAG)
    ProtectedEngine::LLRDiag::print_llr_stats("S10");
#endif
#if defined(HTS_ROW_CONSISTENCY_DIAG)
    ProtectedEngine::RowConsistencyDiag::print_stats("S10");
#endif
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::print_stats_2d("S10");
#endif
}

} // namespace

// ═══════════════════════════════════════════════════════════════
//  main
// ═══════════════════════════════════════════════════════════════
int main() {
    ProtectedEngine::WRC::reset_diag();
#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::reset_joint_label_accum();
#endif
#if defined(HTS_HARQ_DIAG)
    ProtectedEngine::HARQ_Diag::reset_retry_stats();
#endif
#if defined(HTS_AMP_DIAG)
    ProtectedEngine::AmpDiag::reset_amp_stats();
#endif

    std::printf("╔═══════════════════════════════════════════════════╗\n");
    std::printf("║  HTS T6-SIM V3: 무결 채널 시뮬레이션              ║\n");
    std::printf("║  사전 보정 금지 | memcmp 강제 | 음수 SNR 필수     ║\n");
    std::printf("║  FAIL = 엔진 실제 한계 (꼼수 제로)               ║\n");
    std::printf("╚═══════════════════════════════════════════════════╝\n");

    auto t_all = std::chrono::steady_clock::now();

    test_S1();
    test_S2();
    test_S3();
    test_S4();
    test_S5();
    test_S6();
    test_S7();
    test_S8();
    test_S9();
    test_S10();

#if defined(HTS_CW_DETECT_DIAG_V2)
    ProtectedEngine::CWDetectDiag::print_joint_by_label_summary();
#endif

    auto t_end = std::chrono::steady_clock::now();
    double total_sec = std::chrono::duration<double>(t_end - t_all).count();

    // ══ 종합 보고서 ══
    std::printf("\n╔═══════════════════════════════════════════════════════════════╗\n");
    std::printf("║  종합 보고서 (Pass + CRC+payload BER)                         ║\n");
    std::printf("╠═════════╤══════════════════╤═══════╤═══════╤════════╤═══════╣\n");
    std::printf("║ 시나리오│ 조건             │ Pass  │ CRC+  │  BER   │ 판정  ║\n");
    std::printf("╠═════════╪══════════════════╪═══════╪═══════╪════════╪═══════╣\n");

    int grand_pass = 0, grand_total = 0;
    long long grand_bits = 0;
    int cat_pass = 0, cat_total = 0;
    for (int i = 0; i < g_n_results; ++i) {
        auto& r = g_results[i];
        const char* tag = (r.pass == r.total) ? "PASS " :
                          (r.pass == 0)       ? "FAIL " : "PART ";
        const double ber = (r.total > 0)
            ? static_cast<double>(r.total_bit_errors) /
              (static_cast<double>(r.total) * 64.0)
            : 0.0;
        std::printf("║ %-7s │ %-16s │ %5d │ %5d │ %6.4f │ %s ║\n",
                    r.name, r.param, r.pass, r.crc_only_pass, ber, tag);
        grand_pass += r.pass;
        grand_total += r.total;
        grand_bits += r.total_bit_errors;
        ++cat_total;
        if (r.pass == r.total) ++cat_pass;
    }

    std::printf("╠═════════╧══════════════════╧═══════╧═══════╧════════╧═══════╣\n");
    const double grand_ber = (grand_total > 0)
        ? static_cast<double>(grand_bits) /
          (static_cast<double>(grand_total) * 64.0)
        : 0.0;
    std::printf("║  정량 합계: %6d / %6d (%5.1f%%) — 전체 BER: %7.5f        ║\n",
                grand_pass, grand_total,
                (grand_total > 0) ? 100.0 * grand_pass / grand_total : 0.0,
                grand_ber);
    std::printf("║  범주 PASS: %2d / %2d                                           ║\n",
                cat_pass, cat_total);
    std::printf("║  총 bit errors: %lld / %lld                                   ║\n",
                static_cast<long long>(grand_bits),
                static_cast<long long>(grand_total) * 64LL);
    std::printf("║  총 소요:   %.1fs                                              ║\n",
                total_sec);
    std::printf("╚═══════════════════════════════════════════════════════════════╝\n");

    // FAIL 시나리오 목록
    bool any_fail = false;
    for (int i = 0; i < g_n_results; ++i) {
        if (g_results[i].pass < g_results[i].total) {
            if (!any_fail) {
                std::printf("\n  [엔진 한계 — 개선 필요 항목]\n");
                any_fail = true;
            }
            const double fail_ber = (g_results[i].total > 0)
                ? static_cast<double>(g_results[i].total_bit_errors) /
                  (static_cast<double>(g_results[i].total) * 64.0)
                : 0.0;
            std::printf("    %-8s %-16s %3d/%3d  CRC+%3d  BER=%.4f\n",
                        g_results[i].name, g_results[i].param,
                        g_results[i].pass, g_results[i].total,
                        g_results[i].crc_only_pass, fail_ber);
        }
    }
    if (!any_fail) {
        std::printf("\n  전 시나리오 PASS — 한계 미발견\n");
    }

    ProtectedEngine::WRC::print_diag();
#if defined(HTS_HARQ_DIAG)
    ProtectedEngine::HARQ_Diag::print_retry_stats();
#endif
#if defined(HTS_AMP_DIAG)
    ProtectedEngine::AmpDiag::print_amp_stats();
#endif

    return 0;
}

// ── 소스 링크 (단일 TU 빌드) ──
#include "../../HTS_LIM/HTS_Secure_Memory.cpp"
#include "../../HTS_LIM/HTS_Polar_Codec.cpp"
#include "../../HTS_LIM/HTS_FEC_HARQ.cpp"
#include "../../HTS_LIM/HTS_Holo_LPI.cpp"
#include "../../HTS_LIM/HTS_Walsh_Row_Permuter.cpp"
#include "../../HTS_LIM/HTS_V400_Dispatcher.cpp"
#if defined(HTS_HARQ_DIAG)
#include "../../HTS_LIM/HTS_HARQ_Diag.cpp"
#endif
#if defined(HTS_AMP_DIAG)
#include "../../HTS_LIM/HTS_Amp_Diag.cpp"
#endif

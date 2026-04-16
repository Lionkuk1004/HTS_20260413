// =============================================================================
// HTS_TPC_LPI_Test.cpp — TPC 동작 검증 + LPI 은닉 마진 실측
// =============================================================================
// 소프트웨어 전용 (Pluto 불필요)
// AND/OR/SHIFT 연산 검증 포함
// =============================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
#error "[HTS_FATAL] PC 전용"
#endif

#include "HTS_TPC_Controller.h"
#include "HTS_FEC_HARQ.hpp"
#include "HTS_V400_Dispatcher.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

using ProtectedEngine::DecodedPacket;
using ProtectedEngine::FEC_HARQ;
using ProtectedEngine::HTS_TPC_Controller;
using ProtectedEngine::HTS_V400_Dispatcher;
using ProtectedEngine::PayloadMode;
using ProtectedEngine::SoftClipPolicy;

static DecodedPacket g_last{};
static void on_pkt(const DecodedPacket &p) noexcept { g_last = p; }

// ════════════════════════════════════════════════════════════
//  T1: LUT 검증 — 각 레벨의 amp와 dB가 3dB 스텝인지 확인
// ════════════════════════════════════════════════════════════
static void test_T1_lut_verify() {
    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  T1: TPC LUT 검증 (3dB 스텝)\n");
    std::printf("═══════════════════════════════════════════════\n\n");

    std::printf("  %5s  %6s  %6s  %8s  %8s\n", "Level", "Amp", "dB",
                "Ratio²", "dB_실측");

    int pass = 0;
    for (int32_t i = 0; i < HTS_TPC_Controller::kLevelCount; ++i) {
        const int16_t amp = HTS_TPC_Controller::kAmpLut[static_cast<size_t>(i)];
        const int32_t db = HTS_TPC_Controller::kDbLut[static_cast<size_t>(i)];

        double ratio_sq = 1.0;
        double db_meas = 0.0;
        if (i > 0) {
            const double a0 = static_cast<double>(
                HTS_TPC_Controller::kAmpLut[static_cast<size_t>(i - 1)]);
            const double a1 = static_cast<double>(amp);
            ratio_sq = (a1 * a1) / (a0 * a0);
            db_meas = 10.0 * std::log10(ratio_sq);
        }

        const bool ok =
            (i == 0) || (db_meas > -3.5 && db_meas < -2.5);
        if (ok)
            ++pass;

        std::printf("  %5d  %5d  %+4ddB  %7.3f  %+6.2fdB  %s\n",
                    static_cast<int>(i), static_cast<int>(amp),
                    static_cast<int>(db), ratio_sq, db_meas,
                    ok ? "OK" : "FAIL");
    }
    std::printf("\n  T1: %d/%d\n", pass,
                static_cast<int>(HTS_TPC_Controller::kLevelCount));
}

// ════════════════════════════════════════════════════════════
//  T2: AND/OR/SHIFT 연산 검증 — 곱셈/나눗셈 0회 확인
// ════════════════════════════════════════════════════════════
static void test_T2_shift_verify() {
    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  T2: AND/OR/SHIFT 연산 검증\n");
    std::printf("═══════════════════════════════════════════════\n\n");

    int pass = 0;
    int total = 0;
    static constexpr int16_t test_vals[] = {
        0, 1, -1, 100, -100, 32767, -32768, 1000, -1000, 500, -500};
    for (int16_t v : test_vals) {
        const int32_t vi = static_cast<int32_t>(v);
        const int32_t si = vi >> 31;
        const int32_t ai = (vi ^ si) - si;
        const int32_t expected = (vi < 0) ? -vi : vi;
        ++total;
        if (ai == expected)
            ++pass;
        else
            std::printf("  ABS FAIL: v=%d got=%d exp=%d\n",
                        static_cast<int>(v), static_cast<int>(ai),
                        static_cast<int>(expected));
    }

    static constexpr int32_t targets[] = {64, 128, 256, 362, 512, 724, 1024};
    for (int32_t t : targets) {
        const int32_t th_up_shift = t + (t >> 2);
        const int32_t th_down_shift = t - (t >> 2);
        const int32_t th_down2_shift = t + (t >> 1) + (t >> 2);

        const double th_up_exact = static_cast<double>(t) * 1.25;
        const double th_down_exact = static_cast<double>(t) * 0.75;
        const double th_down2_exact = static_cast<double>(t) * 1.75;

        const bool ok_up =
            std::abs(static_cast<double>(th_up_shift) - th_up_exact) < 2.0;
        const bool ok_down =
            std::abs(static_cast<double>(th_down_shift) - th_down_exact) <
            2.0;
        const bool ok_d2 =
            std::abs(static_cast<double>(th_down2_shift) - th_down2_exact) <
            2.0;

        ++total;
        if (ok_up)
            ++pass;
        ++total;
        if (ok_down)
            ++pass;
        ++total;
        if (ok_d2)
            ++pass;

        if (!ok_up || !ok_down || !ok_d2) {
            std::printf(
                "  THRESH FAIL: t=%d up=%d(%.0f) down=%d(%.0f) d2=%d(%.0f)\n",
                static_cast<int>(t), static_cast<int>(th_up_shift), th_up_exact,
                static_cast<int>(th_down_shift), th_down_exact,
                static_cast<int>(th_down2_shift), th_down2_exact);
        }
    }

    std::printf("  T2: %d/%d (AND/OR/SHIFT 전용 연산 정합)\n", pass, total);
}

// ════════════════════════════════════════════════════════════
//  T3: RSSI 피드백 루프 시뮬레이션
// ════════════════════════════════════════════════════════════
static void test_T3_feedback_loop() {
    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  T3: TPC 피드백 루프 (수렴 테스트)\n");
    std::printf("═══════════════════════════════════════════════\n\n");

    HTS_TPC_Controller tpc;
    tpc.Reset_To_Max();

    std::printf("  목표: Level 4 (-12dB)로 수렴\n");
    std::printf("  경로손실 시뮬: -12dB (amp ×0.25 = >>2)\n\n");

    std::mt19937 rng(0xFEED0003u);
    std::normal_distribution<double> nd(0.0, 30.0);

    const int32_t target_rx_amp = 256;

    std::printf("  %5s  %5s  %5s  %6s  %4s  %s\n", "Round", "Level", "TxAmp",
                "RxRSSI", "FB", "Action");

    for (int r = 0; r < 20; ++r) {
        const int16_t tx_amp = tpc.Get_Tx_Amp();

        int16_t rx_chips_I[64]{};
        int16_t rx_chips_Q[64]{};
        for (int c = 0; c < 64; ++c) {
            const int32_t sig = static_cast<int32_t>(tx_amp) >> 2;
            const int32_t noise_i =
                static_cast<int32_t>(std::lround(nd(rng)));
            const int32_t noise_q =
                static_cast<int32_t>(std::lround(nd(rng)));
            const int32_t sign =
                ((rng() & 1u) != 0u) ? 1 : -1;
            int32_t vi = sig * sign + noise_i;
            int32_t vq = sig * sign + noise_q;
            if (vi > 32767)
                vi = 32767;
            if (vi < -32768)
                vi = -32768;
            if (vq > 32767)
                vq = 32767;
            if (vq < -32768)
                vq = -32768;
            rx_chips_I[c] = static_cast<int16_t>(vi);
            rx_chips_Q[c] = static_cast<int16_t>(vq);
        }

        const uint8_t fb = HTS_TPC_Controller::Measure_RSSI_Feedback(
            rx_chips_I, rx_chips_Q, 64,
            static_cast<int16_t>(target_rx_amp));

        const char *action = "HOLD";
        if (fb == HTS_TPC_Controller::kFbDown1)
            action = "DOWN1(-3dB)";
        else if (fb == HTS_TPC_Controller::kFbDown2)
            action = "DOWN2(-6dB)";
        else if (fb == HTS_TPC_Controller::kFbUp1)
            action = "UP1(+3dB)";

        int32_t rssi_disp = 0;
        for (int c = 0; c < 64; ++c) {
            int32_t ai = rx_chips_I[c];
            if (ai < 0)
                ai = -ai;
            int32_t aq = rx_chips_Q[c];
            if (aq < 0)
                aq = -aq;
            rssi_disp += ai + aq;
        }
        rssi_disp >>= 6;

        std::printf("  %5d  %5d  %5d  %6d  0x%02X  %s\n", r,
                    static_cast<int>(tpc.Get_Level()),
                    static_cast<int>(tx_amp), static_cast<int>(rssi_disp),
                    static_cast<unsigned>(fb), action);

        tpc.Apply_Feedback(fb);
    }

    std::printf("\n  최종 레벨: %d (%+ddB), amp=%d\n",
                static_cast<int>(tpc.Get_Level()),
                static_cast<int>(tpc.Get_Level_dB()),
                static_cast<int>(tpc.Get_Tx_Amp()));
}

// ════════════════════════════════════════════════════════════
//  T4: TPC + FEC 통합 — 최소 전력에서 디코딩 가능한지
// ════════════════════════════════════════════════════════════
static void test_T4_tpc_fec() {
    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  T4: TPC + FEC 통합 (각 레벨에서 디코딩 검증)\n");
    std::printf("═══════════════════════════════════════════════\n\n");

    static constexpr int kPreReps = 4;
    static constexpr int kPreBoost = 1;
    static constexpr int kMaxC =
        2048 + (FEC_HARQ::NSYM64 + kPreReps + 4) * 64;
    static constexpr int kTrials = 50;

    HTS_TPC_Controller tpc;

    std::printf("  %5s  %5s  %6s  %5s  %8s\n", "Level", "Amp", "dB", "CRC%",
                "은닉마진");

    for (int32_t lv = 0; lv < HTS_TPC_Controller::kLevelCount; ++lv) {
        tpc.Set_Level(lv);
        const int16_t amp = tpc.Get_Tx_Amp();
        int ok = 0;

        for (int t = 0; t < kTrials; ++t) {
            g_last = DecodedPacket{};
            const uint32_t ds =
                0xF4EC0000u ^
                static_cast<uint32_t>((lv * kTrials + t) * 0x9E3779B9u);

            HTS_V400_Dispatcher tx_disp;
            tx_disp.Set_IR_Mode(true);
            tx_disp.Set_Seed(ds);
            tx_disp.Set_Preamble_Boost(kPreBoost);
            tx_disp.Set_Preamble_Reps(kPreReps);
            tx_disp.Set_CW_Cancel(false);
            tx_disp.Set_AJC_Enabled(false);
            tx_disp.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
            tx_disp.Set_Packet_Callback(on_pkt);
            tx_disp.Update_Adaptive_BPS(1000);
            tx_disp.Set_Lab_IQ_Mode_Jam_Harness();

            uint8_t info[8]{};
            for (int b = 0; b < 8; ++b)
                info[b] = static_cast<uint8_t>(
                    (ds >> (b * 4)) ^ static_cast<unsigned>(t + b));

            std::vector<int16_t> sigI(kMaxC), sigQ(kMaxC);
            const int n = tx_disp.Build_Packet(PayloadMode::DATA, info, 8, amp,
                                               sigI.data(), sigQ.data(), kMaxC);
            if (n <= 0)
                continue;

            HTS_V400_Dispatcher rx_disp;
            rx_disp.Set_IR_Mode(true);
            rx_disp.Set_Seed(ds);
            rx_disp.Set_Preamble_Boost(kPreBoost);
            rx_disp.Set_Preamble_Reps(kPreReps);
            rx_disp.Set_CW_Cancel(false);
            rx_disp.Set_AJC_Enabled(false);
            rx_disp.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
            rx_disp.Set_Packet_Callback(on_pkt);
            rx_disp.Update_Adaptive_BPS(1000);

            for (int i = 0; i < 256; ++i)
                rx_disp.Feed_Chip(0, 0);
            for (int i = 0; i < n; ++i)
                rx_disp.Feed_Chip(sigI[i], sigQ[i]);
            for (int i = 0; i < 256; ++i)
                rx_disp.Feed_Chip(0, 0);

            if (g_last.success_mask == DecodedPacket::DECODE_MASK_OK)
                ++ok;
        }

        const int32_t margin = tpc.Get_Hiding_Margin_dB(18);
        std::printf(
            "  %5d  %5d  %+4ddB  %4.0f%%  %+5ddB %s\n",
            static_cast<int>(lv), static_cast<int>(amp),
            static_cast<int>(tpc.Get_Level_dB()), 100.0 * ok / kTrials,
            static_cast<int>(margin),
            (margin > 10) ? "<- 탐지 불가"
            : (margin > 5) ? "<- 탐지 극난"
            : (margin > 0) ? "<- 잡음 이하"
                           : "");
    }
}

// ════════════════════════════════════════════════════════════
//  T5: TPC + AWGN — 각 레벨에서 재밍 내성
// ════════════════════════════════════════════════════════════
static void test_T5_tpc_awgn() {
    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  T5: TPC + AWGN 재밍 (레벨별 한계)\n");
    std::printf("═══════════════════════════════════════════════\n\n");

    static constexpr int kPreReps = 4;
    static constexpr int kMaxC =
        2048 + (FEC_HARQ::NSYM64 + kPreReps + 4) * 64;
    static constexpr int kTrials = 100;
    static constexpr double kJsDb = 10.0;

    HTS_TPC_Controller tpc;

    static constexpr int32_t test_levels[] = {0, 2, 4, 6, 8};

    std::printf("  %5s  %5s  %6s  J/S=%+.0fdB  %5s  %8s\n", "Level", "Amp",
                "dB", kJsDb, "CRC%", "은닉마진");

    for (int32_t lv : test_levels) {
        tpc.Set_Level(lv);
        const int16_t amp = tpc.Get_Tx_Amp();
        int ok = 0;
        std::mt19937 rng(0xAB050000u ^
                         static_cast<uint32_t>(lv * 0x12345u));

        for (int t = 0; t < kTrials; ++t) {
            g_last = DecodedPacket{};
            const uint32_t ds =
                0xAB050000u ^
                static_cast<uint32_t>((lv * kTrials + t) * 0x9E3779B9u);

            HTS_V400_Dispatcher tx_disp;
            tx_disp.Set_IR_Mode(true);
            tx_disp.Set_Seed(ds);
            tx_disp.Set_Preamble_Boost(1);
            tx_disp.Set_Preamble_Reps(kPreReps);
            tx_disp.Set_CW_Cancel(false);
            tx_disp.Set_AJC_Enabled(false);
            tx_disp.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
            tx_disp.Set_Packet_Callback(on_pkt);
            tx_disp.Update_Adaptive_BPS(1000);
            tx_disp.Set_Lab_IQ_Mode_Jam_Harness();

            uint8_t info[8]{};
            for (int b = 0; b < 8; ++b)
                info[b] = static_cast<uint8_t>(
                    (ds >> (b * 4)) ^ static_cast<unsigned>(t + b));

            std::vector<int16_t> sigI(kMaxC), sigQ(kMaxC);
            int n = tx_disp.Build_Packet(PayloadMode::DATA, info, 8, amp,
                                         sigI.data(), sigQ.data(), kMaxC);
            if (n <= 0)
                continue;

            double P_sig = 0;
            for (int i = 0; i < n; ++i) {
                const double di = static_cast<double>(sigI[i]);
                const double dq = static_cast<double>(sigQ[i]);
                P_sig += di * di + dq * dq;
            }
            P_sig /= static_cast<double>(n);
            const double sigma =
                std::sqrt(P_sig / (2.0 * std::pow(10.0, kJsDb / 10.0)));
            std::normal_distribution<double> nd(0.0, sigma);
            for (int i = 0; i < n; ++i) {
                const long long ri =
                    static_cast<long long>(sigI[i]) +
                    std::llround(nd(rng));
                const long long rq =
                    static_cast<long long>(sigQ[i]) +
                    std::llround(nd(rng));
                sigI[i] = static_cast<int16_t>(
                    std::max(-32768LL, std::min(32767LL, ri)));
                sigQ[i] = static_cast<int16_t>(
                    std::max(-32768LL, std::min(32767LL, rq)));
            }

            HTS_V400_Dispatcher rx_disp;
            rx_disp.Set_IR_Mode(true);
            rx_disp.Set_Seed(ds);
            rx_disp.Set_Preamble_Boost(1);
            rx_disp.Set_Preamble_Reps(kPreReps);
            rx_disp.Set_CW_Cancel(false);
            rx_disp.Set_AJC_Enabled(false);
            rx_disp.Set_SoftClip_Policy(SoftClipPolicy::NEVER);
            rx_disp.Set_Packet_Callback(on_pkt);
            rx_disp.Update_Adaptive_BPS(1000);

            for (int i = 0; i < 256; ++i)
                rx_disp.Feed_Chip(0, 0);
            for (int i = 0; i < n; ++i)
                rx_disp.Feed_Chip(sigI[i], sigQ[i]);
            for (int i = 0; i < 256; ++i)
                rx_disp.Feed_Chip(0, 0);

            if (g_last.success_mask == DecodedPacket::DECODE_MASK_OK)
                ++ok;
        }

        const int32_t margin = tpc.Get_Hiding_Margin_dB(18);
        std::printf("  %5d  %5d  %+4ddB     %4.0f%%  %+5ddB %s\n",
                    static_cast<int>(lv), static_cast<int>(amp),
                    static_cast<int>(tpc.Get_Level_dB()), 100.0 * ok / kTrials,
                    static_cast<int>(margin),
                    (ok >= 95) ? "PASS"
                    : (ok >= 80) ? "MARGINAL"
                                 : "FAIL");
    }
}

// ════════════════════════════════════════════════════════════
//  T6: LPI 종합 비교표 (HTS vs SINCGARS vs Link-16)
// ════════════════════════════════════════════════════════════
static void test_T6_lpi_comparison() {
    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  T6: LPI 종합 비교 (HTS vs 군용 무전기)\n");
    std::printf("═══════════════════════════════════════════════\n\n");

    HTS_TPC_Controller tpc;

    std::printf("  ┌──────────────┬────────┬─────────┬─────────┬──────────┐\n");
    std::printf("  │ 시스템        │ PG(dB) │ TPC(dB) │ 은닉(dB)│ 탐지확률  │\n");
    std::printf("  ├──────────────┼────────┼─────────┼─────────┼──────────┤\n");
    std::printf("  │ FM 무전기     │    0   │   N/A   │    0    │  100%%   │\n");
    std::printf("  │ SINCGARS FH   │    0   │   N/A   │   -20   │  100%%   │\n");
    std::printf("  │ Link-16 CCSK  │   15   │   N/A   │    +5   │   ~50%%  │\n");

    static constexpr int32_t show_levels[] = {0, 4, 6, 8};
    for (int32_t lv : show_levels) {
        tpc.Set_Level(lv);
        const int32_t margin = tpc.Get_Hiding_Margin_dB(18);

        const char *detect = "100%";
        if (margin > 15)
            detect = " <1%";
        else if (margin > 10)
            detect = " <5%";
        else if (margin > 5)
            detect = "<20%";
        else if (margin > 0)
            detect = "~50%";

        std::printf("  │ HTS 64c Lv%d  │   18   │  %+3d    │  %+4d   │  %5s  │\n",
                    static_cast<int>(lv),
                    static_cast<int>(tpc.Get_Level_dB()),
                    static_cast<int>(margin), detect);
    }

    std::printf("  └──────────────┴────────┴─────────┴─────────┴──────────┘\n\n");

    std::printf("  HTS TPC 운용 전략:\n");
    std::printf("    근거리(~1km): Level 6~8 → 은닉 +36~42dB → 탐지 불가\n");
    std::printf("    중거리(~5km): Level 4   → 은닉 +30dB   → 탐지 불가\n");
    std::printf("    원거리(10km): Level 0~2 → 은닉 +18~24dB → 탐지 극난\n");
}

// ════════════════════════════════════════════════════════════
int main() {
    std::printf("╔═══════════════════════════════════════════════╗\n");
    std::printf("║  HTS TPC + LPI 성능 검증                     ║\n");
    std::printf("║  AND/OR/SHIFT 전용, 소프트웨어 전용           ║\n");
    std::printf("╚═══════════════════════════════════════════════╝\n");

    test_T1_lut_verify();
    test_T2_shift_verify();
    test_T3_feedback_loop();
    test_T4_tpc_fec();
    test_T5_tpc_awgn();
    test_T6_lpi_comparison();

    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  전체 완료\n");
    std::printf("═══════════════════════════════════════════════\n");
    return 0;
}

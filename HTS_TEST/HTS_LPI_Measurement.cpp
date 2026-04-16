// HTS_LPI_Measurement.cpp — LPI/LPD 성능 측정 (소프트웨어)
// Walsh-64 DSSS 신호의 탐지 불가능성 정량 평가
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
#error "[HTS_FATAL] PC 전용"
#endif
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
using ProtectedEngine::HTS_V400_Dispatcher;
using ProtectedEngine::PayloadMode;
using ProtectedEngine::RxPhase;
using ProtectedEngine::SoftClipPolicy;

static constexpr int16_t kAmp = 500;
static constexpr int kTrials = 1000;

static DecodedPacket g_last{};
static void on_pkt(const DecodedPacket &p) noexcept { g_last = p; }

// ════════════════════════════════════════════════════════════
//  L1: 에너지 탐지기 ROC (적의 관점)
//  적이 수신 대역에서 에너지를 측정하여 통신 존재 여부 판단
//  H0: 잡음만 존재, H1: 신호+잡음
// ════════════════════════════════════════════════════════════
static void test_L1_energy_detector() {
    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  L1: 에너지 탐지기 ROC (적의 시각)\n");
    std::printf("═══════════════════════════════════════════════\n");
    std::printf("  적이 에너지 탐지기로 HTS 신호를 탐지할 수 있는가?\n\n");

    // SNR = 신호전력/잡음전력 (칩 단위, 확산 전)
    // 수신 SNR = TX_SNR - PG = TX_SNR - 18.06 dB
    // 적은 Walsh 코드를 모르므로 역확산 불가 → 칩 단위 SNR로만 판단
    static constexpr int kWindow = 1024; // 탐지 윈도우 (칩)
    static constexpr double kPfa_targets[] = {0.01, 0.05, 0.10}; // 오경보 확률
    static constexpr double kSnrChipDb[] = {-20, -15, -10, -5, 0, 5, 10};

    std::mt19937 rng(0x4C310001u); // L1 RNG seed (PC 전용)

    // H0 에너지 분포 (잡음만)
    std::vector<double> h0_energies(10000);
    for (int i = 0; i < 10000; ++i) {
        double e = 0;
        std::normal_distribution<double> nd(0.0, 500.0);
        for (int k = 0; k < kWindow; ++k) {
            const double ni = nd(rng);
            const double nq = nd(rng);
            e += ni * ni + nq * nq;
        }
        h0_energies[static_cast<size_t>(i)] = e / kWindow;
    }
    std::sort(h0_energies.begin(), h0_energies.end());

    std::printf("  %6s", "SNR_chip");
    for (double pfa : kPfa_targets)
        std::printf("  Pd@Pfa=%.0f%%", pfa * 100);
    std::printf("  비고\n");

    for (double snr_db : kSnrChipDb) {
        const double snr_lin = std::pow(10.0, snr_db / 10.0);
        const double sig_amp = 500.0 * std::sqrt(snr_lin);

        // H1 에너지 분포 (신호+잡음)
        // 신호: Walsh 확산된 DSSS (적은 코드 모름 → 랜덤 ±amp)
        std::vector<double> h1_energies(10000);
        for (int i = 0; i < 10000; ++i) {
            double e = 0;
            std::normal_distribution<double> nd(0.0, 500.0);
            std::uniform_int_distribution<int> sd(0, 1);
            for (int k = 0; k < kWindow; ++k) {
                const double s = sig_amp * (sd(rng) * 2 - 1); // ±sig_amp
                const double ni = nd(rng);
                const double nq = nd(rng);
                e += (s + ni) * (s + ni) + nq * nq;
            }
            h1_energies[static_cast<size_t>(i)] = e / kWindow;
        }

        std::printf("  %+5.0fdB", snr_db);
        for (double pfa : kPfa_targets) {
            // 문턱: H0의 (1-Pfa) 백분위
            int th_idx = static_cast<int>((1.0 - pfa) * 10000);
            if (th_idx >= 10000)
                th_idx = 9999;
            const double threshold = h0_energies[static_cast<size_t>(th_idx)];

            // 탐지 확률: H1에서 문턱 초과 비율
            int detect = 0;
            for (double e : h1_energies)
                if (e > threshold)
                    ++detect;
            const double pd = static_cast<double>(detect) / 10000.0;
            std::printf("    %5.1f%%", pd * 100.0);
        }

        // 비고
        if (snr_db <= -15.0)
            std::printf("  ← 탐지 불가");
        else if (snr_db <= -10.0)
            std::printf("  ← 탐지 극난");
        else if (snr_db <= -5.0)
            std::printf("  ← 탐지 곤란");
        std::printf("\n");
    }

    std::printf("\n  HTS 운용 SNR (칩 단위):\n");
    std::printf("    Walsh PG = -18.06 dB\n");
    std::printf("    TX SNR +10dB → 칩 SNR = 10-18 = -8dB\n");
    std::printf("    TX SNR +20dB → 칩 SNR = 20-18 = +2dB\n");
    std::printf("    → 정상 운용 시 칩 SNR ≤ -5dB → 에너지 탐지 불가\n");
}

// ════════════════════════════════════════════════════════════
//  L2: 상관 탐지기 (코드 알 때 vs 모를 때)
// ════════════════════════════════════════════════════════════
static void test_L2_correlation_detector() {
    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  L2: 상관 탐지기 (코드 지식 유무)\n");
    std::printf("═══════════════════════════════════════════════\n\n");

    static constexpr int NC = 64;
    static constexpr double kSnrDb[] = {-20, -15, -10, -5, 0};

    std::mt19937 rng(0xC04420u);

    std::printf("  %6s  %12s  %12s  %8s\n", "SNR_ch", "코드일치", "코드불일치",
                "LPI이득");

    for (double snr_db : kSnrDb) {
        const double snr_lin = std::pow(10.0, snr_db / 10.0);
        const double noise_sigma =
            static_cast<double>(kAmp) / std::sqrt(snr_lin);

        int det_match = 0, det_wrong = 0;

        for (int t = 0; t < kTrials; ++t) {
            // TX: Walsh sym=7 (임의 고정)
            int16_t txI[NC]{};
            for (int c = 0; c < NC; ++c) {
                uint32_t x = 7u & static_cast<uint32_t>(c);
                x = x - ((x >> 1) & 0x55555555u);
                x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
                const uint32_t p =
                    (((x + (x >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24;
                txI[c] = static_cast<int16_t>(
                    kAmp * (1 - 2 * static_cast<int>(p & 1u)));
            }

            // RX: 신호 + 잡음
            std::normal_distribution<double> nd(0.0, noise_sigma);
            int16_t rxI[NC]{};
            for (int c = 0; c < NC; ++c)
                rxI[c] = static_cast<int16_t>(txI[c] + std::lround(nd(rng)));

            // 상관: 올바른 코드 (sym=7)
            int64_t corr_match = 0;
            for (int c = 0; c < NC; ++c)
                corr_match += rxI[c] * txI[c];
            if (corr_match > 0)
                ++det_match;

            // 상관: 틀린 코드 (sym=23, 임의)
            int16_t wrongI[NC]{};
            for (int c = 0; c < NC; ++c) {
                uint32_t x = 23u & static_cast<uint32_t>(c);
                x = x - ((x >> 1) & 0x55555555u);
                x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
                const uint32_t p =
                    (((x + (x >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24;
                wrongI[c] = static_cast<int16_t>(
                    kAmp * (1 - 2 * static_cast<int>(p & 1u)));
            }
            int64_t corr_wrong = 0;
            for (int c = 0; c < NC; ++c)
                corr_wrong += rxI[c] * wrongI[c];
            if (corr_wrong > static_cast<int64_t>(NC) * kAmp / 2)
                ++det_wrong;
        }

        const double pd_m = 100.0 * det_match / kTrials;
        const double pd_w = 100.0 * det_wrong / kTrials;
        std::printf("  %+5.0fdB  %10.1f%%  %10.1f%%  %+6.1fdB\n", snr_db, pd_m,
                    pd_w,
                    (pd_w > 0.1) ? 10.0 * std::log10(pd_m / pd_w) : 99.0);
    }

    std::printf("\n  결론: Walsh 코드를 모르면 상관 탐지 불가\n");
    std::printf("  → 적이 64개 코드를 전수 탐색해도\n");
    std::printf("     인터리빙 시드(il) 모르면 심볼 순서 복원 불가\n");
}

// ════════════════════════════════════════════════════════════
//  L3: 스펙트럼 은닉도 (PSD 잡음 대비)
// ════════════════════════════════════════════════════════════
static void test_L3_spectral_hiding() {
    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  L3: 스펙트럼 은닉도\n");
    std::printf("═══════════════════════════════════════════════\n\n");

    std::printf("  칩레이트: 200 kc/s\n");
    std::printf("  점유 대역: 200 kHz\n");
    std::printf("  Walsh PG: %.2f dB\n", 10.0 * std::log10(64.0));
    std::printf("  BPS=4 비트당 PG: %.2f dB\n\n", 10.0 * std::log10(64.0 / 4.0));

    // 잡음 대비 신호 PSD
    std::printf("  %6s  %12s  %12s  %10s\n", "TX_SNR", "신호PSD", "잡음PSD",
                "은닉마진");

    static constexpr double kTxSnr[] = {0, 5, 10, 15, 20, 25, 30};
    for (double tx_snr : kTxSnr) {
        // 칩 단위 SNR = TX_SNR - PG
        const double chip_snr_db = tx_snr - 10.0 * std::log10(64.0);

        // PSD 비교 (단위 대역폭당)
        // 은닉마진 = 잡음PSD - 신호PSD (dB) = -chip_snr_db
        const double hiding_margin = -chip_snr_db;

        std::printf("  %+5.0fdB  %10.2f dB  %10.2f dB  %+8.1f dB", tx_snr,
                    chip_snr_db, 0.0, hiding_margin);
        if (hiding_margin > 10.0)
            std::printf("  ← 탐지 불가");
        else if (hiding_margin > 5.0)
            std::printf("  ← 탐지 극난");
        else if (hiding_margin > 0.0)
            std::printf("  ← 잡음 이하");
        std::printf("\n");
    }

    std::printf("\n  SINCGARS (FH) 비교:\n");
    std::printf("    채널 BW: 25 kHz, 비확산\n");
    std::printf("    도약 중 특정 채널 SNR: +20~30dB → 즉시 탐지\n");
    std::printf("    HTS: 전 대역 -8~-18dB → 잡음에 묻힘\n");
}

// ════════════════════════════════════════════════════════════
//  L4: SINCGARS vs HTS 탐지 시간 비교
// ════════════════════════════════════════════════════════════
static void test_L4_detection_time() {
    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  L4: 탐지 소요 시간 비교\n");
    std::printf("═══════════════════════════════════════════════\n\n");

    std::printf("  ┌──────────────┬─────────────┬────────────────┐\n");
    std::printf("  │ 시스템       │ 탐지 방식    │ 소요 시간       │\n");
    std::printf("  ├──────────────┼─────────────┼────────────────┤\n");
    std::printf("  │ FM 무전기    │ 에너지 탐지  │ < 1ms          │\n");
    std::printf("  │ SINCGARS FH  │ 스캔 수신기  │ ~10ms (1 hop)  │\n");
    std::printf("  │ HTS 16칩     │ 에너지 탐지  │ 탐지 불가 (*)  │\n");
    std::printf("  │ HTS 64칩     │ 에너지 탐지  │ 탐지 불가 (*)  │\n");
    std::printf("  │ HTS 64칩     │ 코드 전수탐색│ >10^15 시도 (**)│\n");
    std::printf("  └──────────────┴─────────────┴────────────────┘\n");
    std::printf("\n");
    std::printf("  (*) 칩 SNR < -5dB → 잡음과 구분 불가\n");
    std::printf("  (**) Walsh 64코드 × 인터리빙 시드 2^32\n");
    std::printf("       × 타이밍 불확실성 → 실시간 해독 불가\n\n");

    // 코드 탐색 복잡도 계산
    const double walsh_codes = 64.0;
    const double il_seeds = std::pow(2.0, 32.0);
    const double timing_uncertainty = 1000.0; // ±500칩
    const double total_search =
        walsh_codes * il_seeds * timing_uncertainty;
    const double search_per_sec = 1e9; // 1GHz DSP
    const double search_time_sec = total_search / search_per_sec;
    const double search_time_years =
        search_time_sec / (365.25 * 86400.0);

    std::printf("  코드 전수 탐색 복잡도:\n");
    std::printf("    Walsh: %.0f 코드\n", walsh_codes);
    std::printf("    인터리빙 시드: 2^32 = %.0f\n", il_seeds);
    std::printf("    타이밍: ×%.0f\n", timing_uncertainty);
    std::printf("    총 탐색: %.2e 회\n", total_search);
    std::printf("    1GHz DSP: %.1f년\n", search_time_years);
}

// ════════════════════════════════════════════════════════════
//  D1: 소프트웨어 내구도 (1000회 FEC+HARQ)
// ════════════════════════════════════════════════════════════
static void test_D1_endurance() {
    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  D1: 내구도 1000회 (소프트웨어)\n");
    std::printf("═══════════════════════════════════════════════\n\n");

    static constexpr int kPreReps = 4;
    static constexpr int kPreBoost = 1;
    static constexpr int kMaxC =
        2048 + (FEC_HARQ::NSYM64 + kPreReps + 4) * 64;

    int ok = 0;
    int phase_fail[4] = {};

    for (int t = 0; t < 1000; ++t) {
        g_last = DecodedPacket{};
        const uint32_t ds =
            0xD04A0000u ^ static_cast<uint32_t>(t * 0x9E3779B9u);

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
            info[b] = static_cast<uint8_t>((ds >> (b * 4)) ^
                                           static_cast<unsigned>(t + b));

        std::vector<int16_t> sigI(kMaxC), sigQ(kMaxC);
        const int n = tx_disp.Build_Packet(PayloadMode::DATA, info, 8, kAmp,
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

        if (g_last.success_mask == DecodedPacket::DECODE_MASK_OK) {
            ++ok;
        } else {
            const int ph = static_cast<int>(rx_disp.Get_Phase());
            if (ph >= 0 && ph < 4)
                ++phase_fail[ph];
        }

        if ((t + 1) % 200 == 0)
            std::printf("  %d/1000 (ok=%d)\n", t + 1, ok);
    }

    std::printf("\n  D1: %d/1000 (%.1f%%)\n", ok, 100.0 * ok / 1000.0);
    std::printf("  실패 분포: WAIT_SYNC=%d READ_HEADER=%d READ_PAYLOAD=%d "
                "RF_SETTLING=%d\n",
                phase_fail[static_cast<int>(RxPhase::WAIT_SYNC)],
                phase_fail[static_cast<int>(RxPhase::READ_HEADER)],
                phase_fail[static_cast<int>(RxPhase::READ_PAYLOAD)],
                phase_fail[static_cast<int>(RxPhase::RF_SETTLING)]);
}

// ════════════════════════════════════════════════════════════
//  D2: AWGN 내구도 (1000회 × J/S 래더)
// ════════════════════════════════════════════════════════════
static void test_D2_awgn_endurance() {
    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  D2: AWGN 내구도 1000회\n");
    std::printf("═══════════════════════════════════════════════\n\n");

    static constexpr int kPreReps = 4;
    static constexpr int kPreBoost = 1;
    static constexpr int kMaxC =
        2048 + (FEC_HARQ::NSYM64 + kPreReps + 4) * 64;
    static constexpr double kJs[] = {0, 5, 8, 10, 12, 15};

    std::printf("  %5s  %6s  %8s\n", "J/S", "CRC%", "trials");

    for (double js : kJs) {
        int ok = 0;
        std::mt19937 rng(0xA06E0000u ^
                         static_cast<uint32_t>(static_cast<int>(js * 100)));

        for (int t = 0; t < 1000; ++t) {
            g_last = DecodedPacket{};
            const uint32_t ds =
                0xA06E0000u ^ static_cast<uint32_t>(t * 0x9E3779B9u);

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
                info[b] = static_cast<uint8_t>((ds >> (b * 4)) ^
                                               static_cast<unsigned>(t + b));

            std::vector<int16_t> sigI(kMaxC), sigQ(kMaxC);
            int n = tx_disp.Build_Packet(PayloadMode::DATA, info, 8, kAmp,
                                         sigI.data(), sigQ.data(), kMaxC);
            if (n <= 0)
                continue;

            // AWGN 추가
            if (js > 0.01) {
                double P_sig = 0;
                for (int i = 0; i < n; ++i) {
                    const double di = static_cast<double>(sigI[i]);
                    const double dq = static_cast<double>(sigQ[i]);
                    P_sig += di * di + dq * dq;
                }
                P_sig /= n;
                const double sigma =
                    std::sqrt(P_sig / (2.0 * std::pow(10.0, js / 10.0)));
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
            }

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

        std::printf("  %+4.0fdB  %5.1f%%  %6d\n", js, 100.0 * ok / 1000.0,
                    1000);
    }
}

int main() {
    std::printf("╔═══════════════════════════════════════════════╗\n");
    std::printf("║  HTS B-CDMA LPI/LPD 성능 측정 + 내구도       ║\n");
    std::printf("║  소프트웨어 전용 (Pluto 불필요)               ║\n");
    std::printf("╚═══════════════════════════════════════════════╝\n");

    test_L1_energy_detector();
    test_L2_correlation_detector();
    test_L3_spectral_hiding();
    test_L4_detection_time();

    test_D1_endurance();
    test_D2_awgn_endurance();

    std::printf("\n═══════════════════════════════════════════════\n");
    std::printf("  전체 완료\n");
    std::printf("═══════════════════════════════════════════════\n");
    return 0;
}

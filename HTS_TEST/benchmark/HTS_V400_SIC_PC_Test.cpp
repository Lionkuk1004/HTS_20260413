// =========================================================================
// HTS_V400_SIC_PC_Test.cpp
// PC 전용: HTS_V400_Dispatcher IR-HARQ + SIC(Successive Interference Cancellation)
// 루프백 — Build_Packet 출력을 AWGN 후 칩 단위 Feed_Chip 으로 주입.
//
// 빌드: CMake 타깃 HTS_V400_SIC_PC_Test_Run
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_V400_SIC_PC_Test — PC 전용"
#endif

#include "HTS_V400_Dispatcher.hpp"

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <random>
#include <vector>

namespace {

using ProtectedEngine::DecodedPacket;
using ProtectedEngine::HTS_V400_Dispatcher;
using ProtectedEngine::PayloadMode;

int         g_cb_ok = 0;
int         g_cb_harq_k = 0;
uint32_t    g_cb_mask = 0;

void on_packet_cb(const DecodedPacket& p) noexcept
{
    g_cb_mask = p.success_mask;
    if (p.success_mask == DecodedPacket::DECODE_MASK_OK) {
        g_cb_ok = 1;
        g_cb_harq_k = p.harq_k;
    }
}

void add_awgn(
    int16_t* I, int16_t* Q, int n, double sigma, std::mt19937& rng) noexcept
{
    std::normal_distribution<double> g(0.0, sigma);
    for (int i = 0; i < n; ++i) {
        const long di =
            static_cast<long>(I[i]) + static_cast<long>(std::lround(g(rng)));
        const long dq =
            static_cast<long>(Q[i]) + static_cast<long>(std::lround(g(rng)));
        long ci = di;
        if (ci > 32767L) {
            ci = 32767L;
        } else if (ci < -32768L) {
            ci = -32768L;
        }
        long cq = dq;
        if (cq > 32767L) {
            cq = 32767L;
        } else if (cq < -32768L) {
            cq = -32768L;
        }
        I[i] = static_cast<int16_t>(ci);
        Q[i] = static_cast<int16_t>(cq);
    }
}

struct TrialResult {
    int ok = 0;
    int harq_k = 0;
    int feeds = 0;
};

TrialResult run_ir_loopback_trial(
    bool sic_on,
    uint32_t disp_seed,
    uint32_t noise_seed,
    double sigma,
    int max_feeds) noexcept
{
    TrialResult r{};
    g_cb_ok = 0;
    g_cb_harq_k = 0;
    g_cb_mask = 0;

    HTS_V400_Dispatcher disp;
    disp.Set_IR_Mode(true);
    disp.Set_Seed(disp_seed);
    disp.Set_IR_SIC_Enabled(sic_on);
    constexpr int16_t k_amp = 2000;
    disp.Set_SIC_Walsh_Amp(k_amp);
    disp.Set_CW_Cancel(false);
    disp.Set_AJC_Enabled(false);
    disp.Set_Packet_Callback(on_packet_cb);

    uint8_t info[8] = {
        0xA1u, 0xA2u, 0xA3u, 0xA4u, 0xA5u, 0xA6u, 0xA7u, 0xA8u
    };

    std::mt19937 rng(noise_seed);
    std::vector<int16_t> oI(20000);
    std::vector<int16_t> oQ(20000);

    for (int feed = 0; feed < max_feeds && g_cb_ok == 0; ++feed) {
        const int n = disp.Build_Packet(
            PayloadMode::DATA,
            info,
            8,
            k_amp,
            oI.data(),
            oQ.data(),
            static_cast<int>(oI.size()));
        if (n <= 0) {
            break;
        }
        if (sigma > 0.0) {
            add_awgn(oI.data(), oQ.data(), n, sigma, rng);
        }
        for (int i = 0; i < n; ++i) {
            disp.Feed_Chip(oI[static_cast<std::size_t>(i)],
                oQ[static_cast<std::size_t>(i)]);
        }
        r.feeds++;
    }

    r.ok = g_cb_ok;
    r.harq_k = g_cb_harq_k;
    return r;
}

} // namespace

int main()
{
    constexpr int k_max_feeds = 48;
    int fail = 0;

    // (1) 무잡음 스모크: 1회 전송으로 CRC OK, HARQ 라운드 1
    {
        const TrialResult a = run_ir_loopback_trial(
            true, 0xC0FFEEu, 0x11111111u, 0.0, k_max_feeds);
        const TrialResult b = run_ir_loopback_trial(
            false, 0xC0FFEEu, 0x11111111u, 0.0, k_max_feeds);
        if (a.ok == 0 || a.harq_k != 1 || a.feeds != 1) {
            std::cerr << "[FAIL] SIC=ON noiseless: ok=" << a.ok
                      << " harq_k=" << a.harq_k << " feeds=" << a.feeds
                      << '\n';
            fail = 1;
        }
        if (b.ok == 0 || b.harq_k != 1 || b.feeds != 1) {
            std::cerr << "[FAIL] SIC=OFF noiseless: ok=" << b.ok
                      << " harq_k=" << b.harq_k << " feeds=" << b.feeds
                      << '\n';
            fail = 1;
        }
    }

    // (2) 동일 잡음·동일 시드: SIC ON 이 평균 HARQ 라운드 또는 성공률에서
    //     SIC OFF 보다 나쁘면 안 됨(회귀 감지). 고정 시드 소표본.
    //     σ는 PC 부동소수 RNG에 따라 조정 가능 — 성공 샘플이 너무 적으면 회귀 검사 생략.
    constexpr double k_sigma = 3200.0;
    constexpr int k_mc = 80;
    constexpr int k_min_ok_for_regression = 12;
    double sum_harq_on = 0.0;
    double sum_harq_off = 0.0;
    int n_ok_on = 0;
    int n_ok_off = 0;
    for (int i = 0; i < k_mc; ++i) {
        const uint32_t dseed =
            0x600DCAFEu ^ static_cast<uint32_t>(i * 0x9E3779B9u);
        const uint32_t nseed =
            0xDEADBEEFu ^ static_cast<uint32_t>(i * 0x85EBCA6Bu);
        const TrialResult ton = run_ir_loopback_trial(
            true, dseed, nseed, k_sigma, k_max_feeds);
        const TrialResult toff = run_ir_loopback_trial(
            false, dseed, nseed, k_sigma, k_max_feeds);
        if (ton.ok != 0) {
            sum_harq_on += static_cast<double>(ton.harq_k);
            n_ok_on++;
        }
        if (toff.ok != 0) {
            sum_harq_off += static_cast<double>(toff.harq_k);
            n_ok_off++;
        }
    }
    const double avg_on =
        (n_ok_on > 0) ? (sum_harq_on / static_cast<double>(n_ok_on)) : 0.0;
    const double avg_off =
        (n_ok_off > 0) ? (sum_harq_off / static_cast<double>(n_ok_off)) : 0.0;

    std::cout << "[HTS_V400_SIC_PC_Test] sigma=" << k_sigma
              << " trials=" << k_mc
              << "  SIC_ON ok=" << n_ok_on << " avg_harq=" << avg_on
              << "  SIC_OFF ok=" << n_ok_off << " avg_harq=" << avg_off
              << '\n';

    if (n_ok_off >= k_min_ok_for_regression) {
        if (n_ok_on < n_ok_off) {
            std::cerr << "[FAIL] SIC reduced success count vs OFF\n";
            fail = 1;
        }
        if (n_ok_on > 0 && n_ok_off > 0 && avg_on > avg_off + 1e-6) {
            std::cerr << "[FAIL] SIC_ON mean harq_k worse than SIC_OFF\n";
            fail = 1;
        }
    } else {
        std::cout << "[HTS_V400_SIC_PC_Test] note: noisy regression skipped "
                     "(SIC_OFF ok=" << n_ok_off << " < " << k_min_ok_for_regression
                  << ") — tune k_sigma if desired\n";
    }

    if (fail != 0) {
        return EXIT_FAILURE;
    }
    std::cout << "[HTS_V400_SIC_PC_Test] PASS\n";
    return EXIT_SUCCESS;
}

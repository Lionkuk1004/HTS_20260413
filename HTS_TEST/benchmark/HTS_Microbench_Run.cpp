// =========================================================================
// HTS_Microbench_Run.cpp — PC 전용 wall-clock 마이크로벤치 (Visual Studio / CMake)
//
//  VS에서 "통과/실패"만으로는 핫패스가 느린지 빠른지 알 수 없으므로,
//  Release 구성에서 반복 호출 + std::chrono 로 ns/호출을 출력한다.
//
//  ⚠ M4(168MHz) 사이클과는 다름 — 상대 비교·회귀(패치 전후)용.
//     실칩은 DWT_CYCCNT 또는 로직애널라이저 권장.
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Microbench_Run — PC 전용"
#endif

#include "HTS_AntiJam_Engine.h"
#include "HTS_Adaptive_BPS_Controller.h"
#include "HTS_RF_Metrics.h"

#include <chrono>
#include <cstdint>
#include <iostream>

namespace {

using clock_hr = std::chrono::high_resolution_clock;

template <class Fn>
void bench_print(const char* name, int iterations, Fn&& fn) {
    if (iterations < 1) { iterations = 1; }
    // 워밍업 (최초 호출·캐시)
    for (int w = 0; w < 8; ++w) { fn(); }
    const auto t0 = clock_hr::now();
    for (int i = 0; i < iterations; ++i) { fn(); }
    const auto t1 = clock_hr::now();
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
                        .count();
    const double per_ns = static_cast<double>(ns) / static_cast<double>(iterations);
    std::cout << name << "  iters=" << iterations
              << "  total_ms=" << (static_cast<double>(ns) / 1e6)
              << "  ns_per_call=" << per_ns << '\n';
}

} // namespace

int main() {
    using ProtectedEngine::AntiJamEngine;
    using ProtectedEngine::HTS_Adaptive_BPS_Controller;
    using ProtectedEngine::HTS_RF_Metrics;

    std::cout << "HTS_Microbench_Run (PC x64, use Release for meaningful timing)\n";

    // ── AntiJamEngine::Process (nc=64) ─────────────────────────────
    constexpr int nc = AntiJamEngine::MAX_NC;
    int16_t I[nc];
    int16_t Q[nc];
    for (int i = 0; i < nc; ++i) {
        I[i] = static_cast<int16_t>((i * 17) & 0x7fff);
        Q[i] = static_cast<int16_t>((i * 31) & 0x7fff);
    }

    AntiJamEngine aj_bypass;
    aj_bypass.Reset(nc);
    aj_bypass.Set_AdaptiveBarrageBypass(true);
    bench_print("AntiJam Process (barrage bypass, same engine)", 200000,
        [&]() { aj_bypass.Process(I, Q, nc); });

    AntiJamEngine aj_work;
    aj_work.Reset(nc);
    for (int i = 0; i < nc; ++i) {
        I[i] = 32000;
        Q[i] = (i == 0) ? 32000 : 100;
    }
    bench_print("AntiJam Process (impulsive-ish block, learned path off)", 80000,
        [&]() { aj_work.Process(I, Q, nc); });

    // ── Adaptive BPS Controller::Update ───────────────────────────
    // 생성자가 current_bps=BPS_MIN 으로 초기화하므로, 이후에 메트릭만 세팅.
    HTS_RF_Metrics metrics_quiet;
    HTS_Adaptive_BPS_Controller ctrl_quiet(metrics_quiet);
    metrics_quiet.snr_proxy.store(12, std::memory_order_release);
    metrics_quiet.ajc_nf.store(100u, std::memory_order_release);
    bench_print("Adaptive_BPS Update (QUIET-ish metrics)", 500000,
        [&]() { ctrl_quiet.Update(); });

    HTS_RF_Metrics metrics_hold;
    HTS_Adaptive_BPS_Controller ctrl_hold(metrics_hold);
    metrics_hold.snr_proxy.store(12, std::memory_order_release);
    metrics_hold.ajc_nf.store(900u, std::memory_order_release);
    metrics_hold.current_bps.store(5u, std::memory_order_release);
    bench_print("Adaptive_BPS Update (HOLD mid-AJC)", 500000,
        [&]() { ctrl_hold.Update(); });

    HTS_RF_Metrics metrics_heavy;
    HTS_Adaptive_BPS_Controller ctrl_heavy(metrics_heavy);
    metrics_heavy.snr_proxy.store(12, std::memory_order_release);
    metrics_heavy.ajc_nf.store(2500u, std::memory_order_release);
    metrics_heavy.current_bps.store(5u, std::memory_order_release);
    bench_print("Adaptive_BPS Update (HEAVY immediate min)", 500000,
        [&]() { ctrl_heavy.Update(); });

    std::cout << "Done.\n";
    return 0;
}

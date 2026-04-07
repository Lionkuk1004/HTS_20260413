// =========================================================================
// HTS_Pipeline_Sequential_Audit_PC_Test.cpp
// Pipeline V2 + Adaptive BPS + RF Metrics 스모크 (PC 전용)
//
// [참고] HTS_RF_Metrics 는 int32/uint32/uint8 atomic — double NaN 주입 API 없음.
// Dynamic_Fractal_Mapper::Forward: index > FULL_MASK(4095) 이면 입력 그대로 반환
//   (와이어 인덱스로 쓰면 안 되지만, OOB 순열 연산은 수행하지 않음).
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Pipeline_Sequential_Audit_PC_Test — PC 전용"
#endif

#include "HTS_Adaptive_BPS_Controller.h"
#include "HTS_Dynamic_Fractal_Mapper.h"
#include "HTS_Pipeline_V2_Dispatcher.h"
#include "HTS_RF_Metrics.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>

using namespace ProtectedEngine;

namespace {

static HTS_RF_Metrics g_metrics;
static HTS_Adaptive_BPS_Controller g_bps_ctrl(g_metrics);
static HTS_Pipeline_V2_Dispatcher g_pipe(g_metrics, g_bps_ctrl);

alignas(64) static uint8_t g_dummy_logical[8192] = {};
alignas(64) static uint8_t g_dummy_wire[8192] = {};

static bool Bps_In_Range(uint8_t b) noexcept
{
    return b >= HTS_Adaptive_BPS_Controller::BPS_MIN &&
        b <= HTS_Adaptive_BPS_Controller::BPS_MAX;
}

struct TestResult {
    const char* id;
    bool passed;
};

// [2.1] Scatter/Gather — nullptr 또는 계약 위반 시 run=0 (크래시 없음)
static bool Test_2_1_Pipeline_Null_Ptr()
{
    g_pipe.Begin_Frame(1u, 1u, 0u);
    g_pipe.Fractal_Scatter_Tx(
        nullptr,
        HTS_Pipeline_V2_Dispatcher::kMapperDomain,
        g_dummy_logical,
        8192u,
        100u);
    g_pipe.Fractal_Scatter_Tx(
        g_dummy_wire,
        HTS_Pipeline_V2_Dispatcher::kMapperDomain,
        nullptr,
        8192u,
        100u);
    g_pipe.Fractal_Gather_Rx(
        nullptr,
        8192u,
        g_dummy_wire,
        HTS_Pipeline_V2_Dispatcher::kMapperDomain,
        100u);
    g_pipe.Fractal_Gather_Rx(
        g_dummy_logical,
        8192u,
        nullptr,
        HTS_Pipeline_V2_Dispatcher::kMapperDomain,
        100u);
    return true;
}

// [2.2] 세션·프레임·HARQ 극한값 — Begin_Frame 이 생존, harq는 슬롯 15로 클램프
static bool Test_2_2_Frame_Counter_Extreme()
{
    g_pipe.Begin_Frame(
        0xFFFFFFFFFFFFFFFFull,
        0xFFFFFFFFu,
        255u);
    if (g_pipe.Last_Session_Id() != 0xFFFFFFFFFFFFFFFFull) {
        return false;
    }
    if (g_pipe.Last_Frame_Counter() != 0xFFFFFFFFu) {
        return false;
    }
    return Bps_In_Range(g_pipe.Cached_Bps());
}

// [2.3] dst_wire_cap < kMapperDomain → 산란 루프 0회
static bool Test_2_3_Wire_Cap_Underflow()
{
    g_pipe.Begin_Frame(2u, 0u, 0u);
    std::memset(g_dummy_wire, 0xA5u, sizeof(g_dummy_wire));
    g_pipe.Fractal_Scatter_Tx(
        g_dummy_wire,
        HTS_Pipeline_V2_Dispatcher::kMapperDomain - 1u,
        g_dummy_logical,
        8192u,
        HTS_Pipeline_V2_Dispatcher::kMapperDomain);
    return true;
}

// [2.4] 매퍼: 유효 인덱스는 항상 12비트 도메인 내부 / 비유효는 패스스루
static bool Test_2_4_Mapper_Index_Contract()
{
    g_pipe.Begin_Frame(3u, 0u, 0u);
    Dynamic_Fractal_Mapper& m = g_pipe.Mapper();

    const uint32_t huge = 99999999u;
    if (m.Forward(huge) != huge) {
        return false;
    }

    for (uint32_t i = 0u; i < HTS_Pipeline_V2_Dispatcher::kMapperDomain; ++i) {
        const uint32_t o = m.Forward(i);
        if (o > Dynamic_Fractal_Mapper::FULL_MASK) {
            return false;
        }
    }
    return true;
}

// [2.5] 메트릭 극단값 — 정수 필드만; Update 후 BPS 클램프
static bool Test_2_5_Extreme_Metrics()
{
    g_bps_ctrl.Reset();
    g_metrics.snr_proxy.store(
        (std::numeric_limits<int32_t>::max)(), std::memory_order_relaxed);
    g_metrics.ajc_nf.store(
        (std::numeric_limits<uint32_t>::max)(), std::memory_order_relaxed);
    g_pipe.Begin_Frame(4u, 0u, 0u);
    if (!Bps_In_Range(g_metrics.current_bps.load(std::memory_order_relaxed))) {
        return false;
    }

    g_metrics.snr_proxy.store(
        (std::numeric_limits<int32_t>::min)(), std::memory_order_relaxed);
    g_metrics.ajc_nf.store(0u, std::memory_order_relaxed);
    g_pipe.Begin_Frame(5u, 1u, 0u);
    return Bps_In_Range(g_metrics.current_bps.load(std::memory_order_relaxed));
}

// [2.6] BPS 컨트롤러 연속 Update — 범위 이탈 없음
static bool Test_2_6_BPS_Controller_Stress()
{
    g_bps_ctrl.Reset();
    for (int i = 0; i < 200; ++i) {
        const int32_t snr = static_cast<int32_t>((i * 7919) % 20);
        const uint32_t ajc = static_cast<uint32_t>((i * 6151u) % 2500u);
        g_metrics.snr_proxy.store(snr, std::memory_order_relaxed);
        g_metrics.ajc_nf.store(ajc, std::memory_order_relaxed);
        g_pipe.Begin_Frame(
            6u,
            static_cast<uint32_t>(i),
            static_cast<uint32_t>(i & 15));
        const uint8_t b = g_metrics.current_bps.load(
            std::memory_order_relaxed);
        if (!Bps_In_Range(b)) {
            return false;
        }
    }
    return true;
}

// [2.7] Scatter→Gather 라운드트립 (동일 Begin_Frame·동일 키)
static bool Test_2_7_Scatter_Gather_Roundtrip()
{
    g_pipe.Begin_Frame(7u, 42u, 3u);
    constexpr uint32_t N = HTS_Pipeline_V2_Dispatcher::kMapperDomain;
    alignas(64) static uint8_t src[8192];
    alignas(64) static uint8_t wire[8192];
    alignas(64) static uint8_t dst[8192];
    for (uint32_t i = 0u; i < N; ++i) {
        src[i] = static_cast<uint8_t>(i & 0xFFu);
    }
    g_pipe.Fractal_Scatter_Tx(wire, N, src, N, N);
    g_pipe.Fractal_Gather_Rx(dst, N, wire, N, N);
    return std::memcmp(src, dst, N) == 0;
}

// [2.8] Cached_Bps 가 메트릭과 일치
static bool Test_2_8_Cached_Bps_Sync()
{
    g_bps_ctrl.Reset();
    g_pipe.Begin_Frame(8u, 0u, 0u);
    const uint8_t m = g_metrics.current_bps.load(std::memory_order_relaxed);
    return g_pipe.Cached_Bps() == m;
}

// [2.9] Reset() 후 BPS_MIN
static bool Test_2_9_Controller_Reset()
{
    g_metrics.snr_proxy.store(100, std::memory_order_relaxed);
    g_metrics.ajc_nf.store(0u, std::memory_order_relaxed);
    g_pipe.Begin_Frame(9u, 0u, 0u);
    g_bps_ctrl.Reset();
    const uint8_t b = g_metrics.current_bps.load(std::memory_order_relaxed);
    return b == HTS_Adaptive_BPS_Controller::BPS_MIN;
}

// [2.10] fec_symbol_count 클램프 — cap 미만이면 부분만 처리
static bool Test_2_10_Fec_Count_Clamp()
{
    g_pipe.Begin_Frame(10u, 0u, 0u);
    constexpr uint32_t N = HTS_Pipeline_V2_Dispatcher::kMapperDomain;
    for (uint32_t i = 0u; i < N; ++i) {
        g_dummy_logical[i] = static_cast<uint8_t>(i ^ 0x5Au);
    }
    std::memset(g_dummy_wire, 0u, N);
    g_pipe.Fractal_Scatter_Tx(
        g_dummy_wire,
        N,
        g_dummy_logical,
        16u,
        N);
    return true;
}

static void Run_Pipeline_Sequential_Audit()
{
    const TestResult results[] = {
        { "2.1_NullScatterGather", Test_2_1_Pipeline_Null_Ptr() },
        { "2.2_FrameExtreme", Test_2_2_Frame_Counter_Extreme() },
        { "2.3_WireCapSmall", Test_2_3_Wire_Cap_Underflow() },
        { "2.4_MapperDomain", Test_2_4_Mapper_Index_Contract() },
        { "2.5_MetricsExtreme", Test_2_5_Extreme_Metrics() },
        { "2.6_BPSStress", Test_2_6_BPS_Controller_Stress() },
        { "2.7_Roundtrip", Test_2_7_Scatter_Gather_Roundtrip() },
        { "2.8_CachedBps", Test_2_8_Cached_Bps_Sync() },
        { "2.9_ResetMIN", Test_2_9_Controller_Reset() },
        { "2.10_FecClamp", Test_2_10_Fec_Count_Clamp() },
    };
    const int n = static_cast<int>(sizeof(results) / sizeof(results[0]));

    std::cout << "\n[HTS_Pipeline_Audit] Sequential checks (" << n << ")\n";

    int fail_count = 0;
    for (int i = 0; i < n; ++i) {
        if (!results[i].passed) {
            std::cerr << "[FAIL] " << results[i].id << '\n';
            fail_count++;
        } else {
            std::cout << "[PASS] " << results[i].id << '\n';
        }
    }

    if (fail_count != 0) {
        std::cerr << "\n[ALERT] " << fail_count << " test(s) failed.\n";
        std::exit(EXIT_FAILURE);
    }
    std::cout << "\n[SUCCESS] Pipeline / Metrics audit passed.\n";
}

} // namespace

int main()
{
    std::cout << "========================================================\n";
    std::cout << " HTS Pipeline V2 + Adaptive BPS + RF_Metrics PC Audit\n";
    std::cout << "========================================================\n";
    Run_Pipeline_Sequential_Audit();
    return EXIT_SUCCESS;
}

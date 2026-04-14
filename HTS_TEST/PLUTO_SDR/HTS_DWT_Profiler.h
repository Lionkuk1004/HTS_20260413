// HTS_DWT_Profiler.h — DWT CYCCNT 기반 사이클 프로파일러 (T18)
// Cortex-M4F: DWT CYCCNT. PC 호스트: TSC(__rdtsc) 폴백
// (QueryPerformanceCounter는 타이머 주파수가 168MHz와 무관해 WCET 비교에 부적합)
#pragma once
#include <cstdint>

#if !defined(HTS_PLATFORM_ARM) && !defined(HTS_PLATFORM_PC)
#if (defined(__arm__) || defined(__TARGET_ARCH_ARM) ||                        \
     defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)) &&                   \
    !defined(HTS_ALLOW_HOST_BUILD)
#define HTS_PLATFORM_ARM 1
#else
#define HTS_PLATFORM_PC 1
#endif
#endif

#if defined(HTS_PLATFORM_PC) && defined(_MSC_VER)
#include <intrin.h>
#endif
#if defined(HTS_PLATFORM_PC)
#include <cstdio>
#endif

namespace HTS_DWT {

#if defined(HTS_PLATFORM_ARM)

static inline void Init() noexcept {
    volatile uint32_t *const demcr =
        reinterpret_cast<volatile uint32_t *>(0xE000EDFCu);
    volatile uint32_t *const dwt_ctrl =
        reinterpret_cast<volatile uint32_t *>(0xE0001000u);
    volatile uint32_t *const dwt_cyccnt =
        reinterpret_cast<volatile uint32_t *>(0xE0001004u);
    *demcr |= (1u << 24); // TRCENA
    *dwt_cyccnt = 0u;
    *dwt_ctrl |= 1u; // CYCCNTENA
}

static inline uint32_t Read() noexcept {
    return *reinterpret_cast<volatile uint32_t *>(0xE0001004u);
}

static inline void Reset() noexcept {
    *reinterpret_cast<volatile uint32_t *>(0xE0001004u) = 0u;
}

#elif defined(HTS_PLATFORM_PC)

#if defined(_MSC_VER)
static inline void Init() noexcept {}
static inline uint32_t Read() noexcept {
    return static_cast<uint32_t>(__rdtsc());
}
static inline void Reset() noexcept {}
#else
static inline void Init() noexcept {}
static inline uint32_t Read() noexcept {
    uint32_t lo = 0u;
    uint32_t hi = 0u;
    __asm__ volatile("rdtsc" : "=a"(lo), "=d"(hi));
    return lo;
}
static inline void Reset() noexcept {}
#endif

#else
#error "HTS_DWT_Profiler: unknown platform (define HTS_PLATFORM_ARM or HTS_PLATFORM_PC)"
#endif

struct Profile {
    uint32_t fwht_cycles{};
    uint32_t llr_cycles{};
    uint32_t fold_cycles{};
    uint32_t scl_cycles{};
    uint32_t total_cycles{};
    uint32_t rounds{};
};

inline Profile g_prof{};

struct Stats {
    uint32_t count{};
    uint32_t total_min{};
    uint32_t total_max{};
    uint64_t total_sum{};
    uint32_t scl_min{};
    uint32_t scl_max{};
    uint64_t scl_sum{};
};
inline Stats g_stats{};

static inline void Stats_Update(const Profile &p) noexcept {
    if (g_stats.count == 0u) {
        g_stats.total_min = p.total_cycles;
        g_stats.total_max = p.total_cycles;
        g_stats.scl_min = p.scl_cycles;
        g_stats.scl_max = p.scl_cycles;
    } else {
        if (p.total_cycles < g_stats.total_min) {
            g_stats.total_min = p.total_cycles;
        }
        if (p.total_cycles > g_stats.total_max) {
            g_stats.total_max = p.total_cycles;
        }
        if (p.scl_cycles < g_stats.scl_min) {
            g_stats.scl_min = p.scl_cycles;
        }
        if (p.scl_cycles > g_stats.scl_max) {
            g_stats.scl_max = p.scl_cycles;
        }
    }
    g_stats.total_sum += static_cast<uint64_t>(p.total_cycles);
    g_stats.scl_sum += static_cast<uint64_t>(p.scl_cycles);
    ++g_stats.count;
}

static inline void Stats_Reset() noexcept { g_stats = {}; }

#if defined(HTS_PLATFORM_PC)
static inline void Stats_Print() noexcept {
    if (g_stats.count == 0u) {
        return;
    }
    std::printf("  [CYCCNT] frames=%u\n", g_stats.count);
    std::printf("  total: min=%u  max=%u  avg=%u\n", g_stats.total_min,
                g_stats.total_max,
                static_cast<uint32_t>(g_stats.total_sum / g_stats.count));
    std::printf("  SCL:   min=%u  max=%u  avg=%u\n", g_stats.scl_min,
                g_stats.scl_max,
                static_cast<uint32_t>(g_stats.scl_sum / g_stats.count));
    constexpr uint32_t F_CPU = 168000000u;
    const uint32_t wcet_us = static_cast<uint32_t>(
        (static_cast<uint64_t>(g_stats.total_max) * 1000000u) / F_CPU);
    std::printf("  WCET @168MHz: %u us (%u.%03u ms)\n", wcet_us,
                wcet_us / 1000u, wcet_us % 1000u);
}
#else
static inline void Stats_Print() noexcept {}
#endif

} // namespace HTS_DWT

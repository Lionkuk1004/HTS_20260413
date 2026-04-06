// =========================================================================
// HTS_Dynamic_Fractal_Mapper_PC_Test.cpp
// HTS_PC_TEST_Calibration (CMake) → HTS_Fractal_Mapper_Run.exe
//
// - Forward(Inverse(i)) == i, Inverse(Forward(i)) == i (0..4095 전수)
// - Forward 가 전역 순열인지(중복 없음) 검사
// - 4096회 Forward wall-clock (Release 기준 0.5ms 미만 권고)
// =========================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM) || \
    defined(__TARGET_ARCH_THUMB) || defined(__ARM_ARCH)
#error "[HTS_FATAL] HTS_Dynamic_Fractal_Mapper_PC_Test — PC 전용"
#endif

#ifndef HTS_PC_FRACTAL_MAPPER_TEST
#error "HTS_PC_FRACTAL_MAPPER_TEST must be defined for this TU (CMake)."
#endif

#include "HTS_Dynamic_Fractal_Mapper.h"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {

using Mapper = ProtectedEngine::Dynamic_Fractal_Mapper;
constexpr uint32_t FULL_MASK = Mapper::FULL_MASK;

constexpr uint32_t kNumIndices = FULL_MASK + 1u;
constexpr int kBenchForwardCount = 4096;

bool run_roundtrip_all(const Mapper& m)
{
    for (uint32_t i = 0u; i <= FULL_MASK; ++i) {
        if (m.Forward(m.Inverse(i)) != i) {
            return false;
        }
        if (m.Inverse(m.Forward(i)) != i) {
            return false;
        }
    }
    return true;
}

bool run_permutation_forward(const Mapper& m)
{
    std::vector<uint8_t> seen(kNumIndices, 0);
    for (uint32_t i = 0u; i <= FULL_MASK; ++i) {
        const uint32_t o = m.Forward(i) & FULL_MASK;
        if (seen[static_cast<size_t>(o)] != 0u) {
            return false;
        }
        seen[static_cast<size_t>(o)] = 1u;
    }
    return true;
}

double bench_4096_forwards(const Mapper& m)
{
    volatile uint32_t sink = 0u;
    for (int w = 0; w < 100; ++w) {
        for (int k = 0; k < kBenchForwardCount; ++k) {
            sink ^= m.Forward(static_cast<uint32_t>(k) & FULL_MASK);
        }
    }
    (void)sink;

    using clock = std::chrono::high_resolution_clock;
    const auto t0 = clock::now();
    for (int k = 0; k < kBenchForwardCount; ++k) {
        sink ^= m.Forward(static_cast<uint32_t>(k) & FULL_MASK);
    }
    const auto t1 = clock::now();
    (void)sink;

    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

struct KeyCase {
    uint64_t session_id;
    uint32_t frame_counter;
    const char* label;
};

const KeyCase kCases[] = {
    {0u, 0u, "zero"},
    {0xA5A5A5A5A5A5A5A5u, 0u, "alt_session"},
    {0u, 0xFFFFFFFFu, "max_frame"},
    {0xDEADBEEF01234567u, 0xC0FFEEu, "mixed"},
};

} // namespace

int main()
{
    int failures = 0;

    for (const KeyCase& tc : kCases) {
        Mapper m;
        m.Update_Frame(tc.session_id, tc.frame_counter);

        const bool rt = run_roundtrip_all(m);
        const bool perm = run_permutation_forward(m);
        if (!rt || !perm) {
            std::cerr << "[FAIL] case \"" << tc.label << "\""
                      << " roundtrip=" << (rt ? "ok" : "BAD")
                      << " permutation=" << (perm ? "ok" : "BAD") << '\n';
            ++failures;
            continue;
        }
        std::cout << "[PASS] \"" << tc.label << "\""
                  << " roundtrip+permutation (4096 indices)\n";
    }

    {
        Mapper m;
        m.Update_Frame(kCases[3].session_id, kCases[3].frame_counter);
        constexpr uint32_t kOob = 0xFFFFFFFFu;
        if (m.Forward(kOob) != kOob || m.Inverse(kOob) != kOob) {
            std::cerr << "[FAIL] OOB index must be identity (FPE fallback)\n";
            ++failures;
        } else {
            std::cout << "[PASS] OOB identity Forward/Inverse\n";
        }
    }

    {
        Mapper m;
        m.Update_Frame(kCases[3].session_id, kCases[3].frame_counter);
        const double ms = bench_4096_forwards(m);
        std::cout << "[BENCH] 4096x Forward: " << ms << " ms"
                  << (ms < 0.5 ? " (within 0.5ms)\n" : " (exceeds 0.5ms advisory)\n");
    }

    if (failures != 0) {
        std::cerr << "HTS_Fractal_Mapper_Run: " << failures << " case(s) failed.\n";
        return 1;
    }
    std::cout << "HTS_Fractal_Mapper_Run: all checks passed.\n";
    return 0;
}

// =============================================================================
// pn_masked_unit_test.cpp — PC-only: PN-masked 64-row LUT vs FWHT row ID (Step 2)
// INNOViD / HTS_TEST
//
// fwht64_int32_mirrors_internal(): HTS_V400_Dispatcher_Internal.hpp 의
//   fwht_raw(d, 64) 언롤 경로와 동일 (Phase0 / Payload FWHT 와 정렬 일치).
// HNS HTS_Holo_LPI.cpp 의 static fwht64() 는 동일 butterfly 이지만 스테이지
// 순서가 다를 수 있어, Phase0 통합 검증에는 본 미러를 사용한다.
//
// 빌드 예:
//   cd HTS_TEST
//   cursor_pn_masked_unit_test.cmd
// =============================================================================
#if defined(__arm__) || defined(__TARGET_ARCH_ARM)
#error "PC-only unit test"
#endif

#include <chrono>
#include <cstdint>
#include <cstdio>

#if defined(HTS_USE_PN_MASKED)
#include "HTS_V400_Dispatcher_PNMasked.hpp"
#endif

namespace {

/// `ProtectedEngine::detail::fwht_raw(d, 64)` 언롤 본문과 동일 (Dispatcher).
void fwht64_int32_mirrors_internal(int32_t* d) noexcept {
    for (int i = 0; i < 64; i += 2) {
        int32_t u = d[i], v = d[i + 1];
        d[i] = u + v;
        d[i + 1] = u - v;
    }
    for (int i = 0; i < 64; i += 4) {
        int32_t u = d[i], v = d[i + 2];
        d[i] = u + v;
        d[i + 2] = u - v;
        u = d[i + 1];
        v = d[i + 3];
        d[i + 1] = u + v;
        d[i + 3] = u - v;
    }
    for (int i = 0; i < 64; i += 8) {
        for (int k = 0; k < 4; ++k) {
            int32_t u = d[i + k], v = d[i + 4 + k];
            d[i + k] = u + v;
            d[i + 4 + k] = u - v;
        }
    }
    for (int i = 0; i < 64; i += 16) {
        for (int k = 0; k < 8; ++k) {
            int32_t u = d[i + k], v = d[i + 8 + k];
            d[i + k] = u + v;
            d[i + 8 + k] = u - v;
        }
    }
    for (int i = 0; i < 64; i += 32) {
        for (int k = 0; k < 16; ++k) {
            int32_t u = d[i + k], v = d[i + 16 + k];
            d[i + k] = u + v;
            d[i + 16 + k] = u - v;
        }
    }
    for (int k = 0; k < 32; ++k) {
        int32_t u = d[k], v = d[k + 32];
        d[k] = u + v;
        d[k + 32] = u - v;
    }
}

/// |x| BPTE (분기 없음)
inline int32_t abs_i32_bpte(int32_t x) noexcept {
    const int32_t m = x >> 31;
    return (x ^ m) - m;
}

/// argmax |d[i]|, 동률 시 작은 인덱스 우선 (단위 테스트용 단순 스캔)
int argmax_abs(const int32_t* d, int n) noexcept {
    int32_t best_abs = -1;
    int best_i = 0;
    for (int i = 0; i < n; ++i) {
        const int32_t ai = abs_i32_bpte(d[i]);
        if (ai > best_abs || (ai == best_abs && i < best_i)) {
            best_abs = ai;
            best_i = i;
        }
    }
    return best_i;
}

#if defined(HTS_USE_PN_MASKED)

int test_row_detection_clean() noexcept {
    int pass = 0;
    std::printf("Test 1: clean row detection (FWHT = fwht_raw 64 mirror)\n");
    for (int row = 0; row < 64; ++row) {
        const int16_t* tx = ::detail::GetPnMaskedPreambleI(row);
        int32_t buf[64];
        for (int i = 0; i < 64; ++i) {
            buf[i] = static_cast<int32_t>(tx[i]);
        }
        fwht64_int32_mirrors_internal(buf);
        const int max_idx = argmax_abs(buf, 64);
        if (max_idx == row) {
            ++pass;
            std::printf("  row %2d: PASS (argmax|FWHT|=%d, |peak|=%d)\n", row,
                        max_idx, abs_i32_bpte(buf[max_idx]));
        } else {
            std::printf("  row %2d: FAIL (argmax|FWHT|=%d, expected %d)\n", row,
                        max_idx, row);
        }
    }
    std::printf("Result: %d/64 PASS\n\n", pass);
    return pass;
}

int test_pte_subchip() noexcept {
    using ProtectedEngine::detail::pn_masked_pte_subchip;
    int pass = 0;
    int total = 0;
    auto check = [&](const char* name, int32_t got, int32_t exp) noexcept {
        ++total;
        if (got == exp) {
            ++pass;
            std::printf("  PTE %s: PASS (got %d)\n", name, static_cast<int>(got));
        } else {
            std::printf("  PTE %s: FAIL (got %d, expected %d)\n", name,
                        static_cast<int>(got), static_cast<int>(exp));
        }
    };

    std::printf("Test 2: pn_masked_pte_subchip (parabolic Q14)\n");
    // y_zero peak, symmetric neighbours → offset 0
    check("sym_peak", pn_masked_pte_subchip(100, 1000, 100), 0);
    check("sym_peak2", pn_masked_pte_subchip(800, 1000, 800), 0);
    // den raw 0 (flat), num 0 → finite 0
    check("den0_flat", pn_masked_pte_subchip(500, 500, 500), 0);
    // Integer PTE: num=1, den=6 → 16384/6
    check("frac", pn_masked_pte_subchip(1002, 1000, 1001), 2730);
    // Negative offset (num>0, den<0)
    // num<0, den>0 → negative offset (den=240, C++ trunc toward zero)
    check("neg_off", pn_masked_pte_subchip(1020, 1000, 1100), -5461);
    // Upper clamp
    check("clamp_hi", pn_masked_pte_subchip(100000, 0, 0), 8192);
    // Lower clamp
    check("clamp_lo", pn_masked_pte_subchip(0, 0, 100000), -8192);

    std::printf("Result: %d/%d PTE checks PASS\n\n", pass, total);
    return (pass == total) ? total : -1;
}

static int test_device_id_to_row() noexcept {
    using ProtectedEngine::detail::pn_masked_device_id_to_row;
    int pass = 0;
    std::printf("Test 3: pn_masked_device_id_to_row (XOR fold + clamp)\n");
    const uint32_t ids[] = {0x12345678u, 0xABCDEF00u, 0xDEADBEEFu, 0u,
                             0xFFFFFFFFu};
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        const int row = pn_masked_device_id_to_row(ids[i]);
        std::printf("  device_id=0x%08X → row %d\n",
                      static_cast<unsigned>(ids[i]), row);
        if (row >= 0 && row < 64) {
            ++pass;
        }
    }
    const int a = pn_masked_device_id_to_row(0x12345678u);
    const int b = pn_masked_device_id_to_row(0x12345678u);
    if (a == b) {
        ++pass;
        std::printf("  deterministic: 0x12345678 → %d (repeat OK)\n", a);
    } else {
        std::printf("  deterministic: FAIL (%d vs %d)\n", a, b);
    }
    std::printf("Result: %d/6 PASS\n\n", pass);
    return (pass == 6) ? 1 : 0;
}

/// Step 5-2: PC `pn_masked_phase0_scan` 타이밍 (KCMVP / Step 7 참고).
static int profile_phase0_scan() noexcept {
    using ProtectedEngine::detail::pn_masked_phase0_scan;
    constexpr int kIterations = 100000;

    int16_t test_input[64];
    const int16_t* row0 = ::detail::GetPnMaskedPreambleI(0);
    for (int i = 0; i < 64; ++i) {
        test_input[i] = row0[i];
    }

    int row = 0;
    int32_t peak = 0;

    for (int it = 0; it < 1000; ++it) {
        pn_masked_phase0_scan(test_input, &row, &peak);
    }

    const auto start = std::chrono::high_resolution_clock::now();
    for (int it = 0; it < kIterations; ++it) {
        pn_masked_phase0_scan(test_input, &row, &peak);
    }
    const auto end = std::chrono::high_resolution_clock::now();

    const auto ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start)
            .count();

    const double ns_per_call = static_cast<double>(ns) /
                                static_cast<double>(kIterations);
    const double total_ms = static_cast<double>(ns) / 1.0e6;

    std::printf("\n=== Step 5-2 PC Profile ===\n");
    std::printf("pn_masked_phase0_scan:\n");
    std::printf("  per-call:   %.1f ns\n", ns_per_call);
    std::printf("  iterations: %d\n", kIterations);
    std::printf("  total:      %.2f ms\n", total_ms);
    std::printf("  result row: %d (expected 0)\n", row);
    std::printf("  result peak: %d\n", static_cast<int>(peak));

    const double phase0_total_us = ns_per_call * 32.0 / 1000.0;
    std::printf("\nEstimated Phase0 (32 calls @ step 2): %.1f us\n",
                phase0_total_us);
    const bool within_1ms = phase0_total_us <= 1000.0;
    std::printf("  vs 1ms target (32 calls): %s (%.1f us <= 1000 us)\n",
                within_1ms ? "PASS" : "FAIL", phase0_total_us);

    return (row == 0) ? 1 : 0;
}

#endif  // HTS_USE_PN_MASKED

}  // namespace

int main() {
#if !defined(HTS_USE_PN_MASKED)
    std::printf("HTS_USE_PN_MASKED not defined — test skipped\n");
    return 0;
#else
    std::printf(
        "=== PN-masked unit test (Step 2 + Step 4-1 PTE + Step 6-1 device "
        "+ Step 5-2 profile) ===\n\n");
    const int n = test_row_detection_clean();
    const int pte = test_pte_subchip();
    const int dev = test_device_id_to_row();
    if (n != 64) {
        std::printf("FAIL row tests %d/64\n", n);
    }
    if (pte <= 0) {
        std::printf("FAIL PTE sub-chip tests\n");
    }
    if (dev != 1) {
        std::printf("FAIL device_id → row tests\n");
    }
    if (n != 64 || pte <= 0 || dev != 1) {
        return 1;
    }
    const int prof = profile_phase0_scan();
    if (prof != 1) {
        std::printf("FAIL profile row sanity\n");
        return 1;
    }
    std::printf(
        "\nALL PASS — FWHT rows + PTE sub-chip + device_id map + profile\n");
    return 0;
#endif
}

// ============================================================================
// HTS_Holographic_Unit_Test.cpp — Holographic Sync 유틸 단위 테스트
// ============================================================================
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cmath>

#include "HTS_Preamble_Holographic.h"

using ProtectedEngine::Holographic::holographic_dot_segmented;
using ProtectedEngine::Holographic::k_chu_table;
using ProtectedEngine::Holographic::peak_to_median_ratio_x10;
using ProtectedEngine::Holographic::verify_chu_table_max_err_ppm;

static const int8_t k_w63_ut[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1,
    +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1,
    +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1
};

static bool test_chu_table_accuracy() {
    std::printf("[TEST 1] Chu table accuracy...\n");
    const int32_t ppm = verify_chu_table_max_err_ppm();
    std::printf("  max err: %d ppm (Q14 theory ~65 ppm)\n",
                static_cast<int>(ppm));
    const bool pass = (ppm < 100);
    std::printf("  -> %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

static bool test_holo_dot_clean() {
    std::printf("[TEST 2] holographic_dot_segmented (clean TX signal)...\n");

    constexpr int16_t amp = 1000;
    int16_t chip_I[64], chip_Q[64];

    for (int i = 0; i < 64; ++i) {
        const int32_t cos_v = k_chu_table[i].cos_q14;
        const int32_t sin_v = k_chu_table[i].sin_q14;
        const int32_t s = static_cast<int32_t>(k_w63_ut[i]);

        chip_I[i] = static_cast<int16_t>((amp * s * cos_v) >> 14);
        chip_Q[i] = static_cast<int16_t>((amp * s * sin_v) >> 14);
    }

    const int64_t energy = holographic_dot_segmented(chip_I, chip_Q);
    std::printf("  energy = %lld\n", static_cast<long long>(energy));
    // Clean reference order ~5e8 (Q14); margin for rounding
    const int64_t expected_min = 400000000LL;
    std::printf("  expected_min = %lld\n", static_cast<long long>(expected_min));
    const bool pass = (energy >= expected_min);
    std::printf("  -> %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

static bool test_holo_dot_noise() {
    std::printf("[TEST 3] holographic_dot_segmented (noise only)...\n");

    std::srand(12345);
    int16_t chip_I[64], chip_Q[64];
    constexpr int16_t noise_amp = 100;

    for (int i = 0; i < 64; ++i) {
        chip_I[i] = static_cast<int16_t>((std::rand() % (2 * noise_amp)) - noise_amp);
        chip_Q[i] = static_cast<int16_t>((std::rand() % (2 * noise_amp)) - noise_amp);
    }

    const int64_t energy = holographic_dot_segmented(chip_I, chip_Q);
    std::printf("  energy = %lld (noise amp=%d)\n",
                static_cast<long long>(energy), static_cast<int>(noise_amp));
    const int64_t expected_max = 5000000000LL;
    std::printf("  expected_max = %lld\n", static_cast<long long>(expected_max));
    const bool pass = (energy < expected_max);
    std::printf("  -> %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

static bool test_ratio_single_peak() {
    std::printf("[TEST 4] peak_to_median_ratio_x10 (single peak)...\n");

    int64_t energies[64];
    for (int i = 0; i < 64; ++i) {
        energies[i] = 1000;
    }
    energies[30] = 50000;

    const int32_t ratio = peak_to_median_ratio_x10(energies, 64);
    std::printf("  ratio_x10 = %d (expected ~500)\n", static_cast<int>(ratio));

    const bool pass = (ratio >= 495 && ratio <= 510);
    std::printf("  -> %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

static bool test_ratio_flat() {
    std::printf("[TEST 5] peak_to_median_ratio_x10 (flat)...\n");

    int64_t energies[64];
    for (int i = 0; i < 64; ++i) {
        energies[i] = 5000;
    }

    const int32_t ratio = peak_to_median_ratio_x10(energies, 64);
    std::printf("  ratio_x10 = %d (expected 10)\n", static_cast<int>(ratio));

    const bool pass = (ratio == 10);
    std::printf("  -> %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

static bool test_ratio_zero_median() {
    std::printf("[TEST 6] peak_to_median_ratio_x10 (median=0)...\n");

    int64_t energies[64];
    for (int i = 0; i < 64; ++i) {
        energies[i] = (i < 33) ? 0 : 100;
    }

    const int32_t ratio = peak_to_median_ratio_x10(energies, 64);
    std::printf("  ratio_x10 = %d (expected 9999 clamp)\n", static_cast<int>(ratio));

    const bool pass = (ratio == 9999);
    std::printf("  -> %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

static bool test_holo_dot_with_cfo() {
    std::printf("[TEST 7] holographic_dot_segmented (CFO 5000 Hz @200kcps)...\n");

    constexpr int16_t amp = 1000;
    constexpr int chip_rate = 200000;
    constexpr double cfo_hz = 5000.0;

    int16_t chip_I[64], chip_Q[64];

    for (int i = 0; i < 64; ++i) {
        const int32_t cos_chu = k_chu_table[i].cos_q14;
        const int32_t sin_chu = k_chu_table[i].sin_q14;
        const int32_t s = static_cast<int32_t>(k_w63_ut[i]);

        const int32_t tx_I = (amp * s * cos_chu) >> 14;
        const int32_t tx_Q = (amp * s * sin_chu) >> 14;

        const double phase =
            2.0 * 3.14159265358979323846 * cfo_hz * static_cast<double>(i) /
            static_cast<double>(chip_rate);
        const double c = std::cos(phase);
        const double sn = std::sin(phase);

        const double rI_d = tx_I * c - tx_Q * sn;
        const double rQ_d = tx_I * sn + tx_Q * c;

        chip_I[i] = static_cast<int16_t>(rI_d);
        chip_Q[i] = static_cast<int16_t>(rQ_d);
    }

    const int64_t energy = holographic_dot_segmented(chip_I, chip_Q);
    std::printf("  energy (CFO=5000 Hz) = %lld\n", static_cast<long long>(energy));

    const int64_t expected_min = 200000000LL;
    std::printf("  expected_min = %lld\n", static_cast<long long>(expected_min));
    const bool pass = (energy >= expected_min);
    std::printf("  -> %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

int main() {
    std::printf("========================================\n");
    std::printf("HTS Holographic Sync Unit Tests\n");
    std::printf("========================================\n\n");

    int pass_count = 0;
    int total = 0;

#define RUN(fn)                                                              \
    do {                                                                     \
        ++total;                                                             \
        if (fn()) {                                                          \
            ++pass_count;                                                    \
        }                                                                    \
        std::printf("\n");                                                   \
    } while (0)

    RUN(test_chu_table_accuracy);
    RUN(test_holo_dot_clean);
    RUN(test_holo_dot_noise);
    RUN(test_ratio_single_peak);
    RUN(test_ratio_flat);
    RUN(test_ratio_zero_median);
    RUN(test_holo_dot_with_cfo);

#undef RUN

    std::printf("========================================\n");
    std::printf("Result: %d / %d PASSED\n", pass_count, total);
    std::printf("========================================\n");

    return (pass_count == total) ? 0 : 1;
}

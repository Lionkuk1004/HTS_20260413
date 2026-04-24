// =============================================================================
// test_cfo_lr_estimator.cpp — CFO V5a Phase 1-4 L&R estimator (host unit test)
// =============================================================================
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "HTS_CFO_V5a.hpp"
#include "HTS_CFO_V5a_TestExport.hpp"
#include "HTS_Rx_CFO_SinCos_Table.hpp"

#if !defined(HTS_ALLOW_HOST_BUILD)
#error "Host-only unit test (HTS_ALLOW_HOST_BUILD required)"
#endif

namespace {

int g_fail = 0;

static constexpr int8_t kW63[64] = {
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    -1, +1, +1, -1, +1, -1, -1, +1, +1, -1, -1, +1, -1, +1, +1, -1,
    +1, -1, -1, +1, -1, +1, +1, -1, -1, +1, +1, -1, +1, -1, -1, +1};

void Generate_Ideal_Preamble(int16_t* preI, int16_t* preQ, int16_t amp) noexcept {
    for (int k = 0; k < hts::rx_cfo::kPreambleChips; ++k) {
        preI[k] = static_cast<int16_t>(amp * static_cast<int32_t>(kW63[k & 63]));
        preQ[k] = 0;
    }
}

void Generate_Preamble_With_CFO(int16_t* preI, int16_t* preQ, int16_t amp,
                                int32_t cfo_hz) noexcept {
    int16_t idealI[hts::rx_cfo::kPreambleChips]{};
    int16_t idealQ[hts::rx_cfo::kPreambleChips]{};
    Generate_Ideal_Preamble(idealI, idealQ, amp);

    double phase = 0.0;
    static constexpr double kPi = 3.14159265358979323846;
    const double phase_inc =
        2.0 * kPi * static_cast<double>(cfo_hz) /
        static_cast<double>(hts::rx_cfo::kChipRateHz);

    for (int k = 0; k < hts::rx_cfo::kPreambleChips; ++k) {
        const double c = std::cos(phase);
        const double s = std::sin(phase);
        const double xi = static_cast<double>(idealI[k]);
        const double yq = static_cast<double>(idealQ[k]);
        const double iR = xi * c - yq * s;
        const double iI = xi * s + yq * c;
        const long long lR = std::llround(iR);
        const long long lI = std::llround(iI);
        preI[k] = (lR > 32767)   ? static_cast<int16_t>(32767)
                  : (lR < -32768) ? static_cast<int16_t>(-32768)
                                  : static_cast<int16_t>(lR);
        preQ[k] = (lI > 32767)   ? static_cast<int16_t>(32767)
                  : (lI < -32768) ? static_cast<int16_t>(-32768)
                                  : static_cast<int16_t>(lI);
        phase += phase_inc;
    }
}

void Test_LR_Zero_CFO() noexcept {
    int16_t preI[hts::rx_cfo::kPreambleChips]{};
    int16_t preQ[hts::rx_cfo::kPreambleChips]{};
    Generate_Ideal_Preamble(preI, preQ, 2000);
    const int32_t cfo = hts::rx_cfo::test_export::LR_Estimate(preI, preQ);
    if (std::abs(cfo) >= 10) {
        std::printf("FAIL: Test 1 CFO=0 got %d Hz (want |.| < 10)\n",
                    static_cast<int>(cfo));
        ++g_fail;
    } else {
        std::printf("Test 1 PASS: CFO=0 estimated %d Hz\n", static_cast<int>(cfo));
    }
}

void Test_LR_Small_CFO() noexcept {
    int16_t preI[hts::rx_cfo::kPreambleChips]{};
    int16_t preQ[hts::rx_cfo::kPreambleChips]{};
    Generate_Preamble_With_CFO(preI, preQ, 2000, 500);
    const int32_t cfo = hts::rx_cfo::test_export::LR_Estimate(preI, preQ);
    if (std::abs(cfo - 500) >= 50) {
        std::printf("FAIL: Test 2 CFO=500 got %d Hz (want 500 +-50)\n",
                    static_cast<int>(cfo));
        ++g_fail;
    } else {
        std::printf("Test 2 PASS: CFO=500 estimated %d Hz\n", static_cast<int>(cfo));
    }
}

void Test_LR_Negative_CFO() noexcept {
    int16_t preI[hts::rx_cfo::kPreambleChips]{};
    int16_t preQ[hts::rx_cfo::kPreambleChips]{};
    Generate_Preamble_With_CFO(preI, preQ, 2000, -500);
    const int32_t cfo = hts::rx_cfo::test_export::LR_Estimate(preI, preQ);
    if (std::abs(cfo - (-500)) >= 50) {
        std::printf("FAIL: Test 3 CFO=-500 got %d Hz (want -500 +-50)\n",
                    static_cast<int>(cfo));
        ++g_fail;
    } else {
        std::printf("Test 3 PASS: CFO=-500 estimated %d Hz\n", static_cast<int>(cfo));
    }
}

void Test_LR_Large_CFO() noexcept {
    int16_t preI[hts::rx_cfo::kPreambleChips]{};
    int16_t preQ[hts::rx_cfo::kPreambleChips]{};
    Generate_Preamble_With_CFO(preI, preQ, 2000, 3000);
    const int32_t cfo = hts::rx_cfo::test_export::LR_Estimate(preI, preQ);
    // int16 양자화·16칩 세그먼트 이산화로 고 CFO에서 이론값 대비 ~10% 저편향 관측.
    if (std::abs(cfo - 3000) >= 320) {
        std::printf("FAIL: Test 4 CFO=3000 got %d Hz (want 3000 +-320)\n",
                    static_cast<int>(cfo));
        ++g_fail;
    } else {
        std::printf("Test 4 PASS: CFO=3000 estimated %d Hz\n", static_cast<int>(cfo));
    }
}

void Test_LR_Near_Limit() noexcept {
    int16_t preI[hts::rx_cfo::kPreambleChips]{};
    int16_t preQ[hts::rx_cfo::kPreambleChips]{};
    Generate_Preamble_With_CFO(preI, preQ, 2000, 6000);
    const int32_t cfo = hts::rx_cfo::test_export::LR_Estimate(preI, preQ);
    if (std::abs(cfo - 6000) >= 650) {
        std::printf("FAIL: Test 5 CFO=6000 got %d Hz (want 6000 +-650)\n",
                    static_cast<int>(cfo));
        ++g_fail;
    } else {
        std::printf("Test 5 PASS: CFO=6000 estimated %d Hz\n", static_cast<int>(cfo));
    }
}

}  // namespace

int main() {
    hts::rx_cfo::Build_SinCos_Table();
    Test_LR_Zero_CFO();
    Test_LR_Small_CFO();
    Test_LR_Negative_CFO();
    Test_LR_Large_CFO();
    Test_LR_Near_Limit();
    if (g_fail != 0) {
        std::printf("test_cfo_lr_estimator: %d failure(s)\n", g_fail);
        return 1;
    }
    std::printf("test_cfo_lr_estimator: PASS\n");
    return 0;
}

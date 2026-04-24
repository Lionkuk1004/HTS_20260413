// =============================================================================
// test_cfo_v5a_full.cpp — CFO V5a full Estimate() pipeline (host unit test)
// =============================================================================
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "HTS_CFO_V5a.hpp"
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

void Generate_Preamble_With_CFO(int16_t* preI, int16_t* preQ, int16_t amp,
                                  int32_t cfo_hz) noexcept {
    static constexpr double kPi = 3.14159265358979323846;
    const double phase_inc =
        2.0 * kPi * static_cast<double>(cfo_hz) /
        static_cast<double>(hts::rx_cfo::kChipRateHz);
    double phase = 0.0;

    for (int k = 0; k < hts::rx_cfo::kPreambleChips; ++k) {
        const int16_t base =
            static_cast<int16_t>(amp * static_cast<int32_t>(kW63[k & 63]));
        const double c = std::cos(phase);
        const double s = std::sin(phase);
        const double iR = static_cast<double>(base) * c;
        const double iI = static_cast<double>(base) * s;
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

void Test_V5a_Zero_CFO() noexcept {
    hts::rx_cfo::CFO_V5a cfo;
    cfo.Init();
    int16_t preI[hts::rx_cfo::kPreambleChips]{};
    int16_t preQ[hts::rx_cfo::kPreambleChips]{};
    Generate_Preamble_With_CFO(preI, preQ, 2000, 0);
    const hts::rx_cfo::CFO_Result res = cfo.Estimate(preI, preQ);
    if (!res.valid || std::abs(res.cfo_hz) >= 50) {
        std::printf("FAIL: Test1 zero CFO valid=%d est=%d\n",
                    res.valid ? 1 : 0, static_cast<int>(res.cfo_hz));
        ++g_fail;
    } else {
        std::printf("Test 1 PASS: CFO=0 est=%d Hz\n", static_cast<int>(res.cfo_hz));
    }
}

void Test_V5a_Small_CFO() noexcept {
    hts::rx_cfo::CFO_V5a cfo;
    cfo.Init();
    int16_t preI[hts::rx_cfo::kPreambleChips]{};
    int16_t preQ[hts::rx_cfo::kPreambleChips]{};
    Generate_Preamble_With_CFO(preI, preQ, 2000, 500);
    const hts::rx_cfo::CFO_Result res = cfo.Estimate(preI, preQ);
    if (!res.valid || std::abs(res.cfo_hz - 500) >= 80) {
        std::printf("FAIL: Test2 CFO=500 valid=%d est=%d\n", res.valid ? 1 : 0,
                    static_cast<int>(res.cfo_hz));
        ++g_fail;
    } else {
        std::printf("Test 2 PASS: CFO=500 est=%d Hz\n", static_cast<int>(res.cfo_hz));
    }
}

void Test_V5a_Medium_CFO() noexcept {
    hts::rx_cfo::CFO_V5a cfo;
    cfo.Init();
    int16_t preI[hts::rx_cfo::kPreambleChips]{};
    int16_t preQ[hts::rx_cfo::kPreambleChips]{};
    Generate_Preamble_With_CFO(preI, preQ, 2000, 3000);
    const hts::rx_cfo::CFO_Result res = cfo.Estimate(preI, preQ);
    if (!res.valid || std::abs(res.cfo_hz - 3000) >= 200) {
        std::printf("FAIL: Test3 CFO=3000 valid=%d est=%d\n", res.valid ? 1 : 0,
                    static_cast<int>(res.cfo_hz));
        ++g_fail;
    } else {
        std::printf("Test 3 PASS: CFO=3000 est=%d Hz\n", static_cast<int>(res.cfo_hz));
    }
}

void Test_V5a_Negative_CFO() noexcept {
    hts::rx_cfo::CFO_V5a cfo;
    cfo.Init();
    int16_t preI[hts::rx_cfo::kPreambleChips]{};
    int16_t preQ[hts::rx_cfo::kPreambleChips]{};
    Generate_Preamble_With_CFO(preI, preQ, 2000, -5000);
    const hts::rx_cfo::CFO_Result res = cfo.Estimate(preI, preQ);
    if (!res.valid || std::abs(res.cfo_hz - (-5000)) >= 300) {
        std::printf("FAIL: Test4 CFO=-5000 valid=%d est=%d\n", res.valid ? 1 : 0,
                    static_cast<int>(res.cfo_hz));
        ++g_fail;
    } else {
        std::printf("Test 4 PASS: CFO=-5000 est=%d Hz\n", static_cast<int>(res.cfo_hz));
    }
}

void Test_V5a_Large_CFO() noexcept {
    hts::rx_cfo::CFO_V5a cfo;
    cfo.Init();
    int16_t preI[hts::rx_cfo::kPreambleChips]{};
    int16_t preQ[hts::rx_cfo::kPreambleChips]{};
    Generate_Preamble_With_CFO(preI, preQ, 2000, 10000);
    const hts::rx_cfo::CFO_Result res = cfo.Estimate(preI, preQ);
    if (!res.valid || std::abs(res.cfo_hz - 10000) >= 500) {
        std::printf("FAIL: Test5 CFO=10000 valid=%d est=%d\n", res.valid ? 1 : 0,
                    static_cast<int>(res.cfo_hz));
        ++g_fail;
    } else {
        std::printf("Test 5 PASS: CFO=10000 est=%d Hz\n", static_cast<int>(res.cfo_hz));
    }
}

void Test_V5a_Determinism() noexcept {
    hts::rx_cfo::CFO_V5a cfo;
    cfo.Init();
    int16_t preI[hts::rx_cfo::kPreambleChips]{};
    int16_t preQ[hts::rx_cfo::kPreambleChips]{};
    Generate_Preamble_With_CFO(preI, preQ, 2000, 1500);
    const hts::rx_cfo::CFO_Result res1 = cfo.Estimate(preI, preQ);
    const hts::rx_cfo::CFO_Result res2 = cfo.Estimate(preI, preQ);
    if (res1.cfo_hz != res2.cfo_hz || res1.peak_energy != res2.peak_energy) {
        std::printf("FAIL: Test6 determinism est1=%d est2=%d\n",
                    static_cast<int>(res1.cfo_hz), static_cast<int>(res2.cfo_hz));
        ++g_fail;
    } else {
        std::printf("Test 6 PASS: Determinism est=%d Hz\n",
                    static_cast<int>(res1.cfo_hz));
    }
}

}  // namespace

int main() {
    hts::rx_cfo::Build_SinCos_Table();
    Test_V5a_Zero_CFO();
    Test_V5a_Small_CFO();
    Test_V5a_Medium_CFO();
    Test_V5a_Negative_CFO();
    Test_V5a_Large_CFO();
    Test_V5a_Determinism();
    if (g_fail != 0) {
        std::printf("test_cfo_v5a_full: %d failure(s)\n", g_fail);
        return 1;
    }
    std::printf("test_cfo_v5a_full: PASS\n");
    return 0;
}
